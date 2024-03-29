// ROS
#include <ros/ros.h>

// Common
#include <vector>
#include <math.h>
#include <iostream>
#include <chrono>

// For acos, fabs
#include <cmath>

// G2O: infrastructure
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_multi_edge.h>

#include <g2o/core/block_solver.h>

// G2O: optimization algorithms
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>

// G2O: linear equation solver
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>

// For Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry> 

// For KDL
#include <kdl/kdl.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>

// For file write and read
#include <string>
#include "H5Cpp.h"
//#include "H5Location.h"

// Process the terminal arguments
#include <getopt.h>

// For collision checking
#include "collision_checking_yumi.h"

// Macros
#define JOINT_DOF 38


using namespace g2o;
using namespace Eigen;
using namespace H5;


typedef struct {

  // Human motion data
  Vector3d l_shoulder_pos_goal; // this is human shoulder position
  Vector3d l_elbow_pos_goal;
  Vector3d l_wrist_pos_goal;
  Matrix3d l_wrist_ori_goal;

  Vector3d r_shoulder_pos_goal; // this is human shoulder position
  Vector3d r_elbow_pos_goal;
  Vector3d r_wrist_pos_goal;
  Matrix3d r_wrist_ori_goal;

  Matrix<double, 14, 1> l_finger_pos_goal;
  Matrix<double, 14, 1> r_finger_pos_goal;


  // Pre-required data of robot
  Vector3d l_robot_shoulder_pos; // this is normally fixed for a non-mobile robot
  Vector3d r_robot_shoulder_pos;

  Matrix<double, 12, 1> l_robot_finger_start; // joint angle range, for direct scaling and linear mapping
  Matrix<double, 12, 1> l_robot_finger_final;
  Matrix<double, 12, 1> r_robot_finger_start;
  Matrix<double, 12, 1> r_robot_finger_final; 

  Matrix<double, 14, 1> glove_start;
  Matrix<double, 14, 1> glove_final;

  // Constraint func related
  //std::vector<double> qlb, qub;

  // KDL FK related
  //KDL::ChainFkSolverPos_recursive fk_solver;
  unsigned int l_num_wrist_seg = 0;
  unsigned int l_num_elbow_seg = 0; // initialized as flag
  unsigned int l_num_shoulder_seg = 0;

  unsigned int r_num_wrist_seg = 0;
  unsigned int r_num_elbow_seg = 0; // initialized as flag
  unsigned int r_num_shoulder_seg = 0;

  // Record different parts of the cost function
  double upperarm_direction_cost = 0;
  double shoulder_wrist_direction_cost = 0; // from shoulder to wrist pos
  double forearm_direction_cost = 0;
  double wrist_ori_cost = 0;
  double wrist_pos_cost = 0;
  double elbow_pos_cost = 0;

  double scaled_wrist_pos_cost = 0;
  double scaled_elbow_pos_cost = 0;

  double scaled_l_finger_pos_cost = 0;
  double scaled_r_finger_pos_cost = 0;


  double smoothness_cost = 0;
  double total_cost = 0;


  double l_r_pos_diff_cost = 0;


  // Joint limits (pos, vel, acc)
  Matrix<double, JOINT_DOF, 1> q_pos_lb;
  Matrix<double, JOINT_DOF, 1> q_pos_ub;

  Matrix<double, JOINT_DOF, 1> q_vel_lb;  
  Matrix<double, JOINT_DOF, 1> q_vel_ub;

  Matrix<double, JOINT_DOF, 1> q_acc_lb;
  Matrix<double, JOINT_DOF, 1> q_acc_ub;



  // Record information for calculating the cost of dual arm coordination
  Vector3d l_wrist_cur;
  Vector3d r_wrist_cur;
  double l_r_wrist_diff_cost;


  // A class for distance computation (collision avoidance)
  int argc;
  char **argv;
  //std::string urdf_string;
  //std::string srdf_string;
  //DualArmDualHandMinDistance dual_arm_dual_hand_min_distance;


} my_constraint_struct;


// This function sets elbow ID and wrist ID in constraint_data, and returns the KDL_FK solver 
KDL::ChainFkSolverPos_recursive setup_left_kdl(my_constraint_struct &constraint_data)
{
  // Params
  const std::string URDF_FILE = "/home/liangyuwei/sign_language_robot_ws/src/yumi_description/urdf/yumi_with_hands.urdf";
  const std::string BASE_LINK = "world"; // use /world as base_link for convenience in simulation; when transfer across different robot arms, may use mid-point between shoulders as the common base(or world)
  const std::string SHOULDER_LINK = "yumi_link_1_l";
  const std::string ELBOW_LINK = "yumi_link_4_l";
  const std::string WRIST_LINK = "yumi_link_7_l";

  // Get tree
  KDL::Tree kdl_tree; 
   if (!kdl_parser::treeFromFile(URDF_FILE, kdl_tree)){ 
      ROS_ERROR("Failed to construct kdl tree");
      exit(-1);
   }
  //ROS_INFO("Successfully built a KDL tree from URDF file.");

  // Get chain  
  KDL::Chain kdl_chain; 
  if(!kdl_tree.getChain(BASE_LINK, WRIST_LINK, kdl_chain)){
    ROS_INFO("Failed to obtain chain from root to wrist");
    exit(-1);
  }
  //ROS_INFO("Successfully obtained chain from root to wrist.");


  // Find segment number for wrist and elbow links, store in constraint_dat
  if (constraint_data.l_num_wrist_seg == 0 || constraint_data.l_num_elbow_seg == 0 || constraint_data.l_num_shoulder_seg == 0) // if the IDs not set
  {
    unsigned int num_segments = kdl_chain.getNrOfSegments();
    constraint_data.l_num_wrist_seg = num_segments - 1;
    //ROS_INFO_STREAM("There are " << num_segments << " segments in the kdl_chain");
    for (unsigned int i = 0; i < num_segments; ++i){
      if (kdl_chain.getSegment(i).getName() == ELBOW_LINK){
        constraint_data.l_num_elbow_seg = i;
        //ROS_INFO_STREAM("Elbow link found.");
        break;
      }
      if (kdl_chain.getSegment(i).getName() == SHOULDER_LINK){
        constraint_data.l_num_shoulder_seg = i;
        //ROS_INFO_STREAM("Elbow link found.");
        //break;
      }
    }
  }


  // Set up FK solver and compute the homogeneous representations
  KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain);
  //ROS_INFO_STREAM("Joint dimension is: " << kdl_chain.getNrOfJoints()); // 6 joints, 8 segments, checked!

  return fk_solver;

}


// This function sets elbow ID and wrist ID in constraint_data, and returns the KDL_FK solver 
KDL::ChainFkSolverPos_recursive setup_right_kdl(my_constraint_struct &constraint_data)
{
  // Params
  const std::string URDF_FILE = "/home/liangyuwei/sign_language_robot_ws/src/yumi_description/urdf/yumi_with_hands.urdf";
  const std::string BASE_LINK = "world"; // use /world as base_link for convenience in simulation; when transfer across different robot arms, may use mid-point between shoulders as the common base(or world)
  const std::string SHOULDER_LINK = "yumi_link_1_r";
  const std::string ELBOW_LINK = "yumi_link_4_r";
  const std::string WRIST_LINK = "yumi_link_7_r";

  // Get tree
  KDL::Tree kdl_tree; 
   if (!kdl_parser::treeFromFile(URDF_FILE, kdl_tree)){ 
      ROS_ERROR("Failed to construct kdl tree");
      exit(-1);
   }
  //ROS_INFO("Successfully built a KDL tree from URDF file.");

  // Get chain  
  KDL::Chain kdl_chain; 
  if(!kdl_tree.getChain(BASE_LINK, WRIST_LINK, kdl_chain)){
    ROS_INFO("Failed to obtain chain from root to wrist");
    exit(-1);
  }
  //ROS_INFO("Successfully obtained chain from root to wrist.");


  // Find segment number for wrist and elbow links, store in constraint_dat
  if (constraint_data.r_num_wrist_seg == 0 || constraint_data.r_num_elbow_seg == 0 || constraint_data.r_num_shoulder_seg == 0) // if the IDs not set
  {
    unsigned int num_segments = kdl_chain.getNrOfSegments();
    constraint_data.r_num_wrist_seg = num_segments - 1;
    //ROS_INFO_STREAM("There are " << num_segments << " segments in the kdl_chain");
    for (unsigned int i = 0; i < num_segments; ++i){
      //std::cout << "Segment name: " << kdl_chain.getSegment(i).getName() << std::endl;
      if (kdl_chain.getSegment(i).getName() == ELBOW_LINK){
        constraint_data.r_num_elbow_seg = i;
        //ROS_INFO_STREAM("Elbow link found.");
        //break;
      }
      if (kdl_chain.getSegment(i).getName() == SHOULDER_LINK){
        constraint_data.r_num_shoulder_seg = i;
        //ROS_INFO_STREAM("Elbow link found.");
        //break;
      }
    }
    //std::cout << "Shoulder ID: " << constraint_data.r_num_shoulder_seg << ", Elbow ID: " << constraint_data.r_num_elbow_seg << ", Wrist ID: " << constraint_data.r_num_wrist_seg << std::endl;
  }


  // Set up FK solver and compute the homogeneous representations
  KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain);
  //ROS_INFO_STREAM("Joint dimension is: " << kdl_chain.getNrOfJoints()); // 6 joints, 8 segments, checked!

  return fk_solver;

}


bool write_h5(const std::string file_name, const std::string group_name, const std::string dataset_name, const int ROW, const int COL, std::vector<std::vector<double>> data_vector)
{
  // Set up file name and dataset name
  const H5std_string FILE_NAME(file_name);
  const H5std_string GROUP_NAME(group_name);
  const H5std_string DATASET_NAME(dataset_name);

  // Convert 2-dim std::vector to 2-dim raw buffer(array)
  double data[ROW][COL];
  for (int j = 0; j < ROW; j++)
  {
    for (int i = 0; i < COL; i++)
    data[j][i] = data_vector[j][i];
  }

  try
  {

    // Shutdown auto-print of error information
    herr_t status = H5Eset_auto(H5E_DEFAULT, NULL, NULL);

    // Create a file(create, fail if it exists)
    H5Fcreate(FILE_NAME.c_str(), H5F_ACC_EXCL, H5P_DEFAULT, H5P_DEFAULT);
    
    // Create a file (must be an existing file)
    H5File file( FILE_NAME, H5F_ACC_RDWR );

    // Create a group (if exists, destroy it, and re-create another)
    Group group;
    status = H5Lget_info(file.getId(), GROUP_NAME.c_str(), NULL, H5P_DEFAULT);
    if (status==0)
    {
      std::cout << "The group already exists, open it." << std::endl;
      group = file.openGroup(GROUP_NAME);
    }
    else
    {
      std::cout << "The group doesn't exist, create one." << std::endl;
      group = file.createGroup(GROUP_NAME);
    }

  
    // Set up datatype and dataspace for the dataset to be store
    hsize_t dimsf[2];              // dataset dimensions
    dimsf[0] = ROW;
    dimsf[1] = COL;
    DataSpace dataspace(2, dimsf);
    IntType datatype( PredType::NATIVE_DOUBLE );
    datatype.setOrder( H5T_ORDER_LE );


    // Way 1 - Create a dataset within a 'group'
    status = H5Lget_info(group.getId(), DATASET_NAME.c_str(), NULL, H5P_DEFAULT);
    if (status == 0)
    {
      std::cout << "The dataset already exists, remove it and re-create another one." << std::endl;
      group.unlink(DATASET_NAME.c_str());
    }
    else
    {
      std::cout << "The dataset doesn't exist, create one." << std::endl;
    }
    DataSet dataset1 = group.createDataSet(DATASET_NAME, datatype, dataspace);


    // Way 2 - Create a new dataset within the 'file'
    //DataSet dataset2 = file.createDataSet( DATASET_NAME, datatype, dataspace );


    //Write the data to the dataset using default memory space, file space, and transfer properties.
    dataset1.write( data, PredType::NATIVE_DOUBLE );
    //dataset2.write( data, PredType::NATIVE_DOUBLE );

  } // File and group will be closed as their instances go out of scope

  // catch failure caused by the H5File operations
  catch( FileIException error )
  {
    error.printErrorStack();
    return -1;
  }
  // catch failure caused by the DataSet operations
  catch( DataSetIException error )
  {
    error.printErrorStack();
    return -1;
  }
  // catch failure caused by the DataSpace operations
  catch( DataSpaceIException error )
  {
    error.printErrorStack();
    return -1;
  }
  // catch failure caused by the DataSpace operations
  catch( DataTypeIException error )
  {
    error.printErrorStack();
    return -1;
  }

  // Finish
  return true;

}


// Read h5 file for joint path
std::vector<std::vector<double>> read_h5(const std::string file_name, const std::string group_name, const std::string dataset_name)
{
  // Set up file name and dataset name
  const H5std_string FILE_NAME(file_name);
  const H5std_string DATASET_NAME(dataset_name);
  const H5std_string GROUP_NAME(group_name);

  try
  {
    // Open the specified file and the specified dataset in the file.
    H5File file( FILE_NAME, H5F_ACC_RDONLY );
    //DataSet dataset = file.openDataSet(DATASET_NAME)

    // Open a group 
    Group group = file.openGroup(GROUP_NAME);
    DataSet dataset = group.openDataSet(DATASET_NAME);

    // Get the class of the datatype that is used by the dataset.
    //H5T_class_t type_class = dataset.getTypeClass();

    // Get dataspace of the dataset.
    DataSpace dataspace = dataset.getSpace();

    // Get the dimension size of each dimension in the dataspace and display them.
    hsize_t dims_out[2];
    //int ndims = dataspace.getSimpleExtentDims( dims_out, NULL);
    int ROW = dims_out[0], COL = dims_out[1];

    // Read data into raw buffer(array) and convert to std::vector
    double data_array[ROW][COL];
    dataset.read(data_array, PredType::NATIVE_DOUBLE);
    std::vector<std::vector<double>> data_vector(ROW, std::vector<double>(COL));
    for (int j = 0; j < dims_out[0]; j++)
    {
      for (int i = 0; i < dims_out[1]; i++)
        data_vector[j][i] = data_array[j][i];
    }

    return data_vector;

  } 
   // catch failure caused by the H5File operations
   catch( FileIException error )
   {
      error.printErrorStack();
      exit(-1);
   }
   // catch failure caused by the DataSet operations
   catch( DataSetIException error )
   {
      error.printErrorStack();
      exit(-1);
   }
   // catch failure caused by the DataSpace operations
   catch( DataSpaceIException error )
   {
      error.printErrorStack();
      exit(-1);
   }
   // catch failure caused by the DataSpace operations
   catch( DataTypeIException error )
   {
      error.printErrorStack();
      exit(-1);
   }

}


/* Define my own vertex, i.e. parameter block to optimize over */
class DualArmDualHandVertex : public BaseVertex<JOINT_DOF, Matrix<double, JOINT_DOF, 1> >
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual void setToOriginImpl() // reset
    {
      _estimate << Matrix<double, JOINT_DOF, 1>::Zero();
    }
  
    virtual void oplusImpl(const double *update) // update rule
    {
      for (unsigned int i = 0; i < JOINT_DOF; ++i)
        _estimate[i] += update[i];//Map<Matrix<double, JOINT_DOF, 1> >(update, JOINT_DOF, 1);
    }

    // Read and write, leave blank
    virtual bool read( std::istream& in ) {}
    virtual bool write( std::ostream& out ) const {}
};


/* Define constraint for ... */
class MyUnaryConstraints : public BaseUnaryEdge<1, my_constraint_struct, DualArmDualHandVertex>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MyUnaryConstraints(KDL::ChainFkSolverPos_recursive &_left_fk_solver, KDL::ChainFkSolverPos_recursive &_right_fk_solver, boost::shared_ptr<DualArmDualHandCollision> &_dual_arm_dual_hand_collision_ptr) : left_fk_solver(_left_fk_solver), right_fk_solver(_right_fk_solver), dual_arm_dual_hand_collision_ptr(_dual_arm_dual_hand_collision_ptr){};

    // functions to compute costs
    double compute_arm_cost(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, unsigned int num_elbow_seg, unsigned int num_shoulder_seg, bool left_or_right, my_constraint_struct &fdata);
    double linear_map(double x_, double min_, double max_, double min_hat, double max_hat);
    double compute_finger_cost(Matrix<double, 12, 1> q_finger_robot, bool left_or_right, my_constraint_struct &fdata);
    double compute_collision_cost(Matrix<double, JOINT_DOF, 1> q_whole, boost::shared_ptr<DualArmDualHandCollision> &dual_arm_dual_hand_collision_ptr);
    double compute_pos_limit_cost(Matrix<double, JOINT_DOF, 1> q_whole, my_constraint_struct &fdata);

    void computeError();

    // Read and write, leave blank
    virtual bool read( std::istream& in ) {}
    virtual bool write( std::ostream& out ) const {}

  public:
    // FK solvers
    KDL::ChainFkSolverPos_recursive &left_fk_solver;
    KDL::ChainFkSolverPos_recursive &right_fk_solver;
    // Collision checker
    boost::shared_ptr<DualArmDualHandCollision> &dual_arm_dual_hand_collision_ptr;

};


double MyUnaryConstraints::compute_collision_cost(Matrix<double, JOINT_DOF, 1> q_whole, boost::shared_ptr<DualArmDualHandCollision> &dual_arm_dual_hand_collision_ptr)
{
  // convert from matrix to std::vector
  std::vector<double> x(JOINT_DOF);
  for (unsigned int i = 0; i < JOINT_DOF; ++i)
    x[i] = q_whole[i];

  // Collision checking (or distance computation), check out the DualArmDualHandCollision class
  double min_distance = dual_arm_dual_hand_collision_ptr->check_self_collision(x); 
  // check_world_collision, check_full_collision, compute_self_distance, compute_world_distance

  // Compute cost for collision avoidance
  double k_col = 1.0;
  double cost = k_col * (min_distance + 1) * (min_distance + 1); // 1 for colliding state, -1 for non

  return cost;

}


double MyUnaryConstraints::compute_pos_limit_cost(Matrix<double, JOINT_DOF, 1> q_whole, my_constraint_struct &fdata)
{

  // compute out-of-bound cost
  double cost = 0;
  for (unsigned int i = 0; i < JOINT_DOF; ++i)
  {
    if (fdata.q_pos_ub[i] < q_whole[i])
      cost += (q_whole[i] - fdata.q_pos_ub[i]) * (q_whole[i] - fdata.q_pos_ub[i]);
    if (fdata.q_pos_lb[i] > q_whole[i])
      cost += (fdata.q_pos_lb[i] - q_whole[i]) * (fdata.q_pos_lb[i] - q_whole[i]);
  }

  return cost;

}




double MyUnaryConstraints::compute_arm_cost(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, unsigned int num_elbow_seg, unsigned int num_shoulder_seg, bool left_or_right, my_constraint_struct &fdata)
    {
      // Get joint angles
      KDL::JntArray q_in(q_cur.size()); 
      for (unsigned int i = 0; i < q_cur.size(); ++i)
      {
        q_in(i) = q_cur(i);
      }

      // Do FK using KDL, get the current elbow/wrist/shoulder state
      KDL::Frame elbow_cart_out, wrist_cart_out, shoulder_cart_out; // Output homogeneous transformation
      int result;
      result = fk_solver.JntToCart(q_in, elbow_cart_out, num_elbow_seg+1); // notice that the number here is not the segment ID, but the number of segments till target segment
      if (result < 0){
        ROS_INFO_STREAM("FK solver failed when processing elbow link, something went wrong");
        exit(-1);
      }
      else{
        //ROS_INFO_STREAM("FK solver succeeded for elbow link.");
      }
      result = fk_solver.JntToCart(q_in, wrist_cart_out, num_wrist_seg+1);
      if (result < 0){
        ROS_INFO_STREAM("FK solver failed when processing wrist link, something went wrong");
        exit(-1);
      }
      else{
        //ROS_INFO_STREAM("FK solver succeeded for wrist link.");
      }
      result = fk_solver.JntToCart(q_in, shoulder_cart_out, num_shoulder_seg+1);
      if (result < 0){
        ROS_INFO_STREAM("FK solver failed when processing shoulder link, something went wrong");
        exit(-1);
        }
      else{
        //ROS_INFO_STREAM("FK solver succeeded for wrist link.");
      }

      // Preparations
      Vector3d elbow_pos_cur = Map<Vector3d>(elbow_cart_out.p.data, 3, 1);
      Vector3d wrist_pos_cur = Map<Vector3d>(wrist_cart_out.p.data, 3, 1);
      Matrix3d wrist_ori_cur = Map<Matrix<double, 3, 3, RowMajor> >(wrist_cart_out.M.data, 3, 3); 
      Vector3d shoulder_pos_cur = Map<Vector3d>(shoulder_cart_out.p.data, 3, 1);

      Vector3d shoulder_pos_human, elbow_pos_human, wrist_pos_human;
      Matrix3d wrist_ori_human;
      if (left_or_right) // left arm
      {
        shoulder_pos_human = fdata.l_shoulder_pos_goal;
        elbow_pos_human = fdata.l_elbow_pos_goal;
        wrist_pos_human = fdata.l_wrist_pos_goal;
        wrist_ori_human = fdata.l_wrist_ori_goal;

        fdata.l_wrist_cur = wrist_pos_cur;
      }
      else // right arm
      {
        shoulder_pos_human = fdata.r_shoulder_pos_goal;
        elbow_pos_human = fdata.r_elbow_pos_goal;
        wrist_pos_human = fdata.r_wrist_pos_goal;
        wrist_ori_human = fdata.r_wrist_ori_goal;

        fdata.r_wrist_cur = wrist_pos_cur;

      }


  // Compute cost function
  double scaled_wrist_pos_cost = (wrist_pos_cur - (wrist_pos_human - elbow_pos_human)/(wrist_pos_human - elbow_pos_human).norm() * (wrist_pos_cur - elbow_pos_cur).norm() - elbow_pos_cur).norm(); // coordinate difference, not in the way of vector difference // written as elbow-wrist form
  double scaled_elbow_pos_cost = (elbow_pos_cur - (elbow_pos_human - shoulder_pos_human)/(elbow_pos_human - shoulder_pos_human).norm() * (elbow_pos_cur - shoulder_pos_cur).norm() - shoulder_pos_cur).norm();
  double wrist_ori_cost = std::fabs( std::acos (( (wrist_ori_human * wrist_ori_cur.transpose()).trace() - 1.0) / 2.0));
  double cost = 1.0 * wrist_ori_cost + 1.0 * scaled_wrist_pos_cost + 1.0 * scaled_elbow_pos_cost;


  // Return cost function value
  return cost;

}


double MyUnaryConstraints::linear_map(double x_, double min_, double max_, double min_hat, double max_hat)
{
  return (x_ - min_) / (max_ - min_) * (max_hat - min_hat) + min_hat;
}


double MyUnaryConstraints::compute_finger_cost(Matrix<double, 12, 1> q_finger_robot, bool left_or_right, my_constraint_struct &fdata)
{
  // Obtain required data and parameter settings
  Matrix<double, 14, 1> q_finger_human;
  Matrix<double, 14, 1> human_finger_start = fdata.glove_start;
  Matrix<double, 14, 1> human_finger_final = fdata.glove_final;
  Matrix<double, 12, 1> robot_finger_start, robot_finger_final;
  if (left_or_right)
  {
    // Get the sensor data
    q_finger_human = fdata.l_finger_pos_goal;
    // Get bounds
    robot_finger_start = fdata.l_robot_finger_start;
    robot_finger_final = fdata.l_robot_finger_final;
  }
  else
  {
    // Get the sensor data
    q_finger_human = fdata.r_finger_pos_goal;    
    // Get bounds
    robot_finger_start = fdata.r_robot_finger_start;
    robot_finger_final = fdata.r_robot_finger_final;
  }


  // Direct mapping and linear scaling
  Matrix<double, 12, 1> q_finger_robot_goal;
  q_finger_robot_goal[0] = linear_map(q_finger_human[3], human_finger_start[3], human_finger_final[3], robot_finger_start[0], robot_finger_final[0]);
  q_finger_robot_goal[1] = linear_map(q_finger_human[4], human_finger_start[4], human_finger_final[4], robot_finger_start[1], robot_finger_final[1]);
  q_finger_robot_goal[2] = linear_map(q_finger_human[6], human_finger_start[6], human_finger_final[6], robot_finger_start[2], robot_finger_final[2]);
  q_finger_robot_goal[3] = linear_map(q_finger_human[7], human_finger_start[7], human_finger_final[7], robot_finger_start[3], robot_finger_final[3]);
  q_finger_robot_goal[4] = linear_map(q_finger_human[9], human_finger_start[9], human_finger_final[9], robot_finger_start[4], robot_finger_final[4]);
  q_finger_robot_goal[5] = linear_map(q_finger_human[10], human_finger_start[10], human_finger_final[10], robot_finger_start[5], robot_finger_final[5]);
  q_finger_robot_goal[6] = linear_map(q_finger_human[12], human_finger_start[12], human_finger_final[12], robot_finger_start[6], robot_finger_final[6]);
  q_finger_robot_goal[7] = linear_map(q_finger_human[13], human_finger_start[13], human_finger_final[13], robot_finger_start[7], robot_finger_final[7]);
  q_finger_robot_goal[8] = (robot_finger_start[8] + robot_finger_final[8]) / 2.0;
  q_finger_robot_goal[9] = linear_map(q_finger_human[2], human_finger_start[2], human_finger_final[2], robot_finger_start[9], robot_finger_final[9]);
  q_finger_robot_goal[10] = linear_map(q_finger_human[0], human_finger_start[0], human_finger_final[0], robot_finger_start[10], robot_finger_final[10]);
  q_finger_robot_goal[11] = linear_map(q_finger_human[1], human_finger_start[1], human_finger_final[1], robot_finger_start[11], robot_finger_final[11]); 

  
  // Compute cost
  double finger_cost = (q_finger_robot_goal - q_finger_robot).norm();


  if (left_or_right)
  {
    fdata.scaled_l_finger_pos_cost = finger_cost;
  }
  else
  {
    fdata.scaled_r_finger_pos_cost = finger_cost;
  }

  return finger_cost;

}



void MyUnaryConstraints::computeError()
{
  
  // get the current joint value
  // _vertices is a VertexContainer type, a std::vector<Vertex*>
  const DualArmDualHandVertex *v = static_cast<const DualArmDualHandVertex*>(_vertices[0]);
  const Matrix<double, JOINT_DOF, 1> x = v->estimate(); // return the current estimate of the vertex

  // Get joint angles
  Matrix<double, 7, 1> q_cur_l, q_cur_r;
  Matrix<double, 12, 1> q_cur_finger_l, q_cur_finger_r;
  q_cur_l = x.block<7, 1>(0, 0);
  q_cur_r = x.block<7, 1>(7, 0);
  q_cur_finger_l = x.block<12, 1>(14, 0);
  q_cur_finger_r = x.block<12, 1>(26, 0); 
  
  // Compute unary costs
  // 1
  double arm_cost = compute_arm_cost(left_fk_solver, q_cur_l, _measurement.l_num_wrist_seg, _measurement.l_num_elbow_seg, _measurement.l_num_shoulder_seg, true, _measurement); // user data is stored in _measurement now
  arm_cost += compute_arm_cost(right_fk_solver, q_cur_r, _measurement.r_num_wrist_seg, _measurement.r_num_elbow_seg, _measurement.r_num_shoulder_seg, false, _measurement);
  //std::cout << "Arm cost=";
  //std::cout << arm_cost << ", ";
  // 2
  double finger_cost = compute_finger_cost(q_cur_finger_l, true, _measurement);  
  finger_cost += compute_finger_cost(q_cur_finger_r, false, _measurement);  
  //std::cout << "Finger cost=";
  //std::cout << finger_cost << ", ";
  // 3
  double collision_cost = compute_collision_cost(x, dual_arm_dual_hand_collision_ptr);
  //std::cout << "Collision cost=";
  //std::cout << collision_cost << ", ";
  // 4
  double pos_limit_cost = compute_pos_limit_cost(x, _measurement);
  //std::cout << "Pos limit cost=";
  //std::cout << pos_limit_cost << "; ";

  // total cost
  double cost = arm_cost + finger_cost + 2.0*collision_cost + 5.0*pos_limit_cost;
  //std::cout << "Total cost=";
  //std::cout << cost << std::endl;


  // protected attributes
  // _measurement, set by setMeasurement() method, a deep copy;
  // _information, set by setInformation() method;
  // _error, needed to be computed

  // compute the cost (constraint value)
  _error(0, 0) = cost;


  // Display message
  //std::cout << "Computing unary edge's cost: " << cost << std::endl;
            
}




/* Constraint for smoothness between adjacent path points */
class SmoothnessConstraint : public BaseBinaryEdge<1, double, DualArmDualHandVertex, DualArmDualHandVertex>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    void computeError()
    {
      
      // Get the values of the two vertices
      const DualArmDualHandVertex *v0 = static_cast<const DualArmDualHandVertex*>(_vertices[0]);
      const DualArmDualHandVertex *v1 = static_cast<const DualArmDualHandVertex*>(_vertices[1]);
      const Matrix<double, JOINT_DOF, 1> x0 = v0->estimate(); 
      const Matrix<double, JOINT_DOF, 1> x1 = v1->estimate(); 

      // Compute smoothness cost
      _error(0, 0) = 10.0 * (x0 - x1).norm();
      //std::cout << "Smoothness cost=";
      //std::cout << _error(0, 0) << std::endl;

      // Display message
      //std::cout << "Computing binary edge's cost: " << _error(0, 0) << std::endl;

    }

    // Read and write, leave blank
    virtual bool read( std::istream& in ) {}
    virtual bool write( std::ostream& out ) const {}

};



// class VecLimitConstraint : public BaseBinaryEdge<>{}; // add in timestamp variables

// class AccLimitConstraint : public BaseMultiEdge<>{}; // add in timestamp variables

// class CoordinationConstraint : public BaseBinaryEdge<>{}; // Coordination constraints to keep


std::stringstream read_file(std::string file_name)
{
  std::ifstream ifs(file_name);
  std::stringstream ss;
  ss << ifs.rdbuf();
  return ss;
}



int main(int argc, char *argv[])
{

  // Initialize a ros node, for the calculation of collision distance
  ros::init(argc, argv, "yumi_sign_language_robot_retargeting");


  // Optimization settings
  std::string in_file_name = "test_imi_data_YuMi.h5";
  std::string in_group_name = "fengren_1";
  std::string out_file_name = "mocap_ik_results_YuMi_g2o.h5";


  // Process the terminal arguments
  static struct option long_options[] = 
  {
    {"in-h5-filename", required_argument, NULL, 'i'},
    {"in-group-name", required_argument, NULL, 'g'},
    {"out-h5-filename", required_argument, NULL, 'o'},
    {"help", no_argument, NULL, 'h'},
    {0, 0, 0, 0}
  };
  int c;
  while(1)
  {
    int opt_index = 0;
    // Get arguments
    c = getopt_long(argc, argv, "i:g:o:h", long_options, &opt_index);
    if (c == -1)
      break;

    // Process
    switch(c)
    {
      case 'h':
        std::cout << "Help: \n" << std::endl;
        std::cout << "    This program reads imitation data from h5 file and performs optimization on the joint angles. The results are stored in a h5 file at last.\n" << std::endl; 
        std::cout << "Arguments:\n" << std::endl;
        std::cout << "    -i, --in-h5-filename, specify the name of the input h5 file, otherwise a default name specified inside the program will be used. Suffix is required.\n" << std::endl;
        std::cout << "    -g, --in-group-name, specify the group name in the h5 file, which is actually the motion's name.\n" << std::endl;
        std::cout << "    -o, --out-h5-name, specify the name of the output h5 file to store the resultant joint trajectory.\n" << std::endl;
        return 0;
        break;

      case 'i':
        in_file_name = optarg;
        break;

      case 'o':
        out_file_name = optarg;
        break;

      case 'g':
        in_group_name = optarg;
        break;

      default:
        break;
    }

  }
  std::cout << "The input h5 file name is: " << in_file_name << std::endl;
  std::cout << "The motion name is: " << in_group_name << std::endl;
  std::cout << "The output h5 file name is: " << out_file_name << std::endl;






  // Create a struct for storing user-defined data
  my_constraint_struct constraint_data; 


  // Input Cartesian trajectories
  std::vector<double> x(JOINT_DOF);
  std::cout << "========== Reading imitation data from h5 file ==========" << std::endl;
  std::vector<std::vector<double>> read_l_wrist_pos_traj = read_h5(in_file_name, in_group_name, "l_wrist_pos"); 
  std::vector<std::vector<double>> read_l_wrist_ori_traj = read_h5(in_file_name, in_group_name, "l_wrist_ori"); 
  std::vector<std::vector<double>> read_l_elbow_pos_traj = read_h5(in_file_name, in_group_name, "l_elbow_pos"); 
  std::vector<std::vector<double>> read_l_shoulder_pos_traj = read_h5(in_file_name, in_group_name, "l_shoulder_pos"); 

  std::vector<std::vector<double>> read_r_wrist_pos_traj = read_h5(in_file_name, in_group_name, "r_wrist_pos"); 
  std::vector<std::vector<double>> read_r_wrist_ori_traj = read_h5(in_file_name, in_group_name, "r_wrist_ori"); 
  std::vector<std::vector<double>> read_r_elbow_pos_traj = read_h5(in_file_name, in_group_name, "r_elbow_pos"); 
  std::vector<std::vector<double>> read_r_shoulder_pos_traj = read_h5(in_file_name, in_group_name, "r_shoulder_pos"); 

  std::vector<std::vector<double>> read_l_finger_pos_traj = read_h5(in_file_name, in_group_name, "l_glove_angle"); // N * 14 size
  std::vector<std::vector<double>> read_r_finger_pos_traj = read_h5(in_file_name, in_group_name, "r_glove_angle"); // N * 14 size  
  std::vector<std::vector<double>> read_time_stamps = read_h5(in_file_name, in_group_name, "time"); 


  unsigned int num_datapoints = read_l_wrist_pos_traj.size(); 


 // Variables' bounds
  const std::vector<double> q_l_arm_lb = {-2.9, -2.49, 0.0, -1.7, -1.578, -1.5, -1.57};//{-2.94, -2.5, -2.94, -2.16, -5.06, -1.54, -4.0};
  const std::vector<double> q_r_arm_lb = {-2.9, -2.49, 0.0, -1.7, -1.578, -1.5, -1.57};//{-2.94, -2.50, -2.94, -2.16, -5.06, -1.54, -4.0};
  const std::vector<double> q_l_arm_ub = {0.5, 0.75, 2.9, 1.4, 1.578, 2.1, 1.57};//{2.94, 0.76, 2.94, 1.4, 5.06, 2.41, 4.0};
  const std::vector<double> q_r_arm_ub = {0.5, 0.75, 2.9, 1.4, 1.578, 2.1, 1.57};//{2.94, 0.76, 2.94, 1.4, 5.06, 2.41, 4.0};

  const std::vector<double> q_l_finger_lb = {-1.6, -1.7, -1.6, -1.7, -1.6, -1.7, -1.6, -1.7, -1.0, 0.0, -0.4, -1.0};
  const std::vector<double> q_l_finger_ub = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3, 0.4, 0.0, 0.0};
  const std::vector<double> q_r_finger_lb = {-1.6, -1.7, -1.6, -1.7, -1.6, -1.7, -1.6, -1.7, -1.0, 0.0, -0.4, -1.0};
  const std::vector<double> q_r_finger_ub = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3, 0.4, 0.0, 0.0};

  std::vector<double> qlb = q_l_arm_lb;
  std::vector<double> qub = q_l_arm_ub;
  qlb.insert(qlb.end(), q_r_arm_lb.cbegin(), q_r_arm_lb.cend());
  qlb.insert(qlb.end(), q_l_finger_lb.cbegin(), q_l_finger_lb.cend());
  qlb.insert(qlb.end(), q_r_finger_lb.cbegin(), q_r_finger_lb.cend());
  qub.insert(qub.end(), q_r_arm_ub.cbegin(), q_r_arm_ub.cend());
  qub.insert(qub.end(), q_l_finger_ub.cbegin(), q_l_finger_ub.cend());
  qub.insert(qub.end(), q_r_finger_ub.cbegin(), q_r_finger_ub.cend());
  Matrix<double, JOINT_DOF, 1> q_pos_lb = Map<Matrix<double, JOINT_DOF, 1> >(qlb.data(), JOINT_DOF, 1);
  Matrix<double, JOINT_DOF, 1> q_pos_ub = Map<Matrix<double, JOINT_DOF, 1> >(qub.data(), JOINT_DOF, 1);
  constraint_data.q_pos_lb = q_pos_lb;
  constraint_data.q_pos_ub = q_pos_ub;


  // Setup KDL FK solver( and set IDs for wrist, elbow and shoulder)
  std::cout << "========== Setting KDL FK Solvers ==========" << std::endl;
  KDL::ChainFkSolverPos_recursive left_fk_solver = setup_left_kdl(constraint_data);
  KDL::ChainFkSolverPos_recursive right_fk_solver = setup_right_kdl(constraint_data); 


  // Robot configuration-related data
  // robot's shoulder position
  constraint_data.l_robot_shoulder_pos = Vector3d(0.05355, 0.0725, 0.51492); //Vector3d(-0.06, 0.235, 0.395);
  constraint_data.r_robot_shoulder_pos = Vector3d(0.05255, -0.0725, 0.51492); //Vector3d(-0.06, -0.235, 0.395);
  constraint_data.l_robot_finger_start << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3, 0.4, 0.0, 0.0; 
  constraint_data.l_robot_finger_final << -1.6, -1.7, -1.6, -1.7, -1.6, -1.7, -1.6, -1.7, -1.0, 0.0, -0.4, -1.0;
  constraint_data.r_robot_finger_start << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3, 0.4, 0.0, 0.0;
  constraint_data.r_robot_finger_final << -1.6, -1.7, -1.6, -1.7, -1.6, -1.7, -1.6, -1.7, -1.0, 0.0, -0.4, -1.0; // right and left hands' joint ranges are manually set to be the same, but according to Inspire Hand Inc, this will keep changing in the future.
  constraint_data.glove_start << 0, 0, 53, 0, 0, 22, 0, 0, 22, 0, 0, 35, 0, 0;
  constraint_data.glove_start = constraint_data.glove_start * M_PI / 180.0; // in radius  
  constraint_data.glove_final << 45, 100, 0, 90, 120, 0, 90, 120, 0, 90, 120, 0, 90, 120;
  constraint_data.glove_final = constraint_data.glove_final * M_PI / 180.0; 


  // Others
  constraint_data.argc = argc;
  constraint_data.argv = argv;

  
  // Construct a graph optimization problem 
  std::cout << "========== Constructing an optimization graph ==========" << std::endl;
  // Construct solver (be careful, use unique_ptr instead!!! )
  typedef BlockSolver<BlockSolverTraits<JOINT_DOF, 1> > Block; // BlockSolverTraits<_PoseDim, _LandmarkDim>

  //std::unique_ptr<Block::LinearSolverType> linearSolver( new LinearSolverDense<Block::PoseMatrixType>()); // dense solution
  std::unique_ptr<Block::LinearSolverType> linearSolver( new LinearSolverCholmod<Block::PoseMatrixType>()); // Cholmod
  //std::unique_ptr<Block::LinearSolverType> linearSolver( new LinearSolverCSparse<Block::PoseMatrixType>()); // CSparse
  //std::unique_ptr<Block::LinearSolverType> linearSolver( new LinearSolverPCG<Block::PoseMatrixType>()); // PCG

  std::unique_ptr<Block> solver_ptr( new Block(std::move(linearSolver)) ); // Matrix block solver

  // Choose update rule
  OptimizationAlgorithmLevenberg *solver = new OptimizationAlgorithmLevenberg(std::move(solver_ptr));
  //OptimizationAlgorithmGaussNewton *solver = new OptimizationAlgorithmGaussNewton(std::move(solver_ptr));
  //OptimizationAlgorithmDogleg *solver = new OptimizationAlgorithmDogleg(solver_ptr);

  // Construct an optimizer (a graph model)
  SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(true);


  // Prepare collision checker
  std::cout << "========== Preparing collision checker ==========" << std::endl;
  // Get URDF and SRDF for distance computation class
  std::string urdf_file_name = "/home/liangyuwei/sign_language_robot_ws/src/yumi_description/urdf/yumi_with_hands.urdf";
  std::string srdf_file_name = "/home/liangyuwei/sign_language_robot_ws/src/yumi_sign_language_robot_moveit_config/config/yumi.srdf";
  std::stringstream urdf_string = read_file(urdf_file_name);
  std::stringstream srdf_string = read_file(srdf_file_name);
  boost::shared_ptr<DualArmDualHandCollision> dual_arm_dual_hand_collision_ptr;
  dual_arm_dual_hand_collision_ptr.reset( new DualArmDualHandCollision(urdf_string.str(), srdf_string.str()) );


  // Add vertices and edges
  std::cout << "========== Adding vertices and unary edges ==========" << std::endl;
  //num_datapoints = 3;
  std::vector<DualArmDualHandVertex*> v_list(num_datapoints);
  std::vector<MyUnaryConstraints*> unary_edges;
  std::vector<SmoothnessConstraint*> smoothness_edges;  
  for (unsigned int it = 0; it < num_datapoints; ++it)
  {
    // add vertices
    //DualArmDualHandVertex *v = new DualArmDualHandVertex();
    v_list[it] = new DualArmDualHandVertex();
    v_list[it]->setEstimate(Matrix<double, JOINT_DOF, 1>::Identity()); // feed in initial guess
    v_list[it]->setId(it); // set a unique id
    optimizer.addVertex(v_list[it]);

    // set up path point info
    std::vector<double> l_wrist_pos = read_l_wrist_pos_traj[it]; // 3-dim
    std::vector<double> l_wrist_ori = read_l_wrist_ori_traj[it]; // 9-dim
    std::vector<double> l_elbow_pos = read_l_elbow_pos_traj[it]; //end()); // 3-dim
    std::vector<double> l_shoulder_pos = read_l_shoulder_pos_traj[it]; 
  
    std::vector<double> r_wrist_pos = read_r_wrist_pos_traj[it]; // 3-dim
    std::vector<double> r_wrist_ori = read_r_wrist_ori_traj[it]; // 9-dim
    std::vector<double> r_elbow_pos = read_r_elbow_pos_traj[it]; //end()); // 3-dim
    std::vector<double> r_shoulder_pos = read_r_shoulder_pos_traj[it]; 

    std::vector<double> l_finger_pos = read_l_finger_pos_traj[it];
    std::vector<double> r_finger_pos = read_r_finger_pos_traj[it];

    // convert to Eigen data type
    Vector3d l_wrist_pos_goal = Map<Vector3d>(l_wrist_pos.data(), 3, 1);
    Matrix3d l_wrist_ori_goal = Map<Matrix<double, 3, 3, RowMajor>>(l_wrist_ori.data(), 3, 3);
    Vector3d l_elbow_pos_goal = Map<Vector3d>(l_elbow_pos.data(), 3, 1);
    Vector3d l_shoulder_pos_goal = Map<Vector3d>(l_shoulder_pos.data(), 3, 1);

    Vector3d r_wrist_pos_goal = Map<Vector3d>(r_wrist_pos.data(), 3, 1);
    Matrix3d r_wrist_ori_goal = Map<Matrix<double, 3, 3, RowMajor>>(r_wrist_ori.data(), 3, 3);
    Vector3d r_elbow_pos_goal = Map<Vector3d>(r_elbow_pos.data(), 3, 1);
    Vector3d r_shoulder_pos_goal = Map<Vector3d>(r_shoulder_pos.data(), 3, 1);

    Matrix<double, 14, 1> l_finger_pos_goal = Map<Matrix<double, 14, 1>>(l_finger_pos.data(), 14, 1);
    Matrix<double, 14, 1> r_finger_pos_goal = Map<Matrix<double, 14, 1>>(r_finger_pos.data(), 14, 1);

    // Save in constraint_data for use in optimization
    constraint_data.l_wrist_pos_goal = l_wrist_pos_goal;
    constraint_data.l_wrist_ori_goal = l_wrist_ori_goal;
    constraint_data.l_elbow_pos_goal = l_elbow_pos_goal;
    constraint_data.l_shoulder_pos_goal = l_shoulder_pos_goal;

    constraint_data.r_wrist_pos_goal = r_wrist_pos_goal;
    constraint_data.r_wrist_ori_goal = r_wrist_ori_goal;
    constraint_data.r_elbow_pos_goal = r_elbow_pos_goal;
    constraint_data.r_shoulder_pos_goal = r_shoulder_pos_goal;

    constraint_data.l_finger_pos_goal = l_finger_pos_goal * M_PI / 180.0; // remember to convert from degree to radius!!!
    constraint_data.r_finger_pos_goal = r_finger_pos_goal * M_PI / 180.0;

    // add unary edges
    MyUnaryConstraints *unary_edge = new MyUnaryConstraints(left_fk_solver, right_fk_solver, dual_arm_dual_hand_collision_ptr);
    unary_edge->setId(it);
    unary_edge->setVertex(0, optimizer.vertex(it)); //(0, v_list[it]); // set the 0th vertex on the edge to point to v_list[it]
    unary_edge->setMeasurement(constraint_data); // set _measurement attribute (by deep copy), can be used to pass in user data, e.g. my_constraint_struct
    unary_edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity()); // information matrix, inverse of covariance.. importance // Information type correct
    optimizer.addEdge(unary_edge);

    unary_edges.push_back(unary_edge);
    
  }

  
  std::cout << "========== Adding binary edges ==========" << std::endl;
  for (unsigned int it = 0; it < num_datapoints - 1; ++it)
  {
    // Add binary edges
    SmoothnessConstraint *smoothness_edge = new SmoothnessConstraint();
    smoothness_edge->setId(it+num_datapoints); // set a unique ID
    smoothness_edge->setVertex(0, optimizer.vertex(it)); //v_list[it]);
    smoothness_edge->setVertex(1, optimizer.vertex(it+1)); //v_list[it+1]); // binary edge, connects only 2 vertices, i.e. i=0 and i=1
    smoothness_edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity()); // Information type correct
    optimizer.addEdge(smoothness_edge);

    smoothness_edges.push_back(smoothness_edge);

  }


  

  // Start optimization
  std::cout << "Start optimization:" << std::endl;
  std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
  optimizer.initializeOptimization();
  //optimizer.computeInitialGuess();  
  //optimizer.computeActiveErrors();

  std::cout << "optimizing graph, vertices: " << optimizer.vertices().size() << std::endl;

  //bool saveFlag = optimizer.save("./g2o_results/result_before.g2o");
  //std::cout << "g2o file saved " << (saveFlag? "successfully" : "unsuccessfully") << " ." << std::endl;


  optimizer.optimize(100); // optimize for a number of iterations
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  std::chrono::duration<double> t_spent = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  std::cout << "Total time used for optimization: " << t_spent.count() << " s" << std::endl;

  //saveFlag = optimizer.save("./g2o_results/result_after.g2o");
  //std::cout << "g2o file saved " << (saveFlag? "successfully" : "unsuccessfully") << " ." << std::endl;

  std::cout << "Optimization done." << std::endl;
  

  // estimate Count of inlier ???
  /*
  int inliers = 0;
  for (auto e:unary_edges)
  {
    e->computeError();

    std::cout << "unary edge error = " << e->chi2() << std::endl;

  }
  for (auto e:smoothness_edges)
  {
    e->computeError();

    std::cout << "smoothness edge error = " << e->chi2() << std::endl;

  }
  */



  // Store the optimization value
  Matrix<double, JOINT_DOF, 1> q_result = v_list[0]->estimate();
  std::cout << std::endl << "Joint values for the path point 1/" << num_datapoints << ": " << q_result.transpose() << std::endl; // check if the setId() method can be used to get the value

  q_result = v_list[1]->estimate();
  std::cout << std::endl << "Joint values for the path point 2/" << num_datapoints << ": " << q_result.transpose() << std::endl; // check if the setId() method can be used to get the value

  q_result = v_list[2]->estimate();
  std::cout << std::endl << "Joint values for the path point 3/" << num_datapoints << ": " << q_result.transpose() << std::endl; // check if the setId() method can be used to get the value

  
  bool PSDFlag = optimizer.verifyInformationMatrices(true);

  if(PSDFlag)
    std::cout << "All information matrices are Positive Semi-Definite." << std::endl;




  return 0;

}













