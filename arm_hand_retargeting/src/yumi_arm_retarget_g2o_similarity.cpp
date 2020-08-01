// ROS
#include <ros/ros.h>

// Common
#include <vector>
#include <math.h>
#include <iostream>
#include <chrono>
#include <string>
#include <cfloat>

// For acos, fabs, pow
#include <cmath>

// For max
#include <algorithm>

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
#include <g2o/solvers/eigen/linear_solver_eigen.h>

// For Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <Eigen/QR>


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


// For similarity network (libtorch)
#include "similarity_network_pytorch.h"


// For trajectory generator
//#include "generate_trajectory_from_viewpoints.h" // residual way, encoding trajectories by pass points
#include "generate_trajectory_using_DMP.h" // DMPs encoding relative position trajectories


// Macros
#define JOINT_DOF 38
#define PASSPOINT_DOF 48
#define DMPPOINTS_DOF 24 // 4 (DMP) x 2 (start+goal) x 3 (3d pos)
#define NUM_DATAPOINTS 50 //100 // pre-defined, fixed
//#define NUM_PASSPOINTS 25 // BlockSolver must known this at compile time... yet it could also be dynamic!!! BlockSolver<-1, -1>

// weights for different parts of cost
/*
#define K_COL 10.0
#define K_POS_LIMIT 10.0
#define K_WRIST_ORI 3.0
#define K_WRIST_POS 3.0
#define K_ELBOW_POS 3.0
#define K_FINGER 3.0
#define K_SIMILARITY 5.0//1000.0 // 1.0 // 10.0
#define K_SMOOTHNESS 1.0
*/

double K_COL = 10.0;
double K_POS_LIMIT = 10.0;
double K_WRIST_ORI = 3.0;
double K_WRIST_POS = 3.0;
double K_ELBOW_POS = 3.0;
double K_FINGER = 3.0;
double K_SIMILARITY = 5.0; //1000.0 // 1.0 // 10.0
double K_SMOOTHNESS = 1.0;
double K_DMPSTARTSGOALS = 1.0;
double K_DMPSCALEMARGIN = 1.0;
double K_DMPRELCHANGE = 1.0;

// bool collision_check_or_distance_compute = true;

using namespace g2o;
using namespace Eigen;
using namespace H5;


unsigned int count_col = 0;
unsigned int count_sim = 0;
unsigned int count_traj = 0;
unsigned int count_unary = 0;
unsigned int count_smoothness = 0;
unsigned int count_tracking = 0;

double total_col = 0;
double total_sim = 0;
double total_traj = 0;
double total_unary = 0;
double total_smoothness = 0;
double total_tracking = 0;

unsigned int num_update = 0;
unsigned int num_track = 0;
unsigned int num_sim = 0;

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


  // for selecting point on new trajectory (y_seq)
  unsigned int point_id = 0;
  

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

bool write_h5_3d(const std::string file_name, const std::string group_name, const std::string dataset_name, const int LAYER, const int ROW, const int COL, std::vector<std::vector<std::vector<double> > > data_vector)
{
  // Set up file name and dataset name
  const H5std_string FILE_NAME(file_name);
  const H5std_string GROUP_NAME(group_name);
  const H5std_string DATASET_NAME(dataset_name);

  // Convert 2-dim std::vector to 2-dim raw buffer(array)
  double data[LAYER][ROW][COL];
  for (int k = 0; k < LAYER; k++)
  {
    for (int j = 0; j < ROW; j++)
    {
      for (int i = 0; i < COL; i++)    
        data[k][j][i] = data_vector[k][j][i];
    }
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
    hsize_t dimsf[3];              // dataset dimensions
    dimsf[0] = LAYER;
    dimsf[1] = ROW;
    dimsf[2] = COL;
    DataSpace dataspace(3, dimsf);
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
    H5T_class_t type_class = dataset.getTypeClass();

    // Get dataspace of the dataset.
    DataSpace dataspace = dataset.getSpace();

    // Get the dimension size of each dimension in the dataspace and display them.
    hsize_t dims_out[2];
    int ndims = dataspace.getSimpleExtentDims( dims_out, NULL); // though ndims is not used, this line of code assigns data to dims_out!!!
    int ROW = dims_out[0], COL = dims_out[1];

    // Read data into raw buffer(array) and convert to std::vector
    double data_array[ROW][COL];
    dataset.read(data_array, PredType::NATIVE_DOUBLE);
    std::vector<std::vector<double>> data_vector(ROW, std::vector<double>(COL));
    for (unsigned int j = 0; j < dims_out[0]; j++)
    {
      for (unsigned int i = 0; i < dims_out[1]; i++)
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
      {
        _estimate[i] += update[i];//Map<Matrix<double, JOINT_DOF, 1> >(update, JOINT_DOF, 1);
        last_update(i, 0) = update[i]; // record updates
      }

    }


    Matrix<double, JOINT_DOF, 1> last_update;

    // Read and write, leave blank
    virtual bool read( std::istream& in ) {return true;}
    virtual bool write( std::ostream& out ) const {return true;}
};


/* Define vertex for starts and goals of DMPs that encode position trajectories */
/* Order: lrw, lew, rew, rw; goal, start. Column vector */
class DMPStartsGoalsVertex : public BaseVertex<DMPPOINTS_DOF, Matrix<double, DMPPOINTS_DOF, 1> >
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual void setToOriginImpl() // reset
    {
      _estimate << Matrix<double, DMPPOINTS_DOF, 1>::Zero();
    }
  
    virtual void oplusImpl(const double *update) // update rule
    {
      //std::cout << "debug: update DMP starts and goals " << std::endl;
      num_update++;
      for (unsigned int i = 0; i < DMPPOINTS_DOF; ++i)
      {
        _estimate[i] += update[i];
        last_update(i, 0) = update[i]; // record updates
      }
      //std::cout << "debug: current update = " << last_update.transpose() << std::endl;

    }


    Matrix<double, DMPPOINTS_DOF, 1> last_update;

    // Read and write, leave blank
    virtual bool read( std::istream& in ) {return true;}
    virtual bool write( std::ostream& out ) const {return true;}

};



/* Define vertex for pass points */
class PassPointVertex : public BaseVertex<PASSPOINT_DOF, Matrix<double, PASSPOINT_DOF, 1> >
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual void setToOriginImpl() // reset
    {
      _estimate << Matrix<double, PASSPOINT_DOF, 1>::Zero();
    }
  
    virtual void oplusImpl(const double *update) // update rule
    {
      for (unsigned int i = 0; i < PASSPOINT_DOF; ++i)
        _estimate[i] += update[i];
    }

    // Read and write, leave blank
    virtual bool read( std::istream& in ) {return true;}
    virtual bool write( std::ostream& out ) const {return true;}

};


/* Define constraint for constraining the change of starts and goals */
class DMPConstraints : public BaseUnaryEdge<1, my_constraint_struct, DMPStartsGoalsVertex>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    DMPConstraints(Matrix<double, 1, 3> lrw_goal, Matrix<double, 1, 3> lrw_start, 
                   Matrix<double, 1, 3> lew_goal, Matrix<double, 1, 3> lew_start, 
                   Matrix<double, 1, 3> rew_goal, Matrix<double, 1, 3> rew_start,
                   Matrix<double, 1, 3> rw_goal, Matrix<double, 1, 3> rw_start)
    {

      // initialization
      this->lrw_goal = lrw_goal;
      this->lrw_start = lrw_start;
      this->lew_goal = lew_goal;
      this->lew_start = lew_start;
      this->rew_goal = rew_goal;
      this->rew_start = rew_start;
      this->rw_goal = rw_goal;
      this->rw_start = rw_start;          

      // Get vectors pointing from starts to goals
      this->lrw_vec = lrw_goal - lrw_start;
      this->lew_vec = lew_goal - lew_start;
      this->rew_vec = rew_goal - rew_start;
      this->rw_vec = rw_goal - rw_start;

      // Store the lengths
      this->lrw_len = this->lrw_vec.norm();
      this->lew_len = this->lew_vec.norm();
      this->rew_len = this->rew_vec.norm();
      this->rw_len = this->rw_vec.norm();
      
    }

    // Information about the original trajectories
    Matrix<double, 1, 3> lrw_goal; // 1 x 3
    Matrix<double, 1, 3> lrw_start;
    Matrix<double, 1, 3> lew_goal;
    Matrix<double, 1, 3> lew_start;
    Matrix<double, 1, 3> rew_goal;
    Matrix<double, 1, 3> rew_start;
    Matrix<double, 1, 3> rw_goal;
    Matrix<double, 1, 3> rw_start;  // original starts and goals

    // Vectors pointing from starts to goals
    Matrix<double, 1, 3> lrw_vec; // 1 x 3
    Matrix<double, 1, 3> lew_vec;
    Matrix<double, 1, 3> rew_vec;
    Matrix<double, 1, 3> rw_vec;

    // Lengths of vectors connecting starts and goals
    double lrw_len;
    double lew_len;
    double rew_len;
    double rw_len;

    // thresholds
    double max_theta = 5.0 * M_PI / 180.0;  // 5 deg max

    // Store jacobians
    Matrix<double, 1, DMPPOINTS_DOF> orien_jacobians_for_dmp;  // the jacobians of vector orientation cost w.r.t DMP starts and goals
    Matrix<double, 1, DMPPOINTS_DOF> scale_jacobians_for_dmp;  // the jacobians of scale cost w.r.t DMP starts and goals
    Matrix<double, 1, DMPPOINTS_DOF> rel_jacobians_for_dmp;

    // function to compute costs
    double compute_orien_cost(Matrix<double, DMPPOINTS_DOF, 1> x);
    double compute_scale_cost(Matrix<double, DMPPOINTS_DOF, 1> x);
    double compute_rel_change_cost(Matrix<double, DMPPOINTS_DOF, 1> x);    
    double output_orien_cost();
    double output_scale_cost();
    double output_rel_change_cost();

    void computeError()
    {
      // Get new DMP starts and goals
      const DMPStartsGoalsVertex *v = dynamic_cast<const DMPStartsGoalsVertex*>(_vertices[0]); // the last vertex connected
      Matrix<double, DMPPOINTS_DOF, 1> x = v->estimate();
   
      // set error
      _error(0, 0) = K_DMPSTARTSGOALS * this->compute_orien_cost(x) + 
                     K_DMPSCALEMARGIN * this->compute_scale_cost(x) +
                     K_DMPRELCHANGE * this->compute_rel_change_cost(x);

    }

    // Compute jacobians
    virtual void linearizeOplus();
    Matrix<double, 1, DMPPOINTS_DOF> output_orien_jacobian(); 
    Matrix<double, 1, DMPPOINTS_DOF> output_scale_jacobian(); 
    Matrix<double, 1, DMPPOINTS_DOF> output_rel_change_jacobian();

    // Read and write, leave blank
    virtual bool read( std::istream& in ) {return true;}
    virtual bool write( std::ostream& out ) const {return true;}

};

double DMPConstraints::output_orien_cost()
{
  // Get new DMP starts and goals
  const DMPStartsGoalsVertex *v = dynamic_cast<const DMPStartsGoalsVertex*>(_vertices[0]); // the last vertex connected
  Matrix<double, DMPPOINTS_DOF, 1> x = v->estimate();
   
  // set error
  double orien_cost = this->compute_orien_cost(x);

  return orien_cost;
}

double DMPConstraints::output_scale_cost()
{
  // Get new DMP starts and goals
  const DMPStartsGoalsVertex *v = dynamic_cast<const DMPStartsGoalsVertex*>(_vertices[0]); // the last vertex connected
  Matrix<double, DMPPOINTS_DOF, 1> x = v->estimate();
   
  // set error
  double scale_cost = this->compute_scale_cost(x);

  return scale_cost;

}

double DMPConstraints::output_rel_change_cost()
{
  // Get new DMP starts and goals
  const DMPStartsGoalsVertex *v = dynamic_cast<const DMPStartsGoalsVertex*>(_vertices[0]); // the last vertex connected
  Matrix<double, DMPPOINTS_DOF, 1> x = v->estimate();
   
  // set error
  double rel_change_cost = this->compute_rel_change_cost(x);

  return rel_change_cost;

}


Matrix<double, 1, DMPPOINTS_DOF> DMPConstraints::output_orien_jacobian()
{
  return this->orien_jacobians_for_dmp;
} 

Matrix<double, 1, DMPPOINTS_DOF> DMPConstraints::output_scale_jacobian()
{
  return this->scale_jacobians_for_dmp;
} 

Matrix<double, 1, DMPPOINTS_DOF> DMPConstraints::output_rel_change_jacobian()
{
  return this->rel_jacobians_for_dmp;
} 


double DMPConstraints::compute_orien_cost(Matrix<double, DMPPOINTS_DOF, 1> x)
{
  // get goals and starts
  Matrix<double, 3, 1> lrw_new_goal; lrw_new_goal = x.block(0, 0, 3, 1);
  Matrix<double, 3, 1> lrw_new_start; lrw_new_start = x.block(3, 0, 3, 1);
  Matrix<double, 3, 1> lew_new_goal; lew_new_goal = x.block(6, 0, 3, 1);
  Matrix<double, 3, 1> lew_new_start; lew_new_start = x.block(9, 0, 3, 1);
  Matrix<double, 3, 1> rew_new_goal; rew_new_goal = x.block(12, 0, 3, 1);
  Matrix<double, 3, 1> rew_new_start; rew_new_start = x.block(15, 0, 3, 1);
  Matrix<double, 3, 1> rw_new_goal; rw_new_goal = x.block(18, 0, 3, 1);
  Matrix<double, 3, 1> rw_new_start; rw_new_start = x.block(21, 0, 3, 1);

  // Get new vectors
  Matrix<double, 3, 1> lrw_new_vec; lrw_new_vec = lrw_new_goal - lrw_new_start;
  Matrix<double, 3, 1> lew_new_vec; lew_new_vec = lew_new_goal - lew_new_start;
  Matrix<double, 3, 1> rew_new_vec; rew_new_vec = rew_new_goal - rew_new_start;
  Matrix<double, 3, 1> rw_new_vec; rw_new_vec = rw_new_goal - rw_new_start;

  // Get angle from acos (in radius, absolute value)
  double cos_lrw_theta = std::min((double)(this->lrw_vec * lrw_new_vec) / (this->lrw_vec.norm() * lrw_new_vec.norm()), 1.0);
  double cos_lew_theta = std::min((double)(this->lew_vec * lew_new_vec) / (this->lew_vec.norm() * lew_new_vec.norm()), 1.0);
  double cos_rew_theta = std::min((double)(this->rew_vec * rew_new_vec) / (this->rew_vec.norm() * rew_new_vec.norm()), 1.0);
  double cos_rw_theta = std::min((double)(this->rw_vec * rw_new_vec) / (this->rw_vec.norm() * rw_new_vec.norm()), 1.0);
  double lrw_theta = std::fabs(std::acos(cos_lrw_theta));
  double lew_theta = std::fabs(std::acos(cos_lew_theta));
  double rew_theta = std::fabs(std::acos(cos_rew_theta));
  double rw_theta = std::fabs(std::acos(cos_rw_theta));

  /*
  std::cout << "debug: original lrw_vec = " << this->lrw_vec << ", new lrw_vec = " << lrw_new_vec.transpose() << std::endl;
  std::cout << "debug: original lew_vec = " << this->lew_vec << ", new lew_vec = " << lew_new_vec.transpose() << std::endl;
  std::cout << "debug: original rew_vec = " << this->rew_vec << ", new rew_vec = " << rew_new_vec.transpose() << std::endl;
  std::cout << "debug: original rw_vec = " << this->rw_vec << ", new rw_vec = " << rw_new_vec.transpose() << std::endl;
  std::cout << "debug: " << std::endl << "lrw_theta = " << lrw_theta << "\n"
                                      << "lew_theta = " << lew_theta << "\n"
                                      << "rew_theta = " << rew_theta << "\n"
                                      << "rw_theta = " << rw_theta << std::endl;
  */

  // get orientation error
  // pow makes the values smaller... 
  double orien_cost = std::max(lrw_theta - max_theta, 0.0) +
                      std::max(lew_theta - max_theta, 0.0) +
                      std::max(rew_theta - max_theta, 0.0) +
                      std::max(rw_theta - max_theta, 0.0); // l1 penalty
  

                      /* // l2 penalty
                      std::pow(std::max(lrw_theta - max_theta, 0.0), 2) +
                      std::pow(std::max(lew_theta - max_theta, 0.0), 2) +
                      std::pow(std::max(rew_theta - max_theta, 0.0), 2) +
                      std::pow(std::max(rw_theta - max_theta, 0.0), 2) ; // use pow() so that jacobian increases as the cost rises up, instead of staying the same...     
                      */

                      /*std::pow( std::max(lrw_theta - max_theta, 0.0) +
                                 std::max(lew_theta - max_theta, 0.0) +
                                 std::max(rew_theta - max_theta, 0.0) +
                                 std::max(rw_theta - max_theta, 0.0), 2);*/

  return orien_cost;

}

double DMPConstraints::compute_scale_cost(Matrix<double, DMPPOINTS_DOF, 1> x)
{
  // get goals and starts
  Matrix<double, 3, 1> lrw_new_goal; lrw_new_goal = x.block(0, 0, 3, 1);
  Matrix<double, 3, 1> lrw_new_start; lrw_new_start = x.block(3, 0, 3, 1);
  Matrix<double, 3, 1> lew_new_goal; lew_new_goal = x.block(6, 0, 3, 1);
  Matrix<double, 3, 1> lew_new_start; lew_new_start = x.block(9, 0, 3, 1);
  Matrix<double, 3, 1> rew_new_goal; rew_new_goal = x.block(12, 0, 3, 1);
  Matrix<double, 3, 1> rew_new_start; rew_new_start = x.block(15, 0, 3, 1);
  Matrix<double, 3, 1> rw_new_goal; rw_new_goal = x.block(18, 0, 3, 1);
  Matrix<double, 3, 1> rw_new_start; rw_new_start = x.block(21, 0, 3, 1);

  // Get new vectors
  Matrix<double, 3, 1> lrw_new_vec; lrw_new_vec = lrw_new_goal - lrw_new_start;
  Matrix<double, 3, 1> lew_new_vec; lew_new_vec = lew_new_goal - lew_new_start;
  Matrix<double, 3, 1> rew_new_vec; rew_new_vec = rew_new_goal - rew_new_start;
  Matrix<double, 3, 1> rw_new_vec; rw_new_vec = rw_new_goal - rw_new_start;

  // Get the lengths of new vectors and ratios
  double lrw_new_len = lrw_new_vec.norm();
  double lew_new_len = lew_new_vec.norm();
  double rew_new_len = rew_new_vec.norm();
  double rw_new_len = rw_new_vec.norm();
  double lrw_ratio = lrw_new_len / this->lrw_len;
  double lew_ratio = lew_new_len / this->lew_len;
  double rew_ratio = rew_new_len / this->rew_len;
  double rw_ratio = rw_new_len / this->rw_len;
  Vector4d ratios;
  ratios << lrw_ratio, lew_ratio, rew_ratio, rw_ratio;
  double margin = ratios.maxCoeff() - ratios.minCoeff();
  /*
  std::cout << "debug: " << std::endl << "lrw_ratio = " << lrw_ratio << "\n"
                                      << "lew_ratio = " << lew_ratio << "\n"
                                      << "rew_ratio = " << rew_ratio << "\n"
                                      << "rw_ratio = " << rw_ratio << std::endl;
  std::cout << "debug: margin = " << margin << std::endl;
  */

  // set error
  // setting margin for max and min ratios could yield cheating behavior, i.e. the trajectories become very tiny for the manipulators to track under small cost
  double max_margin = 0.05; // within 5%
  //double scale_cost = std::pow(std::max(margin-max_margin, 0.0), 2); // add pow() to allow jacobian increase as the cost rises, instead of staying the same gradient
  // compromise: set 1.0+-ratio_bound as the region of expected scale
  double ratio_bound = 0.05; // within +-5%
  double scale_cost = std::max(std::fabs(lrw_ratio - 1.0) - ratio_bound, 0.0) +
                      std::max(std::fabs(lew_ratio - 1.0) - ratio_bound, 0.0) + 
                      std::max(std::fabs(rew_ratio - 1.0) - ratio_bound, 0.0) +
                      std::max(std::fabs(rw_ratio - 1.0) - ratio_bound, 0.0) +
                      std::max(margin-max_margin, 0.0); // l1 penalty

                      /*
                      std::pow(std::max(std::fabs(lrw_ratio - 1.0) - ratio_bound, 0.0), 2) +
                      std::pow(std::max(std::fabs(lew_ratio - 1.0) - ratio_bound, 0.0), 2) + 
                      std::pow(std::max(std::fabs(rew_ratio - 1.0) - ratio_bound, 0.0), 2) +
                      std::pow(std::max(std::fabs(rw_ratio - 1.0) - ratio_bound, 0.0), 2) +
                      std::pow(std::max(margin-max_margin, 0.0), 2); // l2 penalty
                      */

  return scale_cost;

}

/* Change of starts and goals of relative trajectories */
double DMPConstraints::compute_rel_change_cost(Matrix<double, DMPPOINTS_DOF, 1> x)
{
  // get goals and starts
  Matrix<double, 3, 1> lrw_new_goal; lrw_new_goal = x.block(0, 0, 3, 1);
  Matrix<double, 3, 1> lrw_new_start; lrw_new_start = x.block(3, 0, 3, 1);
  Matrix<double, 3, 1> lew_new_goal; lew_new_goal = x.block(6, 0, 3, 1);
  Matrix<double, 3, 1> lew_new_start; lew_new_start = x.block(9, 0, 3, 1);
  Matrix<double, 3, 1> rew_new_goal; rew_new_goal = x.block(12, 0, 3, 1);
  Matrix<double, 3, 1> rew_new_start; rew_new_start = x.block(15, 0, 3, 1);

  // Get the lengths of new vectors and ratios
  double lrw_start_change = (lrw_new_start.transpose() - this->lrw_start).norm();
  double lrw_goal_change = (lrw_new_goal.transpose() - this->lrw_goal).norm();
  double lew_start_change = (lew_new_start.transpose() - this->lew_start).norm();
  double lew_goal_change = (lew_new_goal.transpose() - this->lew_goal).norm();
  double rew_start_change = (rew_new_start.transpose() - this->rew_start).norm();
  double rew_goal_change = (rew_new_goal.transpose() - this->rew_goal).norm();

  // set error
  double lrw_margin = 0.0;//0.02;//0.01; // within 1 cm (a bad result displays 0.05 offset, calculated in MATLAB) // relax a few
  double ew_margin = 0.05; //0.02; // relative change between elbow and wrist, allow for robot with different configuration to track
  double rel_change_cost = std::max(lrw_start_change - lrw_margin, 0.0) +
                           std::max(lrw_goal_change - lrw_margin, 0.0) +
                           std::max(lew_start_change - ew_margin, 0.0) +
                           std::max(lew_goal_change - ew_margin, 0.0) +
                           std::max(rew_start_change - ew_margin, 0.0) +
                           std::max(rew_goal_change - ew_margin, 0.0);


  return rel_change_cost;

}




void DMPConstraints::linearizeOplus()
{
  // Prep
  double dmp_eps = 0.01;//0.005;//0.02;

  // Get new DMP starts and goals
  const DMPStartsGoalsVertex *v = dynamic_cast<const DMPStartsGoalsVertex*>(_vertices[0]); // the last vertex connected
  Matrix<double, DMPPOINTS_DOF, 1> x = v->estimate();
  Matrix<double, DMPPOINTS_DOF, 1> delta_x = Matrix<double, DMPPOINTS_DOF, 1>::Zero();
  double e_plus, e_minus;
  for (unsigned int d = 0; d < DMPPOINTS_DOF; d++)
  {
    // set delta_x
    delta_x[d] = dmp_eps;

    // evaluate jacobians for orientation cost
    e_plus = this->compute_orien_cost(x+delta_x);
    e_minus = this->compute_orien_cost(x-delta_x);
    _jacobianOplusXi(0, d) = K_DMPSTARTSGOALS * (e_plus - e_minus) / (2*dmp_eps);
    this->orien_jacobians_for_dmp(0, d) = K_DMPSTARTSGOALS * (e_plus - e_minus) / (2*dmp_eps);
    // if (this->orien_jacobians_for_dmp(0, d) > 1 || this->orien_jacobians_for_dmp(0, d) < -1)
    // {
    //   std::cout << "gradient value is " << this->orien_jacobians_for_dmp(0, d) << "." << std::endl;
    //   std::cout << "breakpoint for debug" << std::endl;
    // }

    // evaluate jacobians for scale cost
    e_plus = this->compute_scale_cost(x+delta_x);
    e_minus = this->compute_scale_cost(x-delta_x);
    _jacobianOplusXi(0, d) += K_DMPSCALEMARGIN * (e_plus - e_minus) / (2*dmp_eps); // add up the influence
    this->scale_jacobians_for_dmp(0, d) = K_DMPSCALEMARGIN * (e_plus - e_minus) / (2*dmp_eps);
    // if (this->scale_jacobians_for_dmp(0, d) > 1 || this->scale_jacobians_for_dmp(0, d) < -1)
    // {
    //   std::cout << "gradient value is " << this->scale_jacobians_for_dmp(0, d) << "." << std::endl;
    //   std::cout << "breakpoint for debug" << std::endl;
    // }

    // evaluate jacobians for rel change cost
    e_plus = this->compute_rel_change_cost(x+delta_x);
    e_minus = this->compute_rel_change_cost(x-delta_x);
    _jacobianOplusXi(0, d) += K_DMPRELCHANGE * (e_plus - e_minus) / (2*dmp_eps); // add up the influence
    this->rel_jacobians_for_dmp(0, d) = K_DMPRELCHANGE * (e_plus - e_minus) / (2*dmp_eps);

    // reset delta_x
    delta_x[d] = 0.0;

  }

}



/* Define constraint for collision and position limit */
class MyUnaryConstraints : public BaseUnaryEdge<1, my_constraint_struct, DualArmDualHandVertex>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MyUnaryConstraints(KDL::ChainFkSolverPos_recursive &_left_fk_solver, KDL::ChainFkSolverPos_recursive &_right_fk_solver, boost::shared_ptr<DualArmDualHandCollision> &_dual_arm_dual_hand_collision_ptr) : left_fk_solver(_left_fk_solver), right_fk_solver(_right_fk_solver), dual_arm_dual_hand_collision_ptr(_dual_arm_dual_hand_collision_ptr){};

    // functions to compute costs
    double compute_collision_cost(Matrix<double, JOINT_DOF, 1> q_whole, boost::shared_ptr<DualArmDualHandCollision> &dual_arm_dual_hand_collision_ptr);
    
    double compute_dual_hands_collision_cost(Matrix<double, JOINT_DOF, 1> q_whole, boost::shared_ptr<DualArmDualHandCollision> &dual_arm_dual_hand_collision_ptr);
    
    // double compute_distance_cost(Matrix<double, JOINT_DOF, 1> q_whole, boost::shared_ptr<DualArmDualHandCollision> &dual_arm_dual_hand_collision_ptr);
    double compute_pos_limit_cost(Matrix<double, JOINT_DOF, 1> q_whole, my_constraint_struct &fdata);

    void computeError();

    // functions for recording costs
    double return_pos_limit_cost();
    double return_col_cost();

    // display for debug
    void output_distance_result();

    // for computing collision updates
    Eigen::MatrixXd compute_col_q_update(Eigen::MatrixXd jacobian, Eigen::Vector3d dx, double speed);


    // Read and write, leave blank
    virtual bool read( std::istream& in ) {return true;}
    virtual bool write( std::ostream& out ) const {return true;}

    // Re-implement linearizeOplus for jacobians calculation
    virtual void linearizeOplus();

    Matrix<double, JOINT_DOF, 1> col_jacobians = Matrix<double, JOINT_DOF, 1>::Zero();
    Matrix<double, JOINT_DOF, 1> pos_limit_jacobians;
    Matrix<double, JOINT_DOF, 1> whole_jacobians;

    // safety setting, note that d_check must be stricly greater than d_safe !!!
    // use different margins of safety for arms and hands
    // here the safety margin should be set according to actual condition: set a collision-free state and check minimum distance for dual_arms and dual_hands using collision_checking_yumi.cpp
    double d_arm_check = 0.003; // 0.02;
    double d_arm_safe = 0.001; //0.0; //0.002; //0.01 is actually too large, for some links are very close to each other, e.g. _5_l and _7_l
    double d_hand_check = 0.002; // threshold, pairs with distance above which would not be checked further, so as to reduce number of queries
    double d_hand_safe = 0.001;//0.0; //0.001; // margin of safety
  

  public:
    // FK solvers
    KDL::ChainFkSolverPos_recursive &left_fk_solver;
    KDL::ChainFkSolverPos_recursive &right_fk_solver;
    // Collision checker
    boost::shared_ptr<DualArmDualHandCollision> &dual_arm_dual_hand_collision_ptr;

};

/* This function outputs distance computation results, for checking on collision. */
void MyUnaryConstraints::output_distance_result()
{
  std::cout << "Distance result: " << std::endl;
  std::cout << "Collision between " << dual_arm_dual_hand_collision_ptr->link_names[0] << " and " 
                                    << dual_arm_dual_hand_collision_ptr->link_names[1] << ", with min_dist = "
                                    << dual_arm_dual_hand_collision_ptr->min_distance << "." << std::endl;

}


/* This function computes q velocity from Cartesian velocity, through the use of robot jacobian. 
 * From robotics, we have [dx, dy, dz, dp, dq, dw] = J * dq, i.e. [d_pos, d_ori] = J * dq.
 * Here we computes dq from the given d_pos = [dx, dy, dz] and J, the jacobian matrix.
 * Note that jacobian is of the size 6 x N, with N the number of joints.
 * Output d_q has the size of N x 1.
 */
Eigen::MatrixXd MyUnaryConstraints::compute_col_q_update(Eigen::MatrixXd jacobian, Eigen::Vector3d d_pos, double speed)
{
  // dx = J * dq, given dx and J, solve for dq ==> J'*(J*J')^-1 * dx = dq. This is solvable only when rank(J) = rank(J, dx).
  // So when rank(J) != 6, there is rank(J,dx) > rank(J), and the result would either be incorrect or NaN.
  // Constrain the rows of J to 3, i.e. processing only position data, this way when row rank of J is >= 3, there is rank(J, dx) >= 3, and therefore rank(J, dx) = 3 = rank(J).

  // prep
  Eigen::Matrix<double, 6, 1> d_pos_ori = Eigen::Matrix<double, 6, 1>::Zero();
  d_pos_ori.block(0, 0, 3, 1) = d_pos;
  unsigned int num_cols = jacobian.cols();
  Eigen::MatrixXd d_q;


  // pre-processing J and dx to cope with rank issue. If rank is smaller than 6, J rows will be reduced to the same number, and so does d_pos_ori.
  // *** Pros and Cons: Although dpos calculated by J*dq would be consistent with d_pos_ori, J*dq might produce huge deviation in dori part !!!
  unsigned int rank = jacobian.colPivHouseholderQr().rank(); 
  Eigen::MatrixXd d_x = d_pos_ori;//d_pos_ori.block(0, 0, rank, 1);
  Eigen::MatrixXd J = jacobian;//jacobian.block(0, 0, rank, num_cols);
  // std::cout << "debug: rank(J) = " << rank << std::endl;
  // std::cout << "debug: original J = " << jacobian << std::endl;
  // std::cout << "debug: processed J = " << J << std::endl;
  // std::cout << "debug: original dx = " << d_pos_ori.transpose() << std::endl;
  // std::cout << "debug: processed dx = " << d_x.transpose() << std::endl;


  // solve for dq
  std::chrono::steady_clock::time_point t00 = std::chrono::steady_clock::now();
  // 1 - direct calculation
  // d_q = J.transpose() * (J * J.transpose()).inverse() * d_x;
  // 2 - use SVD
  JacobiSVD<MatrixXd> svd(J, ComputeThinU | ComputeThinV); // svd.singularValues()/.matrixU()/.matrixV()
  d_q = svd.solve(d_x);
  std::chrono::steady_clock::time_point t11 = std::chrono::steady_clock::now();
  std::chrono::duration<double> t_0011 = std::chrono::duration_cast<std::chrono::duration<double>>(t11 - t00);
  // std::cout << "SVD solution took " << t_0011.count() << " s." << std::endl;
  // debug:
  // std::cout << "debug: d_q = " << d_q.transpose() << std::endl;
  // std::cout << "debug: original J * dq = " << (jacobian * d_q).transpose() << std::endl;
  // std::cout << "debug: processed J * dq = " << (J * d_q).transpose() << std::endl;
  // std::cout << "debug: d_x = " << d_x.transpose() << std::endl;
  // std::cout << "debug: d_pos_ori = " << d_pos_ori.transpose() << std::endl;
  // std::cout << "size of d_q is: " << d_q.rows() << " x " << d_q.cols() << std::endl;


  // post-processing on dq, to speed up and apply weighting
  d_q = K_COL * speed * d_q;

  
  // debug on NaN issue
  /*
  bool debug = isnan(d_q.norm());
  if (debug)
  {
    std::cout << "NaN error: d_q = " << d_q.transpose() << std::endl;
    std::cout << "J = \n" << jacobian << std::endl << std::endl;
    std::cout << "J*JT = \n" << (jacobian * jacobian.transpose()) << std::endl << std::endl;
    std::cout << "(J*JT)^-1 = \n" << (jacobian * jacobian.transpose()).inverse() << std::endl << std::endl;
    std::cout << "d_pos_ori = " << d_pos_ori.transpose() << std::endl;
    std::cout << "End" << std::endl;

    // Qr can't solve ...
    std::chrono::steady_clock::time_point t00 = std::chrono::steady_clock::now();
    // MatrixXd dq = jacobian.householderQr().solve(d_pos_ori);
    Eigen::MatrixXd dq = jacobian.colPivHouseholderQr().solve(d_pos_ori); 
    std::chrono::steady_clock::time_point t11 = std::chrono::steady_clock::now();
    std::chrono::duration<double> t_0011 = std::chrono::duration_cast<std::chrono::duration<double>>(t11 - t00);
    std::cout << "QR decomposition spent " << t_0011.count() << " s." << std::endl;
    // assert(d_pos_ori.isApprox(jacobian * d_q));
    std::cout << "Results of QR decomposition: dq = " << dq.transpose() << std::endl;
    std::cout << "J * dq = " << (jacobian * dq).transpose() << std::endl;
    std::cout << "d_pos_ori = " << d_pos_ori.transpose() << std::endl;
    std::cout << "End" << std::endl;
  }
  */


  return d_q;

}



void MyUnaryConstraints::linearizeOplus()
{
  if (K_COL == 0.0)
  {
    _jacobianOplusXi = Matrix<double, 1, JOINT_DOF>::Zero();
    return;
  }

  // epsilons
  double col_eps = 5.0 * M_PI / 180.0; //0.05; // in radius, approximately 3 deg
  double simple_update = 0.2;//0.1;//0.1; // update step for (4,0) or (0,4), i.e. only one end is in colliding state, close to boundary
  double pos_limit_eps = 0.02;

  // Get current joint angles
  DualArmDualHandVertex *v = dynamic_cast<DualArmDualHandVertex*>(_vertices[0]);
  Matrix<double, JOINT_DOF, 1> x = v->estimate(); // return the current estimate of the vertex
  Matrix<double, JOINT_DOF, 1> delta_x = Matrix<double, JOINT_DOF, 1>::Zero();


  // Real-time collision checking strategy using robot jacobian and normal
  // double e_cur = compute_collision_cost(x, dual_arm_dual_hand_collision_ptr);
  // convert from matrix to std::vector
  std::vector<double> xx(JOINT_DOF);
  for (unsigned int i = 0; i < JOINT_DOF; ++i)
    xx[i] = x[i];

  // Collision checking (or distance computation), check out the DualArmDualHandCollision class
  double e_cur;
  double speed = 1.0;//1000.0;//10.0;//1.0;//100.0;//1.0;//10.0;//50.0;//100.0;//10.0;//1.0;//0.1;//1.0; // speed up collision updates, since .normal is a normalized vector, we may need this term to modify the speed (or step)  
  double hand_speed = 400.0;//200.0;//100.0;// 1.0; // for collision between links belonging to the same hand !!!
  double min_distance;
  // check arms first, if ok, then hands. i.e. ensure arms safety before checking hands
  std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
  min_distance = dual_arm_dual_hand_collision_ptr->compute_self_distance_test(xx, "dual_arms", d_arm_check); 
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  std::chrono::duration<double> t_spent = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  total_col += t_spent.count();
  count_col++;
  // if *dual_arms* group does not satisfy safety requirement, the lastest distance result would be used directly
  // if not, *dual_hands* group would be checked and overwriting the last distance result
  if (min_distance > d_arm_safe) // arm safe
  {
    // check hands 
    t0 = std::chrono::steady_clock::now();
    min_distance = dual_arm_dual_hand_collision_ptr->compute_self_distance_test(xx, "dual_hands", d_hand_check); 
    t1 = std::chrono::steady_clock::now();
    t_spent = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
    total_col += t_spent.count();
    count_col++;
    e_cur = std::max(d_hand_safe - min_distance, 0.0);
  }
  else
  {
    e_cur = std::max(d_arm_safe - min_distance, 0.0); // <0 means colliding, >0 is ok
  }

  // determine updates when in collision state or safety is not met
  if (e_cur > 0.0) // in collision state or within safety of margin
  {
    // std::cout << "debug: e_cur = " << e_cur << std::endl;

    // get group belonging information: 0 - left_arm, 1 - right_arm, 2 - left_hand, 3 - right_hand, -1 - others
    int group_id_1 = dual_arm_dual_hand_collision_ptr->check_link_belonging(dual_arm_dual_hand_collision_ptr->link_names[0]);
    int group_id_2 = dual_arm_dual_hand_collision_ptr->check_link_belonging(dual_arm_dual_hand_collision_ptr->link_names[1]);
    
    // prep
    std::string link_name_1 = dual_arm_dual_hand_collision_ptr->link_names[0];
    std::string link_name_2 = dual_arm_dual_hand_collision_ptr->link_names[1];

    // std::cout << "debug: Possible collision between " << link_name_1 << " and " << link_name_2 << std::endl;
    // std::cout << "debug: minimum distance is: " << dual_arm_dual_hand_collision_ptr->min_distance << std::endl;

    // calculate global location of nearest/colliding links (reference position is independent of base frame, so don't worry)
    Eigen::Vector3d link_pos_1 = dual_arm_dual_hand_collision_ptr->get_global_link_transform(link_name_1);
    Eigen::Vector3d link_pos_2 = dual_arm_dual_hand_collision_ptr->get_global_link_transform(link_name_2);
    Eigen::Vector3d ref_point_pos_1 = dual_arm_dual_hand_collision_ptr->nearest_points[0] - link_pos_1;
    Eigen::Vector3d ref_point_pos_2 = dual_arm_dual_hand_collision_ptr->nearest_points[1] - link_pos_2;    

    // Inspect the circumstance
    if ((group_id_1 == 0 && group_id_2 == 0) ||
        (group_id_1 == 0 && group_id_2 == 1) || 
        (group_id_1 == 1 && group_id_2 == 0) || 
        (group_id_1 == 1 && group_id_2 == 1) ) // collision between arm and arm (could be the same), update only q_arm
    {
      // debug display
      // std::cout << "debug: possible collision between arm and arm... update only q_arm" << std::endl;
      
      // determine left or right
      bool left_or_right_1 = (group_id_1 == 0); 
      bool left_or_right_2 = (group_id_2 == 0);

      // compute jacobians (for arms) - return is 6 x 7, 6 for instantaneous pos/ori ([dx, dy, dz, dp, dq, dw])
      Eigen::MatrixXd jacobian_1 = dual_arm_dual_hand_collision_ptr->get_robot_arm_jacobian(link_name_1, 
                                                                                            ref_point_pos_1, 
                                                                                            left_or_right_1);
      Eigen::MatrixXd jacobian_2 = dual_arm_dual_hand_collision_ptr->get_robot_arm_jacobian(link_name_2, 
                                                                                            ref_point_pos_2, 
                                                                                            left_or_right_2);  
      
      // note that .normal is the normalized vector pointing from link_names[0] to link_names[1]; output is column vector with size N x 1
      Eigen::MatrixXd dq_col_update_1 = this->compute_col_q_update(jacobian_1, - dual_arm_dual_hand_collision_ptr->normal, speed);
      Eigen::MatrixXd dq_col_update_2 = this->compute_col_q_update(jacobian_2, dual_arm_dual_hand_collision_ptr->normal, speed);

      // std::cout << "debug: \n" << "dq_col_update_1 = " << dq_col_update_1.transpose() 
      //                          << ", dq_col_update_2 = " << dq_col_update_2.transpose() << std::endl;

      // assign jacobians for collision cost
      // init
      _jacobianOplusXi = Matrix<double, 1, JOINT_DOF>::Zero();
      // first link
      if (left_or_right_1)
      {
        _jacobianOplusXi.block(0, 0, 1, 7) += dq_col_update_1.transpose();
      }
      else
      {
        _jacobianOplusXi.block(0, 7, 1, 7) += dq_col_update_1.transpose();
      }
      // std::cout << "1 - jacobian = " << _jacobianOplusXi << std::endl;
      // for debug
      bool debug = isnan(_jacobianOplusXi.norm());
      if (debug)
      {
        std::cout << "debug: jacobian = " << _jacobianOplusXi << std::endl;
        double a;
      }

      // second link
      if (left_or_right_2)
      {
        _jacobianOplusXi.block(0, 0, 1, 7) += dq_col_update_2.transpose();
      }
      else
      {
        _jacobianOplusXi.block(0, 7, 1, 7) += dq_col_update_2.transpose();
      }
      // std::cout << "2 - jacobian = " << _jacobianOplusXi << std::endl;

      // for debug
      debug = isnan(_jacobianOplusXi.norm());
      if (debug)
      {
        std::cout << "debug: jacobian = " << _jacobianOplusXi << std::endl;
        double a;
      }

    }
    else if ((group_id_1 == 2 && group_id_2 == 2) || (group_id_1 == 3 && group_id_2 == 3) ) // collision between hand and hand (the same one), update only q_finger
    {
      // debug display
      // std::cout << "debug: possible collision between fingers of the same hand... update only q_finger" << std::endl;

      // prep
      bool left_or_right = (group_id_1 == 2);
      int finger_id_1 = dual_arm_dual_hand_collision_ptr->check_finger_belonging(link_name_1, left_or_right);
      int finger_id_2 = dual_arm_dual_hand_collision_ptr->check_finger_belonging(link_name_2, left_or_right);

      // compute robot jacobians (for fingers), return is 6 x N, N could be 4(thumb) or 2(others); and compute updates
      /*
      Eigen::MatrixXd jacobian_1;
      Eigen::MatrixXd jacobian_2;
      Eigen::MatrixXd dq_col_update_1;
      Eigen::MatrixXd dq_col_update_2;
      if (finger_id_1 != -1) // if not belonging to palm groups!!!
      {
        jacobian_1 = dual_arm_dual_hand_collision_ptr->get_robot_hand_jacobian(link_name_1, 
                                                                               ref_point_pos_1, 
                                                                               finger_id_1,
                                                                               left_or_right);
        dq_col_update_1 = this->compute_col_q_update(jacobian_1, - dual_arm_dual_hand_collision_ptr->normal, hand_speed*speed);                                                                               
        std::cout << "debug: \n" << "dq_col_update_1 = " << dq_col_update_1.transpose() << std::endl;
      }
      if (finger_id_2 != -1) // if not belonging to palm groups!!!
      {
        jacobian_2 = dual_arm_dual_hand_collision_ptr->get_robot_hand_jacobian(link_name_2, 
                                                                               ref_point_pos_2, 
                                                                               finger_id_2,
                                                                               left_or_right);
        dq_col_update_2 = this->compute_col_q_update(jacobian_2, dual_arm_dual_hand_collision_ptr->normal, hand_speed*speed);                                                                               
        std::cout << "debug: \n" << "dq_col_update_2 = " << dq_col_update_2.transpose() << std::endl;
      }
      */

      // assign jacobians for collision cost
      // init
      _jacobianOplusXi = Matrix<double, 1, JOINT_DOF>::Zero();
      Matrix<double, 1, JOINT_DOF> jacobian_way_1 = Matrix<double, 1, JOINT_DOF>::Zero();
      Matrix<double, 1, JOINT_DOF> jacobian_way_2 = Matrix<double, 1, JOINT_DOF>::Zero();
      unsigned int d = left_or_right ? 0 : 12;
      double e_up, e_down;
      // first link
      switch (finger_id_1)
      {
        case 0: // thumb
          for (unsigned int s = 0; s < 4; s++)
          {
            // set
            delta_x[22+d+s] = col_eps;
            // compute
            e_up = compute_dual_hands_collision_cost(x+delta_x, dual_arm_dual_hand_collision_ptr);
            e_down = compute_dual_hands_collision_cost(x-delta_x, dual_arm_dual_hand_collision_ptr);
            _jacobianOplusXi(0, 22+d+s) += hand_speed * (e_up - e_down) / (2*col_eps); //
            jacobian_way_1(0, 22+d+s) += K_COL * ((e_up > e_down) ? simple_update : -simple_update);
            jacobian_way_2(0, 22+d+s) += hand_speed * (e_up - e_down) / (2*col_eps); //
          // _jacobianOplusXi.block(0, 22 + d, 1, 4) += dq_col_update_1.transpose();
            // reset
            delta_x[22+d+s] = 0.0;
          }
          break;
        case 1: //index
          for (unsigned int s = 0; s < 2; s++)
          {
            // set
            delta_x[14+d+s] = col_eps;
            // compute
            e_up = compute_dual_hands_collision_cost(x+delta_x, dual_arm_dual_hand_collision_ptr);
            e_down = compute_dual_hands_collision_cost(x-delta_x, dual_arm_dual_hand_collision_ptr);
            _jacobianOplusXi(0, 14+d+s) += hand_speed * (e_up - e_down) / (2*col_eps); //
            jacobian_way_1(0, 14+d+s) += K_COL * ((e_up > e_down) ? simple_update : -simple_update);
            jacobian_way_2(0, 14+d+s) += hand_speed * (e_up - e_down) / (2*col_eps); //
          // _jacobianOplusXi.block(0, 14 + d, 1, 2) += dq_col_update_1.transpose();
            // reset
            delta_x[14+d+s] = 0.0;
          }
          break;
        case 2: //middle
          for (unsigned int s = 0; s < 2; s++)
          {
            // set
            delta_x[16+d+s] = col_eps;
            // compute
            e_up = compute_dual_hands_collision_cost(x+delta_x, dual_arm_dual_hand_collision_ptr);
            e_down = compute_dual_hands_collision_cost(x-delta_x, dual_arm_dual_hand_collision_ptr);
            _jacobianOplusXi(0, 16+d+s) += hand_speed * (e_up - e_down) / (2*col_eps); //
            jacobian_way_1(0, 16+d+s) += K_COL * ((e_up > e_down) ? simple_update : -simple_update);
            jacobian_way_2(0, 16+d+s) += hand_speed * (e_up - e_down) / (2*col_eps); //
          // _jacobianOplusXi.block(0, 16 + d, 1, 2) += dq_col_update_1.transpose();
            // reset
            delta_x[16+d+s] = 0.0;
          }
          break;
        case 3: // ring
          for (unsigned int s = 0; s < 2; s++)
          {
            // set
            delta_x[18+d+s] = col_eps;
            // compute
            e_up = compute_dual_hands_collision_cost(x+delta_x, dual_arm_dual_hand_collision_ptr);
            e_down = compute_dual_hands_collision_cost(x-delta_x, dual_arm_dual_hand_collision_ptr);
            _jacobianOplusXi(0, 18+d+s) += hand_speed * (e_up - e_down) / (2*col_eps); //
            jacobian_way_1(0, 18+d+s) += K_COL * ((e_up > e_down) ? simple_update : -simple_update);
            jacobian_way_2(0, 18+d+s) += hand_speed * (e_up - e_down) / (2*col_eps); //
          // _jacobianOplusXi.block(0, 18 + d, 1, 2) += dq_col_update_1.transpose();
            // reset
            delta_x[18+d+s] = 0.0;
          }
          break;
        case 4: // little
          for (unsigned int s = 0; s < 2; s++)
          {
            // set
            delta_x[20+d+s] = col_eps;
            // compute
            e_up = compute_dual_hands_collision_cost(x+delta_x, dual_arm_dual_hand_collision_ptr);
            e_down = compute_dual_hands_collision_cost(x-delta_x, dual_arm_dual_hand_collision_ptr);
            _jacobianOplusXi(0, 20+d+s) += hand_speed * (e_up - e_down) / (2*col_eps); //
            jacobian_way_1(0, 20+d+s) += K_COL * ((e_up > e_down) ? simple_update : -simple_update);
            jacobian_way_2(0, 20+d+s) += hand_speed * (e_up - e_down) / (2*col_eps); //
          // _jacobianOplusXi.block(0, 20 + d, 1, 2) += dq_col_update_1.transpose();
            // reset
            delta_x[20+d+s] = 0.0;
          }
          break;
        case -1: // palm, do nothing
          break;
        default:
          std::cerr << "error: Finger ID doesn't lie in {-1,0,1,2,3,4}!!!" << std::endl;
          exit(-1);
          break;
      }
      // std::cout << "1 - jacobian = " << _jacobianOplusXi << std::endl;
      // std::cout << "1 - jacobian_way_1 = " << jacobian_way_1 << std::endl;
      // std::cout << "1 - jacobian_way_2 = " << jacobian_way_2 << std::endl;

      // for debug
      bool debug = isnan(_jacobianOplusXi.norm());
      if (debug)
      {
        std::cout << "debug: jacobian = " << _jacobianOplusXi << std::endl;
        double a;
      }

      // second link
      switch (finger_id_2)
      {
        case 0: // thumb
          for (unsigned int s = 0; s < 4; s++)
          {
            // set
            delta_x[22+d+s] = col_eps;
            // compute
            e_up = compute_dual_hands_collision_cost(x+delta_x, dual_arm_dual_hand_collision_ptr);
            e_down = compute_dual_hands_collision_cost(x-delta_x, dual_arm_dual_hand_collision_ptr);
            _jacobianOplusXi(0, 22+d+s) += hand_speed * (e_up - e_down) / (2*col_eps); //
            jacobian_way_1(0, 22+d+s) += K_COL * ((e_up > e_down) ? simple_update : -simple_update);
            jacobian_way_2(0, 22+d+s) += hand_speed * (e_up - e_down) / (2*col_eps); //
          // _jacobianOplusXi.block(0, 22 + d, 1, 4) += dq_col_update_2.transpose();
            // reset
            delta_x[22+d+s] = 0.0;
          }
          break;
        case 1: //index
          for (unsigned int s = 0; s < 2; s++)
          {
            // set
            delta_x[14+d+s] = col_eps;
            // compute
            e_up = compute_dual_hands_collision_cost(x+delta_x, dual_arm_dual_hand_collision_ptr);
            e_down = compute_dual_hands_collision_cost(x-delta_x, dual_arm_dual_hand_collision_ptr);
            _jacobianOplusXi(0, 14+d+s) += hand_speed * (e_up - e_down) / (2*col_eps); //
            jacobian_way_1(0, 14+d+s) += K_COL * ((e_up > e_down) ? simple_update : -simple_update);
            jacobian_way_2(0, 14+d+s) += hand_speed * (e_up - e_down) / (2*col_eps); //
          // _jacobianOplusXi.block(0, 14 + d, 1, 2) += dq_col_update_2.transpose();
            // reset
            delta_x[14+d+s] = 0.0;
          }
          break;
        case 2: //middle
          for (unsigned int s = 0; s < 2; s++)
          {
            // set
            delta_x[16+d+s] = col_eps;
            // compute
            e_up = compute_dual_hands_collision_cost(x+delta_x, dual_arm_dual_hand_collision_ptr);
            e_down = compute_dual_hands_collision_cost(x-delta_x, dual_arm_dual_hand_collision_ptr);
            _jacobianOplusXi(0, 16+d+s) += hand_speed * (e_up - e_down) / (2*col_eps); //
            jacobian_way_1(0, 16+d+s) += K_COL * ((e_up > e_down) ? simple_update : -simple_update);
            jacobian_way_2(0, 16+d+s) += hand_speed * (e_up - e_down) / (2*col_eps); //
          // _jacobianOplusXi.block(0, 16 + d, 1, 2) += dq_col_update_2.transpose();
            // reset
            delta_x[16+d+s] = 0.0;
          }
          break;
        case 3: // ring
          for (unsigned int s = 0; s < 2; s++)
          {
            // set
            delta_x[18+d+s] = col_eps;
            // compute
            e_up = compute_dual_hands_collision_cost(x+delta_x, dual_arm_dual_hand_collision_ptr);
            e_down = compute_dual_hands_collision_cost(x-delta_x, dual_arm_dual_hand_collision_ptr);
            _jacobianOplusXi(0, 18+d+s) += hand_speed * (e_up - e_down) / (2*col_eps); // 
            jacobian_way_1(0, 18+d+s) += K_COL * ((e_up > e_down) ? simple_update : -simple_update);
            jacobian_way_2(0, 18+d+s) += hand_speed * (e_up - e_down) / (2*col_eps); //
          // _jacobianOplusXi.block(0, 18 + d, 1, 2) += dq_col_update_2.transpose();
            // reset
            delta_x[18+d+s] = 0.0;
          }
          break;
        case 4: // little
          for (unsigned int s = 0; s < 2; s++)
          {
            // set
            delta_x[20+d+s] = col_eps;
            // compute
            e_up = compute_dual_hands_collision_cost(x+delta_x, dual_arm_dual_hand_collision_ptr);
            e_down = compute_dual_hands_collision_cost(x-delta_x, dual_arm_dual_hand_collision_ptr);
            _jacobianOplusXi(0, 20+d+s) +=  hand_speed * (e_up - e_down) / (2*col_eps); //
            jacobian_way_1(0, 20+d+s) += K_COL * ((e_up > e_down) ? simple_update : -simple_update);
            jacobian_way_2(0, 20+d+s) += hand_speed * (e_up - e_down) / (2*col_eps); //
          // _jacobianOplusXi.block(0, 20 + d, 1, 2) += dq_col_update_2.transpose();
            // reset
            delta_x[20+d+s] = 0.0;
          }
          break;
        case -1: // palm, do nothing
          break;
        default:
          std::cerr << "error: Finger ID doesn't lie in {-1,0,1,2,3,4}!!!" << std::endl;
          exit(-1);
          break;
      }

      // std::cout << "2 - jacobian = " << _jacobianOplusXi << std::endl;
      // std::cout << "2 - jacobian_way_1 = " << jacobian_way_1 << std::endl;
      // std::cout << "2 - jacobian_way_2 = " << jacobian_way_2 << std::endl;

       // for debug
      debug = isnan(_jacobianOplusXi.norm());
      if (debug)
      {
        std::cout << "debug: jacobian = " << _jacobianOplusXi << std::endl;
        double a;
      }

    }
    else if (group_id_1 == -1 || group_id_2 == -1) // one of the object doesn't belong to arms or hands, update only one arm+hand
    {

      // std::cout << "debug: possible collision between robot arms/hands and others..." << std::endl;

      // just in case
      if (group_id_1 == -1 && group_id_2 == -1)
      {
        std::cerr << "Error: Something might be wrong, both links in collision do not belong to arms or hands, which should never happen.." << std::endl;
        exit(-1);
      }

      // prep
      int group_id = (group_id_1 == -1) ? group_id_2 : group_id_1;
      std::string link_name = (group_id_1 == -1) ? link_name_2 : link_name_1;
      Eigen::Vector3d ref_point_pos = (group_id_1 == -1) ? ref_point_pos_2 : ref_point_pos_1;
      Eigen::MatrixXd jacobian;
      Eigen::MatrixXd dq_col_update;

      // initialization
      _jacobianOplusXi = Matrix<double, 1, JOINT_DOF>::Zero();

      // decide if the link is arm or finger
      if (group_id == 2 || group_id == 3) // hand (should update for arm+hand group)
      {
        // compute robot jacobians
        bool left_or_right = (group_id == 2);
        int finger_id = dual_arm_dual_hand_collision_ptr->check_finger_belonging(link_name, left_or_right);
        if (finger_id != -1) // if not palm group !!!
        {
          jacobian = dual_arm_dual_hand_collision_ptr->get_robot_arm_hand_jacobian(link_name,
                                                                                  ref_point_pos,
                                                                                  finger_id,
                                                                                  left_or_right);
          // compute updates
          double direction = (group_id_1 == -1) ? 1.0 : -1.0;
          dq_col_update = this->compute_col_q_update(jacobian, direction * dual_arm_dual_hand_collision_ptr->normal, speed);
          // std::cout << "debug: \n" << "dq_col_update = " << dq_col_update.transpose() << std::endl;                                                                                  
        }
        

        // assign updates to col jacobian
        // arm part
        unsigned int d_arm = left_or_right ? 0 : 7;
        _jacobianOplusXi.block(0, 0+d_arm, 1, 7) += dq_col_update.block(0, 0, 7, 1).transpose();
        // hand part
        unsigned int d_hand = left_or_right ? 0 : 12;
        switch (finger_id)
        {
          case 0: // thummb
            _jacobianOplusXi.block(0, 22 + d_hand, 1, 4) += dq_col_update.block(7, 0, 4, 1).transpose();
            break;
          case 1: //index
            _jacobianOplusXi.block(0, 14 + d_hand, 1, 2) += dq_col_update.block(7, 0, 2, 1).transpose();
            break;
          case 2: //middle
            _jacobianOplusXi.block(0, 16 + d_hand, 1, 2) += dq_col_update.block(7, 0, 2, 1).transpose();
            break;
          case 3: // ring
            _jacobianOplusXi.block(0, 18 + d_hand, 1, 2) += dq_col_update.block(7, 0, 2, 1).transpose();
            break;
          case 4: // little
            _jacobianOplusXi.block(0, 20 + d_hand, 1, 2) += dq_col_update.block(7, 0, 2, 1).transpose();
            break;
          case -1: // palm, do nothing
            break;
          default:
            std::cerr << "error: Finger ID doesn't lie in {-1,0,1,2,3,4}!!!" << std::endl;
            exit(-1);
            break;
        }
        // for debug
        bool debug = isnan(_jacobianOplusXi.norm());
        if (debug)
        {
          std::cout << "debug: jacobian = " << _jacobianOplusXi << std::endl;
          double a;
        }

      }
      else // arm (update only for arm group)
      {
        // compute robot jacobians
        bool left_or_right = (group_id == 0);
        jacobian = dual_arm_dual_hand_collision_ptr->get_robot_arm_jacobian(link_name,
                                                                            ref_point_pos,
                                                                            left_or_right);
        
        // compute updates
        double direction = (group_id_1 == -1) ? 1.0 : -1.0;
        dq_col_update = this->compute_col_q_update(jacobian, direction * dual_arm_dual_hand_collision_ptr->normal, speed);
        // std::cout << "debug: \n" << "dq_col_update = " << dq_col_update.transpose() << std::endl;

        // assign updates to col jacobian
        unsigned int d_arm = left_or_right ? 0 : 7;
        _jacobianOplusXi.block(0, 0+d_arm, 1, 7) += dq_col_update.transpose();

        // for debug
        bool debug = isnan(_jacobianOplusXi.norm());
        if (debug)
        {
          std::cout << "debug: jacobian = " << _jacobianOplusXi << std::endl;
          double a;
        }

      }

      // std::cout << "debug: jacobian = " << _jacobianOplusXi << std::endl;
  
    }
    else // all the other conditions, update both q_arm and q_finger ()
    {
      // std::cout << "debug: Possible collision between arm and hand, or left and right hands... update both q_arm and q_finger" << std::endl;

      // prep
      bool left_or_right_1 = ( (group_id_1 == 0) || (group_id_1 == 2) );
      bool left_or_right_2 = ( (group_id_2 == 0) || (group_id_2 == 2) );
      // initialization
      _jacobianOplusXi = Matrix<double, 1, JOINT_DOF>::Zero();

      // process the first link
      if (group_id_1 == 2 || group_id_1 == 3) // hand, should calculate robot jacobian for arm+hand group
      {
        // prep
        int finger_id = dual_arm_dual_hand_collision_ptr->check_finger_belonging(link_name_1, left_or_right_1);

        // compute robot jacobian for arm+hand group and compute updates
        Eigen::MatrixXd jacobian = dual_arm_dual_hand_collision_ptr->get_robot_arm_hand_jacobian(link_name_1,
                                                                                                 ref_point_pos_1,
                                                                                                 finger_id,
                                                                                                 left_or_right_1);
        Eigen::MatrixXd dq_col_update = this->compute_col_q_update(jacobian, - dual_arm_dual_hand_collision_ptr->normal, speed);                                                                                  
        // std::cout << "debug: \n" << "dq_col_update_1 = " << dq_col_update.transpose() << std::endl;


        // assign updates to col jacobian
        // arm part
        unsigned int d_arm = left_or_right_1 ? 0 : 7;
        _jacobianOplusXi.block(0, 0+d_arm, 1, 7) += dq_col_update.block(0, 0, 7, 1).transpose();
        // for debug
        bool debug = isnan(_jacobianOplusXi.norm());
        if (debug)
        {
          std::cout << "debug: jacobian = " << _jacobianOplusXi << std::endl;
          double a;
        }

        // hand part
        unsigned int d_hand = left_or_right_1 ? 0 : 12;
        switch (finger_id)
        {
          case 0: // thummb
            _jacobianOplusXi.block(0, 22 + d_hand, 1, 4) += dq_col_update.block(7, 0, 4, 1).transpose();
            break;
          case 1: //index
            _jacobianOplusXi.block(0, 14 + d_hand, 1, 2) += dq_col_update.block(7, 0, 2, 1).transpose();
            break;
          case 2: //middle
            _jacobianOplusXi.block(0, 16 + d_hand, 1, 2) += dq_col_update.block(7, 0, 2, 1).transpose();
            break;
          case 3: // ring
            _jacobianOplusXi.block(0, 18 + d_hand, 1, 2) += dq_col_update.block(7, 0, 2, 1).transpose();
            break;
          case 4: // little
            _jacobianOplusXi.block(0, 20 + d_hand, 1, 2) += dq_col_update.block(7, 0, 2, 1).transpose();
            break;
          case -1: // palm, zero updates would be provided for finger joints, but the size is in consistent with little finger_group since it's used in get_robot_arm_hand_jacobian()
            _jacobianOplusXi.block(0, 20 + d_hand, 1, 2) += dq_col_update.block(7, 0, 2, 1).transpose();
            break;
          default:
            std::cerr << "error: Finger ID doesn't lie in {-1,0,1,2,3,4}!!!" << std::endl;
            exit(-1);
            break;
        }
        // for debug
        debug = isnan(_jacobianOplusXi.norm());
        if (debug)
        {
          std::cout << "debug: jacobian = " << _jacobianOplusXi << std::endl;
          double a;
        }

      }
      else // 0 or 1, arm, should calculate robot jacobian for arm group
      {

        // compute robot jacobian for arm group
        Eigen::MatrixXd jacobian = dual_arm_dual_hand_collision_ptr->get_robot_arm_jacobian(link_name_1,
                                                                                            ref_point_pos_1,
                                                                                            left_or_right_1);
        
        // compute updates
        Eigen::MatrixXd dq_col_update = this->compute_col_q_update(jacobian, - dual_arm_dual_hand_collision_ptr->normal, speed);
        // std::cout << "debug: \n" << "dq_col_update_1 = " << dq_col_update.transpose() << std::endl;

        // assign updates to col jacobian
        // arm part
        unsigned int d_arm = left_or_right_1 ? 0 : 7;
        _jacobianOplusXi.block(0, 0+d_arm, 1, 7) += dq_col_update.transpose();
        // for debug
        bool debug = isnan(_jacobianOplusXi.norm());
        if (debug)
        {
          std::cout << "debug: jacobian = " << _jacobianOplusXi << std::endl;
          double a;
        }
      }
      // std::cout << "debug: 1 - jacobian = " << _jacobianOplusXi << std::endl;
      
     
      // process the second link
      if (group_id_2 == 2 || group_id_2 == 3) // hand, should calculate robot jacobian for arm+hand group
      {
        // prep
        int finger_id = dual_arm_dual_hand_collision_ptr->check_finger_belonging(link_name_2, left_or_right_2);


        // compute robot jacobian for arm+hand group
        Eigen::MatrixXd jacobian = dual_arm_dual_hand_collision_ptr->get_robot_arm_hand_jacobian(link_name_2,
                                                                                                 ref_point_pos_2,
                                                                                                 finger_id,
                                                                                                 left_or_right_2);

        // compute updates
        Eigen::MatrixXd dq_col_update = this->compute_col_q_update(jacobian, dual_arm_dual_hand_collision_ptr->normal, speed);
       

        // assign updates to col jacobian
        // arm part
        unsigned int d_arm = left_or_right_2 ? 0 : 7;
        _jacobianOplusXi.block(0, 0+d_arm, 1, 7) += dq_col_update.block(0, 0, 7, 1).transpose();
        // for debug
        bool debug = isnan(_jacobianOplusXi.norm());
        if (debug)
        {
          std::cout << "debug: jacobian = " << _jacobianOplusXi << std::endl;
          double a;
        }

        // hand part
        unsigned int d_hand = left_or_right_2 ? 0 : 12;
        switch (finger_id)
        {
          case 0: // thummb
            _jacobianOplusXi.block(0, 22 + d_hand, 1, 4) += dq_col_update.block(7, 0, 4, 1).transpose();
            break;
          case 1: //index
            _jacobianOplusXi.block(0, 14 + d_hand, 1, 2) += dq_col_update.block(7, 0, 2, 1).transpose();
            break;
          case 2: //middle
            _jacobianOplusXi.block(0, 16 + d_hand, 1, 2) += dq_col_update.block(7, 0, 2, 1).transpose();
            break;
          case 3: // ring
            _jacobianOplusXi.block(0, 18 + d_hand, 1, 2) += dq_col_update.block(7, 0, 2, 1).transpose();
            break;
          case 4: // little
            _jacobianOplusXi.block(0, 20 + d_hand, 1, 2) += dq_col_update.block(7, 0, 2, 1).transpose();
            break;
          case -1: // palm, zero updates would be provided for finger joints, but the size is in consistent with little finger_group since it's used in get_robot_arm_hand_jacobian()
            _jacobianOplusXi.block(0, 20 + d_hand, 1, 2) += dq_col_update.block(7, 0, 2, 1).transpose();
            break;
          default:
            std::cerr << "error: Finger ID doesn't lie in {-1,0,1,2,3,4}!!!" << std::endl;
            exit(-1);
            break;
        }
        // for debug
        debug = isnan(_jacobianOplusXi.norm());
        if (debug)
        {
          std::cout << "debug: jacobian = " << _jacobianOplusXi << std::endl;
          double a;
        }

      }
      else // 0 or 1, arm, should calculate robot jacobian for arm group
      {

        // compute robot jacobian for arm group
        Eigen::MatrixXd jacobian = dual_arm_dual_hand_collision_ptr->get_robot_arm_jacobian(link_name_2,
                                                                                            ref_point_pos_2,
                                                                                            left_or_right_2);
        
        // compute updates
        Eigen::MatrixXd dq_col_update = this->compute_col_q_update(jacobian, dual_arm_dual_hand_collision_ptr->normal, speed);
        // std::cout << "debug: \n" << "dq_col_update_2 = " << dq_col_update.transpose() << std::endl;

        // assign updates to col jacobian
        // arm part
        unsigned int d_arm = left_or_right_2 ? 0 : 7;
        _jacobianOplusXi.block(0, 0+d_arm, 1, 7) += dq_col_update.transpose();
        // for debug
        bool debug = isnan(_jacobianOplusXi.norm());
        if (debug)
        {
          std::cout << "debug: jacobian = " << _jacobianOplusXi << std::endl;
          double a;
        }

      }
      // std::cout << "debug: 2 - jacobian = " << _jacobianOplusXi << std::endl;

    } // END of calculating collision jacobian for optimization
  }
  else // in collision-free state and outside of safety margin (meaning safe now)
  {
    _jacobianOplusXi = Matrix<double, 1, JOINT_DOF>::Zero(); // assign no updates for avoiding collision
  } // END of calculating collision jacobian for optimization

  // debug:
  // std::cout << "debug: col jacobians = " << _jacobianOplusXi << std::endl;

  // record col jacobians
  col_jacobians = _jacobianOplusXi.transpose();

  
  // iterate to calculate pos_limit jacobians
  double e_plus, e_minus;
  for (unsigned int d = 0; d < JOINT_DOF; d++)
  {
    // 1 - Collision - obsolete
    /*
    delta_x[d] = col_eps;
    // std::chrono::steady_clock::time_point t00 = std::chrono::steady_clock::now();
  
    e_plus = compute_collision_cost(x+delta_x, dual_arm_dual_hand_collision_ptr);
    e_minus = compute_collision_cost(x-delta_x, dual_arm_dual_hand_collision_ptr);
    
    // std::chrono::steady_clock::time_point t11 = std::chrono::steady_clock::now();
    // std::chrono::duration<double> t_0011 = std::chrono::duration_cast<std::chrono::duration<double>>(t11 - t00);
    // std::cout << "spent " << t_0011.count() << " s." << std::endl;

    _jacobianOplusXi(0, d) = K_COL * (e_plus - e_minus) / (2*col_eps);

    // record
    col_jacobians[d] = _jacobianOplusXi(0, d);
    */

    // 2 - Position Limit
    delta_x[d] = pos_limit_eps;
    e_plus = compute_pos_limit_cost(x+delta_x, _measurement);
    e_minus = compute_pos_limit_cost(x-delta_x, _measurement);
    _jacobianOplusXi(0, d) += K_POS_LIMIT * (e_plus - e_minus) / (2*pos_limit_eps);
    pos_limit_jacobians[d] = K_POS_LIMIT * (e_plus - e_minus) / (2*pos_limit_eps);

    whole_jacobians[d] = _jacobianOplusXi(0, d);

    // Reset
    delta_x[d] = 0.0;
  }


  // debug:
  /*
  std::cout << "debug: norms of col_jacobian = " << col_jacobians.norm() << std::endl;
  std::cout << "debug: norms of whole_jacobian = " << whole_jacobians.norm() << std::endl;
  std::cout << "debug: col_jacobians = " << col_jacobians.transpose() << std::endl;
  std::cout << "debug: whole_jacobians = " << whole_jacobians.transpose() << std::endl;

  bool result1 = isnan(col_jacobians.norm());
  bool result2 = isnan(whole_jacobians.norm());

  if (result1)
  {
    std::cout << "debug: col_jacobians contains NaN!!!" << std::endl;
    double a;
  }

  if (result2)
  {
    std::cout << "debug: whole_jacobians contains NaN!!!" << std::endl;
    double a;
  }
  // std::cout << "debug: unary jacobians = " <<  _jacobianOplusXi << std::endl;
  //double debug;
  */

}

double MyUnaryConstraints::compute_dual_hands_collision_cost(Matrix<double, JOINT_DOF, 1> q_whole, boost::shared_ptr<DualArmDualHandCollision> &dual_arm_dual_hand_collision_ptr)
{
  // convert from matrix to std::vector
  std::vector<double> x(JOINT_DOF);
  for (unsigned int i = 0; i < JOINT_DOF; ++i)
    x[i] = q_whole[i];
  // Collision checking (or distance computation), check out the DualArmDualHandCollision class
  double cost;
  double min_distance;
  // check arms first, if ok, then hands. i.e. ensure arms safety before checking hands
  // std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
  // min_distance = dual_arm_dual_hand_collision_ptr->compute_self_distance_test(x, "dual_arms", d_arm_check); 
  // std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  // std::chrono::duration<double> t_spent = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  // total_col += t_spent.count();
  // count_col++;
  // std::cout << "debug: time spent on computing minimum distance = " << t_spent.count() << std::endl;
  // check_world_collision, check_full_collision, compute_self_distance, compute_world_distance
  // if (min_distance > d_arm_safe) // arm safe
  // {
    // check hands 
   std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    min_distance = dual_arm_dual_hand_collision_ptr->compute_self_distance_test(x, "dual_hands", d_hand_check); 
   std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
   std::chrono::duration<double> t_spent = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
    total_col += t_spent.count();
    count_col++;
    cost = std::max(d_hand_safe - min_distance, 0.0);
  // }
  // else
  // {
  //   cost = std::max(d_arm_safe - min_distance, 0.0); // <0 means colliding, >0 is ok
  // }
    

  cost = K_COL * cost; // weighting...

  return cost;

}


double MyUnaryConstraints::compute_collision_cost(Matrix<double, JOINT_DOF, 1> q_whole, boost::shared_ptr<DualArmDualHandCollision> &dual_arm_dual_hand_collision_ptr)
{
  // bypass
  if (K_COL == 0.0)
    return 0.0;

  // convert from matrix to std::vector
  std::vector<double> x(JOINT_DOF);
  for (unsigned int i = 0; i < JOINT_DOF; ++i)
    x[i] = q_whole[i];
  // Collision checking (or distance computation), check out the DualArmDualHandCollision class
  double cost;
  double min_distance;
  // check arms first, if ok, then hands. i.e. ensure arms safety before checking hands
  std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
  min_distance = dual_arm_dual_hand_collision_ptr->compute_self_distance_test(x, "dual_arms", d_arm_check); 
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  std::chrono::duration<double> t_spent = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  total_col += t_spent.count();
  count_col++;
  // std::cout << "debug: time spent on computing minimum distance = " << t_spent.count() << std::endl;
  // check_world_collision, check_full_collision, compute_self_distance, compute_world_distance
  if (min_distance > d_arm_safe) // arm safe
  {
    // check hands 
    t0 = std::chrono::steady_clock::now();
    min_distance = dual_arm_dual_hand_collision_ptr->compute_self_distance_test(x, "dual_hands", d_hand_check); 
    t1 = std::chrono::steady_clock::now();
    t_spent = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
    total_col += t_spent.count();
    count_col++;
    cost = std::max(d_hand_safe - min_distance, 0.0);
  }
  else
  {
    cost = std::max(d_arm_safe - min_distance, 0.0); // <0 means colliding, >0 is ok
  }
    

  cost = K_COL * cost; // weighting...

  return cost;

}

/*
double MyUnaryConstraints::compute_distance_cost(Matrix<double, JOINT_DOF, 1> q_whole, boost::shared_ptr<DualArmDualHandCollision> &dual_arm_dual_hand_collision_ptr)
{
  // convert from matrix to std::vector
  std::vector<double> x(JOINT_DOF);
  for (unsigned int i = 0; i < JOINT_DOF; ++i)
    x[i] = q_whole[i];

  // Collision checking (or distance computation), check out the DualArmDualHandCollision class
  std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
  // 1 - check collision (+1 / -1) is not quite reasonable in that points deep inside infeasible areas won't get updated
  // double min_distance = dual_arm_dual_hand_collision_ptr->check_self_collision(x); 
  // 2 - penetration depth can cope with the above issue, and all points in colliding state will be dealt with in order (since it's minimum depth..)
  double min_distance = dual_arm_dual_hand_collision_ptr->compute_self_distance(x); 
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  std::chrono::duration<double> t_spent = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  total_col += t_spent.count();
  count_col++;
  // std::cout << "debug: time spent on computing minimum distance = " << t_spent.count() << std::endl;
  // check_world_collision, check_full_collision, compute_self_distance, compute_world_distance

  // Compute cost for collision avoidance
  // 1
  // double cost = (min_distance + 1) * (min_distance + 1); // 1 for colliding state, -1 for non
  // 2
  double margin_of_safety = 0.01;//0.0;//0.01;
  double cost = std::max(margin_of_safety - min_distance, 0.0); // <0 means colliding, >0 is ok

  return cost;

}
*/

double MyUnaryConstraints::compute_pos_limit_cost(Matrix<double, JOINT_DOF, 1> q_whole, my_constraint_struct &fdata)
{
  // add safety margin for ease of limiting
  double safety_margin = 0.0;//0.01;

  // compute out-of-bound cost
  double cost = 0;
  for (unsigned int i = 0; i < JOINT_DOF; ++i)
  {
    // within [lb+margin, ub-margin], sub-set of [lb, ub]
    if (fdata.q_pos_ub[i] - safety_margin < q_whole[i])
      cost += std::max(q_whole[i] - (fdata.q_pos_ub[i] - safety_margin), 0.0); //(q_whole[i] - fdata.q_pos_ub[i] + safety_margin) * (q_whole[i] - fdata.q_pos_ub[i] + safety_margin);
    if (fdata.q_pos_lb[i] + safety_margin > q_whole[i])
      cost += std::max((fdata.q_pos_lb[i] + safety_margin) - q_whole[i], 0.0);//(fdata.q_pos_lb[i] + safety_margin - q_whole[i]) * (fdata.q_pos_lb[i] + safety_margin - q_whole[i]);
  }

  return cost;

}


double MyUnaryConstraints::return_col_cost()
{
  if (K_COL == 0.0)
    return 0.0;

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
  
  // 3 (for ease of distinction, won't use distance_computation here)
  std::vector<double> xx(JOINT_DOF);
  for (unsigned int d = 0; d < JOINT_DOF; d++)
    xx[d] = x[d];
 
  double col_result = dual_arm_dual_hand_collision_ptr->check_self_collision(xx);
  double collision_cost = (col_result > 0.0)? 1.0 : 0.0; // col_result 1 for colliding, -1 for collision-free
    //(col_result > 0.0)? 1.0 : 0.0; // col_result 1 for colliding, -1 for collision-free
    //compute_collision_cost(xx, dual_arm_dual_hand_collision_ptr);
  
  //std::cout << "Collision cost=";
  //std::cout << collision_cost << ", ";


  return collision_cost;
}

double MyUnaryConstraints::return_pos_limit_cost()
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
  

  // 4
  double pos_limit_cost = compute_pos_limit_cost(x, _measurement);
  //std::cout << "Pos limit cost=";
  //std::cout << pos_limit_cost << "; ";


  return pos_limit_cost;
}

void MyUnaryConstraints::computeError()
{


  // statistics
  count_unary++;  
  std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();

  //std::cout << "Computing Unary Constraint..." << std::endl;

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
  double collision_cost = compute_collision_cost(x, dual_arm_dual_hand_collision_ptr);
  
  // 2
  double pos_limit_cost = compute_pos_limit_cost(x, _measurement);

  // total cost
  double cost = K_COL * collision_cost + K_POS_LIMIT * pos_limit_cost;
  _error(0, 0) = cost;

  // Display message
  //std::cout << "Computing unary edge's cost: " << cost << std::endl;
  
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  std::chrono::duration<double> t_01 = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  total_unary += t_01.count();

}




/* Constraint for smoothness between adjacent path points */
class SmoothnessConstraint : public BaseBinaryEdge<1, double, DualArmDualHandVertex, DualArmDualHandVertex>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    // set up bounds ?
    double margin_of_smoothness = std::sqrt(std::pow(1.0 * M_PI / 180.0, 2) * JOINT_DOF);

    void computeError()
    {

      // statistics
      count_smoothness++;  
      std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();

      //std::cout << "Computing Smoothness Constraint..." << std::endl;

      // Get the values of the two vertices
      const DualArmDualHandVertex *v0 = static_cast<const DualArmDualHandVertex*>(_vertices[0]);
      const DualArmDualHandVertex *v1 = static_cast<const DualArmDualHandVertex*>(_vertices[1]);
      const Matrix<double, JOINT_DOF, 1> x0 = v0->estimate(); 
      const Matrix<double, JOINT_DOF, 1> x1 = v1->estimate(); 

      // Compute smoothness cost
      _error(0, 0) = K_SMOOTHNESS * max( (x0 - x1).norm() - margin_of_smoothness, 0.0);
      //std::cout << "Smoothness cost=";
      //std::cout << _error(0, 0) << std::endl;

      // Display message
      //std::cout << "Computing binary edge's cost: " << _error(0, 0) << std::endl;

      std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
      std::chrono::duration<double> t_spent = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
      total_smoothness += t_spent.count();

    }

    // function for recording cost hsitory
    double return_smoothness_cost()
    {
      // Get the values of the two vertices
      const DualArmDualHandVertex *v0 = static_cast<const DualArmDualHandVertex*>(_vertices[0]);
      const DualArmDualHandVertex *v1 = static_cast<const DualArmDualHandVertex*>(_vertices[1]);
      const Matrix<double, JOINT_DOF, 1> x0 = v0->estimate(); 
      const Matrix<double, JOINT_DOF, 1> x1 = v1->estimate(); 

      return max( (x0 - x1).norm() - margin_of_smoothness, 0.0);
    }

    virtual void linearizeOplus()
    {
      // setup
      double q_arm_eps = 2.0 * M_PI / 180.0;
      double q_finger_eps = 2.0 * M_PI / 180.0;

      // Get vertex values
      DualArmDualHandVertex *v0 = static_cast<DualArmDualHandVertex*>(_vertices[0]);
      DualArmDualHandVertex *v1 = static_cast<DualArmDualHandVertex*>(_vertices[1]);
      const Matrix<double, JOINT_DOF, 1> x0 = v0->estimate(); 
      const Matrix<double, JOINT_DOF, 1> x1 = v1->estimate(); 

      // prep
      Matrix<double, JOINT_DOF, 1> delta_x = Matrix<double, JOINT_DOF, 1>::Zero();
      double e_plus, e_minus;

      // For arms 
      for (unsigned int d = 0; d < 7; d++)
      {
        // --- Left Arm --- //

        // set delta
        delta_x[d] = q_arm_eps;

        // (1) - jacobian for the former vertex
        v0->setEstimate(x0+delta_x);
        this->computeError();
        e_plus = _error(0, 0);
        v0->setEstimate(x0-delta_x);
        this->computeError();
        e_minus = _error(0, 0);
        _jacobianOplusXi(0, d) = (e_plus-e_minus) / (2*q_arm_eps);
        v0->setEstimate(x0); // reset

        // (2) - jacobian for the latter vertex
        v1->setEstimate(x1+delta_x);
        this->computeError();
        e_plus = _error(0, 0);
        v1->setEstimate(x1-delta_x);
        this->computeError();
        e_minus = _error(0, 0);
        _jacobianOplusXj(0, d) = (e_plus-e_minus) / (2*q_arm_eps);
        v1->setEstimate(x1); // reset

        // reset delta
        delta_x[d] = 0.0;


        // --- Right Arm --- //

        // set delta
        delta_x[d+7] = q_arm_eps;

        // (1) - jacobian for the former vertex
        v0->setEstimate(x0+delta_x);
        this->computeError();
        e_plus = _error(0, 0);
        v0->setEstimate(x0-delta_x);
        this->computeError();
        e_minus = _error(0, 0);
        _jacobianOplusXi(0, d+7) = (e_plus-e_minus) / (2*q_arm_eps);
        v0->setEstimate(x0); // reset

        // (2) - jacobian for the latter vertex
        v1->setEstimate(x1+delta_x);
        this->computeError();
        e_plus = _error(0, 0);
        v1->setEstimate(x1-delta_x);
        this->computeError();
        e_minus = _error(0, 0);
        _jacobianOplusXj(0, d+7) = (e_plus-e_minus) / (2*q_arm_eps);
        v1->setEstimate(x1); // reset

        // reset delta
        delta_x[d+7] = 0.0;

      }

      // For fingers
      for (unsigned int d = 0; d < 12; d++)
      {
        // --- Left Hand --- //
        
        // set delta
        delta_x[d+14] = q_finger_eps;

        // (1) - jacobian for the former vertex
        v0->setEstimate(x0+delta_x);
        this->computeError();
        e_plus = _error(0, 0);
        v0->setEstimate(x0-delta_x);
        this->computeError();
        e_minus = _error(0, 0);
        _jacobianOplusXi(0, d+14) = (e_plus-e_minus) / (2*q_finger_eps);
        v0->setEstimate(x0); // reset

        // (2) - jacobian for the latter vertex
        v1->setEstimate(x1+delta_x);
        this->computeError();
        e_plus = _error(0, 0);
        v1->setEstimate(x1-delta_x);
        this->computeError();
        e_minus = _error(0, 0);
        _jacobianOplusXj(0, d+14) = (e_plus-e_minus) / (2*q_finger_eps);
        v1->setEstimate(x1); // reset

        // reset delta
        delta_x[d+14] = 0.0;


        // --- Right Arm --- //

        // set delta
        delta_x[d+26] = q_finger_eps;

        // (1) - jacobian for the former vertex
        v0->setEstimate(x0+delta_x);
        this->computeError();
        e_plus = _error(0, 0);
        v0->setEstimate(x0-delta_x);
        this->computeError();
        e_minus = _error(0, 0);
        _jacobianOplusXi(0, d+26) = (e_plus-e_minus) / (2*q_finger_eps);
        v0->setEstimate(x0); // reset

        // (2) - jacobian for the latter vertex
        v1->setEstimate(x1+delta_x);
        this->computeError();
        e_plus = _error(0, 0);
        v1->setEstimate(x1-delta_x);
        this->computeError();
        e_minus = _error(0, 0);
        _jacobianOplusXj(0, d+26) = (e_plus-e_minus) / (2*q_finger_eps);
        v1->setEstimate(x1); // reset

        // reset delta
        delta_x[d+26] = 0.0;

      }

      // store jacobians for debug
      for (unsigned int j = 0; j < JOINT_DOF; j++)
      {
        q_jacobian(0, j) = _jacobianOplusXi(0, j);
        q_jacobian(1, j) = _jacobianOplusXj(0, j);
      }


    }


    // output jacobians for debug
    Matrix<double, 2, JOINT_DOF> q_jacobian;
    Matrix<double, 2, JOINT_DOF> output_q_jacobian()
    {
      return q_jacobian;
    }

    // Read and write, leave blank
    virtual bool read( std::istream& in ) {return true;}
    virtual bool write( std::ostream& out ) const {return true;}

};



/* Define constraint for computing tracking error */
/* addVertex rule: connect pass_points first, then datapoints */
class TrackingConstraint : public BaseMultiEdge<1, my_constraint_struct> // <D, E>, dimension and measurement datatype
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // Constructor and destructor
    TrackingConstraint(boost::shared_ptr<DMPTrajectoryGenerator> &_trajectory_generator_ptr, 
                      KDL::ChainFkSolverPos_recursive &_left_fk_solver, 
                      KDL::ChainFkSolverPos_recursive &_right_fk_solver, 
                      boost::shared_ptr<DualArmDualHandCollision> &_dual_arm_dual_hand_collision_ptr,
                      //unsigned int num_passpoints,
                      unsigned int num_datapoints) : trajectory_generator_ptr(_trajectory_generator_ptr), 
                                                     left_fk_solver(_left_fk_solver), 
                                                     right_fk_solver(_right_fk_solver), 
                                                     dual_arm_dual_hand_collision_ptr(_dual_arm_dual_hand_collision_ptr)
    {
      // record
      this->num_datapoints = num_datapoints;
      //this->num_passpoints = num_passpoints;
      // resize the number of vertices this edge connects to
      //std::cout << "resizing tracking constraint..." << std::endl;
      //resize(num_datapoints+num_passpoints);
      this->num_vertices = num_datapoints + 1;
      resize(num_datapoints+1); // q path points plus one DMP vertex
    }
    ~TrackingConstraint(){};    

    // numbers
    unsigned int num_datapoints, num_passpoints;
    unsigned int num_vertices;

    // trajectory generator
    //boost::shared_ptr<TrajectoryGenerator> &trajectory_generator_ptr;
    boost::shared_ptr<DMPTrajectoryGenerator> &trajectory_generator_ptr;


    // KDL solver
    KDL::ChainFkSolverPos_recursive &left_fk_solver;
    KDL::ChainFkSolverPos_recursive &right_fk_solver;

    // Collision Checker, which constains RobotState that can be used to compute jacobians, global link transforms, etc.
    boost::shared_ptr<DualArmDualHandCollision> &dual_arm_dual_hand_collision_ptr;
    const std::string LEFT_SHOULDER_LINK = "yumi_link_1_l";
    const std::string LEFT_ELBOW_LINK = "yumi_link_4_l";
    const std::string LEFT_WRIST_LINK = "yumi_link_7_l";
    const std::string RIGHT_SHOULDER_LINK = "yumi_link_1_r";
    const std::string RIGHT_ELBOW_LINK = "yumi_link_4_r";
    const std::string RIGHT_WRIST_LINK = "yumi_link_7_r";

    // functions to compute costs
    double compute_arm_cost(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, unsigned int num_elbow_seg, unsigned int num_shoulder_seg, bool left_or_right, my_constraint_struct &fdata);
    double linear_map(double x_, double min_, double max_, double min_hat, double max_hat);
    double compute_finger_cost(Matrix<double, 12, 1> q_finger_robot, bool left_or_right, my_constraint_struct &fdata);
    void computeError();

    // functions for recording cost history
    std::vector<double> return_finger_cost_history();
    double return_wrist_pos_cost(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, unsigned int num_elbow_seg, unsigned int num_shoulder_seg, bool left_or_right, my_constraint_struct &fdata);
    std::vector<double> return_wrist_pos_cost_history();
    std::vector<double> return_l_wrist_pos_cost_history();
    std::vector<double> return_r_wrist_pos_cost_history();

    double return_wrist_ori_cost(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, unsigned int num_elbow_seg, unsigned int num_shoulder_seg, bool left_or_right, my_constraint_struct &fdata);
    std::vector<double> return_wrist_ori_cost_history();

    double return_elbow_pos_cost(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, unsigned int num_elbow_seg, unsigned int num_shoulder_seg, bool left_or_right, my_constraint_struct &fdata);
    std::vector<double> return_elbow_pos_cost_history();
    std::vector<double> return_l_elbow_pos_cost_history();
    std::vector<double> return_r_elbow_pos_cost_history();

    Vector3d return_wrist_pos_offset(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, unsigned int num_elbow_seg, unsigned int num_shoulder_seg, bool left_or_right, my_constraint_struct &fdata);
    Vector3d return_elbow_pos_offset(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, unsigned int num_elbow_seg, unsigned int num_shoulder_seg, bool left_or_right, my_constraint_struct &fdata);
    MatrixXd return_wrist_pos_offsets();

    // for manual manipulation of DMP starts and goals
    MatrixXd return_l_wrist_pos_offsets();
    MatrixXd return_r_wrist_pos_offsets();
    MatrixXd return_l_elbow_pos_offsets();
    MatrixXd return_r_elbow_pos_offsets();

    // return currently executed Cartesian trajectories for debug
    std::vector<std::vector<double> > return_wrist_pos_traj(bool left_or_right);
    std::vector<std::vector<double> > return_elbow_pos_traj(bool left_or_right);
    Vector3d return_wrist_pos(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, unsigned int num_elbow_seg, unsigned int num_shoulder_seg, bool left_or_right, my_constraint_struct &fdata);
    Vector3d return_elbow_pos(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, unsigned int num_elbow_seg, unsigned int num_shoulder_seg, bool left_or_right, my_constraint_struct &fdata);
  
    // return Jacobians results for debug
    MatrixXd output_dmp_jacobian();
    MatrixXd output_q_jacobian();
    Matrix<double, 1, DMPPOINTS_DOF> jacobians_for_dmp;
    Matrix<double, NUM_DATAPOINTS, JOINT_DOF> jacobians_for_q;

    // for debugging every parts' jacobians
    double cur_wrist_pos_cost;
    double cur_wrist_ori_cost;
    double cur_elbow_pos_cost;
    double cur_finger_pos_cost; // for a single path point; used in compute_arm_cost() or compute_finger_cost()
    Matrix<double, 14, NUM_DATAPOINTS> wrist_pos_jacobian_for_q_arm;
    Matrix<double, 14, NUM_DATAPOINTS> wrist_ori_jacobian_for_q_arm;
    Matrix<double, 14, NUM_DATAPOINTS> elbow_pos_jacobian_for_q_arm;

    Matrix<double, DMPPOINTS_DOF, 1> wrist_pos_jacobian_for_dmp;
    Matrix<double, DMPPOINTS_DOF, 1> wrist_ori_jacobian_for_dmp;
    Matrix<double, DMPPOINTS_DOF, 1> elbow_pos_jacobian_for_dmp;    
    Matrix<double, DMPPOINTS_DOF, 1> finger_pos_jacobian_for_dmp;

    double cur_wrist_pos_cost_total;
    double cur_wrist_ori_cost_total;
    double cur_elbow_pos_cost_total;
    double cur_finger_pos_cost_total; // for all path points; used in computeError()

    // Re-implement numeric differentiation
    virtual void linearizeOplus();

    // Read and write, leave blank
    virtual bool read( std::istream& in ) {return true;}
    virtual bool write( std::ostream& out ) const {return true;}
    
};


/* Return the wrist position corresponding to current joint configuration */
Vector3d TrackingConstraint::return_wrist_pos(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, unsigned int num_elbow_seg, unsigned int num_shoulder_seg, bool left_or_right, my_constraint_struct &fdata)
{
  // Get joint angles
  KDL::JntArray q_in(q_cur.size()); 
  for (unsigned int i = 0; i < q_cur.size(); ++i)
  {
    q_in(i) = q_cur(i);
  }

  // Do FK using KDL, get the current elbow/wrist/shoulder state
  KDL::Frame wrist_cart_out;//, shoulder_cart_out; // Output homogeneous transformation
  int result;
  result = fk_solver.JntToCart(q_in, wrist_cart_out, num_wrist_seg+1);
  if (result < 0){
    ROS_INFO_STREAM("FK solver failed when processing wrist link, something went wrong");
    exit(-1);
  }
  else{
    //ROS_INFO_STREAM("FK solver succeeded for wrist link.");
  }
 
  // Preparations
  Vector3d wrist_pos_cur = Map<Vector3d>(wrist_cart_out.p.data, 3, 1);
  
  // return results
  return wrist_pos_cur;

}

/* Return the elbow position corresponding to current joint configuration */
Vector3d TrackingConstraint::return_elbow_pos(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, unsigned int num_elbow_seg, unsigned int num_shoulder_seg, bool left_or_right, my_constraint_struct &fdata)
{
 // Get joint angles
  KDL::JntArray q_in(q_cur.size()); 
  for (unsigned int i = 0; i < q_cur.size(); ++i)
  {
    q_in(i) = q_cur(i);
  }

  // Do FK using KDL, get the current elbow/wrist/shoulder state
  KDL::Frame elbow_cart_out;//, shoulder_cart_out; // Output homogeneous transformation
  int result;
  result = fk_solver.JntToCart(q_in, elbow_cart_out, num_elbow_seg+1); // notice that the number here is not the segment ID, but the number of segments till target segment
  if (result < 0){
    ROS_INFO_STREAM("FK solver failed when processing elbow link, something went wrong");
    exit(-1);
  }
  else{
    //ROS_INFO_STREAM("FK solver succeeded for elbow link.");
  }
 
  // Preparations
  Vector3d elbow_pos_cur = Map<Vector3d>(elbow_cart_out.p.data, 3, 1);
 
  // return results
  return elbow_pos_cur;

}


/* Return the current wrist position trajectory via FK */
std::vector<std::vector<double> > TrackingConstraint::return_wrist_pos_traj(bool left_or_right)
{
  // prep
  std::vector<std::vector<double>> wrist_pos_traj;
  std::vector<double> cur_wrist_pos(3);

  for (unsigned int n = 0; n < num_datapoints; n++)
  {
    // get the current joint value
    // _vertices is a VertexContainer type, a std::vector<Vertex*>
    const DualArmDualHandVertex *v = static_cast<const DualArmDualHandVertex*>(_vertices[n+1]);
    const Matrix<double, JOINT_DOF, 1> x = v->estimate(); // return the current estimate of the vertex
    //std::cout << "debug: x size is: " << x.rows() << " x " << x.cols() << std::endl;
    //std::cout << "debug: x = \n" << x.transpose() << std::endl;

    // Get joint angles
    Matrix<double, 7, 1> q_cur_l, q_cur_r;
    Matrix<double, 12, 1> q_cur_finger_l, q_cur_finger_r;
    q_cur_l = x.block<7, 1>(0, 0);
    q_cur_r = x.block<7, 1>(7, 0);
    q_cur_finger_l = x.block<12, 1>(14, 0);
    q_cur_finger_r = x.block<12, 1>(26, 0); 
    
    
    // Compute unary costs
    Vector3d cur_wrist_pos_vec;
    if(left_or_right) 
    {
      cur_wrist_pos_vec = return_wrist_pos(left_fk_solver, q_cur_l, _measurement.l_num_wrist_seg, _measurement.l_num_elbow_seg, _measurement.l_num_shoulder_seg, true, _measurement);
    }
    else
    {
      cur_wrist_pos_vec = return_wrist_pos(right_fk_solver, q_cur_r, _measurement.r_num_wrist_seg, _measurement.r_num_elbow_seg, _measurement.r_num_shoulder_seg, false, _measurement);
    }

    // Copy to std::vector
    cur_wrist_pos[0] = cur_wrist_pos_vec[0]; 
    cur_wrist_pos[1] = cur_wrist_pos_vec[1]; 
    cur_wrist_pos[2] = cur_wrist_pos_vec[2]; 
    wrist_pos_traj.push_back(cur_wrist_pos);

  }

  return wrist_pos_traj;

}

/* Return the current elbow position trajectory via FK */
std::vector<std::vector<double> > TrackingConstraint::return_elbow_pos_traj(bool left_or_right)
{
  // prep
  std::vector<std::vector<double>> elbow_pos_traj;
  std::vector<double> cur_elbow_pos(3);

  for (unsigned int n = 0; n < num_datapoints; n++)
  {
    // get the current joint value
    // _vertices is a VertexContainer type, a std::vector<Vertex*>
    const DualArmDualHandVertex *v = static_cast<const DualArmDualHandVertex*>(_vertices[n+1]);
    const Matrix<double, JOINT_DOF, 1> x = v->estimate(); // return the current estimate of the vertex
    //std::cout << "debug: x size is: " << x.rows() << " x " << x.cols() << std::endl;
    //std::cout << "debug: x = \n" << x.transpose() << std::endl;

    // Get joint angles
    Matrix<double, 7, 1> q_cur_l, q_cur_r;
    Matrix<double, 12, 1> q_cur_finger_l, q_cur_finger_r;
    q_cur_l = x.block<7, 1>(0, 0);
    q_cur_r = x.block<7, 1>(7, 0);
    q_cur_finger_l = x.block<12, 1>(14, 0);
    q_cur_finger_r = x.block<12, 1>(26, 0); 
    
    
    // Compute unary costs
    Vector3d cur_elbow_pos_vec;
    if(left_or_right) 
    {
      cur_elbow_pos_vec = return_elbow_pos(left_fk_solver, q_cur_l, _measurement.l_num_wrist_seg, _measurement.l_num_elbow_seg, _measurement.l_num_shoulder_seg, true, _measurement);
    }
    else
    {
      cur_elbow_pos_vec = return_elbow_pos(right_fk_solver, q_cur_r, _measurement.r_num_wrist_seg, _measurement.r_num_elbow_seg, _measurement.r_num_shoulder_seg, false, _measurement);
    }

    // Copy to std::vector
    cur_elbow_pos[0] = cur_elbow_pos_vec[0]; 
    cur_elbow_pos[1] = cur_elbow_pos_vec[1]; 
    cur_elbow_pos[2] = cur_elbow_pos_vec[2]; 
    elbow_pos_traj.push_back(cur_elbow_pos);

  }

  return elbow_pos_traj;

}



/* Output just the jacobians for DMP starts_and_goals vertex */
MatrixXd TrackingConstraint::output_dmp_jacobian()
{
  // TrackingConstraint is a MultiEdge, _jacobianOplus contains jacobians for DMP starts_and_goals vertex and q vertices!!! 
  /*
  MatrixXd jacobians(1, DMPPOINTS_DOF);
  for (unsigned int n = 0; n < 1; n++) // the first item is for DMP starts_and_goals vertex !
  {
    for (unsigned int p = 0; p < DMPPOINTS_DOF; p++)
      jacobians(n, p) = _jacobianOplus[n](0, p);
  }
  */

  //std::cout << "debug: " << _jacobianOplus.size() << " x " << _jacobianOplus[0].rows() << " x " << _jacobianOplus[0].cols() << std::endl;
  
  return jacobians_for_dmp;

}

/* Output just the jacobians for q vertices */
MatrixXd TrackingConstraint::output_q_jacobian()
{
  return jacobians_for_q;
}

void TrackingConstraint::linearizeOplus()
{

  double dmp_eps = 0.001; // better be small enough so that gradients on different dimensions won't diverge too much
  double q_arm_eps = 2.0 * M_PI / 180; //0.5; // may need tests // in radius!!!
  double q_finger_eps = 2.0 * M_PI / 180; // in radius
  double e_plus, e_minus;
  // for calculating and storing jacobians for wrist_pos, wrist_ori and elbow_pos
  double e_wrist_pos_plus, e_wrist_pos_minus;  
  double e_wrist_ori_plus, e_wrist_ori_minus;
  double e_elbow_pos_plus, e_elbow_pos_minus;
  double e_finger_pos_plus, e_finger_pos_minus;

  // double dmp_update_bound = 10; //20; //50;//1; //5; // 10;
  // double scale = 0.1;

  // 1 - For DMP starts and goals
  DMPStartsGoalsVertex *v = dynamic_cast<DMPStartsGoalsVertex*>(_vertices[0]); // the last vertex connected
  Matrix<double, DMPPOINTS_DOF, 1> x = v->estimate();
  Matrix<double, DMPPOINTS_DOF, 1> delta_x = Matrix<double, DMPPOINTS_DOF, 1>::Zero();
  //std::cout << "debug: error = ";
  for (unsigned int n = 0; n < DMPPOINTS_DOF; n++)
  {
    // set delta
    delta_x[n] = dmp_eps;

    // assign and compute
    v->setEstimate(x+delta_x);
    this->computeError();
    e_plus = _error(0, 0);
    e_wrist_pos_plus = this->cur_wrist_pos_cost_total; // record after calling computeError(); different from that for q
    e_wrist_ori_plus = this->cur_wrist_ori_cost_total;
    e_elbow_pos_plus = this->cur_elbow_pos_cost_total;
    e_finger_pos_plus = this->cur_finger_pos_cost_total;

    v->setEstimate(x-delta_x);
    this->computeError();
    e_minus = _error(0, 0);
    e_wrist_pos_minus = this->cur_wrist_pos_cost_total; // record after calling computeError(); different from that for q 
    e_wrist_ori_minus = this->cur_wrist_ori_cost_total;
    e_elbow_pos_minus = this->cur_elbow_pos_cost_total; 
    e_finger_pos_minus = this->cur_finger_pos_cost_total;   

    // reset delta
    delta_x[n] = 0.0;

    // set and store jacobians 
    _jacobianOplus[0](0, n) = (e_plus - e_minus) / (2*dmp_eps); // for DMP starts and goals
    //std::cout << e_plus - e_minus << " ";
    wrist_pos_jacobian_for_dmp[n] = K_WRIST_POS * (e_wrist_pos_plus - e_wrist_pos_minus) / ( 2 * dmp_eps);
    wrist_ori_jacobian_for_dmp[n] = K_WRIST_ORI * (e_wrist_ori_plus - e_wrist_ori_minus) / ( 2 * dmp_eps);
    elbow_pos_jacobian_for_dmp[n] = K_ELBOW_POS * (e_elbow_pos_plus - e_elbow_pos_minus) / ( 2 * dmp_eps); // remember to change the eps !!
    finger_pos_jacobian_for_dmp[n] = K_FINGER * (e_finger_pos_plus - e_finger_pos_minus) / (2 * dmp_eps);

    // set bounds for DMP jacobians
    /*
    if(_jacobianOplus[0](0, n) > dmp_update_bound)
      _jacobianOplus[0](0, n) = dmp_update_bound;
    if(_jacobianOplus[0](0, n) < -dmp_update_bound)
      _jacobianOplus[0](0, n) = -dmp_update_bound;
    */

    // store jacobians
    this->jacobians_for_dmp[n] = _jacobianOplus[0](0, n);


    // tmp debug:
    /*
    std::cout << "debug: current gradient = " << this->jacobians_for_dmp[n] << std::endl;
    std::cout << "debug: current wrist_pos gradient = " << wrist_pos_jacobian_for_dmp[n] << std::endl;
    std::cout << "debug: current wrist_ori gradient = " << wrist_ori_jacobian_for_dmp[n] << std::endl;
    std::cout << "debug: current elbow_pos gradient = " << elbow_pos_jacobian_for_dmp[n] << std::endl;
    std::cout << "debug: current finger_pos gradient = " << finger_pos_jacobian_for_dmp[n] << std::endl;

    double p=0;
    */

  }
  // reset vertex value
  v->setEstimate(x);
  //std::cout << std::endl;                                                                                                                                                             


  // normalize the jacobians of DMP starts and goals 
  /*
  this->jacobians_for_dmp = this->jacobians_for_dmp.normalized();
  for (unsigned int n = 0; n < DMPPOINTS_DOF; n++)
    _jacobianOplus[0](0, n) = this->jacobians_for_dmp[n];
  */


  // print jacobians for debug
  //std::cout << "debug: tracking jacobians = " << this->jacobians_for_dmp << std::endl;

  
  // show jacobians for computing condition number
  // not a useful way to estimate, since J^T*J is bound to be a rank 1 matrix, and thus yielding huge condition number
  /*
  MatrixXd JTJ = _jacobianOplus[0].transpose() * _jacobianOplus[0]; // Matrix<double, DMPPOINTS_DOF, DMPPOINTS_DOF> 
  //std::cout << "debug: J^T * J = " << JTJ << std::endl;
  double lpnorm = JTJ.colwise().lpNorm<1>().maxCoeff(); // L1 norm (column norm)
  double inverse_lpnorm = JTJ.inverse().colwise().lpNorm<1>().maxCoeff(); // L1 norm (column norm)
  //double lpnorm = JTJ.rowwise().lpNorm<1>().maxCoeff(); // infinity norm (row norm)
  //double inverse_lpnorm = JTJ.inverse().rowwise().lpNorm<1>().maxCoeff(); // infinity norm (row norm)
  double cond = lpnorm * inverse_lpnorm; 
  std::cout << "debug: Cond(J^T * T) " << (isnan(cond) ? "is not" : "is") << " a number. (check NaN)." << std::endl;
  std::cout << "debug: Cond(J^T * T) " << (isinf(cond) ? "is" : "is not") << " Infinity. (check Inf)." << std::endl;  
  std::cout << "debug: Cond(J^T * T) = " << cond << std::endl;

  // check different bounds
  std::vector<double> bounds = {0.1, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0};
  for (unsigned int b = 0; b < bounds.size(); b++)
  {
    Matrix<double, 1, DMPPOINTS_DOF> jacobian_tmp = this->jacobians_for_dmp;
    for (unsigned int d = 0; d < DMPPOINTS_DOF; d++)
    {
      // clamp the original values
      if(jacobian_tmp(0, d) > bounds[b])
        jacobian_tmp(0, d) = bounds[b];
      if(jacobian_tmp(0, d) < -bounds[b])
        jacobian_tmp(0, d) = -bounds[b];
    }
    // compute condition numbers
    JTJ = jacobian_tmp.transpose() * jacobian_tmp; // Matrix<double, DMPPOINTS_DOF, DMPPOINTS_DOF> 
    lpnorm = JTJ.colwise().lpNorm<1>().maxCoeff(); // L1 norm (column norm)
    inverse_lpnorm = JTJ.inverse().colwise().lpNorm<1>().maxCoeff(); // L1 norm (column norm)
    //double lpnorm = JTJ.rowwise().lpNorm<1>().maxCoeff(); // infinity norm (row norm)
    //double inverse_lpnorm = JTJ.inverse().rowwise().lpNorm<1>().maxCoeff(); // infinity norm (row norm)
    cond = lpnorm * inverse_lpnorm; 
    std::cout << ">> Bound is " << bounds[b] << std::endl;
    std::cout << "Cond(J^T * T) " << (isnan(cond) ? "is not" : "is") << " a number. (check NaN)." << std::endl;
    std::cout << "Cond(J^T * T) " << (isinf(cond) ? "is" : "is not") << " Infinity. (check Inf)." << std::endl;  
    std::cout << "Cond(J^T * T) = " << cond << std::endl;
  }
  */


  // 2 - For q vertices
  MatrixXd lrw_new_goal(3, 1); lrw_new_goal = x.block(0, 0, 3, 1);
  MatrixXd lrw_new_start(3, 1); lrw_new_start = x.block(3, 0, 3, 1);
  MatrixXd lew_new_goal(3, 1); lew_new_goal = x.block(6, 0, 3, 1);
  MatrixXd lew_new_start(3, 1); lew_new_start = x.block(9, 0, 3, 1);
  MatrixXd rew_new_goal(3, 1); rew_new_goal = x.block(12, 0, 3, 1);
  MatrixXd rew_new_start(3, 1); rew_new_start = x.block(15, 0, 3, 1);
  MatrixXd rw_new_goal(3, 1); rw_new_goal = x.block(18, 0, 3, 1);
  MatrixXd rw_new_start(3, 1); rw_new_start = x.block(21, 0, 3, 1);
  std::chrono::steady_clock::time_point t00 = std::chrono::steady_clock::now();  
  DMP_trajs result = trajectory_generator_ptr->generate_trajectories(lrw_new_goal.transpose(), lrw_new_start.transpose(), // should be row vectors
                                                                      lew_new_goal.transpose(), lew_new_start.transpose(),
                                                                      rew_new_goal.transpose(), rew_new_start.transpose(),
                                                                      rw_new_goal.transpose(), rw_new_start.transpose(),
                                                                      NUM_DATAPOINTS); // results are 3 x N
  std::chrono::steady_clock::time_point t11 = std::chrono::steady_clock::now();
  std::chrono::duration<double> t_spent1 = std::chrono::duration_cast<std::chrono::duration<double>>(t11 - t00);
  total_traj += t_spent1.count();
  count_traj++;  
  // Iterate to compute jacobians
  Matrix<double, 7, 1> delta_q_arm = Matrix<double, 7, 1>::Zero();
  Matrix<double, 12, 1> delta_q_finger = Matrix<double, 12, 1>::Zero();  
  for (unsigned int n = 0; n < num_datapoints; n++)
  {
    // get the current joint values
    const DualArmDualHandVertex *v_tmp = dynamic_cast<const DualArmDualHandVertex*>(_vertices[n+1]);
    const Matrix<double, JOINT_DOF, 1> q = v_tmp->estimate(); // return the current estimate of the vertex

    // Get joint angles
    Matrix<double, 7, 1> q_cur_l, q_cur_r;    
    Matrix<double, 12, 1> q_cur_finger_l, q_cur_finger_r;
    q_cur_l = q.block<7, 1>(0, 0);
    q_cur_r = q.block<7, 1>(7, 0);
    q_cur_finger_l = q.block<12, 1>(14, 0);
    q_cur_finger_r = q.block<12, 1>(26, 0); 
    
    // Set new goals(expected trajectory) to _measurement
    _measurement.l_wrist_pos_goal = result.y_lw.block(0, n, 3, 1); // Vector3d
    Quaterniond q_l(trajectory_generator_ptr->l_wrist_quat_traj(n, 0), 
                    trajectory_generator_ptr->l_wrist_quat_traj(n, 1),
                    trajectory_generator_ptr->l_wrist_quat_traj(n, 2),
                    trajectory_generator_ptr->l_wrist_quat_traj(n, 3));
    Matrix3d l_wrist_ori_goal = q_l.toRotationMatrix();
    _measurement.l_wrist_ori_goal = l_wrist_ori_goal; // Matrix3d
    _measurement.l_elbow_pos_goal = result.y_le.block(0, n, 3, 1); // Vector3d

    // right arm part:
    _measurement.r_wrist_pos_goal = result.y_rw.block(0, n, 3, 1); // Vector3d
    Quaterniond q_r(trajectory_generator_ptr->r_wrist_quat_traj(n, 0), 
                    trajectory_generator_ptr->r_wrist_quat_traj(n, 1),
                    trajectory_generator_ptr->r_wrist_quat_traj(n, 2),
                    trajectory_generator_ptr->r_wrist_quat_traj(n, 3));
    Matrix3d r_wrist_ori_goal = q_r.toRotationMatrix();  
    _measurement.r_wrist_ori_goal = r_wrist_ori_goal; // Matrix3d
    _measurement.r_elbow_pos_goal = result.y_re.block(0, n, 3, 1); // Vector3d is column vector

    // hand parts:
    _measurement.l_finger_pos_goal = trajectory_generator_ptr->l_glove_angle_traj.block(n, 0, 1, 14).transpose() * M_PI / 180.0; // y_seq's glove data is already in radius
    _measurement.r_finger_pos_goal = trajectory_generator_ptr->r_glove_angle_traj.block(n, 0, 1, 14).transpose() * M_PI / 180.0; // size is 50 x DOF

    
    // (1) - jacobians for arms
    double right_tmp = compute_arm_cost(right_fk_solver, q_cur_r, _measurement.r_num_wrist_seg, _measurement.r_num_elbow_seg, _measurement.r_num_shoulder_seg, false, _measurement);
    double wrist_pos_cost_right_tmp = this->cur_wrist_pos_cost; // record after calling compute_arm_cost()
    double wrist_ori_cost_right_tmp = this->cur_wrist_ori_cost;
    double elbow_pos_cost_right_tmp = this->cur_elbow_pos_cost;
    double left_tmp = compute_arm_cost(left_fk_solver, q_cur_l, _measurement.l_num_wrist_seg, _measurement.l_num_elbow_seg, _measurement.l_num_shoulder_seg, true, _measurement); // user data is stored in _measurement now
    double wrist_pos_cost_left_tmp = this->cur_wrist_pos_cost;
    double wrist_ori_cost_left_tmp = this->cur_wrist_ori_cost;
    double elbow_pos_cost_left_tmp = this->cur_elbow_pos_cost; // record after calling compute_arm_cost()
    for (unsigned int d = 0; d < 7; d++)
    {
      // set delta
      delta_q_arm[d] = q_arm_eps;

      // left arm
      e_plus = compute_arm_cost(left_fk_solver, q_cur_l+delta_q_arm, _measurement.l_num_wrist_seg, _measurement.l_num_elbow_seg, _measurement.l_num_shoulder_seg, true, _measurement); // user data is stored in _measurement now
      e_wrist_pos_plus = this->cur_wrist_pos_cost;
      e_wrist_pos_plus += wrist_pos_cost_right_tmp;
      e_wrist_ori_plus = this->cur_wrist_ori_cost;
      e_wrist_ori_plus += wrist_ori_cost_right_tmp;
      e_elbow_pos_plus = this->cur_elbow_pos_cost;
      e_elbow_pos_plus += elbow_pos_cost_right_tmp;            
      e_plus += right_tmp;

      e_minus = compute_arm_cost(left_fk_solver, q_cur_l-delta_q_arm, _measurement.l_num_wrist_seg, _measurement.l_num_elbow_seg, _measurement.l_num_shoulder_seg, true, _measurement); // user data is stored in _measurement now
      e_wrist_pos_minus = this->cur_wrist_pos_cost;
      e_wrist_pos_minus += wrist_pos_cost_left_tmp;
      e_wrist_ori_minus = this->cur_wrist_ori_cost;
      e_wrist_ori_minus += wrist_ori_cost_left_tmp;
      e_elbow_pos_minus = this->cur_elbow_pos_cost;
      e_elbow_pos_minus += elbow_pos_cost_left_tmp;    
      e_minus += right_tmp;

      _jacobianOplus[n+1](0, d) = (e_plus - e_minus) / (2 * q_arm_eps);
      wrist_pos_jacobian_for_q_arm(d, n) = K_WRIST_POS * (e_wrist_pos_plus - e_wrist_pos_minus) / ( 2 * q_arm_eps);
      wrist_ori_jacobian_for_q_arm(d, n) = K_WRIST_ORI * (e_wrist_ori_plus - e_wrist_ori_minus) / ( 2 * q_arm_eps);
      elbow_pos_jacobian_for_q_arm(d, n) = K_ELBOW_POS * (e_elbow_pos_plus - e_elbow_pos_minus) / ( 2 * q_arm_eps);


      // right arm
      e_plus = left_tmp;
      e_wrist_pos_plus = wrist_pos_cost_left_tmp;
      e_wrist_ori_plus = wrist_ori_cost_left_tmp;
      e_elbow_pos_plus = elbow_pos_cost_left_tmp; 
      e_plus += compute_arm_cost(right_fk_solver, q_cur_r+delta_q_arm, _measurement.r_num_wrist_seg, _measurement.r_num_elbow_seg, _measurement.r_num_shoulder_seg, false, _measurement);
      e_wrist_pos_plus += this->cur_wrist_pos_cost;
      e_wrist_ori_plus += this->cur_wrist_ori_cost;
      e_elbow_pos_plus += this->cur_elbow_pos_cost;

      e_minus = left_tmp;
      e_wrist_pos_minus = wrist_pos_cost_left_tmp;
      e_wrist_ori_minus = wrist_ori_cost_left_tmp;
      e_elbow_pos_minus = elbow_pos_cost_left_tmp;       
      e_minus += compute_arm_cost(right_fk_solver, q_cur_r-delta_q_arm, _measurement.r_num_wrist_seg, _measurement.r_num_elbow_seg, _measurement.r_num_shoulder_seg, false, _measurement);
      e_wrist_pos_minus += this->cur_wrist_pos_cost;
      e_wrist_ori_minus += this->cur_wrist_ori_cost;
      e_elbow_pos_minus += this->cur_elbow_pos_cost;      

      _jacobianOplus[n+1](0, d+7) = (e_plus - e_minus) / (2 * q_arm_eps);      
      wrist_pos_jacobian_for_q_arm(d+7, n) = K_WRIST_POS * (e_wrist_pos_plus - e_wrist_pos_minus) / ( 2 * q_arm_eps);
      wrist_ori_jacobian_for_q_arm(d+7, n) = K_WRIST_ORI * (e_wrist_ori_plus - e_wrist_ori_minus) / ( 2 * q_arm_eps);
      elbow_pos_jacobian_for_q_arm(d+7, n) = K_ELBOW_POS * (e_elbow_pos_plus - e_elbow_pos_minus) / ( 2 * q_arm_eps);

      // reset delta
      delta_q_arm[d] = 0.0;
    }
    //std::cout << "debug: jacobians w.r.t left arm joints of datapoint " << (n+1) << "/" << num_datapoints 
    //          << " = " << _jacobianOplus[n+1].block(0, 0, 1, 7) << std::endl;
    //std::cout << "debug: jacobians w.r.t right arm joints of datapoint " << (n+1) << "/" << num_datapoints 
    //          << " = " << _jacobianOplus[n+1].block(0, 7, 1, 7) << std::endl;


    // (2) - jacobians for fingers
    left_tmp = compute_finger_cost(q_cur_finger_l, true, _measurement);  
    right_tmp = compute_finger_cost(q_cur_finger_r, false, _measurement); 
    for (unsigned int d = 0; d < 12; d++)
    {
      // set delta
      delta_q_finger[d] = q_finger_eps;

      // left hand
      e_plus = right_tmp + compute_finger_cost(q_cur_finger_l+delta_q_finger, true, _measurement);
      e_minus = right_tmp + compute_finger_cost(q_cur_finger_l-delta_q_finger, true, _measurement);
      _jacobianOplus[n+1](0, d+14) = K_FINGER * (e_plus - e_minus) / (2*q_finger_eps);

      // right hand
      e_plus = left_tmp + compute_finger_cost(q_cur_finger_r+delta_q_finger, false, _measurement);  
      e_minus = left_tmp + compute_finger_cost(q_cur_finger_r-delta_q_finger, false, _measurement);  
      _jacobianOplus[n+1](0, d+26) = K_FINGER * (e_plus - e_minus) / (2*q_finger_eps);

      // reset delta
      delta_q_finger[d] = 0.0;
    }
    //std::cout << "debug: jacobians w.r.t left finger joints of datapoint " << (n+1) << "/" << num_datapoints 
    //            << " = " << _jacobianOplus[n+1].block(0, 14, 1, 12) << std::endl;
    //std::cout << "debug: jacobians w.r.t right finger joints of datapoint " << (n+1) << "/" << num_datapoints 
    //          << " = " << _jacobianOplus[n+1].block(0, 26, 1, 12) << std::endl;

  }


  // 3 - Save jacobians for q vertices
  for (unsigned int n = 0; n < num_datapoints; n++)
  {
    for (unsigned int d = 0; d < JOINT_DOF; d++)
      jacobians_for_q(n, d) = _jacobianOplus[n+1](0, d); // starts from 1
  }



}

double TrackingConstraint::return_wrist_pos_cost(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, unsigned int num_elbow_seg, unsigned int num_shoulder_seg, bool left_or_right, my_constraint_struct &fdata)
{
      // Get joint angles
      KDL::JntArray q_in(q_cur.size()); 
      for (unsigned int i = 0; i < q_cur.size(); ++i)
      {
        q_in(i) = q_cur(i);
      }

      // Do FK using KDL, get the current elbow/wrist/shoulder state
      KDL::Frame wrist_cart_out; // Output homogeneous transformation
      int result;
      result = fk_solver.JntToCart(q_in, wrist_cart_out, num_wrist_seg+1);
      if (result < 0){
        ROS_INFO_STREAM("FK solver failed when processing wrist link, something went wrong");
        exit(-1);
      }
      else{
        //ROS_INFO_STREAM("FK solver succeeded for wrist link.");
      }

      // Preparations
      Vector3d wrist_pos_cur = Map<Vector3d>(wrist_cart_out.p.data, 3, 1);

      Vector3d wrist_pos_human;
      if (left_or_right) // left arm
      {
        wrist_pos_human = fdata.l_wrist_pos_goal;

        fdata.l_wrist_cur = wrist_pos_cur;
      }
      else // right arm
      {
        wrist_pos_human = fdata.r_wrist_pos_goal;

        fdata.r_wrist_cur = wrist_pos_cur;

      }


  // Compute cost function
  double wrist_pos_cost = (wrist_pos_cur - wrist_pos_human).norm(); // _human is actually the newly generated trajectory

  // Return cost function value
  return wrist_pos_cost;

}


Vector3d TrackingConstraint::return_wrist_pos_offset(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, unsigned int num_elbow_seg, unsigned int num_shoulder_seg, bool left_or_right, my_constraint_struct &fdata)
{
      // Get joint angles
      KDL::JntArray q_in(q_cur.size()); 
      for (unsigned int i = 0; i < q_cur.size(); ++i)
      {
        q_in(i) = q_cur(i);
      }

      // Do FK using KDL, get the current elbow/wrist/shoulder state
      KDL::Frame wrist_cart_out; // Output homogeneous transformation
      int result;
      result = fk_solver.JntToCart(q_in, wrist_cart_out, num_wrist_seg+1);
      if (result < 0){
        ROS_INFO_STREAM("FK solver failed when processing wrist link, something went wrong");
        exit(-1);
      }
      else{
        //ROS_INFO_STREAM("FK solver succeeded for wrist link.");
      }

      // Preparations
      Vector3d wrist_pos_cur = Map<Vector3d>(wrist_cart_out.p.data, 3, 1);

      Vector3d wrist_pos_human;
      if (left_or_right) // left arm
      {
        wrist_pos_human = fdata.l_wrist_pos_goal;

        fdata.l_wrist_cur = wrist_pos_cur;
      }
      else // right arm
      {
        wrist_pos_human = fdata.r_wrist_pos_goal;

        fdata.r_wrist_cur = wrist_pos_cur;

      }


  // Compute cost function
  Vector3d wrist_pos_offset = wrist_pos_cur - wrist_pos_human; 

  // Return cost function value
  return wrist_pos_offset;

}


Vector3d TrackingConstraint::return_elbow_pos_offset(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, unsigned int num_elbow_seg, unsigned int num_shoulder_seg, bool left_or_right, my_constraint_struct &fdata)
{
      // Get joint angles
      KDL::JntArray q_in(q_cur.size()); 
      for (unsigned int i = 0; i < q_cur.size(); ++i)
      {
        q_in(i) = q_cur(i);
      }

      // Do FK using KDL, get the current elbow/wrist/shoulder state
      KDL::Frame elbow_cart_out; // Output homogeneous transformation
      int result;
      result = fk_solver.JntToCart(q_in, elbow_cart_out, num_elbow_seg+1);
      if (result < 0){
        ROS_INFO_STREAM("FK solver failed when processing elbow link, something went wrong");
        exit(-1);
      }
      else{
        //ROS_INFO_STREAM("FK solver succeeded for wrist link.");
      }

      // Preparations
      Vector3d elbow_pos_cur = Map<Vector3d>(elbow_cart_out.p.data, 3, 1);

      Vector3d elbow_pos_human;
      if (left_or_right) // left arm
      {
        elbow_pos_human = fdata.l_elbow_pos_goal;
      }
      else // right arm
      {
        elbow_pos_human = fdata.r_elbow_pos_goal;
      }

  // Compute cost function
  Vector3d elbow_pos_offset = elbow_pos_cur - elbow_pos_human; 

  // Return cost function value
  return elbow_pos_offset;


}


double TrackingConstraint::return_wrist_ori_cost(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, unsigned int num_elbow_seg, unsigned int num_shoulder_seg, bool left_or_right, my_constraint_struct &fdata)
{
      // Get joint angles
      KDL::JntArray q_in(q_cur.size()); 
      for (unsigned int i = 0; i < q_cur.size(); ++i)
      {
        q_in(i) = q_cur(i);
      }

      // Do FK using KDL, get the current elbow/wrist/shoulder state
      KDL::Frame wrist_cart_out, shoulder_cart_out; // Output homogeneous transformation
      int result;
      result = fk_solver.JntToCart(q_in, wrist_cart_out, num_wrist_seg+1);
      if (result < 0){
        ROS_INFO_STREAM("FK solver failed when processing wrist link, something went wrong");
        exit(-1);
      }
      else{
        //ROS_INFO_STREAM("FK solver succeeded for wrist link.");
      }

      // Preparations
      Matrix3d wrist_ori_cur = Map<Matrix<double, 3, 3, RowMajor> >(wrist_cart_out.M.data, 3, 3); 

      Matrix3d wrist_ori_human;
      if (left_or_right) // left arm
      {
        wrist_ori_human = fdata.l_wrist_ori_goal;
      }
      else // right arm
      {
        wrist_ori_human = fdata.r_wrist_ori_goal;
      }


  // Compute cost function
  double wrist_ori_cost = std::fabs( std::acos (( (wrist_ori_human * wrist_ori_cur.transpose()).trace() - 1.0) / 2.0));

  // Return cost function value
  return wrist_ori_cost;

}

double TrackingConstraint::return_elbow_pos_cost(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, unsigned int num_elbow_seg, unsigned int num_shoulder_seg, bool left_or_right, my_constraint_struct &fdata)
{
      // Get joint angles
      KDL::JntArray q_in(q_cur.size()); 
      for (unsigned int i = 0; i < q_cur.size(); ++i)
      {
        q_in(i) = q_cur(i);
      }

      // Do FK using KDL, get the current elbow/wrist/shoulder state
      KDL::Frame elbow_cart_out; // Output homogeneous transformation
      int result;
      result = fk_solver.JntToCart(q_in, elbow_cart_out, num_elbow_seg+1); // notice that the number here is not the segment ID, but the number of segments till target segment
      if (result < 0){
        ROS_INFO_STREAM("FK solver failed when processing elbow link, something went wrong");
        exit(-1);
      }
      else{
        //ROS_INFO_STREAM("FK solver succeeded for elbow link.");
      }
     

      // Preparations
      Vector3d elbow_pos_cur = Map<Vector3d>(elbow_cart_out.p.data, 3, 1);

      Vector3d elbow_pos_human;
      if (left_or_right) // left arm
      {
        elbow_pos_human = fdata.l_elbow_pos_goal;
      }
      else // right arm
      {
        elbow_pos_human = fdata.r_elbow_pos_goal;
      }


  // Compute cost function
  double elbow_pos_cost = (elbow_pos_cur - elbow_pos_human).norm();

  // Return cost function value
  return elbow_pos_cost;

}

double TrackingConstraint::compute_arm_cost(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, unsigned int num_elbow_seg, unsigned int num_shoulder_seg, bool left_or_right, my_constraint_struct &fdata)
{
  // Way 1: KDL FK solver
  std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();  
  // Get joint angles
  KDL::JntArray q_in(q_cur.size()); 
  for (unsigned int i = 0; i < q_cur.size(); ++i)
  {
    q_in(i) = q_cur(i);
  }

  // Do FK using KDL, get the current elbow/wrist/shoulder state
  KDL::Frame elbow_cart_out, wrist_cart_out;//, shoulder_cart_out; // Output homogeneous transformation
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
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();  
  std::chrono::duration<double> t_spent_10 = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  std::cout << "compute_arm_cost: KDL solver spent " << t_spent_10.count() << " s." << std::endl;


  // Way 2: RobotState
  /*
  std::chrono::steady_clock::time_point t00 = std::chrono::steady_clock::now();  
  // select links
  std::string elbow_link_name = (left_or_right ? this->LEFT_ELBOW_LINK : this->RIGHT_ELBOW_LINK);
  std::string wrist_link_name = (left_or_right ? this->LEFT_WRIST_LINK : this->RIGHT_WRIST_LINK);
  // get joint values
  std::vector<double> q_current(JOINT_DOF);
  unsigned int d = (left_or_right ? 0 : q_cur.size()); // offset to cope with left and right arm joints
  for (unsigned int s = 0; s < q_cur.size(); s++)
    q_current[s+d] = q_cur[s];
  std::cout << "q_current = ";
  for (unsigned int s = 0; s < JOINT_DOF; s++)
    std::cout << q_current[s] << " ";
  std::cout << std::endl;  
  // compute pos and ori for elbow and wrist
  dual_arm_dual_hand_collision_ptr->set_joint_values_yumi(q_current);
  Eigen::Vector3d elbow_pos_tmp = this->dual_arm_dual_hand_collision_ptr->get_link_pos(q_current, elbow_link_name);
  Eigen::Vector3d wrist_pos_tmp = this->dual_arm_dual_hand_collision_ptr->get_link_pos(q_current, wrist_link_name);
  Eigen::Matrix3d elbow_ori_tmp = this->dual_arm_dual_hand_collision_ptr->get_link_ori(q_current, elbow_link_name);
  Eigen::Matrix3d wrist_ori_tmp = this->dual_arm_dual_hand_collision_ptr->get_link_ori(q_current, wrist_link_name);

  std::chrono::steady_clock::time_point t11 = std::chrono::steady_clock::now();  
  std::chrono::duration<double> t_spent_1100 = std::chrono::duration_cast<std::chrono::duration<double>>(t11 - t00);
  std::cout << "compute_arm_cost: RobotState spent " << t_spent_1100.count() << " s." << std::endl;
  */

  // Get the results
  Vector3d elbow_pos_cur = Map<Vector3d>(elbow_cart_out.p.data, 3, 1);
  Vector3d wrist_pos_cur = Map<Vector3d>(wrist_cart_out.p.data, 3, 1);
  Matrix3d wrist_ori_cur = Map<Matrix<double, 3, 3, RowMajor> >(wrist_cart_out.M.data, 3, 3); 
  //Vector3d shoulder_pos_cur = Map<Vector3d>(shoulder_cart_out.p.data, 3, 1);

  // Results comparison:
  /*
  std::cout << "Compare Results: " << std::endl;
  std::cout << "1 - KDL: wrist_pos = " << wrist_pos_cur.transpose() << ", elbow_pos = " << elbow_pos_cur.transpose()
            << ", wrist_ori = " << wrist_ori_cur << std::endl;
  std::cout << "2 - RobotState: wrist_pos = " << wrist_pos_tmp.transpose() << ", elbow_pos = " << elbow_pos_tmp.transpose()
            << ", wrist_ori = " << wrist_ori_tmp << std::endl;
  */

  // Specify human data
  Vector3d elbow_pos_human, wrist_pos_human; //,shoulder_pos_human
  Matrix3d wrist_ori_human;
  if (left_or_right) // left arm
  {
    //shoulder_pos_human = fdata.l_shoulder_pos_goal;
    elbow_pos_human = fdata.l_elbow_pos_goal;
    wrist_pos_human = fdata.l_wrist_pos_goal;
    wrist_ori_human = fdata.l_wrist_ori_goal;

    fdata.l_wrist_cur = wrist_pos_cur;
  }
  else // right arm
  {
    //shoulder_pos_human = fdata.r_shoulder_pos_goal;
    elbow_pos_human = fdata.r_elbow_pos_goal;
    wrist_pos_human = fdata.r_wrist_pos_goal;
    wrist_ori_human = fdata.r_wrist_ori_goal;

    fdata.r_wrist_cur = wrist_pos_cur;
  }
  

  // Compute cost function
  double wrist_pos_cost = (wrist_pos_cur - wrist_pos_human).norm(); // _human is actually the newly generated trajectory
  double elbow_pos_cost = (elbow_pos_cur - elbow_pos_human).norm();
  double wrist_ori_cost = std::fabs( std::acos (( (wrist_ori_human * wrist_ori_cur.transpose()).trace() - 1.0) / 2.0));
  double cost = K_WRIST_ORI * wrist_ori_cost + K_WRIST_POS * wrist_pos_cost + K_ELBOW_POS * elbow_pos_cost;

  // store for debugging jacobians
  this->cur_wrist_pos_cost = wrist_pos_cost;
  this->cur_wrist_ori_cost = wrist_ori_cost;
  this->cur_elbow_pos_cost = elbow_pos_cost;

  // Return cost function value
  return cost;

}


double TrackingConstraint::linear_map(double x_, double min_, double max_, double min_hat, double max_hat)
{
  return (x_ - min_) / (max_ - min_) * (max_hat - min_hat) + min_hat;
}


double TrackingConstraint::compute_finger_cost(Matrix<double, 12, 1> q_finger_robot, bool left_or_right, my_constraint_struct &fdata)
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


  // store for debug
  this->cur_finger_pos_cost = finger_cost;


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


std::vector<double> TrackingConstraint::return_finger_cost_history()
{
  
  // iterate to compute costs
  std::vector<double> finger_cost_history;
  for (unsigned int n = 0; n < num_datapoints; n++)
  {
    // get the current joint value
    // _vertices is a VertexContainer type, a std::vector<Vertex*>
    const DualArmDualHandVertex *v = static_cast<const DualArmDualHandVertex*>(_vertices[n+1]);
    const Matrix<double, JOINT_DOF, 1> x = v->estimate(); // return the current estimate of the vertex

    // Get joint angles
    Matrix<double, 7, 1> q_cur_l, q_cur_r;
    Matrix<double, 12, 1> q_cur_finger_l, q_cur_finger_r;
    q_cur_l = x.block<7, 1>(0, 0);
    q_cur_r = x.block<7, 1>(7, 0);
    q_cur_finger_l = x.block<12, 1>(14, 0);
    q_cur_finger_r = x.block<12, 1>(26, 0); 
    
    // Set new goals(expected trajectory) to _measurement
    _measurement.l_finger_pos_goal = trajectory_generator_ptr->l_glove_angle_traj.block(n, 0, 1, 14).transpose() * M_PI / 180.0; // y_seq's glove data is already in radius
    _measurement.r_finger_pos_goal = trajectory_generator_ptr->r_glove_angle_traj.block(n, 0, 1, 14).transpose() * M_PI / 180.0; // size is 50 x DOF

    // Compute unary costs
    double finger_cost = compute_finger_cost(q_cur_finger_l, true, _measurement);  
    finger_cost += compute_finger_cost(q_cur_finger_r, false, _measurement);  
  
    finger_cost_history.push_back(finger_cost);

  }

  return finger_cost_history;

}


std::vector<double> TrackingConstraint::return_wrist_pos_cost_history()
{
  // Get trajectories using DMP
  const DMPStartsGoalsVertex *v = static_cast<const DMPStartsGoalsVertex*>(_vertices[0]); // the last vertex connected
  Matrix<double, DMPPOINTS_DOF, 1> x = v->estimate();
  MatrixXd lrw_new_goal(3, 1); lrw_new_goal = x.block(0, 0, 3, 1);
  MatrixXd lrw_new_start(3, 1); lrw_new_start = x.block(3, 0, 3, 1);
  MatrixXd lew_new_goal(3, 1); lew_new_goal = x.block(6, 0, 3, 1);
  MatrixXd lew_new_start(3, 1); lew_new_start = x.block(9, 0, 3, 1);
  MatrixXd rew_new_goal(3, 1); rew_new_goal = x.block(12, 0, 3, 1);
  MatrixXd rew_new_start(3, 1); rew_new_start = x.block(15, 0, 3, 1);
  MatrixXd rw_new_goal(3, 1); rw_new_goal = x.block(18, 0, 3, 1);
  MatrixXd rw_new_start(3, 1); rw_new_start = x.block(21, 0, 3, 1);
  DMP_trajs result = trajectory_generator_ptr->generate_trajectories(lrw_new_goal.transpose(), lrw_new_start.transpose(), // should be row vectors
                                                                      lew_new_goal.transpose(), lew_new_start.transpose(),
                                                                      rew_new_goal.transpose(), rew_new_start.transpose(),
                                                                      rw_new_goal.transpose(), rw_new_start.transpose(),
                                                                      NUM_DATAPOINTS); // results are 3 x N

  // iterate to compute costs
  std::vector<double> wrist_pos_costs;
  for (unsigned int n = 0; n < num_datapoints; n++)
  {
    // get the current joint value
    const DualArmDualHandVertex *v = static_cast<const DualArmDualHandVertex*>(_vertices[n+1]);
    const Matrix<double, JOINT_DOF, 1> x = v->estimate(); // return the current estimate of the vertex

    // Get joint angles
    Matrix<double, 7, 1> q_cur_l, q_cur_r;
    Matrix<double, 12, 1> q_cur_finger_l, q_cur_finger_r;
    q_cur_l = x.block<7, 1>(0, 0);
    q_cur_r = x.block<7, 1>(7, 0);
    q_cur_finger_l = x.block<12, 1>(14, 0);
    q_cur_finger_r = x.block<12, 1>(26, 0); 
    
    // Set new goals(expected trajectory) to _measurement
    _measurement.l_wrist_pos_goal = result.y_lw.block(0, n, 3, 1); // Vector3d
    _measurement.r_wrist_pos_goal = result.y_rw.block(0, n, 3, 1); // Vector3d

    // Compute unary costs
    double wrist_pos_cost = return_wrist_pos_cost(left_fk_solver, q_cur_l, _measurement.l_num_wrist_seg, _measurement.l_num_elbow_seg, _measurement.l_num_shoulder_seg, true, _measurement); // user data is stored in _measurement now
    wrist_pos_cost += return_wrist_pos_cost(right_fk_solver, q_cur_r, _measurement.r_num_wrist_seg, _measurement.r_num_elbow_seg, _measurement.r_num_shoulder_seg, false, _measurement);

    wrist_pos_costs.push_back(wrist_pos_cost);

  }

  return wrist_pos_costs;

}

/* right wrist cost */
std::vector<double> TrackingConstraint::return_r_wrist_pos_cost_history()
{
  // Get trajectories using DMP
  const DMPStartsGoalsVertex *v = static_cast<const DMPStartsGoalsVertex*>(_vertices[0]); // the last vertex connected
  Matrix<double, DMPPOINTS_DOF, 1> x = v->estimate();
  MatrixXd lrw_new_goal(3, 1); lrw_new_goal = x.block(0, 0, 3, 1);
  MatrixXd lrw_new_start(3, 1); lrw_new_start = x.block(3, 0, 3, 1);
  MatrixXd lew_new_goal(3, 1); lew_new_goal = x.block(6, 0, 3, 1);
  MatrixXd lew_new_start(3, 1); lew_new_start = x.block(9, 0, 3, 1);
  MatrixXd rew_new_goal(3, 1); rew_new_goal = x.block(12, 0, 3, 1);
  MatrixXd rew_new_start(3, 1); rew_new_start = x.block(15, 0, 3, 1);
  MatrixXd rw_new_goal(3, 1); rw_new_goal = x.block(18, 0, 3, 1);
  MatrixXd rw_new_start(3, 1); rw_new_start = x.block(21, 0, 3, 1);
  DMP_trajs result = trajectory_generator_ptr->generate_trajectories(lrw_new_goal.transpose(), lrw_new_start.transpose(), // should be row vectors
                                                                      lew_new_goal.transpose(), lew_new_start.transpose(),
                                                                      rew_new_goal.transpose(), rew_new_start.transpose(),
                                                                      rw_new_goal.transpose(), rw_new_start.transpose(),
                                                                      NUM_DATAPOINTS); // results are 3 x N

  // iterate to compute costs
  std::vector<double> wrist_pos_costs;
  for (unsigned int n = 0; n < num_datapoints; n++)
  {
    // get the current joint value
    const DualArmDualHandVertex *v = static_cast<const DualArmDualHandVertex*>(_vertices[n+1]);
    const Matrix<double, JOINT_DOF, 1> x = v->estimate(); // return the current estimate of the vertex

    // Get joint angles
    Matrix<double, 7, 1> q_cur_l, q_cur_r;
    Matrix<double, 12, 1> q_cur_finger_l, q_cur_finger_r;
    q_cur_l = x.block<7, 1>(0, 0);
    q_cur_r = x.block<7, 1>(7, 0);
    q_cur_finger_l = x.block<12, 1>(14, 0);
    q_cur_finger_r = x.block<12, 1>(26, 0); 
    
    // Set new goals(expected trajectory) to _measurement
    _measurement.l_wrist_pos_goal = result.y_lw.block(0, n, 3, 1); // Vector3d
    _measurement.r_wrist_pos_goal = result.y_rw.block(0, n, 3, 1); // Vector3d

    // Compute unary costs
    double wrist_pos_cost = return_wrist_pos_cost(right_fk_solver, q_cur_r, _measurement.r_num_wrist_seg, _measurement.r_num_elbow_seg, _measurement.r_num_shoulder_seg, false, _measurement);

    wrist_pos_costs.push_back(wrist_pos_cost);

  }

  return wrist_pos_costs;

}

/* left wrist cost */
std::vector<double> TrackingConstraint::return_l_wrist_pos_cost_history()
{
  // Get trajectories using DMP
  const DMPStartsGoalsVertex *v = static_cast<const DMPStartsGoalsVertex*>(_vertices[0]); // the last vertex connected
  Matrix<double, DMPPOINTS_DOF, 1> x = v->estimate();
  MatrixXd lrw_new_goal(3, 1); lrw_new_goal = x.block(0, 0, 3, 1);
  MatrixXd lrw_new_start(3, 1); lrw_new_start = x.block(3, 0, 3, 1);
  MatrixXd lew_new_goal(3, 1); lew_new_goal = x.block(6, 0, 3, 1);
  MatrixXd lew_new_start(3, 1); lew_new_start = x.block(9, 0, 3, 1);
  MatrixXd rew_new_goal(3, 1); rew_new_goal = x.block(12, 0, 3, 1);
  MatrixXd rew_new_start(3, 1); rew_new_start = x.block(15, 0, 3, 1);
  MatrixXd rw_new_goal(3, 1); rw_new_goal = x.block(18, 0, 3, 1);
  MatrixXd rw_new_start(3, 1); rw_new_start = x.block(21, 0, 3, 1);
  DMP_trajs result = trajectory_generator_ptr->generate_trajectories(lrw_new_goal.transpose(), lrw_new_start.transpose(), // should be row vectors
                                                                      lew_new_goal.transpose(), lew_new_start.transpose(),
                                                                      rew_new_goal.transpose(), rew_new_start.transpose(),
                                                                      rw_new_goal.transpose(), rw_new_start.transpose(),
                                                                      NUM_DATAPOINTS); // results are 3 x N

  // iterate to compute costs
  std::vector<double> wrist_pos_costs;
  for (unsigned int n = 0; n < num_datapoints; n++)
  {
    // get the current joint value
    const DualArmDualHandVertex *v = static_cast<const DualArmDualHandVertex*>(_vertices[n+1]);
    const Matrix<double, JOINT_DOF, 1> x = v->estimate(); // return the current estimate of the vertex

    // Get joint angles
    Matrix<double, 7, 1> q_cur_l, q_cur_r;
    Matrix<double, 12, 1> q_cur_finger_l, q_cur_finger_r;
    q_cur_l = x.block<7, 1>(0, 0);
    q_cur_r = x.block<7, 1>(7, 0);
    q_cur_finger_l = x.block<12, 1>(14, 0);
    q_cur_finger_r = x.block<12, 1>(26, 0); 
    
    // Set new goals(expected trajectory) to _measurement
    _measurement.l_wrist_pos_goal = result.y_lw.block(0, n, 3, 1); // Vector3d
    _measurement.r_wrist_pos_goal = result.y_rw.block(0, n, 3, 1); // Vector3d

    // Compute unary costs
    double wrist_pos_cost = return_wrist_pos_cost(left_fk_solver, q_cur_l, _measurement.l_num_wrist_seg, _measurement.l_num_elbow_seg, _measurement.l_num_shoulder_seg, true, _measurement); // user data is stored in _measurement now

    wrist_pos_costs.push_back(wrist_pos_cost);

  }

  return wrist_pos_costs;

}


/* Only the right wrist */
MatrixXd TrackingConstraint::return_wrist_pos_offsets()
{
  // Get trajectories using DMP
  const DMPStartsGoalsVertex *v = static_cast<const DMPStartsGoalsVertex*>(_vertices[0]); // the last vertex connected
  Matrix<double, DMPPOINTS_DOF, 1> x = v->estimate();
  MatrixXd lrw_new_goal(3, 1); lrw_new_goal = x.block(0, 0, 3, 1);
  MatrixXd lrw_new_start(3, 1); lrw_new_start = x.block(3, 0, 3, 1);
  MatrixXd lew_new_goal(3, 1); lew_new_goal = x.block(6, 0, 3, 1);
  MatrixXd lew_new_start(3, 1); lew_new_start = x.block(9, 0, 3, 1);
  MatrixXd rew_new_goal(3, 1); rew_new_goal = x.block(12, 0, 3, 1);
  MatrixXd rew_new_start(3, 1); rew_new_start = x.block(15, 0, 3, 1);
  MatrixXd rw_new_goal(3, 1); rw_new_goal = x.block(18, 0, 3, 1);
  MatrixXd rw_new_start(3, 1); rw_new_start = x.block(21, 0, 3, 1);
  DMP_trajs result = trajectory_generator_ptr->generate_trajectories(lrw_new_goal.transpose(), lrw_new_start.transpose(), // should be row vectors
                                                                      lew_new_goal.transpose(), lew_new_start.transpose(),
                                                                      rew_new_goal.transpose(), rew_new_start.transpose(),
                                                                      rw_new_goal.transpose(), rw_new_start.transpose(),
                                                                      NUM_DATAPOINTS); // results are 3 x N

  // iterate to compute costs
  std::vector<double> wrist_pos_costs;
  MatrixXd wrist_pos_offsets(3, num_datapoints);
  for (unsigned int n = 0; n < num_datapoints; n++)
  {
    // get the current joint value
    const DualArmDualHandVertex *v = static_cast<const DualArmDualHandVertex*>(_vertices[n+1]);
    const Matrix<double, JOINT_DOF, 1> x = v->estimate(); // return the current estimate of the vertex

    // Get joint angles
    Matrix<double, 7, 1> q_cur_l, q_cur_r;
    Matrix<double, 12, 1> q_cur_finger_l, q_cur_finger_r;
    q_cur_l = x.block<7, 1>(0, 0);
    q_cur_r = x.block<7, 1>(7, 0);
    q_cur_finger_l = x.block<12, 1>(14, 0);
    q_cur_finger_r = x.block<12, 1>(26, 0); 
    
    // Set new goals(expected trajectory) to _measurement
    _measurement.l_wrist_pos_goal = result.y_lw.block(0, n, 3, 1); // Vector3d
    _measurement.r_wrist_pos_goal = result.y_rw.block(0, n, 3, 1); // Vector3d

    // Compute unary costs
    // only the right wrist
    Vector3d wrist_pos_offset = return_wrist_pos_offset(right_fk_solver, q_cur_r, _measurement.r_num_wrist_seg, _measurement.r_num_elbow_seg, _measurement.r_num_shoulder_seg, false, _measurement);

    wrist_pos_offsets.block(0, n, 3, 1) = wrist_pos_offset;

  }

  return wrist_pos_offsets;

}


/*** RIGHT WRIST OFFSET ***/
MatrixXd TrackingConstraint::return_r_wrist_pos_offsets()
{
  // Get trajectories using DMP
  const DMPStartsGoalsVertex *v = static_cast<const DMPStartsGoalsVertex*>(_vertices[0]); // the last vertex connected
  Matrix<double, DMPPOINTS_DOF, 1> x = v->estimate();
  MatrixXd lrw_new_goal(3, 1); lrw_new_goal = x.block(0, 0, 3, 1);
  MatrixXd lrw_new_start(3, 1); lrw_new_start = x.block(3, 0, 3, 1);
  MatrixXd lew_new_goal(3, 1); lew_new_goal = x.block(6, 0, 3, 1);
  MatrixXd lew_new_start(3, 1); lew_new_start = x.block(9, 0, 3, 1);
  MatrixXd rew_new_goal(3, 1); rew_new_goal = x.block(12, 0, 3, 1);
  MatrixXd rew_new_start(3, 1); rew_new_start = x.block(15, 0, 3, 1);
  MatrixXd rw_new_goal(3, 1); rw_new_goal = x.block(18, 0, 3, 1);
  MatrixXd rw_new_start(3, 1); rw_new_start = x.block(21, 0, 3, 1);
  DMP_trajs result = trajectory_generator_ptr->generate_trajectories(lrw_new_goal.transpose(), lrw_new_start.transpose(), // should be row vectors
                                                                      lew_new_goal.transpose(), lew_new_start.transpose(),
                                                                      rew_new_goal.transpose(), rew_new_start.transpose(),
                                                                      rw_new_goal.transpose(), rw_new_start.transpose(),
                                                                      NUM_DATAPOINTS); // results are 3 x N

  // iterate to compute costs
  std::vector<double> wrist_pos_costs;
  MatrixXd wrist_pos_offsets(3, num_datapoints);
  for (unsigned int n = 0; n < num_datapoints; n++)
  {
    // get the current joint value
    const DualArmDualHandVertex *v = static_cast<const DualArmDualHandVertex*>(_vertices[n+1]);
    const Matrix<double, JOINT_DOF, 1> x = v->estimate(); // return the current estimate of the vertex

    // Get joint angles
    Matrix<double, 7, 1> q_cur_l, q_cur_r;
    Matrix<double, 12, 1> q_cur_finger_l, q_cur_finger_r;
    q_cur_l = x.block<7, 1>(0, 0);
    q_cur_r = x.block<7, 1>(7, 0);
    q_cur_finger_l = x.block<12, 1>(14, 0);
    q_cur_finger_r = x.block<12, 1>(26, 0); 
    
    // Set new goals(expected trajectory) to _measurement
    _measurement.l_wrist_pos_goal = result.y_lw.block(0, n, 3, 1); // Vector3d
    _measurement.r_wrist_pos_goal = result.y_rw.block(0, n, 3, 1); // Vector3d

    // Compute unary costs
    // only the right wrist
    Vector3d wrist_pos_offset = return_wrist_pos_offset(right_fk_solver, q_cur_r, _measurement.r_num_wrist_seg, _measurement.r_num_elbow_seg, _measurement.r_num_shoulder_seg, false, _measurement);

    wrist_pos_offsets.block(0, n, 3, 1) = wrist_pos_offset;

  }

  return wrist_pos_offsets;

}


/*** LEFT WRIST OFFSET ***/
MatrixXd TrackingConstraint::return_l_wrist_pos_offsets()
{
  // Get trajectories using DMP
  const DMPStartsGoalsVertex *v = static_cast<const DMPStartsGoalsVertex*>(_vertices[0]); // the last vertex connected
  Matrix<double, DMPPOINTS_DOF, 1> x = v->estimate();
  MatrixXd lrw_new_goal(3, 1); lrw_new_goal = x.block(0, 0, 3, 1);
  MatrixXd lrw_new_start(3, 1); lrw_new_start = x.block(3, 0, 3, 1);
  MatrixXd lew_new_goal(3, 1); lew_new_goal = x.block(6, 0, 3, 1);
  MatrixXd lew_new_start(3, 1); lew_new_start = x.block(9, 0, 3, 1);
  MatrixXd rew_new_goal(3, 1); rew_new_goal = x.block(12, 0, 3, 1);
  MatrixXd rew_new_start(3, 1); rew_new_start = x.block(15, 0, 3, 1);
  MatrixXd rw_new_goal(3, 1); rw_new_goal = x.block(18, 0, 3, 1);
  MatrixXd rw_new_start(3, 1); rw_new_start = x.block(21, 0, 3, 1);
  DMP_trajs result = trajectory_generator_ptr->generate_trajectories(lrw_new_goal.transpose(), lrw_new_start.transpose(), // should be row vectors
                                                                      lew_new_goal.transpose(), lew_new_start.transpose(),
                                                                      rew_new_goal.transpose(), rew_new_start.transpose(),
                                                                      rw_new_goal.transpose(), rw_new_start.transpose(),
                                                                      NUM_DATAPOINTS); // results are 3 x N

  // iterate to compute costs
  std::vector<double> wrist_pos_costs;
  MatrixXd wrist_pos_offsets(3, num_datapoints);
  for (unsigned int n = 0; n < num_datapoints; n++)
  {
    // get the current joint value
    const DualArmDualHandVertex *v = static_cast<const DualArmDualHandVertex*>(_vertices[n+1]);
    const Matrix<double, JOINT_DOF, 1> x = v->estimate(); // return the current estimate of the vertex

    // Get joint angles
    Matrix<double, 7, 1> q_cur_l, q_cur_r;
    Matrix<double, 12, 1> q_cur_finger_l, q_cur_finger_r;
    q_cur_l = x.block<7, 1>(0, 0);
    q_cur_r = x.block<7, 1>(7, 0);
    q_cur_finger_l = x.block<12, 1>(14, 0);
    q_cur_finger_r = x.block<12, 1>(26, 0); 
    
    // Set new goals(expected trajectory) to _measurement
    _measurement.l_wrist_pos_goal = result.y_lw.block(0, n, 3, 1); // Vector3d
    _measurement.r_wrist_pos_goal = result.y_rw.block(0, n, 3, 1); // Vector3d

    // Compute unary costs
    // only the right wrist
    Vector3d wrist_pos_offset = return_wrist_pos_offset(left_fk_solver, q_cur_l, _measurement.l_num_wrist_seg, _measurement.l_num_elbow_seg, _measurement.l_num_shoulder_seg, true, _measurement);

    wrist_pos_offsets.block(0, n, 3, 1) = wrist_pos_offset;

  }

  return wrist_pos_offsets;

}


/*** RIGHT ELBOW OFFSET ***/
MatrixXd TrackingConstraint::return_r_elbow_pos_offsets()
{
  // Get trajectories using DMP
  const DMPStartsGoalsVertex *v = static_cast<const DMPStartsGoalsVertex*>(_vertices[0]); // the last vertex connected
  Matrix<double, DMPPOINTS_DOF, 1> x = v->estimate();
  MatrixXd lrw_new_goal(3, 1); lrw_new_goal = x.block(0, 0, 3, 1);
  MatrixXd lrw_new_start(3, 1); lrw_new_start = x.block(3, 0, 3, 1);
  MatrixXd lew_new_goal(3, 1); lew_new_goal = x.block(6, 0, 3, 1);
  MatrixXd lew_new_start(3, 1); lew_new_start = x.block(9, 0, 3, 1);
  MatrixXd rew_new_goal(3, 1); rew_new_goal = x.block(12, 0, 3, 1);
  MatrixXd rew_new_start(3, 1); rew_new_start = x.block(15, 0, 3, 1);
  MatrixXd rw_new_goal(3, 1); rw_new_goal = x.block(18, 0, 3, 1);
  MatrixXd rw_new_start(3, 1); rw_new_start = x.block(21, 0, 3, 1);
  DMP_trajs result = trajectory_generator_ptr->generate_trajectories(lrw_new_goal.transpose(), lrw_new_start.transpose(), // should be row vectors
                                                                      lew_new_goal.transpose(), lew_new_start.transpose(),
                                                                      rew_new_goal.transpose(), rew_new_start.transpose(),
                                                                      rw_new_goal.transpose(), rw_new_start.transpose(),
                                                                      NUM_DATAPOINTS); // results are 3 x N

  // iterate to compute costs
  std::vector<double> elbow_pos_costs;
  MatrixXd elbow_pos_offsets(3, num_datapoints);
  for (unsigned int n = 0; n < num_datapoints; n++)
  {
    // get the current joint value
    const DualArmDualHandVertex *v = static_cast<const DualArmDualHandVertex*>(_vertices[n+1]);
    const Matrix<double, JOINT_DOF, 1> x = v->estimate(); // return the current estimate of the vertex

    // Get joint angles
    Matrix<double, 7, 1> q_cur_l, q_cur_r;
    Matrix<double, 12, 1> q_cur_finger_l, q_cur_finger_r;
    q_cur_l = x.block<7, 1>(0, 0);
    q_cur_r = x.block<7, 1>(7, 0);
    q_cur_finger_l = x.block<12, 1>(14, 0);
    q_cur_finger_r = x.block<12, 1>(26, 0); 
    
    // Set new goals(expected trajectory) to _measurement
    _measurement.l_elbow_pos_goal = result.y_le.block(0, n, 3, 1); // Vector3d
    _measurement.r_elbow_pos_goal = result.y_re.block(0, n, 3, 1); // Vector3d is column vector


    // Compute unary costs
    // only the right elbow
    Vector3d elbow_pos_offset = return_elbow_pos_offset(right_fk_solver, q_cur_r, _measurement.r_num_wrist_seg, _measurement.r_num_elbow_seg, _measurement.r_num_shoulder_seg, false, _measurement);

    elbow_pos_offsets.block(0, n, 3, 1) = elbow_pos_offset;

  }

  return elbow_pos_offsets;

}


/*** LEFT ELBOW OFFSET ***/
MatrixXd TrackingConstraint::return_l_elbow_pos_offsets()
{
  // Get trajectories using DMP
  const DMPStartsGoalsVertex *v = static_cast<const DMPStartsGoalsVertex*>(_vertices[0]); // the last vertex connected
  Matrix<double, DMPPOINTS_DOF, 1> x = v->estimate();
  MatrixXd lrw_new_goal(3, 1); lrw_new_goal = x.block(0, 0, 3, 1);
  MatrixXd lrw_new_start(3, 1); lrw_new_start = x.block(3, 0, 3, 1);
  MatrixXd lew_new_goal(3, 1); lew_new_goal = x.block(6, 0, 3, 1);
  MatrixXd lew_new_start(3, 1); lew_new_start = x.block(9, 0, 3, 1);
  MatrixXd rew_new_goal(3, 1); rew_new_goal = x.block(12, 0, 3, 1);
  MatrixXd rew_new_start(3, 1); rew_new_start = x.block(15, 0, 3, 1);
  MatrixXd rw_new_goal(3, 1); rw_new_goal = x.block(18, 0, 3, 1);
  MatrixXd rw_new_start(3, 1); rw_new_start = x.block(21, 0, 3, 1);
  DMP_trajs result = trajectory_generator_ptr->generate_trajectories(lrw_new_goal.transpose(), lrw_new_start.transpose(), // should be row vectors
                                                                      lew_new_goal.transpose(), lew_new_start.transpose(),
                                                                      rew_new_goal.transpose(), rew_new_start.transpose(),
                                                                      rw_new_goal.transpose(), rw_new_start.transpose(),
                                                                      NUM_DATAPOINTS); // results are 3 x N

  // iterate to compute costs
  std::vector<double> elbow_pos_costs;
  MatrixXd elbow_pos_offsets(3, num_datapoints);
  for (unsigned int n = 0; n < num_datapoints; n++)
  {
    // get the current joint value
    const DualArmDualHandVertex *v = static_cast<const DualArmDualHandVertex*>(_vertices[n+1]);
    const Matrix<double, JOINT_DOF, 1> x = v->estimate(); // return the current estimate of the vertex

    // Get joint angles
    Matrix<double, 7, 1> q_cur_l, q_cur_r;
    Matrix<double, 12, 1> q_cur_finger_l, q_cur_finger_r;
    q_cur_l = x.block<7, 1>(0, 0);
    q_cur_r = x.block<7, 1>(7, 0);
    q_cur_finger_l = x.block<12, 1>(14, 0);
    q_cur_finger_r = x.block<12, 1>(26, 0); 
    
    // Set new goals(expected trajectory) to _measurement
    _measurement.l_elbow_pos_goal = result.y_le.block(0, n, 3, 1); // Vector3d
    _measurement.r_elbow_pos_goal = result.y_re.block(0, n, 3, 1); // Vector3d is column vector


    // Compute unary costs
    // only the right elbow
    Vector3d elbow_pos_offset = return_elbow_pos_offset(left_fk_solver, q_cur_l, _measurement.l_num_wrist_seg, _measurement.l_num_elbow_seg, _measurement.l_num_shoulder_seg, true, _measurement);

    elbow_pos_offsets.block(0, n, 3, 1) = elbow_pos_offset;

  }

  return elbow_pos_offsets;

}


std::vector<double> TrackingConstraint::return_wrist_ori_cost_history()
{

  // iterate to compute costs
  std::vector<double> wrist_ori_costs;
  for (unsigned int n = 0; n < num_datapoints; n++)
  {
    // get the current joint value
    const DualArmDualHandVertex *v = static_cast<const DualArmDualHandVertex*>(_vertices[n+1]);
    const Matrix<double, JOINT_DOF, 1> x = v->estimate(); // return the current estimate of the vertex

    // Get joint angles
    Matrix<double, 7, 1> q_cur_l, q_cur_r;
    Matrix<double, 12, 1> q_cur_finger_l, q_cur_finger_r;
    q_cur_l = x.block<7, 1>(0, 0);
    q_cur_r = x.block<7, 1>(7, 0);
    q_cur_finger_l = x.block<12, 1>(14, 0);
    q_cur_finger_r = x.block<12, 1>(26, 0); 
    
    // Set new goals(expected trajectory) to _measurement
    Quaterniond q_l(trajectory_generator_ptr->l_wrist_quat_traj(n, 0), 
                    trajectory_generator_ptr->l_wrist_quat_traj(n, 1),
                    trajectory_generator_ptr->l_wrist_quat_traj(n, 2),
                    trajectory_generator_ptr->l_wrist_quat_traj(n, 3));
    Matrix3d l_wrist_ori_goal = q_l.toRotationMatrix();
    _measurement.l_wrist_ori_goal = l_wrist_ori_goal; // Matrix3d
    Quaterniond q_r(trajectory_generator_ptr->r_wrist_quat_traj(n, 0), 
                    trajectory_generator_ptr->r_wrist_quat_traj(n, 1),
                    trajectory_generator_ptr->r_wrist_quat_traj(n, 2),
                    trajectory_generator_ptr->r_wrist_quat_traj(n, 3));
    Matrix3d r_wrist_ori_goal = q_r.toRotationMatrix();  
    _measurement.r_wrist_ori_goal = r_wrist_ori_goal; // Matrix3d


    // Compute unary costs
    double wrist_ori_cost = return_wrist_ori_cost(left_fk_solver, q_cur_l, _measurement.l_num_wrist_seg, _measurement.l_num_elbow_seg, _measurement.l_num_shoulder_seg, true, _measurement); // user data is stored in _measurement now
    wrist_ori_cost += return_wrist_ori_cost(right_fk_solver, q_cur_r, _measurement.r_num_wrist_seg, _measurement.r_num_elbow_seg, _measurement.r_num_shoulder_seg, false, _measurement);

    wrist_ori_costs.push_back(wrist_ori_cost);

  }

  return wrist_ori_costs;

}


std::vector<double> TrackingConstraint::return_elbow_pos_cost_history()
{
  // Generate new trajectories using DMP
  const DMPStartsGoalsVertex *v = static_cast<const DMPStartsGoalsVertex*>(_vertices[0]); // the last vertex connected
  Matrix<double, DMPPOINTS_DOF, 1> x = v->estimate();
  MatrixXd lrw_new_goal(3, 1); lrw_new_goal = x.block(0, 0, 3, 1);
  MatrixXd lrw_new_start(3, 1); lrw_new_start = x.block(3, 0, 3, 1);
  MatrixXd lew_new_goal(3, 1); lew_new_goal = x.block(6, 0, 3, 1);
  MatrixXd lew_new_start(3, 1); lew_new_start = x.block(9, 0, 3, 1);
  MatrixXd rew_new_goal(3, 1); rew_new_goal = x.block(12, 0, 3, 1);
  MatrixXd rew_new_start(3, 1); rew_new_start = x.block(15, 0, 3, 1);
  MatrixXd rw_new_goal(3, 1); rw_new_goal = x.block(18, 0, 3, 1);
  MatrixXd rw_new_start(3, 1); rw_new_start = x.block(21, 0, 3, 1);
  DMP_trajs result = trajectory_generator_ptr->generate_trajectories(lrw_new_goal.transpose(), lrw_new_start.transpose(), // should be row vectors
                                                                      lew_new_goal.transpose(), lew_new_start.transpose(),
                                                                      rew_new_goal.transpose(), rew_new_start.transpose(),
                                                                      rw_new_goal.transpose(), rw_new_start.transpose(),
                                                                      NUM_DATAPOINTS); // results are 3 x N

  // iterate to compute costs
  std::vector<double> elbow_pos_costs;
  for (unsigned int n = 0; n < num_datapoints; n++)
  {
    // get the current joint value
    const DualArmDualHandVertex *v = static_cast<const DualArmDualHandVertex*>(_vertices[n+1]);
    const Matrix<double, JOINT_DOF, 1> x = v->estimate(); // return the current estimate of the vertex

    // Get joint angles
    Matrix<double, 7, 1> q_cur_l, q_cur_r;
    Matrix<double, 12, 1> q_cur_finger_l, q_cur_finger_r;
    q_cur_l = x.block<7, 1>(0, 0);
    q_cur_r = x.block<7, 1>(7, 0);
    q_cur_finger_l = x.block<12, 1>(14, 0);
    q_cur_finger_r = x.block<12, 1>(26, 0); 
    
    // Set new goals(expected trajectory) to _measurement
    _measurement.l_elbow_pos_goal = result.y_le.block(0, n, 3, 1); // Vector3d
    _measurement.r_elbow_pos_goal = result.y_re.block(0, n, 3, 1); // Vector3d is column vector

    // Compute unary costs
    double elbow_pos_cost = return_elbow_pos_cost(left_fk_solver, q_cur_l, _measurement.l_num_wrist_seg, _measurement.l_num_elbow_seg, _measurement.l_num_shoulder_seg, true, _measurement); // user data is stored in _measurement now
    elbow_pos_cost += return_elbow_pos_cost(right_fk_solver, q_cur_r, _measurement.r_num_wrist_seg, _measurement.r_num_elbow_seg, _measurement.r_num_shoulder_seg, false, _measurement);

    elbow_pos_costs.push_back(elbow_pos_cost);

  }

  return elbow_pos_costs;

}


/* right elbow cost */
std::vector<double> TrackingConstraint::return_r_elbow_pos_cost_history()
{
  // Generate new trajectories using DMP
  const DMPStartsGoalsVertex *v = static_cast<const DMPStartsGoalsVertex*>(_vertices[0]); // the last vertex connected
  Matrix<double, DMPPOINTS_DOF, 1> x = v->estimate();
  MatrixXd lrw_new_goal(3, 1); lrw_new_goal = x.block(0, 0, 3, 1);
  MatrixXd lrw_new_start(3, 1); lrw_new_start = x.block(3, 0, 3, 1);
  MatrixXd lew_new_goal(3, 1); lew_new_goal = x.block(6, 0, 3, 1);
  MatrixXd lew_new_start(3, 1); lew_new_start = x.block(9, 0, 3, 1);
  MatrixXd rew_new_goal(3, 1); rew_new_goal = x.block(12, 0, 3, 1);
  MatrixXd rew_new_start(3, 1); rew_new_start = x.block(15, 0, 3, 1);
  MatrixXd rw_new_goal(3, 1); rw_new_goal = x.block(18, 0, 3, 1);
  MatrixXd rw_new_start(3, 1); rw_new_start = x.block(21, 0, 3, 1);
  DMP_trajs result = trajectory_generator_ptr->generate_trajectories(lrw_new_goal.transpose(), lrw_new_start.transpose(), // should be row vectors
                                                                      lew_new_goal.transpose(), lew_new_start.transpose(),
                                                                      rew_new_goal.transpose(), rew_new_start.transpose(),
                                                                      rw_new_goal.transpose(), rw_new_start.transpose(),
                                                                      NUM_DATAPOINTS); // results are 3 x N

  // iterate to compute costs
  std::vector<double> elbow_pos_costs;
  for (unsigned int n = 0; n < num_datapoints; n++)
  {
    // get the current joint value
    const DualArmDualHandVertex *v = static_cast<const DualArmDualHandVertex*>(_vertices[n+1]);
    const Matrix<double, JOINT_DOF, 1> x = v->estimate(); // return the current estimate of the vertex

    // Get joint angles
    Matrix<double, 7, 1> q_cur_l, q_cur_r;
    Matrix<double, 12, 1> q_cur_finger_l, q_cur_finger_r;
    q_cur_l = x.block<7, 1>(0, 0);
    q_cur_r = x.block<7, 1>(7, 0);
    q_cur_finger_l = x.block<12, 1>(14, 0);
    q_cur_finger_r = x.block<12, 1>(26, 0); 
    
    // Set new goals(expected trajectory) to _measurement
    _measurement.l_elbow_pos_goal = result.y_le.block(0, n, 3, 1); // Vector3d
    _measurement.r_elbow_pos_goal = result.y_re.block(0, n, 3, 1); // Vector3d is column vector

    // Compute unary costs
    double elbow_pos_cost = return_elbow_pos_cost(right_fk_solver, q_cur_r, _measurement.r_num_wrist_seg, _measurement.r_num_elbow_seg, _measurement.r_num_shoulder_seg, false, _measurement);

    elbow_pos_costs.push_back(elbow_pos_cost);

  }

  return elbow_pos_costs;

}

/* left elbow cost */
std::vector<double> TrackingConstraint::return_l_elbow_pos_cost_history()
{
  // Generate new trajectories using DMP
  const DMPStartsGoalsVertex *v = static_cast<const DMPStartsGoalsVertex*>(_vertices[0]); // the last vertex connected
  Matrix<double, DMPPOINTS_DOF, 1> x = v->estimate();
  MatrixXd lrw_new_goal(3, 1); lrw_new_goal = x.block(0, 0, 3, 1);
  MatrixXd lrw_new_start(3, 1); lrw_new_start = x.block(3, 0, 3, 1);
  MatrixXd lew_new_goal(3, 1); lew_new_goal = x.block(6, 0, 3, 1);
  MatrixXd lew_new_start(3, 1); lew_new_start = x.block(9, 0, 3, 1);
  MatrixXd rew_new_goal(3, 1); rew_new_goal = x.block(12, 0, 3, 1);
  MatrixXd rew_new_start(3, 1); rew_new_start = x.block(15, 0, 3, 1);
  MatrixXd rw_new_goal(3, 1); rw_new_goal = x.block(18, 0, 3, 1);
  MatrixXd rw_new_start(3, 1); rw_new_start = x.block(21, 0, 3, 1);
  DMP_trajs result = trajectory_generator_ptr->generate_trajectories(lrw_new_goal.transpose(), lrw_new_start.transpose(), // should be row vectors
                                                                      lew_new_goal.transpose(), lew_new_start.transpose(),
                                                                      rew_new_goal.transpose(), rew_new_start.transpose(),
                                                                      rw_new_goal.transpose(), rw_new_start.transpose(),
                                                                      NUM_DATAPOINTS); // results are 3 x N

  // iterate to compute costs
  std::vector<double> elbow_pos_costs;
  for (unsigned int n = 0; n < num_datapoints; n++)
  {
    // get the current joint value
    const DualArmDualHandVertex *v = static_cast<const DualArmDualHandVertex*>(_vertices[n+1]);
    const Matrix<double, JOINT_DOF, 1> x = v->estimate(); // return the current estimate of the vertex

    // Get joint angles
    Matrix<double, 7, 1> q_cur_l, q_cur_r;
    Matrix<double, 12, 1> q_cur_finger_l, q_cur_finger_r;
    q_cur_l = x.block<7, 1>(0, 0);
    q_cur_r = x.block<7, 1>(7, 0);
    q_cur_finger_l = x.block<12, 1>(14, 0);
    q_cur_finger_r = x.block<12, 1>(26, 0); 
    
    // Set new goals(expected trajectory) to _measurement
    _measurement.l_elbow_pos_goal = result.y_le.block(0, n, 3, 1); // Vector3d
    _measurement.r_elbow_pos_goal = result.y_re.block(0, n, 3, 1); // Vector3d is column vector

    // Compute unary costs
    double elbow_pos_cost = return_elbow_pos_cost(left_fk_solver, q_cur_l, _measurement.l_num_wrist_seg, _measurement.l_num_elbow_seg, _measurement.l_num_shoulder_seg, true, _measurement); // user data is stored in _measurement now

    elbow_pos_costs.push_back(elbow_pos_cost);

  }

  return elbow_pos_costs;

}



void TrackingConstraint::computeError()
{
  num_track++;

  //std::cout << "Tracking Edge is called!" << std::endl;

  // statistics
  count_tracking++;  
  std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();

  
  // Use DMP to generate new trajectories
  const DMPStartsGoalsVertex *v = dynamic_cast<const DMPStartsGoalsVertex*>(_vertices[0]); // the last vertex connected
  Matrix<double, DMPPOINTS_DOF, 1> x = v->estimate();
  MatrixXd lrw_new_goal(3, 1); lrw_new_goal = x.block(0, 0, 3, 1);
  MatrixXd lrw_new_start(3, 1); lrw_new_start = x.block(3, 0, 3, 1);
  MatrixXd lew_new_goal(3, 1); lew_new_goal = x.block(6, 0, 3, 1);
  MatrixXd lew_new_start(3, 1); lew_new_start = x.block(9, 0, 3, 1);
  MatrixXd rew_new_goal(3, 1); rew_new_goal = x.block(12, 0, 3, 1);
  MatrixXd rew_new_start(3, 1); rew_new_start = x.block(15, 0, 3, 1);
  MatrixXd rw_new_goal(3, 1); rw_new_goal = x.block(18, 0, 3, 1);
  MatrixXd rw_new_start(3, 1); rw_new_start = x.block(21, 0, 3, 1);

  /*
  std::cout << "debug: \nx = " << x.transpose() << std::endl
            << "lrw_new_goal = " << lrw_new_goal.transpose() << ", lrw_new_start = " << lrw_new_start.transpose() << std::endl
            << "lew_new_goal = " << lew_new_goal.transpose() << ", lew_new_start = " << lew_new_start.transpose() << std::endl
            << "rew_new_goal = " << rew_new_goal.transpose() << ", rew_new_start = " << rew_new_start.transpose() << std::endl
            << "rw_new_goal = " << rw_new_goal.transpose() << ", rw_new_start = " << rw_new_start.transpose() << std::endl;
  */

  std::chrono::steady_clock::time_point t00 = std::chrono::steady_clock::now();  
  DMP_trajs result = trajectory_generator_ptr->generate_trajectories(lrw_new_goal.transpose(), lrw_new_start.transpose(), // should be row vectors
                                                                      lew_new_goal.transpose(), lew_new_start.transpose(),
                                                                      rew_new_goal.transpose(), rew_new_start.transpose(),
                                                                      rw_new_goal.transpose(), rw_new_start.transpose(),
                                                                      NUM_DATAPOINTS); // results are 3 x N
  std::chrono::steady_clock::time_point t11 = std::chrono::steady_clock::now();
  std::chrono::duration<double> t_spent1 = std::chrono::duration_cast<std::chrono::duration<double>>(t11 - t00);
  total_traj += t_spent1.count();
  count_traj++;  

  /*
  std::cout << "lrw_new_goal = " << result.y_lrw.block(0, num_datapoints-1, 3, 1).transpose() << ", lrw_new_start = " << result.y_lrw.block(0, 0, 3, 1).transpose() << std::endl
            << "lew_new_goal = " << result.y_lew.block(0, num_datapoints-1, 3, 1).transpose() << ", lew_new_start = " << result.y_lew.block(0, 0, 3, 1).transpose() << std::endl
            << "rew_new_goal = " << result.y_rew.block(0, num_datapoints-1, 3, 1).transpose() << ", rew_new_start = " << result.y_rew.block(0, 0, 3, 1).transpose() << std::endl
            << "rw_new_goal = " << result.y_rw.block(0, num_datapoints-1, 3, 1).transpose() << ", rw_new_start = " << result.y_rw.block(0, 0, 3, 1).transpose() << std::endl;
  */


  // Iterate to compute costs
  double cost = 0;
  double total_cost = 0;
  // store for debug
  this->cur_wrist_pos_cost_total = 0;
  this->cur_wrist_ori_cost_total = 0;
  this->cur_elbow_pos_cost_total = 0;
  this->cur_finger_pos_cost_total = 0;  
  for (unsigned int n = 0; n < num_datapoints; n++)
  {
    // get the current joint value
    // _vertices is a VertexContainer type, a std::vector<Vertex*>
    const DualArmDualHandVertex *v = static_cast<const DualArmDualHandVertex*>(_vertices[n+1]);
    const Matrix<double, JOINT_DOF, 1> x = v->estimate(); // return the current estimate of the vertex
    //std::cout << "debug: x size is: " << x.rows() << " x " << x.cols() << std::endl;
    //std::cout << "debug: x = \n" << x.transpose() << std::endl;

    // Get joint angles
    Matrix<double, 7, 1> q_cur_l, q_cur_r;
    Matrix<double, 12, 1> q_cur_finger_l, q_cur_finger_r;
    q_cur_l = x.block<7, 1>(0, 0);
    q_cur_r = x.block<7, 1>(7, 0);
    q_cur_finger_l = x.block<12, 1>(14, 0);
    q_cur_finger_r = x.block<12, 1>(26, 0); 
    
   
    // Use DMP to generate new position trajectories, orientation & glove trajs are resampled and loaded in DMPTrajectoryGenerator
    // Set new goals(expected trajectory) to _measurement
    // left arm part:
    _measurement.l_wrist_pos_goal = result.y_lw.block(0, n, 3, 1); // Vector3d
    Quaterniond q_l(trajectory_generator_ptr->l_wrist_quat_traj(n, 0), 
                    trajectory_generator_ptr->l_wrist_quat_traj(n, 1),
                    trajectory_generator_ptr->l_wrist_quat_traj(n, 2),
                    trajectory_generator_ptr->l_wrist_quat_traj(n, 3));
    Matrix3d l_wrist_ori_goal = q_l.toRotationMatrix();
    _measurement.l_wrist_ori_goal = l_wrist_ori_goal; // Matrix3d
    _measurement.l_elbow_pos_goal = result.y_le.block(0, n, 3, 1); // Vector3d

    // right arm part:
    _measurement.r_wrist_pos_goal = result.y_rw.block(0, n, 3, 1); // Vector3d
    Quaterniond q_r(trajectory_generator_ptr->r_wrist_quat_traj(n, 0), 
                    trajectory_generator_ptr->r_wrist_quat_traj(n, 1),
                    trajectory_generator_ptr->r_wrist_quat_traj(n, 2),
                    trajectory_generator_ptr->r_wrist_quat_traj(n, 3));
    Matrix3d r_wrist_ori_goal = q_r.toRotationMatrix();  
    _measurement.r_wrist_ori_goal = r_wrist_ori_goal; // Matrix3d
    _measurement.r_elbow_pos_goal = result.y_re.block(0, n, 3, 1); // Vector3d is column vector

    // hand parts:
    _measurement.l_finger_pos_goal = trajectory_generator_ptr->l_glove_angle_traj.block(n, 0, 1, 14).transpose() * M_PI / 180.0; // y_seq's glove data is already in radius
    _measurement.r_finger_pos_goal = trajectory_generator_ptr->r_glove_angle_traj.block(n, 0, 1, 14).transpose() * M_PI / 180.0; // size is 50 x DOF

    /*
    std::cout << "l_wrist_pos_goal: \n" << _measurement.l_wrist_pos_goal.transpose() << "\n"
              << "l_wrist_ori_goal: \n" << _measurement.l_wrist_ori_goal << "\n"
              << "l_elbow_pos_goal: \n" << _measurement.l_elbow_pos_goal.transpose() << "\n"
              << "quaternion q_l: \n" << q_l.w() << " " << q_l.x() << " " << q_l.y() << " " << q_l.z() << "\n"

              << "r_wrist_pos_goal: \n" << _measurement.r_wrist_pos_goal.transpose() << "\n"
              << "r_wrist_ori_goal: \n" << _measurement.r_wrist_ori_goal << "\n"
              << "r_elbow_pos_goal: \n" << _measurement.r_elbow_pos_goal.transpose() << "\n"
              << "quaternion q_r: \n" << q_r.w() << " " << q_r.x() << " " << q_r.y() << " " << q_r.z() << "\n"

              << "l_finger_pos_goal: \n" << _measurement.l_finger_pos_goal.transpose() << "\n"
              << "r_finger_pos_goal: \n" << _measurement.r_finger_pos_goal.transpose() << std::endl;
    */

    //21,22,23,24, 25,26,27,28, 29,30,31,32, 33,34,
    //35,36,37,38,39,40,41,42,43,44,45,46,47,48

    
    // Compute unary costs
    // 1
    double arm_cost = compute_arm_cost(left_fk_solver, q_cur_l, _measurement.l_num_wrist_seg, _measurement.l_num_elbow_seg, _measurement.l_num_shoulder_seg, true, _measurement); // user data is stored in _measurement now
    this->cur_wrist_pos_cost_total += this->cur_wrist_pos_cost;
    this->cur_wrist_ori_cost_total += this->cur_wrist_ori_cost;
    this->cur_elbow_pos_cost_total += this->cur_elbow_pos_cost;
    arm_cost += compute_arm_cost(right_fk_solver, q_cur_r, _measurement.r_num_wrist_seg, _measurement.r_num_elbow_seg, _measurement.r_num_shoulder_seg, false, _measurement);
    this->cur_wrist_pos_cost_total += this->cur_wrist_pos_cost;
    this->cur_wrist_ori_cost_total += this->cur_wrist_ori_cost;
    this->cur_elbow_pos_cost_total += this->cur_elbow_pos_cost;
    //std::cout << "Arm cost=";
    //std::cout << arm_cost << ", ";

    // 2
    double finger_cost = compute_finger_cost(q_cur_finger_l, true, _measurement);  
    finger_cost += compute_finger_cost(q_cur_finger_r, false, _measurement);  
    //std::cout << "Finger cost=";
    //std::cout << finger_cost << ", ";

    // total cost
    cost = arm_cost + K_FINGER * finger_cost; // arm_cost is already weighted in compute_arm_cost()
    //std::cout << "debug: arm_cost = " << arm_cost << ", finger_cost = " << finger_cost << std::endl;
    total_cost += cost;

    this->cur_finger_pos_cost_total += K_FINGER * finger_cost;  

  }

  // compute the cost (constraint value)
  _error(0, 0) = total_cost;

  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  std::chrono::duration<double> t_spent = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  total_tracking += t_spent.count();

}




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


  // test
//  std::cout << "Test the torch..." << std::endl;
//  test_torch();


  // Optimization settings
  std::string in_file_name = "test_imi_data_YuMi.h5";
  std::string in_group_name = "fengren_1";
  std::string out_file_name = "mocap_ik_results_YuMi_g2o.h5";
  bool continue_optim = false; // If using previously optimized result as the initial value
  bool pre_iteration = false; // whether load pre-iteration results or perform a new round of pre-iterations

  // Process the terminal arguments
  static struct option long_options[] = 
  {
    {"in-h5-filename",        required_argument, NULL, 'i'},
    {"in-group-name",         required_argument, NULL, 'g'},
    {"out-h5-filename",       required_argument, NULL, 'o'},
    {"continue-optimization", required_argument, NULL, 'c'},
    {"pre-iteration",               no_argument, NULL, 'p'},    
    {"help",                        no_argument, NULL, 'h'},
    {0,                                       0,    0,   0}
  };
  int c;
  while(1)
  {
    int opt_index = 0;
    // Get arguments
    c = getopt_long(argc, argv, "i:g:o:c:ph", long_options, &opt_index);
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
        std::cout << "    -c, --continue-optimization, If using the previously optimized results stored in out-h5-name as the initial guess.\n" << std::endl;
        std::cout << "    -p, --pre-iteration, Whether load pre-iteration results from out-h5-name file or start a new round of pre-iteration.\n" << std::endl;
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

      case 'c':
        continue_optim = true;
        break;

      case 'p':
        pre_iteration = true;
        break;

      default:
        break;
    }

  }
  std::cout << "The input h5 file name is: " << in_file_name << std::endl;
  std::cout << "The motion name is: " << in_group_name << std::endl;
  std::cout << "The output h5 file name is: " << out_file_name << std::endl;
  std::cout << "The current optimization " << (continue_optim ? "is" : "is not") << " following the previously optimized results." << std::endl; 
  std::cout << "debug: -c is set as " << (continue_optim ? "true" : "false") << "..." << std::endl;
  std::cout << "debug: -p is set as " << (pre_iteration ? "true" : "false") << "..." << std::endl;

  // Create a struct for storing user-defined data
  my_constraint_struct constraint_data; 


  // Input Cartesian trajectories
  //std::vector<double> x(JOINT_DOF);
  //std::cout << "========== Reading imitation data from h5 file ==========" << std::endl;
/*
  std::cout << "left part: " << std::endl;
  std::vector<std::vector<double>> read_l_wrist_pos_traj = read_h5(in_file_name, in_group_name, "l_wrist_pos"); 
  std::vector<std::vector<double>> read_l_wrist_ori_traj = read_h5(in_file_name, in_group_name, "l_wrist_ori"); 
  std::vector<std::vector<double>> read_l_elbow_pos_traj = read_h5(in_file_name, in_group_name, "l_elbow_pos"); 
  std::vector<std::vector<double>> read_l_shoulder_pos_traj = read_h5(in_file_name, in_group_name, "l_shoulder_pos"); 

  std::cout << "right part: " << std::endl;
  std::vector<std::vector<double>> read_r_wrist_pos_traj = read_h5(in_file_name, in_group_name, "r_wrist_pos"); 
  std::vector<std::vector<double>> read_r_wrist_ori_traj = read_h5(in_file_name, in_group_name, "r_wrist_ori"); 
  std::vector<std::vector<double>> read_r_elbow_pos_traj = read_h5(in_file_name, in_group_name, "r_elbow_pos"); 
  std::vector<std::vector<double>> read_r_shoulder_pos_traj = read_h5(in_file_name, in_group_name, "r_shoulder_pos"); 

  std::cout << "finger part: " << std::endl;
  std::vector<std::vector<double>> read_l_finger_pos_traj = read_h5(in_file_name, in_group_name, "l_glove_angle"); // N * 14 size
  std::vector<std::vector<double>> read_r_finger_pos_traj = read_h5(in_file_name, in_group_name, "r_glove_angle"); // N * 14 size  


  std::vector<std::vector<double>> read_time_stamps = read_h5(in_file_name, in_group_name, "time"); 
*/


  std::cout << ">>>> Reading data from h5 file" << std::endl;

  //std::vector<std::vector<double>> read_pass_points = read_h5(in_file_name, in_group_name, "pass_points"); 
  // std::vector<std::vector<double>> read_original_traj = read_h5(in_file_name, in_group_name, "resampled_normalized_flattened_oritraj"); 

  //unsigned int num_passpoints = read_pass_points.size();
  //std::cout << "Number of pass points: " << num_passpoints << std::endl;


  //std::cout << "size: " << read_original_traj.size() << " x " << read_original_traj[0].size() << std::endl;
  //std::cout << "test: " << read_original_traj[0][0] << " " << read_original_traj[1][0] << " " << read_original_traj[2][0] << std::endl;
  //std::cout << "number of pass points: " << num_passpoints << std::endl;


 // Variables' bounds
  const std::vector<double> q_l_arm_lb = {-2.8, -2.49, -1.2, -1.7, -2.0, -1.5, -2.0};
  //{-2.9, -2.49, 0.0, -1.7, -1.578, -1.5, -1.57};//{-2.94, -2.5, -2.94, -2.16, -5.06, -1.54, -4.0};
  const std::vector<double> q_l_arm_ub = {0.5, 0.75, 2.2, 1.4, 1.578, 2.1, 1.578};//{2.94, 0.76, 2.94, 1.4, 5.06, 2.41, 4.0};

  const std::vector<double> q_r_arm_lb = {-0.5, -2.49, -2.2, -1.7, -2.0, -1.5, -2.0}; // modified on 2020/07/20
  //{-2.9, -2.49, 0.0, -1.7, -1.578, -1.5, -1.57};//{-2.94, -2.50, -2.94, -2.16, -5.06, -1.54, -4.0};
  const std::vector<double> q_r_arm_ub = {2.8, 0.75, 1.2, 1.4, 1.578, 2.1, 1.578}; // modified on 2020/07/20
  //{0.5, 0.75, 2.9, 1.4, 1.578, 2.1, 1.57};//{2.94, 0.76, 2.94, 1.4, 5.06, 2.41, 4.0};


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

  // Read previously optimized result
  std::vector<std::vector<double>> prev_q_results; 
  //std::vector<std::vector<double>> prev_passpoint_results = read_h5(out_file_name, in_group_name, "passpoint_traj_1"); 
  std::vector<std::vector<double>> prev_dmp_starts_goals_results;
  if (continue_optim)
  {
    prev_q_results = read_h5(out_file_name, in_group_name, "arm_traj_1"); 
    prev_dmp_starts_goals_results = read_h5(out_file_name, in_group_name, "dmp_starts_goals_1");
  }


  // Setup KDL FK solver( and set IDs for wrist, elbow and shoulder)
  std::cout << ">>>> Setting KDL FK Solvers " << std::endl;
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
  std::cout << ">>>> Constructing an optimization graph " << std::endl;
  // Construct solver (be careful, use unique_ptr instead!!! )
  //typedef BlockSolver<BlockSolverTraits<JOINT_DOF+NUM_PASSPOINTS, 1> > Block; // BlockSolverTraits<_PoseDim, _LandmarkDim>
  typedef BlockSolver<BlockSolverTraits<-1, -1> > Block; // BlockSolverTraits<_PoseDim, _LandmarkDim>

  //std::unique_ptr<Block::LinearSolverType> linearSolver( new LinearSolverDense<Block::PoseMatrixType>()); // dense solution
  std::unique_ptr<Block::LinearSolverType> linearSolver( new LinearSolverCholmod<Block::PoseMatrixType>()); // Cholmod
  //std::unique_ptr<Block::LinearSolverType> linearSolver( new LinearSolverCSparse<Block::PoseMatrixType>()); // CSparse
  //std::unique_ptr<Block::LinearSolverType> linearSolver( new LinearSolverPCG<Block::PoseMatrixType>()); // PCG
  //std::unique_ptr<Block::LinearSolverType> linearSolver( new LinearSolverEigen<Block::PoseMatrixType>()); // Eigen

  std::unique_ptr<Block> solver_ptr( new Block(std::move(linearSolver)) ); // Matrix block solver

  // Choose update rule
  OptimizationAlgorithmLevenberg *solver = new OptimizationAlgorithmLevenberg(std::move(solver_ptr));
  // OptimizationAlgorithmGaussNewton *solver = new OptimizationAlgorithmGaussNewton(std::move(solver_ptr));
  //OptimizationAlgorithmDogleg *solver = new OptimizationAlgorithmDogleg(std::move(solver_ptr));

  // Construct an optimizer (a graph model)
  SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(true);


  // Prepare collision checker
  std::cout << ">>>> Preparing collision checker " << std::endl;
  // Get URDF and SRDF for distance computation class
  std::string urdf_file_name = "/home/liangyuwei/sign_language_robot_ws/src/yumi_description/urdf/yumi_with_hands.urdf";
  std::string srdf_file_name = "/home/liangyuwei/sign_language_robot_ws/src/yumi_sign_language_robot_moveit_config/config/yumi.srdf";
  std::stringstream urdf_string = read_file(urdf_file_name);
  std::stringstream srdf_string = read_file(srdf_file_name);
  boost::shared_ptr<DualArmDualHandCollision> dual_arm_dual_hand_collision_ptr;
  dual_arm_dual_hand_collision_ptr.reset( new DualArmDualHandCollision(urdf_string.str(), srdf_string.str()) );


  // Prepare trajectory generator
  std::cout << ">>>> Preparing trajectory generator " << std::endl;
  boost::shared_ptr<DMPTrajectoryGenerator> trajectory_generator_ptr;
  trajectory_generator_ptr.reset( new DMPTrajectoryGenerator(in_file_name, in_group_name) );  


  // Prepare similarity network
  // std::cout << ">>>> Preparing similarity network " << std::endl;
  // std::string model_path = "/home/liangyuwei/sign_language_robot_ws/test_imi_data/trained_model_adam_euclidean_epoch2000_bs1024_group_split_dataset_50p.pt";
  // //VectorXd original_traj(4800);
  // Matrix<double, PASSPOINT_DOF*NUM_DATAPOINTS, 1> original_traj; // size is known at compile time 
  // for (int i = 0; i < read_original_traj.size(); i++)
  //   original_traj[i] = read_original_traj[i][0];
  // boost::shared_ptr<SimilarityNetwork> similarity_network_ptr;
  // //std::cout << "debug: original_traj size is " << original_traj.rows() << " x " << original_traj.cols() << std::endl;
  // similarity_network_ptr.reset( new SimilarityNetwork(model_path, original_traj) );  



  // Add vertices and edges
  std::vector<DualArmDualHandVertex*> v_list(NUM_DATAPOINTS);
  std::vector<PassPointVertex*> pv_list(NUM_DATAPOINTS);
  DMPStartsGoalsVertex* dmp_vertex;
  std::vector<MyUnaryConstraints*> unary_edges;
  std::vector<SmoothnessConstraint*> smoothness_edges;  
  TrackingConstraint* tracking_edge;
  DMPConstraints* dmp_edge;

 
  tracking_edge = new TrackingConstraint(trajectory_generator_ptr, 
                                         left_fk_solver, right_fk_solver, 
                                         dual_arm_dual_hand_collision_ptr,
                                         NUM_DATAPOINTS);
  tracking_edge->setId(1);


  /* Edge order: similarity(0), tracking(1), unary(2 ~ num_datapoints+1), smoothness()*/
  /* Vertex order: pass_points(0 ~ num_passpoints-1), datapoints(num_passpoints ~ num_passpoints+num_datapoints-1) */
  /* Vertex new order(with DMP): DMP starts and goals (1), datapoints(1~num_datapoints)*/

  // For pass points
  /*
  std::cout << ">>>> Adding pass_points vertices, similarity edge and tracking edge " << std::endl;
  for (unsigned int it = 0; it < num_passpoints; it++)
  {
    // preparation
    std::vector<double> pass_point = read_pass_points[it]; // 48-dim
    Matrix<double, PASSPOINT_DOF, 1> pass_point_mat = Map<Matrix<double, PASSPOINT_DOF, 1>>(pass_point.data(), pass_point.size());
    //std::cout << "Check size: " << std::endl;
    //std::cout << "std::vector: " << pass_point[0] << ", " << pass_point[1] << ", " << pass_point[2] << std::endl;
    //std::cout << "Matrix: " << pass_point_mat[0] << ", " << pass_point_mat[1] << ", " << pass_point_mat[2] << std::endl;

    // create vertex
    pv_list[it] = new PassPointVertex();
    if(continue_optim)
    {
      std::vector<double> prev_passpoint_result = prev_passpoint_results[it]; // 48-dim
      Matrix<double, PASSPOINT_DOF, 1> prev_passpoint_mat = \
                Map<Matrix<double, PASSPOINT_DOF, 1>>(prev_passpoint_result.data(), prev_passpoint_result.size());
      pv_list[it]->setEstimate(prev_passpoint_mat); // use previously optimized result as an initial guess
    }
    else
    {
      pv_list[it]->setEstimate(pass_point_mat);
    }
    pv_list[it]->setId(it); // set a unique id
    optimizer.addVertex(pv_list[it]);    

    // connect to edge
    similarity_edge->setVertex(it, optimizer.vertex(it));

    tracking_edge->setVertex(it, optimizer.vertex(it));

  }
  similarity_edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
  optimizer.addEdge(similarity_edge);
  */


  // For DMP
  std::cout << ">>>> Adding DMP vertex, similarity edge and tracking edge " << std::endl;
  // preparation
  dmp_vertex = new DMPStartsGoalsVertex();
  Matrix<double, DMPPOINTS_DOF, 1> DMP_ori_starts_goals; // lrw, lew, rew, rw; goal, start. 
  if (continue_optim)
  {
    std::vector<double> prev_dmp_starts_goals = prev_dmp_starts_goals_results[0];
    for (unsigned int d = 0; d < DMPPOINTS_DOF; d++)
      DMP_ori_starts_goals[d] = prev_dmp_starts_goals[d];
  }
  else
  {
    // move right wrist starts and goals to be symmetric to x-z plane
    /*
    std::cout << ">>>> Move the whole trajectories to be symmetric to x-z plane, as initial guess" << std::endl;
    double rw_g_y = trajectory_generator_ptr->rw_goal(0, 1); // 1 x 3
    double lw_g_y = rw_g_y + trajectory_generator_ptr->lrw_goal(0, 1);
    double rw_s_y = trajectory_generator_ptr->rw_start(0, 1);
    double lw_s_y = rw_s_y + trajectory_generator_ptr->lrw_start(0, 1);
    double half_g_y = (rw_g_y + lw_g_y) / 2;
    double half_s_y = (rw_s_y + lw_s_y) / 2;
    std::cout << "Original right wrist start = " << trajectory_generator_ptr->rw_start << std::endl;
    std::cout << "Original right wrist goal = " << trajectory_generator_ptr->rw_goal << std::endl;
    trajectory_generator_ptr->rw_goal(0, 1) = -half_g_y;
    trajectory_generator_ptr->rw_start(0, 1) = -half_s_y;
    std::cout << "Moved right wrist start = " << trajectory_generator_ptr->rw_start << std::endl;
    std::cout << "Moved right wrist goal = " << trajectory_generator_ptr->rw_goal << std::endl;
    */

    // manually move the DMP trajectories to be within workspace, for modifying coefficients for q tracking desired trajectories
    /*
    std::cout << ">>>> Manually move the whole trajectories to be within workspace" << std::endl;
    MatrixXd rw_tmp_start(1, 3); rw_tmp_start(0, 0) = 0.5; rw_tmp_start(0, 1) = -0.18; rw_tmp_start(0, 2) = 0.5;
    MatrixXd rw_tmp_offset(1, 3); rw_tmp_offset = rw_tmp_start - trajectory_generator_ptr->rw_start;
    std::cout << "Original right wrist start = " << trajectory_generator_ptr->rw_start << std::endl;
    std::cout << "Original right wrist goal = " << trajectory_generator_ptr->rw_goal << std::endl;
    trajectory_generator_ptr->rw_goal += rw_tmp_offset;
    trajectory_generator_ptr->rw_start += rw_tmp_offset;
    std::cout << "Moved right wrist start = " << trajectory_generator_ptr->rw_start << std::endl;
    std::cout << "Moved right wrist goal = " << trajectory_generator_ptr->rw_goal << std::endl;
    */

    // Manually set an initial state for DMPs
    std::cout << ">>>> Manually set initial state for DMPs" << std::endl;
    MatrixXd lw_set_start(1, 3); //lw_set_start(0, 0) = 0.409; lw_set_start(0, 1) = 0.181; lw_set_start(0, 2) = 0.191; 
    MatrixXd le_set_start(1, 3); le_set_start(0, 0) = 0.218; le_set_start(0, 1) = 0.310; le_set_start(0, 2) = 0.378; 
    MatrixXd rw_set_start(1, 3); rw_set_start(0, 0) = 0.410; rw_set_start(0, 1) = -0.179; rw_set_start(0, 2) = 0.191; 
    MatrixXd re_set_start(1, 3); re_set_start(0, 0) = 0.218; re_set_start(0, 1) = -0.310; re_set_start(0, 2) = 0.377; 
    // set rw start to ensure rw start and lw start symmetric to x-z plane, and leave the others
    lw_set_start = rw_set_start + trajectory_generator_ptr->lrw_start;
    double dist_y = std::abs(lw_set_start(0, 1) - rw_set_start(0,1));
    lw_set_start(0, 1) = dist_y / 2.0;
    rw_set_start(0, 1) = -dist_y / 2.0;

    // compute starts for corresponding relative DMPs
    MatrixXd lrw_set_start(1, 3); lrw_set_start = lw_set_start - rw_set_start;
    MatrixXd lew_set_start(1, 3); lew_set_start = le_set_start - lw_set_start;
    MatrixXd rew_set_start(1, 3); rew_set_start = re_set_start - rw_set_start;
    // compute offsets for starts and goals of DMPs
    MatrixXd lrw_tmp_offset(1, 3); lrw_tmp_offset = lrw_set_start - trajectory_generator_ptr->lrw_start;
    MatrixXd lew_tmp_offset(1, 3); lew_tmp_offset = lew_set_start - trajectory_generator_ptr->lew_start;
    MatrixXd rew_tmp_offset(1, 3); rew_tmp_offset = rew_set_start - trajectory_generator_ptr->rew_start;    
    MatrixXd rw_tmp_offset(1, 3); rw_tmp_offset = rw_set_start - trajectory_generator_ptr->rw_start;
    // add offset to move the whole trajectories
    std::cout << "Original lrw_start = " << trajectory_generator_ptr->lrw_start << std::endl;
    std::cout << "Original lrw_goal = " << trajectory_generator_ptr->lrw_goal << std::endl;
    std::cout << "Original lew_start = " << trajectory_generator_ptr->lew_start << std::endl;
    std::cout << "Original lew_goal = " << trajectory_generator_ptr->lew_goal << std::endl;
    std::cout << "Original rew_start = " << trajectory_generator_ptr->rew_start << std::endl;
    std::cout << "Original rew_goal = " << trajectory_generator_ptr->rew_goal << std::endl;         
    std::cout << "Original rw_start = " << trajectory_generator_ptr->rw_start << std::endl;
    std::cout << "Original rw_goal = " << trajectory_generator_ptr->rw_goal << std::endl;
    trajectory_generator_ptr->lrw_goal += lrw_tmp_offset;
    trajectory_generator_ptr->lrw_start += lrw_tmp_offset;
    trajectory_generator_ptr->lew_goal += lew_tmp_offset;
    trajectory_generator_ptr->lew_start += lew_tmp_offset;
    trajectory_generator_ptr->rew_goal += rew_tmp_offset;
    trajectory_generator_ptr->rew_start += rew_tmp_offset;
    trajectory_generator_ptr->rw_goal += rw_tmp_offset;
    trajectory_generator_ptr->rw_start += rw_tmp_offset;
    std::cout << "Moved lrw_start = " << trajectory_generator_ptr->lrw_start << std::endl;
    std::cout << "Moved lrw_goal = " << trajectory_generator_ptr->lrw_goal << std::endl;
    std::cout << "Moved lew_start = " << trajectory_generator_ptr->lew_start << std::endl;
    std::cout << "Moved lew_goal = " << trajectory_generator_ptr->lew_goal << std::endl;
    std::cout << "Moved rew_start = " << trajectory_generator_ptr->rew_start << std::endl;
    std::cout << "Moved rew_goal = " << trajectory_generator_ptr->rew_goal << std::endl;         
    std::cout << "Moved rw_start = " << trajectory_generator_ptr->rw_start << std::endl;
    std::cout << "Moved rw_goal = " << trajectory_generator_ptr->rw_goal << std::endl;

    DMP_ori_starts_goals.block(0, 0, 3, 1) = trajectory_generator_ptr->lrw_goal.transpose(); // 1 x 3
    DMP_ori_starts_goals.block(3, 0, 3, 1) = trajectory_generator_ptr->lrw_start.transpose();

    DMP_ori_starts_goals.block(6, 0, 3, 1) = trajectory_generator_ptr->lew_goal.transpose();
    DMP_ori_starts_goals.block(9, 0, 3, 1) = trajectory_generator_ptr->lew_start.transpose();

    DMP_ori_starts_goals.block(12, 0, 3, 1) = trajectory_generator_ptr->rew_goal.transpose();
    DMP_ori_starts_goals.block(15, 0, 3, 1) = trajectory_generator_ptr->rew_start.transpose();

    DMP_ori_starts_goals.block(18, 0, 3, 1) = trajectory_generator_ptr->rw_goal.transpose();
    DMP_ori_starts_goals.block(21, 0, 3, 1) = trajectory_generator_ptr->rw_start.transpose();

    
    // store for comparison and presentation
    std::vector<std::vector<double> > dmp_starts_goals_store;
    std::vector<double> dmp_starts_goals_vec(DMPPOINTS_DOF);
    DMPStartsGoalsVertex* vertex_tmp = dynamic_cast<DMPStartsGoalsVertex*>(optimizer.vertex(0));
    for (unsigned int d = 0; d < DMPPOINTS_DOF; d++)
      dmp_starts_goals_vec[d] = DMP_ori_starts_goals[d];
    dmp_starts_goals_store.push_back(dmp_starts_goals_vec);
    bool result2 = write_h5(out_file_name, in_group_name, "dmp_starts_goals_moved", dmp_starts_goals_store.size(), dmp_starts_goals_store[0].size(), dmp_starts_goals_store);
    std::cout << "dmp results stored " << (result2 ? "successfully" : "unsuccessfully") << "!" << std::endl;

  }
  dmp_vertex->setEstimate(DMP_ori_starts_goals);
  dmp_vertex->setId(0);
  optimizer.addVertex(dmp_vertex);    


  // connect to edge
  tracking_edge->setVertex(0, optimizer.vertex(0));
  // similarity_edge->setVertex(0, optimizer.vertex(0));  
  // similarity_edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
  // optimizer.addEdge(similarity_edge);


  
  std::cout << ">>>> Adding joint angle vertices, unary edges and tracking_error edges " << std::endl;  
  // set non-colliding initial state for ease of collision avoidance
  Matrix<double, JOINT_DOF, 1> q_initial = Matrix<double, JOINT_DOF, 1>::Zero();
  q_initial.block(0, 0, 14, 1) << -1.5, -1.5, 1.5, 0.0, 0.0, 0.0, 0.0, 1.5, -1.5, -1.5, 0.0, 0.0, 0.0, 0.0;
  std::cout << "debug: new initial q = " << q_initial.transpose() << std::endl;
  for (unsigned int it = 0; it < NUM_DATAPOINTS; ++it)
  {
    // add vertices
    //DualArmDualHandVertex *v = new DualArmDualHandVertex();
    v_list[it] = new DualArmDualHandVertex();
    if(continue_optim)
    {
      std::vector<double> prev_q_result = prev_q_results[it]; // 38-dim
      Matrix<double, JOINT_DOF, 1> prev_q_mat = Map<Matrix<double, JOINT_DOF, 1>>(prev_q_result.data(), prev_q_result.size());
      v_list[it]->setEstimate(prev_q_mat); // use previously optimized result as an initial guess
    }
    else
    {
      //v_list[it]->setEstimate(Matrix<double, JOINT_DOF, 1>::Zero()); // feed in initial guess
      v_list[it]->setEstimate(q_initial);
    }
    v_list[it]->setId(1+it); // set a unique id
    optimizer.addVertex(v_list[it]);

    /*
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
    */

    // id for selecting point on new trajectory (y_seq)
    //constraint_data.point_id = it; 


    // add unary edges
    MyUnaryConstraints *unary_edge = new MyUnaryConstraints(left_fk_solver, right_fk_solver, dual_arm_dual_hand_collision_ptr);
    unary_edge->setId(it+2); // similarity and tracking edges ahead of it
    unary_edge->setVertex(0, optimizer.vertex(1+it)); //(0, v_list[it]); // set the 0th vertex on the edge to point to v_list[it]
    unary_edge->setMeasurement(constraint_data); // set _measurement attribute (by deep copy), can be used to pass in user data, e.g. my_constraint_struct
    unary_edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity()); // information matrix, inverse of covariance.. importance // Information type correct
    optimizer.addEdge(unary_edge);

    unary_edges.push_back(unary_edge);
    
    
    // add tracking edges
    tracking_edge->setVertex(1+it, optimizer.vertex(1+it)); 
 
  }
  tracking_edge->setMeasurement(constraint_data);
  tracking_edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
  optimizer.addEdge(tracking_edge);

  
  std::cout << ">>>> Adding binary edges " << std::endl;
  for (unsigned int it = 0; it < NUM_DATAPOINTS - 1; ++it)
  {
    // Add binary edges
    SmoothnessConstraint *smoothness_edge = new SmoothnessConstraint();
    smoothness_edge->setId(NUM_DATAPOINTS+2+it); // set a unique ID
    smoothness_edge->setVertex(0, optimizer.vertex(it+1)); //v_list[it]);
    smoothness_edge->setVertex(1, optimizer.vertex(it+2)); //v_list[it+1]); // binary edge, connects only 2 vertices, i.e. i=0 and i=1
    smoothness_edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity()); // Information type correct
    optimizer.addEdge(smoothness_edge);

    smoothness_edges.push_back(smoothness_edge);

  }
  

  std::cout << ">>>> Adding constraints for DMP starts and goals" << std::endl;
  dmp_edge = new DMPConstraints(trajectory_generator_ptr->lrw_goal, trajectory_generator_ptr->lrw_start,
                                trajectory_generator_ptr->lew_goal, trajectory_generator_ptr->lew_start,
                                trajectory_generator_ptr->rew_goal, trajectory_generator_ptr->rew_start,
                                trajectory_generator_ptr->rw_goal, trajectory_generator_ptr->rw_start);
  dmp_edge->setId(2*NUM_DATAPOINTS+1); 
  dmp_edge->setVertex(0, optimizer.vertex(0)); // DMP vertex
  //dmp_edge->setMeasurement(constraint_data); // set _measurement attribute (by deep copy), can be used to pass in user data, e.g. my_constraint_struct
  dmp_edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity()); // information matrix, inverse of covariance.. importance // Information type correct
  optimizer.addEdge(dmp_edge);



  // Try optimizing without similarity edge
  // remove edges before .initializeOptimization()!!!
  //std::cout << ">>>> Removing edges for debug before .initializeOptimization()" << std::endl;
  //bool remove_result = optimizer.removeEdge(similarity_edge); // must call .initializeOptimization() after removing edges
  //std::cout << "Removing similarity edge " << (remove_result? "successfully" : "unsuccessfully") << "!" << std::endl;


  // Start optimization
  std::cout << ">>>> Start optimization:" << std::endl;
  optimizer.initializeOptimization();
  //optimizer.computeInitialGuess();  
  //optimizer.computeActiveErrors();

  std::cout << "optimizing graph: " << optimizer.vertices().size() << " vertices, " 
                                    << optimizer.edges().size() << " edges." << std::endl;

  // save for fun...
  //bool saveFlag = optimizer.save("/home/liangyuwei/sign_language_robot_ws/g2o_results/result_before.g2o");
  //std::cout << "g2o file saved " << (saveFlag? "successfully" : "unsuccessfully") << " ." << std::endl;

  std::vector<std::vector<double> > col_cost_history;
  std::vector<std::vector<double> > pos_limit_cost_history;
  std::vector<std::vector<double> > wrist_pos_cost_history;
  std::vector<std::vector<double> > l_wrist_pos_cost_history;
  std::vector<std::vector<double> > r_wrist_pos_cost_history;

  std::vector<std::vector<double> > wrist_ori_cost_history;
  std::vector<std::vector<double> > elbow_pos_cost_history;
  std::vector<std::vector<double> > l_elbow_pos_cost_history;
  std::vector<std::vector<double> > r_elbow_pos_cost_history;
  
  std::vector<std::vector<double> > finger_cost_history;
  // std::vector<std::vector<double> > similarity_cost_history;
  std::vector<std::vector<double> > smoothness_cost_history;
  std::vector<std::vector<double> > dmp_orien_cost_history;
  std::vector<std::vector<double> > dmp_scale_cost_history;
  std::vector<std::vector<double> > dmp_rel_change_cost_history;

  //std::vector<std::vector<std::vector<double> > > jacobian_history; // for pass points, store in 3d
  // std::vector<std::vector<double> > sim_jacobian_history; // for DMP, store in 2d
  std::vector<std::vector<double> > track_jacobian_history; // for DMP, store in 2d
  std::vector<std::vector<double> > orien_jacobian_history; // for DMP orientation cost (vectors pointing from starts to goals)
  std::vector<std::vector<double> > scale_jacobian_history; // for DMP scale cost (scale margin of vectors)
  std::vector<std::vector<double> > rel_change_jacobian_history;

  std::vector<std::vector<double> > dmp_update_history;


  // Start optimization and store cost history
  std::cout << ">>>> Start optimization of the whole graph" << std::endl;

  std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
  
  unsigned int num_rounds = 1;//20;//1;//10;//20;//200;
  unsigned int dmp_per_iterations = 10;//5;//10;
  // unsigned int q_per_iterations = 50;//10;//30;//50;//5;//50;
  
  unsigned int q_trk_per_iterations = 20; //50;
  unsigned int q_col_per_iterations = 10; //50;
  
  unsigned int max_round; // record for ease

  // coefficients search space
  double K_COL_MAX = 300.0;//100.0;//1000.0;//1000000.0;//200.0;//1000;//20.0;//15.0;//10.0;
  double K_POS_LIMIT_MAX = 50.0;//30.0;//20;//20.0;//15.0;//10.0;
  double K_SMOOTHNESS_MAX = 50.0;//30.0;//20.0;//15.0;//10.0;//10.0;

  double K_DMPSTARTSGOALS_MAX = 2.0;
  double K_DMPSCALEMARGIN_MAX = 2.0;
  double K_DMPRELCHANGE_MAX = 2.0;

  double K_WRIST_POS_MAX = 100.0;//20.0;//10.0;
  double K_ELBOW_POS_MAX = 100.0;//20.0;//10.0;

  
  // constraints bounds
  double col_cost_bound = 0.5; // to cope with possible numeric error (here we still use check_self_collision() for estimating, because it might not be easy to keep minimum distance outside safety margin... )
  double smoothness_bound = std::sqrt(std::pow(2.0*M_PI/180.0, 2) * JOINT_DOF) * (NUM_DATAPOINTS-1); // in average, 2 degree allowable difference for each joint 
  double pos_limit_bound = 0.0;

  double dmp_orien_cost_bound = 0.0; // better be 0, margin is already set in it!!!
  double dmp_scale_cost_bound = 0.0; // better be 0
  double dmp_rel_change_cost_bound = 0.0; // better be 0

  double wrist_pos_cost_bound = std::sqrt( (std::pow(0.02, 2) * 3) ) * NUM_DATAPOINTS; // 2 cm allowable error
  double elbow_pos_cost_bound = std::sqrt( (std::pow(0.03, 2) * 3) ) * NUM_DATAPOINTS; // 3 cm allowable error


  double scale = 1.5;  // increase coefficients by 20% 
  double outer_scale = 1.5;//2.0;
  double inner_scale = 1.5;
  // double step = 1.0;
  // double level = 0.0;

  // store initial DMP starts and goals
  // initialization
  Matrix<double, DMPPOINTS_DOF, 1> dmp_starts_goals_initial;
  std::vector<std::vector<double> > dmp_starts_goals_initial_vec_vec;
  std::vector<double> dmp_starts_goals_initial_vec(DMPPOINTS_DOF);
  // get data and convert
  DMPStartsGoalsVertex* vertex_tmp = dynamic_cast<DMPStartsGoalsVertex*>(optimizer.vertex(0));
  dmp_starts_goals_initial = vertex_tmp->estimate();
  for (unsigned int d = 0; d < DMPPOINTS_DOF; d++)
      dmp_starts_goals_initial_vec[d] = dmp_starts_goals_initial[d];
  dmp_starts_goals_initial_vec_vec.push_back(dmp_starts_goals_initial_vec);
  // store and display results
  std::cout << "dmp_results size is: " << dmp_starts_goals_initial_vec_vec.size() << " x " << dmp_starts_goals_initial_vec_vec[0].size() << std::endl;
  std::cout << "dmp_results = " << dmp_starts_goals_initial.transpose() << std::endl;
  bool result_initial = write_h5(out_file_name, in_group_name, "dmp_starts_goals_initial", 
                                dmp_starts_goals_initial_vec_vec.size(), dmp_starts_goals_initial_vec_vec[0].size(), 
                                dmp_starts_goals_initial_vec_vec);
  std::cout << "initial dmp starts and goals stored " << (result_initial ? "successfully" : "unsuccessfully") << "!" << std::endl;


  for (unsigned int n = 0; n < num_rounds; n++)
  {
    // 1 - Optimize q vertices
    // coefficients
    // K_COL = 1.0; //50.0;//10.0;//100.0;//1.0;
    // K_POS_LIMIT = 1.0;//10.0;//5.0;//10.0;//5.0; 
    // K_WRIST_ORI = 1.0;
    // K_WRIST_POS = 1.0;
    // K_ELBOW_POS = 1.0;
    // K_FINGER = 1.0;
    // K_SMOOTHNESS = 1.0;//2.0;//4.0;//2.0; 
    // for checking constraints values
    double tmp_col_cost;
    double tmp_pos_limit_cost;
    double tmp_smoothness_cost;
    double tmp_wrist_pos_cost;
    double tmp_elbow_pos_cost;

    double wrist_pos_cost_before_optim;
    double wrist_pos_cost_after_optim;
    double elbow_pos_cost_before_optim;
    double elbow_pos_cost_after_optim;    

    double cur_wrist_pos_cost_update;
    double cur_elbow_pos_cost_update;
    double last_wrist_pos_cost_update = 0.0;
    double last_elbow_pos_cost_update = 0.0;

    std::cout << ">>>> Round " << (n+1) << "/" << num_rounds << ", q stage: "<< std::endl;

    // Fix DMP starts and goals
    std::cout << "Fix DMP starts and goals." << std::endl;
    DMPStartsGoalsVertex* dmp_vertex_tmp = dynamic_cast<DMPStartsGoalsVertex*>(optimizer.vertex(0));    
    dmp_vertex_tmp->setFixed(true);
    

    // Initialize coefficients
    // coefficients
    K_COL = 1.0; // shut collision avoidance
    K_POS_LIMIT = 1.0;//0.2;//0.5;//1.0;
    K_WRIST_ORI = 1.0;//10.0;//1.0;
    K_WRIST_POS = 1.0;//10.0;//1.0;
    K_ELBOW_POS = 1.0;//10.0;
    K_FINGER = 1.0;//10.0;
    K_SMOOTHNESS = 1.0;//0.2;//0.5;//1.0;

    // signal flag for adjusting coefficients
    double last_col_cost = NUM_DATAPOINTS; // worst case
    double cur_col_cost;
    double last_pos_limit_cost = 1000000; // worst case
    double cur_pos_limit_cost;
    double last_smoothness_cost = 1000000; // worst case
    double cur_smoothness_cost;
    double last_wrist_pos_cost = 1000000; // worst case
    double cur_wrist_pos_cost;
    double last_elbow_pos_cost = 1000000; // worst case
    double cur_elbow_pos_cost;

    // iterate to ensure collision, pos_limit costs
    // do
    // {
      // Reset coefficients for tracking loop to adjust
      // K_WRIST_ORI = 1.0;//10.0;//1.0;
      // K_WRIST_POS = 1.0;//10.0;//1.0;
      // K_ELBOW_POS = 1.0;//10.0;
      // K_FINGER = 1.0;//10.0;

    // initialize coefficients
    K_COL = 0.0;
    K_SMOOTHNESS = 1.0;
    K_POS_LIMIT = 1.0;
    
    do
    {
      std::cout << ">>>> Tracking loop: automatically adjust penalties coefficients." << std::endl;
      std::cout << "Current coefficient: K_WRIST_POS = " << K_WRIST_POS << std::endl;
      std::cout << "Current coefficient: K_ELBOW_POS = " << K_ELBOW_POS << std::endl;
      // std::cout << "Current coefficients: K_COL = " << K_COL << ", K_POS_LIMIT = " << K_POS_LIMIT << ", K_SMOOTHNESS = " << K_SMOOTHNESS << std::endl;
      std::cout << "Last updates and costs: wrist_pos_cost = " << last_wrist_pos_cost << ", elbow_pos_cost = " << last_elbow_pos_cost 
                                                               << ", col_cost = " << last_col_cost 
                                                               << ", pos_limit_cost = " << last_pos_limit_cost
                                                               << ", smoothness_cost = " << last_smoothness_cost
                                                               << "." << std::endl;

      // record costs before optimizing
      std::vector<double> wrist_pos_cost = tracking_edge->return_wrist_pos_cost_history();
      std::vector<double> elbow_pos_cost = tracking_edge->return_elbow_pos_cost_history();
      wrist_pos_cost_before_optim = 0.0;
      for (unsigned s = 0; s < wrist_pos_cost.size(); s++)
      {  
        wrist_pos_cost_before_optim += wrist_pos_cost[s];
      }
      elbow_pos_cost_before_optim = 0.0;
      for (unsigned s = 0; s < elbow_pos_cost.size(); s++)
      {  
        elbow_pos_cost_before_optim += elbow_pos_cost[s];
      }


    // optimize for a few iterations
    std::cout << "Optimizing q..." << std::endl;
    unsigned int q_iter = optimizer.optimize(q_trk_per_iterations); 
    
    // Store cost results
    std::cout << ">> Costs <<" << std::endl;
    // collision 
    std::vector<double> col_cost;
    std::cout << "debug: col_cost = ";
    tmp_col_cost = 0.0;
    for (unsigned int t = 0; t < unary_edges.size(); t++)
    {
      double c = unary_edges[t]->return_col_cost();
      col_cost.push_back(c); // store
      tmp_col_cost += c; // check
      std::cout << c << " "; // display
    }
    std::cout << std::endl << std::endl;
    cur_col_cost = tmp_col_cost; // for adjusting K_COL
    col_cost_history.push_back(col_cost);
    // pos_limit
    std::vector<double> pos_limit_cost;
    std::cout << "debug: pos_limit_cost = ";
    tmp_pos_limit_cost = 0.0;
    for (unsigned int t = 0; t < unary_edges.size(); t++)
    {
      double p = unary_edges[t]->return_pos_limit_cost();
      pos_limit_cost.push_back(p); // store
      tmp_pos_limit_cost += p; // check
      std::cout << p << " "; // display      
    }
    std::cout << std::endl << std::endl;    
    cur_pos_limit_cost = tmp_pos_limit_cost; // for adjusting K_POS_LIMIT
    pos_limit_cost_history.push_back(pos_limit_cost);
    // smoothness
    std::vector<double> smoothness_cost;
    std::cout << "debug: smoothness_cost = ";
    tmp_smoothness_cost = 0.0;
    for (unsigned int t = 0; t < smoothness_edges.size(); t++)
    {
      double s = smoothness_edges[t]->return_smoothness_cost();
      smoothness_cost.push_back(s); // store
      tmp_smoothness_cost += s; // check
      std::cout << s << " "; // display
    }
    std::cout << std::endl << std::endl;     
    cur_smoothness_cost = tmp_smoothness_cost; // for adjusting K_SMOOTHNESS   
    smoothness_cost_history.push_back(smoothness_cost);  
    // tracking   
    wrist_pos_cost = tracking_edge->return_wrist_pos_cost_history();
    std::vector<double> l_wrist_pos_cost = tracking_edge->return_l_wrist_pos_cost_history();
    std::vector<double> r_wrist_pos_cost = tracking_edge->return_r_wrist_pos_cost_history();
    std::vector<double> wrist_ori_cost = tracking_edge->return_wrist_ori_cost_history();
    elbow_pos_cost = tracking_edge->return_elbow_pos_cost_history();
    std::vector<double> l_elbow_pos_cost = tracking_edge->return_l_elbow_pos_cost_history();
    std::vector<double> r_elbow_pos_cost = tracking_edge->return_r_elbow_pos_cost_history();
    std::vector<double> finger_cost = tracking_edge->return_finger_cost_history(); 
    wrist_pos_cost_history.push_back(wrist_pos_cost);
    l_wrist_pos_cost_history.push_back(l_wrist_pos_cost);
    r_wrist_pos_cost_history.push_back(r_wrist_pos_cost);
    wrist_ori_cost_history.push_back(wrist_ori_cost);
    elbow_pos_cost_history.push_back(elbow_pos_cost);
    l_elbow_pos_cost_history.push_back(l_elbow_pos_cost);
    r_elbow_pos_cost_history.push_back(r_elbow_pos_cost);
    finger_cost_history.push_back(finger_cost); // store
    // display
    std::cout << "debug: wrist_pos_cost = ";
    tmp_wrist_pos_cost = 0.0;
    for (unsigned s = 0; s < wrist_pos_cost.size(); s++)
    {  
      std::cout << wrist_pos_cost[s] << " ";
      tmp_wrist_pos_cost += wrist_pos_cost[s];
    }
    std::cout << std::endl << std::endl;    
    cur_wrist_pos_cost = tmp_wrist_pos_cost; // for adjusting K_WRIST_POS
    wrist_pos_cost_after_optim = tmp_wrist_pos_cost;
    //
    std::cout << "debug: l_wrist_pos_cost = ";
    for (unsigned s = 0; s < l_wrist_pos_cost.size(); s++)
      std::cout << l_wrist_pos_cost[s] << " ";
    std::cout << std::endl << std::endl;    
    // 
    std::cout << "debug: r_wrist_pos_cost = ";
    for (unsigned s = 0; s < r_wrist_pos_cost.size(); s++)
      std::cout << r_wrist_pos_cost[s] << " ";
    std::cout << std::endl << std::endl;    
    // 
    std::cout << "debug: wrist_ori_cost = ";
    for (unsigned s = 0; s < wrist_ori_cost.size(); s++)
      std::cout << wrist_ori_cost[s] << " ";
    std::cout << std::endl << std::endl;    
    // 
    std::cout << "debug: elbow_pos_cost = ";
    tmp_elbow_pos_cost = 0.0;
    for (unsigned s = 0; s < elbow_pos_cost.size(); s++)
    {
      std::cout << elbow_pos_cost[s] << " ";
      tmp_elbow_pos_cost += elbow_pos_cost[s];
    }
    std::cout << std::endl << std::endl;    
    cur_elbow_pos_cost = tmp_elbow_pos_cost;
    elbow_pos_cost_after_optim = tmp_elbow_pos_cost;
    // 
    std::cout << "debug: l_elbow_pos_cost = ";
    for (unsigned s = 0; s < l_elbow_pos_cost.size(); s++)
      std::cout << l_elbow_pos_cost[s] << " ";
    std::cout << std::endl << std::endl;    
    // 
    std::cout << "debug: r_elbow_pos_cost = ";
    for (unsigned s = 0; s < r_elbow_pos_cost.size(); s++)
      std::cout << r_elbow_pos_cost[s] << " ";
    std::cout << std::endl << std::endl;    
    // 
    std::cout << "debug: finger_cost = ";
    for (unsigned s = 0; s < finger_cost.size(); s++)
      std::cout << finger_cost[s] << " ";
    std::cout << std::endl << std::endl;    

    // Store jacobians results
    std::cout << ">> Jacobians <<" << std::endl;
    // output jacobians of wrist_pos, wrist_ori and elbow_pos costs for q vertices
    Matrix<double, 14, NUM_DATAPOINTS> wrist_pos_jacobian_for_q_arm;
    Matrix<double, 14, NUM_DATAPOINTS> wrist_ori_jacobian_for_q_arm;
    Matrix<double, 14, NUM_DATAPOINTS> elbow_pos_jacobian_for_q_arm;
    wrist_pos_jacobian_for_q_arm = tracking_edge->wrist_pos_jacobian_for_q_arm;
    wrist_ori_jacobian_for_q_arm = tracking_edge->wrist_ori_jacobian_for_q_arm;
    elbow_pos_jacobian_for_q_arm = tracking_edge->elbow_pos_jacobian_for_q_arm;
    std::cout << "Norms of wrist_pos_jacobian_for_q_arm = ";
    for (unsigned int n = 0; n < NUM_DATAPOINTS; n++)
    {
      std::cout << wrist_pos_jacobian_for_q_arm.block(0, n, 14, 1).norm() << " ";
    }
    std::cout << std::endl << std::endl;    
    std::cout << "Norms of wrist_ori_jacobian_for_q_arm = ";
    for (unsigned int n = 0; n < NUM_DATAPOINTS; n++)
    {
      std::cout << wrist_ori_jacobian_for_q_arm.block(0, n, 14, 1).norm() << " ";
    }
    std::cout << std::endl << std::endl;    
    std::cout << "Norms of elbow_pos_jacobian_for_q_arm = ";
    for (unsigned int n = 0; n < NUM_DATAPOINTS; n++)
    {
      std::cout << elbow_pos_jacobian_for_q_arm.block(0, n, 14, 1).norm() << " ";
    }
    std::cout << std::endl << std::endl;    
    // output collision jacobians for debug
    std::cout << "Norms of col_jacobians = ";
    for (unsigned int n = 0; n < NUM_DATAPOINTS; n++)
    {
      std::cout << unary_edges[n]->col_jacobians.norm() << " ";
    }
    std::cout << std::endl << std::endl;    
    // output position limit jacobians for debug
    std::cout << "Norms of pos_limit_jacobians = ";
    for (unsigned int n = 0; n < NUM_DATAPOINTS; n++)
    {
      std::cout << unary_edges[n]->pos_limit_jacobians.norm() << " ";
    }
    std::cout << std::endl << std::endl;    
    // output jacobians of Smoothness edge for q vertices
    std::cout << "Norms of smoothness_q_jacobian = ";
    Matrix<double, 2, JOINT_DOF> smoothness_q_jacobian;
    for (unsigned int n = 0; n < NUM_DATAPOINTS-1; n++)
    {
      smoothness_q_jacobian = smoothness_edges[n]->output_q_jacobian();
      std::cout << smoothness_q_jacobian.norm() << " ";
    }
    std::cout << std::endl << std::endl;  
    // output jacobians of unary edges for q vertices
    std::cout << "Norms of unary_jacobians = ";
    for (unsigned int n = 0; n < NUM_DATAPOINTS; n++)
    {
      std::cout << unary_edges[n]->whole_jacobians.norm() << " ";
    }
    std::cout << std::endl << std::endl;    
    // output jacobians of Tracking edge for q vertices
    std::cout << "Norms of tracking_q_jacobians = ";
    Matrix<double, NUM_DATAPOINTS, JOINT_DOF> q_jacobian = tracking_edge->output_q_jacobian();
    for (unsigned int n = 0; n < NUM_DATAPOINTS; n++)
    {
      std::cout << q_jacobian.block(n, 0, 1, JOINT_DOF).norm() << " ";
    }
    std::cout << std::endl << std::endl;    

    // check wrist cost
    std::cout << "Total wrist_cost = " << tmp_wrist_pos_cost << " (bound: " << wrist_pos_cost_bound << ")" << std::endl;
    std::cout << "Total elbow_cost = " << tmp_elbow_pos_cost << " (bound: " << elbow_pos_cost_bound << ")" << std::endl;
    std::cout << "(not concerned now) Total col_cost = " << tmp_col_cost << " (bound: " << col_cost_bound << ")" << std::endl;
    std::cout << "(not concerned now) Total pos_limit_cost = " << tmp_pos_limit_cost << " (bound: " << pos_limit_bound << ")" << std::endl;
    std::cout << "(not concerned now) Total smoothness_cost = " << tmp_smoothness_cost << " (bound: " << smoothness_bound << ")" << std::endl;
    std::cout << "Current coefficients: K_WRIST_POS = " << K_WRIST_POS << ", K_ELBOW_POS = " << K_ELBOW_POS << std::endl << std::endl;


    // evaluate updates under the current setting
    cur_wrist_pos_cost_update = wrist_pos_cost_before_optim - wrist_pos_cost_after_optim;
    cur_elbow_pos_cost_update = elbow_pos_cost_before_optim - elbow_pos_cost_after_optim;

    if (tmp_wrist_pos_cost > wrist_pos_cost_bound && K_WRIST_POS <= K_WRIST_POS_MAX)
    {
      // if (cur_wrist_pos_cost_update <= last_wrist_pos_cost_update) // update slowing down, increase coefficients
        if (cur_wrist_pos_cost >= last_wrist_pos_cost) // if it keeps descending, no need to rush
          K_WRIST_POS = K_WRIST_POS * inner_scale;// + step; //inner_scale * (++level); //* inner_scale;
    }
    last_wrist_pos_cost = cur_wrist_pos_cost;

    if (tmp_elbow_pos_cost > elbow_pos_cost_bound && K_ELBOW_POS <= K_ELBOW_POS_MAX)
    {
      // if (cur_elbow_pos_cost_update <= last_elbow_pos_cost_update) // update slowing down, increase coefficients
      if (cur_elbow_pos_cost >= last_elbow_pos_cost) // if descending, no need to rush
        K_ELBOW_POS = K_ELBOW_POS * inner_scale;// + step; //inner_scale * (++level); //* inner_scale;
    }
    last_elbow_pos_cost = cur_elbow_pos_cost;

    // update update information
    last_wrist_pos_cost_update = cur_wrist_pos_cost_update;
    last_elbow_pos_cost_update = cur_elbow_pos_cost_update;

    // test: use collision checking just one round, only for fast tracking convergence!!!
    // break;

    }while( (K_WRIST_POS <= K_WRIST_POS_MAX && tmp_wrist_pos_cost > wrist_pos_cost_bound) ||
            (K_ELBOW_POS <= K_ELBOW_POS_MAX && tmp_elbow_pos_cost > elbow_pos_cost_bound) );

    // std::cout << "Tracking loop done." << std::endl;


      // Initialize coefficients for collision loop
      // keep K_WRIST_POS and K_ELBOW_POS...
      K_WRIST_ORI = 1.0;
      K_WRIST_POS = 1.0;
      K_ELBOW_POS = 1.0;
      K_FINGER = 1.0; // keeping K_WRIST and K_ELBOW_POS as above doesn't work quite well... try setting low values, for ease of collision avoidance

      K_COL = 1.0; //50.0;//10.0;//100.0;//1.0;
      K_POS_LIMIT = 1.0;//10.0;//5.0;//10.0;//5.0; 
      K_SMOOTHNESS = 1.0;//2.0;//4.0;//2.0; 

    do{
    
      std::cout << ">>>> Constraints fixing loop: adjust K_COL, K_POS_LIMIT and K_SMOOTHNESS, with K_WRIST_POS and K_ELBOW_POS fixed." << std::endl;
      std::cout << "Current coefficient: K_COL = " << K_COL << std::endl;
      std::cout << "Current coefficient: K_POS_LIMIT = " << K_POS_LIMIT << std::endl;
      std::cout << "Current coefficient: K_SMOOTHNESS = " << K_SMOOTHNESS << std::endl;

    // optimize for a few iterations
    std::cout << "Optimizing q..." << std::endl;
    unsigned int q_iter = optimizer.optimize(q_col_per_iterations); 
    
    // Store cost results
    std::cout << ">> Costs <<" << std::endl;
    // collision 
    std::vector<double> col_cost;
    std::cout << "debug: col_cost = ";
    tmp_col_cost = 0.0;
    for (unsigned int t = 0; t < unary_edges.size(); t++)
    {
      double c = unary_edges[t]->return_col_cost();
      col_cost.push_back(c); // store
      tmp_col_cost += c; // check
      std::cout << c << " "; // display
    }
    std::cout << std::endl << std::endl;
    cur_col_cost = tmp_col_cost;
    col_cost_history.push_back(col_cost);
    // pos_limit
    std::vector<double> pos_limit_cost;
    std::cout << "debug: pos_limit_cost = ";
    tmp_pos_limit_cost = 0.0;
    for (unsigned int t = 0; t < unary_edges.size(); t++)
    {
      double p = unary_edges[t]->return_pos_limit_cost();
      pos_limit_cost.push_back(p); // store
      tmp_pos_limit_cost += p; // check
      std::cout << p << " "; // display      
    }
    std::cout << std::endl << std::endl;    
    cur_pos_limit_cost = tmp_pos_limit_cost;
    pos_limit_cost_history.push_back(pos_limit_cost);
    // smoothness
    std::vector<double> smoothness_cost;
    std::cout << "debug: smoothness_cost = ";
    tmp_smoothness_cost = 0.0;
    for (unsigned int t = 0; t < smoothness_edges.size(); t++)
    {
      double s = smoothness_edges[t]->return_smoothness_cost();
      smoothness_cost.push_back(s); // store
      tmp_smoothness_cost += s; // check
      std::cout << s << " "; // display
    }
    std::cout << std::endl << std::endl;     
    cur_smoothness_cost = tmp_smoothness_cost;   
    smoothness_cost_history.push_back(smoothness_cost);  
    // tracking   
    std::vector<double> wrist_pos_cost = tracking_edge->return_wrist_pos_cost_history();
    std::vector<double> l_wrist_pos_cost = tracking_edge->return_l_wrist_pos_cost_history();
    std::vector<double> r_wrist_pos_cost = tracking_edge->return_r_wrist_pos_cost_history();
    std::vector<double> wrist_ori_cost = tracking_edge->return_wrist_ori_cost_history();
    std::vector<double> elbow_pos_cost = tracking_edge->return_elbow_pos_cost_history();
    std::vector<double> l_elbow_pos_cost = tracking_edge->return_l_elbow_pos_cost_history();
    std::vector<double> r_elbow_pos_cost = tracking_edge->return_r_elbow_pos_cost_history();
    std::vector<double> finger_cost = tracking_edge->return_finger_cost_history(); 
    wrist_pos_cost_history.push_back(wrist_pos_cost);
    l_wrist_pos_cost_history.push_back(l_wrist_pos_cost);
    r_wrist_pos_cost_history.push_back(r_wrist_pos_cost);
    wrist_ori_cost_history.push_back(wrist_ori_cost);
    elbow_pos_cost_history.push_back(elbow_pos_cost);
    l_elbow_pos_cost_history.push_back(l_elbow_pos_cost);
    r_elbow_pos_cost_history.push_back(r_elbow_pos_cost);
    finger_cost_history.push_back(finger_cost); // store
    // display
    std::cout << "debug: wrist_pos_cost = ";
    tmp_wrist_pos_cost = 0.0;
    for (unsigned s = 0; s < wrist_pos_cost.size(); s++)
    {  
      std::cout << wrist_pos_cost[s] << " ";
      tmp_wrist_pos_cost += wrist_pos_cost[s];
    }
    std::cout << std::endl << std::endl;    
    //
    std::cout << "debug: l_wrist_pos_cost = ";
    for (unsigned s = 0; s < l_wrist_pos_cost.size(); s++)
      std::cout << l_wrist_pos_cost[s] << " ";
    std::cout << std::endl << std::endl;    
    // 
    std::cout << "debug: r_wrist_pos_cost = ";
    for (unsigned s = 0; s < r_wrist_pos_cost.size(); s++)
      std::cout << r_wrist_pos_cost[s] << " ";
    std::cout << std::endl << std::endl;    
    // 
    std::cout << "debug: wrist_ori_cost = ";
    for (unsigned s = 0; s < wrist_ori_cost.size(); s++)
      std::cout << wrist_ori_cost[s] << " ";
    std::cout << std::endl << std::endl;    
    // 
    std::cout << "debug: elbow_pos_cost = ";
    tmp_elbow_pos_cost = 0.0;
    for (unsigned s = 0; s < elbow_pos_cost.size(); s++)
    {
      std::cout << elbow_pos_cost[s] << " ";
      tmp_elbow_pos_cost += elbow_pos_cost[s];
    }
    std::cout << std::endl << std::endl;    
    // 
    std::cout << "debug: l_elbow_pos_cost = ";
    for (unsigned s = 0; s < l_elbow_pos_cost.size(); s++)
      std::cout << l_elbow_pos_cost[s] << " ";
    std::cout << std::endl << std::endl;    
    // 
    std::cout << "debug: r_elbow_pos_cost = ";
    for (unsigned s = 0; s < r_elbow_pos_cost.size(); s++)
      std::cout << r_elbow_pos_cost[s] << " ";
    std::cout << std::endl << std::endl;    
    // 
    std::cout << "debug: finger_cost = ";
    for (unsigned s = 0; s < finger_cost.size(); s++)
      std::cout << finger_cost[s] << " ";
    std::cout << std::endl << std::endl;    

    // Store jacobians results
    std::cout << ">> Jacobians <<" << std::endl;
    // output jacobians of wrist_pos, wrist_ori and elbow_pos costs for q vertices
    Matrix<double, 14, NUM_DATAPOINTS> wrist_pos_jacobian_for_q_arm;
    Matrix<double, 14, NUM_DATAPOINTS> wrist_ori_jacobian_for_q_arm;
    Matrix<double, 14, NUM_DATAPOINTS> elbow_pos_jacobian_for_q_arm;
    wrist_pos_jacobian_for_q_arm = tracking_edge->wrist_pos_jacobian_for_q_arm;
    wrist_ori_jacobian_for_q_arm = tracking_edge->wrist_ori_jacobian_for_q_arm;
    elbow_pos_jacobian_for_q_arm = tracking_edge->elbow_pos_jacobian_for_q_arm;
    std::cout << "Norms of wrist_pos_jacobian_for_q_arm = ";
    for (unsigned int n = 0; n < NUM_DATAPOINTS; n++)
    {
      std::cout << wrist_pos_jacobian_for_q_arm.block(0, n, 14, 1).norm() << " ";
    }
    std::cout << std::endl << std::endl;    
    std::cout << "Norms of wrist_ori_jacobian_for_q_arm = ";
    for (unsigned int n = 0; n < NUM_DATAPOINTS; n++)
    {
      std::cout << wrist_ori_jacobian_for_q_arm.block(0, n, 14, 1).norm() << " ";
    }
    std::cout << std::endl << std::endl;    
    std::cout << "Norms of elbow_pos_jacobian_for_q_arm = ";
    for (unsigned int n = 0; n < NUM_DATAPOINTS; n++)
    {
      std::cout << elbow_pos_jacobian_for_q_arm.block(0, n, 14, 1).norm() << " ";
    }
    std::cout << std::endl << std::endl;    
    // output collision jacobians for debug
    std::cout << "Norms of col_jacobians = ";
    for (unsigned int n = 0; n < NUM_DATAPOINTS; n++)
    {
      std::cout << unary_edges[n]->col_jacobians.norm() << " ";
    }
    std::cout << std::endl << std::endl;    
    // output position limit jacobians for debug
    std::cout << "Norms of pos_limit_jacobians = ";
    for (unsigned int n = 0; n < NUM_DATAPOINTS; n++)
    {
      std::cout << unary_edges[n]->pos_limit_jacobians.norm() << " ";
    }
    std::cout << std::endl << std::endl;    
    // output jacobians of Smoothness edge for q vertices
    std::cout << "Norms of smoothness_q_jacobian = ";
    Matrix<double, 2, JOINT_DOF> smoothness_q_jacobian;
    for (unsigned int n = 0; n < NUM_DATAPOINTS-1; n++)
    {
      smoothness_q_jacobian = smoothness_edges[n]->output_q_jacobian();
      std::cout << smoothness_q_jacobian.norm() << " ";
    }
    std::cout << std::endl; 
    // output jacobians of unary edges for q vertices
    std::cout << "Norms of unary_jacobians = ";
    for (unsigned int n = 0; n < NUM_DATAPOINTS; n++)
    {
      std::cout << unary_edges[n]->whole_jacobians.norm() << " ";
    }
    std::cout << std::endl << std::endl;    
    // output jacobians of Tracking edge for q vertices
    std::cout << "Norms of tracking_q_jacobians = ";
    Matrix<double, NUM_DATAPOINTS, JOINT_DOF> q_jacobian = tracking_edge->output_q_jacobian();
    for (unsigned int n = 0; n < NUM_DATAPOINTS; n++)
    {
      std::cout << q_jacobian.block(n, 0, 1, JOINT_DOF).norm() << " ";
    }
    std::cout << std::endl << std::endl;    


    // Checking collision situation
    /*
    std::cout << ">> Collision condition <<" << std::endl;
    for (unsigned int n = 0; n < NUM_DATAPOINTS; n++)
    {
      std::cout << ">> Path point " << (n+1) << "/" << NUM_DATAPOINTS << " <<" << std::endl;
      unary_edges[n]->output_distance_result();
      std::cout << "col_jacobians = " << unary_edges[n]->col_jacobians.transpose() << std::endl;
    }
    */
    

    // Conclusion
    std::cout << "Deciding adjustment on K_COL, K_POS_LIMIT and K_SMOOTHNESS..." << std::endl;
    std::cout << "(not concerned now) Total wrist_cost = " << tmp_wrist_pos_cost << " (bound: " << wrist_pos_cost_bound << ")" << std::endl;
    std::cout << "(not concerned now) Total elbow_cost = " << tmp_elbow_pos_cost << " (bound: " << elbow_pos_cost_bound << ")" << std::endl;
    std::cout << "Total col_cost = " << tmp_col_cost << " (bound: " << col_cost_bound << ")" << std::endl;
    std::cout << "Total pos_limit_cost = " << tmp_pos_limit_cost << " (bound: " << pos_limit_bound << ")" << std::endl;
    std::cout << "Total smoothness_cost = " << tmp_smoothness_cost << " (bound: " << smoothness_bound << ")" << std::endl;
    std::cout << "Current coefficients: K_COL = " << K_COL << ", K_POS_LIMIT = " << K_POS_LIMIT << ", K_SMOOTHNESS = " << K_SMOOTHNESS << std::endl;

    // check if constraints met, and automatically adjust weights
    if (tmp_col_cost > col_cost_bound && K_COL <= K_COL_MAX) 
    { 
      if (cur_col_cost >= last_col_cost) // if it's descending, not need to rush 
        K_COL = K_COL * outer_scale;
      // K_COL = K_COL + step;
    }
    last_col_cost = cur_col_cost;

    if (tmp_smoothness_cost > smoothness_bound && K_SMOOTHNESS <= K_SMOOTHNESS_MAX) 
    {
      if (cur_smoothness_cost >= last_smoothness_cost) // if not descending, or even increase, increase the weight
        K_SMOOTHNESS = K_SMOOTHNESS * outer_scale;
      // K_SMOOTHNESS = K_SMOOTHNESS + step;
    }
    last_smoothness_cost = cur_smoothness_cost;

    if (tmp_pos_limit_cost > pos_limit_bound && K_POS_LIMIT <= K_POS_LIMIT_MAX) 
    {
      if (cur_pos_limit_cost >= last_pos_limit_cost) // if not descending, or even rise up, increase the weight
        K_POS_LIMIT = K_POS_LIMIT * outer_scale;
      // K_POS_LIMIT = K_POS_LIMIT + step;
    }
    last_pos_limit_cost = cur_pos_limit_cost;

  
    }while( (K_COL <= K_COL_MAX && tmp_col_cost > col_cost_bound) || 
            (K_SMOOTHNESS <= K_SMOOTHNESS_MAX && tmp_smoothness_cost > smoothness_bound) || 
            (K_POS_LIMIT <= K_POS_LIMIT_MAX && tmp_pos_limit_cost > pos_limit_bound)); // when coefficients still within bounds, and costs still out of bounds
    
    std::cout << "Final weights: K_COL = " << K_COL 
              << ", K_SMOOTHNESS = " << K_SMOOTHNESS 
              << ", K_POS_LIMIT = " << K_POS_LIMIT 
              << ", K_WRIST_POS = " << K_WRIST_POS
              << ", K_ELBOW_POS = " << K_ELBOW_POS
              << ", with col_cost = " << tmp_col_cost
              << ", smoothness_cost = " << tmp_smoothness_cost
              << ", pos_limit_cost = " << tmp_pos_limit_cost 
              << ", wrist_cost = " << tmp_wrist_pos_cost 
              << ", elbow_cost = " << tmp_elbow_pos_cost << std::endl << std::endl;


    // reset
    std::cout << "Re-activate DMP vertex." << std::endl;
    dmp_vertex_tmp->setFixed(false);


    // store actually executed trajectories
    // Perform FK on pre-iteration results (output functions are alread implemented in TrackingConstraint)
    std::vector<std::vector<double>> l_wrist_pos_traj, r_wrist_pos_traj, l_elbow_pos_traj, r_elbow_pos_traj;
    l_wrist_pos_traj = tracking_edge->return_wrist_pos_traj(true);
    r_wrist_pos_traj = tracking_edge->return_wrist_pos_traj(false);
    l_elbow_pos_traj = tracking_edge->return_elbow_pos_traj(true);
    r_elbow_pos_traj = tracking_edge->return_elbow_pos_traj(false);
    std::cout << ">>>> Storing current executed Cartesian trajectories to h5 file" << std::endl;
    bool tmp_flag = write_h5(out_file_name, in_group_name, "actual_l_wrist_pos_traj_"+std::to_string(n), \
                             l_wrist_pos_traj.size(), l_wrist_pos_traj[0].size(), l_wrist_pos_traj);
    std::cout << "actual_l_wrist_pos_traj stored " << (tmp_flag ? "successfully" : "unsuccessfully") << "!" << std::endl;
      
    tmp_flag = write_h5(out_file_name, in_group_name, "actual_r_wrist_pos_traj_"+std::to_string(n), \
                        r_wrist_pos_traj.size(), r_wrist_pos_traj[0].size(), r_wrist_pos_traj);
    std::cout << "actual_r_wrist_pos_traj stored " << (tmp_flag ? "successfully" : "unsuccessfully") << "!" << std::endl;
      
    tmp_flag = write_h5(out_file_name, in_group_name, "actual_l_elbow_pos_traj_"+std::to_string(n), \
                        l_elbow_pos_traj.size(), l_elbow_pos_traj[0].size(), l_elbow_pos_traj);
    std::cout << "actual_l_elbow_pos_traj stored " << (tmp_flag ? "successfully" : "unsuccessfully") << "!" << std::endl;
      
    tmp_flag = write_h5(out_file_name, in_group_name, "actual_r_elbow_pos_traj_"+std::to_string(n), \
                        r_elbow_pos_traj.size(), r_elbow_pos_traj[0].size(), r_elbow_pos_traj);
    std::cout << "actual_r_elbow_pos_traj stored " << (tmp_flag ? "successfully" : "unsuccessfully") << "!" << std::endl;
    

    // Check stopping criteria (q and DMP!!!)
    if (tmp_col_cost <= col_cost_bound && 
        tmp_pos_limit_cost <= pos_limit_bound && 
        tmp_smoothness_cost <= smoothness_bound)
    {
      if (n!=0) // not for the first round
      {
        // check dmp
        double tmp_dmp_orien_cost = dmp_edge->output_orien_cost();
        double tmp_dmp_scale_cost = dmp_edge->output_scale_cost();
        double tmp_dmp_rel_change_cost = dmp_edge->output_rel_change_cost();

        if (tmp_dmp_orien_cost <= dmp_orien_cost_bound &&
            tmp_dmp_scale_cost <= dmp_scale_cost_bound &&
            tmp_dmp_rel_change_cost <= dmp_rel_change_cost_bound)
          break;
      }
    }



    // 2 - Manually move DMPs starts and goals; 
    /*
    std::cout << ">>>> Round " << (n+1) << "/" << num_rounds << ", Manually set initial guess for rw DMP: "<< std::endl;
    // get current estimates on relative DMPs
    DMPStartsGoalsVertex *dmp_vertex_tmp_tmp = dynamic_cast<DMPStartsGoalsVertex*>(optimizer.vertex(0)); // the last vertex connected
    Matrix<double, DMPPOINTS_DOF, 1> xx = dmp_vertex_tmp_tmp->estimate();
    MatrixXd lrw_new_goal(3, 1); lrw_new_goal = xx.block(0, 0, 3, 1);
    MatrixXd lrw_new_start(3, 1); lrw_new_start = xx.block(3, 0, 3, 1);
    MatrixXd lew_new_goal(3, 1); lew_new_goal = xx.block(6, 0, 3, 1);
    MatrixXd lew_new_start(3, 1); lew_new_start = xx.block(9, 0, 3, 1);
    MatrixXd rew_new_goal(3, 1); rew_new_goal = xx.block(12, 0, 3, 1);
    MatrixXd rew_new_start(3, 1); rew_new_start = xx.block(15, 0, 3, 1);
    MatrixXd rw_new_goal(3, 1); rw_new_goal = xx.block(18, 0, 3, 1);
    MatrixXd rw_new_start(3, 1); rw_new_start = xx.block(21, 0, 3, 1);
    // current starts and goals for wrists and elbows
    MatrixXd lw_new_goal(3, 1); lw_new_goal = rw_new_goal + lrw_new_goal;
    MatrixXd lw_new_start(3, 1); lw_new_start = rw_new_start + lrw_new_start;
    MatrixXd re_new_goal(3, 1); re_new_goal = rw_new_goal + rew_new_goal;
    MatrixXd re_new_start(3, 1); re_new_start = rw_new_start + rew_new_start;
    MatrixXd le_new_goal(3, 1); le_new_goal = lw_new_goal + lew_new_goal;
    MatrixXd le_new_start(3, 1); le_new_start = lw_new_start + lew_new_start;
    
    // set new goals and starts
    MatrixXd wrist_pos_offsets(3, NUM_DATAPOINTS), elbow_pos_offsets(3, NUM_DATAPOINTS);    
    // - right wrist
    wrist_pos_offsets = tracking_edge->return_r_wrist_pos_offsets(); 
    Vector3d wrist_pos_offset = wrist_pos_offsets.rowwise().mean();        
    rw_new_goal = rw_new_goal + wrist_pos_offset;
    rw_new_start = rw_new_start + wrist_pos_offset; 
    // - right elbow
    elbow_pos_offsets = tracking_edge->return_r_elbow_pos_offsets(); 
    Vector3d elbow_pos_offset = elbow_pos_offsets.rowwise().mean();    
    re_new_goal = re_new_goal + elbow_pos_offset;
    re_new_start = re_new_start + elbow_pos_offset;
     // - left wrist
    wrist_pos_offsets = tracking_edge->return_l_wrist_pos_offsets(); 
    wrist_pos_offset = wrist_pos_offsets.rowwise().mean();        
    lw_new_goal = lw_new_goal + wrist_pos_offset;
    lw_new_start = lw_new_start + wrist_pos_offset; 
    // - left elbow
    elbow_pos_offsets = tracking_edge->return_l_elbow_pos_offsets(); 
    elbow_pos_offset = elbow_pos_offsets.rowwise().mean();    
    le_new_goal = le_new_goal + elbow_pos_offset;
    le_new_start = le_new_start + elbow_pos_offset;

    // re-calculate DMP starts and goals
    lrw_new_goal = lw_new_goal - rw_new_goal;
    lrw_new_start = lw_new_start - rw_new_start;
    lew_new_goal = le_new_goal - lw_new_goal;
    lew_new_start = le_new_start - lw_new_start;    
    rew_new_goal = re_new_goal - rw_new_goal;
    rew_new_start = re_new_start - rw_new_start;

    // assign initial guess (change only the starts and goals of right wrist traj ?)
    xx.block(0, 0, 3, 1) = lrw_new_goal;
    xx.block(3, 0, 3, 1) = lrw_new_start;
    xx.block(6, 0, 3, 1) = lew_new_goal;
    xx.block(9, 0, 3, 1) = lew_new_start;
    xx.block(12, 0, 3, 1) = rew_new_goal;
    xx.block(15, 0, 3, 1) = rew_new_start;
    xx.block(18, 0, 3, 1) = rw_new_goal;
    xx.block(21, 0, 3, 1) = rw_new_start;
    dmp_vertex_tmp_tmp->setEstimate(xx);

    // store the manually moved DMP starts and goals
    // initialization
    std::vector<std::vector<double> > dmp_starts_goals_moved_vec_vec;
    std::vector<double> dmp_starts_goals_moved_vec(DMPPOINTS_DOF);
    // convert
    for (unsigned int d = 0; d < DMPPOINTS_DOF; d++)
        dmp_starts_goals_moved_vec[d] = xx[d];
    dmp_starts_goals_moved_vec_vec.push_back(dmp_starts_goals_moved_vec);
    // store and display results
    std::cout << "dmp_results size is: " << dmp_starts_goals_moved_vec_vec.size() << " x " << dmp_starts_goals_moved_vec_vec[0].size() << std::endl;
    std::cout << "dmp_results = " << xx.transpose() << std::endl;
    bool result_moved = write_h5(out_file_name, in_group_name, "dmp_starts_goals_moved_"+std::to_string(n),
                                 dmp_starts_goals_moved_vec_vec.size(), dmp_starts_goals_moved_vec_vec[0].size(), 
                                 dmp_starts_goals_moved_vec_vec);
    std::cout << "moved dmp starts and goals stored " << (result_moved ? "successfully" : "unsuccessfully") << "!" << std::endl;

    */


    // 3 - optimize DMP for a number of iterations
    std::cout << ">>>> Round " << (n+1) << "/" << num_rounds << ", DMP stage: "<< std::endl;
    K_WRIST_ORI = 0.1;  // actually useless since orientation data are specified independent of DMP !!! (the jacobian test checks it out!!)
    K_WRIST_POS = 0.1; 
    K_ELBOW_POS = 0.1; 
    K_FINGER = 0.1;  // actually useless since finger data are specified independent of DMP !!!
    K_SIMILARITY = 100.0;  // nothing actually...
    K_DMPSTARTSGOALS = 0.1;//0.5;//1.0;//2.0;
    K_DMPSCALEMARGIN = 0.1; //0.5;//1.0;//2.0;
    K_DMPRELCHANGE = 0.1; //2.0;//1.0; // relax a little to allow small variance
    std::cout << "Fix q vertices." << std::endl;
    // fix q vertices
    for (unsigned int m = 0; m < NUM_DATAPOINTS; m++)
    {
      DualArmDualHandVertex* vertex_tmp = dynamic_cast<DualArmDualHandVertex*>(optimizer.vertex(1+m));
      vertex_tmp->setFixed(true);
    }

    double tmp_dmp_orien_cost;
    double tmp_dmp_scale_cost;
    double tmp_dmp_rel_change_cost;

    do
    {

    // iterate to optimize DMP
    std::cout << "Optimizing DMP..." << std::endl;
    unsigned int dmp_iter = optimizer.optimize(dmp_per_iterations);
    // Save cost history
    std::cout << ">>>> Costs <<<<" << std::endl;
    // 1)
    std::cout << "Recording tracking cost, similarity cost and DMP costs..." << std::endl;
    // 3)  
    std::vector<double> wrist_pos_cost = tracking_edge->return_wrist_pos_cost_history();
    std::vector<double> l_wrist_pos_cost = tracking_edge->return_l_wrist_pos_cost_history();
    std::vector<double> r_wrist_pos_cost = tracking_edge->return_r_wrist_pos_cost_history();
    std::vector<double> wrist_ori_cost = tracking_edge->return_wrist_ori_cost_history();
    std::vector<double> elbow_pos_cost = tracking_edge->return_elbow_pos_cost_history();
    std::vector<double> l_elbow_pos_cost = tracking_edge->return_l_elbow_pos_cost_history();
    std::vector<double> r_elbow_pos_cost = tracking_edge->return_r_elbow_pos_cost_history();
    std::vector<double> finger_cost = tracking_edge->return_finger_cost_history();      
    wrist_pos_cost_history.push_back(wrist_pos_cost);
    l_wrist_pos_cost_history.push_back(l_wrist_pos_cost);
    r_wrist_pos_cost_history.push_back(r_wrist_pos_cost);
    wrist_ori_cost_history.push_back(wrist_ori_cost);
    elbow_pos_cost_history.push_back(elbow_pos_cost);
    l_elbow_pos_cost_history.push_back(l_elbow_pos_cost);
    r_elbow_pos_cost_history.push_back(r_elbow_pos_cost);
    finger_cost_history.push_back(finger_cost); // store
    // display for debug:
    // 
    std::cout << "debug: wrist_pos_cost = ";
    for (unsigned s = 0; s < wrist_pos_cost.size(); s++)
      std::cout << wrist_pos_cost[s] << " ";
    std::cout << std::endl;
    // 
    std::cout << "debug: l_wrist_pos_cost = ";
    for (unsigned s = 0; s < l_wrist_pos_cost.size(); s++)
      std::cout << l_wrist_pos_cost[s] << " ";
    std::cout << std::endl;
    // 
    std::cout << "debug: r_wrist_pos_cost = ";
    for (unsigned s = 0; s < r_wrist_pos_cost.size(); s++)
      std::cout << r_wrist_pos_cost[s] << " ";
    std::cout << std::endl;
    // 
    std::cout << "debug: wrist_ori_cost = ";
    for (unsigned s = 0; s < wrist_ori_cost.size(); s++)
      std::cout << wrist_ori_cost[s] << " ";
    std::cout << std::endl;
    // 
    std::cout << "debug: elbow_pos_cost = ";
    for (unsigned s = 0; s < elbow_pos_cost.size(); s++)
      std::cout << elbow_pos_cost[s] << " ";
    std::cout << std::endl;
    // 
    std::cout << "debug: l_elbow_pos_cost = ";
    for (unsigned s = 0; s < l_elbow_pos_cost.size(); s++)
      std::cout << l_elbow_pos_cost[s] << " ";
    std::cout << std::endl;                
    // 
    std::cout << "debug: r_elbow_pos_cost = ";
    for (unsigned s = 0; s < r_elbow_pos_cost.size(); s++)
      std::cout << r_elbow_pos_cost[s] << " ";
    std::cout << std::endl;  
    // 
    std::cout << "debug: finger_cost = ";
    for (unsigned s = 0; s < finger_cost.size(); s++)
      std::cout << finger_cost[s] << " ";
    std::cout << std::endl;      
    // 4)
    // std::vector<double> similarity_cost;
    // similarity_cost.push_back(similarity_edge->return_similarity_cost());
    // similarity_cost_history.push_back(similarity_cost);
    // std::cout << "debug: similarity_cost = ";
    // for (unsigned s = 0; s < similarity_cost.size(); s++)
    //   std::cout << similarity_cost[s] << " ";
    // std::cout << std::endl;       
    // 5)
    std::vector<double> dmp_orien_cost;
    dmp_orien_cost.push_back(dmp_edge->output_orien_cost());
    dmp_orien_cost_history.push_back(dmp_orien_cost);
    std::vector<double> dmp_scale_cost;
    dmp_scale_cost.push_back(dmp_edge->output_scale_cost());
    dmp_scale_cost_history.push_back(dmp_scale_cost);
    std::vector<double> dmp_rel_change_cost;
    dmp_rel_change_cost.push_back(dmp_edge->output_rel_change_cost());
    dmp_rel_change_cost_history.push_back(dmp_rel_change_cost);
    // 
    std::cout << "debug: dmp_orien_cost = ";
    for (unsigned s = 0; s < dmp_orien_cost.size(); s++)
      std::cout << dmp_orien_cost[s] << " ";
    std::cout << std::endl;       
    // 
    std::cout << "debug: dmp_scale_cost = ";
    for (unsigned s = 0; s < dmp_scale_cost.size(); s++)
      std::cout << dmp_scale_cost[s] << " ";
    std::cout << std::endl;    
    // 
    std::cout << "debug: dmp_rel_change_cost = ";
    for (unsigned s = 0; s < dmp_rel_change_cost.size(); s++)
      std::cout << dmp_rel_change_cost[s] << " ";
    std::cout << std::endl;    


    // Save Jacobians of constraints w.r.t DMP starts and goals
    std::cout << ">>>> Jacobians <<<<" << std::endl;
    std::cout << "Recording DMP jacobians.." << std::endl;
    MatrixXd jacobians(1, DMPPOINTS_DOF);    
    // 1)
    // jacobians = similarity_edge->output_jacobian();
    std::vector<double> jacobian_vec(DMPPOINTS_DOF); // from MatrixXd to std::vector<std::vector<double>>
    // std::cout << "debug: sim_jacobian = ";
    // for (unsigned int j = 0; j < DMPPOINTS_DOF; j++)
    // {
    //   jacobian_vec[j] = jacobians(0, j);
    //   std::cout << jacobians(0, j) << " ";
    // }
    // std::cout << std::endl;
    // sim_jacobian_history.push_back(jacobian_vec);
    // 2)
    jacobians = tracking_edge->output_dmp_jacobian();
    std::cout << "debug: track_jacobian = ";
    for (unsigned int j = 0; j < DMPPOINTS_DOF; j++)
    {
      jacobian_vec[j] = jacobians(0, j);
      std::cout << jacobians(0, j) << " ";
    }
    std::cout << std::endl;
    track_jacobian_history.push_back(jacobian_vec);
    // 3)
    jacobians = dmp_edge->output_orien_jacobian();
    std::cout << "debug: orien_jacobian = ";    
    for (unsigned int j = 0; j < DMPPOINTS_DOF; j++)
    {
      jacobian_vec[j] = jacobians(0, j);
      std::cout << jacobians(0, j) << " ";
    }
    std::cout << std::endl;
    orien_jacobian_history.push_back(jacobian_vec);    
    // 4)
    jacobians = dmp_edge->output_scale_jacobian();
    std::cout << "debug: scale_jacobian = ";    
    for (unsigned int j = 0; j < DMPPOINTS_DOF; j++)
    {
      jacobian_vec[j] = jacobians(0, j);
      std::cout << jacobians(0, j) << " ";      
    }
    std::cout << std::endl;
    scale_jacobian_history.push_back(jacobian_vec);    
    // 5)
    jacobians = dmp_edge->output_rel_change_jacobian();
    std::cout << "debug: rel_change_jacobian = ";    
    for (unsigned int j = 0; j < DMPPOINTS_DOF; j++)
    {
      jacobian_vec[j] = jacobians(0, j);
      std::cout << jacobians(0, j) << " ";      
    }
    std::cout << std::endl;
    rel_change_jacobian_history.push_back(jacobian_vec);    

    // output jacobians for wrist_pos, wrist_ori and elbow_pos costs for DMP vertex
    Matrix<double, DMPPOINTS_DOF, 1> wrist_pos_jacobian_for_dmp;
    Matrix<double, DMPPOINTS_DOF, 1> wrist_ori_jacobian_for_dmp;
    Matrix<double, DMPPOINTS_DOF, 1> elbow_pos_jacobian_for_dmp;
    Matrix<double, DMPPOINTS_DOF, 1> finger_pos_jacobian_for_dmp;
    wrist_pos_jacobian_for_dmp = tracking_edge->wrist_pos_jacobian_for_dmp;
    wrist_ori_jacobian_for_dmp = tracking_edge->wrist_ori_jacobian_for_dmp;
    elbow_pos_jacobian_for_dmp = tracking_edge->elbow_pos_jacobian_for_dmp;    
    finger_pos_jacobian_for_dmp = tracking_edge->finger_pos_jacobian_for_dmp;    
    std::cout << "debug: wrist_pos_jacobian_for_dmp = " << wrist_pos_jacobian_for_dmp.transpose() << std::endl;
    std::cout << "debug: wrist_ori_jacobian_for_dmp = " << wrist_ori_jacobian_for_dmp.transpose() << std::endl;
    std::cout << "debug: elbow_pos_jacobian_for_dmp = " << elbow_pos_jacobian_for_dmp.transpose() << std::endl;
    std::cout << "debug: finger_pos_jacobian_for_dmp = " << finger_pos_jacobian_for_dmp.transpose() << std::endl;


    // store dmp updates
    std::cout << "Recording DMP updates.." << std::endl;
    std::vector<double> dmp_update_vec(DMPPOINTS_DOF);
    // DMPStartsGoalsVertex* dmp_vertex_tmp = dynamic_cast<DMPStartsGoalsVertex*>(optimizer.vertex(0));    
    Matrix<double, DMPPOINTS_DOF, 1> last_update = dmp_vertex_tmp->last_update;
    for (unsigned int j = 0; j < DMPPOINTS_DOF; j++)
      dmp_update_vec[j] = last_update(j, 0);
    dmp_update_history.push_back(dmp_update_vec);


    // check dmp constratins
    tmp_dmp_orien_cost = dmp_edge->output_orien_cost();
    tmp_dmp_scale_cost = dmp_edge->output_scale_cost();
    tmp_dmp_rel_change_cost = dmp_edge->output_rel_change_cost();

    if (tmp_dmp_orien_cost > dmp_orien_cost_bound && K_DMPSTARTSGOALS <= K_DMPSTARTSGOALS_MAX)
    {
      K_DMPSTARTSGOALS = K_DMPSTARTSGOALS * scale;
    }

    if (tmp_dmp_scale_cost > dmp_scale_cost_bound && K_DMPSCALEMARGIN <= K_DMPSCALEMARGIN_MAX)
    {
      K_DMPSCALEMARGIN = K_DMPSCALEMARGIN * scale;
    }

    if (tmp_dmp_rel_change_cost > dmp_rel_change_cost_bound && K_DMPRELCHANGE <= K_DMPRELCHANGE_MAX)
    {
      K_DMPRELCHANGE = K_DMPRELCHANGE * scale;
    }

    }while( (K_DMPSTARTSGOALS <= K_DMPSTARTSGOALS_MAX && tmp_dmp_orien_cost > dmp_orien_cost_bound) || 
            (K_DMPSCALEMARGIN <= K_DMPSCALEMARGIN_MAX && tmp_dmp_scale_cost > dmp_scale_cost_bound) || 
            (K_DMPRELCHANGE <= K_DMPRELCHANGE_MAX && tmp_dmp_rel_change_cost > dmp_rel_change_cost_bound) );


    // store the optimized DMP starts and goals
    // initialization
    Matrix<double, DMPPOINTS_DOF, 1> dmp_starts_goals_optimed;
    std::vector<std::vector<double> > dmp_starts_goals_optimed_vec_vec;
    std::vector<double> dmp_starts_goals_optimed_vec(DMPPOINTS_DOF);
    dmp_starts_goals_optimed = dmp_vertex_tmp->estimate();
    // convert
    for (unsigned int d = 0; d < DMPPOINTS_DOF; d++)
        dmp_starts_goals_optimed_vec[d] = dmp_starts_goals_optimed[d];
    dmp_starts_goals_optimed_vec_vec.push_back(dmp_starts_goals_optimed_vec);
    // store and display results
    std::cout << "dmp_results size is: " << dmp_starts_goals_optimed_vec_vec.size() << " x " << dmp_starts_goals_optimed_vec_vec[0].size() << std::endl;
    std::cout << "dmp_results = " << dmp_starts_goals_optimed.transpose() << std::endl;
    bool result_optimed = write_h5(out_file_name, in_group_name, "dmp_starts_goals_optimed_"+std::to_string(n), 
                                   dmp_starts_goals_optimed_vec_vec.size(), dmp_starts_goals_optimed_vec_vec[0].size(), 
                                   dmp_starts_goals_optimed_vec_vec);
    std::cout << "optimized dmp starts and goals stored " << (result_optimed ? "successfully" : "unsuccessfully") << "!" << std::endl;

    // Reset q vertices
    std::cout << "Re-activate q vertices." << std::endl;
    for (unsigned int m = 0; m < NUM_DATAPOINTS; m++)
    {
      DualArmDualHandVertex* vertex_tmp = dynamic_cast<DualArmDualHandVertex*>(optimizer.vertex(1+m));
      vertex_tmp->setFixed(false);
    }

    // record for ease
    max_round = n;

    // Terminate the process if conditions met
    std::cout << ">>>> Check terminate conditions: " << std::endl;
    // collision 
    double cur_col_cost_check = 0.0;
    std::cout << "Current collision costs = ";
    for (unsigned int t = 0; t < unary_edges.size(); t++)
    {
      double c = unary_edges[t]->return_col_cost();
      cur_col_cost_check += c; // check
      std::cout << c << " "; // display
    }
    std::cout << "\nTotal col_cost = " << cur_col_cost_check << std::endl;
    // pos_limit
    std::cout << "Current pos_limit_cost = ";
    double cur_pos_limit_cost_check = 0.0;
    for (unsigned int t = 0; t < unary_edges.size(); t++)
    {
      double p = unary_edges[t]->return_pos_limit_cost();
      cur_pos_limit_cost_check += p; // check
      std::cout << p << " "; // display      
    }
    std::cout << "\nTotal pos_limit_cost = " << cur_pos_limit_cost_check << std::endl;    
    // smoothness
    std::cout << "Current smoothness_cost = ";
    double cur_smoothness_cost_check = 0.0;
    for (unsigned int t = 0; t < smoothness_edges.size(); t++)
    {
      double s = smoothness_edges[t]->return_smoothness_cost();
      cur_smoothness_cost_check += s; // check
      std::cout << s << " "; // display
    }
    std::cout << "\nTotal smoothness_cost = " << cur_smoothness_cost_check << std::endl;

    if(cur_col_cost_check < col_cost_bound && cur_pos_limit_cost_check < pos_limit_bound && cur_smoothness_cost_check < smoothness_bound) 
    {
      std::cout << ">>>>>>>> Terminate condition met. Optimization stopped after " << n+1 << " rounds." << std::endl;
      break;
    }


  }

  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  std::chrono::duration<double> t_spent = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  std::cout << "Total time used for optimization: " << t_spent.count() << " s" << std::endl;

  //saveFlag = optimizer.save("./g2o_results/result_after.g2o");
  //std::cout << "g2o file saved " << (saveFlag? "successfully" : "unsuccessfully") << " ." << std::endl;

  std::cout << ">>>> Optimization done." << std::endl;

  // Store the actually executed Cartesian trajectories (after optimization)
  /*
  std::vector<std::vector<double>> l_wrist_pos_traj, r_wrist_pos_traj, l_elbow_pos_traj, r_elbow_pos_traj;
  l_wrist_pos_traj = tracking_edge->return_wrist_pos_traj(true);
  r_wrist_pos_traj = tracking_edge->return_wrist_pos_traj(false);
  l_elbow_pos_traj = tracking_edge->return_elbow_pos_traj(true);
  r_elbow_pos_traj = tracking_edge->return_elbow_pos_traj(false);
  std::cout << ">>>> Storing current executed Cartesian trajectories to h5 file" << std::endl;
  bool tmp_flag = write_h5(out_file_name, in_group_name, "optimed_l_wrist_pos_traj", \
                           l_wrist_pos_traj.size(), l_wrist_pos_traj[0].size(), l_wrist_pos_traj);
  std::cout << "optimed_l_wrist_pos_traj stored " << (tmp_flag ? "successfully" : "unsuccessfully") << "!" << std::endl;
  
  tmp_flag = write_h5(out_file_name, in_group_name, "optimed_r_wrist_pos_traj", \
                           r_wrist_pos_traj.size(), r_wrist_pos_traj[0].size(), r_wrist_pos_traj);
  std::cout << "optimed_r_wrist_pos_traj stored " << (tmp_flag ? "successfully" : "unsuccessfully") << "!" << std::endl;
  
  tmp_flag = write_h5(out_file_name, in_group_name, "optimed_l_elbow_pos_traj", \
                           l_elbow_pos_traj.size(), l_elbow_pos_traj[0].size(), l_elbow_pos_traj);
  std::cout << "optimed_l_elbow_pos_traj stored " << (tmp_flag ? "successfully" : "unsuccessfully") << "!" << std::endl;
  
  tmp_flag = write_h5(out_file_name, in_group_name, "optimed_r_elbow_pos_traj", \
                           r_elbow_pos_traj.size(), r_elbow_pos_traj[0].size(), r_elbow_pos_traj);
  std::cout << "optimed_r_elbow_pos_traj stored " << (tmp_flag ? "successfully" : "unsuccessfully") << "!" << std::endl;
  */

  // Store for ease of debug
  std::vector<double> max_round_vec;
  max_round_vec.push_back(max_round);
  std::vector<std::vector<double> > max_round_vec_vec;
  max_round_vec_vec.push_back(max_round_vec);
  bool result_max_round = write_h5(out_file_name, in_group_name, "max_round", \
                              max_round_vec_vec.size(), max_round_vec_vec[0].size(), max_round_vec_vec);
  std::cout << "max_round stored " << (result_max_round ? "successfully" : "unsuccessfully") << "!" << std::endl;

  // Store the cost history
  std::cout << ">>>> Storing the cost history..." << std::endl;
  bool result_flag = write_h5(out_file_name, in_group_name, "col_cost_history", \
                              col_cost_history.size(), col_cost_history[0].size(), col_cost_history);
  std::cout << "col_cost_history stored " << (result_flag ? "successfully" : "unsuccessfully") << "!" << std::endl;
  
  result_flag = write_h5(out_file_name, in_group_name, "pos_limit_cost_history", \
                         pos_limit_cost_history.size(), pos_limit_cost_history[0].size(), pos_limit_cost_history);
  std::cout << "pos_limit_cost_history stored " << (result_flag ? "successfully" : "unsuccessfully") << "!" << std::endl;

  result_flag = write_h5(out_file_name, in_group_name, "wrist_pos_cost_history", \
                         wrist_pos_cost_history.size(), wrist_pos_cost_history[0].size(), wrist_pos_cost_history);
  std::cout << "wrist_pos_cost_history stored " << (result_flag ? "successfully" : "unsuccessfully") << "!" << std::endl;

  result_flag = write_h5(out_file_name, in_group_name, "l_wrist_pos_cost_history", \
                         l_wrist_pos_cost_history.size(), l_wrist_pos_cost_history[0].size(), l_wrist_pos_cost_history);
  std::cout << "l_wrist_pos_cost_history stored " << (result_flag ? "successfully" : "unsuccessfully") << "!" << std::endl;

  result_flag = write_h5(out_file_name, in_group_name, "r_wrist_pos_cost_history", \
                         r_wrist_pos_cost_history.size(), r_wrist_pos_cost_history[0].size(), r_wrist_pos_cost_history);
  std::cout << "r_wrist_pos_cost_history stored " << (result_flag ? "successfully" : "unsuccessfully") << "!" << std::endl;

  result_flag = write_h5(out_file_name, in_group_name, "wrist_ori_cost_history", \
                         wrist_ori_cost_history.size(), wrist_ori_cost_history[0].size(), wrist_ori_cost_history);
  std::cout << "wrist_ori_cost_history stored " << (result_flag ? "successfully" : "unsuccessfully") << "!" << std::endl;

  result_flag = write_h5(out_file_name, in_group_name, "elbow_pos_cost_history", \
                         elbow_pos_cost_history.size(), elbow_pos_cost_history[0].size(), elbow_pos_cost_history);
  std::cout << "elbow_pos_cost_history stored " << (result_flag ? "successfully" : "unsuccessfully") << "!" << std::endl;

  result_flag = write_h5(out_file_name, in_group_name, "l_elbow_pos_cost_history", \
                         l_elbow_pos_cost_history.size(), l_elbow_pos_cost_history[0].size(), l_elbow_pos_cost_history);
  std::cout << "l_elbow_pos_cost_history stored " << (result_flag ? "successfully" : "unsuccessfully") << "!" << std::endl;

  result_flag = write_h5(out_file_name, in_group_name, "r_elbow_pos_cost_history", \
                         r_elbow_pos_cost_history.size(), r_elbow_pos_cost_history[0].size(), r_elbow_pos_cost_history);
  std::cout << "r_elbow_pos_cost_history stored " << (result_flag ? "successfully" : "unsuccessfully") << "!" << std::endl;

  result_flag = write_h5(out_file_name, in_group_name, "finger_cost_history", \
                         finger_cost_history.size(), finger_cost_history[0].size(), finger_cost_history);
  std::cout << "finger_cost_history stored " << (result_flag ? "successfully" : "unsuccessfully") << "!" << std::endl;

  // result_flag = write_h5(out_file_name, in_group_name, "similarity_cost_history", \
  //                        similarity_cost_history.size(), similarity_cost_history[0].size(), similarity_cost_history);
  // std::cout << "similarity_cost_history stored " << (result_flag ? "successfully" : "unsuccessfully") << "!" << std::endl;

  result_flag = write_h5(out_file_name, in_group_name, "smoothness_cost_history", \
                         smoothness_cost_history.size(), smoothness_cost_history[0].size(), smoothness_cost_history);
  std::cout << "smoothness_cost_history stored " << (result_flag ? "successfully" : "unsuccessfully") << "!" << std::endl;

  result_flag = write_h5(out_file_name, in_group_name, "dmp_orien_cost_history", \
                         dmp_orien_cost_history.size(), dmp_orien_cost_history[0].size(), dmp_orien_cost_history);
  std::cout << "dmp_orien_cost_history stored " << (result_flag ? "successfully" : "unsuccessfully") << "!" << std::endl;

  result_flag = write_h5(out_file_name, in_group_name, "dmp_scale_cost_history", \
                         dmp_scale_cost_history.size(), dmp_scale_cost_history[0].size(), dmp_scale_cost_history);
  std::cout << "dmp_scale_cost_history stored " << (result_flag ? "successfully" : "unsuccessfully") << "!" << std::endl;

  result_flag = write_h5(out_file_name, in_group_name, "dmp_rel_change_cost_history", \
                         dmp_rel_change_cost_history.size(), dmp_rel_change_cost_history[0].size(), dmp_rel_change_cost_history);
  std::cout << "dmp_rel_change_cost_history stored " << (result_flag ? "successfully" : "unsuccessfully") << "!" << std::endl;


  // Store the jacobian history for DMP starts and goals
  // 1 - use residuals and pass points
  //result_flag = write_h5_3d(out_file_name, in_group_name, "jacobian_history", \
                            jacobian_history.size(), jacobian_history[0].size(), jacobian_history[0][0].size(), jacobian_history);
  //std::cout << "jacobian_history stored " << (result_flag ? "successfully" : "unsuccessfully") << "!" << std::endl;
  // 2 - use DMP
  // result_flag = write_h5(out_file_name, in_group_name, "sim_jacobian_history", \
  //                           sim_jacobian_history.size(), sim_jacobian_history[0].size(), sim_jacobian_history);
  // std::cout << "sim_jacobian_history stored " << (result_flag ? "successfully" : "unsuccessfully") << "!" << std::endl;
  
  result_flag = write_h5(out_file_name, in_group_name, "track_jacobian_history", \
                            track_jacobian_history.size(), track_jacobian_history[0].size(), track_jacobian_history);
  std::cout << "track_jacobian_history stored " << (result_flag ? "successfully" : "unsuccessfully") << "!" << std::endl;

  result_flag = write_h5(out_file_name, in_group_name, "orien_jacobian_history", \
                            orien_jacobian_history.size(), orien_jacobian_history[0].size(), orien_jacobian_history);
  std::cout << "orien_jacobian_history stored " << (result_flag ? "successfully" : "unsuccessfully") << "!" << std::endl;

  result_flag = write_h5(out_file_name, in_group_name, "scale_jacobian_history", \
                            scale_jacobian_history.size(), scale_jacobian_history[0].size(), scale_jacobian_history);
  std::cout << "scale_jacobian_history stored " << (result_flag ? "successfully" : "unsuccessfully") << "!" << std::endl;

  result_flag = write_h5(out_file_name, in_group_name, "rel_change_jacobian_history", \
                            rel_change_jacobian_history.size(), rel_change_jacobian_history[0].size(), rel_change_jacobian_history);
  std::cout << "rel_change_jacobian_history stored " << (result_flag ? "successfully" : "unsuccessfully") << "!" << std::endl;


  result_flag = write_h5(out_file_name, in_group_name, "dmp_update_history", \
                            dmp_update_history.size(), dmp_update_history[0].size(), dmp_update_history);
  std::cout << "dmp_update_history stored " << (result_flag ? "successfully" : "unsuccessfully") << "!" << std::endl;


  // Statistics:
  std::cout << ">>>> Statistics: " << std::endl;
  std::cout << "Collision checking " << count_col 
            << " times, with total time = " << total_col 
            << " s, average time = " << total_col / count_col << " s." << std::endl;
  // std::cout << "Similarity computation " << count_sim 
  //           << " times, with total time = " << total_sim 
  //           << " s, average time = " << total_sim / count_sim << " s." << std::endl;
  std::cout << "Trajectory generation " << count_traj 
            << " times, with total time = " << total_traj 
            << " s, average time = " << total_traj / count_traj << " s." << std::endl;
  std::cout << "Unary constraint " << count_unary 
            << " times, with total time = " << total_unary
            << " s, average time = " << total_unary / count_unary << " s." << std::endl;
  std::cout << "Smoothness constraint " << count_smoothness
            << " times, with total time = " << total_smoothness 
            << " s, average time = " << total_smoothness / count_smoothness << " s." << std::endl;
  std::cout << "Trackig constraint " << count_tracking
            << " times, with total time = " << total_tracking 
            << " s, average time = " << total_tracking / count_tracking << " s." << std::endl;                                                

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
  std::cout << std::endl << "Joint values for the path point 1/" << NUM_DATAPOINTS << ": " << q_result.transpose() << std::endl; // check if the setId() method can be used to get the value

  q_result = v_list[1]->estimate();
  std::cout << std::endl << "Joint values for the path point 2/" << NUM_DATAPOINTS << ": " << q_result.transpose() << std::endl; // check if the setId() method can be used to get the value

  q_result = v_list[2]->estimate();
  std::cout << std::endl << "Joint values for the path point 3/" << NUM_DATAPOINTS << ": " << q_result.transpose() << std::endl; // check if the setId() method can be used to get the value

  
  bool PSDFlag = optimizer.verifyInformationMatrices(true);

  if(PSDFlag)
    std::cout << "All information matrices are Positive Semi-Definite." << std::endl;


  // Convert and store q results (which can be used later for continuing the optimization!!!)
  std::vector<std::vector<double> > q_results;
  Matrix<double, JOINT_DOF, 1> q_tmp;
  std::vector<double> q_vec(JOINT_DOF);
  for (unsigned int n = 0; n < NUM_DATAPOINTS; n++)
  {
    DualArmDualHandVertex* vertex_tmp = dynamic_cast<DualArmDualHandVertex*>(optimizer.vertex(1+n));
    q_tmp = vertex_tmp->estimate();
    for (unsigned int d = 0; d < JOINT_DOF; d++)
      q_vec[d] = q_tmp[d];
    q_results.push_back(q_vec);
    // display for debug
    /*std::cout << "original, q_tmp: " << q_tmp.transpose() << std::endl;
    std::cout << "pushed_back q_result: ";
    for (unsigned int d = 0; d < JOINT_DOF; d++)
      std::cout << q_results[n][d] << " ";
    std::cout << std::endl;*/
  }
  std::cout << "q_results size is: " << q_results.size() << " x " << q_results[0].size() << std::endl;
  
  bool result1 = write_h5(out_file_name, in_group_name, "arm_traj_1", NUM_DATAPOINTS, JOINT_DOF, q_results);
  std::cout << "q_results stored " << (result1 ? "successfully" : "unsuccessfully") << "!" << std::endl;


  return 0;

}



