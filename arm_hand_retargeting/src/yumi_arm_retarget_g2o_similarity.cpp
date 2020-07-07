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
    double max_theta = M_PI / 36.0;  // 5 deg max

    // Store jacobians
    Matrix<double, 1, DMPPOINTS_DOF> orien_jacobians_for_dmp;  // the jacobians of vector orientation cost w.r.t DMP starts and goals
    Matrix<double, 1, DMPPOINTS_DOF> scale_jacobians_for_dmp;  // the jacobians of scale cost w.r.t DMP starts and goals

    // function to compute costs
    double compute_orien_cost(Matrix<double, DMPPOINTS_DOF, 1> x);
    double compute_scale_cost(Matrix<double, DMPPOINTS_DOF, 1> x);
    double output_orien_cost();
    double output_scale_cost();
    void computeError()
    {
      // Get new DMP starts and goals
      const DMPStartsGoalsVertex *v = dynamic_cast<const DMPStartsGoalsVertex*>(_vertices[0]); // the last vertex connected
      Matrix<double, DMPPOINTS_DOF, 1> x = v->estimate();
   
      // set error
      _error(0, 0) = K_DMPSTARTSGOALS * this->compute_orien_cost(x) + K_DMPSCALEMARGIN * this->compute_scale_cost(x);

    }

    // Compute jacobians
    virtual void linearizeOplus();
    Matrix<double, 1, DMPPOINTS_DOF> output_orien_jacobian(); 
    Matrix<double, 1, DMPPOINTS_DOF> output_scale_jacobian(); 
    

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

Matrix<double, 1, DMPPOINTS_DOF> DMPConstraints::output_orien_jacobian()
{
  return this->orien_jacobians_for_dmp;
} 

Matrix<double, 1, DMPPOINTS_DOF> DMPConstraints::output_scale_jacobian()
{
  return this->scale_jacobians_for_dmp;
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
  double orien_cost = std::pow(std::max(lrw_theta - max_theta, 0.0), 2) +
                      std::pow(std::max(lew_theta - max_theta, 0.0), 2) +
                      std::pow(std::max(rew_theta - max_theta, 0.0), 2) +
                      std::pow(std::max(rw_theta - max_theta, 0.0), 2) ; // use pow() so that jacobian increases as the cost rises up, instead of staying the same...                     
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
  double scale_cost = std::pow(std::max(std::fabs(lrw_ratio - 1.0) - ratio_bound, 0.0), 2) +
                      std::pow(std::max(std::fabs(lew_ratio - 1.0) - ratio_bound, 0.0), 2) + 
                      std::pow(std::max(std::fabs(rew_ratio - 1.0) - ratio_bound, 0.0), 2) +
                      std::pow(std::max(std::fabs(rw_ratio - 1.0) - ratio_bound, 0.0), 2) +
                      std::pow(std::max(margin-max_margin, 0.0), 2); // scale to 1, and scale margin cost

  return scale_cost;

}

void DMPConstraints::linearizeOplus()
{
  // Prep
  double dmp_eps = 0.02;

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

    // evaluate jacobians for scale cost
    e_plus = this->compute_scale_cost(x+delta_x);
    e_minus = this->compute_scale_cost(x-delta_x);
    _jacobianOplusXi(0, d) += K_DMPSCALEMARGIN * (e_plus - e_minus) / (2*dmp_eps); // add up the influence
    this->scale_jacobians_for_dmp(0, d) = K_DMPSCALEMARGIN * (e_plus - e_minus) / (2*dmp_eps);

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
    double compute_arm_cost(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, unsigned int num_elbow_seg, unsigned int num_shoulder_seg, bool left_or_right, my_constraint_struct &fdata);
    double linear_map(double x_, double min_, double max_, double min_hat, double max_hat);
    double compute_finger_cost(Matrix<double, 12, 1> q_finger_robot, bool left_or_right, my_constraint_struct &fdata);
    double compute_collision_cost(Matrix<double, JOINT_DOF, 1> q_whole, boost::shared_ptr<DualArmDualHandCollision> &dual_arm_dual_hand_collision_ptr);
    double compute_pos_limit_cost(Matrix<double, JOINT_DOF, 1> q_whole, my_constraint_struct &fdata);

    void computeError();

    // functions for recording costs
    double return_pos_limit_cost();
    double return_col_cost();

    // Read and write, leave blank
    virtual bool read( std::istream& in ) {return true;}
    virtual bool write( std::ostream& out ) const {return true;}

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
  std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
  double min_distance = dual_arm_dual_hand_collision_ptr->check_self_collision(x); 
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  std::chrono::duration<double> t_spent = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  total_col += t_spent.count();
  count_col++;
  // check_world_collision, check_full_collision, compute_self_distance, compute_world_distance

  // Compute cost for collision avoidance
  //double k_col = 1.0;
  double cost = (min_distance + 1) * (min_distance + 1); // 1 for colliding state, -1 for non

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



/*
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
*/


double MyUnaryConstraints::linear_map(double x_, double min_, double max_, double min_hat, double max_hat)
{
  return (x_ - min_) / (max_ - min_) * (max_hat - min_hat) + min_hat;
}


/*
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
*/

double MyUnaryConstraints::return_col_cost()
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
  
  // 3
  double collision_cost = compute_collision_cost(x, dual_arm_dual_hand_collision_ptr);
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
  /*
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
*/
  
  // 3
  double collision_cost = compute_collision_cost(x, dual_arm_dual_hand_collision_ptr);
  //std::cout << "Collision cost=";
  //std::cout << collision_cost << ", ";
  
  // 4
  double pos_limit_cost = compute_pos_limit_cost(x, _measurement);
  //std::cout << "Pos limit cost=";
  //std::cout << pos_limit_cost << "; ";

  // total cost
  //double cost = arm_cost + finger_cost + 2.0*collision_cost + 5.0*pos_limit_cost;
  double cost = K_COL * collision_cost + K_POS_LIMIT * pos_limit_cost;
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
  
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  std::chrono::duration<double> t_01 = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  total_unary += t_01.count();

}




/* Constraint for smoothness between adjacent path points */
class SmoothnessConstraint : public BaseBinaryEdge<1, double, DualArmDualHandVertex, DualArmDualHandVertex>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
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
      _error(0, 0) = K_SMOOTHNESS * (x0 - x1).norm();
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

      return (x0 - x1).norm();
    }


    // Read and write, leave blank
    virtual bool read( std::istream& in ) {return true;}
    virtual bool write( std::ostream& out ) const {return true;}

};



// class VecLimitConstraint : public BaseBinaryEdge<>{}; // add in timestamp variables

// class AccLimitConstraint : public BaseMultiEdge<>{}; // add in timestamp variables

// class CoordinationConstraint : public BaseBinaryEdge<>{}; // Coordination constraints to keep


/* Define constraint for evaluating trajectory similarity */
/* Convert to UnaryEdge for use with DMPStartsGoalsVertex */
class SimilarityConstraint : public BaseUnaryEdge<1, my_constraint_struct, DMPStartsGoalsVertex> // <D, E>, dimension and measurement datatype
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    SimilarityConstraint(boost::shared_ptr<DMPTrajectoryGenerator> &_trajectory_generator_ptr, boost::shared_ptr<SimilarityNetwork> &_similarity_network_ptr) : trajectory_generator_ptr(_trajectory_generator_ptr), similarity_network_ptr(_similarity_network_ptr) 
    {
      // resize the number of vertices this edge connects to
      //std::cout << "resizing similarity constraint..." << std::endl;
      //this->num_vertices = num_vertices;
      //resize(num_vertices);
    }
  
    ~SimilarityConstraint(){};    

    // trajectory generator and similarity network
    boost::shared_ptr<DMPTrajectoryGenerator> &trajectory_generator_ptr;
    boost::shared_ptr<SimilarityNetwork> &similarity_network_ptr;
    

    // function to compute costs
    void computeError()
    {

      num_sim++;

      //std::cout << "Similarity edge is called !" << std::endl;

      // statistics
      count_sim++;  

      //std::cout << "Computing Similarity Constraint..." << std::endl;

      // 1 - Use residuals and pass points to generate new trajectories
      /*
      // Get pass_points as stack
      int num_passpoints = _vertices.size(); // VertexContainer, i.e. std::vector<Vertex*>
      MatrixXd pass_points(num_passpoints, PASSPOINT_DOF); // num_passpoints is not known at compile time
      for (int n = 0; n < num_passpoints; n++)
      { 
        const PassPointVertex *v = static_cast<const PassPointVertex*>(_vertices[n]);
        pass_points.block(n, 0, 1, PASSPOINT_DOF) = v->estimate().transpose(); // PassPointVertex size is PASSPOINT_DOF x 1        
      }
      //std::cout << "debug: pass_points = \n" << pass_points << std::endl;
    
      // Generate new trajectory
      std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
      MatrixXd y_seq = trajectory_generator_ptr->generate_trajectory_from_passpoints(pass_points);
      std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
      std::chrono::duration<double> t_spent = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
      total_traj += t_spent.count();
      //std::cout << "y_seq size is: " << y_seq.rows() << " x " << y_seq.cols() << std::endl;
      count_traj++;
      // rearrange: y_seq is 100*48, reshape to 4800 vectorxd, and feed into similarity network
      MatrixXd y_seq_tmp = y_seq.transpose();
      VectorXd new_traj = Map<VectorXd>(y_seq_tmp.data(), PASSPOINT_DOF*NUM_DATAPOINTS, 1);
      //std::cout << "debug: original y_seq = " << y_seq << std::endl;
      //std::cout << "debug: reshaped y_seq = " << y_seq_tmp.transpose() << std::endl;

      */


      // 2 - Use DMP to generate new trajectories
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

      //std::cout << "debug: original lrw_goal = " << lrw_new_goal.transpose() << std::endl;

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

      //std::cout << "debug: generated lrw_goal = " << result.y_lrw.block(0, NUM_DATAPOINTS-1, 3, 1).transpose() << std::endl;

      // combine with the resampled orientation and glove angle trajectories to become a whole
      // order: path point 1 (48 DOF) - path point 2(48 DOF) - ... ... - path point 50 (48 DOF); btw, a column vectory
      VectorXd new_traj(PASSPOINT_DOF*NUM_DATAPOINTS);
      for (unsigned int n = 0; n < NUM_DATAPOINTS; n++)
      {
        // left arm:
        new_traj.block(n*PASSPOINT_DOF, 0, 3, 1) = result.y_lw.block(0, n, 3, 1); // 3 x N
        new_traj.block(n*PASSPOINT_DOF+3, 0, 4, 1) = trajectory_generator_ptr->l_wrist_quat_traj.block(n, 0, 1, 4).transpose(); // N x 4
        new_traj.block(n*PASSPOINT_DOF+7, 0, 3, 1) = result.y_le.block(0, n, 3, 1);
        // right arm:
        new_traj.block(n*PASSPOINT_DOF+10, 0, 3, 1) = result.y_rw.block(0, n, 3, 1);
        new_traj.block(n*PASSPOINT_DOF+13, 0, 4, 1) = trajectory_generator_ptr->r_wrist_quat_traj.block(n, 0, 1, 4).transpose();
        new_traj.block(n*PASSPOINT_DOF+17, 0, 3, 1) = result.y_re.block(0, n, 3, 1);
        // hands:
        new_traj.block(n*PASSPOINT_DOF+20, 0, 14, 1) = trajectory_generator_ptr->l_glove_angle_traj.block(n, 0, 1, 14).transpose() * M_PI / 180.0; // N x 14
        new_traj.block(n*PASSPOINT_DOF+34, 0, 14, 1) = trajectory_generator_ptr->r_glove_angle_traj.block(n, 0, 1, 14).transpose() * M_PI / 180.0;
      }


      // compute similariity distance(it would do the normalization)
      double dist = 0;
      try{
        std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
        dist = similarity_network_ptr->compute_distance(new_traj);
        //std::cout << "debug1: dist = " << dist << std::endl;
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        std::chrono::duration<double> t_spent = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
        total_sim += t_spent.count();
      }
      catch (Exception ex)
      {
        std::cout << "Error in similarity network" << std::endl;
        exit(-1);
      }
      //std::cout << "debug2: dist = " << dist << std::endl;

      _error(0, 0) = K_SIMILARITY * dist;
    
    }

    // function for recording cost history
    double return_similarity_cost()
    {
      // 1 - Use residuals and pass points to generate new trajectories
      /*
      // Get pass_points as stack
      int num_passpoints = _vertices.size(); // VertexContainer, i.e. std::vector<Vertex*>
      MatrixXd pass_points(num_passpoints, PASSPOINT_DOF); // num_passpoints is not known at compile time
      for (int n = 0; n < num_passpoints; n++)
      { 
        const PassPointVertex *v = static_cast<const PassPointVertex*>(_vertices[n]);
        pass_points.block(n, 0, 1, PASSPOINT_DOF) = v->estimate().transpose(); // PassPointVertex size is PASSPOINT_DOF x 1        
      }
    
      // Generate new trajectory
      MatrixXd y_seq = trajectory_generator_ptr->generate_trajectory_from_passpoints(pass_points);


      // rearrange: y_seq is 100*48, reshape to 4800 vectorxd, and feed into similarity network
      MatrixXd y_seq_tmp = y_seq.transpose();
      VectorXd new_traj = Map<VectorXd>(y_seq_tmp.data(), PASSPOINT_DOF*NUM_DATAPOINTS, 1);
      */

      // 2 - Use DMP to generate new trajectories
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

      // combine with the resampled orientation and glove angle trajectories to become a whole
      // order: path point 1 (48 DOF) - path point 2(48 DOF) - ... ... - path point 50 (48 DOF); btw, a column vectory
      VectorXd new_traj(PASSPOINT_DOF*NUM_DATAPOINTS);
      for (unsigned int n = 0; n < NUM_DATAPOINTS; n++)
      {
        // left arm:
        new_traj.block(n*PASSPOINT_DOF, 0, 3, 1) = result.y_lw.block(0, n, 3, 1); // 3 x N
        new_traj.block(n*PASSPOINT_DOF+3, 0, 4, 1) = trajectory_generator_ptr->l_wrist_quat_traj.block(n, 0, 1, 4).transpose(); // N x 4
        new_traj.block(n*PASSPOINT_DOF+7, 0, 3, 1) = result.y_le.block(0, n, 3, 1);
        // right arm:
        new_traj.block(n*PASSPOINT_DOF+10, 0, 3, 1) = result.y_rw.block(0, n, 3, 1);
        new_traj.block(n*PASSPOINT_DOF+13, 0, 4, 1) = trajectory_generator_ptr->r_wrist_quat_traj.block(n, 0, 1, 4).transpose();
        new_traj.block(n*PASSPOINT_DOF+17, 0, 3, 1) = result.y_re.block(0, n, 3, 1);
        // hands:
        new_traj.block(n*PASSPOINT_DOF+20, 0, 14, 1) = trajectory_generator_ptr->l_glove_angle_traj.block(n, 0, 1, 14).transpose() * M_PI / 180.0; // N x 14
        new_traj.block(n*PASSPOINT_DOF+34, 0, 14, 1) = trajectory_generator_ptr->r_glove_angle_traj.block(n, 0, 1, 14).transpose() * M_PI / 180.0;
      }


      // compute similariity distance(it would do the normalization)
      double dist = 0;
      try{
        dist = similarity_network_ptr->compute_distance(new_traj);
      }
      catch (Exception ex)
      {
        std::cout << "Error in similarity network" << std::endl;
        exit(-1);
      }
      //std::cout << "debug2: dist = " << dist << std::endl;

      return dist;
    
    }

    // Read and write, leave blank
    virtual bool read( std::istream& in ) {return true;}
    virtual bool write( std::ostream& out ) const {return true;}

    // Re-implement linearizeOplus() for recording Jacobians of this edge
    Matrix<double, 1, DMPPOINTS_DOF> jacobians_for_dmp;
    virtual void linearizeOplus();
    Matrix<double, 1, DMPPOINTS_DOF> output_jacobian(); 
    //void set_jacobian(); // set _jacobianOplusXi for debug: see if _jacobianOplusXi and _jacobianOplus share common memory...
    
};


Matrix<double, 1, DMPPOINTS_DOF> SimilarityConstraint::output_jacobian()
{
  // 1 - Use residuals and pass points
  /*
  MatrixXd jacobians(this->num_vertices, PASSPOINT_DOF);
  for (unsigned int n = 0; n < this->num_vertices; n++)
  {
    for (unsigned int p = 0; p < PASSPOINT_DOF; p++)
      jacobians(n, p) = _jacobianOplus[n](0, p);
  }
  */

  // 2 - Use DMP 
  /*
  MatrixXd jacobians(1, DMPPOINTS_DOF);
  for (unsigned int d = 0; d < DMPPOINTS_DOF; d++)
    jacobians(0, d) = _jacobianOplusXi(0, d); // simply do a copy..?

  //std::cout << "debug: " << _jacobianOplus.size() << " x " << _jacobianOplus[0].rows() << " x " << _jacobianOplus[0].cols() << std::endl;
  */
  
  return this->jacobians_for_dmp;
}




// change _jacobianOplusXi for debug
/*
void SimilarityConstraint::set_jacobian()
{
  for (unsigned int d = 0; d < DMPPOINTS_DOF; d++)
    _jacobianOplusXi(0, d) = 0.3; // set to 0.1 for debug
}
*/



// Re-implemented linearizeOplus() for recording Jacobians
void SimilarityConstraint::linearizeOplus()
{
  //std::cout << "similarity numeric differentiation..." << std::endl;

  DMPStartsGoalsVertex* v = static_cast<DMPStartsGoalsVertex*>(_vertices[0]);
  Matrix<double, DMPPOINTS_DOF, 1> x = v->estimate();
  Matrix<double, DMPPOINTS_DOF, 1> delta_x = Matrix<double, DMPPOINTS_DOF, 1>::Zero();
  double eps = 0.1; // 3; //0.1;//10; // even setting to 10 doesn't yields nonzero number... similarity network is not well or useless ? 
  double e_plus, e_minus;
  //std::cout << "debug: similarity error = ";
  for (unsigned int n = 0; n < DMPPOINTS_DOF; n++)
  {
    // foward
    // set delta
    delta_x[n] = eps;

    // Assign and compute errors
    v->setEstimate(x+delta_x);
    this->computeError();
    e_plus = _error(0, 0);
    v->setEstimate(x-delta_x);
    this->computeError();
    e_minus = _error(0, 0);

    // Reset the original vertex estimate 
    delta_x[n] = 0;

    // Compute and set numeric derivative
    _jacobianOplusXi(0, n) = (e_plus - e_minus) / (2*eps);
    //std::cout << e_plus - e_minus << " ";

    // Store jacobians for debug
    this->jacobians_for_dmp(0, n) = _jacobianOplusXi(0, n);

  }

  // Recover the original vertex value 
  v->setEstimate(x);
  //std::cout << std::endl;

  // display for debug
  //std::cout << "debug: similarity jacobians = " << this->jacobians_for_dmp << std::endl;


}





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
                      //unsigned int num_passpoints,
                      unsigned int num_datapoints) : trajectory_generator_ptr(_trajectory_generator_ptr), left_fk_solver(_left_fk_solver), right_fk_solver(_right_fk_solver)
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
  

    MatrixXd output_jacobian();
    Matrix<double, 1, DMPPOINTS_DOF> jacobians_for_dmp;

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
MatrixXd TrackingConstraint::output_jacobian()
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

void TrackingConstraint::linearizeOplus()
{

  double dmp_eps = 0.02; //0.02; // maybe below 0.05
  double q_arm_eps = 0.5; // may need tests
  double q_finger_eps = 1.0 * M_PI / 180; // in radius
  double e_plus, e_minus;
  double dmp_update_bound = 10; //20; //50;//1; //5; // 10;
  double scale = 0.1;

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
    v->setEstimate(x-delta_x);
    this->computeError();
    e_minus = _error(0, 0);

    // reset delta
    delta_x[n] = 0.0;

    // set and store jacobians 
    _jacobianOplus[0](0, n) = (e_plus - e_minus) / (2*dmp_eps); // for DMP starts and goals
    //std::cout << e_plus - e_minus << " ";


    // set bounds for DMP jacobians
    /*
    if(_jacobianOplus[0](0, n) > dmp_update_bound)
      _jacobianOplus[0](0, n) = dmp_update_bound;
    if(_jacobianOplus[0](0, n) < -dmp_update_bound)
      _jacobianOplus[0](0, n) = -dmp_update_bound;
    */

    // store jacobians
    this->jacobians_for_dmp[n] = _jacobianOplus[0](0, n);

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
    double left_tmp = compute_arm_cost(left_fk_solver, q_cur_l, _measurement.l_num_wrist_seg, _measurement.l_num_elbow_seg, _measurement.l_num_shoulder_seg, true, _measurement); // user data is stored in _measurement now
    for (unsigned int d = 0; d < 7; d++)
    {
      // set delta
      delta_q_arm[d] = q_arm_eps;

      // left arm
      e_plus = compute_arm_cost(left_fk_solver, q_cur_l+delta_q_arm, _measurement.l_num_wrist_seg, _measurement.l_num_elbow_seg, _measurement.l_num_shoulder_seg, true, _measurement); // user data is stored in _measurement now
      e_plus += right_tmp;
      e_minus = compute_arm_cost(left_fk_solver, q_cur_l-delta_q_arm, _measurement.l_num_wrist_seg, _measurement.l_num_elbow_seg, _measurement.l_num_shoulder_seg, true, _measurement); // user data is stored in _measurement now
      e_minus += right_tmp;
      _jacobianOplus[n+1](0, d) = (e_plus - e_minus) / (2 * q_arm_eps);

      // right arm
      e_plus = left_tmp;
      e_plus += compute_arm_cost(right_fk_solver, q_cur_r+delta_q_arm, _measurement.r_num_wrist_seg, _measurement.r_num_elbow_seg, _measurement.r_num_shoulder_seg, false, _measurement);
      e_minus = left_tmp;
      e_minus += compute_arm_cost(right_fk_solver, q_cur_r-delta_q_arm, _measurement.r_num_wrist_seg, _measurement.r_num_elbow_seg, _measurement.r_num_shoulder_seg, false, _measurement);
      _jacobianOplus[n+1](0, d+7) = (e_plus - e_minus) / (2 * q_arm_eps);      

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
      /*
      result = fk_solver.JntToCart(q_in, shoulder_cart_out, num_shoulder_seg+1);
      if (result < 0){
        ROS_INFO_STREAM("FK solver failed when processing shoulder link, something went wrong");
        exit(-1);
        }
      else{
        //ROS_INFO_STREAM("FK solver succeeded for wrist link.");
      }
      */

      // Preparations
      Vector3d elbow_pos_cur = Map<Vector3d>(elbow_cart_out.p.data, 3, 1);
      Vector3d wrist_pos_cur = Map<Vector3d>(wrist_cart_out.p.data, 3, 1);
      Matrix3d wrist_ori_cur = Map<Matrix<double, 3, 3, RowMajor> >(wrist_cart_out.M.data, 3, 3); 
      //Vector3d shoulder_pos_cur = Map<Vector3d>(shoulder_cart_out.p.data, 3, 1);

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

  // 1 - Use residual and pass points to generate new trajectries
  /*
  // Get pass_points as stack
  //unsigned int num_passpoints = _vertices.size() - 1; // first vertex is q, the others are pass_points
  MatrixXd pass_points(num_passpoints, PASSPOINT_DOF); // num_passpoints is not known at compile time
  //pass_points.resize(num_passpoints, PASSPOINT_DOF);
  for (unsigned int n = 0; n < num_passpoints; n++) 
  { 
    const PassPointVertex *v = static_cast<const PassPointVertex*>(_vertices[n]);
    pass_points.block(n, 0, 1, PASSPOINT_DOF) = v->estimate().transpose(); // PassPointVertex size is PASSPOINT_DOF x 1 !!!   
  }
  //std::cout << "debug: pass_points = \n" << pass_points << std::endl;
  //std::cout << "debug: pass_points size is: " << pass_points.rows() << " x " << pass_points.cols() << std::endl;
  //std::cout << "debug: pass_points' last row = : " << pass_points.row(num_passpoints-1) << std::endl;

  // Generate new trajectory
  std::chrono::steady_clock::time_point t00 = std::chrono::steady_clock::now();
  MatrixXd y_seq = trajectory_generator_ptr->generate_trajectory_from_passpoints(pass_points);
  std::chrono::steady_clock::time_point t11 = std::chrono::steady_clock::now();
  std::chrono::duration<double> t_spent1 = std::chrono::duration_cast<std::chrono::duration<double>>(t11 - t00);
  total_traj += t_spent1.count();
  //std::cout << "y_seq size is: " << y_seq.rows() << " x " << y_seq.cols() << std::endl;
  // rearrange: y_seq is 100*48, reshape to 4800 vectorxd, and feed into similarity network
  //MatrixXd y_seq_tmp = y_seq.transpose();
  //VectorXd new_traj = Map<VectorXd>(y_seq_tmp.data(), 4800);
  //std::cout << "debug: y_seq = \n" << y_seq << std::endl;
  //std::cout << "debug: y_seq size is: " << y_seq.rows() << " x " << y_seq.cols() << std::endl;
  // statistics
  count_traj++;  
  */

  // 2 - Use DMP to generate new trajectories
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
    
    // 1 - Use residuals and pass points to generate new trajectories, including position, orientation and glove angle data.
    /*
    // Set new goals(expected trajectory) to _measurement
    unsigned int point_id = n; //_measurement.point_id;
    Matrix<double, PASSPOINT_DOF, 1> y_seq_point = y_seq.block(point_id, 0, 1, 48).transpose(); // VectorXd is column vector

    _measurement.l_wrist_pos_goal = y_seq_point.block(0, 0, 3, 1); // Vector3d
    Quaterniond q_l(y_seq_point(3, 0), y_seq_point(4, 0), y_seq_point(5, 0), y_seq_point(6, 0));
    //std::cout << "debug: q_l = " << y_seq_point(3, 0) << " " << y_seq_point(4, 0) << " " 
    //                             << y_seq_point(5, 0) << " " << y_seq_point(6, 0) << std::endl; 
    Matrix3d l_wrist_ori_goal = q_l.toRotationMatrix();
    _measurement.l_wrist_ori_goal = l_wrist_ori_goal; // Matrix3d
    _measurement.l_elbow_pos_goal = y_seq_point.block(7, 0, 3, 1);


    _measurement.r_wrist_pos_goal = y_seq_point.block(10, 0, 3, 1); // Vector3d
    Quaterniond q_r(y_seq_point[13], y_seq_point[14], y_seq_point[15], y_seq_point[16]);
    Matrix3d r_wrist_ori_goal = q_r.toRotationMatrix();  
    _measurement.r_wrist_ori_goal = r_wrist_ori_goal; // Matrix3d
    _measurement.r_elbow_pos_goal = y_seq_point.block(17, 0, 3, 1); // Vector3d is column vector

    _measurement.l_finger_pos_goal = y_seq_point.block(20, 0, 14, 1); // y_seq's glove data is already in radius
    _measurement.r_finger_pos_goal = y_seq_point.block(34, 0, 14, 1);
    */


    // 2 - Use DMP to generate new position trajectories, orientation & glove trajs are resampled and loaded in DMPTrajectoryGenerator
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
    arm_cost += compute_arm_cost(right_fk_solver, q_cur_r, _measurement.r_num_wrist_seg, _measurement.r_num_elbow_seg, _measurement.r_num_shoulder_seg, false, _measurement);
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
  std::vector<std::vector<double>> read_original_traj = read_h5(in_file_name, in_group_name, "resampled_normalized_flattened_oritraj"); 

  //unsigned int num_passpoints = read_pass_points.size();
  //std::cout << "Number of pass points: " << num_passpoints << std::endl;


  //std::cout << "size: " << read_original_traj.size() << " x " << read_original_traj[0].size() << std::endl;
  //std::cout << "test: " << read_original_traj[0][0] << " " << read_original_traj[1][0] << " " << read_original_traj[2][0] << std::endl;
  //std::cout << "number of pass points: " << num_passpoints << std::endl;


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
  //OptimizationAlgorithmGaussNewton *solver = new OptimizationAlgorithmGaussNewton(std::move(solver_ptr));
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
  std::cout << ">>>> Preparing similarity network " << std::endl;
  std::string model_path = "/home/liangyuwei/sign_language_robot_ws/test_imi_data/trained_model_adam_euclidean_epoch2000_bs1024_group_split_dataset_50p.pt";
  //VectorXd original_traj(4800);
  Matrix<double, PASSPOINT_DOF*NUM_DATAPOINTS, 1> original_traj; // size is known at compile time 
  for (int i = 0; i < read_original_traj.size(); i++)
    original_traj[i] = read_original_traj[i][0];
  boost::shared_ptr<SimilarityNetwork> similarity_network_ptr;
  //std::cout << "debug: original_traj size is " << original_traj.rows() << " x " << original_traj.cols() << std::endl;
  similarity_network_ptr.reset( new SimilarityNetwork(model_path, original_traj) );  



  // Add vertices and edges
  std::vector<DualArmDualHandVertex*> v_list(NUM_DATAPOINTS);
  std::vector<PassPointVertex*> pv_list(NUM_DATAPOINTS);
  DMPStartsGoalsVertex* dmp_vertex;
  std::vector<MyUnaryConstraints*> unary_edges;
  std::vector<SmoothnessConstraint*> smoothness_edges;  
  //std::vector<TrackingConstraint*> tracking_edges;
  TrackingConstraint* tracking_edge;
  SimilarityConstraint* similarity_edge;
  DMPConstraints* dmp_edge;

  similarity_edge = new SimilarityConstraint(trajectory_generator_ptr, similarity_network_ptr);
  similarity_edge->setId(0);

//  tracking_edge = new TrackingConstraint(trajectory_generator_ptr, 
//                                         left_fk_solver, right_fk_solver, 
//                                         num_passpoints, NUM_DATAPOINTS);
  tracking_edge = new TrackingConstraint(trajectory_generator_ptr, 
                                         left_fk_solver, right_fk_solver, 
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
  similarity_edge->setVertex(0, optimizer.vertex(0));  
  similarity_edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
  optimizer.addEdge(similarity_edge);


  
  std::cout << ">>>> Adding joint angle vertices, unary edges and tracking_error edges " << std::endl;  
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
      v_list[it]->setEstimate(Matrix<double, JOINT_DOF, 1>::Zero()); // feed in initial guess
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

  // Test if something wrong with edges computation
  /*
  double cost_tmp = 0;
  // 1
  std::cout << "Unary edges' values: ";
  for (unsigned int t = 0; t < unary_edges.size(); t++)
  {
    unary_edges[t]->computeError();
    cost_tmp = unary_edges[t]->error()[0];
    std::cout << cost_tmp << ", ";
  }
  std::cout << std::endl;
  std::cout << "Unary edges all right!" << std::endl;
  // 2
  std::cout << "Smoothness edges' values: ";
  for (unsigned int t = 0; t < smoothness_edges.size(); t++)
  {
    smoothness_edges[t]->computeError();
    cost_tmp = smoothness_edges[t]->error()[0];
    std::cout << cost_tmp << ", ";
  }
  std::cout << std::endl;
  std::cout << "Smoothness edges all right!" << std::endl;
  // 3
  //std::cout << "Tracking edges' values: ";
  //for (unsigned int t = 96; t < tracking_edges.size(); t++)
  //{
  //  tracking_edges[t]->computeError();
  //  cost_tmp = tracking_edges[t]->error()[0];
  //  std::cout << cost_tmp << ", ";
  //}
  tracking_edge->computeError();
  cost_tmp = tracking_edge->error()[0];
  std::cout << "Tracking edge's values: " << cost_tmp << std::endl;
  std::cout << "Tracking edge all right!" << std::endl;
  // 4
  std::cout << "Similarity edge's value: ";
  similarity_edge->computeError();
  cost_tmp = similarity_edge->error()[0];
  std::cout << cost_tmp << std::endl;
  std::cout << "Similarity edge all right!" << std::endl;
  */


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
  std::vector<std::vector<double> > similarity_cost_history;
  std::vector<std::vector<double> > smoothness_cost_history;
  std::vector<std::vector<double> > dmp_orien_cost_history;
  std::vector<std::vector<double> > dmp_scale_cost_history;

  //std::vector<std::vector<std::vector<double> > > jacobian_history; // for pass points, store in 3d
  std::vector<std::vector<double> > sim_jacobian_history; // for DMP, store in 2d
  std::vector<std::vector<double> > track_jacobian_history; // for DMP, store in 2d
  std::vector<std::vector<double> > orien_jacobian_history; // for DMP orientation cost (vectors pointing from starts to goals)
  std::vector<std::vector<double> > scale_jacobian_history; // for DMP scale cost (scale margin of vectors)

  std::vector<std::vector<double> > dmp_update_history;


  // Consider do DMP optimization before simultaneous optimization of q and DMP
  /*
  K_WRIST_ORI = 5.0; //50.0;
  K_WRIST_POS = 5.0; //50.0;
  K_ELBOW_POS = 5.0;//50.0;
  K_FINGER = 5.0;//50.0;  // how about setting larger tracking errors so that b is bigger...? and 
  K_SIMILARITY = 100.0; //1000.0 // 1.0 // 10.0
  K_DMPSTARTSGOALS = 50.0; // for constraining DMP starts and goals
  K_DMPSCALEMARGIN = 100.0;
  // pre-post iteration...
  // fix joints and optimize dmp starts and goals...
  std::cout << ">>>> Fix q vertices and try to optimize dmp starts and goals..." << std::endl;
  Matrix<double, DMPPOINTS_DOF, 1> dmp_vertex_tmp_mat;
  DMPStartsGoalsVertex* dmp_vertex_tmp = dynamic_cast<DMPStartsGoalsVertex*>(optimizer.vertex(0));
  dmp_vertex_tmp_mat = dmp_vertex_tmp->estimate();
  std::cout << "before pre-post-iteration: DMP starts and goals = " << dmp_vertex_tmp_mat.transpose() << std::endl;  
  for (unsigned int n = 0; n < NUM_DATAPOINTS; n++)
  {
    DualArmDualHandVertex* vertex_tmp = dynamic_cast<DualArmDualHandVertex*>(optimizer.vertex(1+n));
    vertex_tmp->setFixed(true);
  }
  // iterate to optimize DMP starts and goals
  std::cout << "Iterate to optimize DMP starts and goals alone..." << std::endl;
  unsigned int num_trials = 50;
  unsigned int iters_per_trail = 3;
  for (unsigned int n = 0; n < num_trials; n++)
  {
    // optimize for a few iterations
    unsigned int iter = optimizer.optimize(iters_per_trail);    
    
    // display jacobians and updates for debug
    MatrixXd jacobians(1, DMPPOINTS_DOF);    
    jacobians = similarity_edge->output_jacobian();
    std::cout << "Last Jacobians of similarity edge w.r.t DMP starts_and_goals vertex = " << jacobians << std::endl;
    jacobians = tracking_edge->output_jacobian();
    std::cout << "Last Jacobians of tracking edge w.r.t DMP starts_and_goals vertex = " << jacobians << std::endl;
    Matrix<double, DMPPOINTS_DOF, 1> last_update = dmp_vertex_tmp->last_update;
    std::cout << "Last update of DMP starts_and_goals vertex = " << last_update.transpose() << std::endl;

    dmp_edge->computeError();
    double dmp_error = dmp_edge->error()[0];
    std::cout << "Constraint value of DMP starts and goals = " << dmp_error << std::endl;

    // store dmp updates
    std::vector<double> dmp_update_vec(DMPPOINTS_DOF);
    for (unsigned int j = 0; j < DMPPOINTS_DOF; j++)
      dmp_update_vec[j] = last_update(j, 0);
    dmp_update_history.push_back(dmp_update_vec);

    // Terminate the process if early stopping
    if(iter < iters_per_trail) 
    {
      std::cout << "Stopped after " << n * iters_per_trail + iter << " iterations." << std::endl;
      break;
    }
  }

  // reset
  for (unsigned int n = 0; n < NUM_DATAPOINTS; n++)
  {
    DualArmDualHandVertex* vertex_tmp = dynamic_cast<DualArmDualHandVertex*>(optimizer.vertex(1+n));
    vertex_tmp->setFixed(false);
  }
  // check the results
  dmp_vertex_tmp_mat = dmp_vertex_tmp->estimate();
  std::cout << "after pre-post-iteration: DMP starts and goals = " << dmp_vertex_tmp_mat.transpose() << std::endl;
  std::cout << "debug: num_update = " << num_update << ", num_track = " << num_track << ", num_sim = " << num_sim << std::endl;
  // store for comparison and presentation  
  std::vector<std::vector<double> > dmp_starts_goals_store2;
  std::vector<double> dmp_starts_goals_vec2(DMPPOINTS_DOF);
  for (unsigned int d = 0; d < DMPPOINTS_DOF; d++)
    dmp_starts_goals_vec2[d] = dmp_vertex_tmp_mat[d];
  dmp_starts_goals_store2.push_back(dmp_starts_goals_vec2);
  bool result4 = write_h5(out_file_name, in_group_name, "dmp_starts_goals_moved_optimed", dmp_starts_goals_store2.size(), dmp_starts_goals_store2[0].size(), dmp_starts_goals_store2);
  std::cout << "dmp results stored " << (result4 ? "successfully" : "unsuccessfully") << "!" << std::endl;
  */

  // PreIteration
  K_COL = 10.0; //3.0; //5.0; //5.0; // difference is 4... jacobian would be 4 * K_COL. so no need to set huge K_COL
  K_POS_LIMIT = 10.0; //5.0;//10.0;
  K_WRIST_ORI = 4.0; // 1.0;
  K_WRIST_POS = 4.0; // 1.0;
  K_ELBOW_POS = 4.0; // 1.0;
  K_FINGER = 4.0; //2.0; // 1.0; // finger angle is updated independently(jacobians!!), setting a large weight to it wouldn't affect others, only accelerate the speed
  //K_SIMILARITY = 1.0; //1000.0 // 1.0 // 10.0
  K_SMOOTHNESS = 10.0;//4.0; //10.0;
  std::cout << ">>>> Pre Iteration..." << std::endl;
  if(pre_iteration)
  { 
    std::cout << "Load pre-iteration results from last time..." << std::endl;
    // read from file
    std::vector<std::vector<double>> preiter_q_results; 
    preiter_q_results = read_h5(out_file_name, in_group_name, "preiter_arm_traj_1"); 
    // convert to Matrix and assign to q vertices
    for (unsigned int it = 0; it < NUM_DATAPOINTS; it++)
    {
      // convert
      std::vector<double> preiter_q_result = preiter_q_results[it]; // 38-dim
      Matrix<double, JOINT_DOF, 1> preiter_q_mat = Map<Matrix<double, JOINT_DOF, 1>>(preiter_q_result.data(), preiter_q_result.size());
      // assign
      DualArmDualHandVertex* vertex_tmp = dynamic_cast<DualArmDualHandVertex*>(optimizer.vertex(1+it));
      vertex_tmp->setEstimate(preiter_q_mat);
    }
  }
  else
  {
    std::cout << "Start a new run of pre-iteration..." << std::endl;

    std::cout << "before pre-iteration: " << std::endl;
    std::vector<double> wrist_pos_cost_tmp = tracking_edge->return_wrist_pos_cost_history();
    std::vector<double> l_wrist_pos_cost_tmp = tracking_edge->return_l_wrist_pos_cost_history();
    std::vector<double> r_wrist_pos_cost_tmp = tracking_edge->return_r_wrist_pos_cost_history();
    
    std::vector<double> wrist_ori_cost_tmp = tracking_edge->return_wrist_ori_cost_history();
    std::vector<double> elbow_pos_cost_tmp = tracking_edge->return_elbow_pos_cost_history();
    std::vector<double> l_elbow_pos_cost_tmp = tracking_edge->return_l_elbow_pos_cost_history();
    std::vector<double> r_elbow_pos_cost_tmp = tracking_edge->return_r_elbow_pos_cost_history();

    std::vector<double> finger_cost_tmp = tracking_edge->return_finger_cost_history();
    std::cout << "wrist_pos_cost = ";
    for (unsigned int s = 0; s < wrist_pos_cost_tmp.size(); s++)
      std::cout << wrist_pos_cost_tmp[s] << " ";
    std::cout << std::endl;

    std::cout << "wrist_ori_cost = ";
    for (unsigned int s = 0; s < wrist_ori_cost_tmp.size(); s++)
      std::cout << wrist_ori_cost_tmp[s] << " ";
    std::cout << std::endl;

    std::cout << "elbow_pos_cost = ";
    for (unsigned int s = 0; s < elbow_pos_cost_tmp.size(); s++)
      std::cout << elbow_pos_cost_tmp[s] << " ";
    std::cout << std::endl;

    std::cout << "finger_cost = ";
    for (unsigned int s = 0; s < finger_cost_tmp.size(); s++)
      std::cout << finger_cost_tmp[s] << " ";
    std::cout << std::endl;


    // fix DMP starts and goals (good for lowering tracking cost quickly, also for tracking orientation and glove angle ahead of position trajectory!!!)
    DMPStartsGoalsVertex* dmp_vertex_tmp = dynamic_cast<DMPStartsGoalsVertex*>(optimizer.vertex(0));
    dmp_vertex_tmp->setFixed(true);
    // optimize for a few iterations
    optimizer.optimize(50);//(60); //(80); // 40 is enough for now... not much improvement after 40 iterations
    // reset
    dmp_vertex_tmp->setFixed(false);
    // check the results
    Matrix<double, JOINT_DOF, 1> q_result_tmp = v_list[0]->estimate();
    std::cout << std::endl << "Joint values for the path point 1/" << NUM_DATAPOINTS << ": " << q_result_tmp.transpose() << std::endl; // check if the setId() method can be used to get the value

    q_result_tmp = v_list[1]->estimate();
    std::cout << std::endl << "Joint values for the path point 2/" << NUM_DATAPOINTS << ": " << q_result_tmp.transpose() << std::endl; // check if the setId() method can be used to get the value

    q_result_tmp = v_list[2]->estimate();
    std::cout << std::endl << "Joint values for the path point 3/" << NUM_DATAPOINTS << ": " << q_result_tmp.transpose() << std::endl; // check if the setId() method can be used to get the value

    // for comparison
    std::cout << "after pre-iteration: " << std::endl;
    wrist_pos_cost_tmp = tracking_edge->return_wrist_pos_cost_history();
    l_wrist_pos_cost_tmp = tracking_edge->return_l_wrist_pos_cost_history();
    r_wrist_pos_cost_tmp = tracking_edge->return_r_wrist_pos_cost_history();
    
    wrist_ori_cost_tmp = tracking_edge->return_wrist_ori_cost_history();
    elbow_pos_cost_tmp = tracking_edge->return_elbow_pos_cost_history();
    l_elbow_pos_cost_tmp = tracking_edge->return_l_elbow_pos_cost_history();
    r_elbow_pos_cost_tmp = tracking_edge->return_r_elbow_pos_cost_history();
    
    finger_cost_tmp = tracking_edge->return_finger_cost_history();

    std::cout << "wrist_pos_cost = ";
    for (unsigned int s = 0; s < wrist_pos_cost_tmp.size(); s++)
      std::cout << wrist_pos_cost_tmp[s] << " ";
    std::cout << std::endl;

    std::cout << "l_wrist_pos_cost = ";
    for (unsigned int s = 0; s < l_wrist_pos_cost_tmp.size(); s++)
      std::cout << l_wrist_pos_cost_tmp[s] << " ";
    std::cout << std::endl;

    std::cout << "r_wrist_pos_cost = ";
    for (unsigned int s = 0; s < r_wrist_pos_cost_tmp.size(); s++)
      std::cout << r_wrist_pos_cost_tmp[s] << " ";
    std::cout << std::endl;

    std::cout << "wrist_ori_cost = ";
    for (unsigned int s = 0; s < wrist_ori_cost_tmp.size(); s++)
      std::cout << wrist_ori_cost_tmp[s] << " ";
    std::cout << std::endl;

    std::cout << "elbow_pos_cost = ";
    for (unsigned int s = 0; s < elbow_pos_cost_tmp.size(); s++)
      std::cout << elbow_pos_cost_tmp[s] << " ";
    std::cout << std::endl;

    std::cout << "l_elbow_pos_cost = ";
    for (unsigned int s = 0; s < l_elbow_pos_cost_tmp.size(); s++)
      std::cout << l_elbow_pos_cost_tmp[s] << " ";
    std::cout << std::endl;

    std::cout << "r_elbow_pos_cost = ";
    for (unsigned int s = 0; s < r_elbow_pos_cost_tmp.size(); s++)
      std::cout << r_elbow_pos_cost_tmp[s] << " ";
    std::cout << std::endl;

    std::cout << "finger_cost = ";
    for (unsigned int s = 0; s < finger_cost_tmp.size(); s++)
      std::cout << finger_cost_tmp[s] << " ";
    std::cout << std::endl;  

    // store preIteration results for reuse
    std::vector<std::vector<double> > preiter_q_results;
    Matrix<double, JOINT_DOF, 1> preiter_q_tmp;
    std::vector<double> preiter_q_vec(JOINT_DOF);
    for (unsigned int n = 0; n < NUM_DATAPOINTS; n++)
    {
      DualArmDualHandVertex* vertex_tmp = dynamic_cast<DualArmDualHandVertex*>(optimizer.vertex(1+n));
      preiter_q_tmp = vertex_tmp->estimate();
      for (unsigned int d = 0; d < JOINT_DOF; d++)
        preiter_q_vec[d] = preiter_q_tmp[d];
      preiter_q_results.push_back(preiter_q_vec);
      // display for debug
      //std::cout << "original, q_tmp: " << q_tmp.transpose() << std::endl;
      //std::cout << "pushed_back q_result: ";
      //for (unsigned int d = 0; d < JOINT_DOF; d++)
      //  std::cout << q_results[n][d] << " ";
      //std::cout << std::endl;
    }
    bool preiter_result = write_h5(out_file_name, in_group_name, "preiter_arm_traj_1", NUM_DATAPOINTS, JOINT_DOF, preiter_q_results);
    std::cout << "PreIteration results stored " << (preiter_result ? "successfully" : "unsuccessfully") << "!" << std::endl;

  }

  // Perform FK on pre-iteration results (output functions are alread implemented in TrackingConstraint)
  std::vector<std::vector<double>> l_wrist_pos_traj, r_wrist_pos_traj, l_elbow_pos_traj, r_elbow_pos_traj;
  l_wrist_pos_traj = tracking_edge->return_wrist_pos_traj(true);
  r_wrist_pos_traj = tracking_edge->return_wrist_pos_traj(false);
  l_elbow_pos_traj = tracking_edge->return_elbow_pos_traj(true);
  r_elbow_pos_traj = tracking_edge->return_elbow_pos_traj(false);
  std::cout << ">>>> Storing current executed Cartesian trajectories to h5 file" << std::endl;
  bool tmp_flag = write_h5(out_file_name, in_group_name, "preiter_l_wrist_pos_traj", \
                           l_wrist_pos_traj.size(), l_wrist_pos_traj[0].size(), l_wrist_pos_traj);
  std::cout << "preiter_l_wrist_pos_traj stored " << (tmp_flag ? "successfully" : "unsuccessfully") << "!" << std::endl;
  
  tmp_flag = write_h5(out_file_name, in_group_name, "preiter_r_wrist_pos_traj", \
                           r_wrist_pos_traj.size(), r_wrist_pos_traj[0].size(), r_wrist_pos_traj);
  std::cout << "preiter_r_wrist_pos_traj stored " << (tmp_flag ? "successfully" : "unsuccessfully") << "!" << std::endl;
  
  tmp_flag = write_h5(out_file_name, in_group_name, "preiter_l_elbow_pos_traj", \
                           l_elbow_pos_traj.size(), l_elbow_pos_traj[0].size(), l_elbow_pos_traj);
  std::cout << "preiter_l_elbow_pos_traj stored " << (tmp_flag ? "successfully" : "unsuccessfully") << "!" << std::endl;
  
  tmp_flag = write_h5(out_file_name, in_group_name, "preiter_r_elbow_pos_traj", \
                           r_elbow_pos_traj.size(), r_elbow_pos_traj[0].size(), r_elbow_pos_traj);
  std::cout << "preiter_r_elbow_pos_traj stored " << (tmp_flag ? "successfully" : "unsuccessfully") << "!" << std::endl;


  // 1
  std::cout << "Collision costs: ";
  for (unsigned int t = 0; t < unary_edges.size(); t++)
  {
    std::cout << unary_edges[t]->return_col_cost() << " ";
  }
  std::cout << std::endl;
  // 2
  std::cout << "Pos Limit costs: ";
  for (unsigned int t = 0; t < unary_edges.size(); t++)
  {
    std::cout << unary_edges[t]->return_pos_limit_cost() << " ";
  }
  std::cout << std::endl;
  // 3
  tracking_edge->computeError();
  double cost_display = tracking_edge->error()[0];
  std::cout << "Tracking edge's values: " << cost_display << std::endl;  
  // 4
  similarity_edge->computeError();
  cost_display = similarity_edge->error()[0];
  std::cout << "Similarity edge's values: " << cost_display << std::endl;  
  // 5
  std::cout << "Smoothness edges' values: ";
  for (unsigned int t = 0; t < smoothness_edges.size(); t++)
  {
    std::cout << smoothness_edges[t]->return_smoothness_cost() << " ";
  }    
  std::cout << std::endl;


  // Set new initial guess for DMP
  /*
  MatrixXd wrist_pos_offsets(3, NUM_DATAPOINTS);
  wrist_pos_offsets = tracking_edge->return_wrist_pos_offsets(); // only the right wrist's 
  std::cout << ">>>> Set new initial guess for DMP starts and goals " << std::endl;
  Vector3d wrist_pos_offset = wrist_pos_offsets.rowwise().mean();
  // Verified in MATLAB
  //std::cout << "debug: wrist_pos_offset size = " << wrist_pos_offset.rows() << " x " << wrist_pos_offset.cols() << std::endl;
  //std::cout << "debug: wrist_pos_offset = " << wrist_pos_offset << std::endl;
  //std::cout << "debug: wrist_pos_offsets = " << wrist_pos_offsets << std::endl;
  // get current DMP starts and goals and add offset
  DMPStartsGoalsVertex *dmp_vertex_tmp_tmp = dynamic_cast<DMPStartsGoalsVertex*>(optimizer.vertex(0)); // the last vertex connected
  Matrix<double, DMPPOINTS_DOF, 1> xx = dmp_vertex_tmp_tmp->estimate();
  MatrixXd rw_new_goal(3, 1); rw_new_goal = xx.block(18, 0, 3, 1);
  MatrixXd rw_new_start(3, 1); rw_new_start = xx.block(21, 0, 3, 1);
  rw_new_goal = rw_new_goal + wrist_pos_offset;
  rw_new_start = rw_new_start + wrist_pos_offset;
  xx.block(18, 0, 3, 1) = rw_new_goal;
  xx.block(21, 0, 3, 1) = rw_new_start;
  dmp_vertex_tmp_tmp->setEstimate(xx);

  // store for comparison and presentation
  std::vector<std::vector<double> > dmp_starts_goals_store1;
  std::vector<double> dmp_starts_goals_vec(DMPPOINTS_DOF);
  for (unsigned int d = 0; d < DMPPOINTS_DOF; d++)
    dmp_starts_goals_vec[d] = xx[d];
  dmp_starts_goals_store1.push_back(dmp_starts_goals_vec);
  bool result5 = write_h5(out_file_name, in_group_name, "dmp_starts_goals_moved_pulled", dmp_starts_goals_store1.size(), dmp_starts_goals_store1[0].size(), dmp_starts_goals_store1);
  std::cout << "dmp results stored " << (result5 ? "successfully" : "unsuccessfully") << "!" << std::endl;

  */


  // Start optimization and store cost history
  std::cout << ">>>> Start optimization of the whole graph" << std::endl;
  /*
  K_COL = 10.0;
  K_POS_LIMIT = 3.0;
  K_WRIST_ORI = 1.0;
  K_WRIST_POS = 1.0;
  K_ELBOW_POS = 1.0;
  K_FINGER = 1.0;
  K_SIMILARITY = 100.0; //1000.0 // 1.0 // 10.0
  K_SMOOTHNESS = 5.0;
  K_DMPSTARTSGOALS = 20.0;
  K_DMPSCALEMARGIN = 30.0;
  */
  std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
  //unsigned int num_records = 50; 
  //unsigned int per_iterations = 5; // record data for every 10 iterations

  unsigned int num_rounds = 1;//10;//20;//200;
  unsigned int dmp_per_iterations = 20;//10; //5; //10;
  unsigned int q_per_iterations = 80;//50;//40;//50; //30;//20; //40;

  DMPStartsGoalsVertex* dmp_vertex_tmp_check = dynamic_cast<DMPStartsGoalsVertex*>(optimizer.vertex(0));
  std::cout << "debug: DMP starts_and_goals vertex " << (dmp_vertex_tmp_check->fixed()? "is" : "is not") << " fixed during optimization." << std::endl;


  // num_iterations = num_records * per_iterations
  for (unsigned int n = 0; n < num_rounds; n++)
  {

    // Set new initial guess for DMPs; 
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
    // get new starts and goals
    lrw_new_goal = lw_new_goal - rw_new_goal;
    lrw_new_start = lw_new_start - rw_new_start;
    lew_new_goal = le_new_goal - lw_new_goal;
    lew_new_start = le_new_start - lw_new_start;
    rew_new_goal = re_new_goal - rw_new_goal;
    rew_new_start = re_new_start - rw_new_start;
    // assign initial guess
    // xx.block(0, 0, 3, 1) = lrw_new_goal;
    // xx.block(3, 0, 3, 1) = lrw_new_start;
    // xx.block(6, 0, 3, 1) = lew_new_goal;
    // xx.block(9, 0, 3, 1) = lew_new_start;
    // xx.block(12, 0, 3, 1) = rew_new_goal;
    // xx.block(15, 0, 3, 1) = rew_new_start;
    xx.block(18, 0, 3, 1) = rw_new_goal;
    xx.block(21, 0, 3, 1) = rw_new_start;
    dmp_vertex_tmp_tmp->setEstimate(xx);


    // 1 - optimize DMP for a number of iterations
    std::cout << ">>>> Round " << (n+1) << "/" << num_rounds << ", DMP stage: "<< std::endl;
    K_WRIST_ORI = 1.0;
    K_WRIST_POS = 1.0;//5.0; // 1.0;
    K_ELBOW_POS = 1.0;
    K_FINGER = 1.0;
    K_SIMILARITY = 100.0; //1000.0 // 1.0 // 10.0
    K_DMPSTARTSGOALS = 50.0; //50.0;
    K_DMPSCALEMARGIN = 100.0; //50.0;
    std::cout << "Fix q vertices." << std::endl;
    // fix q vertices
    for (unsigned int m = 0; m < NUM_DATAPOINTS; m++)
    {
      DualArmDualHandVertex* vertex_tmp = dynamic_cast<DualArmDualHandVertex*>(optimizer.vertex(1+m));
      vertex_tmp->setFixed(true);
    }
    // iterate to optimize DMP
    std::cout << "Optimizing DMP..." << std::endl;
    unsigned int dmp_iter = optimizer.optimize(dmp_per_iterations);
    // Save cost history
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
    finger_cost_history.push_back(finger_cost);
    // 4)
    std::vector<double> similarity_cost;
    similarity_cost.push_back(similarity_edge->return_similarity_cost());
    similarity_cost_history.push_back(similarity_cost);
    // 5)
    std::vector<double> dmp_orien_cost;
    dmp_orien_cost.push_back(dmp_edge->output_orien_cost());
    dmp_orien_cost_history.push_back(dmp_orien_cost);
    std::vector<double> dmp_scale_cost;
    dmp_scale_cost.push_back(dmp_edge->output_scale_cost());
    dmp_scale_cost_history.push_back(dmp_scale_cost);

    // Save Jacobians of constraints w.r.t DMP starts and goals
    std::cout << "Recording DMP jacobians.." << std::endl;
    MatrixXd jacobians(1, DMPPOINTS_DOF);    
    // 1)
    jacobians = similarity_edge->output_jacobian();
    std::vector<double> jacobian_vec(DMPPOINTS_DOF); // from MatrixXd to std::vector<std::vector<double>>
    for (unsigned int j = 0; j < DMPPOINTS_DOF; j++)
    {
      jacobian_vec[j] = jacobians(0, j);
    }
    sim_jacobian_history.push_back(jacobian_vec);
    // 2)
    jacobians = tracking_edge->output_jacobian();
    for (unsigned int j = 0; j < DMPPOINTS_DOF; j++)
    {
      jacobian_vec[j] = jacobians(0, j);
    }
    track_jacobian_history.push_back(jacobian_vec);
    // 3)
    jacobians = dmp_edge->output_orien_jacobian();
    for (unsigned int j = 0; j < DMPPOINTS_DOF; j++)
    {
      jacobian_vec[j] = jacobians(0, j);
    }
    orien_jacobian_history.push_back(jacobian_vec);    
    // 4)
    jacobians = dmp_edge->output_scale_jacobian();
    for (unsigned int j = 0; j < DMPPOINTS_DOF; j++)
    {
      jacobian_vec[j] = jacobians(0, j);
    }
    scale_jacobian_history.push_back(jacobian_vec);    

    // store dmp updates
    std::cout << "Recording DMP updates.." << std::endl;
    std::vector<double> dmp_update_vec(DMPPOINTS_DOF);
    DMPStartsGoalsVertex* dmp_vertex_tmp = dynamic_cast<DMPStartsGoalsVertex*>(optimizer.vertex(0));    
    Matrix<double, DMPPOINTS_DOF, 1> last_update = dmp_vertex_tmp->last_update;
    for (unsigned int j = 0; j < DMPPOINTS_DOF; j++)
      dmp_update_vec[j] = last_update(j, 0);
    dmp_update_history.push_back(dmp_update_vec);

    // Reset q vertices
    std::cout << "Activate q vertices." << std::endl;
    for (unsigned int m = 0; m < NUM_DATAPOINTS; m++)
    {
      DualArmDualHandVertex* vertex_tmp = dynamic_cast<DualArmDualHandVertex*>(optimizer.vertex(1+m));
      vertex_tmp->setFixed(false);
    }


    // Reset q vertices to zeros for common initial condition
    for (unsigned int m = 0; m < NUM_DATAPOINTS; m++)
    {
      DualArmDualHandVertex* vertex_tmp = dynamic_cast<DualArmDualHandVertex*>(optimizer.vertex(1+m));
      vertex_tmp->setToOriginImpl();
    }

    // 2 - Optimize q vertices
    K_COL = 10.0;//10.0; //20.0; //10.0;
    K_POS_LIMIT = 10.0; //30.0; //20.0; //10.0;
    K_WRIST_ORI = 4.0;//3.0; //2.0; //1.0;
    K_WRIST_POS = 4.0;//3.0; //2.0; //1.0;
    K_ELBOW_POS = 4.0;//1.0;
    K_FINGER = 4.0;//3.0; //2.0; //1.0;
    K_SMOOTHNESS = 10.0; //20.0; //10.0;
    std::cout << ">>>> Round " << (n+1) << "/" << num_rounds << ", q stage: "<< std::endl;
    // Fix DMP starts and goals
    std::cout << "Fix DMP starts and goals." << std::endl;
    //DMPStartsGoalsVertex* dmp_vertex_tmp = dynamic_cast<DMPStartsGoalsVertex*>(optimizer.vertex(0));    
    dmp_vertex_tmp->setFixed(true);
    // optimize for a few iterations
    std::cout << "Optimizing q..." << std::endl;
    unsigned int q_iter = optimizer.optimize(q_per_iterations); 
    // Store cost results
    std::cout << "Recording collision cost, position limit cost, smoothness cost and tracking cost..." << std::endl;
    std::vector<double> col_cost;
    std::vector<double> pos_limit_cost;
    for (unsigned int t = 0; t < unary_edges.size(); t++)
    {
      col_cost.push_back(unary_edges[t]->return_col_cost());
      pos_limit_cost.push_back(unary_edges[t]->return_pos_limit_cost());
    }
    col_cost_history.push_back(col_cost);
    pos_limit_cost_history.push_back(pos_limit_cost);
    // 2)
    std::vector<double> smoothness_cost;
    for (unsigned int t = 0; t < smoothness_edges.size(); t++)
    {
      smoothness_cost.push_back(smoothness_edges[t]->return_smoothness_cost());
    }
    smoothness_cost_history.push_back(smoothness_cost);
    // 3)    
    wrist_pos_cost = tracking_edge->return_wrist_pos_cost_history();
    l_wrist_pos_cost = tracking_edge->return_l_wrist_pos_cost_history();
    r_wrist_pos_cost = tracking_edge->return_r_wrist_pos_cost_history();    
    wrist_ori_cost = tracking_edge->return_wrist_ori_cost_history();
    elbow_pos_cost = tracking_edge->return_elbow_pos_cost_history();
    l_elbow_pos_cost = tracking_edge->return_l_elbow_pos_cost_history();
    r_elbow_pos_cost = tracking_edge->return_r_elbow_pos_cost_history();    
    finger_cost = tracking_edge->return_finger_cost_history(); 
    wrist_pos_cost_history.push_back(wrist_pos_cost);
    l_wrist_pos_cost_history.push_back(l_wrist_pos_cost);
    r_wrist_pos_cost_history.push_back(r_wrist_pos_cost);
    wrist_ori_cost_history.push_back(wrist_ori_cost);
    elbow_pos_cost_history.push_back(elbow_pos_cost);
    l_elbow_pos_cost_history.push_back(l_elbow_pos_cost);
    r_elbow_pos_cost_history.push_back(r_elbow_pos_cost);
    finger_cost_history.push_back(finger_cost);

    // reset
    std::cout << "Activate DMP vertex." << std::endl;
    dmp_vertex_tmp->setFixed(false);


    // Terminate the process if early stopping
    if(dmp_iter < dmp_per_iterations && q_iter < q_per_iterations) 
    {
      std::cout << "Stopped after " << n * (dmp_per_iterations+q_per_iterations) + dmp_iter + q_iter << " iterations." << std::endl;
      break;
    }


  }

  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  std::chrono::duration<double> t_spent = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  std::cout << "Total time used for optimization: " << t_spent.count() << " s" << std::endl;

  //saveFlag = optimizer.save("./g2o_results/result_after.g2o");
  //std::cout << "g2o file saved " << (saveFlag? "successfully" : "unsuccessfully") << " ." << std::endl;

  std::cout << ">>>> Optimization done." << std::endl;


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

  result_flag = write_h5(out_file_name, in_group_name, "similarity_cost_history", \
                         similarity_cost_history.size(), similarity_cost_history[0].size(), similarity_cost_history);
  std::cout << "similarity_cost_history stored " << (result_flag ? "successfully" : "unsuccessfully") << "!" << std::endl;

  result_flag = write_h5(out_file_name, in_group_name, "smoothness_cost_history", \
                         smoothness_cost_history.size(), smoothness_cost_history[0].size(), smoothness_cost_history);
  std::cout << "smoothness_cost_history stored " << (result_flag ? "successfully" : "unsuccessfully") << "!" << std::endl;

  result_flag = write_h5(out_file_name, in_group_name, "dmp_orien_cost_history", \
                         dmp_orien_cost_history.size(), dmp_orien_cost_history[0].size(), dmp_orien_cost_history);
  std::cout << "dmp_orien_cost_history stored " << (result_flag ? "successfully" : "unsuccessfully") << "!" << std::endl;

  result_flag = write_h5(out_file_name, in_group_name, "dmp_scale_cost_history", \
                         dmp_scale_cost_history.size(), dmp_scale_cost_history[0].size(), dmp_scale_cost_history);
  std::cout << "dmp_scale_cost_history stored " << (result_flag ? "successfully" : "unsuccessfully") << "!" << std::endl;


  // Store the jacobian history for DMP starts and goals
  // 1 - use residuals and pass points
  //result_flag = write_h5_3d(out_file_name, in_group_name, "jacobian_history", \
                            jacobian_history.size(), jacobian_history[0].size(), jacobian_history[0][0].size(), jacobian_history);
  //std::cout << "jacobian_history stored " << (result_flag ? "successfully" : "unsuccessfully") << "!" << std::endl;
  // 2 - use DMP
  result_flag = write_h5(out_file_name, in_group_name, "sim_jacobian_history", \
                            sim_jacobian_history.size(), sim_jacobian_history[0].size(), sim_jacobian_history);
  std::cout << "sim_jacobian_history stored " << (result_flag ? "successfully" : "unsuccessfully") << "!" << std::endl;
  
  result_flag = write_h5(out_file_name, in_group_name, "track_jacobian_history", \
                            track_jacobian_history.size(), track_jacobian_history[0].size(), track_jacobian_history);
  std::cout << "track_jacobian_history stored " << (result_flag ? "successfully" : "unsuccessfully") << "!" << std::endl;

  result_flag = write_h5(out_file_name, in_group_name, "orien_jacobian_history", \
                            orien_jacobian_history.size(), orien_jacobian_history[0].size(), orien_jacobian_history);
  std::cout << "orien_jacobian_history stored " << (result_flag ? "successfully" : "unsuccessfully") << "!" << std::endl;

  result_flag = write_h5(out_file_name, in_group_name, "scale_jacobian_history", \
                            scale_jacobian_history.size(), scale_jacobian_history[0].size(), scale_jacobian_history);
  std::cout << "scale_jacobian_history stored " << (result_flag ? "successfully" : "unsuccessfully") << "!" << std::endl;


  result_flag = write_h5(out_file_name, in_group_name, "dmp_update_history", \
                            dmp_update_history.size(), dmp_update_history[0].size(), dmp_update_history);
  std::cout << "dmp_update_history stored " << (result_flag ? "successfully" : "unsuccessfully") << "!" << std::endl;


  // Statistics:
  std::cout << "Statistics: " << std::endl;
  std::cout << "Collision checking " << count_col 
            << " times, with total time = " << total_col 
            << " s, average time = " << total_col / count_col << " s." << std::endl;
  std::cout << "Similarity computation " << count_sim 
            << " times, with total time = " << total_sim 
            << " s, average time = " << total_sim / count_sim << " s." << std::endl;
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

  // Convert and store pass_points results (can be used as a breakpoint for later continuing optimization)
  /*
  std::vector<std::vector<double> > passpoint_results;
  Matrix<double, PASSPOINT_DOF, 1> passpoint_tmp;
  std::vector<double> passpoint_vec(PASSPOINT_DOF);
  for (unsigned int n = 0; n < num_passpoints; n++)
  {
    PassPointVertex* vertex_tmp = dynamic_cast<PassPointVertex*>(optimizer.vertex(n));
    passpoint_tmp = vertex_tmp->estimate();
    for (unsigned int d = 0; d < PASSPOINT_DOF; d++)
      passpoint_vec[d] = passpoint_tmp[d];
    passpoint_results.push_back(passpoint_vec);
    // display for debug
    //std::cout << "original, q_tmp: " << q_tmp.transpose() << std::endl;
    //std::cout << "pushed_back q_result: ";
    //for (unsigned int d = 0; d < JOINT_DOF; d++)
    //  std::cout << q_results[n][d] << " ";
    //std::cout << std::endl;
  }
  std::cout << "passpoint_results size is: " << passpoint_results.size() << " x " << passpoint_results[0].size() << std::endl;
  
  bool result2 = write_h5(out_file_name, in_group_name, "passpoint_traj_1", num_passpoints, PASSPOINT_DOF, passpoint_results);
  std::cout << "passpoint_results stored " << (result2 ? "successfully" : "unsuccessfully") << "!" << std::endl;

  */

  // Convert and store optimized DMP starts and goals (can be used as a breakpoint for later continuing optimization)
  std::vector<std::vector<double> > dmp_starts_goals_store;
  Matrix<double, DMPPOINTS_DOF, 1> dmp_starts_goals_tmp;
  std::vector<double> dmp_starts_goals_vec_final(DMPPOINTS_DOF);

  DMPStartsGoalsVertex* vertex_tmp = dynamic_cast<DMPStartsGoalsVertex*>(optimizer.vertex(0));
  dmp_starts_goals_tmp = vertex_tmp->estimate();
  for (unsigned int d = 0; d < DMPPOINTS_DOF; d++)
      dmp_starts_goals_vec_final[d] = dmp_starts_goals_tmp[d];
  dmp_starts_goals_store.push_back(dmp_starts_goals_vec_final);
  
  std::cout << "dmp_results size is: " << dmp_starts_goals_store.size() << " x " << dmp_starts_goals_store[0].size() << std::endl;
  std::cout << "dmp_results = " << dmp_starts_goals_tmp.transpose() << std::endl;
  bool result2 = write_h5(out_file_name, in_group_name, "dmp_starts_goals_1", dmp_starts_goals_store.size(), dmp_starts_goals_store[0].size(), dmp_starts_goals_store);
  std::cout << "dmp results stored " << (result2 ? "successfully" : "unsuccessfully") << "!" << std::endl;



  return 0;

}



