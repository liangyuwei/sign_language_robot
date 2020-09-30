// ROS
#include <ros/ros.h>

// NLopt
#include <nlopt.hpp> // C++ version!!!

// Common
#include <vector>
#include <iostream>
#include <fstream>
#include <math.h>

// For acos, fabs
#include <cmath>

// For Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry> // for Map()

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
#include "tools/h5_io.h"

// Process the terminal arguments
#include <getopt.h>

// For collision checking
//#include "distance_computation.h"
// #include "tools/collision_checking_yumi.h"

double K_WRIST_POS = 0.1;
double K_ELBOW_POS = 0.1;


std::stringstream read_file(std::string file_name)
{
  std::ifstream ifs(file_name);
  std::stringstream ss;
  ss << ifs.rdbuf();
  return ss;
}

// Get URDF and SRDF for distance computation class
std::string urdf_file_name = "/home/liangyuwei/sign_language_robot_ws/src/yumi_description/urdf/yumi_with_hands.urdf";
std::string srdf_file_name = "/home/liangyuwei/sign_language_robot_ws/src/yumi_sign_language_robot_moveit_config/config/yumi.srdf";
//std::ifstream urdf_file(urdf_file_name);
//std::ifstream srdf_file(srdf_file_name);
std::stringstream urdf_string = read_file(urdf_file_name);
std::stringstream srdf_string = read_file(srdf_file_name);


// static boost::shared_ptr<DualArmDualHandCollision> dual_arm_dual_hand_collision_ptr;
//static boost::shared_ptr<DualArmDualHandMinDistance> dual_arm_dual_hand_min_distance_ptr;


#define JOINT_DOF 38
#define ARM_DOF 7

unsigned int num_iter = 0;
unsigned int max_iter = 500;

using namespace Eigen;
using namespace H5;

typedef struct {
  // Cost func related
  Matrix<double, 38, 1> q_prev; // used across optimizations over the whole trajectory

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


  double arm_cost = 0;
  double finger_cost = 0;
  double col_cost = 0;

  double smoothness_cost = 0;
  double total_cost = 0;

  double l_upperarm_length = 0;
  double r_upperarm_length = 0;
  double l_forearm_length = 0;
  double r_forearm_length = 0;

  double l_r_pos_diff_cost = 0;


  // Record information for calculating the cost of dual arm coordination
  Vector3d l_wrist_pos_cur;
  Vector3d r_wrist_pos_cur;
  double l_r_wrist_diff_cost;

  Matrix3d l_wrist_ori_cur;
  Matrix3d r_wrist_ori_cur;

  // For comparison with the human motion path, to check if the scaled version is still smooth
  Vector3d scaled_l_wrist_pos;
  Vector3d scaled_r_wrist_pos;
  Vector3d scaled_l_elbow_pos;
  Vector3d scaled_r_elbow_pos;

  Vector3d actual_l_wrist_pos;
  Vector3d actual_r_wrist_pos;
  Vector3d actual_l_elbow_pos;
  Vector3d actual_r_elbow_pos;


  // A class for distance computation (collision avoidance)
  int argc;
  char **argv;

} my_constraint_struct;


/**
 * This class is used for passing user-defined data into the optimization, e.g. goal position trajectories.
 */
class OptimUserData
{
  public:
    OptimUserData(){}
    ~OptimUserData(){}

    unsigned int num_datapoints = 0;

    // Human motion data 
    std::vector<Vector3d> l_shoulder_pos_goal; // this is human shoulder position
    std::vector<Vector3d> l_elbow_pos_goal;
    std::vector<Vector3d> l_wrist_pos_goal;
    std::vector<Matrix3d> l_wrist_ori_goal;

    std::vector<Vector3d> r_shoulder_pos_goal; // this is human shoulder position
    std::vector<Vector3d> r_elbow_pos_goal;
    std::vector<Vector3d> r_wrist_pos_goal;
    std::vector<Matrix3d> r_wrist_ori_goal;

    // Pre-required data of robot (for modifying human motion data)
    Vector3d l_robot_shoulder_pos; // this is normally fixed for a non-mobile robot
    Vector3d r_robot_shoulder_pos;

    // Joint Pos/Vel/Acc limits
    Matrix<double, ARM_DOF, 1> pos_ub_l;
    Matrix<double, ARM_DOF, 1> pos_lb_l;
    Matrix<double, ARM_DOF, 1> vel_ub_l;
    Matrix<double, ARM_DOF, 1> vel_lb_l;
    Matrix<double, ARM_DOF, 1> acc_ub_l;
    Matrix<double, ARM_DOF, 1> acc_lb_l;
    Matrix<double, ARM_DOF, 1> pos_ub_r;
    Matrix<double, ARM_DOF, 1> pos_lb_r;
    Matrix<double, ARM_DOF, 1> vel_ub_r;
    Matrix<double, ARM_DOF, 1> vel_lb_r;
    Matrix<double, ARM_DOF, 1> acc_ub_r;
    Matrix<double, ARM_DOF, 1> acc_lb_r;

    // Human IK generated joint trajectories (on which we are going to apply affine deformations)
    std::vector<Matrix<double, ARM_DOF, 1>> l_joint_angles_ik;
    std::vector<Matrix<double, ARM_DOF, 1>> r_joint_angles_ik;

    // IDs indicating link sequence in KDL chain, for use in KDL FK
    unsigned int l_num_wrist_seg = 0;
    unsigned int l_num_elbow_seg = 0;
    unsigned int l_num_shoulder_seg = 0;
    unsigned int r_num_wrist_seg = 0;
    unsigned int r_num_elbow_seg = 0;
    unsigned int r_num_shoulder_seg = 0;

    // Flags for specifying index of constraints
    unsigned int pos_id = 0;
    unsigned int vel_id_0 = 0, vel_id_1 = 0;
    unsigned int acc_id_0 = 0, acc_id_1 = 0, acc_id_2 = 0;

};



class MyNLopt
{

  public:
    MyNLopt();

    // constructor to use
    MyNLopt(int argc, char **argv, std::string in_file_name, std::string joint_ik_result_filename, std::string in_group_name, std::string out_file_name);

    ~MyNLopt(){};

    bool write_h5(const std::string file_name, const std::string group_name, const std::string dataset_name, const int ROW, const int COL, std::vector<std::vector<double>> data_vector);
    std::vector<std::vector<double>> read_h5(const std::string file_name, const std::string group_name, const std::string dataset_name);

    // static double linear_map(double x_, double min_, double max_, double min_hat, double max_hat);
    static Matrix<double, 12, 1> map_finger_joint_values(Matrix<double, 14, 1> q_finger_human, bool left_or_right, my_constraint_struct fdata);

    // Evaluate left and right wrist, elbow costs under the given affine deformation matrix
    static Matrix<double, 4, 1> evaluate_elbow_wrist_cost(KDL::ChainFkSolverPos_recursive left_fk_solver, KDL::ChainFkSolverPos_recursive right_fk_solver, 
                                                            Matrix<double, ARM_DOF, ARM_DOF> M_l, Matrix<double, ARM_DOF, ARM_DOF> M_r, 
                                                            unsigned int l_num_wrist_seg, unsigned int r_num_wrist_seg, 
                                                            unsigned int l_num_elbow_seg, unsigned int r_num_elbow_seg, 
                                                            OptimUserData *fdata);

    // Store the actually tracked Cartesian trajectories
    void store_actual_trajs(Matrix<double, ARM_DOF, ARM_DOF> M_l, Matrix<double, ARM_DOF, ARM_DOF> M_r, std::string file_name, std::string group_name, OptimUserData* fdata, std::string suffix);


  private:
    // Initialization list
    //DualArmDualHandCollision dual_arm_dual_hand_collision;
    
    // Cost-Func 
    static double linear_map(double x_, double min_, double max_, double min_hat, double max_hat);
    static double compute_finger_cost(Matrix<double, 12, 1> q_finger_robot, bool left_or_right, my_constraint_struct *fdata);

    static double myfunc(const std::vector<double> &x, std::vector<double> &grad, void *f_data);

    // Function for setting up FK solver
    static KDL::ChainFkSolverPos_recursive setup_left_kdl(OptimUserData &constraint_data);
    static KDL::ChainFkSolverPos_recursive setup_right_kdl(OptimUserData &constraint_data);

    // Constraints
    static void my_inequality_constraint_pos(unsigned m, double *result, unsigned n, const double *x,
                                         double *grad, /* NULL if not needed */
                                         void *f_data);
    static void my_inequality_constraint_vel(unsigned m, double *result, unsigned n, const double *x,
                                         double *grad, /* NULL if not needed */
                                         void *f_data);
    static void my_inequality_constraint_acc(unsigned m, double *result, unsigned n, const double *x,
                                         double *grad, /* NULL if not needed */
                                         void *f_data);

    static void my_equality_constraint_pos(unsigned m, double *result, unsigned n, const double *x,
                                          double *grad, /* NULL if not needed */
                                          void *f_data);
    static void my_equality_constraint_vel(unsigned m, double *result, unsigned n, const double *x,
                                          double *grad, /* NULL if not needed */
                                          void *f_data);
    
    static Matrix<double, ARM_DOF, ARM_DOF*ARM_DOF> compute_eq_con_grad_helper(Matrix<double, ARM_DOF, 1> error);

    static Matrix<double, 2*2*ARM_DOF, 1> calculate_ineq_constraints_pos_helper(Matrix<double, ARM_DOF*ARM_DOF*2, 1> x, void *f_data);
    static Matrix<double, 2*2*ARM_DOF, 1> calculate_ineq_constraints_vel_helper(Matrix<double, ARM_DOF*ARM_DOF*2, 1> x, void *f_data);
    static Matrix<double, 2*2*ARM_DOF, 1> calculate_ineq_constraints_acc_helper(Matrix<double, ARM_DOF*ARM_DOF*2, 1> x, void *f_data);

    static Matrix<double, 2*ARM_DOF, 1> calculate_eq_constraints_pos_helper(Matrix<double, ARM_DOF*ARM_DOF*2, 1> x, unsigned int pos_id, OptimUserData *fdata);
    static Matrix<double, 2*ARM_DOF, 1> calculate_eq_constraints_vel_helper(Matrix<double, ARM_DOF*ARM_DOF*2, 1> x, unsigned int vel_id, OptimUserData *fdata);


    static double compute_myfunc_helper(Matrix<double, ARM_DOF*ARM_DOF*2, 1> x, KDL::ChainFkSolverPos_recursive left_fk_solver, KDL::ChainFkSolverPos_recursive right_fk_solver, OptimUserData *fdata);

    static double compute_elbow_wrist_cost(KDL::ChainFkSolverPos_recursive fk_solver, Matrix<double, 7, 1> q_cur, 
                                    Vector3d wrist_pos_goal, Vector3d elbow_pos_goal, 
                                    unsigned int num_wrist_seg, unsigned int num_elbow_seg,
                                    bool left_or_right, OptimUserData *fdata);

    Vector3d return_elbow_pos(Matrix<double, ARM_DOF, 1> q_cur, KDL::ChainFkSolverPos_recursive fk_solver, unsigned int num_elbow_seg, bool left_or_right);
    Vector3d return_wrist_pos(Matrix<double, ARM_DOF, 1> q_cur, KDL::ChainFkSolverPos_recursive fk_solver, unsigned int num_wrist_seg, bool left_or_right);



    // NLopt static wrapper functionality for use with class
    //static double wrap_myfunc(const std::vector<double> &x, std::vector<double> &grad, void *data);// {      return (*reinterpret_cast`<MyNLopt*>`(data))(x, grad); }

};


/**
 * This function stores the actually tracked Cartesian trajectories into h5 file.
 * 
 * Input an affine deformation matrix for one arm, and use it to deform the original joint trajectoreis in fdata.
 */
void MyNLopt::store_actual_trajs(Matrix<double, ARM_DOF, ARM_DOF> M_l, Matrix<double, ARM_DOF, ARM_DOF> M_r, 
                                 std::string file_name, std::string group_name, OptimUserData* fdata, std::string suffix)
{
  // Prepare FK solver
  KDL::ChainFkSolverPos_recursive left_fk_solver = setup_left_kdl(*fdata);
  KDL::ChainFkSolverPos_recursive right_fk_solver = setup_right_kdl(*fdata);

  // Obtain the original trajectories
  std::vector<Matrix<double, ARM_DOF, 1>> l_joint_angles_ik = fdata->l_joint_angles_ik;
  std::vector<Matrix<double, ARM_DOF, 1>> r_joint_angles_ik = fdata->r_joint_angles_ik;

  // Iterate to perform FK
  std::vector<std::vector<double>> l_wrist_traj, r_wrist_traj, l_elbow_traj, r_elbow_traj;
  for (unsigned int t = 0; t < fdata->num_datapoints; t++)
  {
    // 1 - l wrist
    Vector3d l_wrist_pos = return_wrist_pos(l_joint_angles_ik[0] + M_l * (l_joint_angles_ik[t] - l_joint_angles_ik[0]), left_fk_solver, fdata->l_num_wrist_seg, true);
    std::vector<double> l_wrist_pos_vec = {l_wrist_pos[0], l_wrist_pos[1], l_wrist_pos[2]};
    l_wrist_traj.push_back(l_wrist_pos_vec);
    // 2 - r wrist
    Vector3d r_wrist_pos = return_wrist_pos(r_joint_angles_ik[0] + M_r * (r_joint_angles_ik[t] - r_joint_angles_ik[0]), right_fk_solver, fdata->r_num_wrist_seg, false);
    std::vector<double> r_wrist_pos_vec = {r_wrist_pos[0], r_wrist_pos[1], r_wrist_pos[2]};
    r_wrist_traj.push_back(r_wrist_pos_vec);

    // 3 - r elbow
    Vector3d l_elbow_pos = return_elbow_pos(l_joint_angles_ik[0] + M_l * (l_joint_angles_ik[t] - l_joint_angles_ik[0]), left_fk_solver, fdata->l_num_elbow_seg, true);
    std::vector<double> l_elbow_pos_vec = {l_elbow_pos[0], l_elbow_pos[1], l_elbow_pos[2]};
    l_elbow_traj.push_back(l_elbow_pos_vec);

    // 4 - r elbow
    Vector3d r_elbow_pos = return_elbow_pos(r_joint_angles_ik[0] + M_r * (r_joint_angles_ik[t] - r_joint_angles_ik[0]), right_fk_solver, fdata->r_num_elbow_seg, false);
    std::vector<double> r_elbow_pos_vec = {r_elbow_pos[0], r_elbow_pos[1], r_elbow_pos[2]};
    r_elbow_traj.push_back(r_elbow_pos_vec);

  }

  // Save to h5 file
  h5_io::write_h5(file_name, group_name, "actual_l_wrist_pos_traj_"+suffix, l_wrist_traj);
  h5_io::write_h5(file_name, group_name, "actual_r_wrist_pos_traj_"+suffix, r_wrist_traj);
  h5_io::write_h5(file_name, group_name, "actual_l_elbow_pos_traj_"+suffix, l_elbow_traj);
  h5_io::write_h5(file_name, group_name, "actual_r_elbow_pos_traj_"+suffix, r_elbow_traj);

}


/**
 * This is used for getting corresponding elbow position.
 */
Vector3d MyNLopt::return_elbow_pos(Matrix<double, ARM_DOF, 1> q_cur, KDL::ChainFkSolverPos_recursive fk_solver, unsigned int num_elbow_seg, bool left_or_right)
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
    std::cerr << "FK solver failed when processing elbow link, something went wrong" << std::endl;
    exit(-1);
  }

  // Get current positions
  Vector3d elbow_pos_cur = Map<Vector3d>(elbow_cart_out.p.data, 3, 1);

  return elbow_pos_cur;
}



/**
 * This is used for getting corresponding elbow position.
 */
Vector3d MyNLopt::return_wrist_pos(Matrix<double, ARM_DOF, 1> q_cur, KDL::ChainFkSolverPos_recursive fk_solver, unsigned int num_wrist_seg, bool left_or_right)
{
  // Get joint angles
  KDL::JntArray q_in(q_cur.size()); 
  for (unsigned int i = 0; i < q_cur.size(); ++i)
  {
    q_in(i) = q_cur(i);
  }

  // Do FK using KDL, get the current wrist/wrist/shoulder state
  KDL::Frame wrist_cart_out; // Output homogeneous transformation
  int result;
  result = fk_solver.JntToCart(q_in, wrist_cart_out, num_wrist_seg+1);
  if (result < 0){
    std::cerr << "FK solver failed when processing wrist link, something went wrong" << std::endl;
    exit(-1);
  }

  // Get current positions
  Vector3d wrist_pos_cur = Map<Vector3d>(wrist_cart_out.p.data, 3, 1);

  return wrist_pos_cur;
}


/*static double MyNLopt::wrap_myfunc(const std::vector<double> &x, std::vector<double> &grad, void *data) 
{
  return (*reinterpret_cast<MyNLopt*>(data))(x, grad); 
}*/

double MyNLopt::linear_map(double x_, double min_, double max_, double min_hat, double max_hat)
{
  return (x_ - min_) / (max_ - min_) * (max_hat - min_hat) + min_hat;
}


/**
 * @brief Directly map human finger data to robot hands. 
 * 
 * Result is used as an initial setup for finger part.
 */
Matrix<double, 12, 1> MyNLopt::map_finger_joint_values(Matrix<double, 14, 1> q_finger_human, bool left_or_right, my_constraint_struct fdata)
{
  // Obtain required data and parameter settings
  Matrix<double, 14, 1> human_finger_start = fdata.glove_start;
  Matrix<double, 14, 1> human_finger_final = fdata.glove_final;
  Matrix<double, 12, 1> robot_finger_start, robot_finger_final;
  if (left_or_right)
  {
    // Get bounds
    robot_finger_start = fdata.l_robot_finger_start;
    robot_finger_final = fdata.l_robot_finger_final;
  }
  else
  {
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

  return q_finger_robot_goal;
}




double MyNLopt::compute_finger_cost(Matrix<double, 12, 1> q_finger_robot, bool left_or_right, my_constraint_struct *fdata)
{
  // Obtain required data and parameter settings
  Matrix<double, 14, 1> q_finger_human;
  Matrix<double, 14, 1> human_finger_start = fdata->glove_start;
  Matrix<double, 14, 1> human_finger_final = fdata->glove_final;
  Matrix<double, 12, 1> robot_finger_start, robot_finger_final;
  if (left_or_right)
  {
    // Get the sensor data
    q_finger_human = fdata->l_finger_pos_goal;
    // Get bounds
    robot_finger_start = fdata->l_robot_finger_start;
    robot_finger_final = fdata->l_robot_finger_final;
  }
  else
  {
    // Get the sensor data
    q_finger_human = fdata->r_finger_pos_goal;    
    // Get bounds
    robot_finger_start = fdata->r_robot_finger_start;
    robot_finger_final = fdata->r_robot_finger_final;
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
    fdata->scaled_l_finger_pos_cost = finger_cost;
  }
  else
  {
    fdata->scaled_r_finger_pos_cost = finger_cost;
  }

  return finger_cost;

}

/**
 * This function is used for calculating cost of Hujin's retargeting method.
 * 
 * The cost is a scalar term, and the sum of squared error of wrist and elbow positions.
 */
double MyNLopt::compute_elbow_wrist_cost(KDL::ChainFkSolverPos_recursive fk_solver, Matrix<double, 7, 1> q_cur, 
                                          Vector3d wrist_pos_goal, Vector3d elbow_pos_goal, 
                                          unsigned int num_wrist_seg, unsigned int num_elbow_seg,
                                          bool left_or_right, OptimUserData *fdata)
{
  // Get joint angles
  KDL::JntArray q_in(q_cur.size()); 
  for (unsigned int i = 0; i < q_cur.size(); ++i)
  {
    q_in(i) = q_cur(i);
  }

  // Do FK using KDL, get the current elbow/wrist/shoulder state
  KDL::Frame elbow_cart_out, wrist_cart_out; // Output homogeneous transformation
  int result;
  result = fk_solver.JntToCart(q_in, elbow_cart_out, num_elbow_seg+1); // notice that the number here is not the segment ID, but the number of segments till target segment
  if (result < 0){
    std::cerr << "FK solver failed when processing elbow link, something went wrong" << std::endl;
    return -1;
  }

  result = fk_solver.JntToCart(q_in, wrist_cart_out, num_wrist_seg+1);
  if (result < 0){
    std::cerr << "FK solver failed when processing wrist link, something went wrong" << std::endl;
    return -1;
  }


  // Get current positions
  Vector3d elbow_pos_cur = Map<Vector3d>(elbow_cart_out.p.data, 3, 1);
  Vector3d wrist_pos_cur = Map<Vector3d>(wrist_cart_out.p.data, 3, 1);


  // Compute cost function
  // double K_WRIST_POS = 1.0; //0.1; //0.01; //0.05; //1.0; // 0.1; 
  // double K_ELBOW_POS = 0.1; //0.01; //0.05; //1.0; // 0.1; 
  // (0.1, 0.1) works well for fengren_1, baozhu_1 and kaoqin_2, but not so good for gun_2 (after setting eps for constraints gradients calculation properly);
  // (0.01, 0.01), works.. so so, for gun_2 motion
  double wrist_pos_cost = (wrist_pos_goal - wrist_pos_cur).norm();
  double elbow_pos_cost = (elbow_pos_goal - elbow_pos_cur).norm();
 
  // Return cost function value
  double cost = K_WRIST_POS * wrist_pos_cost * wrist_pos_cost + K_ELBOW_POS * elbow_pos_cost * elbow_pos_cost;
  // double cost = K_WRIST_POS * wrist_pos_cost + K_ELBOW_POS * elbow_pos_cost;

  return cost;

}



/**
 * This function is used for evaluating elbow cost of optimization results.
 * 
 * Output cost in order: l wrist pos, r wrist pos, l elbow pos, r elbow pos. \n
 * Note that we use a different cost than the one used in optimization objective function. Hujin's method 
 * utilizes squared error of wrist and elbow position, while we use norm of error vector here.
 */
Matrix<double, 4, 1> MyNLopt::evaluate_elbow_wrist_cost(KDL::ChainFkSolverPos_recursive left_fk_solver, KDL::ChainFkSolverPos_recursive right_fk_solver, 
                                                        Matrix<double, ARM_DOF, ARM_DOF> M_l, Matrix<double, ARM_DOF, ARM_DOF> M_r, 
                                                        unsigned int l_num_wrist_seg, unsigned int r_num_wrist_seg, 
                                                        unsigned int l_num_elbow_seg, unsigned int r_num_elbow_seg, 
                                                        OptimUserData *fdata)
{

  // Get the original joint trajectories
  std::vector<Matrix<double, ARM_DOF, 1>> l_joint_angles_ik = fdata->l_joint_angles_ik;
  std::vector<Matrix<double, ARM_DOF, 1>> r_joint_angles_ik = fdata->r_joint_angles_ik;

  // Iterate to calcualte costs
  Matrix<double, ARM_DOF, 1> l_joint_angles_cur, r_joint_angles_cur;
  KDL::JntArray q_in_l(ARM_DOF), q_in_r(ARM_DOF); 
  KDL::Frame cart_out; // Output homogeneous transformation
  Vector3d pos_cur;
  double l_wrist_cost = 0, r_wrist_cost = 0, l_elbow_cost = 0, r_elbow_cost = 0;
  for (unsigned int t = 0; t < fdata->num_datapoints; t++)
  {
    // Apply affine deformation
    l_joint_angles_cur = l_joint_angles_ik[0] + M_l * (l_joint_angles_ik[t] - l_joint_angles_ik[0]);
    r_joint_angles_cur = r_joint_angles_ik[0] + M_r * (r_joint_angles_ik[t] - r_joint_angles_ik[0]);
    
    // Perform FK
    for (unsigned int d = 0; d < ARM_DOF; d++)
    {
      q_in_l(d) = l_joint_angles_cur[d];
      q_in_r(d) = r_joint_angles_cur[d];
    }
    // 1 - l wrist
    int result = left_fk_solver.JntToCart(q_in_l, cart_out, l_num_wrist_seg+1);
    l_wrist_cost += (fdata->l_wrist_pos_goal[t] - Map<Vector3d>(cart_out.p.data, 3, 1)).norm(); 
    if (result < 0){
      std::cerr << "FK solver failed when processing left wrist link, something went wrong" << std::endl;
      exit(-1);
    }
    // 2 - r wrist
    result = right_fk_solver.JntToCart(q_in_r, cart_out, r_num_wrist_seg+1);
    r_wrist_cost += (fdata->r_wrist_pos_goal[t] - Map<Vector3d>(cart_out.p.data, 3, 1)).norm(); 
    if (result < 0){
      std::cerr << "FK solver failed when processing right wrist link, something went wrong" << std::endl;
      exit(-1);
    }
    // 3 - l elbow
    result = left_fk_solver.JntToCart(q_in_l, cart_out, l_num_elbow_seg+1);
    l_elbow_cost += (fdata->l_elbow_pos_goal[t] - Map<Vector3d>(cart_out.p.data, 3, 1)).norm(); 
    if (result < 0){
      std::cerr << "FK solver failed when processing left elbow link, something went wrong" << std::endl;
      exit(-1);
    }
    // 4 - r elbow
    result = right_fk_solver.JntToCart(q_in_r, cart_out, r_num_elbow_seg+1);
    r_elbow_cost += (fdata->r_elbow_pos_goal[t] - Map<Vector3d>(cart_out.p.data, 3, 1)).norm(); 
    if (result < 0){
      std::cerr << "FK solver failed when processing right elbow link, something went wrong" << std::endl;
      exit(-1);
    }

  }

  // Combine all
  Matrix<double, 4, 1> error_vec;
  error_vec << l_wrist_cost, r_wrist_cost, l_elbow_cost, r_elbow_cost;
 
  // Return cost function value
  return error_vec;
}


// This function sets elbow ID and wrist ID in constraint_data, and returns the KDL_FK solver 
KDL::ChainFkSolverPos_recursive MyNLopt::setup_left_kdl(OptimUserData &constraint_data)
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
    for (unsigned int i = 0; i < num_segments; ++i){
      if (kdl_chain.getSegment(i).getName() == ELBOW_LINK){
        constraint_data.l_num_elbow_seg = i;
        break;
      }
      if (kdl_chain.getSegment(i).getName() == SHOULDER_LINK){
        constraint_data.l_num_shoulder_seg = i;
      }
    }
  }


  // Set up FK solver and compute the homogeneous representations
  KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain);

  return fk_solver;
}


// This function sets elbow ID and wrist ID in constraint_data, and returns the KDL_FK solver 
KDL::ChainFkSolverPos_recursive MyNLopt::setup_right_kdl(OptimUserData &constraint_data)
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


  // Find segment number for wrist and elbow links, store in constraint_dat
  if (constraint_data.r_num_wrist_seg == 0 || constraint_data.r_num_elbow_seg == 0 || constraint_data.r_num_shoulder_seg == 0) // if the IDs not set
  {
    unsigned int num_segments = kdl_chain.getNrOfSegments();
    constraint_data.r_num_wrist_seg = num_segments - 1;
    for (unsigned int i = 0; i < num_segments; ++i){
      //std::cout << "Segment name: " << kdl_chain.getSegment(i).getName() << std::endl;
      if (kdl_chain.getSegment(i).getName() == ELBOW_LINK){
        constraint_data.r_num_elbow_seg = i;
        break;
      }
      if (kdl_chain.getSegment(i).getName() == SHOULDER_LINK){
        constraint_data.r_num_shoulder_seg = i;
      }
    }
  }


  // Set up FK solver and compute the homogeneous representations
  KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain);

  return fk_solver;

}


/**
 * This function computes cost value, and helps in computing gradients for myfunc().
 */
double MyNLopt::compute_myfunc_helper(Matrix<double, ARM_DOF*ARM_DOF*2, 1> x, KDL::ChainFkSolverPos_recursive left_fk_solver, KDL::ChainFkSolverPos_recursive right_fk_solver, OptimUserData *fdata)
{
  // Reconstruct affine deformation matrix from vector 
  Matrix<double, ARM_DOF, ARM_DOF> M_l, M_r;
  for (unsigned int r = 0; r < ARM_DOF; r++)
  {
    for (unsigned int c = 0; c < ARM_DOF; c++)
    {
      M_l(r, c) = x[r * ARM_DOF + c]; // first is for left arm
      M_r(r, c) = x[r * ARM_DOF + c + ARM_DOF * ARM_DOF]; // second is for right arm
    }
  }

  // Iterate to calculate costs
  double cost = 0;
  unsigned int num_datapoints = fdata->num_datapoints;
  Matrix<double, ARM_DOF, 1> l_joint_angle_begin = fdata->l_joint_angles_ik[0];
  Matrix<double, ARM_DOF, 1> r_joint_angle_begin = fdata->r_joint_angles_ik[0];
  for (unsigned int t = 0; t < num_datapoints; t++)
  {
    // left 
    cost += compute_elbow_wrist_cost(left_fk_solver, l_joint_angle_begin + M_l * (fdata->l_joint_angles_ik[t] - l_joint_angle_begin), 
                                      fdata->l_wrist_pos_goal[t], fdata->l_elbow_pos_goal[t],
                                      fdata->l_num_wrist_seg, fdata->l_num_elbow_seg,
                                      true, fdata);
    // right
    cost += compute_elbow_wrist_cost(right_fk_solver, r_joint_angle_begin + M_r * (fdata->r_joint_angles_ik[t] - r_joint_angle_begin), 
                                      fdata->r_wrist_pos_goal[t], fdata->r_elbow_pos_goal[t],
                                      fdata->r_num_wrist_seg, fdata->r_num_elbow_seg,
                                      false, fdata);
  }

  return cost;
}


/**
 * Cost function for NLopt.
 * 
 * Apply affine deformation to generate new joint trajectories, and then perform FK to obtain elbow position and wrist position. \n
 * Cost function is a scalar term, with squared error of wrist and elbow positions.
 */
double MyNLopt::myfunc(const std::vector<double> &x, std::vector<double> &grad, void *f_data)
{

  // Prep
  OptimUserData *fdata = (OptimUserData*) f_data;
  Matrix<double, ARM_DOF*ARM_DOF*2, 1> x_mat;
  for (unsigned i = 0; i < x.size(); i++)
    x_mat[i] = x[i];

  // KDL FK solver
  KDL::ChainFkSolverPos_recursive left_fk_solver = setup_left_kdl(*fdata);
  KDL::ChainFkSolverPos_recursive right_fk_solver = setup_right_kdl(*fdata);

  // Calculate costs
  double total_cost = compute_myfunc_helper(x_mat, left_fk_solver, right_fk_solver, fdata);

  // Display debug information
  Matrix<double, ARM_DOF, ARM_DOF> M_l, M_r;
  for (unsigned int r = 0; r < ARM_DOF; r++)
  {
    for (unsigned int c = 0; c < ARM_DOF; c++)
    {
      M_l(r, c) = x[r * ARM_DOF + c];
      M_r(r, c) = x[r * ARM_DOF + c + ARM_DOF * ARM_DOF];
    }
  }
  Matrix<double, 4, 1> elbow_wrist_cost = evaluate_elbow_wrist_cost(left_fk_solver, right_fk_solver, M_l, M_r, 
                                                                    fdata->l_num_wrist_seg, fdata->r_num_wrist_seg, 
                                                                    fdata->l_num_elbow_seg, fdata->r_num_elbow_seg,
                                                                    fdata);
  std::cout << ">>>> Iteration " << (num_iter+1) << "/" << max_iter << ": "
            << "Total cost = " << total_cost << " || "
            << "l_wrist_cost = " << elbow_wrist_cost[0] << ", "
            << "r_wrist_cost = " << elbow_wrist_cost[1] << ", "
            << "l_elbow_cost = " << elbow_wrist_cost[2] << ", "
            << "r_elbow_cost = " << elbow_wrist_cost[3] << std::endl;;
  num_iter++;


  // Compute gradient using Numeric Differentiation
  // only compute gradient if not NULL
  if(!grad.empty())
  {
    double eps = 0.01;
    double cost_plus, cost_minus;
    Matrix<double, ARM_DOF*ARM_DOF*2, 1> delta_x = Matrix<double, ARM_DOF*ARM_DOF*2, 1>::Zero();
    for (unsigned int i = 0; i < x.size(); i++)
    {
      // set eps
      delta_x[i] = eps;

      // compute numeric differentiation
      cost_plus = compute_myfunc_helper(x_mat+delta_x, left_fk_solver, right_fk_solver, fdata);
      cost_minus = compute_myfunc_helper(x_mat-delta_x, left_fk_solver, right_fk_solver, fdata);
      grad[i] = (cost_plus - cost_minus) / (2*eps);

      // reset eps
      delta_x[i] = 0.0;
    }
  }

  // Return cost function value
  return total_cost;
}


/**
 * Inequality Constraint function; expected to be my_inequality_constraint(x)<=0
 * 
 * For position limits.
 */
void MyNLopt::my_inequality_constraint_pos(unsigned m, double *result, unsigned n, const double *x,
                                      double *grad, /* NULL if not needed */
                                      void *f_data)
{
  // Convert the optimization variable to required data format
  Matrix<double, ARM_DOF*ARM_DOF*2, 1> x_mat;
  for (unsigned d = 0; d < ARM_DOF*ARM_DOF*2; d++)
    x_mat[d] = x[d];

  // Calculate constraint values
  Matrix<double, 2*2*ARM_DOF, 1> error = calculate_ineq_constraints_pos_helper(x_mat, f_data);

  // Convert calculated constraint values to required data format
  for (unsigned d = 0; d < 4*ARM_DOF; d++)
    result[d] = error[d];

  // Display for debug
  // for (unsigned int i = 0; i < 2*2*ARM_DOF; i++)
  //   std::cout << result[i] << " ";
  // std::cout << std::endl;

  // Calculate numeric differentiation for SQP
  double eps = 1e-5; //0.01;
  Matrix<double, ARM_DOF*ARM_DOF*2, 1> delta_x = Matrix<double, ARM_DOF*ARM_DOF*2, 1>::Zero();
  Matrix<double, 2*2*ARM_DOF, 1> error_plus, error_minus;
  if (grad)
  {
    for (unsigned int j = 0; j < n; j++) // n is DOF, ARM_DOF*ARM_DOF
    {
      // set eps for plus
      delta_x[j] = eps;

      // numeric differentiation
      error_plus = calculate_ineq_constraints_pos_helper(x_mat+delta_x, f_data);
      error_minus = calculate_ineq_constraints_pos_helper(x_mat-delta_x, f_data);
      Matrix<double, 2*2*ARM_DOF, 1> numeric_grad = (error_plus - error_minus) / (2*eps);

      // assign
      for (unsigned int i = 0; i < m; i++) // m is the error vector dimension, 3*2*ARM_DOF
        grad[i*n+j] = numeric_grad[i];

      // reset 
      delta_x[j] = 0.0;
    }
  }

  // debug
  // std::cout << "debug: pos ineq con, grad = ";
  // for (unsigned int i = 0; i < m; i++)
  // {
  //   for (unsigned int j = 0; j < n; j++)
  //     std::cout << grad[i * m + j] << " ";
  //   std::cout << std::endl;
  // }
  // std::cout << "done." << std::endl;

}



/** 
 * Inequality Constraint function; expected to be my_inequality_constraint(x)<=0
 * 
 * For velocity limits.
 */
void MyNLopt::my_inequality_constraint_vel(unsigned m, double *result, unsigned n, const double *x,
                                      double *grad, /* NULL if not needed */
                                      void *f_data)
{
  // Convert the optimization variable to required data format
  Matrix<double, ARM_DOF*ARM_DOF*2, 1> x_mat;
  for (unsigned d = 0; d < ARM_DOF*ARM_DOF*2; d++)
    x_mat[d] = x[d];

  // Calculate constraint values
  Matrix<double, 2*2*ARM_DOF, 1> error = calculate_ineq_constraints_vel_helper(x_mat, f_data);

  // Convert calculated constraint values to required data format
  for (unsigned d = 0; d < 4*ARM_DOF; d++)
    result[d] = error[d];

  // Display for debug
  // for (unsigned int i = 0; i < 2*2*ARM_DOF; i++)
  //   std::cout << result[i] << " ";
  // std::cout << std::endl;

  // Calculate numeric differentiation for SQP
  double eps = 1e-5; //0.01;
  Matrix<double, ARM_DOF*ARM_DOF*2, 1> delta_x = Matrix<double, ARM_DOF*ARM_DOF*2, 1>::Zero();
  Matrix<double, 2*2*ARM_DOF, 1> error_plus, error_minus;
  if (grad)
  {
    for (unsigned int j = 0; j < n; j++) // n is DOF, ARM_DOF*ARM_DOF
    {
      // set eps for plus
      delta_x[j] = eps;

      // numeric differentiation
      error_plus = calculate_ineq_constraints_vel_helper(x_mat+delta_x, f_data);
      error_minus = calculate_ineq_constraints_vel_helper(x_mat-delta_x, f_data);
      Matrix<double, 2*2*ARM_DOF, 1> numeric_grad = (error_plus - error_minus) / (2*eps);

      // assign
      for (unsigned int i = 0; i < m; i++) // m is the error vector dimension, 3*2*ARM_DOF
        grad[i*n+j] = numeric_grad[i];

      // reset 
      delta_x[j] = 0.0;
    }
  }

  // debug
  // std::cout << "debug: vel ineq con, grad = ";
  // for (unsigned int i = 0; i < m; i++)
  // {
  //   for (unsigned int j = 0; j < n; j++)
  //     std::cout << grad[i * m + j] << " ";
  //   std::cout << std::endl;
  // }
  // std::cout << "done." << std::endl;

}



/** 
 * Inequality Constraint function; expected to be my_inequality_constraint(x)<=0
 * 
 * For acceleration limits.
 */
void MyNLopt::my_inequality_constraint_acc(unsigned m, double *result, unsigned n, const double *x,
                                      double *grad, /* NULL if not needed */
                                      void *f_data)
{
  // Convert the optimization variable to required data format
  Matrix<double, ARM_DOF*ARM_DOF*2, 1> x_mat;
  for (unsigned d = 0; d < ARM_DOF*ARM_DOF*2; d++)
    x_mat[d] = x[d];

  // Calculate constraint values
  Matrix<double, 2*2*ARM_DOF, 1> error = calculate_ineq_constraints_acc_helper(x_mat, f_data);

  // Convert calculated constraint values to required data format
  for (unsigned d = 0; d < 4*ARM_DOF; d++)
    result[d] = error[d];

  // Display for debug
  // for (unsigned int i = 0; i < 2*2*ARM_DOF; i++)
  //   std::cout << result[i] << " ";
  // std::cout << std::endl;

  // Calculate numeric differentiation for SQP
  double eps = 1e-5; //0.01;
  Matrix<double, ARM_DOF*ARM_DOF*2, 1> delta_x = Matrix<double, ARM_DOF*ARM_DOF*2, 1>::Zero();
  Matrix<double, 2*2*ARM_DOF, 1> error_plus, error_minus;
  if (grad)
  {
    for (unsigned int j = 0; j < n; j++) // n is DOF, ARM_DOF*ARM_DOF
    {
      // set eps for plus
      delta_x[j] = eps;

      // numeric differentiation
      error_plus = calculate_ineq_constraints_acc_helper(x_mat+delta_x, f_data);
      error_minus = calculate_ineq_constraints_acc_helper(x_mat-delta_x, f_data);
      Matrix<double, 2*2*ARM_DOF, 1> numeric_grad = (error_plus - error_minus) / (2*eps);

      // assign
      for (unsigned int i = 0; i < m; i++) // m is the error vector dimension, 3*2*ARM_DOF
        grad[i*n+j] = numeric_grad[i];

      // reset 
      delta_x[j] = 0.0;
    }
  }

  // debug
  // std::cout << "debug: acc ineq con, grad = ";
  // for (unsigned int i = 0; i < m; i++)
  // {
  //   for (unsigned int j = 0; j < n; j++)
  //     std::cout << grad[i * m + j] << " ";
  //   std::cout << std::endl;
  // }
  // std::cout << "done." << std::endl;

}


/**
 * Helper function for computing joint angle equality constraints and the corresponding gradients.
 * 
 * Order is: pos_l, pos_r
 * @param[in]   pos_id    The index of the path point on which to apply joint angle equality constraint.
 */
Matrix<double, 2*ARM_DOF, 1> MyNLopt::calculate_eq_constraints_pos_helper(Matrix<double, ARM_DOF*ARM_DOF*2, 1> x, unsigned int pos_id, OptimUserData *fdata)
{
  // Reconstruct affine transform matrix from vector
  Matrix<double, ARM_DOF, ARM_DOF> mat_l, mat_r;
  for (unsigned int r = 0; r < ARM_DOF; r++)
  {
    for (unsigned int c = 0; c < ARM_DOF; c++)
    {
      mat_l(r, c) = x[r * ARM_DOF + c]; // first is for left arm
      mat_r(r, c) = x[r * ARM_DOF + c + ARM_DOF * ARM_DOF]; // second is for right arm
    }
  }
  
  // Left and right final point's positions are what we wish to preserve
  Matrix<double, ARM_DOF, 1> l_joint_angles_ik_begin = fdata->l_joint_angles_ik[0];
  Matrix<double, ARM_DOF, 1> r_joint_angles_ik_begin = fdata->r_joint_angles_ik[0];
  Matrix<double, ARM_DOF, 1> l_joint_angles_ik_target = fdata->l_joint_angles_ik[pos_id];
  Matrix<double, ARM_DOF, 1> r_joint_angles_ik_target = fdata->r_joint_angles_ik[pos_id];

  Matrix<double, ARM_DOF, 1> error_pos_l = l_joint_angles_ik_target - l_joint_angles_ik_begin;
  Matrix<double, ARM_DOF, 1> error_pos_r = r_joint_angles_ik_target - r_joint_angles_ik_begin;

  // order: l pos, r pos
  Matrix<double, 2*ARM_DOF, 1> error;
  error.block(0, 0, ARM_DOF, 1) = (mat_l - Matrix<double, ARM_DOF, ARM_DOF>::Identity()) * error_pos_l;
  error.block(ARM_DOF, 0, ARM_DOF, 1) = (mat_r - Matrix<double, ARM_DOF, ARM_DOF>::Identity()) * error_pos_r;

  // debug:
  // std::cout << "pos eq error = " << error.transpose() << std::endl;

  return error;
}


/**
 * Helper function for computing joint velocity equality constraints and the corresponding gradients.
 * 
 * Order is: vel_l, vel_r
 * @param[in]   vel_id    The index of the path point on which to apply joint velocity equality constraint.
 */
Matrix<double, 2*ARM_DOF, 1> MyNLopt::calculate_eq_constraints_vel_helper(Matrix<double, ARM_DOF*ARM_DOF*2, 1> x, unsigned int vel_id, OptimUserData *fdata)
{

  // Reconstruct affine transform matrix from vector
  Matrix<double, ARM_DOF, ARM_DOF> mat_l, mat_r;
  for (unsigned int r = 0; r < ARM_DOF; r++)
  {
    for (unsigned int c = 0; c < ARM_DOF; c++)
    {
      mat_l(r, c) = x[r * ARM_DOF + c]; // first is for left arm
      mat_r(r, c) = x[r * ARM_DOF + c + ARM_DOF * ARM_DOF]; // second is for right arm
    }
  }
  
  // Left and right final point's position and velocity are what we wish to preserve
  Matrix<double, ARM_DOF, 1> l_joint_angles_ik_begin = fdata->l_joint_angles_ik[0];
  Matrix<double, ARM_DOF, 1> r_joint_angles_ik_begin = fdata->r_joint_angles_ik[0];
  Matrix<double, ARM_DOF, 1> l_joint_angles_ik_last = fdata->l_joint_angles_ik[vel_id];
  Matrix<double, ARM_DOF, 1> r_joint_angles_ik_last = fdata->r_joint_angles_ik[vel_id];
  Matrix<double, ARM_DOF, 1> l_joint_angles_ik_last_but_one = fdata->l_joint_angles_ik[vel_id-1];
  Matrix<double, ARM_DOF, 1> r_joint_angles_ik_last_but_one = fdata->r_joint_angles_ik[vel_id-1];

  double dt = 0.1; // doesn't matter actually
  Matrix<double, ARM_DOF, 1> error_vel_l = (l_joint_angles_ik_last - l_joint_angles_ik_last_but_one) - Matrix<double, ARM_DOF, 1>::Zero();
  Matrix<double, ARM_DOF, 1> error_vel_r = (r_joint_angles_ik_last - r_joint_angles_ik_last_but_one) - Matrix<double, ARM_DOF, 1>::Zero(); 

  // Order: l vel, r vel
  Matrix<double, ARM_DOF*2, 1> error;
  error.block(0, 0, ARM_DOF, 1) = (mat_l - Matrix<double, ARM_DOF, ARM_DOF>::Identity()) * error_vel_l;
  error.block(ARM_DOF, 0, ARM_DOF, 1) = (mat_r - Matrix<double, ARM_DOF, ARM_DOF>::Identity()) * error_vel_r;  

  // debug:
  // std::cout << "vel eq error = " << error.transpose() << std::endl;

  return error;
}




/**
 * Helper function for computing position limit inequality constraints and the corresponding gradients.
 * 
 * We merge pos/vel/acc constraint values of all path points together. 
 * Order is: pos_ub_l, pos_lb_l, pos_ub_r, pos_lb_r
 */
Matrix<double, 2*2*ARM_DOF, 1> MyNLopt::calculate_ineq_constraints_pos_helper(Matrix<double, ARM_DOF*ARM_DOF*2, 1> x, void *f_data)
{
  // Initialization
  Matrix<double, 2*2*ARM_DOF, 1> error = Matrix<double, 2*2*ARM_DOF, 1>::Zero();

  // Obtain goal
  OptimUserData *fdata = (OptimUserData *) f_data;  

  // Left and right final point's position and velocity are what we wish to preserve
  unsigned int num_datapoints = fdata->l_joint_angles_ik.size();
  Matrix<double, ARM_DOF, 1> Q_tau_l = fdata->l_joint_angles_ik[0];
  Matrix<double, ARM_DOF, 1> Q_tau_r = fdata->r_joint_angles_ik[0];

  // Reconstruct affine transform matrices from vector
  Matrix<double, ARM_DOF, ARM_DOF> M_l, M_r;
  for (unsigned int r = 0; r < ARM_DOF; r++)
  {
    for (unsigned int c = 0; c < ARM_DOF; c++)
    {
      M_l(r, c) = x[r * ARM_DOF + c]; // first is for left arm
      M_r(r, c) = x[r * ARM_DOF + c + ARM_DOF * ARM_DOF]; // second is for right arm
    }
  }

  // Position limits
  Matrix<double, ARM_DOF, 1> pos_ub_error_l = Matrix<double, ARM_DOF, 1>::Zero();
  Matrix<double, ARM_DOF, 1> pos_lb_error_l = Matrix<double, ARM_DOF, 1>::Zero();
  Matrix<double, ARM_DOF, 1> pos_ub_error_r = Matrix<double, ARM_DOF, 1>::Zero();
  Matrix<double, ARM_DOF, 1> pos_lb_error_r = Matrix<double, ARM_DOF, 1>::Zero();

  // get the position bounds
  Matrix<double, ARM_DOF, 1> pos_ub_l = fdata->pos_ub_l;
  Matrix<double, ARM_DOF, 1> pos_lb_l = fdata->pos_lb_l;
  Matrix<double, ARM_DOF, 1> pos_ub_r = fdata->pos_ub_r;
  Matrix<double, ARM_DOF, 1> pos_lb_r = fdata->pos_lb_r;

  // get the index of the queried path point
  // unsigned int t = fdata->pos_id;
  for (unsigned int t = 0; t < fdata->num_datapoints; t++)
  {
  // obtain the current joint angles
  Matrix<double, ARM_DOF, 1> Q_cur_l = fdata->l_joint_angles_ik[t];
  Matrix<double, ARM_DOF, 1> Q_cur_r = fdata->r_joint_angles_ik[t];

  // calculate position limit constraint values 
  pos_ub_error_l = Q_tau_l + M_l * (Q_cur_l - Q_tau_l) - pos_ub_l;
  pos_lb_error_l = -Q_tau_l - M_l * (Q_cur_l - Q_tau_l) + pos_lb_l;
  pos_ub_error_r = Q_tau_r + M_r * (Q_cur_r - Q_tau_r) - pos_ub_r;
  pos_lb_error_r = -Q_tau_r - M_r * (Q_cur_r - Q_tau_r) + pos_lb_l;

  // apply max function for saving those that are greater than 0 (ignore those that are lower than 0 because we want my_inequ_con(x) <= 0)
  pos_ub_error_l = (pos_ub_error_l + pos_ub_error_l.cwiseAbs()) / 2.0;
  pos_lb_error_l = (pos_lb_error_l + pos_lb_error_l.cwiseAbs()) / 2.0;
  pos_ub_error_r = (pos_ub_error_r + pos_ub_error_r.cwiseAbs()) / 2.0;
  pos_lb_error_r = (pos_lb_error_r + pos_lb_error_r.cwiseAbs()) / 2.0;

  // save
  error.block(0, 0, ARM_DOF, 1) += pos_ub_error_l;
  error.block(ARM_DOF, 0, ARM_DOF, 1) += pos_lb_error_l;
  error.block(2*ARM_DOF, 0, ARM_DOF, 1) += pos_ub_error_r;
  error.block(3*ARM_DOF, 0, ARM_DOF, 1) += pos_lb_error_r;
  }

  return error;
}


/**
 * Helper function for computing velocity limits inequality constraints and the corresponding gradients.
 * 
 * We merge pos/vel/acc constraint values of all path points together. 
 * Order is: vel_ub_l, vel_lb_l, vel_ub_r, vel_lb_r.
 */
Matrix<double, 2*2*ARM_DOF, 1> MyNLopt::calculate_ineq_constraints_vel_helper(Matrix<double, ARM_DOF*ARM_DOF*2, 1> x, void *f_data)
{

  // Time interval
  double dt = 1.0/15.0; // use 15 Hz?

  // Initialization
  Matrix<double, 2*2*ARM_DOF, 1> error = Matrix<double, 2*2*ARM_DOF, 1>::Zero();

  // Obtain goal
  OptimUserData *fdata = (OptimUserData *) f_data;  

  // Left and right final point's position and velocity are what we wish to preserve
  unsigned int num_datapoints = fdata->l_joint_angles_ik.size();
  Matrix<double, ARM_DOF, 1> Q_tau_l = fdata->l_joint_angles_ik[0];
  Matrix<double, ARM_DOF, 1> Q_tau_r = fdata->r_joint_angles_ik[0];

  // Reconstruct affine transform matrices from vector
  Matrix<double, ARM_DOF, ARM_DOF> M_l, M_r;
  for (unsigned int r = 0; r < ARM_DOF; r++)
  {
    for (unsigned int c = 0; c < ARM_DOF; c++)
    {
      M_l(r, c) = x[r * ARM_DOF + c]; // first is for left arm
      M_r(r, c) = x[r * ARM_DOF + c + ARM_DOF * ARM_DOF]; // second is for right arm
    }
  }
  
  // Velocity limits
  Matrix<double, ARM_DOF, 1> vel_ub_error_l = Matrix<double, ARM_DOF, 1>::Zero();
  Matrix<double, ARM_DOF, 1> vel_lb_error_l = Matrix<double, ARM_DOF, 1>::Zero();
  Matrix<double, ARM_DOF, 1> vel_ub_error_r = Matrix<double, ARM_DOF, 1>::Zero();
  Matrix<double, ARM_DOF, 1> vel_lb_error_r = Matrix<double, ARM_DOF, 1>::Zero();

  // get the position bounds
  Matrix<double, ARM_DOF, 1> vel_ub_l = fdata->vel_ub_l;
  Matrix<double, ARM_DOF, 1> vel_lb_l = fdata->vel_lb_l;
  Matrix<double, ARM_DOF, 1> vel_ub_r = fdata->vel_ub_r;
  Matrix<double, ARM_DOF, 1> vel_lb_r = fdata->vel_lb_r;

  // get the indices of the queried path points
  // unsigned t0 = fdata->vel_id_0;
  // unsigned t1 = fdata->vel_id_1;

  for (unsigned int t = 0; t < fdata->num_datapoints-1; t++)
  {

    unsigned int t0 = t;
    unsigned int t1 = t+1;
  
  // obtain the current joint angles
  Matrix<double, ARM_DOF, 1> Q_cur_l = fdata->l_joint_angles_ik[t0];
  Matrix<double, ARM_DOF, 1> Q_cur_r = fdata->r_joint_angles_ik[t0];
  Matrix<double, ARM_DOF, 1> Q_next_l = fdata->l_joint_angles_ik[t1];
  Matrix<double, ARM_DOF, 1> Q_next_r = fdata->r_joint_angles_ik[t1];

  // calcualte velocity limit constraint values
  vel_ub_error_l = M_l * (Q_next_l - Q_cur_l) / dt - vel_ub_l;
  vel_lb_error_l = -M_l * (Q_next_l - Q_cur_l) / dt + vel_lb_l;
  vel_ub_error_r = M_r * (Q_next_r - Q_cur_r) / dt - vel_ub_r;
  vel_lb_error_r = -M_r * (Q_next_r - Q_cur_r) / dt + vel_lb_r;

  // Apply max function to save only those that are greater than 0 (because we want for my_ineq_con(x)<=0 to be satisfied)
  vel_ub_error_l = (vel_ub_error_l + vel_ub_error_l.cwiseAbs()) / 2.0;
  vel_lb_error_l = (vel_lb_error_l + vel_lb_error_l.cwiseAbs()) / 2.0;
  vel_ub_error_r = (vel_ub_error_r + vel_ub_error_r.cwiseAbs()) / 2.0;
  vel_lb_error_r = (vel_lb_error_r + vel_lb_error_r.cwiseAbs()) / 2.0;

  // debug:
  // std::cout << "vel_ub_error_l = " << vel_ub_error_l.transpose() << std::endl;
  // std::cout << "vel_lb_error_l = " << vel_lb_error_l.transpose() << std::endl;
  // std::cout << "vel_ub_error_r = " << vel_ub_error_r.transpose() << std::endl;
  // std::cout << "vel_lb_error_r = " << vel_lb_error_r.transpose() << std::endl;

  // accumulate the values across different path points
  error.block(0, 0, ARM_DOF, 1) += vel_ub_error_l;
  error.block(ARM_DOF, 0, ARM_DOF, 1) += vel_lb_error_l;
  error.block(2*ARM_DOF, 0, ARM_DOF, 1) += vel_ub_error_r;
  error.block(3*ARM_DOF, 0, ARM_DOF, 1) += vel_lb_error_r;
  
  }

  return error;
}


/**
 * Helper function for computing inequality constraints and the corresponding gradients.
 * 
 * Order is: acc_ub_l, acc_lb_l, acc_ub_r, acc_lb_r.
 */
Matrix<double, 2*2*ARM_DOF, 1> MyNLopt::calculate_ineq_constraints_acc_helper(Matrix<double, ARM_DOF*ARM_DOF*2, 1> x, void *f_data)
{

  // Time interval
  double dt = 1.0/15.0; // use 15 Hz?

  // Initialization
  Matrix<double, 2*2*ARM_DOF, 1> error = Matrix<double, 2*2*ARM_DOF, 1>::Zero();

  // Obtain goal
  OptimUserData *fdata = (OptimUserData *) f_data;  

  // Left and right final point's position and velocity are what we wish to preserve
  unsigned int num_datapoints = fdata->l_joint_angles_ik.size();
  Matrix<double, ARM_DOF, 1> Q_tau_l = fdata->l_joint_angles_ik[0];
  Matrix<double, ARM_DOF, 1> Q_tau_r = fdata->r_joint_angles_ik[0];

  // Reconstruct affine transform matrices from vector
  Matrix<double, ARM_DOF, ARM_DOF> M_l, M_r;
  for (unsigned int r = 0; r < ARM_DOF; r++)
  {
    for (unsigned int c = 0; c < ARM_DOF; c++)
    {
      M_l(r, c) = x[r * ARM_DOF + c]; // first is for left arm
      M_r(r, c) = x[r * ARM_DOF + c + ARM_DOF * ARM_DOF]; // second is for right arm
    }
  }

  // Acceleration limits
  Matrix<double, ARM_DOF, 1> acc_ub_error_l = Matrix<double, ARM_DOF, 1>::Zero();
  Matrix<double, ARM_DOF, 1> acc_lb_error_l = Matrix<double, ARM_DOF, 1>::Zero();
  Matrix<double, ARM_DOF, 1> acc_ub_error_r = Matrix<double, ARM_DOF, 1>::Zero();
  Matrix<double, ARM_DOF, 1> acc_lb_error_r = Matrix<double, ARM_DOF, 1>::Zero();

  // get the position bounds
  Matrix<double, ARM_DOF, 1> acc_ub_l = fdata->acc_ub_l;
  Matrix<double, ARM_DOF, 1> acc_lb_l = fdata->acc_lb_l;
  Matrix<double, ARM_DOF, 1> acc_ub_r = fdata->acc_ub_r;
  Matrix<double, ARM_DOF, 1> acc_lb_r = fdata->acc_lb_r;

  // get the indices of the queried path points
  unsigned int t0 = fdata->acc_id_0;
  unsigned int t1 = fdata->acc_id_1;
  unsigned int t2 = fdata->acc_id_2;

  for (unsigned int t = 0; t < fdata->num_datapoints-2; t++)
  {
    unsigned int t0 = t;
    unsigned int t1 = t+1;
    unsigned int t2 = t+2;

  // obtain the current joint angles
  Matrix<double, ARM_DOF, 1> Q_cur_l = fdata->l_joint_angles_ik[t0];
  Matrix<double, ARM_DOF, 1> Q_cur_r = fdata->r_joint_angles_ik[t0];
  Matrix<double, ARM_DOF, 1> Q_next_l = fdata->l_joint_angles_ik[t1];
  Matrix<double, ARM_DOF, 1> Q_next_r = fdata->r_joint_angles_ik[t1];
  Matrix<double, ARM_DOF, 1> Q_next_next_l = fdata->l_joint_angles_ik[t2];
  Matrix<double, ARM_DOF, 1> Q_next_next_r = fdata->r_joint_angles_ik[t2];

  // calculate accerleration limit constraint values across different path poitns (because we only want my_ineq_con(x)<= to be satisfied)
  acc_ub_error_l = M_l * (Q_next_next_l - 2 * Q_next_l + Q_cur_l) / (dt*dt) - acc_ub_l;
  acc_lb_error_l = -M_l * (Q_next_next_l - 2 * Q_next_l + Q_cur_l) / (dt*dt) + acc_lb_l;
  acc_ub_error_r = M_r * (Q_next_next_r - 2 * Q_next_r + Q_cur_r) / (dt*dt) - acc_ub_r;
  acc_lb_error_r = -M_r * (Q_next_next_r - 2 * Q_next_r + Q_cur_r) / (dt*dt) + acc_lb_r;

  // save only those that are greater than
  acc_ub_error_l = (acc_ub_error_l + acc_ub_error_l.cwiseAbs()) / 2.0;
  acc_lb_error_l = (acc_lb_error_l + acc_lb_error_l.cwiseAbs()) / 2.0;
  acc_ub_error_r = (acc_ub_error_r + acc_ub_error_r.cwiseAbs()) / 2.0;
  acc_lb_error_r = (acc_lb_error_r + acc_lb_error_r.cwiseAbs()) / 2.0;

  // accumulate the results
  error.block(0, 0, ARM_DOF, 1) += acc_ub_error_l;
  error.block(ARM_DOF, 0, ARM_DOF, 1) += acc_lb_error_l;
  error.block(2*ARM_DOF, 0, ARM_DOF, 1) += acc_ub_error_r;
  error.block(3*ARM_DOF, 0, ARM_DOF, 1) += acc_lb_error_r;
  }

  return error;
}




/**
 * Equality Constraint function; expected to be my_equality_constraint(x)=0
 * 
 * For joint angle equality constraint.
 */
void MyNLopt::my_equality_constraint_pos(unsigned m, double *result, unsigned n, const double *x,
                                     double *grad, /* NULL if not needed */
                                     void *f_data)
{
  // Obtain goal
  OptimUserData *fdata = (OptimUserData *) f_data;
  
  // Convert optimization variable to desired data structure
  Matrix<double, ARM_DOF*ARM_DOF*2, 1> x_mat;
  for (unsigned int i = 0; i < ARM_DOF*ARM_DOF*2; i++)
    x_mat[i] = x[i];

  // Compute error
  unsigned int pos_id = fdata->num_datapoints-1;
  Matrix<double, 2*ARM_DOF, 1> error = calculate_eq_constraints_pos_helper(x_mat, pos_id, fdata); // apply joint angle equality constraint on the last path point
  
  // Compute gradients for SQP using numeric differentiation
  double eps = 1e-5; //0.01;
  Matrix<double, 2*ARM_DOF, 1> error_plus, error_minus;
  Matrix<double, ARM_DOF*ARM_DOF*2, 1> delta_x = Matrix<double, ARM_DOF*ARM_DOF*2, 1>::Zero();
  if (grad)
  {
    for (unsigned int j = 0; j < n; j++) // n is DOF, ARM_DOF*ARM_DOF
    {
      // set eps for plus
      delta_x[j] = eps;

      // numeric differentiation
      error_plus = calculate_eq_constraints_pos_helper(x_mat+delta_x, pos_id, fdata);
      error_minus = calculate_eq_constraints_pos_helper(x_mat-delta_x, pos_id, fdata);
      Matrix<double, 2*ARM_DOF, 1> numeric_grad = (error_plus - error_minus) / (2*eps);

      // assign
      for (unsigned int i = 0; i < m; i++) // m is the error vector dimension, 3*2*ARM_DOF
        grad[i*n+j] = numeric_grad[i];

      // reset 
      delta_x[j] = 0.0;
    }
  }
}



/**
 * Equality Constraint function; expected to be my_equality_constraint(x)=0
 * 
 * For joint velocity equality constraint.
 */
void MyNLopt::my_equality_constraint_vel(unsigned m, double *result, unsigned n, const double *x,
                                     double *grad, /* NULL if not needed */
                                     void *f_data)
{
  // Obtain goal
  OptimUserData *fdata = (OptimUserData *) f_data;
  
  // Convert optimization variable to desired data structure
  Matrix<double, ARM_DOF*ARM_DOF*2, 1> x_mat;
  for (unsigned int i = 0; i < ARM_DOF*ARM_DOF*2; i++)
    x_mat[i] = x[i];

  // Compute error
  unsigned int vel_id = fdata->num_datapoints - 2; // path point [vel_id] and [vel_id+1] will be used to derive the velocity
  Matrix<double, 2*ARM_DOF, 1> error = calculate_eq_constraints_vel_helper(x_mat, vel_id, fdata); // apply joint angle equality constraint on the last path point
  
  // Compute gradients for SQP using numeric differentiation
  double eps = 1e-5;//0.01;
  Matrix<double, 2*ARM_DOF, 1> error_plus, error_minus;
  Matrix<double, ARM_DOF*ARM_DOF*2, 1> delta_x = Matrix<double, ARM_DOF*ARM_DOF*2, 1>::Zero();
  if (grad)
  {
    for (unsigned int j = 0; j < n; j++) // n is DOF, ARM_DOF*ARM_DOF
    {
      // set eps for plus
      delta_x[j] = eps;

      // numeric differentiation
      error_plus = calculate_eq_constraints_vel_helper(x_mat+delta_x, vel_id, fdata);
      error_minus = calculate_eq_constraints_vel_helper(x_mat-delta_x, vel_id, fdata);
      Matrix<double, 2*ARM_DOF, 1> numeric_grad = (error_plus - error_minus) / (2*eps);

      // assign
      for (unsigned int i = 0; i < m; i++) // m is the error vector dimension, 3*2*ARM_DOF
        grad[i*n+j] = numeric_grad[i];

      // reset 
      delta_x[j] = 0.0;
    }
  }
}



/**
 * This is a helper function for calculating gradient matrix for equality constraint.
 * 
 * Assumption:
 *    Constraint value is f = M*error - error = (M-I)*error, where M is of the size n*n, and error is of the size n * 1.
 *    And we want to obtain the gradient matrix with the size of n x n^2.
 *    Elements of M are arranged as m_11, ..., m_1n, ..., m_n1, ..., m_nn, i.e. RowMajor. ((m_1*), (m_2*), ..., (m_n*)).
 * Note that we have f_i = sum(e_j * m_ij) - e_i, and we may derive partial derivative df_i / dm_ij = e_j, which is independent of i.
 */
Matrix<double, ARM_DOF, ARM_DOF*ARM_DOF> MyNLopt::compute_eq_con_grad_helper(Matrix<double, ARM_DOF, 1> error)
{
  // Initialization
  Matrix<double, ARM_DOF, ARM_DOF*ARM_DOF> grad_mat = Matrix<double, ARM_DOF, ARM_DOF*ARM_DOF>::Zero();

  // Assign the analytical partial derivatives
  grad_mat = error.transpose().replicate(ARM_DOF, ARM_DOF);

  return grad_mat;
}


/**
 * This is a helper function for calculating position constraint values.
 * 
 * Q_tau + M(Q_cur-Q_tau) - P_ub <= 0
 * -Q_tau - M(Q_cur-Q_tau) + P_lb <= 0
 *
 * Return order is: pos_ub_l, pos_lb_l, pos_ub_r, pos_lb_r
//  */
// Matrix<double, 4*ARM_DOF, 1> MyNLopt::calculate_position_constraint_values(Matrix<double, ARM_DOF, 1> pos_ub_l,
//                                                                           Matrix<double, ARM_DOF, 1> pos_lb_l,
//                                                                           Matrix<double, ARM_DOF, 1> pos_ub_r,
//                                                                           Matrix<double, ARM_DOF, 1> pos_lb_r,
//                                                                           Matrix<double, ARM_DOF, ARM_DOF> M_l, 
//                                                                           Matrix<double, ARM_DOF, ARM_DOF> M_r, 
//                                                                           Matrix<double, ARM_DOF, 1> Q_cur_l, 
//                                                                           Matrix<double, ARM_DOF, 1> Q_cur_r, 
//                                                                           Matrix<double, ARM_DOF, 1> Q_tau_l,
//                                                                           Matrix<double, ARM_DOF, 1> Q_tau_r)
// {
//   // Calculate constraint values
//   Matrix<double, ARM_DOF, 1> pos_ub_error_l = Q_tau_l + M_l * (Q_cur_l - Q_tau_l) - pos_ub_l;
//   Matrix<double, ARM_DOF, 1> pos_lb_error_l = -Q_tau_l - M_l * (Q_cur_l - Q_tau_l) + pos_lb_l;
//   Matrix<double, ARM_DOF, 1> pos_ub_error_r = Q_tau_r + M_r * (Q_cur_r - Q_tau_r) - pos_ub_r;
//   Matrix<double, ARM_DOF, 1> pos_lb_error_r = -Q_tau_r - M_r * (Q_cur_r - Q_tau_r) + pos_lb_l;
  
//   // Assign the constraint values
//   Matrix<double, 4*ARM_DOF, 1> pos_error;
//   pos_error.block(0, 0, ARM_DOF, 1) = pos_ub_error_l;
//   pos_error.block(ARM_DOF, 0, ARM_DOF, 1) = pos_lb_error_l;
//   pos_error.block(2*ARM_DOF, 0, ARM_DOF, 1) = pos_ub_error_r;
//   pos_error.block(3*ARM_DOF, 0, ARM_DOF, 1) = pos_lb_error_r;

//   return pos_error;
// }



bool MyNLopt::write_h5(const std::string file_name, const std::string group_name, const std::string dataset_name, const int ROW, const int COL, std::vector<std::vector<double>> data_vector)
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
    herr_t status = status = H5Eset_auto(H5E_DEFAULT, NULL, NULL);

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
std::vector<std::vector<double>> MyNLopt::read_h5(const std::string file_name, const std::string group_name, const std::string dataset_name)
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
    int ndims = dataspace.getSimpleExtentDims( dims_out, NULL);
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


MyNLopt::MyNLopt(int argc, char **argv, std::string in_file_name, std::string joint_ik_result_filename, std::string in_group_name, std::string out_file_name)// : dual_arm_dual_hand_collision(argc, argv, urdf_string, srdf_string)
{

  // Input Cartesian trajectories
  const unsigned int x_dim = ARM_DOF * ARM_DOF * 2; // 2 affine deformation matrices
  std::vector<double> x(x_dim, 0.0); // 2 matrices for both arms

  // Initialize identity matrices for both affine transform
  Matrix<double, ARM_DOF, ARM_DOF> M = Matrix<double, ARM_DOF, ARM_DOF>::Identity();
  for (unsigned int r = 0; r < ARM_DOF; r++)
  {
    for (unsigned int c = 0; c < ARM_DOF; c++)
    {
      x[r * ARM_DOF + c] = M(r, c);
      x[r * ARM_DOF + c + ARM_DOF * ARM_DOF] = M(r, c);
    }
  }

  std::cout << ">>>> Loading position goal trajectories and original joint trajectories..." << std::endl;
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

  // Original joint angles
  std::vector<std::vector<double>> read_l_joint_angles = read_h5(joint_ik_result_filename, in_group_name, "l_joint_angles_optim_ik_yumi"); // N * 7 size
  std::vector<std::vector<double>> read_r_joint_angles = read_h5(joint_ik_result_filename, in_group_name, "r_joint_angles_optim_ik_yumi"); // N * 7 size

  unsigned int num_datapoints = read_l_wrist_pos_traj.size(); 


  // Variables' bounds (keep consistent with our method in yumi_arm_retarget_g2o_similarity.cpp)
  std::vector<double> q_l_arm_lb = {-2.8, -2.49, -1.2, -1.7, -2.0, -1.5, -2.0};
  std::vector<double> q_l_arm_ub = {0.5, 0.75, 2.2, 1.4, 1.578, 2.1, 1.578};
  std::vector<double> q_r_arm_lb = {-0.5, -2.49, -2.2, -1.7, -2.0, -1.5, -2.0}; // modified on 2020/07/20
  std::vector<double> q_r_arm_ub = {2.8, 0.75, 1.2, 1.4, 1.578, 2.1, 1.578}; // modified on 2020/07/20
  std::vector<double> q_l_finger_lb = {-1.6, -1.6, -1.6, -1.6, -1.6, -1.6, -1.6, -1.6, -0.75, 0.0, -0.2, -0.15};
  std::vector<double> q_l_finger_ub = {0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.3,  0.1,  0.0,  0.0};
  std::vector<double> q_r_finger_lb = {-1.6, -1.6, -1.6, -1.6, -1.6, -1.6, -1.6, -1.6, -1.0, 0.0, -0.2, -0.15};
  std::vector<double> q_r_finger_ub = {0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.3, 0.1,  0.0,  0.0}; 

  // added by LYW on 2020/09/24
  std::vector<double> q_l_arm_vel_lb = {-M_PI, -M_PI, -M_PI, -M_PI, -2*M_PI, -2*M_PI, -2*M_PI}; 
  std::vector<double> q_l_arm_vel_ub = {M_PI, M_PI, M_PI, M_PI, 2*M_PI, 2*M_PI, 2*M_PI};
  std::vector<double> q_r_arm_vel_lb = {-M_PI, -M_PI, -M_PI, -M_PI, -2*M_PI, -2*M_PI, -2*M_PI}; 
  std::vector<double> q_r_arm_vel_ub = {M_PI, M_PI, M_PI, M_PI, 2*M_PI, 2*M_PI, 2*M_PI}; 

  std::vector<double> q_l_arm_acc_lb = {-100.0, -100.0, -100.0, -100.0, -100.0, -100.0, -100.0}; // can be Inf, so set it manually
  std::vector<double> q_l_arm_acc_ub = {100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0};
  std::vector<double> q_r_arm_acc_lb = {-100.0, -100.0, -100.0, -100.0, -100.0, -100.0, -100.0}; 
  std::vector<double> q_r_arm_acc_ub = {100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}; 


  std::vector<double> qlb = q_l_arm_lb;
  std::vector<double> qub = q_l_arm_ub;
  qlb.insert(qlb.end(), q_r_arm_lb.cbegin(), q_r_arm_lb.cend());
  // qlb.insert(qlb.end(), q_l_finger_lb.cbegin(), q_l_finger_lb.cend());
  // qlb.insert(qlb.end(), q_r_finger_lb.cbegin(), q_r_finger_lb.cend());
  qub.insert(qub.end(), q_r_arm_ub.cbegin(), q_r_arm_ub.cend());
  // qub.insert(qub.end(), q_l_finger_ub.cbegin(), q_l_finger_ub.cend());
  // qub.insert(qub.end(), q_r_finger_ub.cbegin(), q_r_finger_ub.cend());


  // Set up KDL FK solver (get solver, wrist ID and elbow ID)
  my_constraint_struct constraint_data; 
 

  // Set a distance computation class
  //DualArmDualHandMinDistance dual_arm_dual_hand_min_distance(argc, argv, urdf_string.str(), srdf_string.str());
  //constraint_data.dual_arm_dual_hand_min_distance = dual_arm_dual_hand_min_distance;
  constraint_data.argc = argc;
  constraint_data.argv = argv;
  //  constraint_data.urdf_string = urdf_string;//.str();
  //  constraint_data.srdf_string = srdf_string;//.str();


  // Pack up a constraint_data to pass in optimization
  // robot's shoulder position
  // Now it's only for use in mapping human hand joint angles to robot hands
  constraint_data.l_robot_shoulder_pos = Vector3d(0.05355, 0.0725, 0.51492); //Vector3d(-0.06, 0.235, 0.395);
  constraint_data.r_robot_shoulder_pos = Vector3d(0.05255, -0.0725, 0.51492); //Vector3d(-0.06, -0.235, 0.395);

  constraint_data.l_robot_finger_start <<  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.3,  0.1,  0.0,  0.0; 
  constraint_data.l_robot_finger_final << -1.6, -1.6, -1.6, -1.6, -1.6, -1.6, -1.6, -1.6, -0.75, 0.0, -0.2, -0.15;  
  constraint_data.r_robot_finger_start <<  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.3, 0.1,  0.0,  0.0;
  constraint_data.r_robot_finger_final << -1.6, -1.6, -1.6, -1.6, -1.6, -1.6, -1.6, -1.6, -1.0, 0.0, -0.2, -0.15; 
  constraint_data.glove_start << 0,    0, 53,  0,   0, 22,  0,   0, 22,  0,   0, 35,  0,   0;
  constraint_data.glove_start = constraint_data.glove_start * M_PI / 180.0; // in radius  
  constraint_data.glove_final << 45, 100,  0, 90, 120,  0, 90, 120,  0, 90, 120,  0, 90, 120;
  constraint_data.glove_final = constraint_data.glove_final * M_PI / 180.0; 


  // Optimization settings
  std::cout << ">>>> Setting variable bounds and stopping conditions..." << std::endl;
  double tol = 1e-4;
  double stopval = 1e-8;
  nlopt::opt opt(nlopt::LD_SLSQP, x_dim); // nlopt::LD_SLSQP  // Optimization variable is affine deformation matrix M!!!
  // opt.set_lower_bounds(qlb); // set lower bounds
  // opt.set_upper_bounds(qub); // set upper bounds
  opt.set_stopval(1e-12); // stop value
  opt.set_ftol_rel(1e-10); // objective function value changes by less than `tol` multiplied by the absolute value of the function value
  //opt.set_ftol_abs(1e-12); // objective function value changes by less than `tol`
  opt.set_xtol_rel(1e-8); // optimization parameters' magnitude changes by less than `tol` multiplied by the current magnitude(can set weights for each dimension)
  //opt.set_xtol_abs(1e-8); // optimization parameters' magnitude changes by less than `tol`
  opt.set_maxeval(max_iter); // maximum evaluation
  //opt.set_maxtime(3.0); // maximum time


  // std::vector container for storing costs history
  std::vector<std::vector<double> > q_results(num_datapoints, std::vector<double>(JOINT_DOF));
  std::vector<std::vector<double> > scaled_wrist_pos_cost_history(num_datapoints, std::vector<double>(1));  
  std::vector<std::vector<double> > scaled_elbow_pos_cost_history(num_datapoints, std::vector<double>(1));  
  std::vector<std::vector<double> > wrist_ori_cost_history(num_datapoints, std::vector<double>(1)); 

  std::vector<std::vector<double> > arm_cost_history(num_datapoints, std::vector<double>(1)); 
  std::vector<std::vector<double> > finger_cost_history(num_datapoints, std::vector<double>(1)); 
  std::vector<std::vector<double> > col_cost_history(num_datapoints, std::vector<double>(1)); 
  std::vector<std::vector<double> > smoothness_cost_history(num_datapoints, std::vector<double>(1)); 


  // For linearly mapping hand joint angles directly
  std::vector<Matrix<double, 12, 1> > l_robot_finger_angle(num_datapoints, Matrix<double, 12, 1>::Zero());
  std::vector<Matrix<double, 12, 1> > r_robot_finger_angle(num_datapoints, Matrix<double, 12, 1>::Zero());


  // Prepare user-defined data
  std::cout << ">>>> Preparing user-defined data for passing into optimization..." << std::endl;
  // initialization
  OptimUserData optim_user_data;
  // 
  optim_user_data.num_datapoints = num_datapoints;
  optim_user_data.l_robot_shoulder_pos = Vector3d(0.05355, 0.0725, 0.51492); //Vector3d(-0.06, 0.235, 0.395);
  optim_user_data.r_robot_shoulder_pos = Vector3d(0.05255, -0.0725, 0.51492); //Vector3d(-0.06, -0.235, 0.395);
  // position/velocity/acceleration's upper and lower bounds
  optim_user_data.pos_ub_l = Map<Matrix<double, ARM_DOF, 1>>(q_l_arm_ub.data(), ARM_DOF, 1);
  optim_user_data.pos_lb_l = Map<Matrix<double, ARM_DOF, 1>>(q_l_arm_lb.data(), ARM_DOF, 1);
  optim_user_data.pos_ub_r = Map<Matrix<double, ARM_DOF, 1>>(q_r_arm_ub.data(), ARM_DOF, 1);
  optim_user_data.pos_lb_r = Map<Matrix<double, ARM_DOF, 1>>(q_r_arm_lb.data(), ARM_DOF, 1);

  optim_user_data.vel_ub_l = Map<Matrix<double, ARM_DOF, 1>>(q_l_arm_vel_ub.data(), ARM_DOF, 1);
  optim_user_data.vel_lb_l = Map<Matrix<double, ARM_DOF, 1>>(q_l_arm_vel_lb.data(), ARM_DOF, 1);
  optim_user_data.vel_ub_r = Map<Matrix<double, ARM_DOF, 1>>(q_r_arm_vel_ub.data(), ARM_DOF, 1);
  optim_user_data.vel_lb_r = Map<Matrix<double, ARM_DOF, 1>>(q_r_arm_vel_lb.data(), ARM_DOF, 1);

  optim_user_data.acc_ub_l = Map<Matrix<double, ARM_DOF, 1>>(q_l_arm_acc_ub.data(), ARM_DOF, 1);
  optim_user_data.acc_lb_l = Map<Matrix<double, ARM_DOF, 1>>(q_l_arm_acc_lb.data(), ARM_DOF, 1);
  optim_user_data.acc_ub_r = Map<Matrix<double, ARM_DOF, 1>>(q_r_arm_acc_ub.data(), ARM_DOF, 1);
  optim_user_data.acc_lb_r = Map<Matrix<double, ARM_DOF, 1>>(q_r_arm_acc_lb.data(), ARM_DOF, 1);

  // KDL FK sovler
  KDL::ChainFkSolverPos_recursive left_fk_solver = setup_left_kdl(optim_user_data); // set IDs, discard the solver handle
  KDL::ChainFkSolverPos_recursive right_fk_solver = setup_right_kdl(optim_user_data); 

  // iterate to store Cartesian goal positions
  for (unsigned int it = 0; it < num_datapoints; it++)
  { 
    // Process the read data
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

    std::vector<double> l_joint_angles = read_l_joint_angles[it];
    std::vector<double> r_joint_angles = read_r_joint_angles[it];

    Vector3d l_shoulder_pos_goal = Map<Vector3d>(l_shoulder_pos.data(), 3, 1);
    Vector3d l_elbow_pos_goal = Map<Vector3d>(l_elbow_pos.data(), 3, 1);
    Vector3d l_wrist_pos_goal = Map<Vector3d>(l_wrist_pos.data(), 3, 1);
    Matrix3d l_wrist_ori_goal = Map<Matrix<double, 3, 3, RowMajor>>(l_wrist_ori.data(), 3, 3);
    Vector3d r_shoulder_pos_goal = Map<Vector3d>(r_shoulder_pos.data(), 3, 1);
    Vector3d r_elbow_pos_goal = Map<Vector3d>(r_elbow_pos.data(), 3, 1);
    Vector3d r_wrist_pos_goal = Map<Vector3d>(r_wrist_pos.data(), 3, 1);
    Matrix3d r_wrist_ori_goal = Map<Matrix<double, 3, 3, RowMajor>>(r_wrist_ori.data(), 3, 3);

    Matrix<double, ARM_DOF, 1> l_joint_angles_original = Map<Matrix<double, ARM_DOF, 1>>(l_joint_angles.data(), ARM_DOF, 1);
    Matrix<double, ARM_DOF, 1> r_joint_angles_original = Map<Matrix<double, ARM_DOF, 1>>(r_joint_angles.data(), ARM_DOF, 1);

    Matrix<double, 14, 1> l_finger_pos_goal = Map<Matrix<double, 14, 1>>(l_finger_pos.data(), 14, 1);
    Matrix<double, 14, 1> r_finger_pos_goal = Map<Matrix<double, 14, 1>>(r_finger_pos.data(), 14, 1);


    // 2 - Linearly mapping human hand joint angles to robot hands (convert to radius)
    l_robot_finger_angle[it] = MyNLopt::map_finger_joint_values(l_finger_pos_goal * M_PI / 180.0, true, constraint_data);
    r_robot_finger_angle[it] = MyNLopt::map_finger_joint_values(r_finger_pos_goal * M_PI / 180.0, false, constraint_data);


    // Store in the user-defined data structure
    optim_user_data.l_shoulder_pos_goal.push_back(l_shoulder_pos_goal);
    optim_user_data.l_elbow_pos_goal.push_back(l_elbow_pos_goal);
    optim_user_data.l_wrist_pos_goal.push_back(l_wrist_pos_goal);
    optim_user_data.l_wrist_ori_goal.push_back(l_wrist_ori_goal);
    optim_user_data.r_shoulder_pos_goal.push_back(r_shoulder_pos_goal);
    optim_user_data.r_elbow_pos_goal.push_back(r_elbow_pos_goal);
    optim_user_data.r_wrist_pos_goal.push_back(r_wrist_pos_goal);
    optim_user_data.r_wrist_ori_goal.push_back(r_wrist_ori_goal);

    optim_user_data.l_joint_angles_ik.push_back(l_joint_angles_original);
    optim_user_data.r_joint_angles_ik.push_back(r_joint_angles_original);
    
  }


  // Re-position the human mocap data to a place suitable for robot execution
  // Pre-processing the human demonstrations
  std::cout << ">>>> Pre-processing on human demonstrations to fit robot configuration and workspace..." << std::endl;
  // way 1 - simply translate the whole trajectory to robot's shoulder
  /*
  for (unsigned int it = 0; it < optim_user_data.num_datapoints; it++)
  {
    optim_user_data.l_wrist_pos_goal[it] = optim_user_data.l_wrist_pos_goal[it] - optim_user_data.l_shoulder_pos_goal[it] + optim_user_data.l_robot_shoulder_pos;
    optim_user_data.l_elbow_pos_goal[it] = optim_user_data.l_elbow_pos_goal[it] - optim_user_data.l_shoulder_pos_goal[it] + optim_user_data.l_robot_shoulder_pos;
    optim_user_data.r_wrist_pos_goal[it] = optim_user_data.r_wrist_pos_goal[it] - optim_user_data.r_shoulder_pos_goal[it] + optim_user_data.r_robot_shoulder_pos;
    optim_user_data.r_elbow_pos_goal[it] = optim_user_data.r_elbow_pos_goal[it] - optim_user_data.r_shoulder_pos_goal[it] + optim_user_data.r_robot_shoulder_pos;
  }
  */
  

  // way 2 - set the initial position of human demonstrations in the same way as we do, i.e. set the x and z to appropriate fixed positions, and set y properly to keep it as symmetrical as possible
  // note that even so we still utilize different trajectories!!! because the reference trajectories we use are pre-processed according to human hand length and robot hand length...
  Vector3d lw_set_start; 
  Vector3d le_set_start; le_set_start << 0.218, 0.310, 0.378; 
  Vector3d rw_set_start; rw_set_start << 0.410, -0.179, 0.191; 
  Vector3d re_set_start; re_set_start << 0.218, -0.310, 0.377;  // manually set, same as what we use
  // set rw start to ensure rw start and lw start symmetric to x-z plane, and leave the others
  lw_set_start = rw_set_start + (optim_user_data.l_wrist_pos_goal[0] - optim_user_data.r_wrist_pos_goal[0]); // trajectory_generator_ptr->lrw_start; // use the start...
  double dist_y = std::abs(lw_set_start[1] - rw_set_start[1]);
  lw_set_start[1] = dist_y / 2.0;
  rw_set_start[1] = -dist_y / 2.0;
  // compute offsets
  Vector3d lw_set_offset = lw_set_start - optim_user_data.l_wrist_pos_goal[0];
  Vector3d le_set_offset = le_set_start - optim_user_data.l_elbow_pos_goal[0];
  Vector3d rw_set_offset = rw_set_start - optim_user_data.r_wrist_pos_goal[0];
  Vector3d re_set_offset = re_set_start - optim_user_data.r_elbow_pos_goal[0];
  for (unsigned int it = 0; it < optim_user_data.num_datapoints; it++)
  {
    optim_user_data.l_wrist_pos_goal[it] = optim_user_data.l_wrist_pos_goal[it] + lw_set_offset;
    optim_user_data.l_elbow_pos_goal[it] = optim_user_data.l_elbow_pos_goal[it] + le_set_offset;
    optim_user_data.r_wrist_pos_goal[it] = optim_user_data.r_wrist_pos_goal[it] + rw_set_offset;
    optim_user_data.r_elbow_pos_goal[it] = optim_user_data.r_elbow_pos_goal[it] + re_set_offset;
  }


  // way 3 - same trajectories as used for human IK (way 2 does a few modifications on the relative trajectories!!!! so it's not a good choice since human IK results are not solved according to that)
  /*
  Vector3d l_shoulder_pos_human = optim_user_data.l_shoulder_pos_goal[0];
  Vector3d r_shoulder_pos_human = optim_user_data.r_shoulder_pos_goal[0];
  Vector3d human_shoulder_center = (l_shoulder_pos_human + r_shoulder_pos_human) / 2.0;
  Vector3d manual_offset; manual_offset << 0.0, 0.0, 0.65; 
  for (unsigned int it = 0; it < optim_user_data.num_datapoints; it++)
  {
    optim_user_data.l_wrist_pos_goal[it] = optim_user_data.l_wrist_pos_goal[it] - human_shoulder_center + manual_offset;
    optim_user_data.l_elbow_pos_goal[it] = optim_user_data.l_elbow_pos_goal[it] - human_shoulder_center + manual_offset;
    optim_user_data.r_wrist_pos_goal[it] = optim_user_data.r_wrist_pos_goal[it] - human_shoulder_center + manual_offset;
    optim_user_data.r_elbow_pos_goal[it] = optim_user_data.r_elbow_pos_goal[it] - human_shoulder_center + manual_offset;
  }
  */


  // store the target trajs
  std::vector<std::vector<double>> l_wrist_pos_goal_store, r_wrist_pos_goal_store, l_elbow_pos_goal_store, r_elbow_pos_goal_store;
  for (unsigned int it = 0; it < optim_user_data.num_datapoints; it++)
  {
    std::vector<double> l_wrist_pos_goal_cur, r_wrist_pos_goal_cur, l_elbow_pos_goal_cur, r_elbow_pos_goal_cur;
    for (unsigned int d = 0; d < 3; d++)
    {
      l_wrist_pos_goal_cur.push_back(optim_user_data.l_wrist_pos_goal[it][d]);
      r_wrist_pos_goal_cur.push_back(optim_user_data.r_wrist_pos_goal[it][d]);
      l_elbow_pos_goal_cur.push_back(optim_user_data.l_elbow_pos_goal[it][d]);
      r_elbow_pos_goal_cur.push_back(optim_user_data.r_elbow_pos_goal[it][d]);
    }
    l_wrist_pos_goal_store.push_back(l_wrist_pos_goal_cur);
    r_wrist_pos_goal_store.push_back(r_wrist_pos_goal_cur);
    l_elbow_pos_goal_store.push_back(l_elbow_pos_goal_cur);
    r_elbow_pos_goal_store.push_back(r_elbow_pos_goal_cur);
  }
  h5_io::write_h5(out_file_name, in_group_name, "ref_l_wrist_traj_hujin", l_wrist_pos_goal_store);
  h5_io::write_h5(out_file_name, in_group_name, "ref_r_wrist_traj_hujin", r_wrist_pos_goal_store);
  h5_io::write_h5(out_file_name, in_group_name, "ref_l_elbow_traj_hujin", l_elbow_pos_goal_store);
  h5_io::write_h5(out_file_name, in_group_name, "ref_r_elbow_traj_hujin", r_elbow_pos_goal_store);


  // Set constraints
  std::cout << ">>>> Setting equality and inequality constraints..." << std::endl;
  OptimUserData *f_data = &optim_user_data;

  unsigned int ineq_con_dim = 2*2*ARM_DOF; // pos/vel/acc constraints for both arms (merge all path points together)
  unsigned int eq_con_dim = 2*ARM_DOF; // pos + vel, ub + lb
  const std::vector<double> ineq_tol(ineq_con_dim, 1e-5);  
  const std::vector<double> eq_tol(eq_con_dim, 1e-5);  
  opt.add_equality_mconstraint(this->my_equality_constraint_pos, (void *)f_data, eq_tol); 
  opt.add_equality_mconstraint(this->my_equality_constraint_vel, (void *)f_data, eq_tol); 
  opt.add_inequality_mconstraint(this->my_inequality_constraint_pos, (void *)f_data, ineq_tol);
  opt.add_inequality_mconstraint(this->my_inequality_constraint_vel, (void *)f_data, ineq_tol);
  opt.add_inequality_mconstraint(this->my_inequality_constraint_acc, (void *)f_data, ineq_tol);


  // Set up objective function and additional data to pass in
  std::cout << ">>>> Setting objective function..." << std::endl;
  opt.set_min_objective(this->myfunc, (void *)f_data); // set objective function to minimize; with no additional information passed(f_data)


  // Start optimization
  std::cout << ">>>> Start optimization..." << std::endl;
  double minf;
  nlopt::result opt_result;
  opt_result = opt.optimize(x, minf);

  switch(opt_result)
  {
    case nlopt::SUCCESS:
      std::cout << "Optimization finished successfully." << std::endl;
      break;
    case nlopt::STOPVAL_REACHED:
      std::cout << "Optimization terminated due to STOPVAL reached." << std::endl;
      break;
    case nlopt::FTOL_REACHED:
      std::cout << "Optimization terminated due to FTOL reached." << std::endl;
      break;
    case nlopt::XTOL_REACHED:
      std::cout << "Optimization terminated due to XTOL reached." << std::endl;
      break;
    case nlopt::MAXEVAL_REACHED:
      std::cout << "Optimization terminated due to MAXEVAL reached." << std::endl;
      break;
    case nlopt::MAXTIME_REACHED:
      std::cout << "Optimization terminated due to MAXTIME reached." << std::endl;
      break;
  }
  std::cout << "NLopt found minimum f: " << minf << " after " << opt.get_numevals() << " evaluations." << std::endl;

  // Do remember to clear the constraints after every iteration !!!
  // otherwise the number of constraints would add up by `m` after every iteration !!!
  std::cout << ">>>> Clearing constraints information..." << std::endl;
  opt.remove_inequality_constraints();
  opt.remove_equality_constraints(); // always good practice to use both, even though equality constraints are not used here.


  // Evaluate optimization results
  std::cout << ">>>> Evaluate optimization results..." << std::endl;
  // obtain affine deformation matrix
  Matrix<double, ARM_DOF, ARM_DOF> mat_l, mat_r;
  for (unsigned r = 0; r < ARM_DOF; r++)
  {
    for (unsigned c = 0; c < ARM_DOF; c++)
    {
      mat_l(r, c) = x[r * ARM_DOF + c]; // first is for left arm
      mat_r(r, c) = x[r * ARM_DOF + c + ARM_DOF * ARM_DOF]; // second is for right arm
    }
  }
  Matrix<double, 4, 1> error_vec = evaluate_elbow_wrist_cost(left_fk_solver, right_fk_solver, 
                                                              mat_l, mat_r, 
                                                              f_data->l_num_wrist_seg, f_data->r_num_wrist_seg, 
                                                              f_data->l_num_elbow_seg, f_data->r_num_elbow_seg, 
                                                              f_data);
  std::cout << "Left Wrist Pos Cost = " << error_vec[0] << std::endl
            << "Right Wrist Pos Cost = " << error_vec[1] << std::endl
            << "Left Elbow Pos Cost = " << error_vec[2] << std::endl
            << "Right Elbow Pos Cost = " << error_vec[3] << std::endl;


  // Store the results
  std::cout << ">>>> Store optimization results to h5 file..." << std::endl;
  const std::string group_name = in_group_name;
  // 1 - affine deformation matrices
  // bool result = write_h5(out_file_name, in_group_name, "affine_matrix_l", mat_l);
  // result = write_h5(out_file_name, in_group_name, "affine_matrix_r", mat_r);
  std::vector<std::vector<double>> x_store; x_store.push_back(x);
  h5_io::write_h5(out_file_name, in_group_name, "affine_matrices", x_store); // Left and right affine matrices, RowMajor
  // 2 - affine deformed new trajectories
  std::vector<std::vector<double>> l_joint_angles_deformed, r_joint_angles_deformed;
  for (unsigned int t = 0; t < num_datapoints; t++)
  {
    // Get deformed joint angles
    Matrix<double, ARM_DOF, 1> l_joint_angles_cur = f_data->l_joint_angles_ik[0] + mat_l * (f_data->l_joint_angles_ik[t] - f_data->l_joint_angles_ik[0]);
    Matrix<double, ARM_DOF, 1> r_joint_angles_cur = f_data->r_joint_angles_ik[0] + mat_r * (f_data->r_joint_angles_ik[t] - f_data->r_joint_angles_ik[0]);
    // Store
    std::vector<double> l_joint_angles_cur_vec, r_joint_angles_cur_vec;
    for (unsigned int d = 0; d < ARM_DOF; d++)
    {
      l_joint_angles_cur_vec.push_back(l_joint_angles_cur[d]);
      r_joint_angles_cur_vec.push_back(r_joint_angles_cur[d]);
    }
    l_joint_angles_deformed.push_back(l_joint_angles_cur_vec);
    r_joint_angles_deformed.push_back(r_joint_angles_cur_vec);
  }
  h5_io::write_h5(out_file_name, in_group_name, "arm_traj_affine_l", l_joint_angles_deformed);
  h5_io::write_h5(out_file_name, in_group_name, "arm_traj_affine_r", r_joint_angles_deformed);
  // 3 - actually tracked trajectories
  store_actual_trajs(mat_l, mat_r, out_file_name, in_group_name, f_data, "hujin");
  // 4 - the original trajectories (the ones obtained by directly mapping from human IK results)
  store_actual_trajs(Matrix<double, ARM_DOF, ARM_DOF>::Identity(), Matrix<double, ARM_DOF, ARM_DOF>::Identity(), out_file_name, in_group_name, f_data, "hujin_original");

}


int main(int argc, char **argv)
{
  
  // Initialize a ros node, for the calculation of collision distance
  // ros::init(argc, argv, "yumi_sign_language_robot_retarget_affine_deformation");

  // reset 
  // dual_arm_dual_hand_collision_ptr.reset( new DualArmDualHandCollision(::urdf_string.str(), ::srdf_string.str()) );
  //dual_arm_dual_hand_min_distance_ptr.reset( new DualArmDualHandMinDistance(::urdf_string.str(), ::srdf_string.str()) );


  // Settings 
  std::string urdf_file_name = "/home/liangyuwei/sign_language_robot_ws/src/yumi_description/urdf/yumi_with_hands.urdf";
  std::string srdf_file_name = "/home/liangyuwei/sign_language_robot_ws/src/yumi_sign_language_robot_moveit_config/config/yumi.srdf";

  std::string in_file_name = "/home/liangyuwei/sign_language_robot_ws/test_imi_data/test_imi_data_YuMi.h5";
  std::string joint_ik_result_filename = "/home/liangyuwei/sign_language_robot_ws/test_imi_data/test_imi_data.h5"; // the file that stores the mapped human IK results (calculated and mapped to YuMi's joint configuration in MATLAB)
  std::string in_group_name = "fengren_1";
  std::string out_file_name = "/home/liangyuwei/sign_language_robot_ws/test_imi_data/mocap_ik_results_YuMi_hujin.h5";
  
  // Process the terminal arguments
  static struct option long_options[] = 
  {
    {"in-h5-filename", required_argument, NULL, 'i'},
    {"in-group-name", required_argument, NULL, 'g'},
    {"out-h5-filename", required_argument, NULL, 'o'},
    {"joint-ik-result-filename", required_argument, NULL, 'j'},
    {"k-wrist-pos", required_argument, NULL, 'w'},
    {"k-elbow-pos", required_argument, NULL, 'e'},
    {"help", no_argument, NULL, 'h'},
    {0, 0, 0, 0}
  };
  int c;
  while(1)
  {
    int opt_index = 0;
    // Get arguments
    c = getopt_long(argc, argv, "i:g:o:j:hw:e:", long_options, &opt_index);
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
        std::cout << "    -j, --joint-ik-result-filename, specify the h5 file that stores the re-mapped results of human IK joint trajectories.\n" << std::endl;
        std::cout << "    -w, --k-wrist-pos, K_WRIST_POS for tracking.\n" << std::endl;
        std::cout << "    -e, --k-elbow-pos, K_ELBOW_POS for tracking.\n" << std::endl;
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

      case 'j':
        joint_ik_result_filename = optarg;
        break;
      
      case 'w':
        K_WRIST_POS = atof(optarg);
        break;
      
      case 'e':
        K_ELBOW_POS = atof(optarg);
        break;

      default:
        break;
    }

  }
  std::cout << "The input h5 file name is: " << in_file_name << std::endl;
  std::cout << "The input h5 file that stores re-mapped human IK results is: " << joint_ik_result_filename << std::endl;
  std::cout << "The motion name is: " << in_group_name << std::endl;
  std::cout << "The output h5 file name is: " << out_file_name << std::endl;
  std::cout << "K_WRIST_POS = " << K_WRIST_POS << std::endl;
  std::cout << "K_ELBOW_POS = " << K_ELBOW_POS << std::endl;

  // Start optimization
  MyNLopt my_nlopt(argc, argv, in_file_name, joint_ik_result_filename, in_group_name, out_file_name);
 
  return 0;
}



