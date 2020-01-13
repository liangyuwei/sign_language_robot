// ROS
#include <ros/ros.h>

// NLopt
#include <nlopt.hpp> // C++ version!!!

// Common
#include <vector>
#include <iostream>
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

// Process the terminal arguments
#include <getopt.h>


// global flags
int count = 0;
bool first_iter = true;

using namespace Eigen;
using namespace H5;

typedef struct {
  // Cost func related
  Matrix<double, 36, 1> q_prev; // used across optimizations over the whole trajectory

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


  // Record information for calculating the cost of dual arm coordination
  Vector3d l_wrist_cur;
  Vector3d r_wrist_cur;
  double l_r_wrist_diff_cost;


} my_constraint_struct;


double linear_map(double x_, double min_, double max_, double min_hat, double max_hat)
{
  return (x_ - min_) / (max_ - min_) * (max_hat - min_hat) + min_hat;
}


double compute_finger_cost(Matrix<double, 12, 1> q_finger_robot, bool left_or_right, my_constraint_struct *fdata)
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


// Used for computing cost and grad(numeric differentiation) in myfunc()
double compute_cost(KDL::ChainFkSolverPos_recursive fk_solver, Matrix<double, 6, 1> q_cur, unsigned int num_wrist_seg, unsigned int num_elbow_seg, unsigned int num_shoulder_seg, bool left_or_right, my_constraint_struct *fdata)
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
    return -1;
  }
  else{
    //ROS_INFO_STREAM("FK solver succeeded for elbow link.");
  }
  result = fk_solver.JntToCart(q_in, wrist_cart_out, num_wrist_seg+1);
  if (result < 0){
    ROS_INFO_STREAM("FK solver failed when processing wrist link, something went wrong");
    return -1;
  }
  else{
    //ROS_INFO_STREAM("FK solver succeeded for wrist link.");
  }
  result = fk_solver.JntToCart(q_in, shoulder_cart_out, num_shoulder_seg+1);
  if (result < 0){
    ROS_INFO_STREAM("FK solver failed when processing shoulder link, something went wrong");
    return -1;
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
  Matrix<double, 36, 1> q_whole = fdata->q_prev; // the whole structure
  Matrix<double, 6, 1> q_prev_arm;
  if (left_or_right) // left arm
  {
    shoulder_pos_human = fdata->l_shoulder_pos_goal;
    elbow_pos_human = fdata->l_elbow_pos_goal;
    wrist_pos_human = fdata->l_wrist_pos_goal;
    wrist_ori_human = fdata->l_wrist_ori_goal;
    q_prev_arm << q_whole[0], q_whole[1], q_whole[2], q_whole[3], q_whole[4], q_whole[5];

    fdata->l_wrist_cur = wrist_pos_cur;

  }
  else // right arm
  {
    shoulder_pos_human = fdata->r_shoulder_pos_goal;
    elbow_pos_human = fdata->r_elbow_pos_goal;
    wrist_pos_human = fdata->r_wrist_pos_goal;
    wrist_ori_human = fdata->r_wrist_ori_goal;
    q_prev_arm << q_whole[6], q_whole[7], q_whole[8], q_whole[9], q_whole[10], q_whole[11];

    fdata->r_wrist_cur = wrist_pos_cur;

  }

  Vector3d shoulder_elbow_vec_human = (elbow_pos_human - shoulder_pos_human).normalized();
  Vector3d shoulder_wrist_vec_human = (wrist_pos_human - shoulder_pos_human).normalized();
  Vector3d elbow_wrist_vec_human = (wrist_pos_human - elbow_pos_human).normalized();

  Vector3d shoulder_elbow_vec_robot = (elbow_pos_cur - shoulder_pos_cur).normalized();
  Vector3d shoulder_wrist_vec_robot = (wrist_pos_cur- shoulder_pos_cur).normalized();
  Vector3d elbow_wrist_vec_robot = (wrist_pos_cur - elbow_pos_cur).normalized();


  // Compute cost function
  double upperarm_direction_cost = (shoulder_elbow_vec_human - shoulder_elbow_vec_robot).norm();
  double forearm_direction_cost = (elbow_wrist_vec_human - elbow_wrist_vec_robot).norm();
  double shoulder_wrist_direction_cost = (shoulder_wrist_vec_robot - shoulder_wrist_vec_human).norm();
  double wrist_ori_cost = std::fabs( std::acos (( (wrist_ori_human * wrist_ori_cur.transpose()).trace() - 1.0) / 2.0));
  double wrist_pos_cost = (wrist_pos_human - wrist_pos_cur).norm();
  double elbow_pos_cost = (elbow_pos_human - elbow_pos_cur).norm();
  double smoothness_cost = 0;

  double l_r_pos_diff_cost = 0;

  double scaled_wrist_pos_cost = (wrist_pos_cur - (wrist_pos_human - elbow_pos_human)/(wrist_pos_human - elbow_pos_human).norm() * (wrist_pos_cur - elbow_pos_cur).norm() - elbow_pos_cur).norm(); // coordinate difference, not in the way of vector difference // written as elbow-wrist form
  //(wrist_pos_cur - (wrist_pos_human - shoulder_pos_human)/(wrist_pos_human - shoulder_pos_human).norm() * (wrist_pos_cur - shoulder_pos_cur).norm() - shoulder_pos_cur).norm(); // written as shoulder-wrist to avoid accumulated error from elbow part
  //

  double scaled_elbow_pos_cost = (elbow_pos_cur - (elbow_pos_human - shoulder_pos_human)/(elbow_pos_human - shoulder_pos_human).norm() * (elbow_pos_cur - shoulder_pos_cur).norm() - shoulder_pos_cur).norm();


  if (!first_iter)
    smoothness_cost = (q_cur - q_prev_arm).norm();    

  if (!left_or_right) // when computing cost of right arm, compute the cost of 
  {
    Vector3d l_r_pos_diff_cur = fdata->l_wrist_cur - fdata->r_wrist_cur;
    Vector3d l_r_pos_diff_human = fdata->l_wrist_pos_goal - fdata->r_wrist_pos_goal;
    l_r_pos_diff_cost = (l_r_pos_diff_cur - l_r_pos_diff_human).norm();
    fdata->l_r_pos_diff_cost = l_r_pos_diff_cost;
  }

  double cost = 0.0 * upperarm_direction_cost
              + 0.0 * forearm_direction_cost
              + 0.0 * shoulder_wrist_direction_cost
              + 1.0 * wrist_ori_cost
              + 0.0 * wrist_pos_cost
              + 0.0 * elbow_pos_cost
              + 1.0 * scaled_wrist_pos_cost
              + 1.0 * scaled_elbow_pos_cost
              + 1.0 * smoothness_cost
              + 1.0 * l_r_pos_diff_cost;

  // record the costs for display
  fdata->upperarm_direction_cost = upperarm_direction_cost;
  fdata->shoulder_wrist_direction_cost = shoulder_wrist_direction_cost;
  fdata->forearm_direction_cost = forearm_direction_cost;
  fdata->wrist_ori_cost = wrist_ori_cost;
  fdata->wrist_pos_cost = wrist_pos_cost;
  fdata->elbow_pos_cost = elbow_pos_cost;
  fdata->scaled_wrist_pos_cost = scaled_wrist_pos_cost;
  fdata->scaled_elbow_pos_cost = scaled_elbow_pos_cost;
  fdata->smoothness_cost = smoothness_cost;
  fdata->total_cost = cost;

  // Display for debug
  /*std::cout << "Cost func structure: " << std::endl
            << "elbow pos err = " << (fdata->elbow_pos_goal - elbow_pos_cur).norm() << std::endl
            << "wrist pos err = " << (fdata->wrist_pos_goal - wrist_pos_cur).norm() << std::endl
            << "wrist ori err = " << std::fabs( std::acos (( (fdata->wrist_ori_goal * wrist_ori_cur.transpose()).trace() - 1) / 2.0)) << std::endl
            << "smoothness err = " << (first_iter? 0 : (q_cur - fdata->q_prev).squaredNorm()) << std::endl;*/
  //std::cout << "Total cost: " << cost << std::endl;
  //std::cout << "During evaluation, q_prev = " << (fdata->q_prev) << std::endl; // checked


  // Return cost function value
  return cost;


}


// This function sets elbow ID and wrist ID in constraint_data, and returns the KDL_FK solver 
KDL::ChainFkSolverPos_recursive setup_left_kdl(my_constraint_struct &constraint_data)
{
  // Params
  const std::string URDF_FILE = "/home/liangyuwei/sign_language_robot_ws/src/ur_description/urdf/ur5_robot_with_hands.urdf";
  const std::string BASE_LINK = "world"; // use /world as base_link for convenience in simulation; when transfer across different robot arms, may use mid-point between shoulders as the common base(or world)
  const std::string SHOULDER_LINK = "left_base_link";
  const std::string ELBOW_LINK = "left_forearm_link";
  const std::string WRIST_LINK = "left_ee_link";

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
  const std::string URDF_FILE = "/home/liangyuwei/sign_language_robot_ws/src/ur_description/urdf/ur5_robot_with_hands.urdf";
  const std::string BASE_LINK = "world"; // use /world as base_link for convenience in simulation; when transfer across different robot arms, may use mid-point between shoulders as the common base(or world)
  const std::string SHOULDER_LINK = "right_base_link";
  const std::string ELBOW_LINK = "right_forearm_link";
  const std::string WRIST_LINK = "right_ee_link";

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
      if (kdl_chain.getSegment(i).getName() == ELBOW_LINK){
        constraint_data.r_num_elbow_seg = i;
        //ROS_INFO_STREAM("Elbow link found.");
        break;
      }
      if (kdl_chain.getSegment(i).getName() == SHOULDER_LINK){
        constraint_data.r_num_shoulder_seg = i;
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


// Loss function
double myfunc(const std::vector<double> &x, std::vector<double> &grad, void *f_data)
{

  // Counter information
  ++count;
  //std::cout << "Evaluation " << count << std::endl;


  // Get additional information by typecasting void* f_data(user-defined data)
  my_constraint_struct *fdata = (my_constraint_struct *) f_data;


  // Get fk solver( and set IDs if first time)
  KDL::ChainFkSolverPos_recursive left_fk_solver = setup_left_kdl(*fdata);
  KDL::ChainFkSolverPos_recursive right_fk_solver = setup_right_kdl(*fdata);

  //std::cout << "At evaluation of cost func, after setting up kdl solver." << std::endl;


  // Calculate loss function(tracking performance + continuity)
  //std::vector<double> xx = x;
  //Matrix<double, 36, 1> x_tmp = Map<Matrix<double, 36, 1>>(xx.data(), 36, 1);
  Matrix<double, 6, 1> q_cur_l, q_cur_r;
  Matrix<double, 12, 1> q_cur_finger_l, q_cur_finger_r;
  q_cur_l << x[0], x[1], x[2], x[3], x[4], x[5]; //Map<Matrix<double, 6, 1>>(x_tmp.data(), 6, 1);
  q_cur_r << x[6], x[7], x[8], x[9], x[10], x[11]; //Map<Matrix<double, 6, 1>>(x_tmp.data(), 6, 1);
  q_cur_finger_l << x[12], x[13], x[14], x[15], x[16], x[17], x[18], x[19], x[20], x[21], x[22], x[23]; //x_tmp.block<12, 1>(12, 0);
  q_cur_finger_r << x[24], x[25], x[26], x[27], x[28], x[29], x[30], x[31], x[32], x[33], x[34], x[35]; //x_tmp.block<12, 1>(24, 0);

  double cost = compute_cost(left_fk_solver, q_cur_l, fdata->l_num_wrist_seg, fdata->l_num_elbow_seg, fdata->l_num_shoulder_seg, true, fdata);
  cost += compute_cost(right_fk_solver, q_cur_r, fdata->r_num_wrist_seg, fdata->r_num_elbow_seg, fdata->r_num_shoulder_seg, false, fdata);
  cost += compute_finger_cost(q_cur_finger_l, true, fdata);
  cost += compute_finger_cost(q_cur_finger_r, false, fdata);


  // Compute gradient using Numeric Differentiation
  // only compute gradient if not NULL
  if(!grad.empty())
  {
    double eps = 0.001;
    Matrix<double, 6, 1> q_tmp_l, q_tmp_r;
    Matrix<double, 12, 1> q_tmp_finger_l, q_tmp_finger_r;    

    double cost1, cost2;
    // gradients on the left arm's joints
    for (unsigned int i = 0; i < q_tmp_l.size(); ++i)
    {
      // 1
      q_tmp_l = q_cur_l;
      q_tmp_r = q_cur_r;
      q_tmp_l[i] += eps;
      cost1 = compute_cost(left_fk_solver, q_tmp_l, fdata->l_num_wrist_seg, fdata->l_num_elbow_seg, fdata->l_num_shoulder_seg, true, fdata);
      cost1 += compute_cost(right_fk_solver, q_tmp_r, fdata->r_num_wrist_seg, fdata->r_num_elbow_seg, fdata->r_num_shoulder_seg, false, fdata);
      cost1 += compute_finger_cost(q_cur_finger_l, true, fdata);
      cost1 += compute_finger_cost(q_cur_finger_r, false, fdata);
      // 2
      q_tmp_l = q_cur_l;
      q_tmp_r = q_cur_r;
      q_tmp_l[i] -= eps;
      cost2 = compute_cost(left_fk_solver, q_tmp_l, fdata->l_num_wrist_seg, fdata->l_num_elbow_seg, fdata->l_num_shoulder_seg, true, fdata);
      cost2 += compute_cost(right_fk_solver, q_tmp_r, fdata->r_num_wrist_seg, fdata->r_num_elbow_seg, fdata->r_num_shoulder_seg, false, fdata);
      cost2 += compute_finger_cost(q_cur_finger_l, true, fdata);
      cost2 += compute_finger_cost(q_cur_finger_r, false, fdata);

      // combine 1 and 2
      grad[i] = (cost1 - cost2) / (2.0 * eps);
    }

    // gradients on the right arm's joints
    for (unsigned int i = q_tmp_l.size(); i < q_tmp_l.size() + q_tmp_r.size(); ++i)
    {
      // 1
      q_tmp_l = q_cur_l;
      q_tmp_r = q_cur_r;
      q_tmp_r[i-q_tmp_l.size()] += eps;
      cost1 = compute_cost(left_fk_solver, q_tmp_l, fdata->l_num_wrist_seg, fdata->l_num_elbow_seg, fdata->l_num_shoulder_seg, true, fdata);
      cost1 += compute_cost(right_fk_solver, q_tmp_r, fdata->r_num_wrist_seg, fdata->r_num_elbow_seg, fdata->r_num_shoulder_seg, false, fdata);
      cost1 += compute_finger_cost(q_cur_finger_l, true, fdata);
      cost1 += compute_finger_cost(q_cur_finger_r, false, fdata);
      // 2
      q_tmp_l = q_cur_l;
      q_tmp_r = q_cur_r;
      q_tmp_r[i-q_tmp_l.size()] -= eps;
      cost2 = compute_cost(left_fk_solver, q_tmp_l, fdata->l_num_wrist_seg, fdata->l_num_elbow_seg, fdata->l_num_shoulder_seg, true, fdata);
      cost2 += compute_cost(right_fk_solver, q_tmp_r, fdata->r_num_wrist_seg, fdata->r_num_elbow_seg, fdata->r_num_shoulder_seg, false, fdata);
      cost2 += compute_finger_cost(q_cur_finger_l, true, fdata);
      cost2 += compute_finger_cost(q_cur_finger_r, false, fdata);
      // combine 1 and 2
      grad[i] = (cost1 - cost2) / (2.0 * eps);
    }

    // gradients on the left hand's joints
    for (unsigned int i = q_tmp_l.size() + q_tmp_r.size(); i < q_tmp_l.size() + q_tmp_r.size() + q_tmp_finger_l.size(); ++i)
    {
      // 1
      q_tmp_l = q_cur_l;
      q_tmp_r = q_cur_r;
      q_tmp_finger_l = q_cur_finger_l;
      q_tmp_finger_r = q_cur_finger_r;

      q_tmp_finger_l[i-q_tmp_l.size()-q_tmp_r.size()] += eps;

      cost1 = compute_cost(left_fk_solver, q_tmp_l, fdata->l_num_wrist_seg, fdata->l_num_elbow_seg, fdata->l_num_shoulder_seg, true, fdata);
      cost1 += compute_cost(right_fk_solver, q_tmp_r, fdata->r_num_wrist_seg, fdata->r_num_elbow_seg, fdata->r_num_shoulder_seg, false, fdata);
      cost1 += compute_finger_cost(q_tmp_finger_l, true, fdata);
      cost1 += compute_finger_cost(q_tmp_finger_r, false, fdata);
      // 2
      q_tmp_l = q_cur_l;
      q_tmp_r = q_cur_r;
      q_tmp_finger_l = q_cur_finger_l;
      q_tmp_finger_r = q_cur_finger_r;

      q_tmp_finger_l[i-q_tmp_l.size()-q_tmp_r.size()] -= eps;

      cost2 = compute_cost(left_fk_solver, q_tmp_l, fdata->l_num_wrist_seg, fdata->l_num_elbow_seg, fdata->l_num_shoulder_seg, true, fdata);
      cost2 += compute_cost(right_fk_solver, q_tmp_r, fdata->r_num_wrist_seg, fdata->r_num_elbow_seg, fdata->r_num_shoulder_seg, false, fdata);
      cost2 += compute_finger_cost(q_tmp_finger_l, true, fdata);
      cost2 += compute_finger_cost(q_tmp_finger_r, false, fdata);
      // combine 1 and 2
      grad[i] = (cost1 - cost2) / (2.0 * eps);
    }    


    // gradients on the right hand's joints
    for (unsigned int i = q_tmp_l.size() + q_tmp_r.size() + q_tmp_finger_l.size(); i < q_tmp_l.size() + q_tmp_r.size() + q_tmp_finger_l.size() + q_tmp_finger_r.size(); ++i)
    {
      // 1
      q_tmp_l = q_cur_l;
      q_tmp_r = q_cur_r;
      q_tmp_finger_l = q_cur_finger_l;
      q_tmp_finger_r = q_cur_finger_r;

      q_tmp_finger_r[i-q_tmp_l.size()-q_tmp_r.size()-q_tmp_finger_l.size()] += eps;

      cost1 = compute_cost(left_fk_solver, q_tmp_l, fdata->l_num_wrist_seg, fdata->l_num_elbow_seg, fdata->l_num_shoulder_seg, true, fdata);
      cost1 += compute_cost(right_fk_solver, q_tmp_r, fdata->r_num_wrist_seg, fdata->r_num_elbow_seg, fdata->r_num_shoulder_seg, false, fdata);
      cost1 += compute_finger_cost(q_tmp_finger_l, true, fdata);
      cost1 += compute_finger_cost(q_tmp_finger_r, false, fdata);
      // 2
      q_tmp_l = q_cur_l;
      q_tmp_r = q_cur_r;
      q_tmp_finger_l = q_cur_finger_l;
      q_tmp_finger_r = q_cur_finger_r;

      q_tmp_finger_r[i-q_tmp_l.size()-q_tmp_r.size()-q_tmp_finger_l.size()] -= eps;

      cost2 = compute_cost(left_fk_solver, q_tmp_l, fdata->l_num_wrist_seg, fdata->l_num_elbow_seg, fdata->l_num_shoulder_seg, true, fdata);
      cost2 += compute_cost(right_fk_solver, q_tmp_r, fdata->r_num_wrist_seg, fdata->r_num_elbow_seg, fdata->r_num_shoulder_seg, false, fdata);
      cost2 += compute_finger_cost(q_tmp_finger_l, true, fdata);
      cost2 += compute_finger_cost(q_tmp_finger_r, false, fdata);
      // combine 1 and 2
      grad[i] = (cost1 - cost2) / (2.0 * eps);
    }    

  }

  // Return cost function value
  return cost;

}



// Constraint function; expected to be myconstraint(x)<=0
//double myconstraint(const std::vector<double> &x, std::vector<double> &grad, void *f_data)
void myconstraint(unsigned m, double *result, unsigned n, const double *x,
                              double *grad, /* NULL if not needed */
                              void *f_data)
{
  // Constraints: relative position of wrists, collision avoidance

  
  // Meta-information
  my_constraint_struct *fdata = (my_constraint_struct *) f_data;

  // Get angles
  //Matrix<double, 36, 1> q_in = Map<Vector3d>(x, 3, 1);
  //std::vector<double> q_in << x[0], x[1], x[2]
  //std::vector<double> q_left_arm = 
  //std::vector<double> q_right_arm = 
  // std::vector<double> q_left_hand = 
  // std::vector<double> q_right_hand = 

  /** Constraint 1: collision avoidance **/
  // collision inside one group

  double col_r_hand_r_hand_cost = 0;
  double col_l_hand_l_hand_cost = 0;
  double col_r_arm_r_arm_cost = 0;
  double col_l_arm_l_arm_cost = 0;

  // collision between different groups
  double col_r_hand_l_hand = 0;
  double col_r_hand_l_arm = 0;
  double col_l_hand_r_arm = 0;
  double col_l_arm_r_arm = 0;

  // compute minimum distance (and penetration depth)
    

   // Get fk solver( and set IDs if first time)
  //KDL::ChainFkSolverPos_recursive left_fk_solver = setup_left_kdl(*fdata);
  //KDL::ChainFkSolverPos_recursive right_fk_solver = setup_right_kdl(*fdata);


  // Calculate loss function(tracking performance + continuity)
  //std::vector<double> x_tmp = x;
  Matrix<double, 6, 1> q_cur_l, q_cur_r;
  q_cur_l << x[0], x[1], x[2], x[3], x[4], x[5]; //Map<Matrix<double, 6, 1>>(x_tmp.data(), 6, 1);
  q_cur_r << x[6], x[7], x[8], x[9], x[10], x[11]; //Map<Matrix<double, 6, 1>>(x_tmp.data(), 6, 1);
  //double cost = compute_cost(left_fk_solver, q_cur_l, fdata->l_num_wrist_seg, fdata->l_num_elbow_seg, fdata->l_num_shoulder_seg, true, fdata);
  //cost += compute_cost(right_fk_solver, q_cur_r, fdata->r_num_wrist_seg, fdata->r_num_elbow_seg, fdata->r_num_shoulder_seg, false, fdata);



  /** Constraint 2: relative wrsit position **/



  // Compute gradients of constraint functions(if non-NULL, it points to an array with the size of m*n; access through)
  if (grad){

    for (unsigned i = 0; i < m; ++i) // number of constraints
    {
      for(unsigned j = 0; j < n; ++j)
      {

        grad[i * n + j] = 0;

      }

    }

  }


  // Compute constraints and store in `result`
  for (unsigned int i = 0; i < m; ++i)
  {
    result[i] = 0;
  }



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



int main(int argc, char *argv[])
{
  
  // Specify required names
  std::string in_file_name = "test_imi_data_UR5.h5";
  std::string in_group_name = "fengren";
  std::string out_file_name = "mocap_ik_results.h5";
  

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
  std::cout << "The output h5 file name is: " << in_group_name << std::endl;


  // Input Cartesian trajectories
  const unsigned int joint_value_dim = 2*6 + 2*12; // dual arm + dual hand = 36 DOF
  std::vector<double> x(joint_value_dim);

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

  // using read_h5() does not need to specify the size!!!
  // elbow pos(3) + wrist pos(3) + wrist rot(9) = 15-dim
  unsigned int num_datapoints = read_l_wrist_pos_traj.size(); 

  // display a few examples(debug)
  /*
  std::cout << "Display the read fake path data: " << std::endl;
  for (int i = 0; i < num_datapoints; ++i)
  {
    for (int j = 0; j < read_wrist_elbow_traj[i].size(); ++j)
      std::cout << read_wrist_elbow_traj[i][j] << " ";
    std::cout << std::endl;
  }
  exit(0);
  */


  // Parameters setting
  const std::vector<double> q_l_arm_lb = {-1.0, -1.0, -1.0, -1.57, -1.57, -1.57};
  const std::vector<double> q_r_arm_lb = {-1.0, -1.0, -1.0, -1.57, -1.57, -1.57};
  const std::vector<double> q_l_arm_ub = {1.0, 1.0, 1.0, 1.57, 1.57, 1.57};
  const std::vector<double> q_r_arm_ub = {1.0, 1.0, 1.0, 1.57, 1.57, 1.57};
  
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
  // variable structure: l_arm, r_arm, l_finger, r_finger  



  //std::vector<double> x(joint_value_dim);
  double minf;
  double tol = 1e-4;
  double stopval = 1e-8;


  // Set up KDL(get solver, wrist ID and elbow ID)
  my_constraint_struct constraint_data; 
  setup_left_kdl(constraint_data); // set IDs, discard the solver handle
  setup_right_kdl(constraint_data); 

  // robot's shoulder position
  constraint_data.l_robot_shoulder_pos = Vector3d(-0.06, 0.235, 0.395);
  constraint_data.r_robot_shoulder_pos = Vector3d(-0.06, -0.235, 0.395);
  constraint_data.l_robot_finger_start << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3, 0.4, 0.0, 0.0; 
  constraint_data.l_robot_finger_final << -1.6, -1.7, -1.6, -1.7, -1.6, -1.7, -1.6, -1.7, -1.0, 0.0, -0.4, -1.0;
  constraint_data.r_robot_finger_start << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3, 0.4, 0.0, 0.0;
  constraint_data.r_robot_finger_final << -1.6, -1.7, -1.6, -1.7, -1.6, -1.7, -1.6, -1.7, -1.0, 0.0, -0.4, -1.0; // right and left hands' joint ranges are manually set to be the same, but according to Inspire Hand Inc, this will keep changing in the future.
  constraint_data.glove_start << 0, 0, 53, 0, 0, 22, 0, 0, 22, 0, 0, 35, 0, 0;
  constraint_data.glove_start = constraint_data.glove_start * M_PI / 180.0; // in radius  
  constraint_data.glove_final << 45, 100, 0, 90, 120, 0, 90, 120, 0, 90, 120, 0, 90, 120;
  constraint_data.glove_final = constraint_data.glove_final * M_PI / 180.0; 



  //std::cout << "At Main func, before set up optimizer." << std::endl;

  // Set up optimizer
  /*try
  {*/
  nlopt::opt opt(nlopt::LD_SLSQP, joint_value_dim); // nlopt::LD_SLSQP
  /*}
  catch(std::bad_alloc e)
  {
    std::cout << "Something went wrong in the constructor: " << e.what() << std::endl;
    return -1;
  }*/

  opt.set_lower_bounds(qlb); // set lower bounds
  opt.set_upper_bounds(qub); // set upper bounds
  opt.set_stopval(1e-6); // stop value
  opt.set_ftol_rel(1e-8); // objective function value changes by less than `tol` multiplied by the absolute value of the function value
  //opt.set_ftol_abs(1e-12); // objective function value changes by less than `tol`
  opt.set_xtol_rel(1e-6); // optimization parameters' magnitude changes by less than `tol` multiplied by the current magnitude(can set weights for each dimension)
  //opt.set_xtol_abs(1e-8); // optimization parameters' magnitude changes by less than `tol`
  opt.set_maxeval(300); // maximum evaluation
  //opt.set_maxtime(3.0); // maximum time


  // Start iterations
  std::vector<std::vector<double> > q_results(num_datapoints, std::vector<double>(joint_value_dim));
  for (unsigned int it = 0; it < num_datapoints; ++it)
  {

    // Reset counter.
    count = 0; 

    // Get one point from the path
    //std::vector<double> path_point = read_wrist_elbow_traj[it];
    /*std::vector<double> wrist_pos(path_point.begin(), path_point.begin()+3); // 3-dim
    std::vector<double> wrist_ori(path_point.begin()+3, path_point.begin()+12); // 9-dim
    std::vector<double> elbow_pos(path_point.begin()+12, path_point.begin()+15); //end()); // 3-dim */
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

    /** check the extracted data sizes **
    std::cout << "wrist_pos.size() = " << wrist_pos.size() << ", ";
    std::cout << "wrist_ori.size() = " << wrist_ori.size() << ", ";
    std::cout << "elbow_pos.size() = " << elbow_pos.size() << "." << std::endl; */
    // convert
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




    /** Be careful with the data assignment above !!!! **
    //if (it == 10){
    std::cout << "Display the goal point: " << std::endl;
    std::cout << "Path point is: ";
    for (int i = 0; i < wrist_pos.size() + wrist_ori.size() + elbow_pos.size(); ++i) std::cout << path_point[i] << " "; 
    std::cout << std::endl << "Wrist pos is: " << constraint_data.wrist_pos_goal << std::endl
                           << "Wrist rot is: " << constraint_data.wrist_ori_goal << std::endl
                           << "Elbow pos is: " << constraint_data.elbow_pos_goal << std::endl;
    exit(0);
    //} */
    /*std::cout << "q_prev is: " << constraint_data.q_prev.transpose() << std::endl;
    if (it == 6)
      exit(0);*/


    // Set up objective function and additional data to pass in
    my_constraint_struct *f_data = &constraint_data;
    opt.set_min_objective(myfunc, (void *) f_data); // set objective function to minimize; with no additional information passed(f_data)

    // Set constraints
    const std::vector<double> tol_vec = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // tol_vec.size() determines the dimensionality m 
    //opt.add_inequality_mconstraint(myconstraint, (void *) f_data, tol_vec);

    // Start optimization
    /*try
    {*/

      // Display messages
      std::cout << "========== Path point " << it + 1  <<"/" << num_datapoints << " ==========" << std::endl;
      nlopt::result opt_result;
      //std::cout << "first_iter is " << first_iter << " before optimizing the first point." << std::endl; // ok, checked
      if (first_iter)
        opt_result = opt.optimize(x, minf);
      else
      {
        //std::cout << "previous result is: " << constraint_data.q_prev << "." << std::endl; // not changing!!
        /*std::cout << "previous result is: ";
        for (int t = 0; t < q_results[it-1].size(); ++t)  std::cout << q_results[it-1][t] << " ";
        std::cout << std::endl;*/
        x = q_results[it-1]; // pass the previous result into x, otherwise the previous result would be modified again!!!(passed as reference into opt.optimize())
        opt_result = opt.optimize(x, minf); // use previous result as initial guess
      }

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
      std::cout << "Upperarm Direction Cost: " << f_data->upperarm_direction_cost << std::endl;
      std::cout << "Shoulder-Wrist Direction Cost: " << f_data->shoulder_wrist_direction_cost << std::endl;
      std::cout << "Forearm Direction Cost: " << f_data->forearm_direction_cost << std::endl;
      std::cout << "Wrist Orientation Cost: " << f_data->wrist_ori_cost << std::endl;
      std::cout << "Wrist Position Cost: " << f_data->wrist_pos_cost << std::endl;
      std::cout << "Elbow Position Cost: " << f_data->elbow_pos_cost << std::endl;
      std::cout << "Scaled Wrist Pos Cost: " << f_data->scaled_wrist_pos_cost << std::endl;
      std::cout << "Scaled Elbow Pos Cost: " << f_data->scaled_elbow_pos_cost << std::endl;
      std::cout << "Smoothness Cost: " << f_data->smoothness_cost << std::endl;
      std::cout << "Two arm's wrist positions difference Cost: " << f_data->l_r_pos_diff_cost << std::endl;
      std::cout << "Left finger scaled pos Cost: " << f_data->scaled_l_finger_pos_cost << std::endl;
      std::cout << "Right finger scaled pos Cost: " << f_data->scaled_r_finger_pos_cost << std::endl;
      std::cout << "Total Cost: " << f_data->total_cost << std::endl;

      // Store the result(joint values)
      q_results[it] = x;
      //std::vector<double> q_tmp(x); // copy construct
      //q_results.push_back(q_tmp);

      // display the current result
      /*std::cout << "q result is: ";
      for (int t = 0; t < x.size(); ++t)  std::cout << x[t] << " ";
      std::cout << std::endl;*/

      // Record the current joint as q_prev
      //Matrix<double, 6, 1> q_prev = Map<Eigen::Matrix<double, 6, 1>>(x.data(), 6, 1);
      constraint_data.q_prev = Map<Eigen::Matrix<double, joint_value_dim, 1>>(x.data(), joint_value_dim, 1); // used across optimizations over the whole trajectory  
      //std::cout << "q_prev is: " << constraint_data.q_prev.transpose() << std::endl;
      first_iter = false;

    /*}
    catch (std::runtime_error e1){
      std::cout << "Runtime error: " << e1.what() << std::endl;
    }
    catch (std::invalid_argument e2){
      std::cout << "Invalid argument: " << e2.what() << std::endl;    
    }
    catch (std::bad_alloc e3){
      std::cout << "Ran out of memory: " << e3.what() << std::endl;    
    }*/

    /*catch (nlopt::roundoff_limited e4){ // will be caught earlier handler for 'std::runtime_error e1'
      std::cout << "Roundoff errors limited progress: " << e4.what() << std::endl;    
    }
    catch (nlopt::forced_stop e5){ // will be caught earlier handler for 'std::runtime_error
      std::cout << "Forced termination: " << e5.what() << std::endl;    
    }*/

    //std::cout << "first_iter is " << first_iter << " after optimizing the first point." << std::endl;
    //exit(0);
  }


  // display the results
  /*
  std::cout << "q_results is: " << std::endl;
  for (int i = 0; i < num_datapoints; ++i)
  {
    for (int j = 0; j < joint_value_dim; ++j)
      std::cout << q_results[i][j] << " ";
    std::cout << std::endl;
  }*/


  // Store the results
  const std::string group_name = in_group_name;
  //const std::string dataset_name = "arm_traj_1";
  bool result1 = write_h5(out_file_name, group_name, "arm_traj_1", num_datapoints, joint_value_dim, q_results);
  bool result2 = write_h5(out_file_name, group_name, "timestamp_1", num_datapoints, 1, read_time_stamps);  

  if(result1 && result2)
    std::cout << "Joint path results successfully stored!" << std::endl;

 
  return 0;

}



