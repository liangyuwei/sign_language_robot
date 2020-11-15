#ifndef YUMI_TRAC_IK_SOLVER_H_
#define YUMI_TRAC_IK_SOLVER_H_

// For Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

// For KDL & trac-ik
#include "ros/ros.h"
#include <trac_ik/trac_ik.hpp>

#include <kdl/kdl.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>

// My header files
#include "config.h"

using namespace std;
using namespace KDL;
using namespace Eigen;
using namespace cfg;

const static string URDF_FILE = "/home/liangyuwei/sign_language_robot_ws/src/yumi_description/urdf/yumi_with_hands.urdf";
const static string BASE_LINK = "world";
const static string LEFT_ELBOW_LINK = "yumi_link_4_l";
const static string RIGHT_ELBOW_LINK = "yumi_link_4_r";
const static string LEFT_WRIST_LINK = "yumi_link_7_l";
const static string RIGHT_WRIST_LINK = "yumi_link_7_r";

class yumi_trac_ik_solver
{
    public:
        yumi_trac_ik_solver();
        int run_trac_ik_left(
                Matrix<double, 7, 1> q_initial, Matrix<double, 7, 1> &q_result, Vector3d pos_goal, Matrix3d ori_goal,
                Matrix<double, 7, 1> lower_joint_limits, Matrix<double, 7, 1> upper_joint_limits, 
                double timeout);
        int run_trac_ik_right(
                Matrix<double, 7, 1> q_initial, Matrix<double, 7, 1> &q_result, Vector3d pos_goal, Matrix3d ori_goal,
                Matrix<double, 7, 1> lower_joint_limits, Matrix<double, 7, 1> upper_joint_limits, 
                double timeout);
    private:
        void setup_kdl();
        KDL::Chain left_kdl_chain;
        KDL::Chain right_kdl_chain;
        KDL::ChainFkSolverPos_recursive* left_kdl_solver = nullptr;
        KDL::ChainFkSolverPos_recursive* right_kdl_solver = nullptr;
};

yumi_trac_ik_solver::yumi_trac_ik_solver()
{
    setup_kdl();
}

void yumi_trac_ik_solver::setup_kdl()
{

  // Get tree
  KDL::Tree kdl_tree; 
  if (!kdl_parser::treeFromFile(URDF_FILE, kdl_tree)){ 
      ROS_ERROR("Failed to construct kdl tree");
      exit(-1);
  }

  // Get chain  
  if(!kdl_tree.getChain(BASE_LINK, LEFT_WRIST_LINK, left_kdl_chain)){
    ROS_INFO("Failed to obtain chain from root to right wrist");
    exit(-1);
  }

  if(!kdl_tree.getChain(BASE_LINK, RIGHT_WRIST_LINK, right_kdl_chain)){
    ROS_INFO("Failed to obtain chain from root to right wrist");
    exit(-1);
  }

  // Set up FK solver
  left_kdl_solver = new KDL::ChainFkSolverPos_recursive(left_kdl_chain);
  right_kdl_solver = new KDL::ChainFkSolverPos_recursive(right_kdl_chain);

}

/**
 * @brief Compute IK using TRAC-IK for the left arm
 * 
 * Perform IK on wrist position and orientation trajectories for YuMi 7-dof robot
 * If returned int is >= 0, then successful, otherwise failed !
 */
int yumi_trac_ik_solver::run_trac_ik_left(
                Matrix<double, 7, 1> q_initial, Matrix<double, 7, 1> &q_result, Vector3d pos_goal, Matrix3d ori_goal,
                Matrix<double, 7, 1> lower_joint_limits, Matrix<double, 7, 1> upper_joint_limits, 
                double timeout)
{
  double eps = 1e-5;

  // Get joint angles and joint limits
  KDL::JntArray q_in(q_initial.size()); 
  for (unsigned int i = 0; i < q_initial.size(); ++i)
  {
    q_in(i) = q_initial(i);
  }

  KDL::JntArray lb(lower_joint_limits.size()); 
  for (unsigned int i = 0; i < lower_joint_limits.size(); ++i)
  {
    lb(i) = lower_joint_limits(i);
  }

  KDL::JntArray ub(upper_joint_limits.size()); 
  for (unsigned int i = 0; i < upper_joint_limits.size(); ++i)
  {
    ub(i) = upper_joint_limits(i);
  }  

  // Get end effector goal
  KDL::Vector pos(pos_goal[0], pos_goal[1], pos_goal[2]);
  KDL::Vector rot_col_x(ori_goal(0, 0), ori_goal(1, 0), ori_goal(2, 0)); 
  KDL::Vector rot_col_y(ori_goal(0, 1), ori_goal(1, 1), ori_goal(2, 1)); 
  KDL::Vector rot_col_z(ori_goal(0, 2), ori_goal(1, 2), ori_goal(2, 2)); 
  KDL::Rotation rot(rot_col_x, rot_col_y, rot_col_z);
  // KDL::Rotation contains [X, Y, Z] three columns, i.e. [rot_col_x, rot_col_y, rot_col_z]
  KDL::Frame end_effector_pose(rot, pos);

  // Construct a TRAC-IK solver
  // last argument is optional
  // Speed: returns very quickly the first solution found
  // Distance: runs for the full timeout_in_secs, then returns the solution that minimizes SSE from the seed
  // Manip1: runs for full timeout, returns solution that maximizes sqrt(det(J*J^T))
  // Manip2: runs for full timeout, returns solution that minimizes cond(J) = |J|*|J^-1|
  TRAC_IK::TRAC_IK ik_solver(left_kdl_chain, lb, ub, timeout, eps, TRAC_IK::Distance);//TRAC_IK::Speed);
  KDL::JntArray q_res;
  int rc = ik_solver.CartToJnt(q_in, end_effector_pose, q_res); //, KDL::Twist tolerances); // optional tolerances, see trac_ik_lib documentation for usage


  // Return the result
  if (rc >= 0) // if successful
  {
    for (unsigned int i = 0; i < q_result.size(); ++i)
    {
      q_result(i) = q_res(i);
    }
  }
  else // if fail, return the initial values
  {
    q_result = q_initial;
  }
  // return q_res_vec;

  return rc;
}

/**
 * @brief Compute IK using TRAC-IK for the right arm
 * 
 * Perform IK on wrist position and orientation trajectories for YuMi 7-dof robot
 * If returned int is >= 0, then successful, otherwise failed !
 */
int yumi_trac_ik_solver::run_trac_ik_right(
                Matrix<double, 7, 1> q_initial, Matrix<double, 7, 1> &q_result, Vector3d pos_goal, Matrix3d ori_goal,
                Matrix<double, 7, 1> lower_joint_limits, Matrix<double, 7, 1> upper_joint_limits, 
                double timeout)
{
  double eps = 1e-5;

  // Get joint angles and joint limits
  KDL::JntArray q_in(q_initial.size()); 
  for (unsigned int i = 0; i < q_initial.size(); ++i)
  {
    q_in(i) = q_initial(i);
  }

  KDL::JntArray lb(lower_joint_limits.size()); 
  for (unsigned int i = 0; i < lower_joint_limits.size(); ++i)
  {
    lb(i) = lower_joint_limits(i);
  }

  KDL::JntArray ub(upper_joint_limits.size()); 
  for (unsigned int i = 0; i < upper_joint_limits.size(); ++i)
  {
    ub(i) = upper_joint_limits(i);
  }  

  // Get end effector goal
  KDL::Vector pos(pos_goal[0], pos_goal[1], pos_goal[2]);
  KDL::Vector rot_col_x(ori_goal(0, 0), ori_goal(1, 0), ori_goal(2, 0)); 
  KDL::Vector rot_col_y(ori_goal(0, 1), ori_goal(1, 1), ori_goal(2, 1)); 
  KDL::Vector rot_col_z(ori_goal(0, 2), ori_goal(1, 2), ori_goal(2, 2)); 
  KDL::Rotation rot(rot_col_x, rot_col_y, rot_col_z);
  // KDL::Rotation contains [X, Y, Z] three columns, i.e. [rot_col_x, rot_col_y, rot_col_z]
  KDL::Frame end_effector_pose(rot, pos);

  // Construct a TRAC-IK solver
  // last argument is optional
  // Speed: returns very quickly the first solution found
  // Distance: runs for the full timeout_in_secs, then returns the solution that minimizes SSE from the seed
  // Manip1: runs for full timeout, returns solution that maximizes sqrt(det(J*J^T))
  // Manip2: runs for full timeout, returns solution that minimizes cond(J) = |J|*|J^-1|
  TRAC_IK::TRAC_IK ik_solver(right_kdl_chain, lb, ub, timeout, eps, TRAC_IK::Distance);//TRAC_IK::Speed);
  KDL::JntArray q_res;
  int rc = ik_solver.CartToJnt(q_in, end_effector_pose, q_res); //, KDL::Twist tolerances); // optional tolerances, see trac_ik_lib documentation for usage


  // Return the result
  if (rc >= 0) // if successful
  {
    for (unsigned int i = 0; i < q_result.size(); ++i)
    {
      q_result(i) = q_res(i);
    }
  }
  else // if fail, return the initial values
  {
    q_result = q_initial;
  }
  // return q_res_vec;

  return rc;
}


#endif