#include "ros/ros.h"
#include <vector>
#include <string>

#include "dual_ur5_control/JntToCart.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "geometry_msgs/Pose.h"


// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// Transform
#include <Eigen/Core>
#include <Eigen/Geometry>


bool joint_to_cartesian(dual_ur5_control::JntToCart::Request &req, dual_ur5_control::JntToCart::Response &res)
{


  /** Obtain the request **/
  ROS_INFO("Obtain the request joint trajectory.");
  std::vector<trajectory_msgs::JointTrajectoryPoint> joint_trajectory = req.joint_trajectory;
  std::string group_name, eef_name;
  if(req.left_or_right)
  {
    group_name = "left_arm";
    eef_name = "left_ee_link";
  }
  else
  {
    group_name = "right_arm";
    eef_name = "right_ee_link";
  }


  /** Preparation **/
  // Construct RobotModelLoder
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  // Construct RobotModel
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  //ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
  // Construct RobotState
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  // set all joints to default positions
  kinematic_state->setToDefaultValues();  
  // get JointModelGroup
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(group_name);
  // get joint names
  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();  


  /** Get the pos information and apply FK **/
  std::vector<double> joint_values;
  std::vector<geometry_msgs::Pose> cart_trajectory(joint_trajectory.size());
  int count = 0;
  auto it_cart = cart_trajectory.begin();
  for (auto it = joint_trajectory.cbegin(); it != joint_trajectory.cend(); ++it)
  {
    // Message
    ROS_INFO("Processing point: %d / %d", count+1, (int) joint_trajectory.size());
    count++;
    // Get joint values
    joint_values = it->positions;
    // Forward Kinematics
    kinematic_state->setVariablePositions(joint_names, joint_values); //
    const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform(eef_name);
    // Assign pose
    it_cart->position.x = end_effector_state.translation()(0);
    it_cart->position.y = end_effector_state.translation()(1);
    it_cart->position.z = end_effector_state.translation()(2);
    Eigen::Matrix3d R = end_effector_state.rotation();
    Eigen::Quaterniond quat(R);
    it_cart->orientation.x = quat.coeffs()(0);
    it_cart->orientation.y = quat.coeffs()(1);
    it_cart->orientation.z = quat.coeffs()(2);
    it_cart->orientation.w = quat.coeffs()(3);
    // Switch to next
    ++it_cart;
  }

  /** Set the response **/
  res.cart_trajectory = cart_trajectory;

  ROS_INFO("Done.");
  return true;


}

int main(int argc, char** argv)
{
  // Initialize a node
  ros::init(argc, argv, "apply_fk_server");
  ros::NodeHandle n;


  // Start the server
  ros::ServiceServer service = n.advertiseService("apply_fk_server", joint_to_cartesian);
  ROS_INFO("Ready to take in joint trajectories...");
  ros::spin();
  

  return 0;

}








