#include <ros/ros.h>


// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// Transform
#include <Eigen/Core>
#include <Eigen/Geometry>


int main(int argc, char** argv)
{
  // Initialize a node
  ros::init(argc, argv, "get_fk");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ROS_INFO("Start testing...");

  // RobotModelLoder for looking up robot description on the ROS parameter server
  // Construct a RobotModel for use
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  // Construct a RobotState that maintains the configuration of the robot
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();  // set all joints to default positions
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("left_arm");


  // Get joint names and joint values
  std::vector<double> joint_values;
  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();  
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }


  // Forward Kinematics
  joint_values = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
  kinematic_state->setVariablePositions(joint_names, joint_values); //
  const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform("left_ee_link");
  ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
  ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");

  // Transform
  Eigen::Matrix3d R = end_effector_state.rotation();
  Eigen::Quaterniond quat(R);
  ROS_INFO_STREAM("Quaternion: \n" << quat.coeffs().transpose() << "\n");
 

  // Shutdown everything
  ros::shutdown();
  return 0;



}








