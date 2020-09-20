cmake_minimum_required(VERSION 2.8.3)
project(human_model)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  std_msgs
  urdf
  controller_manager
  joint_state_controller
  robot_state_publisher
  gazebo_ros_control
)

