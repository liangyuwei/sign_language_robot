/*
#include <ros/ros.h>

#include <vector>
#include <string>
#include <fstream>
#include <algorithm>
#include <chrono>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/rdf_loader/rdf_loader.h>
#include <moveit/planning_scene/planning_scene.h>

// MoveIt msg and srv for using planning_scene_diff
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/ApplyPlanningScene.h>

// For collision checking, distance computation
#include <moveit/collision_detection_fcl/collision_robot_fcl.h>
#include <moveit/collision_detection_fcl/collision_world_fcl.h>
#include <moveit/collision_detection/collision_robot.h>

// For PlanningSceneInterface
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// Eigen
#include <Eigen/Eigen>
*/

// Itself
#include "collision_checking_yumi.h"


DualArmDualHandCollision::DualArmDualHandCollision(std::string urdf_string, std::string srdf_string) : options_(urdf_string, srdf_string), robot_model_loader_(options_), planning_scene_(kinematic_model_)
{
  //this->collision_request_.contacts = false;//true;
  //this->collision_request_.max_contacts = 1000; 
}



double DualArmDualHandCollision::check_collision(const std::vector<double> q_in)
{
  /* This function computes the minimum distance between links inside one group or of different groups */

  // Get the joint angles for each group
  // Remember to change the joint value assignment here when switching to robots with different DOFs!!!
  std::vector<double> q_left_arm(q_in.begin(), q_in.begin()+7); //(6); 
  std::vector<double> q_right_arm(q_in.begin()+7, q_in.begin()+14); //(6);
  std::vector<double> q_left_finger(q_in.begin()+14, q_in.begin()+26); //(12); 
  std::vector<double> q_right_finger(q_in.begin()+26, q_in.begin()+38); //(12);

  // Set current state (joint positions)
  this->current_state_.setJointGroupPositions(this->left_arm_group_, q_left_arm);
  this->current_state_.setJointGroupPositions(this->right_arm_group_, q_right_arm);
  this->current_state_.setJointGroupPositions(this->left_hand_group_, q_left_finger);
  this->current_state_.setJointGroupPositions(this->right_hand_group_, q_right_finger);
  this->current_state_.update();

  // Check if a robot is in collision with itself
  this->planning_scene_.checkSelfCollision(this->collision_request_, this->collision_result_);
  //std::cout << "The robot is " << (collision_result.collision ? "in" : "not in") << " collision with other parts." << std::endl;

  double result = (this->collision_result_.collision ? 1 : -1); // 1 for colliding, -1 for free

  this->collision_result_.clear(); // should clear the results, in case the result object is used again!!!

  return result; // set to 1 if in collision
  
}


