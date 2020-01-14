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
#include "collision_checking.h"


DualArmDualHandCollision::DualArmDualHandCollision(int argc, char** argv, std::string urdf_string, std::string srdf_string) : options_(urdf_string, srdf_string), robot_model_loader_(options_), planning_scene_(kinematic_model_)
{
  //this->collision_request_.contacts = false;//true;
  //this->collision_request_.max_contacts = 1000; 
}



double DualArmDualHandCollision::check_collision(const std::vector<double> q_in)
{
  /* This function computes the minimum distance between links inside one group or of different groups */

  // Get the joint angles for each group
  std::vector<double> q_left_arm(q_in.begin(), q_in.begin()+6); //(6); 
  std::vector<double> q_right_arm(q_in.begin()+6, q_in.begin()+12); //(6);
  std::vector<double> q_left_finger(q_in.begin()+12, q_in.begin()+24); //(12); 
  std::vector<double> q_right_finger(q_in.begin()+24, q_in.begin()+36); //(12);

  // Set current state (joint positions)
  this->current_state_.setJointGroupPositions(this->left_arm_group_, q_left_arm);
  this->current_state_.setJointGroupPositions(this->right_arm_group_, q_right_arm);
  this->current_state_.setJointGroupPositions(this->left_hand_group_, q_left_finger);
  this->current_state_.setJointGroupPositions(this->right_hand_group_, q_right_finger);
  this->current_state_.update();

  // Check if a robot is in collision with itself
  this->planning_scene_.checkSelfCollision(this->collision_request_, this->collision_result_);
  //std::cout << "The robot is " << (collision_result.collision ? "in" : "not in") << " collision with other parts." << std::endl;

  return (this->collision_result_.collision ? 1 : -1);
  
}



int teset_main(int argc, char** argv)
{

  std::vector<double> q_fengren_start = {0.35301845,  0.99484603, -0.99347436, -0.2162805 ,  0.41034526,
        0.95757483, -0.33122886, -0.98664004,  0.99919541,  0.00537776,
       -0.05391638, -0.20172656, -0.07528547, -0.05101342, -0.04827343,
       -0.30030468, -0.04818042, -0.47510671, -0.07566821, -0.05101342,
       -0.34908235,  0.22503758, -0.13536288, -0.12736718, -0.11545444,
       -0.14287161, -0.79437708, -0.91255084, -0.53623023, -1.00002908,
       -0.54937122, -0.74258424, -0.35016895,  0.21113075, -0.17714353,
       -0.44359589};


  // Settings
  std::string urdf_file_name = "/home/liangyuwei/sign_language_robot_ws/src/ur_description/urdf/ur5_robot_with_hands.urdf";
    std::string srdf_file_name = "/home/liangyuwei/sign_language_robot_ws/src/sign_language_robot_moveit_config/config/ur5.srdf";


  // Set a class
  std::chrono::steady_clock::time_point time_start1 = std::chrono::steady_clock::now();

  std::ifstream urdf_file(urdf_file_name);
  std::ifstream srdf_file(srdf_file_name);
  std::stringstream urdf_string, srdf_string;
  urdf_string << urdf_file.rdbuf();
  srdf_string << srdf_file.rdbuf();

  std::chrono::steady_clock::time_point time_end1 = std::chrono::steady_clock::now();
  std::chrono::duration<double> time_used1 = std::chrono::duration_cast<std::chrono::duration<double>>(time_end1 - time_start1);
  std::cout << "time used for reading URDF and SRDF files:" << time_used1.count() << " s" << std::endl;



  //std::vector<double> min_dist_results;
  double min_dist_results;

  std::chrono::steady_clock::time_point time_start2 = std::chrono::steady_clock::now();

  ros::init(argc, argv, "sign_language_robot_collision_computation");

  std::chrono::steady_clock::time_point time_end2 = std::chrono::steady_clock::now();
  std::chrono::duration<double> time_used2 = std::chrono::duration_cast<std::chrono::duration<double>>(time_end2 - time_start2);
  std::cout << "time used for initializing ROS node:" << time_used2.count() << " s" << std::endl;


  DualArmDualHandCollision dual_arm_dual_hand_collision(argc, argv, urdf_string.str(), srdf_string.str());


  min_dist_results = dual_arm_dual_hand_collision.check_collision(q_fengren_start);



  //ros::spin();
  //spinner.stop();

  // Shut down
  //ros::shutdown();
  return 0;


}

