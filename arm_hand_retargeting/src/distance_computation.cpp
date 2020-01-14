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
#include "distance_computation.h"

/*
class DualArmDualHandMinDistance
{

  public:
    DualArmDualHandMinDistance();
    // Initialization list
    DualArmDualHandMinDistance(int argc, char **argv, std::string urdf_string, std::string srdf_string);// : options_(urdf_string, srdf_string), robot_model_loader_(options_), planning_scene_(kinematic_model_), collision_robot_fcl_(kinematic_model_);

    ~DualArmDualHandMinDistance(){};

    //std::vector<double> compute_minimum_distance(const std::vector<double> q_in);
    double compute_minimum_distance(const std::vector<double> q_in);
    //int assign_link_to_groups(std::string link_name);

  private:
    //ros::AsyncSpinner spinner_;
    robot_model_loader::RobotModelLoader::Options options_;
    robot_model_loader::RobotModelLoader robot_model_loader_;
    robot_model::RobotModelPtr kinematic_model_ = robot_model_loader_.getModel();
    planning_scene::PlanningScene planning_scene_;
    robot_state::RobotState& current_state_ = planning_scene_.getCurrentStateNonConst();
    collision_detection::CollisionRobotFCL collision_robot_fcl_;


    const robot_model::JointModelGroup *left_arm_group_ = this->current_state_.getJointModelGroup("left_arm");
    const robot_model::JointModelGroup *right_arm_group_ = this->current_state_.getJointModelGroup("right_arm");
    const robot_model::JointModelGroup *left_hand_group_ = this->current_state_.getJointModelGroup("left_hand");
    const robot_model::JointModelGroup *right_hand_group_ = this->current_state_.getJointModelGroup("right_hand");
    collision_detection::DistanceRequest distance_request_;
    collision_detection::DistanceResult distance_result_;
    collision_detection::AllowedCollisionMatrix acm_;


    std::vector<std::string> left_arm_link_names_ = this->left_arm_group_->getLinkModelNames();
    std::vector<std::string> right_arm_link_names_ = this->right_arm_group_->getLinkModelNames();
    std::vector<std::string> left_hand_link_names_ = this->left_hand_group_->getLinkModelNames();
    std::vector<std::string> right_hand_link_names_ = this->right_hand_group_->getLinkModelNames();


};
*/


DualArmDualHandMinDistance::DualArmDualHandMinDistance(int argc, char** argv, std::string urdf_string, std::string srdf_string) : options_(urdf_string, srdf_string), robot_model_loader_(options_), planning_scene_(kinematic_model_), collision_robot_fcl_(kinematic_model_)
{

  //std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();

  // Set up a DistanceRequest and a DistanceResult
  this->distance_request_ = collision_detection::DistanceRequest();
  this->distance_result_ = collision_detection::DistanceResult();
  this->distance_request_.group_name = ""; // no specified group
  this->distance_request_.enable_signed_distance = true;
  this->distance_request_.type = collision_detection::DistanceRequestType::SINGLE; // global minimum
  //distance_request.enableGroup(kinematic_model); // specify which group to check
  this->acm_ = this->planning_scene_.getAllowedCollisionMatrix();
  this->distance_request_.acm = &(this->acm_); // specify acm to ignore adjacent links' collision check


  // Display link names of each group
  /*
  std::cout << "Link names, that are part of a joint group: " << std::endl;
  std::cout << "Left arm's links: " << std::endl;
  for (auto it = this->left_arm_link_names_.cbegin(); it != this->left_arm_link_names_.cend(); ++it)
  {
    std::cout << *it << " ";
  }
  std::cout << std::endl;

  std::cout << "Right arm's links: " << std::endl;
  for (auto it = this->right_arm_link_names_.cbegin(); it != this->right_arm_link_names_.cend(); ++it)
  {
    std::cout << *it << " ";
  }
  std::cout << std::endl;

  std::cout << "Left hand's links: " << std::endl;
  for (auto it = this->left_hand_link_names_.cbegin(); it != this->left_hand_link_names_.cend(); ++it)
  {
    std::cout << *it << " ";
  }
  std::cout << std::endl;

  std::cout << "Right hand's links: " << std::endl;
  for (auto it = this->right_hand_link_names_.cbegin(); it != this->right_hand_link_names_.cend(); ++it)
  {
    std::cout << *it << " ";
  }
  std::cout << std::endl;
  */


  //std::chrono::steady_clock::time_point time_end = std::chrono::steady_clock::now();
  //std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(time_end - time_start);

  //std::cout << "time used for constructor: " << time_used.count() << " s" << std::endl;


}


/*
int DualArmDualHandMinDistance::assign_link_to_groups(std::string link_name)
{
  return 0;
}
*/

double DualArmDualHandMinDistance::compute_minimum_distance(const std::vector<double> q_in)
{
  /* This function computes the minimum distance between links inside one group or of different groups */

  // Clock
  //std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();


  // The vector to store the results
  std::vector<double> min_dist_intra_group = {10.0, 10.0, 10.0, 10.0}; // r_hand, l_hand, r_arm, l_arm
  std::vector<double> min_dist_inter_group = {10.0, 10.0, 10.0, 10.0}; // rh_lh, rh_la, lh_ra, ra_la
  bool groups_in_contact[4] = {false, false, false, false};

  // Get the joint angles for each group
  std::vector<double> q_left_arm(q_in.begin(), q_in.begin()+6); //(6); 
  std::vector<double> q_right_arm(q_in.begin()+6, q_in.begin()+12); //(6);
  std::vector<double> q_left_finger(q_in.begin()+12, q_in.begin()+24); //(12); 
  std::vector<double> q_right_finger(q_in.begin()+24, q_in.begin()+36); //(12);
  //std::copy(q_in.begin(), q_in.begin()+6, q_left_arm);
  //std::copy(q_in.begin()+6, q_in.begin()+12, q_right_arm);
  //std::copy(q_in.begin()+12, q_in.begin()+24, q_left_finger);
  //std::copy(q_in.begin()+24, q_in.begin()+36, q_right_finger);


  // Set current state (joint positions)
  this->current_state_.setJointGroupPositions(this->left_arm_group_, q_left_arm);
  this->current_state_.setJointGroupPositions(this->right_arm_group_, q_right_arm);
  this->current_state_.setJointGroupPositions(this->left_hand_group_, q_left_finger);
  this->current_state_.setJointGroupPositions(this->right_hand_group_, q_right_finger);
  this->current_state_.update();

  
  // If distance computation for all groups is too expensive, try to eliminate some by checking collision first
  // Check if a robot is in collision with itself
  /*
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  collision_request.contacts = true;
  collision_request.max_contacts = 1000; // overall maximum number of contacts to compute(in contact map?)
  // collision_request.distance = true;  // set to compute distance
  this->planning_scene_.checkSelfCollision(collision_request, collision_result);
  std::cout << "The robot is " << (collision_result.collision ? "in" : "not in") << " collision with other parts." << std::endl;
  if (! (collision_result.collision)) // if not in collision state
  {
    std::vector<double> result;
    result.insert(result.end(), min_dist_intra_group.begin(), min_dist_intra_group.end());
    result.insert(result.end(), min_dist_inter_group.begin(), min_dist_inter_group.end());
    return result;
  }
  else // if in collision, check which parts, and compute the corresponding minimum distance
  {

    // find out which groups are in collision from contact map information
    collision_detection::CollisionResult::ContactMap::const_iterator it;
    for (it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it)
    {
      std::cout << "Contact between: " << it->first.first.c_str() << " and " << it->first.second.c_str() << std::endl;
    }
    std::cout << "Closest distance is " << collision_result.distance << std::endl; 
   
    collision_result.clear();

   // Check if a group is in collision with other parts
    collision_request.group_name = "left_arm";
    this->planning_scene_.checkSelfCollision(collision_request, collision_result);
    std::cout << collision_request.group_name << "is " << (collision_result.collision ? "in" : "not in") << " collision with other parts." << std::endl;
    collision_result.clear();

  } 
  */ 


  // Compute all multiple minimum distance between groups
  /*
  // Get Distance Map
  this->collision_robot_fcl_.distanceSelf(this->distance_request_, this->distance_result_, this->current_state_);
  std::cout << "Distance Map:\n" ;
  collision_detection::DistanceMap::const_iterator itr;
  int i = 0;
  for (itr = this->distance_result_.distances.begin(); itr != this->distance_result_.distances.end(); ++itr)
  {
    //assign_link_to_groups
    //std::find(left_arm_link_names.begin(), left_arm_link_names.end(), itr->first.first)


    std::cout << "Distance between " << itr->first.first << " and " << itr->first.second << " is " << itr->second[i].distance << std::endl;
    i++;
  }

  std::cout << std::endl;
  std::cout << "Minimum distance is " << this->distance_result_.minimum_distance.distance << ", between " << this->distance_result_.minimum_distance.link_names[0] << " and " << this->distance_result_.minimum_distance.link_names[1] << std::endl;

  // Clear the results
  this->distance_result_.clear();


  // The result
  std::vector<double> result;
  result.insert(result.end(), min_dist_intra_group.begin(), min_dist_intra_group.end());
  result.insert(result.end(), min_dist_inter_group.begin(), min_dist_inter_group.end());
  return result;

  */



  // Return just one minimum distance (over all parts, global minimum)
  this->collision_robot_fcl_.distanceSelf(this->distance_request_, this->distance_result_, this->current_state_);
  //std::cout << "Minimum distance is " << this->distance_result_.minimum_distance.distance << ", between " << this->distance_result_.minimum_distance.link_names[0] << " and " << this->distance_result_.minimum_distance.link_names[1] << std::endl;



  //std::chrono::steady_clock::time_point time_end = std::chrono::steady_clock::now();
  //std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(time_end - time_start);
  //std::cout << "time use for computing minimum distance:" << time_used.count() << " s" << std::endl;


  return this->distance_result_.minimum_distance.distance;

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


  DualArmDualHandMinDistance dual_arm_dual_hand_min_distance(argc, argv, urdf_string.str(), srdf_string.str());


  min_dist_results = dual_arm_dual_hand_min_distance.compute_minimum_distance(q_fengren_start);



  //ros::spin();
  //spinner.stop();

  // Shut down
  //ros::shutdown();
  return 0;


}

