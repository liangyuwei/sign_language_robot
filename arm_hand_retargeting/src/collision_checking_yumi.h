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

// For PlanningSceneMonitor
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

// Eigen
#include <Eigen/Eigen>

// For collision objects
#include "geometric_shapes/shapes.h"


class DualArmDualHandCollision
{

  public:
    DualArmDualHandCollision();
    // Initialization list
    DualArmDualHandCollision(std::string urdf_string, std::string srdf_string);// : options_(urdf_string, srdf_string), robot_model_loader_(options_), planning_scene_(kinematic_model_), collision_robot_fcl_(kinematic_model_);

    ~DualArmDualHandCollision(){};

    // API
    double check_self_collision(const std::vector<double> q_in);
    double check_arm_self_collision(const std::vector<double> q_in);
    double check_hand_self_collision(const std::vector<double> q_in);
    double check_world_collision(const std::vector<double> q_in);    
    double check_full_collision(const std::vector<double> q_in);
    double compute_self_distance(const std::vector<double> q_in);
    double compute_world_distance(const std::vector<double> q_in);
    // planning_scene.distanceToCollision??? --> no such API in my knowledge


    // Methods setting joint values for different robot configurations
    void set_joint_values_yumi(const std::vector<double> q_in);
    //void set_joint_values_ur5(const std::vector<double> q_in);


    // Get a copy of the PlanningScene maintained by move_group, to obtain information about collision objects
    planning_scene::PlanningScenePtr get_move_group_planning_scene();


    // Add primitive collision objects(for now) or scan from sensor(?)
    void apply_collision_objects();
    void remove_collision_objects();



  private:
    //ros::AsyncSpinner spinner_;
    ros::NodeHandle node_handle_;
    robot_model_loader::RobotModelLoader::Options options_;
    robot_model_loader::RobotModelLoader robot_model_loader_;
    robot_model::RobotModelPtr kinematic_model_ = robot_model_loader_.getModel();

    // Set up a local PlanningScene (must make sure the one loaded in robot_description is in consistent with the one loaded by URDF and SRDF)
    planning_scene::PlanningScene local_planning_scene_;
    robot_state::RobotState &current_state_ = local_planning_scene_.getCurrentStateNonConst();

    // In order to acquire the move_group's PlanningScene
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description"); // this would print lots of useless information, do it just once


    const robot_model::JointModelGroup *left_arm_group_ = current_state_.getJointModelGroup("left_arm");
    const robot_model::JointModelGroup *right_arm_group_ = current_state_.getJointModelGroup("right_arm");
    const robot_model::JointModelGroup *left_hand_group_ = current_state_.getJointModelGroup("left_hand");
    const robot_model::JointModelGroup *right_hand_group_ = current_state_.getJointModelGroup("right_hand");
    collision_detection::CollisionRequest collision_request_;
    collision_detection::CollisionResult collision_result_;
    collision_detection::DistanceRequest distance_request_;
    collision_detection::DistanceResult distance_result_;
    collision_detection::AllowedCollisionMatrix acm_ = local_planning_scene_.getAllowedCollisionMatrix();


    std::vector<std::string> left_arm_link_names_ = left_arm_group_->getLinkModelNames();
    std::vector<std::string> right_arm_link_names_ = right_arm_group_->getLinkModelNames();
    std::vector<std::string> left_hand_link_names_ = left_hand_group_->getLinkModelNames();
    std::vector<std::string> right_hand_link_names_ = right_hand_group_->getLinkModelNames();


};


