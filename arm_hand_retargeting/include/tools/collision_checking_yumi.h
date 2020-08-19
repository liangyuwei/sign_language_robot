#ifndef COLLISION_CHECKING_YUMI_H
#define COLLISION_CHECKING_YUMI_H

// For ROS
#include <ros/ros.h>

// Common
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
#include <Eigen/Geometry>

// For collision objects
#include "geometric_shapes/shapes.h"

// For file operation
#include <fstream>

/**
 * @brief Collision checking functionality for YuMi dual-arm robot.
 * 
 * This class encapsulates the caller to MoveIt APIs for collision checking, distance computation via FCL. \n
 * q velocity can also be computed with the use of contact normal and robot jacobian information.
 */
class DualArmDualHandCollision
{

  public:
    DualArmDualHandCollision();

    // Initialization list
    /// Constructor, initialized with paths to urdf and srdf files
    DualArmDualHandCollision(std::string urdf_string, std::string srdf_string);
    ~DualArmDualHandCollision(){};

    // Encapsulated usage of APIs
    double check_self_collision(const std::vector<double> q_in);
    double check_arm_self_collision(const std::vector<double> q_in);
    double check_hand_self_collision(const std::vector<double> q_in);
    double check_world_collision(const std::vector<double> q_in);    
    double check_full_collision(const std::vector<double> q_in);
    double compute_self_distance(const std::vector<double> q_in);

    double compute_self_distance_test(const std::vector<double> q_in, std::string group_name, double distance_threshold);
    double compute_two_links_distance(const std::vector<double> q_in, std::string link_name_1, std::string link_name_2, double distance_threshold);

    double compute_world_distance(const std::vector<double> q_in);
    // planning_scene.distanceToCollision??? --> no such API in my knowledge


    // Get robot jacobians under the current state(configuration)
    int check_link_belonging(std::string link_name);
    int check_finger_belonging(std::string link_name, bool left_or_right);

    Eigen::MatrixXd get_robot_arm_jacobian(std::string target_link_name, Eigen::Vector3d ref_point_pos, bool left_or_right);
    Eigen::MatrixXd get_robot_hand_jacobian(std::string target_link_name, Eigen::Vector3d ref_point_pos, int finger_id, bool left_or_right);
    Eigen::MatrixXd get_robot_arm_hand_jacobian(std::string target_link_name, Eigen::Vector3d ref_point_pos, int finger_id, bool left_or_right);
    Eigen::Vector3d get_global_link_transform(std::string target_link_name); // this is the transform of specified link under the current configuration (this->current_state_)


    // for use in g2o tracking constraint(get robot jacobian directly, for more efficient tracking of wrist and elbow trajectories)
    Eigen::MatrixXd get_robot_arm_jacobian(const std::vector<double> q_in, std::string target_link_name, Eigen::Vector3d ref_point_pos, bool left_or_right);
    Eigen::MatrixXd get_arm_jacobian(const std::vector<double> q_in, std::string target_link_name, Eigen::Vector3d ref_point_pos, bool left_or_right);

    Eigen::Vector3d get_link_pos(const std::vector<double> q_in, std::string target_link_name);
    Eigen::Matrix3d get_link_ori(const std::vector<double> q_in, std::string target_link_name); // Test FK functionality, this is about 10 times slower than KDL's ChainFkSolver...


    // Methods setting joint values for different robot configurations
    void set_joint_values_yumi(const std::vector<double> q_in);
    //void set_joint_values_ur5(const std::vector<double> q_in);


    // Get a copy of the PlanningScene maintained by move_group, to obtain information about collision objects
    planning_scene::PlanningScenePtr get_move_group_planning_scene();


    // Add primitive collision objects(for now) or scan from sensor(?)
    void apply_collision_objects();
    void remove_collision_objects();

    // Distance Result
    std::string link_names[2];
    Eigen::Vector3d nearest_points[2]; // array
    double min_distance;
    Eigen::Vector3d normal; // a normalized vector point from link_names[0] to link_names[1]

    // just for debug
    void test_active_components(std::string group_name);


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


    // Set up joint groups for later use
    // arm only
    const robot_model::JointModelGroup *left_arm_group_ = current_state_.getJointModelGroup("left_arm");
    const robot_model::JointModelGroup *right_arm_group_ = current_state_.getJointModelGroup("right_arm");
    const robot_model::JointModelGroup *left_hand_group_ = current_state_.getJointModelGroup("left_hand");
    const robot_model::JointModelGroup *right_hand_group_ = current_state_.getJointModelGroup("right_hand");
    // fingers only
    const robot_model::JointModelGroup *left_palm_group_ = current_state_.getJointModelGroup("left_palm");    
    const robot_model::JointModelGroup *right_palm_group_ = current_state_.getJointModelGroup("right_palm");    
    
    const robot_model::JointModelGroup *left_thumb_group_ = current_state_.getJointModelGroup("left_thumb");    
    const robot_model::JointModelGroup *left_index_group_ = current_state_.getJointModelGroup("left_index");    
    const robot_model::JointModelGroup *left_middle_group_ = current_state_.getJointModelGroup("left_middle");    
    const robot_model::JointModelGroup *left_ring_group_ = current_state_.getJointModelGroup("left_ring");    
    const robot_model::JointModelGroup *left_little_group_ = current_state_.getJointModelGroup("left_little");  

    const robot_model::JointModelGroup *right_thumb_group_ = current_state_.getJointModelGroup("right_thumb");    
    const robot_model::JointModelGroup *right_index_group_ = current_state_.getJointModelGroup("right_index");    
    const robot_model::JointModelGroup *right_middle_group_ = current_state_.getJointModelGroup("right_middle");    
    const robot_model::JointModelGroup *right_ring_group_ = current_state_.getJointModelGroup("right_ring");    
    const robot_model::JointModelGroup *right_little_group_ = current_state_.getJointModelGroup("right_little");  
    // arm + finger
    const robot_model::JointModelGroup *left_arm_thumb_group_ = current_state_.getJointModelGroup("left_arm_thumb");    
    const robot_model::JointModelGroup *left_arm_index_group_ = current_state_.getJointModelGroup("left_arm_index");    
    const robot_model::JointModelGroup *left_arm_middle_group_ = current_state_.getJointModelGroup("left_arm_middle");    
    const robot_model::JointModelGroup *left_arm_ring_group_ = current_state_.getJointModelGroup("left_arm_ring");    
    const robot_model::JointModelGroup *left_arm_little_group_ = current_state_.getJointModelGroup("left_arm_little");

    const robot_model::JointModelGroup *right_arm_thumb_group_ = current_state_.getJointModelGroup("right_arm_thumb");    
    const robot_model::JointModelGroup *right_arm_index_group_ = current_state_.getJointModelGroup("right_arm_index");    
    const robot_model::JointModelGroup *right_arm_middle_group_ = current_state_.getJointModelGroup("right_arm_middle");    
    const robot_model::JointModelGroup *right_arm_ring_group_ = current_state_.getJointModelGroup("right_arm_ring");    
    const robot_model::JointModelGroup *right_arm_little_group_ = current_state_.getJointModelGroup("right_arm_little");

    //
    collision_detection::CollisionRequest collision_request_;
    collision_detection::CollisionResult collision_result_;
    collision_detection::DistanceRequest distance_request_;
    collision_detection::DistanceResult distance_result_;
    collision_detection::AllowedCollisionMatrix acm_ = local_planning_scene_.getAllowedCollisionMatrix();

    // Link names
    std::vector<std::string> left_arm_link_names_ = left_arm_group_->getLinkModelNames();
    std::vector<std::string> right_arm_link_names_ = right_arm_group_->getLinkModelNames();
    std::vector<std::string> left_hand_link_names_ = left_hand_group_->getLinkModelNames();
    std::vector<std::string> right_hand_link_names_ = right_hand_group_->getLinkModelNames();

};

#endif
