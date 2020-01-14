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



class DualArmDualHandCollision
{

  public:
    DualArmDualHandCollision();
    // Initialization list
    DualArmDualHandCollision(int argc, char **argv, std::string urdf_string, std::string srdf_string);// : options_(urdf_string, srdf_string), robot_model_loader_(options_), planning_scene_(kinematic_model_), collision_robot_fcl_(kinematic_model_);

    ~DualArmDualHandCollision(){};

    //std::vector<double> compute_minimum_distance(const std::vector<double> q_in);
    double check_collision(const std::vector<double> q_in);
    //int assign_link_to_groups(std::string link_name);

  private:
    //ros::AsyncSpinner spinner_;
    robot_model_loader::RobotModelLoader::Options options_;
    robot_model_loader::RobotModelLoader robot_model_loader_;
    robot_model::RobotModelPtr kinematic_model_ = robot_model_loader_.getModel();
    planning_scene::PlanningScene planning_scene_;
    robot_state::RobotState& current_state_ = planning_scene_.getCurrentStateNonConst();
    //collision_detection::CollisionRobotFCL collision_robot_fcl_;


    const robot_model::JointModelGroup *left_arm_group_ = this->current_state_.getJointModelGroup("left_arm");
    const robot_model::JointModelGroup *right_arm_group_ = this->current_state_.getJointModelGroup("right_arm");
    const robot_model::JointModelGroup *left_hand_group_ = this->current_state_.getJointModelGroup("left_hand");
    const robot_model::JointModelGroup *right_hand_group_ = this->current_state_.getJointModelGroup("right_hand");
    collision_detection::CollisionRequest collision_request_;
    collision_detection::CollisionResult collision_result_;
    collision_detection::AllowedCollisionMatrix acm_;


    std::vector<std::string> left_arm_link_names_ = this->left_arm_group_->getLinkModelNames();
    std::vector<std::string> right_arm_link_names_ = this->right_arm_group_->getLinkModelNames();
    std::vector<std::string> left_hand_link_names_ = this->left_hand_group_->getLinkModelNames();
    std::vector<std::string> right_hand_link_names_ = this->right_hand_group_->getLinkModelNames();


};


