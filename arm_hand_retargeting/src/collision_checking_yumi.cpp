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

#include <fstream>


/* Add a box to the scene (for now) */
void DualArmDualHandCollision::apply_collision_objects()
{
  // Add a box
  std::cout << "Add a box to the environment..." << std::endl;
  // Prepare collision objects(size and pose)
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = "/world";
  collision_object.id = "box1";
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.5;
  primitive.dimensions[1] = 0.3;
  primitive.dimensions[2] = 0.6;
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.489;
  box_pose.position.y = 0.328;
  box_pose.position.z = 0.408;
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  // way 1(checked): asynchronous way, interact with the move_group node
  std::cout << "Add a box to into the world through /apply_planning_scene" << std::endl;
  moveit_msgs::PlanningScene planning_scene_msg; // this is just a message !!!
  planning_scene_msg.world.collision_objects.clear();
  planning_scene_msg.world.collision_objects.push_back(collision_object);
  planning_scene_msg.is_diff = true;
  ros::ServiceClient planning_scene_diff_client = this->node_handle_.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
  planning_scene_diff_client.waitForExistence();
  moveit_msgs::ApplyPlanningScene srv; // synchronous manner, send the diffs to the planning scene via a service call
  srv.request.scene = planning_scene_msg;
  planning_scene_diff_client.call(srv); // does not continue until we are sure the diff has been applied


  // way 2(checked): add collision object throught PlanningSceneInterface (actually the mechanism is the same as the above one)
  /*
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  
  planning_scene_interface.applyCollisionObject(collision_object);
  */

}



/* Remove the box to the scene (for now) */
void DualArmDualHandCollision::remove_collision_objects()
{

  std::cout << "Remove the box from the scene..." << std::endl;

  // way 1: via service client synchronous way
  ros::ServiceClient planning_scene_diff_client = this->node_handle_.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
  planning_scene_diff_client.waitForExistence();

  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = "/world";
  collision_object.id = "box1";
  collision_object.operation = collision_object.REMOVE; // Remove operation

  moveit_msgs::PlanningScene planning_scene_msg; // this is just a message !!!
  planning_scene_msg.world.collision_objects.clear();
  planning_scene_msg.world.collision_objects.push_back(collision_object);
  planning_scene_msg.is_diff = true;

  moveit_msgs::ApplyPlanningScene srv; // synchronous manner, send the diffs to the planning scene via a service call
  srv.request.scene = planning_scene_msg;

  planning_scene_diff_client.call(srv);


  // way 2: planning scene interface
  /*
  std::vector<std::string> object_ids;
  object_ids.push_back("box1");
  planning_scene_interface.removeCollisionObjects(object_ids);
  */


}



/* Update current robot state with the given joint values (for YuMi robot) */
void DualArmDualHandCollision::set_joint_values_yumi(const std::vector<double> q_in)
{
  // Get the joint angles for each group
  //std::cout << "Update the robot state with the given joint angles" << std::endl;
  // Remember to change the joint value assignment here when switching to robots with different DOFs!!!
  std::vector<double> q_left_arm(q_in.begin(), q_in.begin()+7); //(7); 
  std::vector<double> q_right_arm(q_in.begin()+7, q_in.begin()+14); //(7);
  std::vector<double> q_left_finger(q_in.begin()+14, q_in.begin()+26); //(12); 
  std::vector<double> q_right_finger(q_in.begin()+26, q_in.begin()+38); //(12);

  // Set current state (joint positions)
  this->current_state_.setJointGroupPositions(this->left_arm_group_, q_left_arm);
  this->current_state_.setJointGroupPositions(this->right_arm_group_, q_right_arm);
  this->current_state_.setJointGroupPositions(this->left_hand_group_, q_left_finger);
  this->current_state_.setJointGroupPositions(this->right_hand_group_, q_right_finger);
  this->current_state_.update();

}


//void DualArmDualHandCollision::set_joint_values_ur5(const std::vector<double> q_in)



planning_scene::PlanningScenePtr DualArmDualHandCollision::get_move_group_planning_scene()
{
  // Get the PlanningScene maintained by move_group
  // communicating with move_group is a must now, i.e. a move_group should be launched during collision checking with the world(collision objects) or optimization.
  //std::cout << "========= Create a PlanningSceneMonitor from robot_description =========" << std::endl;
  //this->planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
  //std::cout << "========= Request PlanningSceneState through PlanningSceneMonitor =========" << std::endl;
  const std::string PLANNING_SCENE_SERVICE = "get_planning_scene"; // default service name
  this->planning_scene_monitor_->requestPlanningSceneState(PLANNING_SCENE_SERVICE);
  planning_scene_monitor::LockedPlanningSceneRW ps(this->planning_scene_monitor_);


  // clone a planning scene
  //std::cout << "========= Get child planning scene of the one maintained by move_group =========" << std::endl;
  planning_scene::PlanningScenePtr scene = ps->diff();
  scene->decoupleParent();


  return scene;

}


/* Constructor initialization */
DualArmDualHandCollision::DualArmDualHandCollision(std::string urdf_string, std::string srdf_string) : options_(urdf_string, srdf_string), robot_model_loader_(options_), local_planning_scene_(kinematic_model_)
{

  //this->collision_request_.contacts = false;//true;
  //this->collision_request_.max_contacts = 1000; 

}


/* Get the global transform (translation + rotation) of the specified link under the current configuration/state (current_state_) 
 * For use in calculating reference point position from nearest point position
 */
Eigen::Vector3d DualArmDualHandCollision::get_global_link_transform(std::string target_link_name)
{
  const Eigen::Affine3d target_link_state = this->current_state_.getGlobalLinkTransform(target_link_name);

  Eigen::Vector3d position = target_link_state.translation();
  Eigen::Matrix3d rotation = target_link_state.rotation();

  // std::cout << "debug: " << std::endl;
  // std::cout << "Translation: \n" << target_link_state.translation() << std::endl;
  // std::cout << "Rotation: \n" << target_link_state.rotation() << std::endl;

  return position;
}


Eigen::Vector3d DualArmDualHandCollision::get_link_pos(const std::vector<double> q_in, std::string target_link_name)
{

  // Set joint values and update current_state_ !
  this->set_joint_values_yumi(q_in);

  // Get transform data
  const Eigen::Affine3d target_link_state = this->current_state_.getGlobalLinkTransform(target_link_name);

  Eigen::Vector3d position = target_link_state.translation();
  // Eigen::Matrix3d rotation = target_link_state.rotation();

  return position;
}

Eigen::Matrix3d DualArmDualHandCollision::get_link_ori(const std::vector<double> q_in, std::string target_link_name)
{

  // Set joint values and update current_state_ !
  this->set_joint_values_yumi(q_in);

  // Get transform data
  const Eigen::Affine3d target_link_state = this->current_state_.getGlobalLinkTransform(target_link_name);

  // Eigen::Vector3d position = target_link_state.translation();
  Eigen::Matrix3d rotation = target_link_state.rotation();

  return rotation;
}


/* Get robot arm jacobian with the given link and reference_point_position. 
 * left_or_right: choose left of right
 */
Eigen::MatrixXd DualArmDualHandCollision::get_robot_arm_jacobian(std::string target_link_name, Eigen::Vector3d ref_point_pos, bool left_or_right)
{
  // Prep
  Eigen::MatrixXd jacobian;
  bool result;

  // See which groups to compute robot jacobian for
  if (left_or_right) // left arm + left hand
  {
    result = this->current_state_.getJacobian(this->left_arm_group_, 
                                              this->current_state_.getLinkModel(target_link_name),
                                              ref_point_pos, jacobian);
  }
  else
  {
    result = this->current_state_.getJacobian(this->right_arm_group_, 
                                              this->current_state_.getLinkModel(target_link_name),
                                              ref_point_pos, jacobian);
  }


  // Check the results
  if (result)
  {
    // std::cout << "Jacobian calculated successfully !" << std::endl;
    // std::cout << "Size of robot jacobian: " << jacobian.rows() << " x " << jacobian.cols() << std::endl;
  }
  else
  {
    std::cout << "Failed to compute robot jacobian for " << target_link_name << " at ref_point_pos = " << ref_point_pos.transpose() << std::endl;
    exit(-1);
  }

  return jacobian;

}


/* Calculate robot jacobians for left arm or right arm
 * Note that q_in is just joint angles for left/right arm, no need to set all
 */
Eigen::MatrixXd DualArmDualHandCollision::get_arm_jacobian(const std::vector<double> q_in, std::string target_link_name, Eigen::Vector3d ref_point_pos, bool left_or_right)
{
  // Update current state
  // this->set_joint_values_yumi(q_in);
  if (left_or_right)
  {
    this->current_state_.setJointGroupPositions(this->left_arm_group_, q_in);
  }
  else
  {
    this->current_state_.setJointGroupPositions(this->right_arm_group_, q_in);
  }
  this->current_state_.update(); // must update RobotState after setting joint positions!!!


  // Prep
  Eigen::MatrixXd jacobian;
  bool result;

  // See which groups to compute robot jacobian for
  if (left_or_right) // left arm + left hand
  {
    result = this->current_state_.getJacobian(this->left_arm_group_, 
                                              this->current_state_.getLinkModel(target_link_name),
                                              ref_point_pos, jacobian);
  }
  else
  {
    result = this->current_state_.getJacobian(this->right_arm_group_, 
                                              this->current_state_.getLinkModel(target_link_name),
                                              ref_point_pos, jacobian);
  }


  // Check the results
  if (result)
  {
    // std::cout << "Jacobian calculated successfully !" << std::endl;
    // std::cout << "Size of robot jacobian: " << jacobian.rows() << " x " << jacobian.cols() << std::endl;
  }
  else
  {
    std::cout << "Failed to compute robot jacobian for " << (left_or_right ? "left" : "right") << " arm." << std::endl;
    exit(-1);
  }

  return jacobian;

}

/* Get finger jacobian with the given link and reference_point_position. 
 * finger_id: 0 - thumb, 1 - index, 2 - middle, 3 - ring, 4 - little
 * left_or_right: choose left of right
 */
Eigen::MatrixXd DualArmDualHandCollision::get_robot_hand_jacobian(std::string target_link_name, Eigen::Vector3d ref_point_pos, int finger_id, bool left_or_right)
{
  // Prep
  Eigen::MatrixXd jacobian;
  bool result;

  // See which groups to compute robot jacobian for
  if (left_or_right) // left arm + left hand
  {
    switch (finger_id)
    {
      case 0: // thumb
        result = this->current_state_.getJacobian(this->left_thumb_group_, 
                                                  this->current_state_.getLinkModel(target_link_name),
                                                  ref_point_pos, jacobian);
        break;
      case 1: // index
        result = this->current_state_.getJacobian(this->left_index_group_, 
                                                  this->current_state_.getLinkModel(target_link_name),
                                                  ref_point_pos, jacobian);
        break;
      case 2: // middle
        result = this->current_state_.getJacobian(this->left_middle_group_, 
                                                  this->current_state_.getLinkModel(target_link_name),
                                                  ref_point_pos, jacobian);
        break;
      case 3: // ring
        result = this->current_state_.getJacobian(this->left_ring_group_, 
                                                  this->current_state_.getLinkModel(target_link_name),
                                                  ref_point_pos, jacobian);
        break;
      case 4: // little
        result = this->current_state_.getJacobian(this->left_little_group_, 
                                                  this->current_state_.getLinkModel(target_link_name),
                                                  ref_point_pos, jacobian);
        break;
      case -1: // palm
        result = true;
        break;
      default:
        std::cerr << "Finger ID does not lie in {-1, 0, 1, 2, 3, 4} !!!" << std::endl;
        exit(-1);
        break;
    }
  }
  else
  {
    switch (finger_id)
    {
      case 0: // thumb
        result = this->current_state_.getJacobian(this->right_thumb_group_, 
                                                  this->current_state_.getLinkModel(target_link_name),
                                                  ref_point_pos, jacobian);
        break;
      case 1: // index
        result = this->current_state_.getJacobian(this->right_index_group_, 
                                                  this->current_state_.getLinkModel(target_link_name),
                                                  ref_point_pos, jacobian);
        break;
      case 2: // middle
        result = this->current_state_.getJacobian(this->right_middle_group_, 
                                                  this->current_state_.getLinkModel(target_link_name),
                                                  ref_point_pos, jacobian);
        break;
      case 3: // ring
        result = this->current_state_.getJacobian(this->right_ring_group_, 
                                                  this->current_state_.getLinkModel(target_link_name),
                                                  ref_point_pos, jacobian);
        break;
      case 4: // little
        result = this->current_state_.getJacobian(this->right_little_group_, 
                                                  this->current_state_.getLinkModel(target_link_name),
                                                  ref_point_pos, jacobian);
        break;
      case -1: // palm
        result = true;
        break;

      default:
        std::cerr << "Finger ID does not lie in {0, 1, 2, 3, 4} !!!" << std::endl;
        exit(-1);
        break;
    }
  }


  // Check the results
  if (result)
  {
    // std::cout << "Jacobian calculated successfully !" << std::endl;
    // std::cout << "Size of robot jacobian: " << jacobian.rows() << " x " << jacobian.cols() << std::endl;
  }
  else
  {
    std::cout << "Failed to compute robot jacobian for " << target_link_name << " at ref_point_pos = " << ref_point_pos.transpose() << std::endl;
    exit(-1);
  }

  return jacobian;
}



/* Get arm+hand jacobian with the given link and reference_point_position. 
 * finger_id: 0 - thumb, 1 - index, 2 - middle, 3 - ring, 4 - little
 * left_or_right: choose left of right
 */
Eigen::MatrixXd DualArmDualHandCollision::get_robot_arm_hand_jacobian(std::string target_link_name, Eigen::Vector3d ref_point_pos, int finger_id, bool left_or_right)
{
  // Prep
  Eigen::MatrixXd jacobian;
  bool result;

  // See which groups to compute robot jacobian for
  if (left_or_right) // left arm + left hand
  {
    switch (finger_id)
    {
      case 0: // thumb
        result = this->current_state_.getJacobian(this->left_arm_thumb_group_, 
                                                  this->current_state_.getLinkModel(target_link_name),
                                                  ref_point_pos, jacobian);
        break;
      case 1: // index
        result = this->current_state_.getJacobian(this->left_arm_index_group_, 
                                                  this->current_state_.getLinkModel(target_link_name),
                                                  ref_point_pos, jacobian);
        break;
      case 2: // middle
        result = this->current_state_.getJacobian(this->left_arm_middle_group_, 
                                                  this->current_state_.getLinkModel(target_link_name),
                                                  ref_point_pos, jacobian);
        break;
      case 3: // ring
        result = this->current_state_.getJacobian(this->left_arm_ring_group_, 
                                                  this->current_state_.getLinkModel(target_link_name),
                                                  ref_point_pos, jacobian);
        break;
      case 4: // little
        result = this->current_state_.getJacobian(this->left_arm_little_group_, 
                                                  this->current_state_.getLinkModel(target_link_name),
                                                  ref_point_pos, jacobian);
        break;
      case -1: // palm, treat as any other finger_groups, since palm link is not root link for any arm_hand_groups!!
        result = this->current_state_.getJacobian(this->left_arm_little_group_, 
                                                  this->current_state_.getLinkModel(target_link_name),
                                                  ref_point_pos, jacobian);
        break;
      default:
        std::cerr << "Finger ID does not lie in {-1, 0, 1, 2, 3, 4} !!!" << std::endl;
        exit(-1);
        break;
    }
  }
  else
  {
    switch (finger_id)
    {
      case 0: // thumb
        result = this->current_state_.getJacobian(this->right_arm_thumb_group_, 
                                                  this->current_state_.getLinkModel(target_link_name),
                                                  ref_point_pos, jacobian);
        break;
      case 1: // index
        result = this->current_state_.getJacobian(this->right_arm_index_group_, 
                                                  this->current_state_.getLinkModel(target_link_name),
                                                  ref_point_pos, jacobian);
        break;
      case 2: // middle
        result = this->current_state_.getJacobian(this->right_arm_middle_group_, 
                                                  this->current_state_.getLinkModel(target_link_name),
                                                  ref_point_pos, jacobian);
        break;
      case 3: // ring
        result = this->current_state_.getJacobian(this->right_arm_ring_group_, 
                                                  this->current_state_.getLinkModel(target_link_name),
                                                  ref_point_pos, jacobian);
        break;
      case 4: // little
        result = this->current_state_.getJacobian(this->right_arm_little_group_, 
                                                  this->current_state_.getLinkModel(target_link_name),
                                                  ref_point_pos, jacobian);
        break;
      case -1: // palm, treat as any other finger_groups, since palm link is not root link for any arm_hand_groups!!
        result = this->current_state_.getJacobian(this->right_arm_little_group_, 
                                                  this->current_state_.getLinkModel(target_link_name),
                                                  ref_point_pos, jacobian);
        break;
      default:
        std::cerr << "Finger ID does not lie in {-1, 0, 1, 2, 3, 4} !!!" << std::endl;
        exit(-1);
        break;
    }
  }


  // Check the results
  if (result)
  {
    // std::cout << "Jacobian calculated successfully !" << std::endl;
    // std::cout << "Size of robot jacobian: " << jacobian.rows() << " x " << jacobian.cols() << std::endl;
  }
  else
  {
    std::cout << "Failed to compute robot jacobian for " << target_link_name << " at ref_point_pos = " << ref_point_pos.transpose() << std::endl;
    exit(-1);
  }

  return jacobian;
}



/* Self-collision checking via PlanningScene::checkSelfCollision */
double DualArmDualHandCollision::check_self_collision(const std::vector<double> q_in)
{

  // Prepare collision_request
  this->collision_request_.group_name = "";
  this->collision_request_.distance = false;//true; // only for debug, when used with optimization, remember to shut it off!!!! cause great difference in time usage!!!
  // this->collision_request_.contacts = true;
  // this->collision_request_.max_contacts = 1000;

  this->collision_result_.clear();


  // Get the PlanningScene maintained by move_group (getting PlanningScene from move_group might not be necessary for self-collision checking...)
  //planning_scene::PlanningScenePtr move_group_planning_scene = this->get_move_group_planning_scene();


  // Update robot state with the given joint values
  this->set_joint_values_yumi(q_in);


  // Check if a robot is in collision with itself
  this->local_planning_scene_.checkSelfCollision(this->collision_request_, this->collision_result_, this->current_state_, this->acm_); // using local planning scene is faster
  //move_group_planning_scene->checkSelfCollision(this->collision_request_, this->collision_result_, this->current_state_, this->acm_);



  // Display information for debug
/*
  std::cout << "The robot is " << (this->collision_result_.collision ? "in" : "not in") << " self-collision." << std::endl;  
  std::cout << "Closest distance: " << this->collision_result_.distance << std::endl;
  for (auto it = this->collision_result_.contacts.begin(); it != this->collision_result_.contacts.end(); ++it)
  {
    std::cout << "Contact between: " << it->first.first.c_str() << " and " << it->first.second.c_str() << std::endl;
  }
*/

  // Return the rsult
  double result = (this->collision_result_.collision ? 1 : -1); // 1 for colliding, -1 for free
  this->collision_result_.clear(); // should clear the results, in case the result object is used again!!!
  return result; // set to 1 if in collision
  
}


/* Self-collision checking via PlanningScene::checkSelfCollision (Arms Only) */
double DualArmDualHandCollision::check_arm_self_collision(const std::vector<double> q_in)
{

  // Prepare collision_request
  this->collision_request_.group_name = "dual_arms";
  this->collision_request_.distance = false;//true; // only for debug, when used with optimization, remember to shut it off!!!! cause great difference in time usage!!!
  this->collision_request_.contacts = true;
  this->collision_request_.max_contacts = 1000;

  this->collision_result_.clear();


  // Get the PlanningScene maintained by move_group (getting PlanningScene from move_group might not be necessary for self-collision checking...)
  //planning_scene::PlanningScenePtr move_group_planning_scene = this->get_move_group_planning_scene();


  // Update robot state with the given joint values
  this->set_joint_values_yumi(q_in);


  // Check if a robot is in collision with itself
  this->local_planning_scene_.checkSelfCollision(this->collision_request_, this->collision_result_, this->current_state_, this->acm_); // using local planning scene is faster
  //move_group_planning_scene->checkSelfCollision(this->collision_request_, this->collision_result_, this->current_state_, this->acm_);



  // Display information for debug
  /*
  std::cout << "The robot is " << (this->collision_result_.collision ? "in" : "not in") << " self-collision." << std::endl;  
  std::cout << "Closest distance: " << this->collision_result_.distance << std::endl;
  for (auto it = this->collision_result_.contacts.begin(); it != this->collision_result_.contacts.end(); ++it)
  {
    std::cout << "Contact between: " << it->first.first.c_str() << " and " << it->first.second.c_str() << std::endl;
  }
  */


  // Return the rsult
  double result = (this->collision_result_.collision ? 1 : -1); // 1 for colliding, -1 for free
  this->collision_result_.clear(); // should clear the results, in case the result object is used again!!!
  return result; // set to 1 if in collision
  
}

/* Self-collision checking via PlanningScene::checkSelfCollision (Hands Only) */
double DualArmDualHandCollision::check_hand_self_collision(const std::vector<double> q_in)
{

  // Prepare collision_request
  this->collision_request_.group_name = "dual_hands";
  this->collision_request_.distance = false;//true; // only for debug, when used with optimization, remember to shut it off!!!! cause great difference in time usage!!!
  this->collision_request_.contacts = true;
  this->collision_request_.max_contacts = 1000;

  this->collision_result_.clear();


  // Get the PlanningScene maintained by move_group (getting PlanningScene from move_group might not be necessary for self-collision checking...)
  //planning_scene::PlanningScenePtr move_group_planning_scene = this->get_move_group_planning_scene();


  // Update robot state with the given joint values
  this->set_joint_values_yumi(q_in);


  // Check if a robot is in collision with itself
  this->local_planning_scene_.checkSelfCollision(this->collision_request_, this->collision_result_, this->current_state_, this->acm_); // using local planning scene is faster
  //move_group_planning_scene->checkSelfCollision(this->collision_request_, this->collision_result_, this->current_state_, this->acm_);



  // Display information for debug
  /*
  std::cout << "The robot is " << (this->collision_result_.collision ? "in" : "not in") << " self-collision." << std::endl;  
  std::cout << "Closest distance: " << this->collision_result_.distance << std::endl;
  for (auto it = this->collision_result_.contacts.begin(); it != this->collision_result_.contacts.end(); ++it)
  {
    std::cout << "Contact between: " << it->first.first.c_str() << " and " << it->first.second.c_str() << std::endl;
  }
  */


  // Return the rsult
  double result = (this->collision_result_.collision ? 1 : -1); // 1 for colliding, -1 for free
  this->collision_result_.clear(); // should clear the results, in case the result object is used again!!!
  return result; // set to 1 if in collision
  
}


/* Collision checking with the environment via CollisionWorldFCL::checkRobotCollision */
double DualArmDualHandCollision::check_world_collision(const std::vector<double> q_in)
{

  // Get the PlanningScene maintained by move_group
  planning_scene::PlanningScenePtr move_group_planning_scene = this->get_move_group_planning_scene();


  // Construct CollisionWorldFCL from WorldPtr
  const collision_detection::WorldPtr &world_ptr = move_group_planning_scene->getWorldNonConst();
  collision_detection::CollisionWorldFCL collision_world_fcl(world_ptr);
  collision_detection::CollisionRobotFCL collision_robot_fcl(this->kinematic_model_); 

  // Prepare collision_request
  this->collision_request_.group_name = "";
  this->collision_request_.distance = false; //true; // only for debug, when used with optimization, remember to shut it off!!!! cause great difference in time usage!!!
  this->collision_request_.contacts = true;
  this->collision_request_.max_contacts = 1000;

  this->collision_result_.clear();


  // Update robot state with the given joint values
  this->set_joint_values_yumi(q_in);


  // Check collision with the environment
  collision_world_fcl.checkRobotCollision(this->collision_request_, this->collision_result_, collision_robot_fcl, this->current_state_);


  // Display information for debug
/*
  std::cout << "The robot is " << (this->collision_result_.collision ? "in" : "not in") << " collision with the environment." << std::endl;  
  std::cout << "Closest distance: " << this->collision_result_.distance << std::endl;
  for (auto it = this->collision_result_.contacts.begin(); it != this->collision_result_.contacts.end(); ++it)
  {
    std::cout << "Contact between: " << it->first.first.c_str() << " and " << it->first.second.c_str() << std::endl;
  }
*/

  // Return the rsult
  double result = (this->collision_result_.collision ? 1 : -1); // 1 for colliding, -1 for free
  this->collision_result_.clear(); // should clear the results, in case the result object is used again!!!
  return result; // set to 1 if in collision


}


/* Full collision checking via PlanningScene::checkCollision */
double DualArmDualHandCollision::check_full_collision(const std::vector<double> q_in)
{

  // Get the PlanningScene maintained by move_group
  planning_scene::PlanningScenePtr move_group_planning_scene = this->get_move_group_planning_scene();


  // Prepare collision_request
  this->collision_request_.group_name = "";
  this->collision_request_.distance = false;//true;// only for debug, when used with optimization, remember to shut it off!!!! cause great difference in time usage!!!
  this->collision_request_.contacts = true;
  this->collision_request_.max_contacts = 1000;

  this->collision_result_.clear();


  // Update robot state with the given joint values
  this->set_joint_values_yumi(q_in);


  // Do full collision checking
  move_group_planning_scene->checkCollision(this->collision_request_, this->collision_result_, this->current_state_, this->acm_);


  // Display information for debug
/*
  std::cout << "The robot is " << (this->collision_result_.collision ? "in" : "not in") << " collision." << std::endl;  
  std::cout << "Closest distance: " << this->collision_result_.distance << std::endl;
  for (auto it = this->collision_result_.contacts.begin(); it != this->collision_result_.contacts.end(); ++it)
  {
    std::cout << "Contact between: " << it->first.first.c_str() << " and " << it->first.second.c_str() << std::endl;
  }
*/


  // Get the results
  double result = (this->collision_result_.collision ? 1 : -1); 
  this->collision_result_.clear(); // should clear the results, in case the result object is used again!!!

  return result; // 1 for colliding

}


/* Compute minimum distance to the robot itself */
double DualArmDualHandCollision::compute_self_distance(const std::vector<double> q_in)
{
  
  // Set up a distance request
  this->distance_request_.group_name = ""; 
  this->distance_request_.enable_signed_distance = true;
  this->distance_request_.type = collision_detection::DistanceRequestType::SINGLE; // global minimum
  this->distance_request_.enableGroup(this->kinematic_model_); // specify which group to check
  this->distance_request_.acm = &(this->acm_); // specify acm to ignore adjacent links' collision check
  this->distance_request_.distance_threshold = 0.02;//0.05; // compute only for objects within this threshold to each other

  this->distance_result_.clear();

  // for debug
  bool debug = this->kinematic_model_->hasJointModelGroup("");
  std::cout << "debug: Group \"\" " << (debug ? "does" : "does not") << " exist in kinematic state!" << std::endl;
  std::cout << "debug: Number of active components: " 
            << ((this->distance_request_.active_components_only) ? this->distance_request_.active_components_only->size() : 0) << std::endl;



  // Construct a CollisionRobotFCL for calling distanceSelf function
  collision_detection::CollisionRobotFCL collision_robot_fcl(this->kinematic_model_); // construct collisionrobot from RobotModelConstPtr


  // Update robot state with the given joint values
  this->set_joint_values_yumi(q_in);


  // Compute minimum distance
  collision_robot_fcl.distanceSelf(this->distance_request_, this->distance_result_, this->current_state_);


  // Display debug information
/*
  std::cout << "The robot is " << (this->distance_result_.collision ? "in" : "not in") << " self-collision" << std::endl;  
  std::cout << "Minimum distance is " << this->distance_result_.minimum_distance.distance << ", between " << this->distance_result_.minimum_distance.link_names[0] << " and " << this->distance_result_.minimum_distance.link_names[1] << std::endl;
*/

  // Return result
  double result = this->distance_result_.minimum_distance.distance;
  this->distance_result_.clear(); // should clear the results, in case the result object is used again!!!
  return result;

}


/* Compute minimum distance to the robot itself */
double DualArmDualHandCollision::compute_self_distance_test(const std::vector<double> q_in, std::string group_name, double distance_threshold)
{
  
  // Set up a distance request
  this->distance_request_.group_name = group_name; //"dual_arms"; 
  this->distance_request_.enable_signed_distance = true;
  
  this->distance_request_.compute_gradient = true; // get normalized vector
  this->distance_request_.enable_nearest_points = true; // calculate nearest point information
  this->distance_request_.verbose = true; // Log debug information
  this->distance_request_.max_contacts_per_body = 1000; // ?

  this->distance_request_.type = collision_detection::DistanceRequestType::SINGLE; // global minimum
  this->distance_request_.acm = &(this->acm_); // specify acm to ignore adjacent links' collision check
  this->distance_request_.distance_threshold = distance_threshold;//0.02;//0.05; // compute only for objects within this threshold to each other

  // decide which group to check (set active_components_only)
  // 1 - enableGroup(), includes all the links that get updated when the state of the group is changed, including links that are ***outside*** of this group
  // this->distance_request_.enableGroup(this->kinematic_model_); // specify which group to check
  // 2 - manually set active_components_only, which better satisfies our needs!!!
  const std::vector<const robot_model::LinkModel*>* ac = &(this->kinematic_model_)->getJointModelGroup(group_name)->getLinkModels();
  const std::set<const robot_model::LinkModel*> ac_set(ac->begin(), ac->end()); // from std::vector to std::set
  // const std::set<const robot_model::LinkModel*>* ac_set_p = &ac_set; // get pointer type
  this->distance_request_.active_components_only = &ac_set; // get pointer type


  // Clear distance result for a new query
  this->distance_result_.clear();


  // for debug
  // Note that, enableGroup() assigns to .active_components_only all the links that get updated when the state of the specified group is changed, and thus including links that are ***outside*** of this group!!!
  /*
  bool debug = this->kinematic_model_->hasJointModelGroup(group_name);
  std::cout << "debug: Group " << group_name << " " << (debug ? "does" : "does not") << " exist in kinematic state!" << std::endl;
  std::cout << "debug: Number of active components: " 
            << ((this->distance_request_.active_components_only) ? this->distance_request_.active_components_only->size() : 0) << std::endl;
  if (this->distance_request_.active_components_only)
  {
    std::cout << "debug: active components: " << std::endl;
    auto ac = this->distance_request_.active_components_only;
    for (auto s = ac->begin(); s != ac->end(); s++)
      std::cout << (*s)->getName() << " ";
    std::cout << std::endl;
  }
  */


  // Construct a CollisionRobotFCL for calling distanceSelf function
  collision_detection::CollisionRobotFCL collision_robot_fcl(this->kinematic_model_); // construct collisionrobot from RobotModelConstPtr


  // Update robot state with the given joint values
  this->set_joint_values_yumi(q_in);


  // Compute minimum distance
  collision_robot_fcl.distanceSelf(this->distance_request_, this->distance_result_, this->current_state_);


  // Display debug information
  /*
  std::cout << ">> Debug information: " << std::endl;
  std::cout << "The robot is " << (this->distance_result_.collision ? "in" : "not in") << " self-collision" << std::endl;  
  std::cout << "Minimum distance is " << this->distance_result_.minimum_distance.distance << ", between " << this->distance_result_.minimum_distance.link_names[0] << " and " << this->distance_result_.minimum_distance.link_names[1] << std::endl;
  std::cout << "Closest links are " << this->distance_result_.minimum_distance.link_names[0] << " and " << this->distance_result_.minimum_distance.link_names[1] << std::endl;
  // When in collision, nearest_points[0] and nearest_points[1] are the same.
  std::cout << "Nearest points are p1 = " << this->distance_result_.minimum_distance.nearest_points[0].transpose() 
            << " and p2 = " << this->distance_result_.minimum_distance.nearest_points[1].transpose() << std::endl;
  // a normalized vector pointing from link_names[0] to link_names[1]
  std::cout << "Normal vector is v = " << this->distance_result_.minimum_distance.normal.transpose() << std::endl; 
  */
  
  // Store data for later possible processing
  this->min_distance = this->distance_result_.minimum_distance.distance;
  this->nearest_points[0] = this->distance_result_.minimum_distance.nearest_points[0];
  this->nearest_points[1] = this->distance_result_.minimum_distance.nearest_points[1];
  this->link_names[0] = this->distance_result_.minimum_distance.link_names[0];
  this->link_names[1] = this->distance_result_.minimum_distance.link_names[1];
  this->normal = this->distance_result_.minimum_distance.normal;


  // Return result
  double result = this->distance_result_.minimum_distance.distance;
  this->distance_result_.clear(); // should clear the results, in case the result object is used again!!!
  return result;

}


/* Check out how to add active_components_only that satisfies our needs */
void DualArmDualHandCollision::test_active_components(std::string group_name)
{
  std::cout << ">>>> Test on getting active components (links of a group)" << std::endl;
  const std::set<const robot_model::LinkModel*>* ac1 = &(this->kinematic_model_)->getJointModelGroup(group_name)->getUpdatedLinkModelsSet();
  std::cout << "Results of getUpdatedLinkModelsSet(): " << std::endl;
  for (auto s = ac1->begin(); s != ac1->end(); s++)
    std::cout << (*s)->getName() << " " << std::endl;
  std::cout << std::endl;

  const std::vector<const robot_model::LinkModel*>* ac2 = &(this->kinematic_model_)->getJointModelGroup(group_name)->getLinkModels();
  const std::set<const robot_model::LinkModel*> ac2_set(ac2->begin(), ac2->end()); // from std::vector to std::set
  const std::set<const robot_model::LinkModel*>* ac2_set_p = &ac2_set; // get pointer type (which can be assigned to .active_components_only directly)
  std::cout << "Results of getLinkModels(): " << std::endl;
  for (auto s = ac2_set_p->begin(); s != ac2_set_p->end(); s++)
    std::cout << (*s)->getName() << " " << std::endl;
  std::cout << std::endl;
}


/* Check which part a link belongs to: 0 - left_arm, 1 - right_arm, 2 - left_hand, 3 - right_hand, -1 - others */
int DualArmDualHandCollision::check_link_belonging(std::string link_name)
{
  int group_id = -1;

  if (this->left_arm_group_->hasLinkModel(link_name))
  {
    group_id = 0;
    return group_id;
  }

  if (this->right_arm_group_->hasLinkModel(link_name))
  {
    group_id = 1;
    return group_id;
  }

  if (this->left_hand_group_->hasLinkModel(link_name))
  {
    group_id = 2;
    return group_id;
  }

  if (this->right_hand_group_->hasLinkModel(link_name))
  {
    group_id = 3;
    return group_id;
  }  

  return group_id; // belongs to group outside of left/right arm/hand, possibly body segment or environment obstacles

}


/* Check which finger a link belongs to: 0 - thumb, 1 - index, 2 - middle, 3 - ring, 4 - little, -1 for palm !! */
int DualArmDualHandCollision::check_finger_belonging(std::string link_name, bool left_or_right)
{
  int finger_id;

  if (left_or_right)
  {
    // Must check palm before others, because all finger_groups contain the palm link...
    if (this->left_palm_group_->hasLinkModel(link_name))
    {
      finger_id = -1;
      return finger_id;
    }

    if (this->left_arm_thumb_group_->hasLinkModel(link_name))
    {
      finger_id = 0;
      return finger_id;
    }

    if (this->left_arm_index_group_->hasLinkModel(link_name))
    {
      finger_id = 1;
      return finger_id;
    }

    if (this->left_arm_middle_group_->hasLinkModel(link_name))
    {
      finger_id = 2;
      return finger_id;
    }

    if (this->left_arm_ring_group_->hasLinkModel(link_name))
    {
      finger_id = 3;
      return finger_id;
    }  

    if (this->left_arm_little_group_->hasLinkModel(link_name))
    {
      finger_id = 4;
      return finger_id;
    }      
  }
  else
  {
    // Must check palm before others, because all finger_groups contain the palm link...
    if (this->right_palm_group_->hasLinkModel(link_name))
    {
      finger_id = -1;
      return finger_id;
    }

    if (this->right_arm_thumb_group_->hasLinkModel(link_name))
    {
      finger_id = 0;
      return finger_id;
    }

    if (this->right_arm_index_group_->hasLinkModel(link_name))
    {
      finger_id = 1;
      return finger_id;
    }

    if (this->right_arm_middle_group_->hasLinkModel(link_name))
    {
      finger_id = 2;
      return finger_id;
    }

    if (this->right_arm_ring_group_->hasLinkModel(link_name))
    {
      finger_id = 3;
      return finger_id;
    }  

    if (this->right_arm_little_group_->hasLinkModel(link_name))
    {
      finger_id = 4;
      return finger_id;
    }      
  }

  std::cerr << "Error: something went wrong, the given link does not belong to any finger group !!!" << std::endl;
  exit(-1);
  return -1; 

}


/* Compute minimum distance to the environment */
double DualArmDualHandCollision::compute_world_distance(const std::vector<double> q_in)
{

  // Get the PlanningScene maintained by move_group
  planning_scene::PlanningScenePtr move_group_planning_scene = this->get_move_group_planning_scene();


  // Construct CollisionWorldFCL from WorldPtr
  const collision_detection::WorldPtr &world_ptr = move_group_planning_scene->getWorldNonConst();
  collision_detection::CollisionWorldFCL collision_world_fcl(world_ptr);
  collision_detection::CollisionRobotFCL collision_robot_fcl(this->kinematic_model_); 


  // Set up a distance request
  this->distance_request_.group_name = ""; 
  this->distance_request_.enable_signed_distance = true;
  this->distance_request_.type = collision_detection::DistanceRequestType::SINGLE; // global minimum
  this->distance_request_.enableGroup(this->kinematic_model_); // specify which group to check
  this->distance_request_.acm = &(this->acm_); // specify acm to ignore adjacent links' collision check

  this->distance_result_.clear();


  // Update robot state with the given joint values
  this->set_joint_values_yumi(q_in);


  // Check collision with the environment
  collision_world_fcl.distanceRobot(this->distance_request_, this->distance_result_, collision_robot_fcl, this->current_state_);


  // Display debug information
/*
  std::cout << "The robot is " << (this->distance_result_.collision ? "in" : "not in") << " collision with the world." << std::endl;  
  std::cout << "Minimum distance is " << this->distance_result_.minimum_distance.distance << ", between " << this->distance_result_.minimum_distance.link_names[0] << " and " << this->distance_result_.minimum_distance.link_names[1] << std::endl;
*/

  // Return result
  double result = this->distance_result_.minimum_distance.distance;
  this->distance_result_.clear(); // should clear the results, in case the result object is used again!!!
  return result;

}


std::stringstream read_file(std::string file_name)
{
  std::ifstream ifs(file_name);
  std::stringstream ss;
  ss << ifs.rdbuf();
  return ss;
}




/* Testing the APIs */
int main(int argc, char **argv)
{

  // Get everything output by std::cout in a log file
  //std::ofstream logFile("log.txt");
  //std::streambuf *outbuf = std::cout.rdbuf(logFile.rdbuf());


  // Initialize a ROS node (or creating NodeHandle later)
  ros::init(argc, argv, "TEST_collision_checking_yumi");

  double result;


  // Preparations (structure: left_arm, right_arm, left_hand, right_hand)
  std::vector<double> colliding_joint_values(38); // all 0's
  colliding_joint_values[0] = -0.91;
  colliding_joint_values[1] = 0.44;
  colliding_joint_values[2] = 0.03;
  colliding_joint_values[3] = 0.83; // left arm colliding with right arm
  /*colliding_joint_values[14] = -0.88;
  colliding_joint_values[15] = -0.98;
  colliding_joint_values[22] = -0.66; // left hand group self-collision */

  std::vector<double> noncolliding_joint_values(38);
  noncolliding_joint_values[0] = 1.09; 


  // Set up an object of the class
  std::string urdf_file_name = "/home/liangyuwei/sign_language_robot_ws/src/yumi_description/urdf/yumi_with_hands.urdf";
  std::string srdf_file_name = "/home/liangyuwei/sign_language_robot_ws/src/yumi_sign_language_robot_moveit_config/config/yumi.srdf";
  std::stringstream urdf_string = read_file(urdf_file_name);
  std::stringstream srdf_string = read_file(srdf_file_name);

  boost::shared_ptr<DualArmDualHandCollision> dual_arm_dual_hand_collision_ptr;  
  dual_arm_dual_hand_collision_ptr.reset( new DualArmDualHandCollision(urdf_string.str(), srdf_string.str()) );


  // Record time
  std::chrono::steady_clock::time_point t0, t1;
  std::chrono::duration<double> t0_1;


  // Self-collision checking
  std::cout << ">>>> Self-collision Checking (Test 1) <<<<" << std::endl;
  t0 = std::chrono::steady_clock::now();
  result = dual_arm_dual_hand_collision_ptr->check_self_collision(colliding_joint_values);
  t1 = std::chrono::steady_clock::now();
  t0_1 = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  std::cout << "Result is: " << result << std::endl;
  std::cout << "The robot under self-colliding state is " << (result == 1 ? "in" : "not in") << " self-collision." << std::endl;
  std::cout << "Time used: " << t0_1.count() << " s." << std::endl;


  std::cout << ">>>> Self-collision Checking (Test 2) <<<<" << std::endl;
  t0 = std::chrono::steady_clock::now();
  result = dual_arm_dual_hand_collision_ptr->check_self_collision(noncolliding_joint_values);
  t1 = std::chrono::steady_clock::now();
  t0_1 = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  std::cout << "The robot under collision-free state is " << (result == 1.0 ? "in" : "not in") << " self-collision." << std::endl;
  std::cout << "Time used: " << t0_1.count() << " s." << std::endl;


  // Collision checking with the environment
  std::cout << ">>>> Collision Checking with the environment (Test 1) <<<<" << std::endl;
  t0 = std::chrono::steady_clock::now();
  result = dual_arm_dual_hand_collision_ptr->check_world_collision(colliding_joint_values);
  t1 = std::chrono::steady_clock::now();
  t0_1 = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  std::cout << "The robot under self-colliding state(no collision objects) is " << (result == 1.0 ? "in" : "not in") << " collision with the environment." << std::endl;
  std::cout << "Time used: " << t0_1.count() << " s." << std::endl;


  std::cout << ">>>> Collision Checking with the environment (Test 2) <<<<" << std::endl;
  dual_arm_dual_hand_collision_ptr->apply_collision_objects();
  t0 = std::chrono::steady_clock::now();
  result = dual_arm_dual_hand_collision_ptr->check_world_collision(colliding_joint_values);
  t1 = std::chrono::steady_clock::now();
  t0_1 = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  std::cout << "The robot under self-colliding state(with a collision object) is " << (result == 1.0 ? "in" : "not in") << " collision with the environment." << std::endl;
  dual_arm_dual_hand_collision_ptr->remove_collision_objects();
  std::cout << "Time used: " << t0_1.count() << " s." << std::endl;


  std::cout << ">>>> Collision Checking with the environment (Test 3) <<<<" << std::endl;
  dual_arm_dual_hand_collision_ptr->apply_collision_objects();
  t0 = std::chrono::steady_clock::now();
  result = dual_arm_dual_hand_collision_ptr->check_world_collision(noncolliding_joint_values);
  t1 = std::chrono::steady_clock::now();
  t0_1 = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  std::cout << "The robot under self-collision-free state(with a collision object) is " << (result == 1.0 ? "in" : "not in") << " collision with the environment." << std::endl;
  dual_arm_dual_hand_collision_ptr->remove_collision_objects();
  std::cout << "Time used: " << t0_1.count() << " s." << std::endl;


  // Full collision checking
  std::cout << ">>>> Full Collision Checking (Test 1) <<<<" << std::endl;
  t0 = std::chrono::steady_clock::now();
  result = dual_arm_dual_hand_collision_ptr->check_full_collision(colliding_joint_values);
  t1 = std::chrono::steady_clock::now();
  t0_1 = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  std::cout << "The robot under self-colliding state is " << (result == 1.0 ? "in" : "not in") << " collision." << std::endl;
  std::cout << "Time used: " << t0_1.count() << " s." << std::endl;


  std::cout << ">>>> Full Collision Checking (Test 2) <<<<" << std::endl;
  t0 = std::chrono::steady_clock::now();
  result = dual_arm_dual_hand_collision_ptr->check_full_collision(noncolliding_joint_values);
  t1 = std::chrono::steady_clock::now();
  t0_1 = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  std::cout << "The robot under collision-free state(no collision object) is " << (result == 1.0 ? "in" : "not in") << " collision." << std::endl;
  std::cout << "Time used: " << t0_1.count() << " s." << std::endl;


  std::cout << ">>>> Full Collision Checking (Test 3) <<<<" << std::endl;
  dual_arm_dual_hand_collision_ptr->apply_collision_objects();
  t0 = std::chrono::steady_clock::now();
  result = dual_arm_dual_hand_collision_ptr->check_full_collision(noncolliding_joint_values);
  t1 = std::chrono::steady_clock::now();
  t0_1 = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  std::cout << "The robot under collision-free state (with a collision object) is " << (result == 1.0 ? "in" : "not in") << " collision." << std::endl;
  dual_arm_dual_hand_collision_ptr->remove_collision_objects();
  std::cout << "Time used: " << t0_1.count() << " s." << std::endl;


  // Self-distance computation
  std::cout << ">>>> Distance computation (Self) (Test 1) <<<<" << std::endl;
  t0 = std::chrono::steady_clock::now();
  result = dual_arm_dual_hand_collision_ptr->compute_self_distance(colliding_joint_values);
  std::cout << "Minimum distance of the robot under self-colliding state is " << result << std::endl;
  t1 = std::chrono::steady_clock::now();
  t0_1 = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  std::cout << "Time used: " << t0_1.count() << " s." << std::endl;


  std::cout << ">>>> Distance computation (Self) (Test 2) <<<<" << std::endl;
  t0 = std::chrono::steady_clock::now();
  result = dual_arm_dual_hand_collision_ptr->compute_self_distance(noncolliding_joint_values);
  t1 = std::chrono::steady_clock::now();
  t0_1 = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  std::cout << "Minimum distance of the robot under self-collision-free state is " << result << std::endl;
  std::cout << "Time used: " << t0_1.count() << " s." << std::endl;


  // Distance to the collision objects
  std::cout << ">>>> Distance computation (With the Environment) (Test 1) <<<<" << std::endl;
  dual_arm_dual_hand_collision_ptr->apply_collision_objects();
  t0 = std::chrono::steady_clock::now();
  result = dual_arm_dual_hand_collision_ptr->compute_world_distance(noncolliding_joint_values);
  t1 = std::chrono::steady_clock::now();
  t0_1 = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  std::cout << "Minimum distance of the robot under self-colliding-free state(with a collision object) is " << result << std::endl;
  dual_arm_dual_hand_collision_ptr->remove_collision_objects();
  std::cout << "Time used: " << t0_1.count() << " s." << std::endl;


  std::cout << ">>>> Distance computation (With the Environment) (Test 2) <<<<" << std::endl;
  t0 = std::chrono::steady_clock::now();
  result = dual_arm_dual_hand_collision_ptr->compute_world_distance(noncolliding_joint_values);
  t1 = std::chrono::steady_clock::now();
  t0_1 = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  std::cout << "Minimum distance of the robot under self-colliding-free state(no collision objects) is " << result << std::endl;
  std::cout << "Time used: " << t0_1.count() << " s." << std::endl << std::endl;


  // Full distance computation
  // no such API to our knowledge for now


  // Close the output
  //std::cout.rdbuf(outbuf);


  // Check out partial collision checking
  std::cout << ">>>> NEW TESTS: partial collision checking <<<<" << std::endl 
            << "8 Conditions in total: (Arms self-collision or not) X (Hands self-collision or not) X (Arm-Hand in collision or not)." << std::endl;
  // 1
  std::cout << ">> 1. (Arm Col, Hand Col, Arm-Hand Col) = (Y, N, N): " << std::endl;
  std::vector<double> q_arm_col_hand_noncol_armhand_noncol(38);  
  q_arm_col_hand_noncol_armhand_noncol[0] = -0.23;
  q_arm_col_hand_noncol_armhand_noncol[1] = 0.25;
  q_arm_col_hand_noncol_armhand_noncol[2] = -0.2;
  double result1 = dual_arm_dual_hand_collision_ptr->check_arm_self_collision(q_arm_col_hand_noncol_armhand_noncol);
  double result2 = dual_arm_dual_hand_collision_ptr->check_hand_self_collision(q_arm_col_hand_noncol_armhand_noncol);
  std::cout << "Arms in collision: " << (result1 > 0 ? " yes" : "no") << std::endl;
  std::cout << "Hands in collision: " << (result2 > 0 ? " yes" : "no") << std::endl;
  t0 = std::chrono::steady_clock::now();
  double result3 = dual_arm_dual_hand_collision_ptr->compute_self_distance(q_arm_col_hand_noncol_armhand_noncol);
  t1 = std::chrono::steady_clock::now();
  t0_1 = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  std::cout << "Minimum distance = " << result3 << std::endl;
  std::cout << "Time used for computing distance: " << t0_1.count() << std::endl;
  // 2
  std::cout << ">> 2. (Arm Col, Hand Col, Arm-Hand Col) = (Y, N, Y): " << std::endl;
  std::vector<double> q_arm_col_hand_noncol_armhand_col(38);  
  q_arm_col_hand_noncol_armhand_col[0] = -0.05;
  q_arm_col_hand_noncol_armhand_col[1] = -0.09;
  q_arm_col_hand_noncol_armhand_col[2] = -0.2;
  q_arm_col_hand_noncol_armhand_col[3] = 0.22;
  result1 = dual_arm_dual_hand_collision_ptr->check_arm_self_collision(q_arm_col_hand_noncol_armhand_col);
  result2 = dual_arm_dual_hand_collision_ptr->check_hand_self_collision(q_arm_col_hand_noncol_armhand_col);
  std::cout << "Arms in collision: " << (result1 > 0 ? " yes" : "no") << std::endl;
  std::cout << "Hands in collision: " << (result2 > 0 ? " yes" : "no") << std::endl;  
  t0 = std::chrono::steady_clock::now();
  result3 = dual_arm_dual_hand_collision_ptr->compute_self_distance(q_arm_col_hand_noncol_armhand_col);
  t1 = std::chrono::steady_clock::now();
  t0_1 = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  std::cout << "Minimum distance = " << result3 << std::endl;
  std::cout << "Time used for computing distance: " << t0_1.count() << std::endl;  
  // 3
  std::cout << ">> 3. (Arm Col, Hand Col, Arm-Hand Col) = (Y, Y, Y): " << std::endl;
  std::vector<double> q_arm_col_hand_col_armhand_col(38);  
  q_arm_col_hand_col_armhand_col[0] = -0.05;
  q_arm_col_hand_col_armhand_col[1] = -0.09;
  q_arm_col_hand_col_armhand_col[2] = -0.2;
  q_arm_col_hand_col_armhand_col[3] = 0.22;
  q_arm_col_hand_col_armhand_col[26] = -0.95;
  q_arm_col_hand_col_armhand_col[35] = 0.4;
  q_arm_col_hand_col_armhand_col[36] = -0.4;
  result1 = dual_arm_dual_hand_collision_ptr->check_arm_self_collision(q_arm_col_hand_col_armhand_col);
  result2 = dual_arm_dual_hand_collision_ptr->check_hand_self_collision(q_arm_col_hand_col_armhand_col);
  std::cout << "Arms in collision: " << (result1 > 0 ? " yes" : "no") << std::endl;
  std::cout << "Hands in collision: " << (result2 > 0 ? " yes" : "no") << std::endl; 
  t0 = std::chrono::steady_clock::now();
  result3 = dual_arm_dual_hand_collision_ptr->compute_self_distance(q_arm_col_hand_col_armhand_col);
  t1 = std::chrono::steady_clock::now();
  t0_1 = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  std::cout << "Minimum distance = " << result3 << std::endl;
  std::cout << "Time used for computing distance: " << t0_1.count() << std::endl;   
  // 4
  std::cout << ">> 4. (Arm Col, Hand Col, Arm-Hand Col) = (Y, Y, N): " << std::endl;
  std::vector<double> q_arm_col_hand_col_armhand_noncol(38);  
  q_arm_col_hand_col_armhand_noncol[0] = -0.05;
  q_arm_col_hand_col_armhand_noncol[1] = 0.22;
  q_arm_col_hand_col_armhand_noncol[2] = -0.23;
  q_arm_col_hand_col_armhand_noncol[3] = -0.15;
  q_arm_col_hand_col_armhand_noncol[26] = -0.95;
  q_arm_col_hand_col_armhand_noncol[35] = 0.4;
  q_arm_col_hand_col_armhand_noncol[36] = -0.4;
  result1 = dual_arm_dual_hand_collision_ptr->check_arm_self_collision(q_arm_col_hand_col_armhand_noncol);
  result2 = dual_arm_dual_hand_collision_ptr->check_hand_self_collision(q_arm_col_hand_col_armhand_noncol);
  std::cout << "Arms in collision: " << (result1 > 0 ? " yes" : "no") << std::endl;
  std::cout << "Hands in collision: " << (result2 > 0 ? " yes" : "no") << std::endl; 
  t0 = std::chrono::steady_clock::now();
  result3 = dual_arm_dual_hand_collision_ptr->compute_self_distance(q_arm_col_hand_col_armhand_noncol);
  t1 = std::chrono::steady_clock::now();
  t0_1 = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  std::cout << "Minimum distance = " << result3 << std::endl;
  std::cout << "Time used for computing distance: " << t0_1.count() << std::endl;     
  // 5
  std::cout << ">> 5. (Arm Col, Hand Col, Arm-Hand Col) = (N, Y, Y): " << std::endl;
  std::vector<double> q_arm_noncol_hand_col_armhand_col(38);  
  q_arm_noncol_hand_col_armhand_col[0] = -0.17;
  q_arm_noncol_hand_col_armhand_col[1] = -0.41;
  q_arm_noncol_hand_col_armhand_col[2] = -0.44;
  q_arm_noncol_hand_col_armhand_col[3] = 0.68;
  q_arm_noncol_hand_col_armhand_col[26] = -0.95;
  q_arm_noncol_hand_col_armhand_col[35] = 0.4;
  q_arm_noncol_hand_col_armhand_col[36] = -0.4;
  result1 = dual_arm_dual_hand_collision_ptr->check_arm_self_collision(q_arm_noncol_hand_col_armhand_col);
  result2 = dual_arm_dual_hand_collision_ptr->check_hand_self_collision(q_arm_noncol_hand_col_armhand_col);
  std::cout << "Arms in collision: " << (result1 > 0 ? " yes" : "no") << std::endl;
  std::cout << "Hands in collision: " << (result2 > 0 ? " yes" : "no") << std::endl; 
  t0 = std::chrono::steady_clock::now();
  result3 = dual_arm_dual_hand_collision_ptr->compute_self_distance(q_arm_noncol_hand_col_armhand_col);
  t1 = std::chrono::steady_clock::now();
  t0_1 = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  std::cout << "Minimum distance = " << result3 << std::endl;
  std::cout << "Time used for computing distance: " << t0_1.count() << std::endl;      
  // 6
  std::cout << ">> 6. (Arm Col, Hand Col, Arm-Hand Col) = (N, Y, N): " << std::endl;
  std::vector<double> q_arm_noncol_hand_col_armhand_noncol(38);  
  q_arm_noncol_hand_col_armhand_noncol[0] = 0.96;
  q_arm_noncol_hand_col_armhand_noncol[1] = -0.42;
  q_arm_noncol_hand_col_armhand_noncol[2] = -0.29;
  q_arm_noncol_hand_col_armhand_noncol[3] = 0.59;
  q_arm_noncol_hand_col_armhand_noncol[26] = -0.95;
  q_arm_noncol_hand_col_armhand_noncol[35] = 0.4;
  q_arm_noncol_hand_col_armhand_noncol[36] = -0.4;
  result1 = dual_arm_dual_hand_collision_ptr->check_arm_self_collision(q_arm_noncol_hand_col_armhand_noncol);
  result2 = dual_arm_dual_hand_collision_ptr->check_hand_self_collision(q_arm_noncol_hand_col_armhand_noncol);
  std::cout << "Arms in collision: " << (result1 > 0 ? " yes" : "no") << std::endl;
  std::cout << "Hands in collision: " << (result2 > 0 ? " yes" : "no") << std::endl; 
  t0 = std::chrono::steady_clock::now();
  result3 = dual_arm_dual_hand_collision_ptr->compute_self_distance(q_arm_noncol_hand_col_armhand_noncol);
  t1 = std::chrono::steady_clock::now();
  t0_1 = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  std::cout << "Minimum distance = " << result3 << std::endl;
  std::cout << "Time used for computing distance: " << t0_1.count() << std::endl;      
  // 7
  std::cout << ">> 7. (Arm Col, Hand Col, Arm-Hand Col) = (N, N, Y): " << std::endl;
  std::vector<double> q_arm_noncol_hand_noncol_armhand_col(38);  
  q_arm_noncol_hand_noncol_armhand_col[0] = -0.23;
  q_arm_noncol_hand_noncol_armhand_col[1] = -0.42;
  q_arm_noncol_hand_noncol_armhand_col[2] = -0.29;
  q_arm_noncol_hand_noncol_armhand_col[3] = 0.59;
  result1 = dual_arm_dual_hand_collision_ptr->check_arm_self_collision(q_arm_noncol_hand_noncol_armhand_col);
  result2 = dual_arm_dual_hand_collision_ptr->check_hand_self_collision(q_arm_noncol_hand_noncol_armhand_col);
  std::cout << "Arms in collision: " << (result1 > 0 ? " yes" : "no") << std::endl;
  std::cout << "Hands in collision: " << (result2 > 0 ? " yes" : "no") << std::endl; 
  t0 = std::chrono::steady_clock::now();
  result3 = dual_arm_dual_hand_collision_ptr->compute_self_distance(q_arm_noncol_hand_noncol_armhand_col);
  t1 = std::chrono::steady_clock::now();
  t0_1 = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  std::cout << "Minimum distance = " << result3 << std::endl;
  std::cout << "Time used for computing distance: " << t0_1.count() << std::endl;       
  // 8
  std::cout << ">> 8. (Arm Col, Hand Col, Arm-Hand Col) = (N, N, N): " << std::endl;
  std::vector<double> q_arm_noncol_hand_noncol_armhand_noncol(38);  
  q_arm_noncol_hand_noncol_armhand_noncol[0] = 0.75;
  result1 = dual_arm_dual_hand_collision_ptr->check_arm_self_collision(q_arm_noncol_hand_noncol_armhand_noncol);
  result2 = dual_arm_dual_hand_collision_ptr->check_hand_self_collision(q_arm_noncol_hand_noncol_armhand_noncol);
  std::cout << "Arms in collision: " << (result1 > 0 ? " yes" : "no") << std::endl;
  std::cout << "Hands in collision: " << (result2 > 0 ? " yes" : "no") << std::endl; 
  t0 = std::chrono::steady_clock::now();
  result3 = dual_arm_dual_hand_collision_ptr->compute_self_distance(q_arm_noncol_hand_noncol_armhand_noncol);
  t1 = std::chrono::steady_clock::now();
  t0_1 = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  std::cout << "Minimum distance = " << result3 << std::endl;
  std::cout << "Time used for computing distance: " << t0_1.count() << std::endl << std::endl;      


  // Closest contact points and gradient
  std::cout << ">>>> Checking closest points and gradient information <<<<" << std::endl;
  // 1
  std::cout << ">> 1. (Arm Col, Hand Col, Arm-Hand Col) = (Y, N, N): " << std::endl;
  std::vector<double> q_arm_col_hand_noncol_armhand_noncol_1(38);      
  q_arm_col_hand_noncol_armhand_noncol_1[0] = -0.23;
  q_arm_col_hand_noncol_armhand_noncol_1[1] = 0.49;//0.25;
  q_arm_col_hand_noncol_armhand_noncol_1[2] = -0.2;
  t0 = std::chrono::steady_clock::now();
  result3 = dual_arm_dual_hand_collision_ptr->compute_self_distance_test(q_arm_col_hand_noncol_armhand_noncol_1, "dual_arms", 0.02);
  t1 = std::chrono::steady_clock::now();
  t0_1 = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  std::cout << "Minimum distance = " << result3 << std::endl;
  std::cout << "Nearest links are " << dual_arm_dual_hand_collision_ptr->link_names[0] << " and " 
                                    << dual_arm_dual_hand_collision_ptr->link_names[1] << ", " << std::endl
                                    << "with p1 = " << dual_arm_dual_hand_collision_ptr->nearest_points[0].transpose() << " and "
                                    << "p2 = " << dual_arm_dual_hand_collision_ptr->nearest_points[1].transpose() << std::endl;
  // Test on getting global transform under current configuration/state
  dual_arm_dual_hand_collision_ptr->get_global_link_transform(dual_arm_dual_hand_collision_ptr->link_names[0]);
  dual_arm_dual_hand_collision_ptr->get_global_link_transform(dual_arm_dual_hand_collision_ptr->link_names[1]);
  std::cout << "Time used for computing distance: " << t0_1.count() << std::endl;

  std::vector<double> q_arm_col_hand_noncol_armhand_noncol_2(38);      
  q_arm_col_hand_noncol_armhand_noncol_2[0] = -0.23;
  q_arm_col_hand_noncol_armhand_noncol_2[1] = -0.15;//0.49;//0.25;
  q_arm_col_hand_noncol_armhand_noncol_2[2] = -0.2;
  t0 = std::chrono::steady_clock::now();
  result3 = dual_arm_dual_hand_collision_ptr->compute_self_distance_test(q_arm_col_hand_noncol_armhand_noncol_2, "dual_arms", 0.02);
  t1 = std::chrono::steady_clock::now();
  t0_1 = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  std::cout << "Minimum distance = " << result3 << std::endl;
  std::cout << "Time used for computing distance: " << t0_1.count() << std::endl;

  // 2
  std::cout << ">> 6. (Arm Col, Hand Col, Arm-Hand Col) = (N, Y, N): " << std::endl;
  q_arm_noncol_hand_col_armhand_noncol[0] = 0.96;
  q_arm_noncol_hand_col_armhand_noncol[1] = -0.42;
  q_arm_noncol_hand_col_armhand_noncol[2] = -0.29;
  q_arm_noncol_hand_col_armhand_noncol[3] = 0.59;
  q_arm_noncol_hand_col_armhand_noncol[26] = -0.4;//-0.95;
  q_arm_noncol_hand_col_armhand_noncol[35] = 0.4;
  q_arm_noncol_hand_col_armhand_noncol[36] = -0.4;
  t0 = std::chrono::steady_clock::now();
  result3 = dual_arm_dual_hand_collision_ptr->compute_self_distance_test(q_arm_noncol_hand_col_armhand_noncol, "dual_hands", 0.02);
  t1 = std::chrono::steady_clock::now();
  t0_1 = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  std::cout << "Minimum distance = " << result3 << std::endl;
  std::cout << "Time used for computing distance: " << t0_1.count() << std::endl;     

  q_arm_noncol_hand_col_armhand_noncol[0] = 0.96;
  q_arm_noncol_hand_col_armhand_noncol[1] = -0.42;
  q_arm_noncol_hand_col_armhand_noncol[2] = -0.29;
  q_arm_noncol_hand_col_armhand_noncol[3] = 0.59;
  q_arm_noncol_hand_col_armhand_noncol[26] = -1.38;//-0.4;//-0.95;
  q_arm_noncol_hand_col_armhand_noncol[35] = 0.32;//0.4;
  q_arm_noncol_hand_col_armhand_noncol[36] = -0.4;
  t0 = std::chrono::steady_clock::now();
  result3 = dual_arm_dual_hand_collision_ptr->compute_self_distance_test(q_arm_noncol_hand_col_armhand_noncol, "dual_hands", 0.02);
  t1 = std::chrono::steady_clock::now();
  t0_1 = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  std::cout << "Minimum distance = " << result3 << std::endl;
  std::cout << "Time used for computing distance: " << t0_1.count() << std::endl;  

  // 3
  std::cout << ">> 8. (Arm Col, Hand Col, Arm-Hand Col) = (N, N, N): " << std::endl;
  q_arm_noncol_hand_noncol_armhand_noncol[0] = 0.75;
  std::cout << ">> Trial 1 on dual_arms group: " << std::endl;
  t0 = std::chrono::steady_clock::now();
  result3 = dual_arm_dual_hand_collision_ptr->compute_self_distance_test(q_arm_noncol_hand_noncol_armhand_noncol, "dual_arms", 0.02);
  t1 = std::chrono::steady_clock::now();
  t0_1 = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  std::cout << "Minimum distance = " << result3 << std::endl;
  std::cout << "Time used for computing distance: " << t0_1.count() << std::endl << std::endl;   

  std::cout << ">> Trial 2 on dual_hands group: " << std::endl;
  t0 = std::chrono::steady_clock::now();
  result3 = dual_arm_dual_hand_collision_ptr->compute_self_distance_test(q_arm_noncol_hand_noncol_armhand_noncol, "dual_hands", 0.02);
  t1 = std::chrono::steady_clock::now();
  t0_1 = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  std::cout << "Minimum distance = " << result3 << std::endl;
  std::cout << "Time used for computing distance: " << t0_1.count() << std::endl << std::endl;   

  // 4 (arm collides with hand)
  std::cout << ">> 7. (Arm Col, Hand Col, Arm-Hand Col) = (N, N, Y): " << std::endl;
  q_arm_noncol_hand_noncol_armhand_col[0] = -0.23;
  q_arm_noncol_hand_noncol_armhand_col[1] = -0.42;
  q_arm_noncol_hand_noncol_armhand_col[2] = -0.29;
  q_arm_noncol_hand_noncol_armhand_col[3] = 0.59;
  std::cout << ">> Trial 1 on dual_arms group: " << std::endl;
  t0 = std::chrono::steady_clock::now();
  result3 = dual_arm_dual_hand_collision_ptr->compute_self_distance_test(q_arm_noncol_hand_noncol_armhand_col, "dual_arms", 0.02);
  t1 = std::chrono::steady_clock::now();
  t0_1 = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  std::cout << "Minimum distance = " << result3 << std::endl;
  std::cout << "Time used for computing distance: " << t0_1.count() << std::endl;   

  std::cout << ">> Trial 2 on dual_hands group: " << std::endl;
  t0 = std::chrono::steady_clock::now();
  result3 = dual_arm_dual_hand_collision_ptr->compute_self_distance_test(q_arm_noncol_hand_noncol_armhand_col, "dual_hands", 0.02);
  t1 = std::chrono::steady_clock::now();
  t0_1 = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  std::cout << "Minimum distance = " << result3 << std::endl;
  std::cout << "Time used for computing distance: " << t0_1.count() << std::endl;  

  // Test getting robot jacobian
  Eigen::MatrixXd robot_jacobian;
  std::cout << ">>>> Test on getting robot jacobian" << std::endl;
  bool arm_hand_together, arm_or_hand, left_or_right;
  // 1 - note that if given link is not controlled by any joint, the corresponding jacobian would be zero!!! no need to worry!!!
  arm_hand_together = false; 
  arm_or_hand = true;
  left_or_right = true;
  t0 = std::chrono::steady_clock::now();
  robot_jacobian = dual_arm_dual_hand_collision_ptr->get_robot_arm_jacobian("yumi_link_1_l", Eigen::Vector3d::Zero(), left_or_right);
  t1 = std::chrono::steady_clock::now();
  t0_1 = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  std::cout << ">> Left Arm, yumi_link_1_l" << std::endl << "jacobian = " << robot_jacobian << std::endl;
  std::cout << "Time used for computing robot jacobian: " << t0_1.count() << std::endl;
  // 2
  arm_hand_together = false; 
  arm_or_hand = true;
  left_or_right = false;
  t0 = std::chrono::steady_clock::now();
  robot_jacobian = dual_arm_dual_hand_collision_ptr->get_robot_arm_jacobian("yumi_link_5_r", Eigen::Vector3d::Zero(), left_or_right);
  t1 = std::chrono::steady_clock::now();
  t0_1 = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  std::cout << ">> Right Arm, yumi_link_5_r" << std::endl << "jacobian = " << robot_jacobian << std::endl;
  std::cout << "Time used for computing robot jacobian: " << t0_1.count() << std::endl;
  // 3 
  arm_hand_together = false; 
  arm_or_hand = false;
  left_or_right = true;
  int finger_id = 1;
  t0 = std::chrono::steady_clock::now();
  robot_jacobian = dual_arm_dual_hand_collision_ptr->get_robot_hand_jacobian("link11", Eigen::Vector3d::Zero(), finger_id, left_or_right);
  t1 = std::chrono::steady_clock::now();
  t0_1 = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  std::cout << ">> Left Hand, link11" << std::endl << "jacobian = " << robot_jacobian << std::endl;
  std::cout << "Time used for computing robot jacobian: " << t0_1.count() << std::endl;
  // 4 
  arm_hand_together = false; 
  arm_or_hand = false;
  left_or_right = false;
  finger_id = 0;
  t0 = std::chrono::steady_clock::now();
  robot_jacobian = dual_arm_dual_hand_collision_ptr->get_robot_hand_jacobian("Link53", Eigen::Vector3d::Zero(), finger_id, left_or_right);
  t1 = std::chrono::steady_clock::now();
  t0_1 = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  std::cout << ">> Right Hand, Link53" << std::endl << "jacobian = " << robot_jacobian << std::endl;  
  std::cout << "Time used for computing robot jacobian: " << t0_1.count() << std::endl;
  // 5 
  arm_hand_together = true; 
  left_or_right = true;
  finger_id = 0;
  t0 = std::chrono::steady_clock::now();
  robot_jacobian = dual_arm_dual_hand_collision_ptr->get_robot_arm_hand_jacobian("link53", Eigen::Vector3d::Zero(), finger_id, left_or_right);
  t1 = std::chrono::steady_clock::now();
  t0_1 = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  std::cout << ">> Left Arm + Hand, link53" << std::endl << "jacobian = " << robot_jacobian << std::endl;  
  std::cout << "Time used for computing robot jacobian: " << t0_1.count() << std::endl;
  // 6 
  arm_hand_together = true; 
  left_or_right = false;
  finger_id = 0;
  t0 = std::chrono::steady_clock::now();
  robot_jacobian = dual_arm_dual_hand_collision_ptr->get_robot_arm_hand_jacobian("Link111", Eigen::Vector3d::Zero(), finger_id, left_or_right);
  t1 = std::chrono::steady_clock::now();
  t0_1 = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  std::cout << ">> Right Arm + Hand, Link111" << std::endl << "jacobian = " << robot_jacobian << std::endl;  
  std::cout << "Time used for computing robot jacobian: " << t0_1.count() << std::endl << std::endl;


  // Test getting active components
  // active_components_only is std::set<LinkModel*>.
  // getUpdatedLinkModelsSet() returns std::set, while getLinkModels() returns std::vector
  // getUpdatedLinkModelsSet() would return links that would be transformed when the state of the specified joint group is changed, and therefore may include links that are ***outside*** of the specified joint group!
  // getLinkModels() returns only the links that are part of the group!
  dual_arm_dual_hand_collision_ptr->test_active_components("dual_arms");


  // debug: link111 does not exist in the chain 'left_thumb' or is not a child for this chain
  // left
  left_or_right = true;
  finger_id = -1;// - palm //0;
  Eigen::Vector3d ref_pos = {-0.00126508, -0.0257741, -0.00840286};
  robot_jacobian = dual_arm_dual_hand_collision_ptr->get_robot_hand_jacobian("link111", Eigen::Vector3d::Zero(), finger_id, left_or_right);
  std::cout << "debug: jacobian of link111 = \n" << robot_jacobian << std::endl;
  // right
  left_or_right = false;
  finger_id = -1;// - palm // 0;  
  robot_jacobian = dual_arm_dual_hand_collision_ptr->get_robot_hand_jacobian("Link111", Eigen::Vector3d::Zero(), finger_id, left_or_right);
  std::cout << "debug: jacobian of Link111 = \n" << robot_jacobian << std::endl;




  return 0;

}


