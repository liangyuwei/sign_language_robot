#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
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


// User defined constraints can also be specified to the PlanningScene
/*
bool stateFeasibilityTestExample(const robot_state::RobotState& kinematic_state, bool verbose)
{
  const double* joint_values = kinematic_state.getJointPositions("panda_joint1");
  return (joint_values[0] > 0.0);
}
*/

int main(int argc, char** argv)
{

  // Initialize a ROS node
  ros::init(argc, argv, "sign_language_robot_collision_computation");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::size_t count = 0;

  ros::NodeHandle node_handle;
 

  // Set up PlanningScene class(from RobotModel or URDF & SRDF; see the constructor in documentation)
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);


  // Collision Checking
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  planning_scene.checkSelfCollision(collision_request, collision_result);
  ROS_INFO_STREAM("Test 1: Current state of the whole robot is " << (collision_result.collision ? "in" : "not in") << " self collision");


  // Get current_state for later use
  robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst();


  // Self-collision for a group(In collision), and get contact information
  collision_result.clear();
  std::vector<double> joint_values = {-1.0, -1.47, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.62, 0.0, -0.56, -0.3 };
  // {-1.0, -1.47, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.62, 0.0, -0.56, -0.3 } -- one self-collision configuration for left_hand
  const robot_model::JointModelGroup* joint_model_group = current_state.getJointModelGroup("left_hand");
  current_state.setJointGroupPositions(joint_model_group, joint_values);
  ROS_INFO_STREAM(">>>> Test 1 on self-collision checking(In collision): ");
  ROS_INFO_STREAM("Manually set state of the left hand is " << (current_state.satisfiesBounds(joint_model_group) ? "within joint limits" : "beyond joint limits"));

  collision_request.contacts = true;
  collision_request.max_contacts = 1000;

  collision_request.distance = true;  // set to compute distance
  planning_scene.checkSelfCollision(collision_request, collision_result);
  ROS_INFO_STREAM("Manually set state of the left hand is " << (collision_result.collision ? "in" : "not in") << " self collision");
  collision_detection::CollisionResult::ContactMap::const_iterator it;
  for (it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it)
  {
    ROS_INFO("Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());
  }
  ROS_INFO_STREAM("Closest distance is " << collision_result.distance);


  // Self-collision for a group(No collision)
  collision_result.clear();
  collision_request.group_name = "right_hand";
  joint_values = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  const robot_model::JointModelGroup* joint_model_group2 = current_state.getJointModelGroup("right_hand");
  current_state.setJointGroupPositions(joint_model_group2, joint_values);
  ROS_INFO_STREAM(">>>> Test 2 on self-collision checking(No collision):");
  ROS_INFO_STREAM("Manually set state of the right hand is " << (current_state.satisfiesBounds(joint_model_group2) ? "within joint limits" : "beyond joint limits"));

  collision_request.distance = true;  // set to compute distance
  planning_scene.checkSelfCollision(collision_request, collision_result);
  ROS_INFO_STREAM("Manually set state of the right hand is " << (collision_result.collision ? "in" : "not in") << " self collision");

  ROS_INFO_STREAM("Closest distance: " << collision_result.distance);


  // Self-collision for a group; Use DistanceRequest to compute penetration
  auto distance_request = collision_detection::DistanceRequest();
  auto distance_result = collision_detection::DistanceResult();
  distance_request.group_name = "left_hand"; //"right_hand";
  distance_request.enable_signed_distance = true;
  distance_request.type = collision_detection::DistanceRequestType::SINGLE; // global minimum
  distance_request.enableGroup(kinematic_model); // specify which group to check
  const collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
  distance_request.acm = &acm; // specify acm to ignore adjacent links' collision check
  // construct a CollisionRobotFCL for calling distanceSelf function
  collision_detection::CollisionRobotFCL collision_robot_fcl(kinematic_model); // construct collisionrobot from RobotModelConstPtr
  collision_robot_fcl.distanceSelf(distance_request, distance_result, current_state);
  ROS_INFO_STREAM(">>>> Test 3 on using DistanceSelf to compute penetration/minimum distance:");
  ROS_INFO_STREAM("Left hand is " << (distance_result.collision ? "in" : "not in") << " self collision.");
  ROS_INFO_STREAM("Minimum distance is " << distance_result.minimum_distance.distance << ", between " << distance_result.minimum_distance.link_names[0] << " and " << distance_result.minimum_distance.link_names[1]);
  // compute for the right hand
  distance_result.clear();
  distance_request.group_name = "right_hand";
  distance_request.enableGroup(kinematic_model);
  collision_robot_fcl.distanceSelf(distance_request, distance_result, current_state);
  ROS_INFO_STREAM("Right hand is " << (distance_result.collision ? "in" : "not in") << " self collision.");
  ROS_INFO_STREAM("Minimum distance is " << distance_result.minimum_distance.distance << ", between " << distance_result.minimum_distance.link_names[0] << " and " << distance_result.minimum_distance.link_names[1]);


  // Self-collision between groups(Use DistanceRequest for acquiring penetration/minimum distance information)
  /*
  auto new_distance_request = collision_detection::DistanceRequest();
  auto new_distance_result = collision_detection::DistanceResult();
  new_distance_request.group_name = "left_hand"; //"right_hand";
  new_distance_request.enable_signed_distance = true;
  new_distance_request.type = collision_detection::DistanceRequestType::SINGLE; 
  new_distance_request.acm = &acm; // Allowed Collision Matrix
  // reset left hand's state
  current_state.setJointGroupPositions(joint_model_group, joint_values);
  std::vector<double> arm_joint_values = {-1.3, 1.05, -1.12, 0.02, 0.0, 0.0}; // left arm's joint values; for left hand to collide with the right arm(elbow)
  const robot_model::JointModelGroup* arm_joint_model_group = current_state.getJointModelGroup("left_arm");
  current_state.setJointGroupPositions(arm_joint_model_group, arm_joint_values);
  current_state.update(); // update????
  new_distance_request.group_name = "left_hand";
  //distance_request.enableGroup(kinematic_model);
  collision_robot_fcl.distanceSelf(new_distance_request, new_distance_result, current_state);

  ROS_INFO_STREAM(">>>> Test 4 on self-collision checking between groups(In collision): ");
  ROS_INFO_STREAM("Manually set state of the left arm is " << (current_state.satisfiesBounds(arm_joint_model_group) ? "within joint limits" : "beyond joint limits"));
  ROS_INFO_STREAM("Left hand and left arm is " << (new_distance_result.collision ? "in" : "not in") << " self collision");  
  ROS_INFO_STREAM("Minimum distance is " << new_distance_result.minimum_distance.distance << ", between " << new_distance_result.minimum_distance.link_names[0] << " and " << new_distance_result.minimum_distance.link_names[1]);
  // set left hand in self-collision state
  new_distance_result.clear();
  joint_values = {-1.0, -1.47, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.62, 0.0, -0.56, -0.3 };
  ROS_INFO_STREAM("Set left hand in self-collision state...");
  current_state.setJointGroupPositions(joint_model_group, joint_values);
  current_state.update(); // update????
  collision_robot_fcl.distanceSelf(new_distance_request, new_distance_result, current_state);
  ROS_INFO_STREAM("Left hand and left arm is " << (new_distance_result.collision ? "in" : "not in") << " self collision");  
  ROS_INFO_STREAM("Minimum distance is " << new_distance_result.minimum_distance.distance << ", between " << new_distance_result.minimum_distance.link_names[0] << " and " << new_distance_result.minimum_distance.link_names[1]);
  // get DistanceMap
  ROS_INFO_STREAM("Distance Map:");
  collision_detection::DistanceMap::const_iterator itr;
  int i = 0;
  for (itr = new_distance_result.distances.begin(); itr != new_distance_result.distances.end(); ++itr)
  {
    ROS_INFO_STREAM("Distance between " << itr->first.first << " and " << itr->first.second << " is " << itr->second[i].distance);
    i++;
  }
  */

  // Full collision checking: self-collision checking(intra- and inter-group) + collision with the environment
  //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  ros::Duration(2.0).sleep();
  // Prepare collision objects(size and pose)
 

  /*
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
  box_pose.position.x = 0.3;
  box_pose.position.y = 0.4;
  box_pose.position.z = 0.4;
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;
  ROS_INFO_STREAM("Add a box to into the world");
  */

  // add collision object into the environment through planning_scene_diff
/*  moveit_msgs::PlanningScene planning_scene_msg;
  planning_scene_msg.world.collision_objects.push_back(collision_object);
  planning_scene_msg.is_diff = true;

  ros::ServiceClient planning_scene_diff_client = node_handle.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
  planning_scene_diff_client.waitForExistence();
  moveit_msgs::ApplyPlanningScene srv; // synchronous manner, send the diffs to the planning scene via a service call
  srv.request.scene = planning_scene_msg;
  planning_scene_diff_client.call(srv);
*/

  // add collision object through planning scene interface
  // planning_scene_interface.applyCollisionObject(collision_object); // synchronously, can take action directly after this command, i.e. no need for sleep; collision object is added, yet not used for distanceRobot???


  // Collision checking with the environment
  //current_state.update();
  collision_detection::DistanceRequest distance_world_request;
  collision_detection::DistanceResult distance_world_result;
  distance_world_request.group_name = "left_arm"; //"right_hand";
  distance_world_request.enable_signed_distance = true;
  distance_world_request.type = collision_detection::DistanceRequestType::SINGLE; // global minimum
  distance_world_request.enableGroup(kinematic_model); // specify which group to check
  distance_world_request.acm = &acm; // specify acm to ignore adjacent links' collision check
  // get CollisionWorldFCL from WorldPtr
  const collision_detection::WorldPtr &world_ptr = planning_scene.getWorldNonConst();
  collision_detection::CollisionWorldFCL collision_world_fcl(world_ptr);

  // Add object to the world 
  /*
  std::string id = "box1";   
  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  shapes::ShapeConstPtr shapre_ptr = new &shapes::BOX(0.5, 0.3, 0.6);
  world_ptr->addToObject(id, shapre_ptr, pose); // WorldPtr to the current planning scene
  */

  // Call distanceRobot to compute collision with the environment
  collision_world_fcl.distanceRobot(distance_world_request, distance_world_result, collision_robot_fcl, current_state);
  // Display the result
  ROS_INFO_STREAM(">>>> Test 5 on collision checking with the environment:");
  ROS_INFO_STREAM("Left arm is " << (distance_world_result.collision ? "in" : "not in") << " collision with the environment");
  ROS_INFO_STREAM("Minimum distance is " << distance_world_result.minimum_distance.distance << ", between " << distance_world_result.minimum_distance.link_names[0] << " and " << distance_world_result.minimum_distance.link_names[1]);
  // get DistanceMap
  ROS_INFO_STREAM("Distance Map:");
  collision_detection::DistanceMap::const_iterator iter;
  int ii = 0;
  for (iter = distance_world_result.distances.begin(); iter != distance_world_result.distances.end(); ++iter)
  {
    ROS_INFO_STREAM("Distance between " << iter->first.first << " and " << iter->first.second << " is " << iter->second[ii].distance);
    ii++;
  }


  // Another way to compute
  //double d = planning_scene.distanceToCollision(current_state);
  //ROS_INFO_STREAM("The distance between the robot model to the nearest collision is " << d);


  // Remove collision objects
  ros::Duration(3.0).sleep();
  ROS_INFO_STREAM("Remove the box from the world");  
  //std::vector<std::string> object_ids;
  //object_ids.push_back(collision_object.id);
  //planning_scene_interface.removeCollisionObjects(object_ids);
  /*
  planning_scene_msg.world.collision_objects.clear();
  collision_object.operation = collision_object.REMOVE; // Remove operation
  planning_scene_msg.world.collision_objects.push_back(collision_object);
  srv.request.scene = planning_scene_msg;
  planning_scene_diff_client.call(srv);
  */


  // Shut down
  ros::shutdown();
  return 0;


}

