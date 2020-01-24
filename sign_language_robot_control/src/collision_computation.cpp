#include <ros/ros.h>

#include <vector>
#include <string>
#include <fstream>

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

// For collision object
#include "geometric_shapes/shapes.h"


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
  //std::size_t count = 0;

  ros::NodeHandle node_handle;
 

  // Set up PlanningScene class(from RobotModel or URDF & SRDF; see the constructor in documentation)

  // way 1(checked): load from robot_description
  //robot_model_loader::RobotModelLoader robot_model_loader("robot_description");

  // way 2(checked): construct a robot model ptr from URDF and SRDF so that the moveit needs not be launched
  std::string urdf_file_name = "/home/liangyuwei/sign_language_robot_ws/src/ur_description/urdf/ur5_robot_with_hands.urdf";
  std::string srdf_file_name = "/home/liangyuwei/sign_language_robot_ws/src/sign_language_robot_moveit_config/config/ur5.srdf";
  std::ifstream urdf_file(urdf_file_name);
  std::ifstream srdf_file(srdf_file_name);;
  std::stringstream urdf_string, srdf_string;
  urdf_string << urdf_file.rdbuf();
  srdf_string << srdf_file.rdbuf();
  robot_model_loader::RobotModelLoader::Options options(urdf_string.str(), srdf_string.str());
  robot_model_loader::RobotModelLoader robot_model_loader(options);
  

  // Get robot model from robot_model_loader
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel(); 
  planning_scene::PlanningScene planning_scene(kinematic_model);


  // Collision Checking
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  planning_scene.checkSelfCollision(collision_request, collision_result);
  ROS_INFO_STREAM("Test 1: Current state of the whole robot is " << (collision_result.collision ? "in" : "not in") << " self collision");


  // Get current_state for later use
  robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst();


  // Self-collision for a group(left_hand group; In collision state), and get contact information
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
  ROS_INFO_STREAM("Closest distance is " << collision_result.distance); // If in collision, the collision_result.distance is equal to 0(no penetration depth information)


  // Self-collision for a group(Not in collision state)
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
  collision_detection::DistanceRequest distance_request;
  collision_detection::DistanceResult distance_result;
  distance_request.group_name = "left_hand"; //"right_hand";
  distance_request.enable_signed_distance = true;
  distance_request.type = collision_detection::DistanceRequestType::SINGLE; // global minimum
  distance_request.enableGroup(kinematic_model); // specify which group to check
  const collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
  distance_request.acm = &acm; // specify acm to ignore adjacent links' collision check
  // construct a CollisionRobotFCL for calling distanceSelf function
  collision_detection::CollisionRobotFCL collision_robot_fcl(kinematic_model); // construct collisionrobot from RobotModelConstPtr
  collision_robot_fcl.distanceSelf(distance_request, distance_result, current_state);
  ROS_INFO_STREAM(">>>> Test 3 on self-distance computation(using CollisionRobotFCL::distanceSelf) :");
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
  collision_detection::DistanceRequest new_distance_request;
  collision_detection::DistanceResult new_distance_result;
  new_distance_request.group_name = "left_hand"; //"right_hand";
  new_distance_request.enable_signed_distance = true;
  new_distance_request.type = collision_detection::DistanceRequestType::SINGLE; 
  new_distance_request.acm = &acm; // Allowed Collision Matrix
  // reset left hand's state
  current_state.setJointGroupPositions(joint_model_group, joint_values);
  std::vector<double> arm_joint_values = {-1.3, 1.05, -1.12, 0.02, 0.0, 0.0}; // set left arm's joint values; for left hand to collide with the right arm(elbow)
  const robot_model::JointModelGroup* arm_joint_model_group = current_state.getJointModelGroup("left_arm");
  current_state.setJointGroupPositions(arm_joint_model_group, arm_joint_values);
  current_state.update(); // update????
  new_distance_request.group_name = "left_hand";
  distance_request.enableGroup(kinematic_model);
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
  // get DistanceMap (a long list of distance information)
  /*
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
  // clear the results
  collision_result.clear();
  //ros::Duration(2.0).sleep();
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
  box_pose.position.x = 0.3;
  box_pose.position.y = 0.4;
  box_pose.position.z = 0.4;
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;
  // way 1(checked, able to add visual object, but not useful in detection): add collision object into the environment through planning_scene_diff(synchronous update) (one of two mechanisms available to interact with the move_group node)
  /*ROS_INFO_STREAM("Add a box to into the world through /apply_planning_scene");
  moveit_msgs::PlanningScene planning_scene_msg; // this is just a message !!!
  planning_scene_msg.world.collision_objects.clear();
  planning_scene_msg.world.collision_objects.push_back(collision_object);
  planning_scene_msg.is_diff = true;
  ros::ServiceClient planning_scene_diff_client = node_handle.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
  planning_scene_diff_client.waitForExistence();
  moveit_msgs::ApplyPlanningScene srv; // synchronous manner, send the diffs to the planning scene via a service call
  srv.request.scene = planning_scene_msg;
  planning_scene_diff_client.call(srv); // does not continue until we are sure the diff has been applied*/

  // way 2(checked, able to add visual object, but not useful in detection): add collision object through planning scene interface (apply collision object to the planning scene of the move_group node synchronously)
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  
  planning_scene_interface.applyCollisionObject(collision_object); // synchronously, can take action directly after this command, i.e. no need for sleep; collision object is added, yet not used for distanceRobot???


  // way 3(checked, useless): add collision object through planning_scene_diff(asynchronous update)
  /*ROS_INFO_STREAM("Add a box to into the world through /planning_scene");
  ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  moveit_msgs::PlanningScene planning_scene_msg;
  planning_scene_msg.world.collision_objects.clear();
  planning_scene_msg.world.collision_objects.push_back(collision_object);
  planning_scene_msg.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene_msg);
  ros::Duration(3.0).sleep();*/

  // way4(checked, useless): add collision object through collision_object topic
  /*ros::Publisher add_collision_object_pub = node_handle.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
  add_collision_object_pub.publish(collision_object);*/

  ROS_INFO_STREAM("Box is added now.");


  // full Collision checking (self + environment)
  ROS_INFO_STREAM(">>>> Test 5 on full collision checking:");
  // reset all the joints' positions
  ROS_INFO_STREAM("Reset every joint to default(initial) position.");
  current_state.setToDefaultValues(); //setJointGroupPositions(joint_model_group, joint_values);
  current_state.update();
  // call checkCollision()
  collision_result.clear();
  collision_request.group_name = "left_arm"; //"right_arm"; "";// check for all
  planning_scene.checkCollision(collision_request, collision_result, current_state, acm);
  // .checkCollision() function calls both CollisionWorldPtr->checkRobotCollision() and CollisionRobotPtr->checkSelfCollision() for collision checking with the environment and the robot itself respectively !!!
  ROS_INFO_STREAM(collision_request.group_name << " is " << (collision_result.collision ? "in" : "not in") << " collision");  
  ROS_INFO_STREAM("Closest distance: " << collision_result.distance);
  for (it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it)
  {
    ROS_INFO("Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());
  }


  // Intermediate test for planning scene
  ROS_INFO_STREAM(">>>> Test 5.5 for PlanningScene");
  ROS_INFO_STREAM("Name of the planning scene: " << planning_scene.getName());
  //const planning_scene::PlanningSceneConstPtr parent_planning_scene = planning_scene.getParent();   
  //ROS_INFO_STREAM("Name of the parent of the planning scene: " << parent_planning_scene->getName());
  //planning_scene.pushDiffs(parent_planning_scene);
  /*moveit_msgs::CollisionObject get_collision_object;
  planning_scene.getCollisionObjectMsg(get_collision_object, "box1");
  std::vector<shape_msgs::SolidPrimitive> get_primitives = get_collision_object.primitives;
  ROS_INFO_STREAM("Size of the added box: " << get_primitives[0].dimensions[0] << " x " << get_primitives[0].dimensions[1] << " x " << get_primitives[0].dimensions[2] << ".");*/

  std::vector<std::string> known_object_names = planning_scene_interface.getKnownObjectNames();
  ROS_INFO_STREAM("Known object names: ");
  for (auto it = known_object_names.cbegin(); it != known_object_names.cend(); ++it)
    ROS_INFO_STREAM(*it << " ");


  // Distance checking with the environment
  ros::Duration(3.0).sleep();
  current_state.update();
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


  // way 4(un-checked): Add object to the world 
  /*std::string id = "box1";   
  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  shapes::BOX new_box;
  world_ptr->addToObject(id, shapre_ptr, pose); // WorldPtr to the current planning scene*/

  
  // check out information
  ROS_INFO_STREAM("The object " << collision_object.id << " is " << (world_ptr->hasObject(collision_object.id) ? "in" : "not in") << " the world."); 




  // Call distanceRobot to compute collision with the environment
  collision_world_fcl.distanceRobot(distance_world_request, distance_world_result, collision_robot_fcl, current_state);
  // Display the result
  ROS_INFO_STREAM(">>>> Test 6 on distance checking with the environment(using CollisionWorldFCL::distanceRobot):");
  ROS_INFO_STREAM(distance_world_request.group_name <<" is " << (distance_world_result.collision ? "in" : "not in") << " collision with the environment");
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
  double d = planning_scene.distanceToCollision(current_state); // ignoring self-collision !!!
  ROS_INFO_STREAM(">>>> Test 7 on distance checking with the environment(using PlanningScene::distanceToCollision):");
  ROS_INFO_STREAM("The distance between the robot model to the nearest collision is " << d);


  // Collision checking with the environment(CollisionWorld::checkRobotCollision)
  current_state.update();
  collision_result.clear();
  collision_request.group_name = "left_arm"; 
  collision_world_fcl.checkRobotCollision(collision_request, collision_result, collision_robot_fcl, current_state);
  ROS_INFO_STREAM(">>>> Test 8 on collision checking with the environment(using CollisionWorld::checkRobotCollision)");
  ROS_INFO_STREAM(collision_request.group_name << " is " << (collision_result.collision ? "in" : "not in") << " collision with the environment.");
  ROS_INFO_STREAM("Closest distance: " << collision_result.distance);


  // Get PlanningScene from move_group throught PlanningSceneMonitor
  const std::string PLANNING_SCENE_SERVICE = "get_planning_scene"; // default service name
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_ =  std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
  planning_scene_monitor_->requestPlanningSceneState(PLANNING_SCENE_SERVICE);
  planning_scene_monitor::LockedPlanningSceneRW ps(planning_scene_monitor_); // update how??
  //ps->getCurrentStateNonConst().update();
  // test 
  planning_scene::PlanningScenePtr scene = ps->diff(); // get a child PlanningScene
  scene->decoupleParent(); // decouple from parent; make sure that changes to kmodel_, kstate_, ftf_ and acm_ in the parent are not synced in the child PlanningScene(to avoid unknown changes!!!)
  collision_result.clear();
  collision_request.group_name = "left_arm"; //"right_arm"; "";// check for all
  scene->checkCollision(collision_request, collision_result, current_state, acm);
  ROS_INFO_STREAM(">>>> Test 9 on full collision checking again(using the child scene obtained through PlanningSceneMonitor)");
  ROS_INFO_STREAM(collision_request.group_name << " is " << (collision_result.collision ? "in" : "not in") << " collision (with itself or the environment).");


  // test on state update
  const robot_model::JointModelGroup* left_arm_group_1 = current_state.getJointModelGroup("left_arm");
  std::vector<double> left_arm_joint_values = {0.0, -2.46, 0.0, 0.0, 0.0, 0.0};
  current_state.setJointGroupPositions(left_arm_group_1, left_arm_joint_values);
  current_state.update();
  collision_result.clear();
  collision_request.group_name = "left_arm"; //"right_arm"; "";// check for all
  scene->checkCollision(collision_request, collision_result, current_state, acm);
  ROS_INFO_STREAM(">>>> Test 10 on changing state of the local planning scene");
  ROS_INFO_STREAM(collision_request.group_name << " is " << (collision_result.collision ? "in" : "not in") << " collision with the environment.");


  robot_state::RobotState& child_current_state = scene->getCurrentStateNonConst();
  const robot_model::JointModelGroup* left_arm_group_2 = child_current_state.getJointModelGroup("left_arm");
  child_current_state.setJointGroupPositions(left_arm_group_2, left_arm_joint_values);
  child_current_state.update();
  collision_result.clear();
  collision_request.group_name = "left_arm"; //"right_arm"; "";// check for all
  scene->checkCollision(collision_request, collision_result, child_current_state, acm);
  ROS_INFO_STREAM(">>>> Test 11 on changing state of the child planning scene");
  ROS_INFO_STREAM(collision_request.group_name << " is " << (collision_result.collision ? "in" : "not in") << " collision with the environment.");

  
  // test on (1)collision checking with the environment, and (2)distance computation, using the newly cloned child PlanningScene(child of the one maintained by move_group)
  ROS_INFO_STREAM(">>>> Test 12 on collision checking with the environment using the child PlanningScene.");
  current_state.setToDefaultValues(); // set to colliding state
  current_state.update();
  // get CollisionWorldFCL from WorldPtr (from the child PlanningScene)
  const collision_detection::WorldPtr &new_world_ptr = scene->getWorldNonConst();
  collision_detection::CollisionWorldFCL new_collision_world_fcl(new_world_ptr);
  // check out information
  ROS_INFO_STREAM("The object " << collision_object.id << " is " << (new_world_ptr->hasObject(collision_object.id) ? "in" : "not in") << " the newly exported world of the child PlanningScene."); 
  // check collision with the environment
  collision_result.clear();
  collision_request.group_name = "left_arm"; 
  new_collision_world_fcl.checkRobotCollision(collision_request, collision_result, collision_robot_fcl, current_state);
  ROS_INFO_STREAM(collision_request.group_name << " is " << (collision_result.collision ? "in" : "not in") << " collision with the environment.");
  ROS_INFO_STREAM("Closest distance: " << collision_result.distance);
  // Call distanceRobot to compute collision with the environment
  new_collision_world_fcl.distanceRobot(distance_world_request, distance_world_result, collision_robot_fcl, current_state);
  // Display the result
  ROS_INFO_STREAM(">>>> Test 13 on distance checking with the environment(using CollisionWorldFCL::distanceRobot):");
  ROS_INFO_STREAM(distance_world_request.group_name <<" is " << (distance_world_result.collision ? "in" : "not in") << " collision with the environment");
  ROS_INFO_STREAM("Minimum distance is " << distance_world_result.minimum_distance.distance << ", between " << distance_world_result.minimum_distance.link_names[0] << " and " << distance_world_result.minimum_distance.link_names[1]);
  // get DistanceMap
  ROS_INFO_STREAM("Distance Map:");
  ii = 0;
  for (iter = distance_world_result.distances.begin(); iter != distance_world_result.distances.end(); ++iter)
  {
    ROS_INFO_STREAM("Distance between " << iter->first.first << " and " << iter->first.second << " is " << iter->second[ii].distance);
    ii++;
  }




  // Remove collision objects
  ros::Duration(3.0).sleep();
  ROS_INFO_STREAM("Remove the box from the world");  
  // way 1: via service client synchronous way
  /*planning_scene_msg.world.collision_objects.clear();
  collision_object.operation = collision_object.REMOVE; // Remove operation
  planning_scene_msg.world.collision_objects.push_back(collision_object);
  srv.request.scene = planning_scene_msg;
  planning_scene_diff_client.call(srv);*/

  // way 2: planning scene interface
  std::vector<std::string> object_ids;
  object_ids.push_back(collision_object.id);
  planning_scene_interface.removeCollisionObjects(object_ids);

  // way 3: via topic, asynchronous way
  /*planning_scene_msg.world.collision_objects.clear();
  collision_object.operation = collision_object.REMOVE; // Remove operation
  planning_scene_msg.world.collision_objects.push_back(collision_object);
  planning_scene_diff_publisher.publish(planning_scene_msg);*/

  


  // Shut down
  ros::shutdown();
  return 0;


}

