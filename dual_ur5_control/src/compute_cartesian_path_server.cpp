#include "ros/ros.h"
#include <iostream>
#include <string>
#include <vector>
#include <ctime>

// moveit
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// service and msg
#include "dual_ur5_control/CartToJnt.h"
#include "geometry_msgs/Pose.h"
#include "moveit_msgs/RobotTrajectory.h"


using namespace std;

clock_t t0, t1, t2, t3;

bool cart_to_joint(dual_ur5_control::CartToJnt::Request &req, dual_ur5_control::CartToJnt::Response &res)
{

  // get the request
  t0 = clock();
  ROS_INFO("Obtain the request pose path.");
  string group_name = req.group_name; // do not use static const... static variable is initialized when the function is called the first time, but would not be destroyed as the function terminates
  vector<geometry_msgs::Pose> waypoints = req.waypoints;
  

  // prepare for planning
  t1 = clock();
  ROS_INFO("Getting ready for planning...");
  ROS_INFO_STREAM("Planning for group: " << group_name); // ROS_INFO_STREAM vs. ROS_INFO???
  moveit::planning_interface::MoveGroupInterface move_group(group_name);
  ROS_INFO_STREAM("Planning frame: " << move_group.getPlanningFrame());
  ROS_INFO_STREAM("End effector link: " << move_group.getEndEffectorLink());


  // Compute Cartesian Path
  t2 = clock();
  ROS_INFO("Compute Cartesian Path...");
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01; // 1 cm in workspace
  bool avoid_collisions = false;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, avoid_collisions);
  // if fraction < 1.0, then the planning is not successful...
  t3 = clock();

  // Display time usage on screen
  ROS_INFO(">>> Time used for obtaining the request pose path: %f", (double)(t1-t0)/CLOCKS_PER_SEC);
  ROS_INFO(">>> Time used for getting ready for planning: %f", (double)(t2-t1)/CLOCKS_PER_SEC);
  ROS_INFO(">>> Time used for computing cartesian path: %f", (double)(t3-t2)/CLOCKS_PER_SEC);

  // Return results
  res.plan = trajectory;
  

  return true;


}


int main(int argc, char** argv)
{

  // Initialize a node
  ros::init(argc, argv, "compute_cartesian_path_server");
  ros::NodeHandle n;

  // Start the server
	ros::ServiceServer service = n.advertiseService("compute_cartesian_path_server", cart_to_joint);
	ROS_INFO("Ready to compute cartesian path using C++.");
	ros::spin();
  

  return 0;

}
