#include <iostream>
#include <cstdio>
#include <string>
#include <vector>
#include <ctime>
#include <Eigen/Core>
#include "Trajectory.h"
#include "Path.h"
#include "ros/ros.h"
#include "raw_totg/GetMinTime.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"

using namespace std;
using namespace Eigen;

clock_t t0, t1, t2;

bool path_to_traj(raw_totg::GetMinTime::Request &req, raw_totg::GetMinTime::Response &res)
{

	// Obtain the request
	ROS_INFO("Obtain the request");
	t0 = clock();
	vector<trajectory_msgs::JointTrajectoryPoint> path = req.path;
	vector<double> vel_limits = req.vel_limits;
	vector<double> acc_limits = req.acc_limits;

	// get the pos information
	ROS_INFO("Get the pos information...");
	list<VectorXd> waypoints;
	VectorXd waypoint;
	vector<double> tmp;
	for (auto it = path.cbegin(); it != path.cend(); ++it)
	{
		tmp = it->positions;
		//Map<VectorXd> waypoint(tmp.data(), tmp.size());
		waypoint = Map<VectorXd>(tmp.data(), tmp.size()); // << it->positions[0], it->positions[1], it->positions[2], it->positions[3], it->positions[4], it->positions[5]; //
		waypoints.push_back(waypoint);
	}
	t1 = clock();

	// Set velocity and acceleration limits
	ROS_INFO("Set velocity and acceleration limits...");
	VectorXd maxAcceleration;
	maxAcceleration = Map<VectorXd>(acc_limits.data(), acc_limits.size()); //<< 10.0, 10.0, 10.0, 10.0, 10.0, 10.0;
	VectorXd maxVelocity;
	maxVelocity = Map<VectorXd>(vel_limits.data(), vel_limits.size()); //<< 3.15, 3.15, 3.15, 3.15, 3.15, 3.15;

	// Add time parameterization
	
	ROS_INFO("Get prepared to add TP...");
	Trajectory trajectory(Path(waypoints, 0.1), maxVelocity, maxAcceleration);
	ROS_INFO("Output phase plane trajectory..");
	trajectory.outputPhasePlaneTrajectory();
	ROS_INFO("Check if trajectory is valid");
	if(trajectory.isValid()) {
		ROS_INFO("Trajectory is valid.");
		double duration = trajectory.getDuration();
		ROS_INFO("Got duration: %f", duration);
		// assign the results and return
		res.min_time = duration;
	}
	else 
	{
		ROS_ERROR("Trajectory generation failed.");
		return false;
	}

	t2 = clock();

	ROS_INFO(">>> Time used for getting request path and transforming into other datatypes: %f", (double)(t1-t0)/CLOCKS_PER_SEC);
	ROS_INFO(">>> Time used for getting duration from TOTG: %f", (double)(t2-t1)/CLOCKS_PER_SEC);


	return true;

}


int main(int argc, char **argv) 
{

	// Initialize a ros node
	ros::init(argc, argv, "get_minimum_time_server");
	ros::NodeHandle n;

	// Start the server
	ros::ServiceServer service = n.advertiseService("get_minimum_time_server", path_to_traj);
	ROS_INFO("Ready to compute optimal time usage for paths.");
	ros::spin();

	return 0;
	
}


