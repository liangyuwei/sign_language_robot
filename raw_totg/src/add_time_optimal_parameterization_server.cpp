#include <iostream>
#include <cstdio>
#include <string>
#include <vector>
#include <Eigen/Core>
#include "Trajectory.h"
#include "Path.h"
#include "ros/ros.h"
#include "raw_totg/PathToTraj.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"

using namespace std;
using namespace Eigen;

bool path_to_traj(raw_totg::PathToTraj::Request &req, raw_totg::PathToTraj::Response &res)
{

	// Obtain the request
	ROS_INFO("Obtain the request");
	vector<trajectory_msgs::JointTrajectoryPoint> path = req.path;
	vector<double> vel_limits = req.vel_limits;
	vector<double> acc_limits = req.acc_limits;
	double timestep = req.timestep; // 0.001;

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
		ROS_INFO("Trajectory is valid. Adding TP now...");

		double duration = trajectory.getDuration();

		ROS_INFO("Got duration: %f", duration);

		// Assign to the response
		trajectory_msgs::JointTrajectoryPoint traj_point;
		vector<trajectory_msgs::JointTrajectoryPoint> traj;
		int dim = vel_limits.size(); // get the dimension
		VectorXd tmpPosition(dim), tmpVelocity(dim), tmpAcceleration(dim);

		for(double t = 0.0; t < duration; t += timestep)
		{		
			// get pos, vel and acc
			ROS_INFO("Generating trajectory point at time t = %f / %f", t, duration);
			tmpPosition = trajectory.getPosition(t);
			tmpVelocity = trajectory.getVelocity(t);
			tmpAcceleration = trajectory.getAcceleration(t);

			vector<double> vecPosition(tmpPosition.data(), tmpPosition.data() + tmpPosition.rows() * tmpPosition.cols());
			vector<double> vecVelocity(tmpVelocity.data(), tmpVelocity.data() + tmpVelocity.rows() * tmpVelocity.cols());
			vector<double> vecAcceleration(tmpAcceleration.data(), tmpAcceleration.data() + tmpPosition.rows() * tmpAcceleration.cols());
			// store the info, from VectorXd to vector<double>
			traj_point.positions = vecPosition; //{ tmpPosition[0], tmpPosition[1], tmpPosition[2]}; //= 
			traj_point.velocities = vecVelocity; //{ tmpVelocity[0], tmpVelocity[1], tmpVelocity[2]}; //
			traj_point.accelerations = vecAcceleration; //{ tmpAcceleration[0], tmpAcceleration[1], tmpAcceleration[2] }; //
			traj_point.time_from_start = ros::Duration(t);
			traj.push_back(traj_point);
		}
		ROS_INFO("Generating trajectory point at time t = %f / %f", duration, duration);
		// for the last point
		tmpPosition = trajectory.getPosition(duration);
		tmpVelocity = trajectory.getVelocity(duration);
		tmpAcceleration = trajectory.getAcceleration(duration);
		vector<double> vecPosition(tmpPosition.data(), tmpPosition.data() + tmpPosition.rows() * tmpPosition.cols());
		vector<double> vecVelocity(tmpVelocity.data(), tmpVelocity.data() + tmpVelocity.rows() * tmpVelocity.cols());
		vector<double> vecAcceleration(tmpAcceleration.data(), tmpAcceleration.data() + tmpAcceleration.rows() * tmpAcceleration.cols());
		// store the info, from VectorXd to vector<double>
		traj_point.positions = vecPosition; //= { tmpPosition[0], tmpPosition[1], tmpPosition[2], tmpPosition[3],  tmpPosition[4], tmpPosition[5] };
		traj_point.velocities = vecVelocity; //{ tmpVelocity[0], tmpVelocity[1], tmpVelocity[2], tmpVelocity[3],  tmpVelocity[4], tmpVelocity[5] };
		traj_point.accelerations = vecAcceleration; //{ tmpAcceleration[0], tmpAcceleration[1], tmpAcceleration[2], tmpAcceleration[3],  tmpAcceleration[4], tmpAcceleration[5] };
		traj_point.time_from_start = ros::Duration(duration);
		traj.push_back(traj_point);
		res.traj = traj;

	}
	else 
	{
		ROS_ERROR("Trajectory generation failed.");
		return false;
	}
	

	

	return true;

}


int main(int argc, char **argv) 
{

	// Initialize a ros node
	ros::init(argc, argv, "add_time_optimal_parameterization_server");
	ros::NodeHandle n;

	// Start the server
	ros::ServiceServer service = n.advertiseService("add_time_optimal_parameterization_server", path_to_traj);
	ROS_INFO("Ready to add time optimal parameterization.");
	ros::spin();

	return 0;
	
}


