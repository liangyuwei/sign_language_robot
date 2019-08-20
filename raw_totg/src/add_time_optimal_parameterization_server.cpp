#include <iostream>
#include <cstdio>
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

	// assign the path points
	ROS_INFO("Assigning path points...");
	list<VectorXd> waypoints;
	VectorXd waypoint(6);
	vector<double> tmp;
	for (auto it = path.cbegin(); it != path.cend(); ++it)
	{
		tmp = it->positions;
		//Map<VectorXd> waypoint(tmp.data(), tmp.size());
		waypoint << it->positions[0], it->positions[1], it->positions[2], it->positions[3], it->positions[4], it->positions[5]; // = Map<VectorXd>(tmp.data(), tmp.size()); //
		waypoints.push_back(waypoint);
	}


	// Set velocity and acceleration limits
	ROS_INFO("Set velocity and acceleration limits...");
	VectorXd maxAcceleration(6);
	maxAcceleration << 10.0, 10.0, 10.0, 10.0, 10.0, 10.0;
	VectorXd maxVelocity(6);
	maxVelocity << 3.15, 3.15, 3.15, 3.15, 3.15, 3.15;

	// Add time parameterization
	ROS_INFO("Get prepared to add TP...");
	double timestep = 0.001;
	Trajectory trajectory(Path(waypoints, 0.1), maxVelocity, maxAcceleration);
	ROS_INFO("Output phase plane trajectory..");
	trajectory.outputPhasePlaneTrajectory();
	ROS_INFO("Check if trajectory is valid");
	if(trajectory.isValid()) {
		ROS_INFO("Adding TP...");
		double duration = trajectory.getDuration();
		/*
		cout << "Trajectory duration: " << duration << " s" << endl << endl;
		cout << "Time      Position                  Velocity                  Acceleration" << endl;
		for(double t = 0.0; t < duration; t += timestep) { //0.1) {
			printf("%6.4f   %7.4f %7.4f %7.4f %7.4f %7.4f %7.4f   %7.4f %7.4f %7.4f %7.4f %7.4f %7.4f   %7.4f %7.4f %7.4f %7.4f %7.4f %7.4f\n", t, 
				trajectory.getPosition(t)[0], trajectory.getPosition(t)[1], trajectory.getPosition(t)[2], trajectory.getPosition(t)[3], trajectory.getPosition(t)[4], trajectory.getPosition(t)[5],
				trajectory.getVelocity(t)[0], trajectory.getVelocity(t)[1], trajectory.getVelocity(t)[2], trajectory.getVelocity(t)[3], trajectory.getVelocity(t)[4], trajectory.getVelocity(t)[5],
				trajectory.getAcceleration(t)[0], trajectory.getAcceleration(t)[1], trajectory.getAcceleration(t)[2], trajectory.getAcceleration(t)[3], trajectory.getAcceleration(t)[4], trajectory.getAcceleration(t)[5]);
		}
		// print the last
		printf("%6.4f   %7.4f %7.4f %7.4f %7.4f %7.4f %7.4f   %7.4f %7.4f %7.4f %7.4f %7.4f %7.4f   %7.4f %7.4f %7.4f %7.4f %7.4f %7.4f\n", duration, 
			trajectory.getPosition(duration)[0], trajectory.getPosition(duration)[1], trajectory.getPosition(duration)[2], trajectory.getPosition(duration)[3], trajectory.getPosition(duration)[4], trajectory.getPosition(duration)[5],
			trajectory.getVelocity(duration)[0], trajectory.getVelocity(duration)[1], trajectory.getVelocity(duration)[2], trajectory.getVelocity(duration)[3], trajectory.getVelocity(duration)[4], trajectory.getVelocity(duration)[5],
			trajectory.getAcceleration(duration)[0], trajectory.getAcceleration(duration)[1], trajectory.getAcceleration(duration)[2], trajectory.getAcceleration(duration)[3], trajectory.getAcceleration(duration)[4], trajectory.getAcceleration(duration)[5]);
		*/

		// Assign to the response
		trajectory_msgs::JointTrajectoryPoint traj_point;
		vector<trajectory_msgs::JointTrajectoryPoint> traj;
		VectorXd tmpPosition, tmpVelocity, tmpAcceleration;
		for(double t = 0.0; t < duration; t += timestep)
		{		
			// from VectorXd to vector<double>
			tmpPosition = trajectory.getPosition(t);
			tmpVelocity = trajectory.getVelocity(t);
			tmpAcceleration = trajectory.getAcceleration(t);
			traj_point.positions = { tmpPosition[0], tmpPosition[1], tmpPosition[2], tmpPosition[3],  tmpPosition[4], tmpPosition[5] };
			traj_point.velocities = { tmpVelocity[0], tmpVelocity[1], tmpVelocity[2], tmpVelocity[3],  tmpVelocity[4], tmpVelocity[5] };
			traj_point.accelerations = { tmpAcceleration[0], tmpAcceleration[1], tmpAcceleration[2], tmpAcceleration[3],  tmpAcceleration[4], tmpAcceleration[5] };
			traj_point.time_from_start = ros::Duration(t);
			traj.push_back(traj_point);
		}
		tmpPosition = trajectory.getPosition(duration);
		tmpVelocity = trajectory.getVelocity(duration);
		tmpAcceleration = trajectory.getAcceleration(duration);
		traj_point.positions = { tmpPosition[0], tmpPosition[1], tmpPosition[2], tmpPosition[3],  tmpPosition[4], tmpPosition[5] };
		traj_point.velocities = { tmpVelocity[0], tmpVelocity[1], tmpVelocity[2], tmpVelocity[3],  tmpVelocity[4], tmpVelocity[5] };
		traj_point.accelerations = { tmpAcceleration[0], tmpAcceleration[1], tmpAcceleration[2], tmpAcceleration[3],  tmpAcceleration[4], tmpAcceleration[5] };
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


