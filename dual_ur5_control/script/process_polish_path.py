#!/usr/bin/env python

import rospy
import csv
import io
import numpy as np
import math
import copy
from raw_totg.srv import *
from trajectory_msgs.msg import JointTrajectoryPoint


### TOTG ###
def add_time_optimal_parameterization_client(path, vel_limits, acc_limits, timestep=0.01):
  # wait for service to come online
  rospy.wait_for_service('add_time_optimal_parameterization_server')

  try:
    path_to_traj = rospy.ServiceProxy('add_time_optimal_parameterization_server', PathToTraj)
    res = path_to_traj(path, vel_limits, acc_limits, timestep)
    return res.traj
  except rospy.ServiceException as e:
    print("Service call failed: %s" % e)


### TOPP service ###
def TOPP_client(path, vel_limits, acc_limits, timestep=0.01):
  # wait for service to come online
  rospy.wait_for_service('TOPP_server')

  try:
    path_to_traj = rospy.ServiceProxy('TOPP_server', PathToTraj)
    res = path_to_traj(path, vel_limits, acc_limits, timestep)
    return res.traj
  except rospy.ServiceException as e:
    print("Service call failed: %s" % e)


### TOPP-RA service ###
def TOPPRA_client(path, vel_limits, acc_limits, timestep=0.01):
  # wait for service to come online
  rospy.wait_for_service('TOPPRA_server')

  try:
    path_to_traj = rospy.ServiceProxy('TOPPRA_server', PathToTraj)
    res = path_to_traj(path, vel_limits, acc_limits, timestep)
    return res.traj
  except rospy.ServiceException as e:
    print("Service call failed: %s" % e)


def csv_read(path):
    data = []
    with io.open(path, 'r', encoding='utf-8') as f:
      reader = csv.reader(f,dialect='excel')
      for row in reader:
        data.append(row)
    return data


if __name__ == '__main__':


  # Load the array of path points
  #path_points_array = np.array(csv_read('data_real(1).csv'))
  path_points_array = np.loadtxt(open("data_real(1).csv","rb"),delimiter=",",skiprows=0)


  # Construct a plan of path points 
  path_points_plan = []
  path_point = JointTrajectoryPoint()
  for i in range(path_points_array.shape[0]):
    path_point.positions = path_points_array[i, 0:5].tolist()
    path_points_plan.append(copy.deepcopy(path_point))
  
  
  # Limits
  vel_limits = [math.pi for _ in range(5)]
  acc_limits = [math.pi for _ in range(5)] # 5-dim!!!


  # Call the server to add time parameterization
  import pdb
  pdb.set_trace()
  new_path_points_plan = add_time_optimal_parameterization_client(path_points_plan, vel_limits, acc_limits, 0.04)
  #TOPPRA_client(path_points_plan, vel_limits, acc_limits, 0.01)
  #add_time_optimal_parameterization_client(path_points_plan, vel_limits, acc_limits, 0.04)


  # Write to .csv
  print("Minimum time is: " + str(new_path_points_plan[-1].time_from_start.to_sec()) )
  import pdb
  pdb.set_trace()
  with open('new_data_TOTG.csv', 'w') as f:
    writer = csv.writer(f)
    for i in range(len(new_path_points_plan)):
      writer.writerow(new_path_points_plan[i].positions)









  
