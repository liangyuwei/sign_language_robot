#!/usr/bin/env python

import rospy
import h5py
import copy
import time
import numpy as np


# msg files
from raw_totg.srv import PathToTraj, GetMinTime
from trajectory_msgs.msg import JointTrajectoryPoint


### TOTG service ###
def add_time_optimal_parameterization_client(path, vel_limits, acc_limits, timestep=0.001):
  # wait for service to come online
  rospy.wait_for_service('add_time_optimal_parameterization_server')

  try:
    path_to_traj = rospy.ServiceProxy('add_time_optimal_parameterization_server', PathToTraj)
    res = path_to_traj(path, vel_limits, acc_limits, timestep)
    return res.traj
  except rospy.ServiceException as e:
    print("Service call failed: %s" % e)


### TOTG service, get the duration only ###
def get_minimum_time_client(path, vel_limits, acc_limits):
  # wait for service to come online
  rospy.wait_for_service('get_minimum_time_server')

  try:
    get_min_time = rospy.ServiceProxy('get_minimum_time_server', GetMinTime)
    res = get_min_time(path, vel_limits, acc_limits)
    return res.min_time
  except rospy.ServiceException as e:
    print("Service call failed: %s" % e)


### TOPP service ###
def TOPP_client(path, vel_limits, acc_limits, timestep=0.001):
  # wait for service to come online
  rospy.wait_for_service('TOPP_server')

  try:
    path_to_traj = rospy.ServiceProxy('TOPP_server', PathToTraj)
    res = path_to_traj(path, vel_limits, acc_limits, timestep)
    return res.traj
  except rospy.ServiceException as e:
    print("Service call failed: %s" % e)


### TOPP-RA service ###
def TOPPRA_client(path, vel_limits, acc_limits, timestep=0.001):
  # wait for service to come online
  rospy.wait_for_service('TOPPRA_server')

  try:
    path_to_traj = rospy.ServiceProxy('TOPPRA_server', PathToTraj)
    res = path_to_traj(path, vel_limits, acc_limits, timestep)
    return res.traj
  except rospy.ServiceException as e:
    print("Service call failed: %s" % e)


### Conversion ###
def convert_plan_to_array(traj_plan):

  # input: trajectory_msgs/JointTrajectoryPoint[]
  ndata = len(traj_plan)
  traj_array = np.zeros((ndata, 6))
  for i in range(ndata):
    traj_array[i, :] = np.array(traj_plan[i].positions)
  return traj_array


if __name__ == '__main__':


  ### Load the example path
  f = h5py.File('example_paths_for_TP_comparison.h5', 'r')
  l_joint_path_array = f['l_joint_path_array_1']['pos'][:]
  r_joint_path_array = f['r_joint_path_array_1']['pos'][:]  
  f.close()


  ### Convert array to list of JointTrajectoryPoint
  l_path_points = []
  l_path_point = JointTrajectoryPoint()
  for i in range(l_joint_path_array.shape[1]):
    l_path_point.positions = copy.deepcopy(l_joint_path_array[:, i].tolist())
    l_path_points.append(copy.deepcopy(l_path_point))
  
  r_path_points = []
  r_path_point = JointTrajectoryPoint()
  for i in range(r_joint_path_array.shape[1]):
    r_path_point.positions = copy.deepcopy(r_joint_path_array[:, i].tolist())
    r_path_points.append(copy.deepcopy(r_path_point))


  ### Parameters
  vel_limits = [3.15, 3.15, 3.15, 3.15, 3.15, 3.15] #[0.5 for _ in range(6)] 
  acc_limits = [3.15, 3.15, 3.15, 3.15, 3.15, 3.15] #[0.5 for _ in range(6)] 
  timestep = 0.01


  ### Call TOTG service
  print("=== Call TOTG service.")
  t0_TOTG = time.time()
  l_traj_TOTG = add_time_optimal_parameterization_client(l_path_points, vel_limits, acc_limits, timestep)
  r_traj_TOTG = add_time_optimal_parameterization_client(r_path_points, vel_limits, acc_limits, timestep)
  t1_TOTG = time.time()
  print("=== Total time used for TOTG(including communication): " + str(t1_TOTG-t0_TOTG) + " s")
  l_traj_array_TOTG = convert_plan_to_array(l_traj_TOTG)
  r_traj_array_TOTG = convert_plan_to_array(r_traj_TOTG)


  ### Call TOPP service
  t0_TOPP = time.time()  
  l_traj_TOPP = TOPP_client(l_path_points, vel_limits, acc_limits, timestep)
  r_traj_TOPP = TOPP_client(r_path_points, vel_limits, acc_limits, timestep)
  t1_TOPP = time.time()  
  print("=== Total time used for TOPP(including communication): " + str(t1_TOPP-t0_TOPP) + " s")
  #import pdb
  #pdb.set_trace()
  l_traj_array_TOPP = convert_plan_to_array(l_traj_TOPP)
  r_traj_array_TOPP = convert_plan_to_array(r_traj_TOPP)


  ### Call TOPP-RA service
  t0_TOPPRA = time.time()  
  l_traj_TOPPRA = TOPPRA_client(l_path_points, vel_limits, acc_limits, timestep)
  r_traj_TOPPRA = TOPPRA_client(r_path_points, vel_limits, acc_limits, timestep)
  t1_TOPPRA = time.time()  
  print("=== Total time used for TOPP-RA(including communication): " + str(t1_TOPPRA-t0_TOPPRA) + " s")
  #import pdb
  #pdb.set_trace()
  l_traj_array_TOPPRA = convert_plan_to_array(l_traj_TOPPRA)
  r_traj_array_TOPPRA = convert_plan_to_array(r_traj_TOPPRA)


  ### Print trajectory time
  print(">>> Trajectory time <<<")
  print("TOTG - left: " + str(l_traj_TOTG[-1].time_from_start.to_sec()) + " s")
  print("TOTG - right: " + str(r_traj_TOTG[-1].time_from_start.to_sec()) + " s")  
  print("TOPP - left: " + str(l_traj_TOPP[-1].time_from_start.to_sec()) + " s")
  print("TOPP - right: " + str(r_traj_TOPP[-1].time_from_start.to_sec()) + " s")
  print("TOPP-RA - left: " + str(l_traj_TOPPRA[-1].time_from_start.to_sec()) + " s")
  print("TOPP-RA - right: " + str(r_traj_TOPPRA[-1].time_from_start.to_sec()) + " s")
  print(">>> End <<<")


  ### Display the results
  import pdb
  pdb.set_trace()
  import matplotlib.pyplot as plt
  from mpl_toolkits.mplot3d import Axes3D
  traj_list = [[l_traj_array_TOTG, r_traj_array_TOTG], [l_traj_array_TOPP, r_traj_array_TOPP], [l_traj_array_TOPPRA, r_traj_array_TOPPRA]]
  traj_list_names = ['TOTG', 'TOPP', 'TOPP-RA']
  # iterate to plot
  for i in range(len(traj_list_names)):
    #fig = plt.figure() # create a figure object
    fig, ax = plt.subplots(nrows=6, ncols=2)
    fig.suptitle(traj_list_names[i] + " parameterized joint trajectory: Left and Right", fontsize=18)
    for j in range(6):  
      # left arm  
      ax[j][0].set_xlabel('trajectory point index')
      ax[j][0].set_ylabel('joint angle')
      ax[j][0].plot(range(traj_list[i][0].shape[0]), traj_list[i][0]) # line plot
      #ax[j][0].scatter(range(traj_list[i][0].shape[0]), l_path_points, marker='*') # can't draw scatter points since we don't know where they could be
      # right arm      
      ax[j][1].set_xlabel('trajectory point index')
      ax[j][1].set_ylabel('joint angle')
      ax[j][1].plot(range(traj_list[i][1].shape[0]), traj_list[i][1]) # line plot
      #ax[j][1].scatter(range(traj_list[i][1].shape[0]), traj_list[i][1], marker='*') # scatter points
  # plot  
  plt.show()





