#!/usr/bin/env python
import sys
import copy
import rospy
import math
import numpy as np
from raw_totg.srv import *
import trajectory_msgs.msg


def add_time_optimal_parameterization_client(path, vel_limits, acc_limits, timestep=0.001):

  # wait for service to come online
  rospy.wait_for_service('add_time_optimal_parameterization_server')

  try:
    path_to_traj = rospy.ServiceProxy('add_time_optimal_parameterization_server', PathToTraj)
    res = path_to_traj(path, vel_limits, acc_limits, timestep)
    return res.traj
  except rospy.ServiceException, e:
    print "Service call failed: %s" % e


if __name__ == '__main__':

  # Initialization
  path = []
  traj_point = trajectory_msgs.msg.JointTrajectoryPoint()
  traj_point.positions = [0.0, 0.0, 0.0, 0.0, 0.0]
  path.append(copy.deepcopy(traj_point))
  traj_point.positions = [1.0, 1.0, 2.0, 1.5, 2.5]
  path.append(copy.deepcopy(traj_point))
  traj_point.positions = [3.0, 2.0, 6.0, 3.5, 4.0]
  path.append(copy.deepcopy(traj_point))
  traj_point.positions = [6.0, 7.0, 8.0, 4.5, 5.5]
  path.append(copy.deepcopy(traj_point))
  traj_point.positions = [8.0, 9.0, 15.0, 6.0, 7.5]
  path.append(copy.deepcopy(traj_point))

  vel_limits = [3.0, 3.0, 3.0, 3.0, 3.0]
  acc_limits = [10.0, 10.0, 10.0, 10.0, 10.0]

  timestep = 0.001

  # call the service
  import pdb
  pdb.set_trace()
  traj = add_time_optimal_parameterization_client(path, vel_limits, acc_limits, timestep)

  # check the result
  import pdb
  pdb.set_trace()

  print "done."
