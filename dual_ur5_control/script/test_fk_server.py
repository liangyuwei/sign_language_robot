#!/usr/bin/env python
import sys
import copy
import rospy
import math
import numpy as np
from dual_ur5_control.srv import *
import trajectory_msgs.msg
import geometry_msgs.msg


def apply_fk_client(joint_trajectory, left_or_right):

  if type(joint_trajectory) is list:
    if type(joint_trajectory[0]) is not trajectory_msgs.msg.JointTrajectoryPoint:
      print "Wrong data type for joint_trajectory!"
      return False
  else:
    if type(joint_trajectory) is not trajectory_msgs.msg.JointTrajectoryPoint:
      print "Wrong data type for joint_trajectory!"
      return False

  if type(left_or_right) is not bool:
    print "Wrong data type for left_or_right selection variable!"
    return False

  # wait for service to come online
  rospy.wait_for_service('apply_fk_server')

  try:
    joint_to_cart = rospy.ServiceProxy('apply_fk_server', JntToCart)
    res = joint_to_cart(left_or_right, joint_trajectory)
    return res.cart_trajectory
  except rospy.ServiceException, e:
    print "Service call failed: %s" % e


if __name__ == '__main__':

  # Initialization
  joint_trajectory_l = []
  traj_point = trajectory_msgs.msg.JointTrajectoryPoint()
  traj_point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  joint_trajectory_l.append(copy.deepcopy(traj_point))
  traj_point.positions = [0.0, 1.0, 0.0, 1.0, 0.0, 3.0]
  joint_trajectory_l.append(copy.deepcopy(traj_point))


  # call the service
  import pdb
  pdb.set_trace()
  cart_traj_l = apply_fk_client(joint_trajectory_l, True)


  # check the result
  import pdb
  pdb.set_trace()

  print "done."
