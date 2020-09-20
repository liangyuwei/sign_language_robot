#!/usr/bin/env python

import sys
import copy
import rospy
import math
import tf
import pdb
import numpy as np
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg
from math import pi
from std_msgs.msg import String

import getopt # process the terminal arguments

import h5py


def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


def main():

  file_name = "test_imi_data.h5"
  group_name = "baozhu_1"

  try:
    options, args = getopt.getopt(sys.argv[1:], "hf:g:", ["help", "file-name=", "group-name="])
  except getopt.GetoptError:
    sys.exit()

  for option, value in options:
    if option in ("-h", "--help"):
      print("Help:\n")
      print("   This script executes the IK results.\n")
      print("Arguments:\n")
      print("   -f, --file-name=, specify the name of the h5 file to read joint trajectory from, suffix is required.\n")
      print("   -g, --group-name=, specify the name of the motion.\n")
      exit(0)
    if option in ("-f", "--file-name"):
      print("Name of the h5 file to read joint trajectory from: {0}\n".format(value))
      file_name = value
    if option in ("-g", "--group-name"):
      print("Name of the motion(group) inside the h5 file: {0}\n".format(value))
      group_name = value


  try:

    ### Set up groups
    print("============ Set up group ...")
    left_arm_group = moveit_commander.MoveGroupCommander("left_arm")
    right_arm_group = moveit_commander.MoveGroupCommander("right_arm")
    dual_arms_group = moveit_commander.MoveGroupCommander("dual_arms")


    ### Read h5 file for joint paths (arms only for now)
    f = h5py.File(file_name, "r")
    l_joint_angles_ik = f[group_name]['l_joint_angles_ik'][:]
    r_joint_angles_ik = f[group_name]['r_joint_angles_ik'][:] # N x 7, 7 DOF and N path points
    lr_joint_angles_ik = np.concatenate((l_joint_angles_ik, r_joint_angles_ik), axis=1)
    timestamp = f[group_name]['time'][:] # N x 1
    f.close()


    ### Arms: Go to start positions
    import pdb
    pdb.set_trace()
    print "============ Both arms go to initial positions..."
    # group joints structure: left arm, right arm
    dual_arms_group.allow_replanning(True)
    dual_arms_group.go(lr_joint_angles_ik[0, :], wait=True)
    dual_arms_group.stop()

    import pdb
    pdb.set_trace()


    ### Construct a plan
    print "============ Construct a plan of two arms' motion..."
    cartesian_plan = moveit_msgs.msg.RobotTrajectory()
    cartesian_plan.joint_trajectory.header.frame_id = '/world'
    cartesian_plan.joint_trajectory.joint_names = ['left_shoulder_1_joint', 'left_shoulder_2_joint', 'left_shoulder_3_joint', \
                                                   'left_elbow_1_joint', 'left_elbow_2_joint', \
                                                   'left_wrist_1_joint', 'left_wrist_2_joint', \
                                                   'right_shoulder_1_joint', 'right_shoulder_2_joint', 'right_shoulder_3_joint', \
                                                   'right_elbow_1_joint', 'right_elbow_2_joint', \
                                                   'right_wrist_1_joint', 'right_wrist_2_joint']
    for i in range(lr_joint_angles_ik.shape[0]):
        path_point = trajectory_msgs.msg.JointTrajectoryPoint()
        path_point.positions = lr_joint_angles_ik[i].tolist()
        t = rospy.Time(i*1.0/5.0) 
        path_point.time_from_start.secs = t.secs
        path_point.time_from_start.nsecs = t.nsecs        
        cartesian_plan.joint_trajectory.points.append(copy.deepcopy(path_point))


    ### Execute the plan
    print "============ Execute the plan..."
    # execute the plan
    print "============ Execute the planned path..."        
    dual_arms_group.execute(cartesian_plan, wait=True)


  except rospy.ROSInterruptException:
    return

  except KeyboardInterrupt:
    return


if __name__ == '__main__':

  main()






