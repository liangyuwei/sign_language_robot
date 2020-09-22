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

def main():

  file_name = "mocap_ik_results_YuMi.h5"
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
    dual_arms_group = moveit_commander.MoveGroupCommander("dual_arms")


    ### Read h5 file for joint paths (arms only for now)
    f = h5py.File(file_name, "r")
    l_joint_angles_ik_yumi = f[group_name]['l_joint_angles_optim_ik_yumi']
    r_joint_angles_ik_yumi = f[group_name]['r_joint_angles_optim_ik_yumi']
    lr_joint_angles_ik_yumi = np.concatenate((l_joint_angles_ik_yumi, r_joint_angles_ik_yumi), axis=1)
    f.close()


    ### Arms: Go to start positions
    import pdb
    pdb.set_trace()
    print "============ Both arms go to initial positions..."
    dual_arms_group.allow_replanning(True)
    dual_arms_group.go(lr_joint_angles_ik_yumi[0, :], wait=True)
    dual_arms_group.stop()

    import pdb
    pdb.set_trace()


    ### Construct a plan
    print "============ Construct a plan of two arms' motion..."
    cartesian_plan = moveit_msgs.msg.RobotTrajectory()
    cartesian_plan.joint_trajectory.header.frame_id = '/world'
    cartesian_plan.joint_trajectory.joint_names = ['yumi_joint_1_l', 'yumi_joint_2_l', 'yumi_joint_7_l', 'yumi_joint_3_l', 'yumi_joint_4_l', 'yumi_joint_5_l', 'yumi_joint_6_l'] \
    + ['yumi_joint_1_r', 'yumi_joint_2_r', 'yumi_joint_7_r', 'yumi_joint_3_r', 'yumi_joint_4_r', 'yumi_joint_5_r', 'yumi_joint_6_r'] 

    for i in range(lr_joint_angles_ik_yumi.shape[0]):
        path_point = trajectory_msgs.msg.JointTrajectoryPoint()
        path_point.positions = lr_joint_angles_ik_yumi[i].tolist()
        t = rospy.Time(i*1.0/5.0) 
        path_point.time_from_start.secs = t.secs
        path_point.time_from_start.nsecs = t.nsecs        
        cartesian_plan.joint_trajectory.points.append(copy.deepcopy(path_point))


    ### Execute the plan
    print "============ Execute the planned path..."        
    dual_arms_group.execute(cartesian_plan, wait=True)


  except rospy.ROSInterruptException:
    return

  except KeyboardInterrupt:
    return


if __name__ == '__main__':

  main()







