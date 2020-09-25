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
  traj_name = "arm_traj_1"
  #timestamp_name = "timestamp"

  try:
    options, args = getopt.getopt(sys.argv[1:], "hf:g:t:", ["help", "file-name=", "group-name=", "traj-name"])
  except getopt.GetoptError:
    sys.exit()

  for option, value in options:
    if option in ("-h", "--help"):
      print("Help:\n")
      print("   This script executes the IK results.\n")
      print("Arguments:\n")
      print("   -f, --file-name=, specify the name of the h5 file to read joint trajectory from, suffix is required.\n")
      print("   -g, --group-name=, specify the name of the motion.\n")
      print("   -t, --traj-name=, specify the name of the trajectory.\n")
      exit(0)
    if option in ("-f", "--file-name"):
      print("Name of the h5 file to read joint trajectory from: {0}\n".format(value))
      file_name = value
    if option in ("-g", "--group-name"):
      print("Name of the motion(group) inside the h5 file: {0}\n".format(value))
      group_name = value
    if option in ("-t", "--traj-name"):
      print("Name of the data to read: {0}\n".format(value))
      traj_name = value


  try:

    ### Set up the moveit_commander
    print "============ Setting up the moveit_commander (press ctrl-d to exit) ..."
    #tutorial = MoveGroupPythonIntefaceTutorial()


    ### Set up groups
    print "============ Set up group ..."
    '''
    left_arm_group = moveit_commander.MoveGroupCommander("left_arm")
    left_hand_group = moveit_commander.MoveGroupCommander("left_hand")
    right_hand_group = moveit_commander.MoveGroupCommander("right_hand")
    right_arm_group = moveit_commander.MoveGroupCommander("right_arm")
    dual_arms_group = moveit_commander.MoveGroupCommander("dual_arms")
    '''
    dual_arms_with_hands_group = moveit_commander.MoveGroupCommander("dual_arms_with_hands")


    ### Read h5 file for joint paths (arms only for now)
    f = h5py.File(file_name, "r")

    '''
    l_dataset_name = "fake_path_left_1"
    l_path_array = f[l_dataset_name][:]
    l_finger_pos = l_path_array[0][-12:] # the last 12 digits

    r_dataset_name = "fake_path_right_1"
    r_path_array = f[r_dataset_name][:]
    r_finger_pos = r_path_array[0][-12:]
    '''
    arm_path_array = f[group_name + "/" + traj_name][:]
    #timestamp_array = f[group_name+"/" + timestamp_name][:]

    f.close()


    ### Arms: Go to start positions
    print "============ Both arms go to initial positions..."
    dual_arms_with_hands_start = arm_path_array[0, :7].tolist() + arm_path_array[0, 14:26].tolist() + arm_path_array[0, 7:14].tolist() + arm_path_array[0, 26:38].tolist()
    # group joints structure: left arm, left hand, right arm, right hand
    # IK results structure: left arm(0-7), right arm(7-14), left hand(14-26), right hand(26-38)
    dual_arms_with_hands_group.allow_replanning(True)
    #dual_arms_with_hands_group.set_joint_value_target(dual_arms_with_hands_start)
    dual_arms_with_hands_group.go(dual_arms_with_hands_start, wait=True)
    dual_arms_with_hands_group.stop()


    ### Hands: Go to start positions
    print "============ Both hands go to initial positions..."
    '''
    ## left hand
    print "====== Left hand reaching initial position..."
    left_finger_goal = l_finger_pos.tolist()
    left_hand_group.go(left_finger_goal, wait=True)
    left_hand_group.stop()
    ## right hand
    print "====== Right hand reaching initial position..."
    right_finger_goal = r_finger_pos.tolist()
    right_hand_group.go(right_finger_goal, wait=True)
    right_hand_group.stop()
    '''
    '''
    import pdb
    pdb.set_trace()
    dual_hands_group = moveit_commander.MoveGroupCommander("dual_hands")
    dual_hands_goal = l_finger_pos.tolist() + r_finger_pos.tolist()
    dual_hands_group.go(dual_hands_goal, wait=True)
    dual_hands_group.stop()
    '''


    ### Construct a plan
    print "============ Construct a plan of two arms' motion..."
    cartesian_plan = moveit_msgs.msg.RobotTrajectory()
    cartesian_plan.joint_trajectory.header.frame_id = '/world'
    cartesian_plan.joint_trajectory.joint_names = ['yumi_joint_1_l', 'yumi_joint_2_l', 'yumi_joint_7_l', 'yumi_joint_3_l', 'yumi_joint_4_l', 'yumi_joint_5_l', 'yumi_joint_6_l'] \
    + ['link1', 'link11', 'link2', 'link22', 'link3', 'link33', 'link4', 'link44', 'link5', 'link51', 'link52', 'link53'] \
    + ['yumi_joint_1_r', 'yumi_joint_2_r', 'yumi_joint_7_r', 'yumi_joint_3_r', 'yumi_joint_4_r', 'yumi_joint_5_r', 'yumi_joint_6_r'] \
    + ['Link1', 'Link11', 'Link2', 'Link22', 'Link3', 'Link33', 'Link4', 'Link44', 'Link5', 'Link51', 'Link52', 'Link53']

    # structure: left arm, left hand, right arm, right hand; different from the result of IK

    for i in range(arm_path_array.shape[0]):
        path_point = trajectory_msgs.msg.JointTrajectoryPoint()
        path_point.positions = arm_path_array[i, :7].tolist() + arm_path_array[i, 14:26].tolist() + arm_path_array[i, 7:14].tolist() + arm_path_array[i, 26:38].tolist()
        t = rospy.Time(i*1.0/15.0) #rospy.Time(i*1.0/15.0) # rospy.Time(timestamp_array[i]) # 15 Hz # rospy.Time(0.5*i) #
        path_point.time_from_start.secs = t.secs
        path_point.time_from_start.nsecs = t.nsecs        
        cartesian_plan.joint_trajectory.points.append(copy.deepcopy(path_point))


    ### Execute the plan
    print "============ Execute the plan..."
    # execute the plan
    import pdb
    pdb.set_trace()
    print "============ Execute the planned path..."        
    dual_arms_with_hands_group.execute(cartesian_plan, wait=True)


  except rospy.ROSInterruptException:
    return

  except KeyboardInterrupt:
    return


if __name__ == '__main__':

  main()







