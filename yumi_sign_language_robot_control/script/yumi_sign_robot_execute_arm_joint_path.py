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

  file_name = "mocap_ik_results_YuMi_hujin.h5"
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
    dual_arms_with_hands_group = moveit_commander.MoveGroupCommander("dual_arms_with_hands")


    ### Read h5 file for joint paths (arms only for now)
    f = h5py.File(file_name, "r")
    ## arm
    l_joint_angles_ik_yumi = f[group_name]['arm_traj_affine_l']#['l_joint_angles_optim_ik_yumi']#
    r_joint_angles_ik_yumi = f[group_name]['arm_traj_affine_r']#['r_joint_angles_optim_ik_yumi']#
    ## hand (My code for Hujin/Affine's method does not cope with finger joints, and simply using linear mapping would yield out-of-bound results)
    # use the results from our method
    ff = h5py.File("/home/liangyuwei/sign_language_robot_ws/test_imi_data/mocap_ik_results_YuMi_g2o_similarity.h5")
    arm_path_array = ff[group_name]['arm_traj_1']
    l_finger_joint_angles_our = arm_path_array[:, 14:26]
    r_finger_joint_angles_our = arm_path_array[:, 26:38] # finger joints results from our method
    ff.close()
    # interpolate to obtain finger joint angles for Hujin/Affine's method
    N1 = l_finger_joint_angles_our.shape[0]
    N2 = l_joint_angles_ik_yumi.shape[0]
    seq1 = np.linspace(0, 1, N1)
    seq2 = np.linspace(0, 1, N2)
    l_finger_joint_angles = np.zeros((N2, 12))  #f[group_name]['l_finger_joints']
    r_finger_joint_angles = np.zeros((N2, 12))  #f[group_name]['r_finger_joints']
    for i in range(12):
      l_finger_joint_angles[:, i] = np.interp(seq2, seq1, l_finger_joint_angles_our[:, i])
      r_finger_joint_angles[:, i] = np.interp(seq2, seq1, r_finger_joint_angles_our[:, i])
    # combine dual arms and dual hands (l arm, r arm, l hand, r hand)
    lr_joint_angles_ik_yumi = np.concatenate((l_joint_angles_ik_yumi, r_joint_angles_ik_yumi, l_finger_joint_angles, r_finger_joint_angles), axis=1)
    f.close()


    ### Arms: Go to start positions
    print "============ Both arms go to initial positions..."
    # go to a non-colliding initial state
    q_init = np.zeros(38)
    q_init = [-1.5, -1.5, 1.5, 0.0, 0.0, 0.0, 0.0] + lr_joint_angles_ik_yumi[0, 14:26].tolist() + [1.5, -1.5, -1.5, 0.0, 0.0, 0.0, 0.0] + lr_joint_angles_ik_yumi[0, 26:38].tolist()
    dual_arms_with_hands_start = q_init #arm_path_array[1, :7].tolist() + arm_path_array[1, 14:26].tolist() + arm_path_array[1, 7:14].tolist() + arm_path_array[1, 26:38].tolist()
    dual_arms_with_hands_group.allow_replanning(True)
    #dual_arms_with_hands_group.set_joint_value_target(dual_arms_with_hands_start)
    dual_arms_with_hands_group.go(dual_arms_with_hands_start, wait=True)
    dual_arms_with_hands_group.stop()

    import pdb
    pdb.set_trace()

    ### Construct a plan
    print("============ Construct a plan of two arms' motion...")
    cartesian_plan = moveit_msgs.msg.RobotTrajectory()
    cartesian_plan.joint_trajectory.header.frame_id = '/world'
    cartesian_plan.joint_trajectory.joint_names = ['yumi_joint_1_l', 'yumi_joint_2_l', 'yumi_joint_7_l', 'yumi_joint_3_l', 'yumi_joint_4_l', 'yumi_joint_5_l', 'yumi_joint_6_l'] \
    + ['link1', 'link11', 'link2', 'link22', 'link3', 'link33', 'link4', 'link44', 'link5', 'link51', 'link52', 'link53'] \
    + ['yumi_joint_1_r', 'yumi_joint_2_r', 'yumi_joint_7_r', 'yumi_joint_3_r', 'yumi_joint_4_r', 'yumi_joint_5_r', 'yumi_joint_6_r'] \
    + ['Link1', 'Link11', 'Link2', 'Link22', 'Link3', 'Link33', 'Link4', 'Link44', 'Link5', 'Link51', 'Link52', 'Link53']

    # add non-colliding start state into the trajectory for ease of execution
    path_point = trajectory_msgs.msg.JointTrajectoryPoint()
    path_point.positions = q_init
    t = rospy.Time(0.0) 
    path_point.time_from_start.secs = t.secs
    path_point.time_from_start.nsecs = t.nsecs        
    cartesian_plan.joint_trajectory.points.append(copy.deepcopy(path_point))
    # add the original start state for delaying the demonstration
    path_point = trajectory_msgs.msg.JointTrajectoryPoint()
    path_point.positions = lr_joint_angles_ik_yumi[0, :7].tolist() + lr_joint_angles_ik_yumi[0, 14:26].tolist() + lr_joint_angles_ik_yumi[0, 7:14].tolist() + lr_joint_angles_ik_yumi[0, 26:38].tolist()
    t = rospy.Time(0.1) 
    path_point.time_from_start.secs = t.secs
    path_point.time_from_start.nsecs = t.nsecs        
    cartesian_plan.joint_trajectory.points.append(copy.deepcopy(path_point))

    t_delay = 2.0 # delay for better demonstration

    for i in range(lr_joint_angles_ik_yumi.shape[0]):
        path_point = trajectory_msgs.msg.JointTrajectoryPoint()
        path_point.positions = lr_joint_angles_ik_yumi[i, :7].tolist() + lr_joint_angles_ik_yumi[i, 14:26].tolist() + lr_joint_angles_ik_yumi[i, 7:14].tolist() + lr_joint_angles_ik_yumi[i, 26:38].tolist()
        t = rospy.Time(t_delay + i*1.0/15.0) 
        path_point.time_from_start.secs = t.secs
        path_point.time_from_start.nsecs = t.nsecs        
        cartesian_plan.joint_trajectory.points.append(copy.deepcopy(path_point))


    ### Execute the plan
    print("============ Execute the planned path...")
    dual_arms_group.execute(cartesian_plan, wait=True)


  except rospy.ROSInterruptException:
    return

  except KeyboardInterrupt:
    return


if __name__ == '__main__':

  main()







