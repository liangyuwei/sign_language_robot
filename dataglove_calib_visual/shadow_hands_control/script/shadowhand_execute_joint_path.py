#!/usr/bin/env python

import sys
import copy
import rospy
import math
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


def linear_map():





if __name__ == '__main__':

  file_name = "glove-calib-2020-11-02.h5"
  test_seq_name = 'test_seq_1' 

  try:
    options, args = getopt.getopt(sys.argv[1:], "hf:t:", ["help", "file-name=", "test-seq-name"])
  except getopt.GetoptError:
    sys.exit()

  for option, value in options:
    if option in ("-h", "--help"):
      print("Help:\n")
      print("   This script executes the IK results.\n")
      print("Arguments:\n")
      print("   -f, --file-name=, specify the name of the h5 file to read joint trajectory from, suffix is required.\n")
      print("   -t, --test-seq-name=, specify the name of the test sequence to visualize.\n")
      exit(0)
    if option in ("-f", "--file-name"):
      print("Name of the h5 file to read joint trajectory from: {0}\n".format(value))
      file_name = value
    if option in ("-t", "--test-seq-name"):
      print("Name of the test sequence data: {0}\n".format(value))
      test_seq_name = value


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


    ### Read h5 file for dataglove data
    f = h5py.File(file_name, "r")
    hand_path_array = f[test_seq_name]
    f.close()


    ### Arms: Go to start positions
    print "============ Both arms go to initial positions..."
    q_init = np.zeros(38)
    q_init[0:7] = [-1.5, -1.5, 1.5, 0.0, 0.0, 0.0, 0.0]
    q_init[19:26] = [1.5, -1.5, -1.5, 0.0, 0.0, 0.0, 0.0]
    # q_init = [-1.5, -1.5, 1.5, 0.0, 0.0, 0.0, 0.0] + arm_path_array[0, 14:26].tolist() + [1.5, -1.5, -1.5, 0.0, 0.0, 0.0, 0.0] + arm_path_array[0, 26:38].tolist()
    dual_arms_with_hands_start = q_init #arm_path_array[0, :7].tolist() + arm_path_array[0, 14:26].tolist() + arm_path_array[0, 7:14].tolist() + arm_path_array[0, 26:38].tolist()
    # group joints structure: left arm, left hand, right arm, right hand
    # IK results structure: left arm(0-7), right arm(7-14), left hand(14-26), right hand(26-38)
    dual_arms_with_hands_group.allow_replanning(True)
    #dual_arms_with_hands_group.set_joint_value_target(dual_arms_with_hands_start)
    dual_arms_with_hands_group.go(dual_arms_with_hands_start, wait=True)
    dual_arms_with_hands_group.stop()


    ### Hands: Go to start positions
    print "============ Both hands go to initial positions..."
    
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
    
    
    import pdb
    pdb.set_trace()
    dual_hands_group = moveit_commander.MoveGroupCommander("dual_hands")
    dual_hands_goal = l_finger_pos.tolist() + r_finger_pos.tolist()
    dual_hands_group.go(dual_hands_goal, wait=True)
    dual_hands_group.stop()
    


    ### Construct a plan
    print "============ Construct a plan of two arms' motion..."
    cartesian_plan = moveit_msgs.msg.RobotTrajectory()
    cartesian_plan.joint_trajectory.header.frame_id = '/world'
    cartesian_plan.joint_trajectory.joint_names = ['yumi_joint_1_l', 'yumi_joint_2_l', 'yumi_joint_7_l', 'yumi_joint_3_l', 'yumi_joint_4_l', 'yumi_joint_5_l', 'yumi_joint_6_l'] \
    + ['link1', 'link11', 'link2', 'link22', 'link3', 'link33', 'link4', 'link44', 'link5', 'link51', 'link52', 'link53'] \
    + ['yumi_joint_1_r', 'yumi_joint_2_r', 'yumi_joint_7_r', 'yumi_joint_3_r', 'yumi_joint_4_r', 'yumi_joint_5_r', 'yumi_joint_6_r'] \
    + ['Link1', 'Link11', 'Link2', 'Link22', 'Link3', 'Link33', 'Link4', 'Link44', 'Link5', 'Link51', 'Link52', 'Link53']

    # structure: left arm, left hand, right arm, right hand; different from the result of IK

    # add a non-colliding initial state to the trajectory for it to be able to execute via MoveIt
    path_point = trajectory_msgs.msg.JointTrajectoryPoint()
    path_point.positions = q_init #arm_path_array[0, :7].tolist() + arm_path_array[0, 14:26].tolist() + arm_path_array[0, 7:14].tolist() + arm_path_array[0, 26:38].tolist()
    t = rospy.Time(0) #rospy.Time(i*1.0/15.0) # rospy.Time(timestamp_array[i]) # 15 Hz # rospy.Time(0.5*i) #
    path_point.time_from_start.secs = t.secs
    path_point.time_from_start.nsecs = t.nsecs        
    cartesian_plan.joint_trajectory.points.append(copy.deepcopy(path_point))
    
    # add the original initial point for delay demonstration
    path_point = trajectory_msgs.msg.JointTrajectoryPoint()
    path_point.positions = arm_path_array[0, :7].tolist() + arm_path_array[0, 14:26].tolist() + arm_path_array[0, 7:14].tolist() + arm_path_array[0, 26:38].tolist()
    t = rospy.Time(0.1) #rospy.Time(i*1.0/15.0) # rospy.Time(timestamp_array[i]) # 15 Hz # rospy.Time(0.5*i) #
    path_point.time_from_start.secs = t.secs
    path_point.time_from_start.nsecs = t.nsecs        
    cartesian_plan.joint_trajectory.points.append(copy.deepcopy(path_point))
    
    t_delay = 2.0  # delay for 2 seconds before executing the motion 

    # add velocity and acceleration information
    '''
    last_pos = np.array(path_point.positions)
    last_vec = np.zeros(38) 
    last_acc = np.zeros(38)
    '''

    for i in range(arm_path_array.shape[0]):
        path_point = trajectory_msgs.msg.JointTrajectoryPoint()
        path_point.positions = arm_path_array[i, :7].tolist() + arm_path_array[i, 14:26].tolist() + arm_path_array[i, 7:14].tolist() + arm_path_array[i, 26:38].tolist()
        '''
        # set velocity
        dt = 1.0/15.0
        cur_pos = np.array(path_point.positions)
        path_point.velocities = ((cur_pos - last_pos) / dt).tolist()
        # set acceleration
        cur_vel = np.array(path_point.velocities)
        path_point.accelerations = ((cur_vel - last_vec) / dt).tolist()
        # save the current vel and acc
        last_pos = cur_pos
        last_vec = cur_vel
        last_acc = np.array(path_point.accelerations)
        # set time
        '''
        t = rospy.Time(t_delay + i*1.0/150.0) #rospy.Time(i*1.0/15.0) # rospy.Time(timestamp_array[i]) # 15 Hz # rospy.Time(0.5*i) #
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








