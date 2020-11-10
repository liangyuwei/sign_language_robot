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

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Linear mapping: (x - x_min) / (x_max - x_min) == (x_hat - x_min_hat) / (x_max_hat - x_min_hat)
def linear_map(x_min, x_max, x, x_min_hat, x_max_hat):
  return (x - x_min) / (x_max - x_min) * (x_max_hat - x_min_hat) + x_min_hat


### Process one path point 
def sr_linear_map(human_joint_angles):
  
  ## initialization
  sr_hand_dof = 22
  srhand_joint_angles = np.zeros(sr_hand_dof)

  ## preparation (from start to final: flexion and abduction!!! note the direction of motion!!!)
  abduction = 15 * math.pi / 180.0 # 0.35 max, approximately +-20 deg
  # SR hand - # FF, MF, RF, LF, TH
  # actually, one-to-one is not suitable for Shadowhand's thumb joints;
  # structural difference from the dataglove model... measured data of dataglove cannot be simply one-to-one mapped 
  sr_start = np.array([abduction, 0, 0, 0, \
                       abduction/2, 0, 0, 0, \
                       0, 0, 0, 0, \
                       0, 0, 0, 0, 0, \
                       0, 0, 0, 0, 0])
  sr_final = np.array([-abduction, 1.56, 1.56, 1.56, \
                       -abduction/2, 1.56, 1.56, 1.56, \
                       -abduction, 1.56, 1.56, 1.56, \
                        0, -abduction, 1.56, 1.56, 1.56, \
                        0, 1.21, 0, 0.69, 1.56]) 
  hm_start = np.array([0,    0, 53,  0,   0, 30,  0,   0, 22,  0,   0, 35,  0,   0,  0]) # modify index-middle abduction angle range to allow finger crossing..
            # np.array([0,    0, 53,  0,   0, 22,  0,   0, 22,  0,   0, 35,  0,   0,  0])
  hm_final = np.array([45, 100,  0, 90, 120,  0, 90, 120,  0, 90, 120,  0, 90, 120, 58]) # in dataglove 's sequence

  ## one-to-one joint matching: 2 joint(wrist) + 22 joint(hand) = 24 joint; 2 DOFs(wrist) + 18 DOFs(hand) = 20 DOFs
  # forefinger (abd/add -> flex/ext)
  srhand_joint_angles[0] = linear_map(hm_start[5], hm_final[5], human_joint_angles[5], sr_start[0], sr_final[0])
  srhand_joint_angles[1] = linear_map(hm_start[3], hm_final[3], human_joint_angles[3], sr_start[1], sr_final[1])
  srhand_joint_angles[2] = linear_map(hm_start[4], hm_final[4], human_joint_angles[4], sr_start[2], sr_final[2])
  srhand_joint_angles[3] = srhand_joint_angles[2] * 2.0 / 3.0 # two-thirds rule
  # middle finger
  srhand_joint_angles[4] = linear_map(hm_start[5], hm_final[5], human_joint_angles[5], 0.0, 0.0) # stick to neutral position
  srhand_joint_angles[5] = linear_map(hm_start[6], hm_final[6], human_joint_angles[6], sr_start[5], sr_final[5])
  srhand_joint_angles[6] = linear_map(hm_start[7], hm_final[7], human_joint_angles[7], sr_start[6], sr_final[6])
  srhand_joint_angles[7] = srhand_joint_angles[6] * 2.0 / 3.0 # two-thirds rule
  # ring finger
  srhand_joint_angles[8] = linear_map(hm_start[8], hm_final[8], human_joint_angles[8], sr_start[8], sr_final[8])
  srhand_joint_angles[9] = linear_map(hm_start[9], hm_final[9], human_joint_angles[9], sr_start[9], sr_final[9])
  srhand_joint_angles[10] = linear_map(hm_start[10], hm_final[10], human_joint_angles[10], sr_start[10], sr_final[10])
  srhand_joint_angles[11] = srhand_joint_angles[10] * 2.0 / 3.0 # two-thirds rule
  # little finger
  srhand_joint_angles[12] = 0.0 # joint between little finger and palm
  srhand_joint_angles[13] = linear_map(hm_start[11], hm_final[11], human_joint_angles[11], sr_start[13], sr_final[13])
  srhand_joint_angles[14] = linear_map(hm_start[12], hm_final[12], human_joint_angles[12], sr_start[14], sr_final[14])
  srhand_joint_angles[15] = linear_map(hm_start[13], hm_final[13], human_joint_angles[13], sr_start[15], sr_final[15])
  srhand_joint_angles[16] = srhand_joint_angles[15] * 2.0 / 3.0 # two-thirds rule
  # forefinger
  srhand_joint_angles[17] = 0.0 # fixed at 0, but there should be coupling...
  srhand_joint_angles[18] = linear_map(hm_start[14], hm_final[14], human_joint_angles[14], sr_start[18], sr_final[18])
  srhand_joint_angles[19] = 0.0 # fixed at 0, but there should be coupling...
  srhand_joint_angles[20] = linear_map(hm_start[1], hm_final[1], human_joint_angles[1], sr_start[20], sr_final[20])
  srhand_joint_angles[21] = linear_map(hm_start[0], hm_final[0], human_joint_angles[0], sr_start[21], sr_final[21])

  return srhand_joint_angles


### Arrange mapping for a whole joint path
def map_glove_to_srhand(human_hand_path):
  # input is of the size (N x 30), containing data of two hands

  # prep
  sr_hand_dof = 22
  sr_hand_path = np.zeros((human_hand_path.shape[0], 2*sr_hand_dof))

  import pdb
  pdb.set_trace()
  print(">>>> Iterate to process the input dataglove data...")
  # iterate to process
  for n in range(human_hand_path.shape[0]):
    print("== processing point {}/{}".format((n+1), human_hand_path.shape[0]))
    # left 
    sr_hand_path[n, :sr_hand_dof] = sr_linear_map(human_hand_path[n, :15])
    # right 
    sr_hand_path[n, sr_hand_dof:] = sr_linear_map(human_hand_path[n, 15:])

  return sr_hand_path


def main():

  file_name = "glove-calib-2020-11-02.h5"
  test_seq_name = 'test_finger_1/glove_angle' # 'test_finger_1_calibrated'
  use_old_version = False

  try:
    options, args = getopt.getopt(sys.argv[1:], "hf:t:o", ["help", "file-name=", "test-seq-name=", "old-version"])
  except getopt.GetoptError:
    sys.exit()

  for option, value in options:
    if option in ("-h", "--help"):
      print("Help:\n")
      print("   This script executes the IK results.\n")
      print("Arguments:\n")
      print("   -f, --file-name=, specify the name of the h5 file to read joint trajectory from, suffix is required.\n")
      print("   -t, --test-seq-name=, specify the name of the test sequence to visualize.\n")
      print("   -o, --old-version, specify to use the data collected by old version of dataglove (S14+).\n")
      exit(0)
    if option in ("-f", "--file-name"):
      print("Name of the h5 file to read joint trajectory from: {0}\n".format(value))
      file_name = value
    if option in ("-t", "--test-seq-name"):
      print("Name of the test sequence data: {0}\n".format(value))
      test_seq_name = value
    if option in ("-o", "--old-version"):
      print("Using the old version of dataglove data...")
      use_old_version = True


  try:

    ### Initialize a ros node
    rospy.init_node('yumi_controller_control_node')


    ### Set up a Publisher for connecting to controller
    arm_hand_pub = rospy.Publisher("/yumi/dual_arm_hand_joint_controller/command", JointTrajectory, queue_size=10)


    ### Set up a JointTrajectory
    arm_hand_cmd = JointTrajectory()
    arm_hand_cmd.joint_names = ['yumi_joint_1_l', 'yumi_joint_2_l', 'yumi_joint_7_l', 'yumi_joint_3_l', 'yumi_joint_4_l', 'yumi_joint_5_l', 'yumi_joint_6_l'] \
    + ['yumi_joint_1_r', 'yumi_joint_2_r', 'yumi_joint_7_r', 'yumi_joint_3_r', 'yumi_joint_4_r', 'yumi_joint_5_r', 'yumi_joint_6_r'] \
    + ['link1', 'link11', 'link2', 'link22', 'link3', 'link33', 'link4', 'link44', 'link5', 'link51', 'link52', 'link53'] \
    + ['Link1', 'Link11', 'Link2', 'Link22', 'Link3', 'Link33', 'Link4', 'Link44', 'Link5', 'Link51', 'Link52', 'Link53']
    waypoint = JointTrajectoryPoint()
    waypoint.positions = [-1.5, -2.0, 1.5, 0, 0, 0, 0] \
    + [1.5, -2.0, -1.5, 0, 0, 0, 0] \
    + [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] \
    + [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]     
    waypoint.time_from_start.secs = 1.0
    arm_hand_cmd.points.append(waypoint)
    
    arm_hand_pub.publish(arm_hand_cmd)

    rospy.spin()

  except rospy.ROSInterruptException:
    return

  except KeyboardInterrupt:
    return



if __name__ == '__main__':

  main()



