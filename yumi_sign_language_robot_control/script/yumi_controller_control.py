#!/usr/bin/env python

import rospy
import actionlib

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

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionGoal



class YumiControl():

  def __init__(self):
    
    ### 1 - Command controller via topic
    
    # set up a publisher
    # self.arm_hand_pub = rospy.Publisher("/yumi/dual_arm_hand_joint_controller/command", JointTrajectory, queue_size=10)
    self.arm_pub = rospy.Publisher("/yumi/dual_arm_joint_controller/command", JointTrajectory, queue_size=10)
    self.hand_pub = rospy.Publisher("/yumi/dual_hand_joint_controller/command", JointTrajectory, queue_size=10)


    ### 2 - Command controller via action server
    self.arm_client = actionlib.SimpleActionClient("/yumi/dual_arm_joint_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    self.arm_client.wait_for_server()
    self.hand_client = actionlib.SimpleActionClient("/yumi/dual_hand_joint_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    self.hand_client.wait_for_server()

    ### Prep
    # self.joint_names = ['yumi_joint_1_l', 'yumi_joint_2_l', 'yumi_joint_7_l', 'yumi_joint_3_l', 'yumi_joint_4_l', 'yumi_joint_5_l', 'yumi_joint_6_l'] \
    # + ['yumi_joint_1_r', 'yumi_joint_2_r', 'yumi_joint_7_r', 'yumi_joint_3_r', 'yumi_joint_4_r', 'yumi_joint_5_r', 'yumi_joint_6_r'] \
    # + ['link1', 'link11', 'link2', 'link22', 'link3', 'link33', 'link4', 'link44', 'link5', 'link51', 'link52', 'link53'] \
    # + ['Link1', 'Link11', 'Link2', 'Link22', 'Link3', 'Link33', 'Link4', 'Link44', 'Link5', 'Link51', 'Link52', 'Link53']
    self.arm_joint_names = ['yumi_joint_1_l', 'yumi_joint_2_l', 'yumi_joint_7_l', 'yumi_joint_3_l', 'yumi_joint_4_l', 'yumi_joint_5_l', 'yumi_joint_6_l'] \
    + ['yumi_joint_1_r', 'yumi_joint_2_r', 'yumi_joint_7_r', 'yumi_joint_3_r', 'yumi_joint_4_r', 'yumi_joint_5_r', 'yumi_joint_6_r'] 
    self.hand_joint_names = ['link1', 'link11', 'link2', 'link22', 'link3', 'link33', 'link4', 'link44', 'link5', 'link51', 'link52', 'link53'] \
    + ['Link1', 'Link11', 'Link2', 'Link22', 'Link3', 'Link33', 'Link4', 'Link44', 'Link5', 'Link51', 'Link52', 'Link53']

  '''
  def topic_control(self, q_goal):
    # set up a JointTrajectory msg
    arm_hand_cmd = self.set_arm_hand_joint_goal(q_goal)

    # publish
    # rate = rospy.Rate(10)
    # while not rospy.is_shutdown():
    self.arm_hand_pub.publish(arm_hand_cmd)
    # arm_hand_pub.publish(arm_hand_cmd)
      # rate.sleep()
  '''


  def arm_topic_control(self, q_goal):
    # set up a JointTrajectory msg
    arm_cmd = self.set_arm_joint_goal(q_goal)

    # publish
    # rate = rospy.Rate(10)
    # while not rospy.is_shutdown():
    self.arm_pub.publish(arm_cmd)
    # arm_hand_pub.publish(arm_hand_cmd)
      # rate.sleep()


  def hand_topic_control(self, q_goal):
    # set up a JointTrajectory msg
    hand_cmd = self.set_hand_joint_goal(q_goal)

    # publish
    # rate = rospy.Rate(10)
    # while not rospy.is_shutdown():
    self.hand_pub.publish(hand_cmd)
    # arm_hand_pub.publish(arm_hand_cmd)
      # rate.sleep()

  '''
  def action_control(self, q_goal):
    print("== Waiting for action server... ")
    self.client.wait_for_server()
    # JointTrajectory
    joint_goal = self.set_arm_hand_joint_goal(q_goal)
    # FollowJointTrajectoryActionGoal
    action_goal = FollowJointTrajectoryActionGoal()
    action_goal.goal.trajectory = joint_goal
    # send goal
    self.client.send_goal(action_goal.goal)
    self.client.wait_for_result()
  '''

  def arm_hand_action_control(self, q_arm_goal, q_hand_goal):
    print("== Waiting for action server... ")
    self.arm_client.wait_for_server()
    self.hand_client.wait_for_server()

    # JointTrajectory
    arm_joint_goal = self.set_arm_joint_goal(q_arm_goal)
    hand_joint_goal = self.set_arm_joint_goal(q_hand_goal)

    # FollowJointTrajectoryActionGoal
    arm_action_goal = FollowJointTrajectoryActionGoal()
    arm_action_goal.goal.trajectory = arm_joint_goal
    hand_action_goal = FollowJointTrajectoryActionGoal()
    hand_action_goal.goal.trajectory = hand_joint_goal
    
    # send goals
    self.arm_client.send_goal(arm_action_goal.goal)
    self.hand_client.send_goal(hand_action_goal.goal)

    # wait for results
    self.arm_client.wait_for_result()
    self.hand_client.wait_for_result()


  def arm_action_control(self, q_goal):
    print("== Waiting for action server... ")
    self.arm_client.wait_for_server()
    # JointTrajectory
    joint_goal = self.set_arm_joint_goal(q_goal)
    # FollowJointTrajectoryActionGoal
    action_goal = FollowJointTrajectoryActionGoal()
    action_goal.goal.trajectory = joint_goal
    # send goal
    self.arm_client.send_goal(action_goal.goal)
    self.arm_client.wait_for_result()


  def hand_action_control(self, q_goal):
    print("== Waiting for action server... ")
    self.hand_client.wait_for_server()
    # JointTrajectory
    joint_goal = self.set_hand_joint_goal(q_goal)
    import pdb
    pdb.set_trace()
    # FollowJointTrajectoryActionGoal
    action_goal = FollowJointTrajectoryActionGoal()
    action_goal.goal.trajectory = joint_goal
    # send goal
    self.hand_client.send_goal(action_goal.goal)
    self.hand_client.wait_for_result()


  '''
  def set_arm_hand_joint_goal(self, q_goal):
    # init
    arm_hand_cmd = JointTrajectory()
    waypoint = JointTrajectoryPoint()
    # joint names
    arm_hand_cmd.joint_names = self.joint_names
    # append waypoints
    waypoint.positions = q_goal
    waypoint.time_from_start.secs = 1.0
    arm_hand_cmd.points.append(waypoint)
    # return
    return arm_hand_cmd
  '''

  def set_arm_joint_goal(self, q_goal):
    # init
    arm_cmd = JointTrajectory()
    waypoint = JointTrajectoryPoint()
    # joint names
    arm_cmd.joint_names = self.arm_joint_names
    # append waypoints
    waypoint.positions = q_goal
    waypoint.time_from_start.secs = 1.0
    arm_cmd.points.append(waypoint)
    # return
    return arm_cmd


  def set_hand_joint_goal(self, q_goal):
    # init
    hand_cmd = JointTrajectory()
    waypoint = JointTrajectoryPoint()
    # joint names
    hand_cmd.joint_names = self.hand_joint_names
    # append waypoints
    waypoint.positions = q_goal
    waypoint.time_from_start.secs = 1.0
    hand_cmd.points.append(waypoint)
    # return
    return hand_cmd


def main():

  try:

    ### Initialize a ros node
    rospy.init_node('yumi_controller_control_node')


    ### Prep
    # hand
    q_hand_open = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] \
    + [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]  

    q_hand_close = [-1.57, -1.57, -1.57, -1.57, -1.57, -1.57, -1.57, -1.57, 0, 0, 0, 0] \
    + [-1.57, -1.57, -1.57, -1.57, -1.57, -1.57, -1.57, -1.57, 0, 0, 0, 0] 

    q_hand_thumb_flat = [0, 0, 0, 0, 0, 0, 0, 0, 0.25, 0, 0, 0] \
    + [0, 0, 0, 0, 0, 0, 0, 0, 0.25, 0, 0, 0] 

    q_hand_thumb_rot = [0, 0, 0, 0, 0, 0, 0, 0, -0.95, 0, 0, 0] \
    + [0, 0, 0, 0, 0, 0, 0, 0, -0.95, 0, 0, 0] 

    # arm
    q_arm_initial = [-1.5, -2.0, 1.5, 0, 0, 0, 0] \
    + [1.5, -2.0, -1.5, 0, 0, 0, 0] 
    
    q_arm_tpose = [-0.9, -1.3, 2.0, -1.6, 0, 0, 0] \
    + [0.9, -1.3, -2.0, -1.6, 0, 0, 0]  # palm down

    q_arm_tpose2 = [-0.9, -1.3, 2.0, -1.6, 0, 0, -3.0] \
    + [0.9, -1.3, -2.0, -1.6, 0, 0, 3.0] # palm up

    q_arm_hug = [-1.42, -0.5, 1.57, -1.0, 0, 0, -0.7] \
    + [1.42, -0.5, -1.57, -1.0, 0, 0, 0.7] 


    ### Set up the control class
    yumi_control = YumiControl()


    ### Move to some pre-defined poses for debugging

    # move to an initial state
    yumi_control.arm_action_control(q_arm_hug)

    import pdb
    # arm-hand control
    pdb.set_trace()
    yumi_control.arm_hand_action_control(q_arm_initial, q_hand_open)

    pdb.set_trace()
    yumi_control.arm_hand_action_control(q_arm_hug, q_hand_close)

    # arm control
    pdb.set_trace()
    yumi_control.arm_action_control(q_arm_initial)

    pdb.set_trace()
    yumi_control.arm_action_control(q_arm_tpose)

    pdb.set_trace()
    yumi_control.arm_action_control(q_arm_tpose2)

    pdb.set_trace()
    yumi_control.arm_action_control(q_arm_hug)

    # hand control
    pdb.set_trace()
    yumi_control.hand_action_control(q_hand_open)

    pdb.set_trace()
    yumi_control.hand_action_control(q_hand_close)

    pdb.set_trace()


    ### 1 - Command controller via topic
    '''
    # set up a publisher
    arm_hand_pub = rospy.Publisher("/yumi/dual_arm_hand_joint_controller/command", JointTrajectory, queue_size=10)
    # set up a JointTrajectory message
    # arm_hand_cmd = JointTrajectory()
    # arm_hand_cmd.joint_names = ['yumi_joint_1_l', 'yumi_joint_2_l', 'yumi_joint_7_l', 'yumi_joint_3_l', 'yumi_joint_4_l', 'yumi_joint_5_l', 'yumi_joint_6_l'] \
    # + ['yumi_joint_1_r', 'yumi_joint_2_r', 'yumi_joint_7_r', 'yumi_joint_3_r', 'yumi_joint_4_r', 'yumi_joint_5_r', 'yumi_joint_6_r'] \
    # + ['link1', 'link11', 'link2', 'link22', 'link3', 'link33', 'link4', 'link44', 'link5', 'link51', 'link52', 'link53'] \
    # + ['Link1', 'Link11', 'Link2', 'Link22', 'Link3', 'Link33', 'Link4', 'Link44', 'Link5', 'Link51', 'Link52', 'Link53']
    # waypoint = JointTrajectoryPoint()
    # waypoint.positions = q_initial
    # waypoint.time_from_start.secs = 1.0
    # arm_hand_cmd.points.append(waypoint)
    arm_hand_cmd = set_arm_hand_joint_goal(q_initial)
    # publish
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
      arm_hand_pub.publish(arm_hand_cmd)
      rate.sleep()
    '''

    ### 2 - Command controller via action server
    '''
    client = actionlib.SimpleActionClient("/yumi/dual_arm_hand_joint_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    client.wait_for_server()
    # JointTrajectory
    joint_goal = set_arm_hand_joint_goal(q_tpose)#(q_initial)
    # FollowJointTrajectoryActionGoal
    action_goal = FollowJointTrajectoryActionGoal()
    import pdb
    pdb.set_trace()
    action_goal.goal.trajectory = joint_goal
    # send goal
    client.send_goal(action_goal.goal)
    client.wait_for_result()
    '''


  except rospy.ROSInterruptException:
    return

  except KeyboardInterrupt:
    return



if __name__ == '__main__':

  main()



