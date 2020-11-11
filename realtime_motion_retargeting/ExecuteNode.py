#!/usr/bin/python
import copy
import math
import time
import numpy as np
import numpy.matlib
import random
import h5py
import moveit_commander
import moveit_msgs.msg
from math import pi
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped
from realtime_motion_retargeting.msg import ControlMsg


from sensor_msgs.msg import JointState


import rospy
import actionlib

import sys
import pdb
from math import pi

import getopt # process the terminal arguments

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionGoal



class YumiControl():

  def __init__(self):
       
    ### Command controller via action server
    self.client = actionlib.SimpleActionClient("/yumi/dual_arm_hand_joint_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    print("== Waiting for action server... ")
    self.client.wait_for_server()

    ### Prep
    self.joint_names = ['yumi_joint_1_l', 'yumi_joint_2_l', 'yumi_joint_7_l', 'yumi_joint_3_l', 'yumi_joint_4_l', 'yumi_joint_5_l', 'yumi_joint_6_l'] \
    + ['yumi_joint_1_r', 'yumi_joint_2_r', 'yumi_joint_7_r', 'yumi_joint_3_r', 'yumi_joint_4_r', 'yumi_joint_5_r', 'yumi_joint_6_r'] \
    + ['link1', 'link11', 'link2', 'link22', 'link3', 'link33', 'link4', 'link44', 'link5', 'link51', 'link52', 'link53'] \
    + ['Link1', 'Link11', 'Link2', 'Link22', 'Link3', 'Link33', 'Link4', 'Link44', 'Link5', 'Link51', 'Link52', 'Link53']


  def arm_hand_action_control(self, q_goal):
    # JointTrajectory
    joint_goal = self.set_arm_hand_joint_goal(q_goal)
    # FollowJointTrajectoryActionGoal
    action_goal = FollowJointTrajectoryActionGoal()
    action_goal.goal.trajectory = joint_goal
    # send goal
    self.client.send_goal(action_goal.goal)


  def set_arm_hand_joint_goal(self, q_goal):
    # init
    arm_hand_cmd = JointTrajectory()
    waypoint = JointTrajectoryPoint()
    # joint names
    arm_hand_cmd.joint_names = self.joint_names
    # append waypoints
    waypoint.positions = q_goal
    waypoint.time_from_start = 0.01 #rospy.Time.now()
    # waypoint.time_from_start.secs = 0. #1.0
    arm_hand_cmd.points.append(waypoint)
    # return
    return arm_hand_cmd


class ExecuteNode:

    def callback(self, msg):
        print("[ExecuteNode] Enter execute node callback")
        # joint_goal = msg.l_arm_joint_angle + msg.l_hand_joint_angle \
        #     + msg.r_arm_joint_angle + msg.r_hand_joint_angle
        # self.moveit_group.go(joint_goal)


        q_hand_open = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] \
        + [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]  
        
        # arm
        q_arm_initial = [-1.5, -2.0, 1.5, 0, 0, 0, 0] \
        + [1.5, -2.0, -1.5, 0, 0, 0, 0] 
        
        q_arm_tpose = [-0.9, -1.3, 2.0, -1.6, 0, 0, 0] \
        + [0.9, -1.3, -2.0, -1.6, 0, 0, 0]  # palm down

        q_arm_tpose2 = [-0.9, -1.3, 2.0, -1.6, 0, 0, -3.0] \
        + [0.9, -1.3, -2.0, -1.6, 0, 0, 3.0] # palm up

        q_arm_hug = [-1.42, -0.5, 1.57, -1.0, 0, 0, -0.7] \
        + [1.42, -0.5, -1.57, -1.0, 0, 0, 0.7] 

        # q_goal = q_arm_hug + q_hand_open

        # calcualted results
        q_goal = msg.l_arm_joint_angle \
        + msg.r_arm_joint_angle \
        + msg.l_hand_joint_angle \
        + msg.r_hand_joint_angle

        print("q_goal={}".format(q_goal))

        ### 1 - Use action server
        # self.yumi_controller.arm_hand_action_control(
        #     msg.l_arm_joint_angle \
        #   + msg.r_arm_joint_angle \
        #   + msg.l_hand_joint_angle \
        #   + msg.r_hand_joint_angle
        # )


        ### 2 - Use topic
        # set up a JointTrajectory message
        arm_hand_cmd = JointTrajectory()
        arm_hand_cmd.joint_names = ['yumi_joint_1_l', 'yumi_joint_2_l', 'yumi_joint_7_l', 'yumi_joint_3_l', 'yumi_joint_4_l', 'yumi_joint_5_l', 'yumi_joint_6_l'] \
        + ['yumi_joint_1_r', 'yumi_joint_2_r', 'yumi_joint_7_r', 'yumi_joint_3_r', 'yumi_joint_4_r', 'yumi_joint_5_r', 'yumi_joint_6_r'] \
        + ['link1', 'link11', 'link2', 'link22', 'link3', 'link33', 'link4', 'link44', 'link5', 'link51', 'link52', 'link53'] \
        + ['Link1', 'Link11', 'Link2', 'Link22', 'Link3', 'Link33', 'Link4', 'Link44', 'Link5', 'Link51', 'Link52', 'Link53']
        waypoint = JointTrajectoryPoint()
        waypoint.positions = q_goal
        waypoint.time_from_start.secs = 0.1 # 0.01 #rospy.Time.now() - self.t0
        arm_hand_cmd.points.append(waypoint)
        arm_hand_cmd = self.yumi_controller.set_arm_hand_joint_goal(q_goal)
        self.arm_hand_pub.publish(arm_hand_cmd)


        ### 3 - Use joint_states
        '''
        joint_state_cmd = JointState()
        joint_state_cmd.header.stamp = rospy.Time.now()
        joint_state_cmd.name = ['yumi_joint_1_l', 'yumi_joint_2_l', 'yumi_joint_7_l', 'yumi_joint_3_l', 'yumi_joint_4_l', 'yumi_joint_5_l', 'yumi_joint_6_l'] \
        + ['yumi_joint_1_r', 'yumi_joint_2_r', 'yumi_joint_7_r', 'yumi_joint_3_r', 'yumi_joint_4_r', 'yumi_joint_5_r', 'yumi_joint_6_r'] \
        + ['link1', 'link11', 'link2', 'link22', 'link3', 'link33', 'link4', 'link44', 'link5', 'link51', 'link52', 'link53'] \
        + ['Link1', 'Link11', 'Link2', 'Link22', 'Link3', 'Link33', 'Link4', 'Link44', 'Link5', 'Link51', 'Link52', 'Link53']
        joint_state_cmd.position = q_goal
        joint_state_cmd.velocity = []
        joint_state_cmd.effort = []
        self.joint_state_pub.publish(joint_state_cmd)
        '''


    def runExecuteNode(self):
        # Init node
        rospy.init_node('executeNode', anonymous=True)
        self.t0 = rospy.Time.now()
        # Init YuMi Control class
        self.yumi_controller = YumiControl()

        # topic
        self.arm_hand_pub = rospy.Publisher("/yumi/dual_arm_hand_joint_controller/command", JointTrajectory, queue_size=1)

        # joint_state publisher
        self.joint_state_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)


        # # Init moveit group
        # self.moveit_group = moveit_commander.MoveGroupCommander("dual_arms_with_hands")
        # self.moveit_group.allow_replanning(True)

        # Init subscriber
        rospy.Subscriber('cmdPublisher', ControlMsg, self.callback, queue_size=1)
        rospy.spin()



if __name__=="__main__":
    node = ExecuteNode()
    node.runExecuteNode()