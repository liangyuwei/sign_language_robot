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



class YuMiMoveItControl():

  def __init__(self):
    self.left_arm_group = moveit_commander.MoveGroupCommander("left_arm")
    self.right_arm_group = moveit_commander.MoveGroupCommander("right_arm")

    # allow replanning to solve tough cases
    # self.left_arm_group.allow_replanning(True)
    # self.right_arm_group.allow_replanning(True)


  def go_simul(self, q_goal_l, q_goal_r, wait_for=True):
    self.left_arm_group.go(q_goal_l, wait=wait_for)
    self.right_arm_group.go(q_goal_r, wait=wait_for) # wait?
    
    self.left_arm_group.stop()
    self.right_arm_group.stop()


  def go_separate(self, q_goal, left_or_right):
    if left_or_right:
      self.left_arm_group.go(q_goal, wait=True)
      self.left_arm_group.stop()
    else:
      self.right_arm_group.go(q_goal, wait=True)
      self.right_arm_group.stop()


  def go_pose_simul(self, pos_l, pos_r, wait_for=True):
    ## set pose goal
    # l
    pose_goal_l = geometry_msgs.msg.Pose()
    pose_goal_l.orientation.x = 0.707 #0
    pose_goal_l.orientation.y = 0 #0
    pose_goal_l.orientation.z = 0.707 #0
    pose_goal_l.orientation.w = 0 #1 
    pose_goal_l.position.x = pos_l[0]
    pose_goal_l.position.y = pos_l[1]
    pose_goal_l.position.z = pos_l[2]
    # r
    pose_goal_r = geometry_msgs.msg.Pose()
    pose_goal_r.orientation.x = 0.707 #0
    pose_goal_r.orientation.y = 0 #0
    pose_goal_r.orientation.z = 0.707 #0
    pose_goal_r.orientation.w = 0 #1 
    pose_goal_r.position.x = pos_r[0]
    pose_goal_r.position.y = pos_r[1]
    pose_goal_r.position.z = pos_r[2]

    ## go to pose goal
    # set pose target
    self.left_arm_group.set_pose_target(pose_goal_l)
    self.right_arm_group.set_pose_target(pose_goal_r)
    # go 
    self.left_arm_group.go(wait=wait_for)
    self.left_arm_group.stop()

    self.right_arm_group.go(wait=wait_for) # wait ?
    self.right_arm_group.stop()

    self.left_arm_group.clear_pose_targets()
    self.right_arm_group.clear_pose_targets()
    

  def compute_cart_path_simul(self, waypoints_pos_l, waypoints_pos_r):
    ## waypoints_pos_{l/r} is of the size N x 3
    ## *** Exclude the first point in the waypoint list!!! ***

    ## Go to the first point
    # import pdb
    # pdb.set_trace()
    # self.go_pose_simul(waypoints_pos_l[0, :], waypoints_pos_r[0, :], wait_for=True) # wait for the execution to complete before proceeding

    ## Prepare waypoints
    wpose = geometry_msgs.msg.Pose()
    # left
    waypoints_l = []
    # for n in range(1, waypoints_pos_l.shape[0]): # exclude the first point!!!
    for n in range(waypoints_pos_l.shape[0]): # exclude the first point!!!
      wpose.position.x = waypoints_pos_l[n, 0]
      wpose.position.y = waypoints_pos_l[n, 1]
      wpose.position.z = waypoints_pos_l[n, 2]
      wpose.orientation.x = 0.707 #0
      wpose.orientation.y = 0 #0
      wpose.orientation.z = 0.707 #0
      wpose.orientation.w = 0 #1 
      waypoints_l.append(copy.deepcopy(wpose))
    # right
    waypoints_r = []
    # for n in range(1, waypoints_pos_r.shape[0]): # exclude the first point!!!
    for n in range(waypoints_pos_r.shape[0]): # exclude the first point!!!
      wpose.position.x = waypoints_pos_r[n, 0]
      wpose.position.y = waypoints_pos_r[n, 1]
      wpose.position.z = waypoints_pos_r[n, 2]
      wpose.orientation.x = 0.707 #0
      wpose.orientation.y = 0 #0
      wpose.orientation.z = 0.707 #0
      wpose.orientation.w = 0 #1 
      waypoints_r.append(copy.deepcopy(wpose))

    ## Compute Cartesian plan
    print(">> Planning for left arm...")
    (new_plan_l, fraction) = self.left_arm_group.compute_cartesian_path(
                                        waypoints_l,   # waypoints to follow
                                        0.01, #0.01,        # eef_step # set to 0.001 for collecting the data
                                        0.0,     # jump_threshold
                                        avoid_collisions=True)    
    print("Done!")
    print(">> Planning for right arm...")
    (new_plan_r, fraction) = self.right_arm_group.compute_cartesian_path(
                                        waypoints_r,   # waypoints to follow
                                        0.01, #0.01,        # eef_step # set to 0.001 for collecting the data
                                        0.0,     # jump_threshold
                                        avoid_collisions=True)    
    print("Done!")

    return new_plan_l, new_plan_r


  def execute_plan(self, plan_l, plan_r, wait_for=True):
    self.left_arm_group.execute(plan_l, wait=wait_for) # wait ?
    self.right_arm_group.execute(plan_r, wait=wait_for) 



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
    q_arm_initial_l = [-1.5, -2.0, 1.5, 0, 0, 0, 0] 
    q_arm_initial_r = [1.5, -2.0, -1.5, 0, 0, 0, 0] 
    
    q_arm_tpose_l = [-0.9, -1.3, 2.0, -1.6, 0, 0, 0] 
    q_arm_tpose_r = [0.9, -1.3, -2.0, -1.6, 0, 0, 0]  # palm down

    q_arm_tpose2_l = [-0.9, -1.3, 2.0, -1.6, 0, 0, -3.0] 
    q_arm_tpose2_r = [0.9, -1.3, -2.0, -1.6, 0, 0, 3.0] # palm up

    q_arm_hug_l = [-1.42, -0.5, 1.57, -1.0, 0, 0, -0.7] 
    q_arm_hug_r = [1.42, -0.5, -1.57, -1.0, 0, 0, 0.7] 


    #### Initialize the class
    yumi_moveit_control = YuMiMoveItControl()


    ### Go method (It turns out that go method cannot control two groups simultaneously)
    yumi_moveit_control.go_simul(q_arm_tpose_l, q_arm_tpose_r, wait_for=True)

    '''
    # choose from the above settings
    import pdb
    pdb.set_trace()
    yumi_moveit_control.go_simul(q_arm_initial_l, q_arm_hug_r, wait_for=False)

    pdb.set_trace()
    yumi_moveit_control.go_simul(q_arm_tpose_l, q_arm_tpose_r, wait_for=False)

    pdb.set_trace()
    yumi_moveit_control.go_simul(q_arm_hug_l, q_arm_initial_r, wait_for=False)
    '''


    ### Compute Cart Path method
    # pre-selected positions
    p_l_middle = [0.515, 0.171, 0.513]
    p_l_away_low = [0.492, 0.31, 0.28]
    p_l_away_high = [0.42, 0.45, 0.475]

    p_r_middle = [0.56, -0.17, 0.5]
    p_r_away_low = [0.564, -0.254, 0.3]
    p_r_away_high = [0.388, -0.33, 0.43]

    # set up waypoints
    waypoints_l = np.array([p_l_middle, p_l_away_low, p_l_away_high])
    waypoints_r = np.array([p_r_middle, p_r_away_low, p_r_away_high])

    import pdb
    pdb.set_trace()

    # compute cartesian plan
    (plan_l, plan_r) = yumi_moveit_control.compute_cart_path_simul(waypoints_l, waypoints_r)


    # execute the computed plans
    pdb.set_trace()
    yumi_moveit_control.execute_plan(plan_l, plan_r, False)


  except rospy.ROSInterruptException:
    return

  except KeyboardInterrupt:
    return



if __name__ == '__main__':

  main()



