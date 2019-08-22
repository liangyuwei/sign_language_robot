#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt! interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. (More on these below)
##
## We also import `rospy`_ and some messages that we will use:
##

import sys
import copy
import rospy
import math
import tf
import numpy as np
import numpy.matlib
import h5py
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from raw_totg.srv import *
## END_SUB_TUTORIAL

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

class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)

    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    ## the robot:
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the Panda
    ## arm so we set ``group_name = panda_arm``. If you are using a different robot,
    ## you should change this value to the name of your robot arm planning group.
    ## This interface can be used to plan and execute motions on the Panda:
    group_name = "dual_arms"
    group = moveit_commander.MoveGroupCommander(group_name)

    ## We create a `DisplayTrajectory`_ publisher which is used later to publish
    ## trajectories for RViz to visualize:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## END_SUB_TUTORIAL

    ## BEGIN_SUB_TUTORIAL basic_info
    ##
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

  def go_to_joint_state(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    joint_goal = group.get_current_joint_values()
    ''
    joint_goal[0] = 0#0.0956#-5.9604 #0 
    joint_goal[1] = 0#0.3016#-0.2112 #0
    joint_goal[2] = 0#-0.3907#0.0584 #0
    joint_goal[3] = 0#0.9673#-1.8980 #0
    joint_goal[4] = 0#0.7815#2.4066 #0
    joint_goal[5] = 0#-1.4376#2.0123 # 0
    ''
    joint_goal[6] = 0
    joint_goal[7] = 0
    joint_goal[8] = 0
    joint_goal[9] = 0
    joint_goal[10] = 0
    joint_goal[11] = 0

    '''
    joint_goal[0] = 0
    joint_goal[1] = math.pi/4
    joint_goal[2] = -math.pi/2
    joint_goal[3] = 0
    joint_goal[4] = -math.pi/2
    joint_goal[5] = 0
    ''
    joint_goal[6] = -math.pi
    joint_goal[7] = math.pi * 0.75
    joint_goal[8] = math.pi/2
    joint_goal[9] = math.pi
    joint_goal[10] = math.pi/2
    joint_goal[11] = 0
    '''

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()


    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def go_to_pose_goal(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.5
    pose_goal.position.y = -0.15
    pose_goal.position.z = 0.31 
    group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)


  def plan_cartesian_path(self, mid_point, scale=1):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    ## BEGIN_SUB_TUTORIAL plan_cartesian_path
    ##
    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through:
    ##
    waypoints = []

    wpose = group.get_current_pose().pose

    # first mid point
    wpose.position.x = scale * mid_point[0] # scale * 0.2  
    wpose.position.y = scale * mid_point[1] # scale * 0.0 # move to the middle
    wpose.position.z = scale * mid_point[2] # scale * 0.315  # minimum height
    wpose.orientation.x = 0
    wpose.orientation.y = 0.707
    wpose.orientation.z = 0
    wpose.orientation.w = 0.707
    waypoints.append(copy.deepcopy(wpose))

    # goal 
    wpose.position.x = scale * 0.5
    wpose.position.y = scale * 0.35
    wpose.position.z = scale * 0.315
    wpose.orientation.x = 0
    wpose.orientation.y = 0.707
    wpose.orientation.z = 0
    wpose.orientation.w = 0.707
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01, #0.01,        # eef_step # set to 0.001 for collecting the data
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

    ## END_SUB_TUTORIAL

  def display_trajectory(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    ## BEGIN_SUB_TUTORIAL display_trajectory
    ##
    ## Displaying a Trajectory
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again):
    ##
    ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    ## We populate the trajectory_start with our current robot state to copy over
    ## any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory);

    ## END_SUB_TUTORIAL

  def execute_plan(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    ## BEGIN_SUB_TUTORIAL execute_plan
    ##
    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    group.execute(plan, wait=True)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
    ## END_SUB_TUTORIAL

  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL wait_for_scene_update
    ##
    ## Ensuring Collision Updates Are Receieved
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## If the Python node dies before publishing a collision object update message, the message
    ## could get lost and the box will not appear. To ensure that the updates are
    ## made, we wait until we see the changes reflected in the
    ## ``get_known_object_names()`` and ``get_known_object_names()`` lists.
    ## For the purpose of this tutorial, we call this function after adding,
    ## removing, attaching or detaching an object in the planning scene. We then wait
    ## until the updates have been made or ``timeout`` seconds have passed
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False
    ## END_SUB_TUTORIAL

  def add_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene at the location of the left finger:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "_leftfinger"
    box_pose.pose.orientation.w = 1.0
    box_name = "box"
    scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))

    ## END_SUB_TUTORIAL
    # Copy local variables back to class variables. In practice, you should use the class
    # variables directly unless you have a good reason not to.
    self.box_name=box_name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)
    # return the value that is returned by .wait_for_state_update()

  def add_object(self, object_name, pos, size, frame_id, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = object_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene at the location of the left finger:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = frame_id
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = pos[0]
    box_pose.pose.position.y = pos[1]
    box_pose.pose.position.z = pos[2]
    box_name = object_name
    scene.add_box(box_name, box_pose, size=size)

    ## END_SUB_TUTORIAL
    # Copy local variables back to class variables. In practice, you should use the class
    # variables directly unless you have a good reason not to.
    self.box_name=box_name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)
    # return the value that is returned by .wait_for_state_update()

  def attach_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    robot = self.robot
    scene = self.scene
    eef_link = self.eef_link
    group_names = self.group_names

    ## BEGIN_SUB_TUTORIAL attach_object
    ##
    ## Attaching Objects to the Robot
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
    ## robot be able to touch them without the planning scene reporting the contact as a
    ## collision. By adding link names to the ``touch_links`` array, we are telling the
    ## planning scene to ignore collisions between those links and the box. For the Panda
    ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
    ## you should change this value to the name of your end effector group name.
    grasping_group = 'panda_hand'
    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(eef_link, box_name, touch_links=touch_links)
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

  def detach_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene
    eef_link = self.eef_link

    ## BEGIN_SUB_TUTORIAL detach_object
    ##
    ## Detaching Objects from the Robot
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## We can also detach and remove the object from the planning scene:
    scene.remove_attached_object(eef_link, name=box_name)
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

  def remove_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL remove_object
    ##
    ## Removing Objects from the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## We can remove the box from the world.
    scene.remove_world_object(box_name)

    ## **Note:** The object must be detached before we can remove it from the world
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

  def remove_object(self, object_name, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = object_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL remove_object
    ##
    ## Removing Objects from the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## We can remove the box from the world.
    scene.remove_world_object(box_name)

    ## **Note:** The object must be detached before we can remove it from the world
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)


  def go_left_goal(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = 0
    pose_goal.orientation.y = 0
    pose_goal.orientation.z = 0
    pose_goal.orientation.w = 1
    # crucial, the initial point
    pose_goal.position.x = 0.20#0.5
    pose_goal.position.y = 0#-0.35
    pose_goal.position.z = 0.285 # the closet point between ee_link and the bottom surface
    group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.group.get_current_pose().pose
    return plan, all_close(pose_goal, current_pose, 0.01)


  def go_right_goal(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = 0
    pose_goal.orientation.y = 0.707
    pose_goal.orientation.z = 0
    pose_goal.orientation.w = 0.707
    pose_goal.position.x = 0.5
    pose_goal.position.y = 0.35
    pose_goal.position.z = 0.315 # crucial, goal point
    group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.group.get_current_pose().pose
    return plan, all_close(pose_goal, current_pose, 0.01)

#  def add_mesh(self, mesh_name, pose, file_path, size=[1, 1, 1]):

#    pose = geometry_msgs.msg.PoseStamped()
#    pose.header.frame_id = "world"
#    pose.pose.orientation.w = 1.0
#    pose.pose.position.x = 0.4
#    pose.pose.position.y = 0.0
#    pose.pose.position.z = 0.4
#    file_name = "/home/liangyuwei/dual_ur5_ws/src/dual_ur5_control/meshes/flash_body.STL"
#    mesh_name = "flash_body"

#    self.scene.add_mesh(mesh_name, pose, file_path, size)

  def merge_two_traj(self, x1, t1, x2, t2, dim):
    # liangyuwei: This function merges two differently time-parameterized trajectories by linear interpolation.
    # dim - the dimension of x1 and x2
    
    # Merge the time sequences
    t1 = np.array(t1)
    t2 = np.array(t2)
    t_merged = np.concatenate((t1, t2)) 
    t_merged = np.unique(t_merged) # .unique returns the sorted unique elements of an array

    # Merge different dimensions of x1 and x2
    #for i in range(dim):
      
    
    

    #return x1_merged, x2_merged

  def plan_motion(self, x_start, w_start, x_mid, w_mid, x_final, w_final, planning_time, left_or_right_eef):
    # liangyuwei: This function computes a cartesian path with regard to the specified starting point, via point and the final point.



    ### Set initial pose
    print "== Set initial pose =="
    if (left_or_right_eef): # 1 or left eff
      group = moveit_commander.MoveGroupCommander("left_arm")
      left_pose_target = group.get_current_pose('left_ee_link')
      left_pose_target.pose.position.x = x_start[0]
      left_pose_target.pose.position.y = x_start[1]
      left_pose_target.pose.position.z = x_start[2]
      left_pose_target.pose.orientation.x = w_start[0]
      left_pose_target.pose.orientation.y = w_start[1]
      left_pose_target.pose.orientation.z = w_start[2]
      left_pose_target.pose.orientation.w = w_start[3]
      group.set_pose_target(left_pose_target, 'left_ee_link')
    else:
      group = moveit_commander.MoveGroupCommander("right_arm")
      right_pose_target = group.get_current_pose('right_ee_link')
      right_pose_target.pose.position.x = x_start[0]
      right_pose_target.pose.position.y = x_start[1]
      right_pose_target.pose.position.z = x_start[2]
      right_pose_target.pose.orientation.x = w_start[0]
      right_pose_target.pose.orientation.y = w_start[1]
      right_pose_target.pose.orientation.z = w_start[2]
      right_pose_target.pose.orientation.w = w_start[3]
      group.set_pose_target(right_pose_target, 'right_ee_link')
    # set planning time
    group.set_planning_time(planning_time)
    # plan
    plan = group.go(wait=True)
    # stop
    group.stop()
    # clear targets
    group.clear_pose_targets()

    ### Set via point
    print "== Set via point =="
    waypoints = []
    wpose = group.get_current_pose().pose
    # first mid point
    wpose.position.x = x_mid[0] # scale * 0.2  
    wpose.position.y = x_mid[1] # scale * 0.0 # move to the middle
    wpose.position.z = x_mid[2] # scale * 0.315  # minimum height
    wpose.orientation.x = w_mid[0]
    wpose.orientation.y = w_mid[1]
    wpose.orientation.z = w_mid[2]
    wpose.orientation.w = w_mid[3]
    waypoints.append(copy.deepcopy(wpose))

    ### Set the final point 
    print "== Set final pose =="
    wpose.position.x = x_final[0]
    wpose.position.y = x_final[1]
    wpose.position.z = x_final[2]
    wpose.orientation.x = w_final[0]
    wpose.orientation.y = w_final[1]
    wpose.orientation.z = w_final[2]
    wpose.orientation.w = w_final[3]
    waypoints.append(copy.deepcopy(wpose))

    ### Compute a cartesian path
    print "== Compute a cartesian path =="
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.001, #0.01,        # eef_step # set to 0.001 for collecting the data
                                       0.0,     # jump_threshold
                                       avoid_collisions=False)         

    ### Execute the plan
    print "== Execute the plan =="
    self.execute_plan(plan)


    return plan 


def add_time_optimal_parameterization_client(path, vel_limits, acc_limits, timestep=0.001):
  # wait for service to come online
  rospy.wait_for_service('add_time_optimal_parameterization_server')

  try:
    path_to_traj = rospy.ServiceProxy('add_time_optimal_parameterization_server', PathToTraj)
    res = path_to_traj(path, vel_limits, acc_limits, timestep)
    return res.traj
  except rospy.ServiceException, e:
    print "Service call failed: %s" % e


def merge_two_plans(points_l, points_r):
  ### points_l and points_r should be trajectory_msgs/JointTrajectoryPoint[]

  # initialization
  if len(points_l) > len(points_r):
    points_dual = copy.deepcopy(points_r)
  else:
    points_dual = copy.deepcopy(points_l)    

  # appoint
  for i in range(len(points_dual)):
    # only pos info is needed for TOTG
    points_dual[i].positions = points_l[i].positions + points_r[i].positions
    #points_dual[i].velocities = points_l[i].velocities + points_r[i].velocities
    #points_dual[i].accelerations = points_l[i].accelerations + points_r[i].accelerations

  return copy.deepcopy(points_dual)


def split_dual_plan(points_dual):
  ### points_dual should be trajectory_msgs/JointTrajectoryPoint[]

  # initialization
  points_l = copy.deepcopy(points_dual)
  points_r = copy.deepcopy(points_dual)

  # split
  for i in range(len(points_dual)):
    # for left arm
    points_l[i].positions = points_dual[i].positions[0:6]
    points_l[i].velocities = points_dual[i].velocities[0:6]
    points_l[i].accelerations = points_dual[i].accelerations[0:6]

    # for right arm
    points_r[i].positions = points_dual[i].positions[6:12]    
    points_r[i].velocities = points_dual[i].velocities[6:12]    
    points_r[i].accelerations = points_dual[i].accelerations[6:12]    

  return copy.deepcopy(points_l), copy.deepcopy(points_r)


def main():


  try:

    ### Set up the moveit_commander
    print "============ Setting up the moveit_commander (press ctrl-d to exit) ..."
    tutorial = MoveGroupPythonIntefaceTutorial()


    ### Go to the given joint state
    ''
    print "============ Go to joint state ..."
    tutorial.go_to_joint_state()
    ''


    ### Add mesh
    # flash body
    fb_ee_link = 'right_ee_link'
    fb_pose = geometry_msgs.msg.PoseStamped()
    fb_pose.header.frame_id = fb_ee_link
    fb_pose.pose.orientation.x = 0.0
    fb_pose.pose.orientation.y = 0.0
    fb_pose.pose.orientation.z = 0.0
    fb_pose.pose.orientation.w = 1.0
    fb_pose.pose.position.x = 0.07
    fb_pose.pose.position.y = 0.0
    fb_pose.pose.position.z = 0.0
    fb_file_path = "/home/liangyuwei/dual_ur5_ws/src/dual_ur5_control/meshes/flash_body_final.STL"
    fb_mesh_name = "flash_body"
    fb_size = [0.001, 0.001, 0.001]
    tutorial.scene.add_mesh(fb_mesh_name, fb_pose, fb_file_path, fb_size)
    # flash hat
    fh_ee_link = 'left_ee_link'
    fh_pose = geometry_msgs.msg.PoseStamped()
    fh_pose.header.frame_id = fh_ee_link
    fh_pose.pose.orientation.x = 0.0
    fh_pose.pose.orientation.y = 0.0
    fh_pose.pose.orientation.z = 0.0
    fh_pose.pose.orientation.w = 1.0
    fh_pose.pose.position.x = 0.07
    fh_pose.pose.position.y = 0.0
    fh_pose.pose.position.z = 0.0
    fh_file_path = "/home/liangyuwei/dual_ur5_ws/src/dual_ur5_control/meshes/flash_hat_final.STL"
    fh_mesh_name = "flash_hat"
    fh_size = [0.001, 0.001, 0.001]
    tutorial.scene.add_mesh(fh_mesh_name, fh_pose, fh_file_path, fh_size)


    ### Attach mesh   
    # flash body
    fb_grasping_group = 'right_gripper'
    touch_links = tutorial.robot.get_link_names(group=fb_grasping_group)
    tutorial.scene.attach_mesh(fb_ee_link, fb_mesh_name, fb_pose, touch_links=touch_links)
    fh_grasping_group = 'left_gripper'
    touch_links = tutorial.robot.get_link_names(group=fb_grasping_group)
    tutorial.scene.attach_mesh(fh_ee_link, fh_mesh_name, fh_pose, touch_links=touch_links)


    ### Load a Cartesian path from h5 file
    print "========== Load a Cartesian path from h5 file "
    f = h5py.File("tmp_new_paths_from_primitives.h5", "r")
    n_cols = 2
    n_actions = 2
    n_rows = n_actions
    # temporary: what if actions are not in pair!!! should split the left and right arms' actions in the first place!!!!
    cartesian_paths_lib = [[0 for i in range(n_cols)] for j in range(n_rows)]
    coord_sign = np.array([[0, 0], [1, 1]]) # set coordinated motion to 1; use ndarray, easy for finding the coordinated actions
    cartesian_paths_lib[0][0] = f['l_approach_2_tmp'][:]
    cartesian_paths_lib[1][0] = f['l_insert_2_tmp'][:]
    cartesian_paths_lib[0][1] = f['r_approach_2_tmp'][:]
    cartesian_paths_lib[1][1] = f['r_insert_2_tmp'][:]
    print "========== Convert Carteisna path to joint path through IK... "
    joint_path_plans_lib = [[0 for i in range(n_cols)] for j in range(n_rows)]
    for r in range(n_rows):
      for c in range(n_cols):
        print "== Processing actions " + str(r+1) + "/" + str(n_rows) + " of arms " + str(c+1) + "/" + str(n_cols) + "..."

        # set group and pose target, go to the start pose before planning!!!
        import pdb
        pdb.set_trace()
        if c == 0: # left arm
          l_or_r = "left"
        else:
          l_or_r = "right"
        group = moveit_commander.MoveGroupCommander(l_or_r + "_arm")
        print "-- Go to start pose before planning..."
        pose_target = group.get_current_pose(l_or_r + "_ee_link") # get current eef's pose
        pose_target.pose.position.x = cartesian_paths_lib[r][c][0, 0]
        pose_target.pose.position.y = cartesian_paths_lib[r][c][0, 1]
        pose_target.pose.position.z = cartesian_paths_lib[r][c][0, 2]
        quat = tf.transformations.quaternion_from_euler(cartesian_paths_lib[r][c][0, 3], cartesian_paths_lib[r][c][0, 4], cartesian_paths_lib[r][c][0, 5]) # same order
        pose_target.pose.orientation.x = quat[0]
        pose_target.pose.orientation.y = quat[1]
        pose_target.pose.orientation.z = quat[2]
        pose_target.pose.orientation.w = quat[3] # set the start pose
        group.set_pose_target(pose_target, l_or_r + '_ee_link') # set pose target
        group.allow_replanning(True)
        group.set_planning_time(1.0)
        group.go(wait=True) # plan and go
        group.stop() # stop
        group.clear_pose_targets() # clear targets


        # set Pose trajectories(should go to first pose before planning!!!)
        waypoints = []
        wpose = geometry_msgs.msg.Pose()
        for l in range(1, cartesian_paths_lib[r][c].shape[0]):
          wpose.position.x = cartesian_paths_lib[r][c][l, 0]
          wpose.position.y = cartesian_paths_lib[r][c][l, 1]
          wpose.position.z = cartesian_paths_lib[r][c][l, 2]
          quat = tf.transformations.quaternion_from_euler(cartesian_paths_lib[r][c][l, 3], cartesian_paths_lib[r][c][l, 4], cartesian_paths_lib[r][c][l, 5]) # same order ???
          wpose.orientation.x = quat[0]
          wpose.orientation.y = quat[1]
          wpose.orientation.z = quat[2]
          wpose.orientation.w = quat[3]
          waypoints.append(copy.deepcopy(wpose))
        # compute plan to convert cartesian path to joint plan
        print "-- Start planning now..."
        import pdb
        pdb.set_trace()
        (plan, fraction) = group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01, #0.01,        # eef_step # set to 0.001 for collecting the data
                                        0.0,     # jump_threshold
                                        avoid_collisions=False)    
        # temporary: using compute_cartesian_path is actually not good for dual-arm coordinated actions since the number of the resultant points is not in consistent with the number of cartesian points!!!

        # display the result
        group.execute(plan, wait=True)
        # store the generated joint plans
        joint_path_plans_lib[r][c] = copy.deepcopy(plan) # stored as moveit_msgs/RobotTrajectory

    
    ### Add Time Parameterization(joint path -> joint trajectory)
    import pdb
    pdb.set_trace()
    print "========== Apply TOTG to generate time optimal joint trajectory... "
    joint_traj_plans_lib = [[0 for i in range(n_cols)] for j in range(n_rows)] # stored as moveit_msgs/RobotTrajectory 
    vel_limits = [3.15, 3.15, 3.15, 3.15, 3.15, 3.15]
    acc_limits = [10.0, 10.0, 10.0, 10.0, 10.0, 10.0]

    ## ----- temporary: should split the left and right arms' actions in the first place, not presumed as in pair
    for c in range(n_cols): # uncoordinated actions, add TP separately
      tmp_plan = copy.deepcopy(joint_path_plans_lib[0][c])
      new_plan = moveit_msgs.msg.RobotTrajectory() 
      new_plan.joint_trajectory.points = add_time_optimal_parameterization_client(tmp_plan.joint_trajectory.points, vel_limits, acc_limits, 0.001)
      joint_traj_plans_lib[0][c] = copy.deepcopy(new_plan)

    tmp_plan_l = copy.deepcopy(joint_path_plans_lib[1][0])
    tmp_plan_r = copy.deepcopy(joint_path_plans_lib[1][1])
    tmp_plan_dual = moveit_msgs.msg.RobotTrajectory() #copy.deepcopy(tmp_plan_l) # initialization
    tmp_plan_dual.joint_trajectory.points = merge_two_plans(tmp_plan_l.joint_trajectory.points, tmp_plan_r.joint_trajectory.points)


    new_plan_dual = moveit_msgs.msg.RobotTrajectory() #copy.deepcopy(tmp_plan_dual)  # initialization

    pdb.set_trace()
    new_plan_dual.joint_trajectory.points = add_time_optimal_parameterization_client(tmp_plan_dual.joint_trajectory.points, vel_limits+vel_limits, acc_limits+acc_limits, 0.001) # get the result with TP

    pdb.set_trace()
    new_plan_l = moveit_msgs.msg.RobotTrajectory() #copy.deepcopy(new_plan_dual)
    new_plan_r = moveit_msgs.msg.RobotTrajectory() #copy.deepcopy(new_plan_dual)
    new_plan_l.joint_trajectory.points, new_plan_r.joint_trajectory.points = split_dual_plan(new_plan_dual.joint_trajectory.points) # split the dual plan

    pdb.set_trace()
    joint_traj_plans_lib[1][0] = copy.deepcopy(new_plan_l)
    joint_traj_plans_lib[1][1] = copy.deepcopy(new_plan_r)
    ## ----- end of temporary
              

    # convert plan structure to array structure
    import pdb
    pdb.set_trace()
    print "== Convert plan structure to array type..."
    joint_traj_lib = [[0 for i in range(n_cols)] for j in range(n_rows)]
    joint_time_stamp = [[0 for i in range(n_cols)] for j in range(n_rows)]
    for r in range(n_rows):
      for c in range(n_cols):
        print "== -- for actions " + str(r+1) + "/" + str(n_rows) + " of arms " + str(c+1) + "/" + str(n_cols) + "..."
        # initialization
        num_points = len(joint_traj_plans_lib[r][c].joint_trajectory.points)
        tmp_pos = np.zeros((num_points, 6), dtype=np.float64)
        tmp_vel = np.zeros((num_points, 6), dtype=np.float64)
        tmp_acc = np.zeros((num_points, 6), dtype=np.float64) 
        tmp_timestamp = np.zeros((num_points, 1), dtype=np.float64)
        for n in range(num_points):
          tmp_timestamp[n] = joint_traj_plans_lib[r][c].joint_trajectory.points[n].time_from_start.to_sec()
          tmp_pos[n, :] = np.array(joint_traj_plans_lib[r][c].joint_trajectory.points[n].positions)
          tmp_vel[n, :] = np.array(joint_traj_plans_lib[r][c].joint_trajectory.points[n].velocities)
          tmp_acc[n, :] = np.array(joint_traj_plans_lib[r][c].joint_trajectory.points[n].accelerations)
        # store the result
        joint_traj_lib[r][c] = np.concatenate((tmp_pos, tmp_vel, tmp_acc, tmp_timestamp), axis=1) # stored as ndarray, deep copy!!!

   
    ### Concatenate plans with the help of coord_sign
    print "============= Concatenate plans with the help of coordination signs"
    t_spent_already_l = 0
    t_spent_already_r = 0 # should check if they are equal

    id_start_l = 1
    id_start_r = 1
    id_coord_l, = np.where(coord_sign[1:, 0]==1) # the indices of the coordinated actions
    id_coord_l = id_coord_l + 1 # since the search starts from 1 instead of 0
    id_coord_r, = np.where(coord_sign[1:, 1]==1) # ignore the first
    id_coord_r = id_coord_r + 1
    num_coord_actions = len(id_coord_l) # the same as id_coord_r

    # initialization
    whole_traj_l = joint_traj_lib[0][0][:, :-1].copy() # copy just pos, vel and acc
    whole_timestamp_l = joint_traj_lib[0][0][:, -1].copy() # copy the last column
    t_spent_already_l = whole_timestamp_l[-1] # set new start

    whole_traj_r = joint_traj_lib[0][1][:, :-1].copy() 
    whole_timestamp_r = joint_traj_lib[0][1][:, -1].copy() # copy the last column
    t_spent_already_r = whole_timestamp_r[-1] # set new start

    # start concatenation, note that after the last coordinated actions, there could be more actions behind
    for n in range(num_coord_actions):
      print "== Processing coordinated action " + str(n+1) + "/" + str(num_coord_actions) 

      ## left arm, concatenate uncoordinated actions in-between
      print "== -- Left arm: uncoordinated actions..."
      import pdb
      pdb.set_trace()
      for m in range(id_start_l, id_coord_l[n]): # if two coordinated actions are adjacent, range() would return empty and the for-loop would not be run
        # until coordinated actions(not concatenated yet)
        whole_traj_l = np.concatenate((whole_traj_l, joint_traj_lib[m][0][:, :-1]))
        whole_timestamp_l = np.concatenate((whole_timestamp_l, joint_traj_lib[m][0][:, -1] + t_spent_already_l))
        t_spent_already_l = whole_timestamp_l[-1] # set new start
        
      ## right arm, concatenate uncoordinated actions in-between
      print "== -- Right arm: uncoordinated actions..."
      import pdb
      pdb.set_trace()
      for m in range(id_start_r, id_coord_r[n]):
        # until coordinated actions(not concatenated yet)
        whole_traj_r = np.concatenate((whole_traj_r, joint_traj_lib[m][1][:, :-1]))
        whole_timestamp_r = np.concatenate((whole_timestamp_r, joint_traj_lib[m][1][:, -1] + t_spent_already_r))
        t_spent_already_r = whole_timestamp_r[-1] # set new start

      ## append extra lines to make the lengths equal
      print "== -- Align the lengths..."
      import pdb
      pdb.set_trace()
      len_l = len(whole_timestamp_l)
      len_r = len(whole_timestamp_r)
      if len_l > len_r:
        whole_timestamp_r = np.concatenate((whole_timestamp_r, whole_timestamp_l[len_r:len_l]))
        whole_traj_r = np.concatenate( (whole_traj_r, np.matlib.repmat(whole_traj_l[-1, :], len_l-len_r, 1)) )
        t_spent_already_r = whole_timestamp_r[-1] # set new start
      elif len_l < len_r:
        whole_timestamp_l = np.concatenate((whole_timestamp_l, whole_timestamp_r[len_l:len_r]))
        whole_traj_l = np.concatenate( (whole_traj_l, np.matlib.repmat(whole_traj_r[-1, :], len_r-len_l, 1)) )
        t_spent_already_l = whole_timestamp_l[-1] # set new start
      
      ## concatenate coordinated actions(Note that coordinated actions should have the same length!!!)
      print "== -- Both arms: coordinated actions..."
      import pdb
      pdb.set_trace()
      whole_traj_l = np.concatenate((whole_traj_l, joint_traj_lib[id_coord_l[n]][0][:, :-1]))
      whole_timestamp_l = np.concatenate((whole_timestamp_l, joint_traj_lib[id_coord_l[n]][0][:, -1] + t_spent_already_l)) # copy and concatenate the last column
      t_spent_already_l = whole_timestamp_l[-1] # set new start
      id_start_l = id_coord_l[n] + 1  # set the next action after the current coordinated one

      whole_traj_r = np.concatenate((whole_traj_r, joint_traj_lib[id_coord_r[n]][1][:, :-1]))
      whole_timestamp_r = np.concatenate((whole_timestamp_r, joint_traj_lib[id_coord_r[n]][1][:, -1] + t_spent_already_r))
 # copy the last column
      t_spent_already_r = whole_timestamp_r[-1] # set new start
      id_start_r = id_coord_r[n] + 1  # set the next action after the current coordinated one  

    ### Process uncoordinated actions after the last coordinated actions (if any)
    if id_coord_l[-1] + 1 is not n_actions:
      for m in range(id_coord_l[-1] + 1, n_actions): 
        # until the end
        whole_traj_l = np.concatenate((whole_traj_l, joint_traj_lib[m][0][:, :-1]))
        whole_timestamp_l = np.concatenate((whole_timestamp_l, joint_traj_lib[m][0][:, -1] + t_spent_already_l))
        t_spent_already_l = whole_timestamp_l[-1] # set new start

    if id_coord_r[-1] + 1 is not n_actions:
      for m in range(id_coord_r[-1] + 1, n_actions): 
        # until the end
        whole_traj_r = np.concatenate((whole_traj_r, joint_traj_lib[m][1][:, :-1]))
        whole_timestamp_r = np.concatenate((whole_timestamp_r, joint_traj_lib[m][1][:, -1] + t_spent_already_r))
        t_spent_already_r = whole_timestamp_r[-1] # set new start
      

    ### Convert the result into a plan
    print "=========== Convert the resultant trajectories into a plan"
    import pdb
    pdb.set_trace()
    plan_whole_traj = moveit_msgs.msg.RobotTrajectory()
    plan_whole_traj.joint_trajectory.header.frame_id = '/world'
    plan_whole_traj.joint_trajectory.joint_names = ['left_shoulder_pan_joint', 'left_shoulder_lift_joint', 'left_elbow_joint', 'left_wrist_1_joint', 'left_wrist_2_joint', 'left_wrist_3_joint', 'right_shoulder_pan_joint', 'right_shoulder_lift_joint', 'right_elbow_joint', 'right_wrist_1_joint', 'right_wrist_2_joint', 'right_wrist_3_joint']
    traj_point = trajectory_msgs.msg.JointTrajectoryPoint()
    for n in range(whole_traj_l.shape[0]):
      traj_point.positions = np.concatenate((whole_traj_l[n, 0:6], whole_traj_r[n, 0:6])).tolist()
      traj_point.velocities = np.concatenate((whole_traj_l[n, 6:12], whole_traj_r[n, 6:12])).tolist()
      traj_point.accelerations = np.concatenate((whole_traj_l[n, 12:18], whole_traj_r[n, 12:18])).tolist()
      traj_point.time_from_start = rospy.Duration(whole_timestamp_l[n]) # left and right timestamp sequence are close to each other...
      plan_whole_traj.joint_trajectory.points.append(traj_point)
    

    ### Execute dual arm plan
    print "============ Execute the processed dual arm plan..."
    import pdb
    pdb.set_trace()
    group = moveit_commander.MoveGroupCommander("dual_arms")
    # go to start position first
    joint_goal = plan_whole_traj.joint_trajectory.points[0].positions
    group.go(joint_goal, wait=True)
    group.stop()
    # execute the plan now
    group.execute(plan_whole_traj, wait=True)


    ### Store the result for display in MATLAB
    f = h5py.File("dual_ur5_whole_traj_concat_2.h5", "a")
    joint_traj_name = "traj_pair_approach2_insert2_tmp"
    joint_traj_group =  f.create_group(joint_traj_name)
    joint_traj_group.create_dataset("joint_traj_l", data=whole_traj_l, dtype=float)
    joint_traj_group.create_dataset("joint_traj_r", data=whole_traj_r, dtype=float)
    joint_traj_group.create_dataset("joint_timestamp_l", data=whole_timestamp_l, dtype=float)    
    joint_traj_group.create_dataset("joint_timestamp_r", data=whole_timestamp_r, dtype=float)
    f.close()


    ### Detach mesh
    ''
    import pdb
    pdb.set_trace()
    tutorial.scene.remove_attached_object(fb_ee_link, fb_mesh_name)
    tutorial.scene.remove_attached_object(fh_ee_link, fh_mesh_name)
    ''

    ### Remove mesh
    ''
    tutorial.remove_object(fb_mesh_name, timeout=4)
    tutorial.remove_object(fh_mesh_name, timeout=4)
    ''


  except rospy.ROSInterruptException:
    return

  except KeyboardInterrupt:
    return


if __name__ == '__main__':

  main()







