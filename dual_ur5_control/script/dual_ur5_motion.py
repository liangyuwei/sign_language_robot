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
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
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

  #def execute_plan(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    
    #group = self.group

    ## BEGIN_SUB_TUTORIAL execute_plan
    ##
    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    #group.execute(plan, wait=True)

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
      #group.allow_replanning(True)
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
      #group.allow_replanning(True)

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
    group.execute(plan, wait=True)


    return plan 

  def gripper_control(self, open_or_close=True, left_or_right=True):

    # decide which gripper to control
    if (left_or_right): # left
      group_name = "left_gripper"
    else: # right
      group_name = "right_gripper"

    # initialize the group
    group = moveit_commander.MoveGroupCommander(group_name)

    # get current joint values for later use
    joint_goal = group.get_current_joint_values()

    # set angle
    if (open_or_close): # open
      ang = 0.2
    else: # close
      ang = 0.8  # max is 1.1
    joint_goal[0] = ang
    joint_goal[1] = ang * 1.0 / 1.1
    joint_goal[2] = ang
    joint_goal[3] = ang * 1.0 / 1.1

    # execute
    group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()

  def grasp(self):
  
    # moveit pick
    grasp = moveit_msgs.msg.Grasp()
    
    ## fill in information
    grasp.id = "grasp_flash"

    # pre grasp posture(open gripper)
    grasp.pre_grasp_posture.header.frame_id = "world"
    grasp.pre_grasp_posture.joint_names = ["right_rh_p12_rn", "right_rh_r2", "right_rh_l1", "right_rh_l2"]
    grasp.pre_grasp_posture.points = [trajectory_msgs.msg.JointTrajectoryPoint()]
    ang = 0.0
    grasp.pre_grasp_posture.points[0].positions = [ang, ang/1.1, ang, ang/1.1]
    grasp.pre_grasp_posture.points[0].time_from_start = rospy.Duration(0.5)
    # grasp posture(close gripper)
    grasp.grasp_posture.header.frame_id = "world"
    grasp.grasp_posture.joint_names = ["right_rh_p12_rn", "right_rh_r2", "right_rh_l1", "right_rh_l2"]
    grasp.grasp_posture.points = [trajectory_msgs.msg.JointTrajectoryPoint()]
    ang = 0.8
    grasp.grasp_posture.points[0].positions = [ang, ang/1.1, ang, ang/1.1]
    grasp.grasp_posture.points[0].time_from_start = rospy.Duration(0.5)

    # grasp pose
    grasp.grasp_pose.header.frame_id = "world"
    grasp.grasp_pose.pose.position.x = 0.5 # 0.5
    grasp.grasp_pose.pose.position.y = -0.4 # -0.4
    grasp.grasp_pose.pose.position.z = 0.19 # 0.19
    tmp = tf.transformations.quaternion_from_euler(math.pi, math.pi/2, 0)
    grasp.grasp_pose.pose.orientation.x = tmp[0]
    grasp.grasp_pose.pose.orientation.y = tmp[1]
    grasp.grasp_pose.pose.orientation.z = tmp[2]
    grasp.grasp_pose.pose.orientation.w = tmp[3]
    # pre grasp approach
    grasp.pre_grasp_approach.direction.header.frame_id = "world"
    grasp.pre_grasp_approach.direction.vector.z = 1.0
    grasp.pre_grasp_approach.desired_distance = 0.1
    grasp.pre_grasp_approach.min_distance = 0.095 # error???
    # post grasp retreat
    grasp.post_grasp_retreat.direction.header.frame_id = "world"
    grasp.post_grasp_retreat.direction.vector.z = -1.0
    grasp.post_grasp_retreat.desired_distance = 0.1
    grasp.post_grasp_retreat.min_distance = 0.095

    # move group
    group_name = "right_arm" 
    group = moveit_commander.MoveGroupCommander(group_name)
    object_name = "flash_body"
    group.pick(object_name, grasp, plan_only=False)
    

  def place(self):

    # moveit place
    place_location = moveit_msgs.msg.PlaceLocation()

  def offset_from_origin_along_x_axis(self, origin, rotm, offset):

    # compute offset vector along x axis, in local frame
    v_offset = [offset, 0, 0]
    
    # transform offset vector to world frame
    v_offset_world = np.dot(rotm, v_offset)
  
    # compute new point after shifting along local x axis
    new_point = origin + v_offset_world
  
    return new_point    


def main():


  try:

    ### Set up the moveit_commander
    print "============ Setting up the moveit_commander (press ctrl-d to exit) ..."
    tutorial = MoveGroupPythonIntefaceTutorial()


    ### Go to the given joint state
    '''
    print "============ Go to joint state ..."
    tutorial.go_to_joint_state() # not supported by trac_ik...
    '''


    ### Add mesh
    ''
    print "============ Adding flash models into the scene..."
    import pdb
    pdb.set_trace()
    # flash body
    fb_ee_link = 'world' #'right_ee_link'
    fb_pose = geometry_msgs.msg.PoseStamped()
    fb_pose.header.frame_id = fb_ee_link
    fb_pose_tmp = tf.transformations.quaternion_from_euler(0, math.pi/2, 0)
    fb_pose.pose.orientation.x = fb_pose_tmp[0] #0.0
    fb_pose.pose.orientation.y = fb_pose_tmp[1] #0.0
    fb_pose.pose.orientation.z = fb_pose_tmp[2] #0.0
    fb_pose.pose.orientation.w = fb_pose_tmp[3] #1.0
    fb_pose.pose.position.x = 0.5 #0.07
    fb_pose.pose.position.y = -0.4 #0.0
    fb_pose.pose.position.z = 0.1 #0.0
    fb_file_path = "/home/liangyuwei/dual_ur5_ws/src/dual_ur5_control/meshes/flash_body_final.STL"
    fb_mesh_name = "flash_body"
    fb_size = [0.001, 0.001, 0.001]
    tutorial.scene.add_mesh(fb_mesh_name, fb_pose, fb_file_path, fb_size)
    # flash hat
    fh_ee_link = 'world' #'left_ee_link'
    fh_pose = geometry_msgs.msg.PoseStamped()
    fh_pose.header.frame_id = fh_ee_link
    fh_pose_tmp = tf.transformations.quaternion_from_euler(0, math.pi/2, 0)
    fh_pose.pose.orientation.x = fh_pose_tmp[0] #0.0
    fh_pose.pose.orientation.y = fh_pose_tmp[1] #0.0
    fh_pose.pose.orientation.z = fh_pose_tmp[2] #0.0
    fh_pose.pose.orientation.w = fh_pose_tmp[3] #1.0
    fh_pose.pose.position.x = 0.5 #0.07
    fh_pose.pose.position.y = 0.4 #0.0
    fh_pose.pose.position.z = 0.04 #0.0
    fh_file_path = "/home/liangyuwei/dual_ur5_ws/src/dual_ur5_control/meshes/flash_hat_final.STL"
    fh_mesh_name = "flash_hat"
    fh_size = [0.001, 0.001, 0.001]
    tutorial.scene.add_mesh(fh_mesh_name, fh_pose, fh_file_path, fh_size)
    ''


    ### Go to grasp position
    print "============ Go to grasp position..."
#    import pdb
#    pdb.set_trace()
    # get pose target for dual_arms
    left_pose_target = tutorial.group.get_current_pose('left_ee_link')
    right_pose_target = tutorial.group.get_current_pose('right_ee_link')
    # set new targets
    left_pre_grasp_pos = [0.5, 0.4, 0.15] # 0.04 + 0.07
    left_pose_target.pose.position.x = left_pre_grasp_pos[0]
    left_pose_target.pose.position.y = left_pre_grasp_pos[1]
    left_pose_target.pose.position.z = left_pre_grasp_pos[2]
    left_pre_grasp_pose = tf.transformations.quaternion_from_euler(math.pi, math.pi/2, 0) #[0, 0, -0.707, 0.707]
    left_pose_target.pose.orientation.x = left_pre_grasp_pose[0]
    left_pose_target.pose.orientation.y = left_pre_grasp_pose[1]
    left_pose_target.pose.orientation.z = left_pre_grasp_pose[2]
    left_pose_target.pose.orientation.w = left_pre_grasp_pose[3]

    right_pre_grasp_pos = [0.5, -0.4, 0.19] # 0.1 + 0.07
    right_pose_target.pose.position.x = right_pre_grasp_pos[0]
    right_pose_target.pose.position.y = right_pre_grasp_pos[1]
    right_pose_target.pose.position.z = right_pre_grasp_pos[2]
    right_pre_grasp_pose = tf.transformations.quaternion_from_euler(0, math.pi/2, math.pi) #[0, 0, 0.707, 0.707]
    right_pose_target.pose.orientation.x = right_pre_grasp_pose[0]#0.707#0.0
    right_pose_target.pose.orientation.y = right_pre_grasp_pose[1]#0.707#0.0
    right_pose_target.pose.orientation.z = right_pre_grasp_pose[2]#0.0#0.707
    right_pose_target.pose.orientation.w = right_pre_grasp_pose[3]#0.0#0.707#1.0

    tutorial.group.set_pose_target(left_pose_target, 'left_ee_link')
    tutorial.group.set_pose_target(right_pose_target, 'right_ee_link')
    # plan
    tutorial.group.allow_replanning(True) # added by LYW, 2019/07/09, for the dual-arm to go to grasp position
    plan = tutorial.group.go(wait=True)
    # stop
    tutorial.group.stop()
    # clear targets
    tutorial.group.clear_pose_targets()


    ### Attach mesh(so that grasp action won't cause collision checking)
    ''
    print "============ Attach flash models to eef links..."
    import pdb
    pdb.set_trace()
    # flash body
    fb_pose.pose.orientation.x = 0.0
    fb_pose.pose.orientation.y = 0.0
    fb_pose.pose.orientation.z = 0.0
    fb_pose.pose.orientation.w = 1.0
    fb_pose.pose.position.x = 0.09
    fb_pose.pose.position.y = 0.0
    fb_pose.pose.position.z = 0.0
    fb_grasping_group = 'right_gripper'
    touch_links = tutorial.robot.get_link_names(group=fb_grasping_group)
    tutorial.scene.attach_mesh("right_ee_link", fb_mesh_name, fb_pose, touch_links=touch_links)
    # flash hat
    fh_pose.pose.orientation.x = 0.0
    fh_pose.pose.orientation.y = 0.0
    fh_pose.pose.orientation.z = 0.0
    fh_pose.pose.orientation.w = 1.0
    fh_pose.pose.position.x = 0.11
    fh_pose.pose.position.y = 0.0
    fh_pose.pose.position.z = 0.0
    fh_grasping_group = 'left_gripper'
    touch_links = tutorial.robot.get_link_names(group=fh_grasping_group)
    tutorial.scene.attach_mesh("left_ee_link", fh_mesh_name, fh_pose, touch_links=touch_links)
    ''


    ### Execute grasping
    '''
    print "============ Execute grasp action for Gazebo..."
    import pdb
    pdb.set_trace()
    #open_or_close = True
    #left_or_right = True
    # left gripper grasps flash hat 
    tutorial.gripper_control(False, True)
    # right gripper grasps flash body
    tutorial.gripper_control(False, False)
    '''

    
    ### Planning of two ur5 arms: plan a cartesian path
    print "============ Plan and display an assembly action(cartesian path) ..."
    import pdb
    pdb.set_trace()

    x_contact_origin = [0.5, 0.0, 0.35]
    l_pre_grasp_offset = -0.1
    r_pre_grasp_offset = -0.1
    
    # left arm
#    l_x_start = [0.6, 0.38, 0.4]
#    l_x_mid = [0.5, 0.3, 0.35] #[0.6, 0.26, 0.3] # from mid to final, only y changes and differs
#    l_x_final = [0.5, 0.25, 0.35] #[0.6, 0.16, 0.3]

    l_w_start = tf.transformations.quaternion_from_euler(0, 0, -0.75*math.pi) # [0, 0, -0.707, 0.707] #
    l_w_mid = tf.transformations.quaternion_from_euler(0, 0, -0.45*math.pi) #tf.transformations.quaternion_from_euler(0, 0.25*math.pi, -0.25*math.pi) #tf.transformations.quaternion_from_euler(0, 0, -0.25*math.pi) #[0, 0, -0.707, 0.707]
    l_w_final = l_w_mid #tf.transformations.quaternion_from_euler(0, 0, -0.5*math.pi) #tf.transformations.quaternion_from_euler(0, 0.25*math.pi, -0.25*math.pi) #tf.transformations.quaternion_from_euler(0, 0, -0.25*math.pi) #[0, 0, -0.707, 0.707]

    l_x_start = [0.55, 0.33, 0.35]
    left_rotm = tf.transformations.quaternion_matrix(l_w_final)
    left_rotm = left_rotm[:3, :3]
    l_x_final = tutorial.offset_from_origin_along_x_axis(x_contact_origin, left_rotm, -left_pre_grasp_pos[2] + 0.03) #[0.5, 0.1, 0.35]#
    l_x_mid = tutorial.offset_from_origin_along_x_axis(l_x_final, left_rotm, l_pre_grasp_offset)


    planning_time = 1 # time used for motion planning!!! Not total time of the trajectory!!

    left_or_right_eef = True

    plan_l = tutorial.plan_motion(l_x_start, l_w_start, l_x_mid, l_w_mid, l_x_final, l_w_final, planning_time, left_or_right_eef)

    # right arm
#    r_x_start = [0.55, -0.35, 0.32]
#    r_x_mid = [0.4, -0.2, 0.3] #[0.5, -0.26, 0.3]
#    r_x_final = [0.4, -0.1, 0.3] #[0.5, -0.16, 0.3]

    r_w_start = tf.transformations.quaternion_from_euler(0, 0, 0.45*math.pi)  #[0, 0, 0.707, 0.707]
    relative_quat = tf.transformations.quaternion_from_euler(0, 0, math.pi) #(math.pi, 0, math.pi) # - two ways of inserting flash body into flash hat
    r_w_mid = tf.transformations.quaternion_multiply(l_w_mid, relative_quat) #tf.transformations.quaternion_from_euler(0, 0.25*math.pi, 0.75*math.pi) #tf.transformations.quaternion_from_euler(0, 0, 0.75*math.pi) #[0, 0, 0.707, 0.707]
    r_w_final = r_w_mid #tf.transformations.quaternion_multiply(l_w_final, relative_quat) #tf.transformations.quaternion_from_euler(0, 0.25*math.pi, 0.75*math.pi) #tf.transformations.quaternion_from_euler(0, 0, 0.75*math.pi) #[0, 0, 0.707, 0.707]

    r_x_start = [0.45, -0.34, 0.35]
    right_rotm = tf.transformations.quaternion_matrix(r_w_final)
    right_rotm = right_rotm[:3, :3]
    r_x_final = tutorial.offset_from_origin_along_x_axis(x_contact_origin, right_rotm, -right_pre_grasp_pos[2]) #[0.5, -0.1, 0.35] #
    r_x_mid = tutorial.offset_from_origin_along_x_axis(r_x_final, right_rotm, r_pre_grasp_offset)


    planning_time = 1 # time used for motion planning!!! Not total time of the trajectory!!

    left_or_right_eef = False

    plan_r = tutorial.plan_motion(r_x_start, r_w_start, r_x_mid, r_w_mid, r_x_final, r_w_final, planning_time, left_or_right_eef)
    

    ### Plot the generated joint trajectories to check whether they're within bounds
    ''
    import pdb
    pdb.set_trace()
    import matplotlib.pyplot as plt
    import numpy as np
    fig, axes = plt.subplots(nrows=2, ncols=6) # create a figure object
    for ir in range(2):
      # get plan
      if ir == 0: # left arm
        tmp_plan = plan_l
      else: # right arm
        tmp_plan = plan_r

      # obtain trajectory points
      tmp_points = np.ndarray((len(tmp_plan.joint_trajectory.points), 6), dtype=float)
      tmp_time = np.ndarray(len(tmp_plan.joint_trajectory.points))
      for n in range(len(tmp_plan.joint_trajectory.points)):
        tmp_points[n, :] = tmp_plan.joint_trajectory.points[n].positions
        tmp_time[n] = tmp_plan.joint_trajectory.points[n].time_from_start.to_sec()

      # plot the acquired data
      for ic in range(6):
        # set title
        axes[ir, ic].set(title=tmp_plan.joint_trajectory.joint_names[ic])
        # plot data
        #axes[ir, ic].plot(np.linspace(0, 10, len(tmp_plan.joint_trajectory.points)), tmp_points[:, ic])
        axes[ir, ic].plot(tmp_time, tmp_points[:, ic])

    # display
    plt.show()

    ''


    ### Use h5py to store the generated motion plan
    ''
    print "============ Store the results using h5py ..."
    import pdb
    pdb.set_trace()
    import h5py
    import numpy as np
    # process the data using numpy 
    cartesian_plan = plan_l
    index = "9" 
    # 1 for TP, 2 for spline, 3 for TOTG, 4 for TOTG(Add TP first, ORDER matters), 5 for no TP
    # 6 for TOTG with nonzero(limited) acc(0.0 means unlimited) -- acc 2.0
    # 7 for TP -- acc 2.0
    # 8 for TOTG -- acc 10.0
    # 9 for TP -- acc 10.0
    
    imi_path_name = "traj_pair_l_" + index # traj_pair_r_1
    for j in range(2):
      len_sample = len(cartesian_plan.joint_trajectory.points)
      pos = np.zeros((len_sample, 6), dtype=float)
      vel = np.zeros((len_sample, 6), dtype=float)
      acc = np.zeros((len_sample, 6), dtype=float)
      time_from_start = np.zeros((len_sample,), dtype=float)
      for i in range(len_sample):
        pos[i] = np.array(cartesian_plan.joint_trajectory.points[i].positions)
        vel[i] = np.array(cartesian_plan.joint_trajectory.points[i].velocities)
        acc[i] = np.array(cartesian_plan.joint_trajectory.points[i].accelerations) 
        time_from_start[i] = cartesian_plan.joint_trajectory.points[i].time_from_start.to_sec()

      # store the results using h5py
      f = h5py.File("dual_ur5_joint_trajectory_DIFF_TIME_PARAMETERIZATION.h5", "a") 

      path_group =  f.create_group(imi_path_name)
      path_group.create_dataset("pos", data=pos, dtype=float)
      path_group.create_dataset("vel", data=vel, dtype=float)
      path_group.create_dataset("acc", data=acc, dtype=float)    
      path_group.create_dataset("time_from_start", data=time_from_start, dtype=float)
      f.close()
    

      # set to plan_r for the next iteration
      cartesian_plan = plan_r  
      imi_path_name = "traj_pair_r_" + index

    ''

    ### Open grippers
    '''
    print "============ Open grippers for Gazebo..."
    import pdb
    pdb.set_trace()
    tutorial.gripper_control(True, True)
    tutorial.gripper_control(True, False)
    '''

    ### Detach mesh
    ''
    print "============ Detach flash models..."
    import pdb
    pdb.set_trace()
    tutorial.scene.remove_attached_object("right_ee_link", fb_mesh_name)
    tutorial.scene.remove_attached_object("left_ee_link", fh_mesh_name)
    ''

    ### Remove mesh
    ''
    print "============ Remove flash models from the scene ..."
    import pdb
    pdb.set_trace()
    tutorial.remove_object(fb_mesh_name, timeout=4)
    tutorial.remove_object(fh_mesh_name, timeout=4)
    ''


  except rospy.ROSInterruptException:
    return

  except KeyboardInterrupt:
    return


if __name__ == '__main__':

  main()







