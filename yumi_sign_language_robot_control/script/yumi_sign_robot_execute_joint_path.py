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
    group_name = "left_arm"
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
    waypoints = [] # temporary: for generating trajectory segments
    wpose = group.get_current_pose().pose # temporary: for generating trajectory segments
    '''
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
    '''

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

def add_time_optimal_parameterization_client(points):
  # wait for service to come online
  rospy.wait_for_service('add_time_optimal_parameterization_server')

  try:
    path_to_traj = rospy.ServiceProxy('add_time_optimal_parameterization_server', PathToTraj)
    res = path_to_traj(points)
    return res.traj
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e


def store_h5(cartesian_plan, h5_file_name, path_group_name):
  
  print "============ Store realistic results for concatenation, using h5py ..."
 
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

  f = h5py.File( h5_file_name, "a") 
  path_group =  f.create_group(path_group_name)
  path_group.create_dataset("pos", data=pos, dtype=float)
  path_group.create_dataset("vel", data=vel, dtype=float)
  path_group.create_dataset("acc", data=acc, dtype=float)    
  path_group.create_dataset("time_from_start", data=time_from_start, dtype=float)
  f.close()

  return true


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







