#!/usr/bin/env python

import sys
import copy
import rospy
import math
import tf
import time
import numpy as np
import numpy.matlib
import random
import h5py
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from raw_totg.srv import *
from dual_ur5_control.srv import *

# Import simple_PSO code(copied from online, did a little modification)
import simple_PSO
# Import Gradient Descent code
import simple_GD


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
    print("============ Reference frame: %s" % planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    print("============ End effector: %s" % eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print("============ Robot Groups:", robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print("============ Printing robot state")
    print(robot.get_current_state())
    print("")
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
    print("== Set initial pose ==")
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
    print("== Set via point ==")
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
    print("== Set final pose ==")
    wpose.position.x = x_final[0]
    wpose.position.y = x_final[1]
    wpose.position.z = x_final[2]
    wpose.orientation.x = w_final[0]
    wpose.orientation.y = w_final[1]
    wpose.orientation.z = w_final[2]
    wpose.orientation.w = w_final[3]
    waypoints.append(copy.deepcopy(wpose))

    ### Compute a cartesian path
    print("== Compute a cartesian path ==")
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01, #0.01,        # eef_step # set to 0.001 for collecting the data
                                       0.0,     # jump_threshold
                                       avoid_collisions=False)         

    ### Execute the plan
    print("== Execute the plan ==")
    self.execute_plan(plan)


    return plan    


def add_time_optimal_parameterization_client(path, vel_limits, acc_limits, timestep=0.001):
  # wait for service to come online
  rospy.wait_for_service('add_time_optimal_parameterization_server')

  try:
    path_to_traj = rospy.ServiceProxy('add_time_optimal_parameterization_server', PathToTraj)
    res = path_to_traj(path, vel_limits, acc_limits, timestep)
    return res.traj
  except rospy.ServiceException as e:
    print("Service call failed: %s" % e)


def get_minimum_time_client(path, vel_limits, acc_limits):
  # wait for service to come online
  rospy.wait_for_service('get_minimum_time_server')

  try:
    get_min_time = rospy.ServiceProxy('get_minimum_time_server', GetMinTime)
    res = get_min_time(path, vel_limits, acc_limits)
    return res.min_time
  except rospy.ServiceException as e:
    print("Service call failed: %s" % e)


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


def apply_fk_client(joint_trajectory, left_or_right):

  if type(joint_trajectory) is list:
    if type(joint_trajectory[0]) is not trajectory_msgs.msg.JointTrajectoryPoint:
      print("Wrong data type for joint_trajectory!")
      return False
  else:
    if type(joint_trajectory) is not trajectory_msgs.msg.JointTrajectoryPoint:
      print("Wrong data type for joint_trajectory!")
      return False

  if type(left_or_right) is not bool:
    print("Wrong data type for left_or_right selection variable!")
    return False

  # wait for service to come online
  rospy.wait_for_service('apply_fk_server')

  try:
    joint_to_cart = rospy.ServiceProxy('apply_fk_server', JntToCart)
    res = joint_to_cart(left_or_right, joint_trajectory)
    return res.cart_trajectory
  except rospy.ServiceException as e:
    print("Service call failed: %s" % e)


class PSOCostFunc():

  def __init__(self, goal_rel_pos_pose):

    self.l_group = moveit_commander.MoveGroupCommander("left_arm")
    self.r_group = moveit_commander.MoveGroupCommander("right_arm")
    #self.largest_cost = -1
    self.rel_trans = goal_rel_pos_pose # relative transformation matrix
    #self.cost_history = [] # there should not be a cost history for PSO, since the cost func is not optimized by one set of variables, but a set of particles, each with different set of variables...


  def f(self, left_goal_pos):

    ### Input a target pose for left arm, generate a corresponding right arm's goal.
    # (left_start and right_start as arguments)
    # new goals --> [DMP] --> new cartesian paths --> [IK] --> new joint paths --> [TOTG] --> duration(and new joint trajs) --> Cost function for PSO

    ## -- temporary: should be able to pass arguments
    left_start = np.array([0.55, 0.35, 0.4, 0.0, 0.0, -0.25*math.pi])
    right_start = np.array([0.55, -0.35, 0.4, 0.0, 0.0, 0.25*math.pi]) #[0.45, -0.35, 0.3, 0.0, 0.0, -0.25*math.pi]
#    left_start = np.array([0.55, 0.6, 0.4, 0.0, 0.0, 0.25*math.pi])
#    right_start = np.array([0.55, -0.1, 0.4, 0.0, 0.0, -0.25*math.pi]) #[0.45, -0.35, 0.3, 0.0, 0.0, -0.25*math.pi]
#    left_start = np.array([0.35, 0.4, 0.6, 0.0, 0.0, 0.25*math.pi])
#    right_start = np.array([0.55, -0.4, 0.2, 0.0, 0.0, -0.25*math.pi]) 

    left_goal_pose = [0, 0, -0.5*math.pi]
    left_goal = left_goal_pos + left_goal_pose
    ## -- end of temporary  


    ## 1 - set right arm's goal
    print("========== Set right arm's goal")
    # obtain left_goal's homogeneous matrix representation
    left_goal_trans = tf.transformations.euler_matrix(left_goal[3], left_goal[4], left_goal[5])
    left_goal_trans[:3, 3] = np.array(left_goal_pos)

    # obtain right_goal's homogeneous matrix representation
    right_goal_trans = np.dot(left_goal_trans, self.rel_trans) # right-multiply ???
    # transform into xyz and euler angles
    right_goal = np.concatenate((right_goal_trans[:3, 3], tf.transformations.euler_from_matrix(right_goal_trans[:3][:3])))


    # check right arm's pose
    '''    
    print("==== Check the right arm's pose...")
    l_w_final = tf.transformations.quaternion_from_euler(left_goal[3], left_goal[4], left_goal[5])
    relative_quat = tf.transformations.quaternion_from_euler(0, 0, math.pi)
    r_w_final = tf.transformations.quaternion_multiply(l_w_final, relative_quat)
    right_rotm = tf.transformations.quaternion_matrix(r_w_final)
    right_rotm = right_rotm[:3, :3] # get the rotation part
    # compute offset vector along x axis
    offset = -0.32
    v_offset = [offset, 0, 0] 
    v_offset_world = np.dot(right_rotm, v_offset)
    r_x_final = left_goal[0:3] + v_offset_world
    # construct right arm's goal
    right_goal_tmp = np.concatenate((r_x_final, tf.transformations.euler_from_quaternion(r_w_final)))
    if not all_close(right_goal, right_goal_tmp, 0.001):
      print("******* Error right_goal *******")
    '''


    # check constraint on right goal ~~~
    bounds = [(0.3, 0.6), (-0.6, 0.6), (0.2, 0.6), (-math.pi, math.pi), (-math.pi, math.pi), (-math.pi, math.pi)]
    for l in range(len(bounds)):
      if right_goal[l] <= bounds[l][0] or right_goal[l] >= bounds[l][1]:
        # right_goal is out of bounds, abort this iteration
        return -1 # indicating infeasible solution
        #if self.largest_cost > 0:
        #  return self.largest_cost
        #else:
        #  return 10


    ## 2 - DMP, new cartesian paths
    # -- temporary, generate cartesian paths using DMP
    ndata = 50
    l_cartesian_path = np.zeros((6, ndata), dtype=float)
    r_cartesian_path = np.zeros((6, ndata), dtype=float)
    for i in range(6):
      l_cartesian_path[i, :] = np.linspace(left_start[i], left_goal[i], num=ndata, endpoint=True)
      r_cartesian_path[i, :] = np.linspace(right_start[i], right_goal[i], num=ndata, endpoint=True)
    # -- end of temporary



    ## 3 - IK
    #import pdb
    #pdb.set_trace()
    # LEFT ARM
    print("========== Perform IK for left arm's path...")
    # set Pose trajectories(should go to first pose before planning!!!)
    print("-- Plan for left arm...")
    waypoints = []
    wpose = geometry_msgs.msg.Pose()
    for n in range(1, l_cartesian_path.shape[1]):
      wpose.position.x = l_cartesian_path[0, n]
      wpose.position.y = l_cartesian_path[1, n]
      wpose.position.z = l_cartesian_path[2, n]
      quat = tf.transformations.quaternion_from_euler(l_cartesian_path[3, n], l_cartesian_path[4, n], l_cartesian_path[5, n]) # same order ???
      wpose.orientation.x = quat[0]
      wpose.orientation.y = quat[1]
      wpose.orientation.z = quat[2]
      wpose.orientation.w = quat[3]
      waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = self.l_group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step # set to 0.01 for 1 cm resolution
                                        0.0,         # jump_threshold
                                        avoid_collisions=False)    
    # display the result
    #l_group.execute(plan, wait=True) # do not execute!!! current state should always be set to the  start.
    # store the generated joint plans
    l_joint_path_plan = copy.deepcopy(plan) # stored as moveit_msgs/RobotTrajectory

    # RIGHT ARM
    # set Pose trajectories(should go to first pose before planning!!!)
    print("-- Plan for right arm...")
    waypoints = []
    wpose = geometry_msgs.msg.Pose()
    for n in range(1, r_cartesian_path.shape[1]):
      wpose.position.x = r_cartesian_path[0, n]
      wpose.position.y = r_cartesian_path[1, n]
      wpose.position.z = r_cartesian_path[2, n]
      quat = tf.transformations.quaternion_from_euler(r_cartesian_path[3, n], r_cartesian_path[4, n], r_cartesian_path[5, n]) # same order ???
      wpose.orientation.x = quat[0]
      wpose.orientation.y = quat[1]
      wpose.orientation.z = quat[2]
      wpose.orientation.w = quat[3]
      waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = self.r_group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step # set to 0.01 for 1 cm resolution
                                        0.0,         # jump_threshold
                                        avoid_collisions=False)    
    # display the result
    #r_group.execute(plan, wait=True)
    # store the generated joint plans
    r_joint_path_plan = copy.deepcopy(plan) # stored as moveit_msgs/RobotTrajectory

  
    # 4 - TOTG
    #import pdb
    #pdb.set_trace()
    print("========== Apply TOTG to get minimum time... ")
    # set up joint kinematic constraints
    vel_limits = [3.15, 3.15, 3.15, 3.15, 3.15, 3.15]
    acc_limits = [3.15, 3.15, 3.15, 3.15, 3.15, 3.15] #[10.0, 10.0, 10.0, 10.0, 10.0, 10.0]
    # get minimum time for left arm's motion using TOTG
    print("-- for left arm...")   
    tmp_plan = copy.deepcopy(l_joint_path_plan)
    r_min_time = get_minimum_time_client(tmp_plan.joint_trajectory.points, vel_limits, acc_limits)
    if r_min_time is None: # in case that trajectory generation fails
      return -1 # indicating infeasible solution
      #if self.largest_cost > 0:
      #  return self.largest_cost
      #else:
      #  return 10

    # get minimum time for right arm's motion using TOTG
    print("-- for right arm...")   
    tmp_plan = copy.deepcopy(r_joint_path_plan)
    l_min_time = get_minimum_time_client(tmp_plan.joint_trajectory.points, vel_limits, acc_limits)
    if l_min_time is None: # in case that trajectory generation fails
      return -1 # indicating infeasible solution      
      #if self.largest_cost > 0:
      #  return self.largest_cost
      #else:
      #  return 10    

    # 5 - the costs
    #import pdb
    #pdb.set_trace()
    pso_cost_val = max(r_min_time, l_min_time) # max function is much more reasonable!!

    #if pso_cost_val > self.largest_cost:
    #  self.largest_cost = pso_cost_val # recordd for constraint

    # record the descent process
    #self.cost_history.append(pso_cost_val)


    return pso_cost_val #l_min_time, r_min_time, left_goal[:3], right_goal[:3] # modification for heuristic gradient descent #pso_cost_val 



def compute_cartesian_path_client(group_name, waypoints):

  # wait for service to come online
  rospy.wait_for_service('compute_cartesian_path_server')

  try:
    cart_to_jnt = rospy.ServiceProxy('compute_cartesian_path_server', CartToJnt)
    res = cart_to_jnt(group_name, waypoints)
    return res.plan
  except rospy.ServiceException as e:
    print("Service call failed: %s" % e)
  


def main():


  try:

    ### Set up the moveit_commander
    print("============ Setting up the moveit_commander (press ctrl-d to exit) ...")
    tutorial = MoveGroupPythonIntefaceTutorial()


    ### Go to the given joint state
    ''
    print("============ Go to joint state ...")
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


    ### Set up start poses
    left_start = np.array([0.55, 0.35, 0.4, 0.0, 0.0, -0.25*math.pi])
    right_start = np.array([0.55, -0.35, 0.4, 0.0, 0.0, 0.25*math.pi]) #[0.45, -0.35, 0.3, 0.0, 0.0, -0.25*math.pi]
#    left_start = np.array([0.55, 0.6, 0.4, 0.0, 0.0, 0.25*math.pi])
#    right_start = np.array([0.55, -0.1, 0.4, 0.0, 0.0, -0.25*math.pi]) #[0.45, -0.35, 0.3, 0.0, 0.0, -0.25*math.pi]
#    left_start = np.array([0.35, 0.4, 0.6, 0.0, 0.0, 0.25*math.pi])
#    right_start = np.array([0.55, -0.4, 0.2, 0.0, 0.0, -0.25*math.pi]) 

    # left_goal is the variable to optimize

  
    ### Go to Initial positon, easy for later pso!!!
    print("Left Arm: Go to start pose before planning...")
    group = moveit_commander.MoveGroupCommander("left_arm")
    pose_target = group.get_current_pose("left_ee_link") # get current eef's pose
    pose_target.pose.position.x = left_start[0]
    pose_target.pose.position.y = left_start[1]
    pose_target.pose.position.z = left_start[2]
    quat = tf.transformations.quaternion_from_euler(left_start[3], left_start[4], left_start[5])
    pose_target.pose.orientation.x = quat[0]
    pose_target.pose.orientation.y = quat[1] 
    pose_target.pose.orientation.z = quat[2]
    pose_target.pose.orientation.w = quat[3] # set the start pos
    group.set_pose_target(pose_target, 'left_ee_link') # set pose target
    group.go(wait=True) # plan and go
    group.stop() # stop
    group.clear_pose_targets() # clear targets
    print("Right Arm: Go to start pose before planning...")
    group = moveit_commander.MoveGroupCommander("right_arm")
    pose_target = group.get_current_pose("right_ee_link") # get current eef's pose
    pose_target.pose.position.x = right_start[0]
    pose_target.pose.position.y = right_start[1]
    pose_target.pose.position.z = right_start[2]
    quat = tf.transformations.quaternion_from_euler(right_start[3], right_start[4], right_start[5])
    pose_target.pose.orientation.x = quat[0]
    pose_target.pose.orientation.y = quat[1] 
    pose_target.pose.orientation.z = quat[2]
    pose_target.pose.orientation.w = quat[3] # set the start pos
    group.set_pose_target(pose_target, 'right_ee_link') # set pose target
    group.go(wait=True) # plan and go
    group.stop() # stop
    group.clear_pose_targets() # clear targets


    ### Perform PSO
    ''
    import pdb
    pdb.set_trace()
    t = time.time() # record time used

    # Set-up hyperparameters as dict
    options = {'c1': 2.0, 'c2': 2.0, 'w':0.8} #{'c1': 1.0, 'c2': 2.0, 'w':0.9} #{'c1': 0.5, 'c2': 0.3, 'w':0.9}

    # set bounds
    bounds = [(0.3, 0.6), (-0.6, 0.6), (0.2, 0.6)] #, (-math.pi, math.pi), (-math.pi, math.pi), (-math.pi, math.pi)]

    # set the assembly constraint, i.e. the goal relative transformation matrix
    rot_z = tf.transformations.euler_matrix(0.0, 0.0, math.pi)
    trans_x = tf.transformations.identity_matrix()
    offset = -0.32
    trans_x[0, 3] = offset
    goal_rel_trans = np.dot(rot_z, trans_x) # moving frame, post-multiply

    # set initials for PSO!!! (this could set to the middle point of two arms' starting positions)
    '''
    num_particles = 10
    initials = []
    tmp = (left_start[:3] + right_start[:3]) / 2 # set the middle point as one initial particle
    initials.append(tmp.tolist()) 
    while len(initials) < num_particles:
      # initialize
      tmp = [random.uniform(bounds[0][0], bounds[0][1]), random.uniform(bounds[1][0], bounds[1][1]), random.uniform(bounds[2][0], bounds[2][1])]
      # construct left goal and compute right goal
      left_goal_pose = [0, 0, -0.5*math.pi]
      left_goal = tmp + left_goal_pose
      left_goal_trans = tf.transformations.euler_matrix(left_goal[3], left_goal[4], left_goal[5])
      left_goal_trans[:3, 3] = np.array(tmp)
      right_goal_trans = np.dot(left_goal_trans, goal_rel_trans) # right-multiply ???
      right_goal = np.concatenate((right_goal_trans[:3, 3], tf.transformations.euler_from_matrix(right_goal_trans[:3][:3])))
      # check constraint on right goal
      within_bounds = True
      for l in range(len(bounds)):
        if right_goal[l] <= bounds[l][0] or right_goal[l] >= bounds[l][1]:
          within_bounds = False
      if within_bounds: initials.append(copy.deepcopy(tmp))
     
    # Create an instance of PSO optimizer
    pso_cost_func = PSOCostFunc(goal_rel_trans)
    PSO_instance = simple_PSO.PSO(pso_cost_func.f, initials, bounds, num_particles=num_particles, maxiter=20, verbose=True, options=options)
    cost, pos = PSO_instance.result()
    elapsed = time.time() - t # time used
    print('========= Time used for PSO : ' + str(elapsed) + ' s')
    '''


    # Create an instance of GD optimizer
    ''
    x0 = (left_start[:3] + right_start[:3]) / 2 # set the middle point as one initial particle
    x0 = x0.tolist()
    options = {'alpha':0.01, 'epsilon':0.0001, 'precision':0.02}
    cost_func = PSOCostFunc(goal_rel_trans)    
    gd_instance = simple_GD.GD_Optimizer(cost_func.f, bounds, maxiter=20, options=options)
    cost, pos = gd_instance.train(x0)
    elapsed = time.time() - t # time used
    print('========= Time used for GD Optimizer : ' + str(elapsed) + ' s')
    ''

    import pdb
    pdb.set_trace()    


    # display the cost history
    print("Display the cost history and step history")
    # step size is not a good stopping criterion!!!
    cost_history = copy.deepcopy(gd_instance.cost_history) #copy.deepcopy(PSO_instance.cost_history) #
    step_history = copy.deepcopy(gd_instance.step_history) #copy.deepcopy(PSO_instance.leader_step_history) #
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(nrows=2, ncols=1)
    fig.suptitle('GD Optimizer - learning_rate: ' + str(options['alpha']) + ', epsilon: ' + str(options['epsilon']) + ', precision: ' + str(options['precision']), fontsize=18)
    #fig.suptitle('PSO Optimizer - #particles: ' + str(num_particles) + ', c1: ' + str(options['c1']) + ', c2: ' + str(options['c2']) + ', w: ' + str(options['w']), fontsize=18)
    ax[0].set(title='Cost history') # set title and labels
    ax[0].set_xlabel('Iteration')
    ax[0].set_ylabel('Cost value')
    ax[0].plot(range(len(cost_history)), cost_history) # line plot
    ax[0].scatter(range(len(cost_history)), cost_history, marker='*') # draw scatter points
    ax[0].set_xlim([0, len(cost_history)]) # set x and y limits
    ax[0].set_ylim([0.0, 1.5]) 
    for xy in zip(range(len(cost_history)), cost_history):
      ax[0].annotate('{0:.3f}'.format(xy[1]), xy=xy)
    
    ax[1].set(title='Step history') # set title and labels
    ax[1].set_xlabel('Iteration')
    ax[1].set_ylabel('Step size')
    ax[1].plot(range(len(step_history)), step_history) # line plot
    ax[1].scatter(range(len(step_history)), step_history, marker='o') # draw scatter points
    ax[1].set_xlim([0, len(step_history)]) # set x and y limits
    for xy in zip(range(len(step_history)), step_history):
      ax[1].annotate('{0:.3f}'.format(xy[1]), xy=xy)

    plt.show()
    
    import pdb
    pdb.set_trace()


  
    # add precision(time resolution) when displaying the result!! 
    t0 = time.time()
    left_goal_pos = pos # use the result of PSO/GD

    # re-modify the left_goal here  
    #print("===== Re-modify the left_goal for comparison...")
    #left_goal_pos = [0.452955, 0.009273, 0.473311]
    #left_goal_pos[0] = left_goal_pos[0] + 0.1 # move x, forward
    #left_goal_pos[1] = left_goal_pos[1] + 0.1 # move y, to right
    #left_goal_pos[2] = left_goal_pos[2] + 0.1 # move z, up


    left_goal_pose = [0, 0, -0.5*math.pi]
    left_goal = left_goal_pos + left_goal_pose
    l_group = moveit_commander.MoveGroupCommander("left_arm")
    r_group = moveit_commander.MoveGroupCommander("right_arm")
    t1 = time.time()

    ### Compute trajs for the optimized left_goal
    left_start = np.array([0.55, 0.35, 0.4, 0.0, 0.0, -0.25*math.pi])
    right_start = np.array([0.55, -0.35, 0.4, 0.0, 0.0, 0.25*math.pi]) #[0.45, -0.35, 0.3, 0.0, 0.0, -0.25*math.pi]
#    left_start = np.array([0.55, 0.6, 0.4, 0.0, 0.0, 0.25*math.pi])
#    right_start = np.array([0.55, -0.1, 0.4, 0.0, 0.0, -0.25*math.pi]) #[0.45, -0.35, 0.3, 0.0, 0.0, -0.25*math.pi]
#    left_start = np.array([0.35, 0.4, 0.6, 0.0, 0.0, 0.25*math.pi])
#    right_start = np.array([0.55, -0.4, 0.2, 0.0, 0.0, -0.25*math.pi]) 
    ## 1 - set right arm's goal
    print("========== Set right arm's goal")
    # get right arm's pose
    l_w_final = tf.transformations.quaternion_from_euler(left_goal[3], left_goal[4], left_goal[5])
    relative_quat = tf.transformations.quaternion_from_euler(0, 0, math.pi)
    r_w_final = tf.transformations.quaternion_multiply(l_w_final, relative_quat)
    right_rotm = tf.transformations.quaternion_matrix(r_w_final)
    right_rotm = right_rotm[:3, :3] # get the rotation part
    # compute offset vector along x axis
    offset = -0.32
    v_offset = [offset, 0, 0] 
    v_offset_world = np.dot(right_rotm, v_offset)
    r_x_final = left_goal[0:3] + v_offset_world
    # construct right arm's goal
    right_goal = np.concatenate((r_x_final, tf.transformations.euler_from_quaternion(r_w_final)))

    ## 2 - DMP, new cartesian paths
    ndata = 50
    l_cartesian_path = np.zeros((6, ndata), dtype=float)
    r_cartesian_path = np.zeros((6, ndata), dtype=float)
    for i in range(6):
      l_cartesian_path[i, :] = np.linspace(left_start[i], left_goal[i], num=ndata, endpoint=True)
      r_cartesian_path[i, :] = np.linspace(right_start[i], right_goal[i], num=ndata, endpoint=True)

    t2 = time.time()


    # display to check the smoothness
    '''
    print("Display the DMP path to check smoothness")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    fig = plt.figure() # create a figure object
    ax1 = fig.add_subplot(121, projection='3d')
    ax1.set(title='Left Arm''s cartesian path')
    ax1.scatter3D(l_cartesian_path[0, :], l_cartesian_path[1, :], l_cartesian_path[2, :], cmap='Blues')
    ax1.plot3D(l_cartesian_path[0, :], l_cartesian_path[1, :], l_cartesian_path[2, :], 'gray')
    ax1.set_xlabel('x')
    ax1.set_ylabel('y')
    ax1.set_zlabel('z')
    ax2 = fig.add_subplot(122, projection='3d')
    ax2.set(title='Right Arm''s cartesian path')
    ax2.scatter3D(r_cartesian_path[0, :], r_cartesian_path[1, :], r_cartesian_path[2, :], cmap='Blues')
    ax2.plot3D(r_cartesian_path[0, :], r_cartesian_path[1, :], r_cartesian_path[2, :])
    ax2.set_xlabel('x')
    ax2.set_ylabel('y')
    ax2.set_zlabel('z')
    plt.show()
    
    import pdb
    pdb.set_trace()
    '''



    ## 3 - IK
    # LEFT ARM
    print("========== Perform IK for left arm's path...")
    # set Pose trajectories(should go to first pose before planning!!!)
    print("-- Plan for left arm...")
    waypoints = []
    wpose = geometry_msgs.msg.Pose()
    for n in range(1, l_cartesian_path.shape[1]):
      wpose.position.x = l_cartesian_path[0, n]
      wpose.position.y = l_cartesian_path[1, n]
      wpose.position.z = l_cartesian_path[2, n]
      quat = tf.transformations.quaternion_from_euler(l_cartesian_path[3, n], l_cartesian_path[4, n], l_cartesian_path[5, n]) # same order ???
      wpose.orientation.x = quat[0]
      wpose.orientation.y = quat[1]
      wpose.orientation.z = quat[2]
      wpose.orientation.w = quat[3]
      waypoints.append(copy.deepcopy(wpose))
    tt0 = time.time()
    (plan, fraction) = l_group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01, #0.01,        # eef_step # set to 0.001 for collecting the data
                                        0.0,     # jump_threshold
                                        avoid_collisions=False)    
    tt1 = time.time()
    print(">>> Time used for compute_cartesian_path only(left): " + str(tt1-tt0) + " s")    

    # store the generated joint plans
    l_joint_path_plan = copy.deepcopy(plan) # stored as moveit_msgs/RobotTrajectory
    # RIGHT ARM
    # set Pose trajectories(should go to first pose before planning!!!)
    print("-- Plan for right arm...")
    waypoints = []
    wpose = geometry_msgs.msg.Pose()
    for n in range(1, r_cartesian_path.shape[1]):
      wpose.position.x = r_cartesian_path[0, n]
      wpose.position.y = r_cartesian_path[1, n]
      wpose.position.z = r_cartesian_path[2, n]
      quat = tf.transformations.quaternion_from_euler(r_cartesian_path[3, n], r_cartesian_path[4, n], r_cartesian_path[5, n]) # same order ???
      wpose.orientation.x = quat[0]
      wpose.orientation.y = quat[1]
      wpose.orientation.z = quat[2]
      wpose.orientation.w = quat[3]
      waypoints.append(copy.deepcopy(wpose))
    tt0 = time.time()
    (plan, fraction) = r_group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01, #0.01,        # eef_step # set to 0.001 for collecting the data
                                        0.0,     # jump_threshold
                                        avoid_collisions=False)    
    tt1 = time.time()
    print(">>> Time used for compute_cartesian_path only(right): " + str(tt1-tt0) + " s")  

    # store the generated joint plans
    r_joint_path_plan = copy.deepcopy(plan) # stored as moveit_msgs/RobotTrajectory
    t3 = time.time()

    # display the result
    #import pdb
    #pdb.set_trace()
    #l_group.execute(l_joint_path_plan, wait=True) 
    #r_group.execute(r_joint_path_plan, wait=True)
  

    # 3.5 - store the IK results for comparison between TOTG, TOPP and TOPP-RA 
    import pdb
    pdb.set_trace()
    print("Convert IK plans to arrays and store in h5 file, for later comparison between TOTG, TOPP and TOPP-RA.")
    len_l = len(l_joint_path_plan.joint_trajectory.points)
    len_r = len(r_joint_path_plan.joint_trajectory.points)
    l_joint_path_array = np.zeros((6, len_l))
    r_joint_path_array = np.zeros((6, len_r))    
    for ll in range(len_l):
      l_joint_path_array[:, ll] = np.array(l_joint_path_plan.joint_trajectory.points[ll].positions)
    for rr in range(len_r):
      r_joint_path_array[:, rr] = np.array(r_joint_path_plan.joint_trajectory.points[rr].positions)
    # store
    f = h5py.File("example_paths_for_TP_comparison.h5", "a") 
    l_path_group =  f.create_group("l_joint_path_array_1")
    l_path_group.create_dataset("pos", data=l_joint_path_array, dtype=float)
    r_path_group =  f.create_group("r_joint_path_array_1")
    r_path_group.create_dataset("pos", data=r_joint_path_array, dtype=float)
    f.close()


    # 4 - TOTG
    print("========== Apply TOTG to get minimum time... ")
    # set up joint kinematic constraints
    vel_limits = [3.15, 3.15, 3.15, 3.15, 3.15, 3.15]
    acc_limits = [3.15, 3.15, 3.15, 3.15, 3.15, 3.15] #[10.0, 10.0, 10.0, 10.0, 10.0, 10.0]
    # get minimum time for left arm's motion using TOTG
    print("-- for left arm...")   
    tmp_plan = copy.deepcopy(l_joint_path_plan)
    r_min_time = get_minimum_time_client(tmp_plan.joint_trajectory.points, vel_limits, acc_limits)

    # get minimum time for right arm's motion using TOTG
    print("-- for right arm...")   
    tmp_plan = copy.deepcopy(r_joint_path_plan)
    l_min_time = get_minimum_time_client(tmp_plan.joint_trajectory.points, vel_limits, acc_limits)
  
    t4 = time.time()
    print('>>>>> Statistics:')
    print('>>>>>   Time used for setting up moveit commander: ' + str(t1-t0) + ' s')
    print('>>>>>   Time used for setting up goals and fake DMP: ' + str(t2-t1) + ' s')
    print('>>>>>   Time used for IK: ' + str(t3-t2) + ' s')
    print('>>>>>   Time used for getting minimum time from TOTG: ' + str(t4-t3) + ' s')
    print('>>>>> Total time used: ' + str(t4-t0) + ' s')

    

    # Test on C++ API -------
    '''
    import pdb
    pdb.set_trace()
    ## IK using C++ API through service
    t0 = time.time()
    # LEFT ARM
    print("========== C++ version of IK for left arm...")
    # set Pose trajectories(should go to first pose before planning!!!)
    waypoints = []
    wpose = geometry_msgs.msg.Pose()
    for n in range(1, l_cartesian_path.shape[1]):
      wpose.position.x = l_cartesian_path[0, n]
      wpose.position.y = l_cartesian_path[1, n]
      wpose.position.z = l_cartesian_path[2, n]
      quat = tf.transformations.quaternion_from_euler(l_cartesian_path[3, n], l_cartesian_path[4, n], l_cartesian_path[5, n]) # same order ???
      wpose.orientation.x = quat[0]
      wpose.orientation.y = quat[1]
      wpose.orientation.z = quat[2]
      wpose.orientation.w = quat[3]
      waypoints.append(copy.deepcopy(wpose))

    l_joint_path_plan = compute_cartesian_path_client('left_arm', waypoints)
    elapsed0 = time.time() - t0
    print(">>> Time used for C++ version of IK through service call(including communication): " + str(elapsed0) + " s")

    # RIGHT ARM
    t1 = time.time()
    print("========== C++ version of IK for right arm...")
    # set Pose trajectories(should go to first pose before planning!!!)
    waypoints = []
    wpose = geometry_msgs.msg.Pose()
    for n in range(1, r_cartesian_path.shape[1]):
      wpose.position.x = r_cartesian_path[0, n]
      wpose.position.y = r_cartesian_path[1, n]
      wpose.position.z = r_cartesian_path[2, n]
      quat = tf.transformations.quaternion_from_euler(r_cartesian_path[3, n], r_cartesian_path[4, n], r_cartesian_path[5, n]) # same order ???
      wpose.orientation.x = quat[0]
      wpose.orientation.y = quat[1]
      wpose.orientation.z = quat[2]
      wpose.orientation.w = quat[3]
      waypoints.append(copy.deepcopy(wpose))

    r_joint_path_plan = compute_cartesian_path_client('right_arm', waypoints)
    elapsed1 = time.time() - t1
    print(">>> Time used for C++ version of IK through service call(including communication): " + str(elapsed1) + " s")

    # test the performance of generated plan
    pdb.set_trace()
    l_group.execute(l_joint_path_plan, wait=True) 
    pdb.set_trace()
    r_group.execute(r_joint_path_plan, wait=True)

    # END of Test on C++ API ---------
    
    import pdb
    pdb.set_trace()
    '''


    # 5 - get optimal plans and merge two plans into one dual-arm motion plan
    t5 = time.time()
    print("========= Merge into two plans...")
    tmp_plan_l = copy.deepcopy(l_joint_path_plan)
    new_plan_l = moveit_msgs.msg.RobotTrajectory() 
    new_plan_l.joint_trajectory.points = add_time_optimal_parameterization_client(tmp_plan_l.joint_trajectory.points, vel_limits, acc_limits, 0.01) # keep in consistent with the training procedure
    tmp_plan_r = copy.deepcopy(r_joint_path_plan)
    new_plan_r = moveit_msgs.msg.RobotTrajectory() 
    new_plan_r.joint_trajectory.points = add_time_optimal_parameterization_client(tmp_plan_r.joint_trajectory.points, vel_limits, acc_limits, 0.01) # keep in consistent with the training procedure
    t6 = time.time()
    print('>>>>>   Time used for getting whole trajectory from TOTG: ' + str(t6-t5) + ' s')
    # get minimum time
    r_min_time = new_plan_r.joint_trajectory.points[-1].time_from_start.to_sec()
    l_min_time = new_plan_l.joint_trajectory.points[-1].time_from_start.to_sec()
    # timesteps are the same, so it should be ok to append directly
    len_l = len(new_plan_l.joint_trajectory.points) 
    len_r = len(new_plan_r.joint_trajectory.points)
    max_len = max(len_l, len_r)
    min_len = min(len_l, len_r)
    # assign
    new_plan_dual = moveit_msgs.msg.RobotTrajectory()
    new_plan_dual.joint_trajectory.header.frame_id = '/world'
    new_plan_dual.joint_trajectory.joint_names = ['left_shoulder_pan_joint', 'left_shoulder_lift_joint', 'left_elbow_joint', 'left_wrist_1_joint', 'left_wrist_2_joint', 'left_wrist_3_joint', 'right_shoulder_pan_joint', 'right_shoulder_lift_joint', 'right_elbow_joint', 'right_wrist_1_joint', 'right_wrist_2_joint', 'right_wrist_3_joint']
    traj_point = trajectory_msgs.msg.JointTrajectoryPoint()
    for m in range(min_len):
      traj_point.positions = new_plan_l.joint_trajectory.points[m].positions + new_plan_r.joint_trajectory.points[m].positions
      traj_point.velocities = new_plan_l.joint_trajectory.points[m].velocities + new_plan_r.joint_trajectory.points[m].velocities
      traj_point.accelerations = new_plan_l.joint_trajectory.points[m].accelerations + new_plan_r.joint_trajectory.points[m].accelerations
      traj_point.time_from_start = new_plan_l.joint_trajectory.points[m].time_from_start
      new_plan_dual.joint_trajectory.points.append(copy.deepcopy(traj_point))
    # append the end
    for n in range(min_len, max_len):
      if len_l > len_r:
        traj_point.positions = new_plan_l.joint_trajectory.points[n].positions + new_plan_r.joint_trajectory.points[-1].positions
        traj_point.velocities = new_plan_l.joint_trajectory.points[n].velocities + new_plan_r.joint_trajectory.points[-1].velocities
        traj_point.accelerations = new_plan_l.joint_trajectory.points[n].accelerations + new_plan_r.joint_trajectory.points[-1].accelerations
        traj_point.time_from_start = new_plan_l.joint_trajectory.points[n].time_from_start
        new_plan_dual.joint_trajectory.points.append(copy.deepcopy(traj_point))
      else:
        traj_point.positions = new_plan_l.joint_trajectory.points[-1].positions + new_plan_r.joint_trajectory.points[n].positions
        traj_point.velocities = new_plan_l.joint_trajectory.points[-1].velocities + new_plan_r.joint_trajectory.points[n].velocities
        traj_point.accelerations = new_plan_l.joint_trajectory.points[-1].accelerations + new_plan_r.joint_trajectory.points[n].accelerations
        traj_point.time_from_start = new_plan_r.joint_trajectory.points[-1].time_from_start
        new_plan_dual.joint_trajectory.points.append(copy.deepcopy(traj_point))    


    import pdb
    pdb.set_trace()


    ### Execute the dual-arm plan
    print("======= Get to ready position...")
    group = moveit_commander.MoveGroupCommander("dual_arms")
    # go to start position first
    joint_goal = new_plan_dual.joint_trajectory.points[0].positions
    group.go(joint_goal, wait=True)
    group.stop()
    # execute the plan now
    import pdb
    pdb.set_trace()
    print("======== Execute the dual plan..")
    group.execute(new_plan_dual, wait=True)
    rospy.sleep(3.0)


    import pdb
    pdb.set_trace()


    ### Display the cost history
    '''
    print("Displaying the cost history now")
    import matplotlib.pyplot as plt
    cost_history = pso_cost_func.cost_history
    plt.figure()
    plt.plot(range(len(cost_history)), cost_history)    
    plt.xlabel('Iteration')
    plt.ylabel('Cost')
    plt.show() # can save the image through GUI


    import pdb
    pdb.set_trace()
    '''


    ### Store the result for display in MATLAB
    '''
    f = h5py.File("dual_ur5_whole_traj_concat_2.h5", "a")
    joint_traj_name = "traj_pair_approach2_insert2_tmp"
    joint_traj_group =  f.create_group(joint_traj_name)
    joint_traj_group.create_dataset("joint_traj_l", data=whole_traj_l, dtype=float)
    joint_traj_group.create_dataset("joint_traj_r", data=whole_traj_r, dtype=float)
    joint_traj_group.create_dataset("joint_timestamp_l", data=whole_timestamp_l, dtype=float)    
    joint_traj_group.create_dataset("joint_timestamp_r", data=whole_timestamp_r, dtype=float)
    f.close()
    '''
  
  
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







