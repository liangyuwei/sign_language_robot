#!/usr/bin/env python

import sys
import copy
import rospy
import math
import tf
import h5py
import numpy as np
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import trajectory_msgs.msg

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
    group_name = "right_hand"
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

def linear_map(x_, min_, max_, min_hat, max_hat):

  x_hat = 1.0 * (x_ - min_) / (max_ - min_) * (max_hat - min_hat) + min_hat  

  return x_hat



def map_glove_to_inspire_hand(glove_angles):

  ### This function linearly maps the Wiseglove angle measurement to Inspire hand's joint angles.

  ## preparation, specify the range for linear scaling
  hand_start = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3, 0.4, 0.0, 0.0]) # radius already
  hand_final = np.array([-1.6, -1.7, -1.6, -1.7, -1.6, -1.7, -1.6, -1.7, -1.0, 0.0, -0.4, -1.0])
  glove_start = np.array([0, 0, 53, 0, 0, 22, 0, 0, 22, 0, 0, 35, 0, 0]) * pi / 180.0 # degree to radius
  glove_final = np.array([45, 100, 0, 90, 120, 0, 90, 120, 0, 90, 120, 0, 90, 120]) * pi / 180.0
  length = glove_angles.shape[0]
  hand_angles = np.zeros((length, 12)) # 12 joints

  ## Iterate to map angles
  for i in range(length):
    # four fingers' extension/flexion (abduction/adduction are dumped)
    hand_angles[i, 0] = linear_map(glove_angles[i, 3], glove_start[3], glove_final[3], hand_start[0], hand_final[0]) # Link1 (joint name)
    hand_angles[i, 1] = linear_map(glove_angles[i, 4], glove_start[4], glove_final[4], hand_start[1], hand_final[1]) # Link11
    hand_angles[i, 2] = linear_map(glove_angles[i, 6], glove_start[6], glove_final[6], hand_start[2], hand_final[2]) # Link2
    hand_angles[i, 3] = linear_map(glove_angles[i, 7], glove_start[7], glove_final[7], hand_start[3], hand_final[3]) # Link22
    hand_angles[i, 4] = linear_map(glove_angles[i, 9], glove_start[9], glove_final[9], hand_start[4], hand_final[4]) # Link3
    hand_angles[i, 5] = linear_map(glove_angles[i, 10], glove_start[10], glove_final[10], hand_start[5], hand_final[5]) # Link33
    hand_angles[i, 6] = linear_map(glove_angles[i, 12], glove_start[12], glove_final[12], hand_start[6], hand_final[6]) # Link4
    hand_angles[i, 7] = linear_map(glove_angles[i, 13], glove_start[13], glove_final[13], hand_start[7], hand_final[7]) # Link44

    # thumb
    hand_angles[i, 8] = (hand_start[8] + hand_final[8]) / 2.0 # Link5 (rotation about z axis), fixed!
    hand_angles[i, 9] = linear_map(glove_angles[i, 2], glove_start[2], glove_final[2], hand_start[9], hand_final[9]) # Link 51
    hand_angles[i, 10] = linear_map(glove_angles[i, 0], glove_start[0], glove_final[0], hand_start[10], hand_final[10]) # Link 52
    hand_angles[i, 11] = linear_map(glove_angles[i, 1], glove_start[1], glove_final[1], hand_start[11], hand_final[11]) # Link 53

  return hand_angles


def main():


  try:

    ### Set up the moveit_commander
    print "============ Setting up the moveit_commander (press ctrl-d to exit) ..."
    group = moveit_commander.MoveGroupCommander("right_hand")


    ### Go to the given joint state
    print "============ Go to initial joint state ..."
    joint_goal = np.zeros(12).tolist()
    group.go(joint_goal, wait=True)
    group.stop()


    ### Planning of two ur5 arms: go to pose goal
    print "========== Load joint trajectory from h5 file "
    file_name = 'glove_test_data'
    group_name = 'right_glove_test_1' #'right_glove_test_1'
    f = h5py.File(file_name+'.h5', "r")
    timestamps = f[group_name+'/time'][:]
    r_glove_angles = f[group_name+'/r_glove_angle'][:]
    r_glove_angles = r_glove_angles * pi / 180 # convert degree to radius!!!
    r_hand_angles = map_glove_to_inspire_hand(r_glove_angles) # linearly map to inspire hand


    ### Create a plan using the imported trajectory
    print "========== Create a plan from imported trajectory "
    joint_plan = moveit_msgs.msg.RobotTrajectory()
    joint_plan.joint_trajectory.header.frame_id = '/world'
    joint_plan.joint_trajectory.joint_names = ['Link1', 'Link11', 'Link2', 'Link22', 'Link3', 'Link33', 'Link4', 'Link44', 'Link5', 'Link51', 'Link52', 'Link53']
    for i in range(r_hand_angles.shape[0]):
      traj_point = trajectory_msgs.msg.JointTrajectoryPoint()
      traj_point.positions = r_hand_angles[i, :].tolist()
      t = rospy.Time(i * 1.0/15.0) #rospy.Time(timestamps[i]) # to coordinate with the 15 Hz frame rate of the exported video (though it's not the original timestamps)
      traj_point.time_from_start.secs = t.secs
      traj_point.time_from_start.nsecs = t.nsecs
      joint_plan.joint_trajectory.points.append(copy.deepcopy(traj_point))


    ### Go to the start point
    print "========== Go to the starting position "
    joint_goal = joint_plan.joint_trajectory.points[0].positions
    group.go(joint_goal, wait=True)
    group.stop()


    ### Execute a plan
    print "============ Execute a saved path ..."    
    import pdb
    pdb.set_trace()
    group.execute(joint_plan, wait=True)

        
  except rospy.ROSInterruptException:
    return

  except KeyboardInterrupt:
    return


if __name__ == '__main__':

  main()







