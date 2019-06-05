#!/usr/bin/env python
import rospy
import actionlib

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

if __name__ == '__main__':
    
    # Initialize a ros node
    rospy.init_node('trajectory_demo')


    # the names of the joints of the arm
    gripper_joints =  ['right_rh_p12_rn', 'right_rh_r2', 'right_rh_l1', 'right_rh_l2']


    # Connect to the right arm trajectory action server
    print "Connecting to the right gripper trajectory action server..."
    gripper_client = actionlib.SimpleActionClient('/dual_ur5_arm/right_gripper_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    gripper_client.wait_for_server()
    print "Connected"

    
    # Create an arm trajectory with the arm_goal as the end-point
    print "Setting up a goal trajectory..."
    gripper_trajectory = JointTrajectory()
    gripper_trajectory.joint_names = gripper_joints
    gripper_trajectory.points.append(JointTrajectoryPoint())
    ang = 0.2 # upper is 1.1 # offset from ground is 0.142 when ang=0.72
    gripper_trajectory.points[0].positions =  [ang, ang/1.1, ang, ang/1.1]   
    gripper_trajectory.points[0].velocities = [0.0 for i in gripper_joints]
    gripper_trajectory.points[0].accelerations = [0.0 for i in gripper_joints]
    gripper_trajectory.points[0].time_from_start = rospy.Duration(0.5) 

    
    # Create an empty trajectory goal
    gripper_goal = FollowJointTrajectoryGoal()


    # set the trajectory component to the goal trajectory
    gripper_goal.trajectory = gripper_trajectory


    # specify zero tolerance for the execution time
    gripper_goal.goal_time_tolerance = rospy.Duration(0.0)
    

    # Send the goal to the action server
    print "Sending the goal to the action server..."
    gripper_client.send_goal(gripper_goal) 


    # Wait for up to 5 seconds for the motion to complete
#    print "Wait for the goal transitions to complete..."
#    gripper_client.wait_for_result(rospy.Duration(10.0))
#    if gripper_client.get_state() == actionlib.SimpleClientGoalState.SUCCEEDED:
#        print "The action is done."
#    else:
#        print "action is not yet done."
 
    gripper_client.wait_for_result(rospy.Duration(5.0))
    print "Done."

    #result = gripper_client.get_result()
    #print result.SUCCESSFUL
    
    
    
    
    
    
    

