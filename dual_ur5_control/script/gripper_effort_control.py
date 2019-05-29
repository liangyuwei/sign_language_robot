#!/usr/bin/env python
import rospy
import actionlib
from std_msgs.msg import Float64MultiArray, Float64, MultiArrayDimension

if __name__ == '__main__':

    # Initialize a ROS node
    rospy.init_node('gripper_effort_control')
    
    # Publish command
    topic_name = "/dual_ur5_arm/left_gripper_controller/command"
    pub = rospy.Publisher(topic_name, Float64MultiArray, queue_size=100)

    # Create the message
    msg = Float64MultiArray()
    msg.layout.dim.append(MultiArrayDimension())
    msg.layout.dim[0].size = 2
    msg.layout.dim[0].stride = 1
    msg.layout.dim[0].label = 'open_fingers'
    # msg.data = [-90.0, -90.0]
    msg.data = [10, 10, 10, 10]

    # Publish the message
    #pub.publish(msg)
    pub.publish(msg)
    rospy.sleep(2)
    pub.publish(msg)
    rospy.sleep(2)
    pub.publish(msg)
   # rospy.sleep(2)
   # pub.publish(msg)

    # rate = rospy.Rate(10) # Hertz
    # while not rospy.is_shutdown():
    #     print "Sending messages..."
    #     pub.publish(msg)
    #     rate.sleep()

    ### Control each joint independently
    #pub_l = rospy.Publisher('/gripper_l_finger_controller/command', Float64, queue_size=100)
    #pub_r = rospy.Publisher('/gripper_r_finger_controller/command', Float64, queue_size=100)

    # Publish messages
    #if True:
    #    pub_l.publish(-10.0)
    #    pub_r.publish(-10.0)
