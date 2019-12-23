#!/usr/bin/env python
import rosbag
import pdb

if __name__ == '__main__':


  ## Set up parameters
  bag_name = 'swing_arm.bag'
  start_time = None # used to transform Unix timestamp to timestamps starting from 0


  ## Iterate to get the message contents
  for topic, msg, t in rosbag.Bag('swing_arm.bag').read_messages():
    # topic name
    print("Topic name is: " + topic)

    # do not use `t`!!! because it is the time at which a message is recorded through rosbag, instead of the original timestamp

    # message content
    msg.right_upperarm_pose 




pdb.set_trace()
    

