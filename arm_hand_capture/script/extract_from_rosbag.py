#!/usr/bin/env python
import rosbag
import pdb
import h5py
import numpy as np
import tf


def pack_pose_to_array(wri_pose, elb_pose):

  # from quaternion([x,y,z,w]) to rotation matrix
  wri_rotm = tf.transformations.quaternion_matrix([wri_pose.orientation.x, wri_pose.orientation.y, wri_pose.orientation.z, wri_pose.orientation.w])

  # pack up
  packed_state = [wri_pose.position.x, wri_pose.position.y, wri_pose.position.z] + wri_rotm.reshape(9).tolist() + [elb_pose.position.x, elb_pose.position.y, elb_pose.position.z

  return packed_state


if __name__ == '__main__':

  ### Set up parameters
  bag_name = 'swing_arm' # no `.bag` here
  start_time = None # used to transform Unix timestamp to timestamps starting from 0
  

  ### Iterate to get the message contents
  bag_file = rosbag.Bag(bag_name + '.bag')
  count = bag_file.get_message_count()
  dim = 3 + 9 + 3
  left_path = np.zeros([count, dim])
  right_path = np.zeros([count, dim])
  time = np.zeros([count, 1])
  idx = 0
  for topic, msg, t in bag_file.read_messages():
    ## topic name
    print("Topic name is: " + topic)
    #pdb.set_trace()

    ## do not use `t`!!! because it is the time at which a message is recorded through rosbag, instead of the original timestamp
    if start_time is None:
      start_time = msg.right_forearm_pose.header.stamp.to_sec()

    ## message content (record only the wrist pos+rot and elbow pos; timestamps are `approximately` identical)
    time[idx] = msg.right_forearm_pose.header.stamp.to_sec() - start_time
    right_tmp = 
    pdb.set_trace()
    #left_path[idx, :] = 
    #right_path[idx, :] = 
    #msg.right_forearm_pose
    #msg.right_hand_pose
    #msg.left_forearm_pose
    #msg.left_hand_pose
    #idx = idx + 1


  ### Store the results
  h5_name = 'synced_results.h5'
  group_name = bag_name
  
  


    

