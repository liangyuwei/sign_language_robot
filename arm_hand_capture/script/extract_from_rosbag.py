#!/usr/bin/env python
import rosbag
import pdb
import h5py
import numpy as np
import tf

import sys
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError


def pos_to_ndarray(pos):
  # combined position and orientation information of a geometry_msgs/Pose message into a list

  # from quaternion([x,y,z,w]) to rotation matrix
  #wri_rotm = tf.transformations.quaternion_matrix([wri_pose.orientation.x, wri_pose.orientation.y, wri_pose.orientation.z, wri_pose.orientation.w])

  # pack up
  #packed_state = [wri_pose.position.x, wri_pose.position.y, wri_pose.position.z] + wri_rotm.reshape(9).tolist() + [elb_pose.position.x, elb_pose.position.y, elb_pose.position.z]

  return np.array([pos.x, pos.y, pos.z])

def quat_to_ndarray(quat):

  return np.array([quat.x, quat.y, quat.z, quat.w])


def bag_to_h5_video(bag_name, h5_name, fps=15.0):
  ## This function converts the rosbag file to h5 containing all the data(hand, forearm, upperarm) and the corresponding video.

  
  # Iterate to get the message contents
  bag_file = rosbag.Bag(bag_name + '.bag')
  count = bag_file.get_message_count()
  bridge = CvBridge()

  idx = 0
  start_time = None # used to transform Unix timestamp to timestamps starting from 0

  time = np.zeros([count, 1])

  l_up_pos = np.zeros([count, 3])
  l_up_quat = np.zeros([count, 4])
  l_fr_pos = np.zeros([count, 3])
  l_fr_quat = np.zeros([count, 4])
  l_hd_pos = np.zeros([count, 3])
  l_hd_quat = np.zeros([count, 4])

  r_up_pos = np.zeros([count, 3])
  r_up_quat = np.zeros([count, 4])
  r_fr_pos = np.zeros([count, 3])
  r_fr_quat = np.zeros([count, 4])
  r_hd_pos = np.zeros([count, 3])
  r_hd_quat = np.zeros([count, 4])


  for topic, msg, t in bag_file.read_messages():
    ## topic name
    #print("Topic name is: " + topic)

    ## h5 content
    # do not use `t`!!! because it is the time at which a message is recorded through rosbag, instead of the original timestamp
    if idx == 0:
      start_time = msg.right_forearm_pose.header.stamp.to_sec()

    time[idx] = msg.right_forearm_pose.header.stamp.to_sec() - start_time

    l_up_pos[idx, :] = pos_to_ndarray(msg.left_upperarm_pose.pose.position)
    l_up_quat[idx, :] = quat_to_ndarray(msg.left_upperarm_pose.pose.orientation)
    l_fr_pos[idx, :] = pos_to_ndarray(msg.left_forearm_pose.pose.position)
    l_fr_quat[idx, :] = quat_to_ndarray(msg.left_forearm_pose.pose.orientation)
    l_hd_pos[idx, :] = pos_to_ndarray(msg.left_hand_pose.pose.position)
    l_hd_quat[idx, :] = quat_to_ndarray(msg.left_hand_pose.pose.orientation)   
 
    r_up_pos[idx, :] = pos_to_ndarray(msg.right_upperarm_pose.pose.position)
    r_up_quat[idx, :] = quat_to_ndarray(msg.right_upperarm_pose.pose.orientation)
    r_fr_pos[idx, :] = pos_to_ndarray(msg.right_forearm_pose.pose.position)
    r_fr_quat[idx, :] = quat_to_ndarray(msg.right_forearm_pose.pose.orientation)
    r_hd_pos[idx, :] = pos_to_ndarray(msg.right_hand_pose.pose.position)
    r_hd_quat[idx, :] = quat_to_ndarray(msg.right_hand_pose.pose.orientation)   

    #import pdb
    #pdb.set_trace()


    ## video
    cv2_img = bridge.imgmsg_to_cv2(msg.image, 'bgr8') # shape is (480, 640, 3), type is ndarray, encoding is 'rgb8'
    (rows, cols, channels) = cv2_img.shape
    if idx == 0:
      video = cv2.VideoCapture(bag_name+'.mp4')
      #fps = 15.0
      size = (cols, rows)
      video_writer = cv2.VideoWriter(bag_name+'.mp4', cv2.VideoWriter_fourcc('X', 'V', 'I', 'D'), fps, size) # 'M', 'J', 'P', 'G'
    video_writer.write(cv2_img)

    ## Set counter
    idx = idx + 1

  video_writer.release()  

  ### Store the results
  # to construct a dataset for release, the data content must be complete!!!
  # l_hand_pos/l_hand_quat, l_forearm_pos/l_forearm_quat, l_upperarm_pos/l_upperarm_quat; after this, do an extraction to get wrist, elbow information for use in sign language robot!!!
  # store each part separately!!!  
  h5_name = 'synced_results'
  group_name = bag_name

  f = h5py.File(h5_name+".h5", "a") # open in append mode
  group = f.create_group(group_name)
  group.create_dataset("l_up_pos", data=l_up_pos, dtype=float)
  group.create_dataset("l_up_quat", data=l_up_quat, dtype=float)
  group.create_dataset("l_fr_pos", data=l_fr_pos, dtype=float)
  group.create_dataset("l_fr_quat", data=l_fr_quat, dtype=float)
  group.create_dataset("l_hd_pos", data=l_hd_pos, dtype=float)
  group.create_dataset("l_hd_quat", data=l_hd_quat, dtype=float)

  group.create_dataset("r_up_pos", data=r_up_pos, dtype=float)
  group.create_dataset("r_up_quat", data=r_up_quat, dtype=float)
  group.create_dataset("r_fr_pos", data=r_fr_pos, dtype=float)
  group.create_dataset("r_fr_quat", data=r_fr_quat, dtype=float)
  group.create_dataset("r_hd_pos", data=r_hd_pos, dtype=float)
  group.create_dataset("r_hd_quat", data=r_hd_quat, dtype=float)

  group.create_dataset("time", data=time, dtype=float)

  f.close()



#def h5_to_wrist_elbow(in_h5_name, out_h5_name):




def read_video(video_name):

  video = cv2.VideoCapture(video_name+'.mp4')
  success, frame = video.read()  
  if not success:
    print("Failed to read the video!")
    return

  while success and cv2.waitKey(1) & 0xFF != ord('q'):
    cv2.imshow(video_name, frame)
    success, frame = video.read()

  cv2.destroyAllWindows()
  video.release()



if __name__ == '__main__':

  ### Set up parameters
  bag_name = 'second' # no `.bag` here
  h5_name = 'second'

  
  ### Export *** EVERYTHING *** from rosbag file into h5 file and a video!!
  bag_to_h5_video(bag_name, h5_name)


  ### Test the output
  video_name = bag_name
  read_video(video_name)
    










