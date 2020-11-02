#!/usr/bin/env python
import rosbag
import pdb
import h5py
import numpy as np
import tf
import copy
import math

import sys
import getopt

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
  # returned quaternion is (x,y,z,w)
  return np.array([quat.x, quat.y, quat.z, quat.w])


def bag_to_h5_video(bag_name, h5_name, hand_length, fps=15.0):
  ## This function converts the rosbag file to h5 containing all the data(hand, forearm, upperarm) and the corresponding video.

  
  # Iterate to get the message contents
  bag_file = rosbag.Bag(bag_name + '.bag')
  count = bag_file.get_message_count()

  # set the actual fps 
  duration = bag_file.get_end_time() - bag_file.get_start_time() # for calculating FPS...
  fps = math.ceil(count / duration)
  print("FPS: {}".format(fps))

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

  l_glove_angle = np.zeros([count, 15]) #14])
  r_glove_angle = np.zeros([count, 15]) #14])

  # inferred hand tip by extending corresponding wrist positions along z-axis of local frame (which is specific to the hand marker setting for human motion capture)
  l_hand_tip_pos = np.zeros([count, 3])
  r_hand_tip_pos = np.zeros([count, 3])

  # used in DMP_learn.m to reconstruct robotic expected wrist positions
  hand_length_store = np.zeros([1, 1])
  hand_length_store[0, 0] = hand_length
  

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
    l_hd_quat[idx, :] = quat_to_ndarray(msg.left_hand_pose.pose.orientation)  # returned quaternion is (x, y, z, w) 
 
    r_up_pos[idx, :] = pos_to_ndarray(msg.right_upperarm_pose.pose.position)
    r_up_quat[idx, :] = quat_to_ndarray(msg.right_upperarm_pose.pose.orientation)
    r_fr_pos[idx, :] = pos_to_ndarray(msg.right_forearm_pose.pose.position)
    r_fr_quat[idx, :] = quat_to_ndarray(msg.right_forearm_pose.pose.orientation)
    r_hd_pos[idx, :] = pos_to_ndarray(msg.right_hand_pose.pose.position)
    r_hd_quat[idx, :] = quat_to_ndarray(msg.right_hand_pose.pose.orientation)  

    l_glove_angle[idx, :] = np.array(msg.glove_state.left_glove_state)
    r_glove_angle[idx, :] = np.array(msg.glove_state.right_glove_state)

    #import pdb
    #pdb.set_trace()

    ## infer human hand tip positions from captured wrist pos and ori data
    l_hand_quat = copy.deepcopy(l_hd_quat[idx, :])
    r_hand_quat = copy.deepcopy(r_hd_quat[idx, :])
    l_hand_rot = tf.transformations.quaternion_matrix(l_hand_quat)    
    r_hand_rot = tf.transformations.quaternion_matrix(r_hand_quat)    
    l_hand_tip_pos[idx, :] = l_hd_pos[idx, :] + np.dot(l_hand_rot[0:3, 0:3], np.array([0, 0, 1.0 * hand_length])) # move along z-axis of hand marker's local frame
    r_hand_tip_pos[idx, :] = r_hd_pos[idx, :] + np.dot(r_hand_rot[0:3, 0:3], np.array([0, 0, 1.0 * hand_length]))

    # import pdb
    # pdb.set_trace()

    ## video
    cv2_img = bridge.imgmsg_to_cv2(msg.image, 'bgr8') # shape is (480, 640, 3), type is ndarray, encoding is 'rgb8'
    (rows, cols, channels) = cv2_img.shape
    if idx == 0:
      #fps = 15.0
      size = (cols, rows)
      fourcc = cv2.VideoWriter_fourcc(*"MJPG")#(*"XVID")#
      video_writer = cv2.VideoWriter(bag_name+'.avi', fourcc, fps, size) 
    video_writer.write(cv2_img)

    ## Set counter
    idx = idx + 1

  video_writer.release()  

  ### Store the results
  # to construct a dataset for release, the data content must be complete!!!
  # l_hand_pos/l_hand_quat, l_forearm_pos/l_forearm_quat, l_upperarm_pos/l_upperarm_quat; after this, do an extraction to get wrist, elbow information for use in sign language robot!!!
  # store each part separately!!!  
  group_name = bag_name

  f = h5py.File(h5_name+".h5", "a") # open in append mode
  if group_name in f.keys():
    # the group already exists
    del f[group_name]
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

  group.create_dataset("l_glove_angle", data=l_glove_angle, dtype=float)
  group.create_dataset("r_glove_angle", data=r_glove_angle, dtype=float)

  group.create_dataset("l_hand_tip_pos", data=l_hand_tip_pos, dtype=float)
  group.create_dataset("r_hand_tip_pos", data=r_hand_tip_pos, dtype=float)
  group.create_dataset("demonstrator_hand_length", data=hand_length_store, dtype=float)

  group.create_dataset("time", data=time, dtype=float)

  f.close()


def h5_to_ur5_wrist_elbow(in_h5_name, out_h5_name, group_name):
  ### Extract needed information from the dataset(mocap) for learning 
  # The position data in the exported h5 is already under z-up frame in consistent with the UR5 world.
  # Yet the orientation data is still not under UR5 local frame, this function should transform the required orientation information from human motion data to match the UR5' local frames.
  

  ## Read needed data from mocap file
  f = h5py.File(in_h5_name+".h5", "r") 

  l_wrist_pos = f[group_name + '/l_hd_pos'][:]
  l_wrist_quat = f[group_name + '/l_hd_quat'][:] # quaternion is (x,y,z,w), refer to quat_to_ndarray()
  l_elbow_pos = f[group_name + '/l_fr_pos'][:]
  l_shoulder_pos = f[group_name + '/l_up_pos'][:]

  r_wrist_pos = f[group_name + '/r_hd_pos'][:]
  r_wrist_quat = f[group_name + '/r_hd_quat'][:]
  r_elbow_pos = f[group_name + '/r_fr_pos'][:]  
  r_shoulder_pos = f[group_name + '/r_up_pos'][:]

  l_glove_angle = f[group_name + '/l_glove_angle'][:]
  r_glove_angle = f[group_name + '/r_glove_angle'][:]

  l_hand_tip_pos = f[group_name + '/l_hand_tip_pos'][:]
  r_hand_tip_pos = f[group_name + '/r_hand_tip_pos'][:]
  hand_length = f[group_name + '/demonstrator_hand_length'][:]

  time = f[group_name + '/time'][:] # remember to store the timestamps information

  f.close()


  ## Transform the orientation to match UR5's local frames
  rotm_shift_l_hd = np.array([ [0.0, -1.0, 0.0, 0.0],
                               [0.0, 0.0, -1.0, 0.0],
                               [1.0, 0.0, 0.0, 0.0],
                               [0.0, 0.0, 0.0, 1.0] ])

  rotm_shift_r_hd = np.array([ [0.0, -1.0, 0.0, 0.0],
                               [0.0, 0.0, -1.0, 0.0],
                               [1.0, 0.0, 0.0, 0.0],
                               [0.0, 0.0, 0.0, 1.0] ])
 
  quat_shift_l_hd = tf.transformations.quaternion_from_matrix(rotm_shift_l_hd)
  quat_shift_r_hd = tf.transformations.quaternion_from_matrix(rotm_shift_r_hd)
  for i in range(l_wrist_quat.shape[0]):
    # refer to the following link for the usage of this function:
    # http://wiki.ros.org/tf2/Tutorials/Quaternions
    # tf quaternion is [x,y,z,w]
    l_wrist_quat[i, :] = tf.transformations.quaternion_multiply(l_wrist_quat[i, :], quat_shift_l_hd)
    r_wrist_quat[i, :] = tf.transformations.quaternion_multiply(r_wrist_quat[i, :], quat_shift_r_hd)


  ## Convert quaternions to rotation matrices
  length = l_wrist_pos.shape[0]
  l_wrist_ori = np.zeros([length, 9])
  r_wrist_ori = np.zeros([length, 9])
  for i in range(length):
    # tmp rotation matrix  
    l_wri_rotm = tf.transformations.quaternion_matrix([l_wrist_quat[i, 0],l_wrist_quat[i, 1], l_wrist_quat[i, 2], l_wrist_quat[i, 3]])
    r_wri_rotm = tf.transformations.quaternion_matrix([r_wrist_quat[i, 0],r_wrist_quat[i, 1], r_wrist_quat[i, 2], r_wrist_quat[i, 3]])    
    # assign
    l_wrist_ori[i, :] = l_wri_rotm[:3, :3].reshape(9)
    r_wrist_ori[i, :] = r_wri_rotm[:3, :3].reshape(9)


  ## Write to a new h5 file
  f = h5py.File(out_h5_name+'.h5', "a") # 'a' for read/write/create access
  if group_name in f.keys():
    # the group already exists
    del f[group_name]
  group = f.create_group(group_name)
  group.create_dataset("l_wrist_pos", data=l_wrist_pos, dtype=float)
  group.create_dataset("l_wrist_ori", data=l_wrist_ori, dtype=float)
  group.create_dataset("l_elbow_pos", data=l_elbow_pos, dtype=float)
  group.create_dataset("l_shoulder_pos", data=l_shoulder_pos, dtype=float)

  group.create_dataset("r_wrist_pos", data=r_wrist_pos, dtype=float)
  group.create_dataset("r_wrist_ori", data=r_wrist_ori, dtype=float)
  group.create_dataset("r_elbow_pos", data=r_elbow_pos, dtype=float)
  group.create_dataset("r_shoulder_pos", data=r_shoulder_pos, dtype=float)

  group.create_dataset("l_glove_angle", data=l_glove_angle, dtype=float)
  group.create_dataset("r_glove_angle", data=r_glove_angle, dtype=float)

  group.create_dataset("l_hand_tip_pos", data=l_hand_tip_pos, dtype=float)
  group.create_dataset("r_hand_tip_pos", data=r_hand_tip_pos, dtype=float)
  group.create_dataset("demonstrator_hand_length", data=hand_length, dtype=float)

  group.create_dataset("time", data=time, dtype=float)

  f.close()


def h5_to_yumi_wrist_elbow(in_h5_name, out_h5_name, group_name):
  ### Extract needed information from the dataset(mocap) for learning 
  # The position data in the exported h5 is already under z-up frame in consistent with the UR5 world.
  # Yet the orientation data is still not under YuMi manipulator's local frame, this function should transform the required orientation information from human motion data to match the YuMi's local frames.
  

  ## Read needed data from mocap file
  f = h5py.File(in_h5_name+".h5", "r") 

  l_wrist_pos = f[group_name + '/l_hd_pos'][:]
  l_wrist_quat = f[group_name + '/l_hd_quat'][:] # quaternion is (x,y,z,w), refer to quat_to_ndarray()
  l_elbow_pos = f[group_name + '/l_fr_pos'][:]
  l_shoulder_pos = f[group_name + '/l_up_pos'][:]

  r_wrist_pos = f[group_name + '/r_hd_pos'][:]
  r_wrist_quat = f[group_name + '/r_hd_quat'][:]
  r_elbow_pos = f[group_name + '/r_fr_pos'][:]  
  r_shoulder_pos = f[group_name + '/r_up_pos'][:]

  l_glove_angle = f[group_name + '/l_glove_angle'][:]
  r_glove_angle = f[group_name + '/r_glove_angle'][:]

  hand_length = f[group_name + '/demonstrator_hand_length'][:]

  l_hand_tip_pos = f[group_name + '/l_hand_tip_pos'][:]
  r_hand_tip_pos = f[group_name + '/r_hand_tip_pos'][:]

  time = f[group_name + '/time'][:] # remember to store the timestamps information

  f.close()


  ## Transform the orientation to match UR5's local frames
  rotm_shift_l_hd = np.array([ [0.0, -1.0, 0.0, 0.0],
                               [1.0, 0.0, 0.0, 0.0],
                               [0.0, 0.0, 1.0, 0.0],
                               [0.0, 0.0, 0.0, 1.0] ])

  rotm_shift_r_hd = np.array([ [0.0, -1.0, 0.0, 0.0],
                               [1.0, 0.0, 0.0, 0.0],
                               [0.0, 0.0, 1.0, 0.0],
                               [0.0, 0.0, 0.0, 1.0] ])
 
  quat_shift_l_hd = tf.transformations.quaternion_from_matrix(rotm_shift_l_hd)
  quat_shift_r_hd = tf.transformations.quaternion_from_matrix(rotm_shift_r_hd)
  for i in range(l_wrist_quat.shape[0]):
    # refer to the following link for the usage of this function:
    # http://wiki.ros.org/tf2/Tutorials/Quaternions
    # tf quaternion is [x,y,z,w]
    l_wrist_quat[i, :] = tf.transformations.quaternion_multiply(l_wrist_quat[i, :], quat_shift_l_hd)
    r_wrist_quat[i, :] = tf.transformations.quaternion_multiply(r_wrist_quat[i, :], quat_shift_r_hd)


  ## Convert quaternions to rotation matrices
  length = l_wrist_pos.shape[0]
  l_wrist_ori = np.zeros([length, 9])
  r_wrist_ori = np.zeros([length, 9])
  for i in range(length):
    # tmp rotation matrix  
    l_wri_rotm = tf.transformations.quaternion_matrix([l_wrist_quat[i, 0],l_wrist_quat[i, 1], l_wrist_quat[i, 2], l_wrist_quat[i, 3]])
    r_wri_rotm = tf.transformations.quaternion_matrix([r_wrist_quat[i, 0],r_wrist_quat[i, 1], r_wrist_quat[i, 2], r_wrist_quat[i, 3]])    
    # assign
    l_wrist_ori[i, :] = l_wri_rotm[:3, :3].reshape(9)
    r_wrist_ori[i, :] = r_wri_rotm[:3, :3].reshape(9)


  ## Write to a new h5 file
  f = h5py.File(out_h5_name+'.h5', "a") # 'a' for read/write/create access
  if group_name in f.keys():
    # the group already exists
    del f[group_name]
  group = f.create_group(group_name)
  group.create_dataset("l_wrist_pos", data=l_wrist_pos, dtype=float)
  group.create_dataset("l_wrist_ori", data=l_wrist_ori, dtype=float)
  group.create_dataset("l_elbow_pos", data=l_elbow_pos, dtype=float)
  group.create_dataset("l_shoulder_pos", data=l_shoulder_pos, dtype=float)

  group.create_dataset("r_wrist_pos", data=r_wrist_pos, dtype=float)
  group.create_dataset("r_wrist_ori", data=r_wrist_ori, dtype=float)
  group.create_dataset("r_elbow_pos", data=r_elbow_pos, dtype=float)
  group.create_dataset("r_shoulder_pos", data=r_shoulder_pos, dtype=float)

  group.create_dataset("l_glove_angle", data=l_glove_angle, dtype=float)
  group.create_dataset("r_glove_angle", data=r_glove_angle, dtype=float)

  group.create_dataset("l_hand_tip_pos", data=l_hand_tip_pos, dtype=float)
  group.create_dataset("r_hand_tip_pos", data=r_hand_tip_pos, dtype=float)
  group.create_dataset("demonstrator_hand_length", data=hand_length, dtype=float)

  group.create_dataset("time", data=time, dtype=float)

  f.close()


def read_video(video_name):

  video = cv2.VideoCapture(video_name+'.avi')
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
  # export information from bag file
  bag_name = 'qie' # no `.bag` here
  h5_name = 'glove_test_data'
  demonstrator_hand_length = 0.145 # data obtained by motion capture # actually, should be the distance from hand marker's origin to the tip of hand
  try:
    options, args = getopt.getopt(sys.argv[1:], "hi:o:l:", ["help", "bag-name=", "h5-name=", "length-of-hand="])
  except getopt.GetoptError:
    sys.exit()
  for option, value in options:
    if option in ("-h", "--help"):
      print("Help:\n")
      print("   This script exports *EVERYTHING* from rosbag file into h5 file and a video.\n")
      print("Arguments:\n")
      print("   -i, --bag-name=, Specify the name of the input bag file, otherwise operate on the bag file with the default name specified inside this script. No suffix is required.\n")
      print("   -o, --h5-name=, Specify the name of the h5 output file, otherwise the default file name specified inside the script is used.\n")
      print("   -l, --length-of-hand=, Specify the distance from the origin of hand rigid body marker to the tip of middle finger. Default length is my setup - LYW.\n")
      exit(0)
    if option in ("-i", "--bag-name"):
      print("Input bag file name: {0}\n".format(value))
      bag_name = value
    if option in ("-o", "--h5-name"):
      print("Output h5 file name: {0}\n".format(value))
      h5_name = value
    if option in ("-l", "--length-of-hand"):
      print("Demonstrator's hand length is: {0} m\n".format(value))
      demonstrator_hand_length = float(value) # convert to float type!!
  # export video 
  video_name = bag_name
  # extract necessary info for learning
  in_h5_name = h5_name
  out_h5_name = in_h5_name + '_UR5'
  group_name = bag_name

  
  ### Export *** EVERYTHING *** from rosbag file into h5 file and a video!!
  bag_to_h5_video(bag_name, h5_name, demonstrator_hand_length)


  ### Test the output
  read_video(video_name)
    

  ### Extract necessary information for learning (UR5 configuration)
  #h5_to_ur5_wrist_elbow(in_h5_name, out_h5_name, group_name)


  ### Extract necessary information for learning (YuMi configuration)
  out_yumi_h5_name = in_h5_name + '_YuMi'
  h5_to_yumi_wrist_elbow(in_h5_name, out_yumi_h5_name, group_name)







