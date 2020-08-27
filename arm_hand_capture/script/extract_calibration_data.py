#!/usr/bin/env python
import rosbag
import pdb
import h5py
import numpy as np
import sys

def bag_to_h5(bag_name, h5_name, group_name, left_or_right):
  ## This function converts the rosbag file to h5 containing all the glove calibration data.
  
  # Prep
  glove_elec = []
  bag_file = rosbag.Bag(bag_name + '.bag')
  count = bag_file.get_message_count()
  glove_elec = np.zeros([count, 14])
  glove_angle = np.zeros([count, 14])

  # Iterate to get the message contents
  idx = 0
  for topic, msg, t in bag_file.read_messages():
    ## h5 content
    if left_or_right:
      glove_elec[idx, :] = np.array(msg.glove_state.l_glove_elec)
      glove_angle[idx, :] = np.array(msg.glove_state.left_glove_state)
    else:
      glove_elec[idx, :] = np.array(msg.glove_state.r_glove_elec)
      glove_angle[idx, :] = np.array(msg.glove_state.right_glove_state)
    ## Set counter
    idx = idx + 1

  ### Store the results
  f = h5py.File(h5_name+".h5", "a") # open in append mode
  if group_name in f.keys():
    # the group already exists, delete it
    del f[group_name]
  group = f.create_group(group_name)
  group.create_dataset('glove_elec', data=glove_elec, dtype=int)
  group.create_dataset('glove_angle', data=glove_angle, dtype=float)
  f.close()


if __name__ == '__main__':

  ### Set up parameters
  # left or right
  lr = 'r' # 'r'

  # find finger-related info from bag name
  finger_name = 'thumb' #['thumb', 'index', 'middle', 'ring', 'little']
  
  # ID for the calibrated joint, according to WiseGlove setup
  id = 2
  
  # Actual angle (ground truth), in degree
  angle = 90
  
  # export information from bag file
  group_name = lr + '_' + finger_name + '_s' + str(id) + '_' + str(angle)
  bag_name = group_name # 'test_seq_' + #'test_seq_l_thumb_s1_90' # no suffix is required
  h5_name = 'glove_calib_data-20200827' #'glove_calib_data-20200825'

  # extract necessary info for learning
  group_name = bag_name

  print("Input bag name is: " + bag_name + '.bag')
  print("Output h5 name is: " + h5_name + '.bag')


  ### Export *** EVERYTHING *** from rosbag file into h5 file and a video!!
  bag_to_h5(bag_name, h5_name, group_name, lr == 'l')
  print('Done!')

