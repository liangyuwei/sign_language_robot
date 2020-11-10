#!/usr/bin/env python
import rosbag
import pdb
import h5py
import numpy as np
import sys

import getopt

def bag_to_h5(bag_name, h5_name, group_name):
  ## This function converts the rosbag file to h5 containing all the glove calibration data.
  
  # Prep
  glove_elec = []
  bag_file = rosbag.Bag(bag_name + '.bag')
  count = bag_file.get_message_count()
  glove_elec = np.zeros([count, 30]) #14])
  glove_angle = np.zeros([count, 30]) #14])

  # Iterate to get the message contents
  idx = 0
  for topic, msg, t in bag_file.read_messages():
    ## h5 content
    # l
    glove_elec[idx, :15] = np.array(msg.glove_state.l_glove_elec)
    glove_angle[idx, :15] = np.array(msg.glove_state.left_glove_state)
    # r    
    glove_elec[idx, 15:] = np.array(msg.glove_state.r_glove_elec)
    glove_angle[idx, 15:] = np.array(msg.glove_state.right_glove_state)
    #import pdb
    #pdb.set_trace()
   
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
  # export information from bag file
  bag_name = None 
  h5_name = None 
  try:
    options, args = getopt.getopt(sys.argv[1:], "hi:o:", ["help", "bag-name=", "h5-name="])
  except getopt.GetoptError:
    sys.exit()
  for option, value in options:
    if option in ("-h", "--help"):
      print("Help:\n")
      print("   This script exports *EVERYTHING* from rosbag file into h5 file and a video.\n")
      print("Arguments:\n")
      print("   -i, --bag-name=, Specify the name of the input bag file, otherwise operate on the bag file with the default name specified inside this script. No suffix is required.\n")
      print("   -o, --h5-name=, Specify the name of the h5 output file, otherwise the default file name specified inside the script is used.\n")
      exit(0)
    if option in ("-i", "--bag-name"):
      print("Input bag file name: {0}\n".format(value))
      bag_name = value
    if option in ("-o", "--h5-name"):
      print("Output h5 file name: {0}\n".format(value))
      h5_name = value

  # check
  if bag_name is None or h5_name is None:
    print('Error: please fill in bag name or output h5 file name!')
    sys.exit()

  # extract necessary info for learning
  print("Input bag name is: " + bag_name + '.bag')
  print("Output h5 name is: " + h5_name + '.bag')

  group_name = bag_name


  ### Export *** ONLY *** glove angle data!!!
  bag_to_h5(bag_name, h5_name, group_name)
  print('Done!')

