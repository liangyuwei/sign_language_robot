#!/usr/bin/env python

import os
import sys
import signal


def quit(signum, frame):
  print ''
  print 'stop function'
  sys.exit()


if __name__ == '__main__':


  # prep
  workdir = '/sign_language_mocap_dataset/glove_calib_data/2020-11-02'
  h5_name = 'glove-calib-2020-11-02'
  h5_file = workdir + '/' + h5_name    

  # rule for consituting rosbag filenames
  prefix = ['l', 'r']
  directory = ['index', 'middle', 'ring']
  joint_id = [['s3', 's4'], ['s6', 's7'], ['s9', 's10']]
  suffix = ['90', '120', '150']

  # other bags that are not included by the above rule
  extra = ['fengren_new', 'gun_new', 'jidong_new', 'kaoqin_new', \
           'lr_close', 'lr_index_special', 'lr_open', 'lr_thumb_s14_0', \
           'lr_thumb_s14_90', 'test_finger_1', 'test_finger_2']

  # iterate to process bag files
  for d in range(3): # find directory
    # move to the directory
    cur_dir = workdir + '/' + directory[d]
    os.chdir(cur_dir)
    for j in joint_id[d]:
      for p in prefix:
        for s in suffix:
          # process bag file
          bag_file = p + '_' + directory[d] + '_' + j + '_' + s  
          command = 'rosrun arm_hand_capture extract_calibration_data.py -o ' + h5_file + ' -i ' + bag_file
          #import pdb
          #pdb.set_trace()
          os.system(command)

          # capture Ctrl+C
          signal.signal(signal.SIGINT, quit)
          signal.signal(signal.SIGTERM, quit)


  # process extra bags
  for f in extra:
    # move to the directory
    cur_dir = workdir
    os.chdir(cur_dir)

    # process
    command = 'rosrun arm_hand_capture extract_calibration_data.py -o ' + h5_file + ' -i ' + f
    os.system(command)

    # capture Ctrl+C
    signal.signal(signal.SIGINT, quit)
    signal.signal(signal.SIGTERM, quit)





