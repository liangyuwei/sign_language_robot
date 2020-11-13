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
  workdir = '/sign_language_mocap_dataset/glove_calib_data/2020-11-10'
  h5_name = 'glove-calib-2020-11-10'
  h5_file = workdir + '/' + h5_name    

  # rule for consituting rosbag filenames
  prefix = ['l', 'r']
  directory = ['index', 'middle', 'ring', 'little']
  joint_id = [['s3', 's4'], ['s6', 's7'], ['s9', 's10'], ['s12', 's13']]
  suffix = ['90', '120', '150']

  # other bags that are not included by the above rule
  extra = ['fengren_new', 'gun_new', 'jidong_new', 'kaoqin_new', 'kai_new', \
           'lr_all_close', 'lr_all_open', \
           'lr_index_middle_crossover', \
           'lr_thumb_index_attached', 'lr_thumb_middle_attached', 'lr_thumb_ring_attached', 'lr_thumb_little_attached', \
           'lr_index_special', \
           'lr_thumb_s14_90', \
           'test_fingers_abduct_adduct', \
           'test_fingers_bend_inturn', 'test_fingers_bend_simul', \
           'test_lr_index_bend_simul', \
           'test_fist', 'test_index_middle_crossover', \
           'test_thumb_inbetween_index_middle', 'test_thumb_inbetween_middle_ring', \
           'test_thumb_s14_bend_totally', \
           'test_thumb_s14_rotate', \
           'test_thumbs_reach_inturn', \
           'test_thumbs_up', \
           'lr_hand_align']

  # iterate to process bag files
  for d in range(4): # find directory
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





