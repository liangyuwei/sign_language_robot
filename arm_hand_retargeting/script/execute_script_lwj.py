import h5py
import sys
import os

in_h5_filename = "/home/liangyuwei/sign_language_robot_ws/test_imi_data/zhd/total_mocap_data_YuMi.h5"
out_h5_filename = "/home/liangyuwei/sign_language_robot_ws/test_imi_data/zhd/train_result.h5"
f = h5py.File(in_h5_filename,"r")
group_list = f.keys()
start = False
for group_name in group_list:
    if group_name == 'chuangkou':
        start = True
    if start == True:
        command = "rosrun arm_hand_retargeting yumi_arm_retarget_g2o_similarity -i "+in_h5_filename+\
            " -g "+group_name+" -o "+out_h5_filename
        os.system(command)