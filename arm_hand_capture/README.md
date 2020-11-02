README for collecting motion data:



0. roslaunch arm_hand_capture start_all_with_image.launch


1. (sleep 4 &&) rosbag record <TOPIC-NAME> --duration=4 -O <BAG-NAME>

<TOPIC-NAME>: /dual_arms_dual_hands_state_with_image

(sleep for 4 seconds before collecting...)



2. convert to h5:

rosrun arm_hand_capture extract_from_rosbag.py -i <BAG-NAME-WITHOUT-SUFFIX> -o <OUTPUT-H5-NAME-WITHOUT-SUFFIX> -l <HAND-LENGTH>




Calibration:

remember to record hand length!!!! poke at the ground horizontally...














