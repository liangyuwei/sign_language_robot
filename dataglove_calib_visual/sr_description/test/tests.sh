#!/bin/bash
# launch all the existing arm and hand models in RVIZ for visual test. use CTRL-C to kill current test and enter to launch next one

echo press enter after you tested and killed the launch file
read -p "Press [Enter] key "
SIMULATED=0  MUSCLE=0  BIOTAC_HAND=0  ELLIPSOID=0  ONE_FINGER=0  THREE_FINGER=0  LEFT=0   roslaunch sr_description test_arm_and_hand_models.launch   #arm_and_hand_motor.urdf
read -p "Press [Enter] key "
SIMULATED=0  MUSCLE=0  BIOTAC_HAND=0  ELLIPSOID=1  ONE_FINGER=0  THREE_FINGER=0  LEFT=0   roslaunch sr_description test_arm_and_hand_models.launch   #arm_and_hand_motor_ellipsoid.urdf
read -p "Press [Enter] key "
SIMULATED=0  MUSCLE=0  BIOTAC_HAND=1  ELLIPSOID=0  ONE_FINGER=0  THREE_FINGER=0  LEFT=0   roslaunch sr_description test_arm_and_hand_models.launch   #arm_and_hand_motor_biotac.urdf
read -p "Press [Enter] key "
SIMULATED=0  MUSCLE=0  BIOTAC_HAND=0  ELLIPSOID=0  ONE_FINGER=0  THREE_FINGER=1  LEFT=0   roslaunch sr_description test_arm_and_hand_models.launch   #arm_and_hand_motor_three_finger.urdf
read -p "Press [Enter] key "
SIMULATED=0  MUSCLE=0  BIOTAC_HAND=0  ELLIPSOID=0  ONE_FINGER=1  THREE_FINGER=0  LEFT=0   roslaunch sr_description test_arm_and_hand_models.launch   #arm_and_sr_one_finger_motor.urdf
read -p "Press [Enter] key "
SIMULATED=1  MUSCLE=1  BIOTAC_HAND=0  ELLIPSOID=0  ONE_FINGER=0  THREE_FINGER=0  LEFT=0   roslaunch sr_description test_arm_and_hand_models.launch   #arm_and_hand_muscle.urdf
read -p "Press [Enter] key "
SIMULATED=1  MUSCLE=1  BIOTAC_HAND=1  ELLIPSOID=0  ONE_FINGER=0  THREE_FINGER=0  LEFT=0   roslaunch sr_description test_arm_and_hand_models.launch   #arm_and_hand_muscle_biotac.urdf
