#!/bin/bash
# launch all the existing hand models in RVIZ for visual test. use CTRL-C to kill current test and enter to launch next one
echo press enter after you tested and killed the launch file

read -p "Press [Enter] key "
SIMULATED=0  MUSCLE=0  BIOTAC_HAND=0  ELLIPSOID=0  ONE_FINGER=0  THREE_FINGER=0  LEFT_HAND=0   roslaunch sr_description test_hand_models.launch   #shadowhand_motor.urdf
read -p "Press [Enter] key "
SIMULATED=0  MUSCLE=0  BIOTAC_HAND=0  ELLIPSOID=0  ONE_FINGER=0  THREE_FINGER=0  LEFT_HAND=1   roslaunch sr_description test_hand_models.launch   #shadowhand_left_motor.urdf

read -p "Press [Enter] key "
SIMULATED=0  MUSCLE=0  BIOTAC_HAND=0  ELLIPSOID=1  ONE_FINGER=0  THREE_FINGER=0  LEFT_HAND=0   roslaunch sr_description test_hand_models.launch   #shadowhand_motor_ellipsoid.urdf

read -p "Press [Enter] key "
SIMULATED=0  MUSCLE=0  BIOTAC_HAND=1  ELLIPSOID=0  ONE_FINGER=0  THREE_FINGER=0  LEFT_HAND=0   roslaunch sr_description test_hand_models.launch   #shadowhand_motor_biotac.urdf
read -p "Press [Enter] key "
SIMULATED=0  MUSCLE=0  BIOTAC_HAND=1  ELLIPSOID=0  ONE_FINGER=0  THREE_FINGER=0  LEFT_HAND=1   roslaunch sr_description test_hand_models.launch   #shadowhand_left_motor_biotac.urdf

read -p "Press [Enter] key "
SIMULATED=0  MUSCLE=0  BIOTAC_HAND=0  FF_BIOTAC=1  ELLIPSOID=0  ONE_FINGER=0  THREE_FINGER=0  LEFT_HAND=0   roslaunch sr_description test_hand_models.launch   #shadowhand_motor_ff_biotac.urdf
read -p "Press [Enter] key "
SIMULATED=0  MUSCLE=0  BIOTAC_HAND=0  THFFRF_ELLIPSOID=1  ELLIPSOID=0  ONE_FINGER=0  THREE_FINGER=0  LEFT_HAND=0   roslaunch sr_description test_hand_models.launch   #shadowhand_motor_th_ff_rf_ellipsoid.urdf

read -p "Press [Enter] key "
SIMULATED=0  MUSCLE=0  BIOTAC_HAND=0 BTSP_HAND=1  ELLIPSOID=0  ONE_FINGER=0  THREE_FINGER=0  LEFT_HAND=0   roslaunch sr_description test_hand_models.launch   #shadowhand_motor_btsp.urdf



read -p "Press [Enter] key "
SIMULATED=0  MUSCLE=0  BIOTAC_HAND=0  ELLIPSOID=0  ONE_FINGER=0  THREE_FINGER=1  LEFT_HAND=0   roslaunch sr_description test_hand_models.launch   #sr_three_finger_motor.urdf
read -p "Press [Enter] key "
SIMULATED=0  MUSCLE=0  BIOTAC_HAND=0  ELLIPSOID=0  ONE_FINGER=1  THREE_FINGER=0  LEFT_HAND=0   roslaunch sr_description test_hand_models.launch   #sr_one_finger_motor.urdf

read -p "Press [Enter] key "
SIMULATED=1  MUSCLE=1  BIOTAC_HAND=0  ELLIPSOID=0  ONE_FINGER=0  THREE_FINGER=0  LEFT_HAND=0   roslaunch sr_description test_hand_models.launch   #shadowhand_muscle.urdf
read -p "Press [Enter] key "
SIMULATED=0  MUSCLE=1  BIOTAC_HAND=0  ELLIPSOID=0  ONE_FINGER=0  THREE_FINGER=0  LEFT_HAND=0   roslaunch sr_description test_hand_models.launch   #shadowhand_edc_muscle.urdf
read -p "Press [Enter] key "
SIMULATED=1  MUSCLE=1  BIOTAC_HAND=0  ELLIPSOID=0  ONE_FINGER=0  THREE_FINGER=0  LEFT_HAND=1   roslaunch sr_description test_hand_models.launch   #shadowhand_left_muscle.urdf
read -p "Press [Enter] key "
SIMULATED=0  MUSCLE=1  BIOTAC_HAND=0  ELLIPSOID=0  ONE_FINGER=0  THREE_FINGER=0  LEFT_HAND=1   roslaunch sr_description test_hand_models.launch   #shadowhand_left_edc_muscle.urdf
read -p "Press [Enter] key "
SIMULATED=1  MUSCLE=1  BIOTAC_HAND=1  ELLIPSOID=0  ONE_FINGER=0  THREE_FINGER=0  LEFT_HAND=0   roslaunch sr_description test_hand_models.launch   #shadowhand_muscle_biotac.urdf
read -p "Press [Enter] key "
SIMULATED=0  MUSCLE=1  BIOTAC_HAND=1  ELLIPSOID=0  ONE_FINGER=0  THREE_FINGER=0  LEFT_HAND=0   roslaunch sr_description test_hand_models.launch   #shadowhand_edc_muscle_biotac.urdf
read -p "Press [Enter] key "
SIMULATED=0  MUSCLE=1  BIOTAC_HAND=1  ELLIPSOID=0  ONE_FINGER=0  THREE_FINGER=1  LEFT_HAND=0   roslaunch sr_description test_hand_models.launch   #sr_three_finger_edc_muscle_biotac.urdf
