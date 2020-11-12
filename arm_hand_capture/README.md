## README for collecting motion data:


### Dependencies

1. Camera related

```shell
sudo apt-get install ros-kinetic-libuvc*
```

2. VRPN related
```shell
sudo apt-get install ros-kinetic-vrpn*
```

3. ROS serial
```shell
sudo apt-get install ros-kinetic-rosserial*
```

4.


### Motion capture + dataglove collection system

Receive data from mocap system and dataglove on *Windows*, and transport them to *Ubuntu* for message synchronization and collection.

Windows: OptiTrack Motive (via VRPN) + Wiseglove SDK (via rosserial)

Ubuntu: Launch message syncing code.

### Commands to initiate the mocap+dataglove pipeline

1. Run the message syncing code on Ubuntu
```shell
roslaunch arm_hand_capture start_all_with_image.launch
```

2. Run the rosserial code for transporting Dataglove data on Windows

3. Use rosbag to record the data published on /dual_arms_dual_hands_state_with_image topic:

```shell
rosbag record <TOPIC-NAME> --duration=4 -O <BAG-NAME>
```

* prepend ```sleep 4 && ``` ahead of the above command to delay 4 seconds before recording if you are doing this all by yourself...

* ```<TOPIC-NAME>```: /dual_arms_dual_hands_state_with_image


### Dataglove calibration procedure
#### **Update at 2020-11-10: Record only aduction/adduction for four fingers, S14 for thumb and index-middle finger crossover, i.e. lr_all_close, lr_all_open, lr_thumb_s14_90 and lr_index_middle_crossover. That'll do for now.**

1. Push on mocap clothes, the do not hang it to hands before collecting dataglove calibration data.

2. Perform the calibration gestures given in the Wiseglove manual, the save the corresponding calibration texts.    
Although it doesn't affect the measured electrical signal, the linearly mapped results can provide us reference.

3. Record flexion/extension of the four fingers for both hands, 30 deg, 60 deg, 90 deg, and 0 deg (place hands on a flat plane and adduct all the fingers). Require **2 * 4 * 2 * 3 + 1 = 49** times in total.    
Naming convention: **{l/r}\_{index/middle/ring/little}\_{s3/s4/s6/s7/s9/s10/s12/s13}\_{90/120/150}** or **lr_all_close**.

4. Record abduction/adduction data for all the fingers of both hands. Since the above step includes an adducted gesture, all we need to do for this step is simply to record **1** piece of data by keeping all the fingers fully abducted.   
Naming convention: **lr_all_open**

5. Record data for newly installed bend sensor S14 which measures the rotation of thumb around the index finger. Since **lr_all_close** provides an initial state for sensor S14, all that's left to do is find a table or something that has two perpendicular planes and place your hands onto them, for the S14 angle to remain 90 deg.    
Naming convention: **lr_thumb_s14_90** (better do both hands together)

6. Record **1** piece of data *specifically* for the case of index-middle finger crossover, e.g. word 'is' in Chinese Sign Language.    
Naming convention: **lr_index_middle_crossover**.

7. Poke at the ground vertically or align both hands horizontally,to obtain the hand length of the demonstrator, i.e. distance from hand mocap marker to the handtip'.

8. (Optional) Precaution for possible requirement of *IK accurate* in the future, e.g. motions that require the tip of thumb to reach another fingertip.    
Naming convention: **lr_thumb_{index/middle/ring/little}_attached**.

9. (Optional) Some test cases for evaluating the effectiveness of dataglove calibration:

    (1) Move thumbs to reach the fingertips of the other fingers in turn. (*IK accurate*)   
    (2) Attach thumbs and other fingertips, and move around. (*IK accurate*, already recorded in Step 7)    
    (3) Bend all four fingers of both hands simultaneously. (*FK accurate*)   
    (4) Abduct and adduct all the fingers multiple times. (*FK accurate*)   
    (5) 'gun' motion in CSL. (*FK accurate*)    
    (6) Keep the index fingers of both hands aligned, and bend them simultaneously. (*FK accurate*, can be used to inspect the accuracy of S3 sensors of both hands.)   
    (7) Rotate thumbs around the index fingers. (*FK accurate*)   
    (8) Bend thumbs until they reach the palms. (*FK accurate*)   
    (9) Put thumbs in-between index and middle, middle and ring. (*FK accurate*)    
    (10) Thumbs up. (*FK accurate*)   
    (11) Hold a Fist. (*FK accurate*)   
    (12) Cross the middle over index finger. (*FK accurate*, a special case)

### Post-processing of the motion-captured data
1. Convert to h5:
```shell
rosrun arm_hand_capture extract_from_rosbag.py -i <BAG-NAME-WITHOUT-SUFFIX> -o <OUTPUT-H5-NAME-WITHOUT-SUFFIX> -l <HAND-LENGTH>
```

2. DMP learning (adjust the wrist trajectories and learn the weights).



### Post-processing of the dataglove calibration data

Use ```arm_hand_capture/script/extract_calibration_data.py``` to convert dataglove calibration data from rosbag files to h5 file.

A batch processing script ```arm_hand_capture/script/batch_process.py``` is provided.


### Some useful commands
1. See what resolutions and FPS are available for your camera.
```shell
v4l2-ctl --list-formats-ext
```
