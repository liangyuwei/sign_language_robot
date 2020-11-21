# sign_language_robot
This is the code for the paper [_"Dynamic Movement Primitive based Motion Retargeting for Dual-Arm Sign Language Motions"_](http://arxiv.org/abs/2011.03914) submitted to ICRA 2021.

# Structure
This repo contains code for recording and processing sign language demonstrations, motion retargeting, and other stuffs. Will be re-organized soon...

# About data collection
We use OptiTrack Motive and Wiseglove for recording human arm motions and finger movements. For further details, please refer to ```arm_hand_capture/README.md.```

# Dependencies
For data collection:
usb_cam  
vrpn_client_ros  
rosserial_server  

# FAQ
1. Error “sh: 1: v4l2-ctl: not found” requires the installation of v4l2, run ```sudo apt-get install v4l-utils```
