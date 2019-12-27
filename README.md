# sign_language_robot
This is a dual-arm robot with two anthropomorphic hands designed specifically for sign-language application.
This repo is manually "forked" from my original "dual_ur5_arm" repo.



# Dependencies
0. 两个submodules，usb_cam和vrpn_client_ros

1. 如果出现报错“sh: 1: v4l2-ctl: not found”则需要安装v4l2 运行: sudo apt-get install v4l-utils

#2. For submodule libuvc_camera:
#sudo apt-get install ros-kinetic-libuvc

#3. 在使用libuvc_camera之前，需要配置一下permissions，具体的根据ROS Wiki的步骤进行设置：
#http://wiki.ros.org/libuvc_camera

#4. 在roslaunch使用libuvc_camera之前，需要根据所采用的camera把相关param参数都设置好。其中最重要的是：resolutions和frame rates。
#可以查看某个摄像头的supported resolutions and frame rates的命令：v4l2-ctl --list-formats-ext
