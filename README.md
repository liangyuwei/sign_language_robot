## Motion Retargeting for Dual-Arm Motions

[![arXiv](https://img.shields.io/badge/arxiv-2011.03914-B31B1B.svg)](http://arxiv.org/abs/2011.03914)[![Youtube Video](https://img.shields.io/badge/video-ICRA2021-blue.svg)](https://www.youtube.com/watch?v=jPvrAsN1Iwk&t=7s)[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

This is the code for the paper [_"Dynamic Movement Primitive based Motion Retargeting for Dual-Arm Sign Language Motions"_](http://arxiv.org/abs/2011.03914) accepted by ICRA 2021.

Bilibili Video: [_Click Here_](https://www.bilibili.com/video/BV12i4y1K76G/)



### Contents

* `Docker` Docker files for setting up the environment.
* `arm_hand_capture` Launch files and scripts for recording and post-processing motion capture data of arms and fingers. 

* `arm_hand_retargeting`Retargeting code, along with tools including collision checking, distance computation, etc.

* `dataglove_calib_visual` Visualize recorded hand movements on Inspire Hand or Shadow Hand models.
* `doc` Doxygen stuff.
* `dual_ur5_control`, `dual_ur5_moveit_config` Dual UR5 robot model and MoveIt configuration package.
* `dynamic_reconfigure` Dynamic reconfigure.
* `human_model*` Human model for visualizing recorded movements.
* `inspire_hand_*` Inspire Hands' models, drivers and MoveIt configuration.
* `raw_totg` ROS Wrapper of Time-Optimal Path Parameterization code.
* `realtime_motion_retargeting`
* `sign_language_robot_*` MoveIt configuration and scripts for controlling the sign language robot (Dual UR5 plus Inspire Robot Hand \* 2). 
* `universal_robot`, `ur5_moveit_config`, `ur_*` UR robot stuff.
* `yumi`, `yumi_control`: ABB YuMi stuff.
* `yumi_description`, `yumi_sign_language_robot_*`: YuMi robot model (with Inspire Hands), MoveIt configurations and scripts for controlling it.



### Data Preparation
We use OptiTrack Motive and Wiseglove for recording human arm motions and finger movements. For further details, please refer to ```arm_hand_capture/README.md.```



### Dependencies
For data collection:
usb_cam  
vrpn_client_ros  
rosserial_server  



### Code Procedure

main file: `arm_hand_retargeting/src/yumi_arm_retarget_g2o_similarity.cpp`

```mermaid
flowchart TD
A[Start] --> B[Load parameters from commandline]
B --> C[Set up variables' bounds]
C --> D["KDL::ChainFkSolverPos_recursive() \n Set up FK solver"]
D --> E["g2o::SparseOptimizer() \n Set up graph optimizer"]
E --> F["DualArmDualHandCollision() \n Set up collision checker for collision checking and distance computation"]
F --> G["DMPTrajectoryGenerator() \n Set up DMP trajectory generator"]
G --> H["g2o::SparseOptimizer::initializeOptimization() \n Construct graph optimization problems with vertices and edges"]
H --> I["Fill up optimization details"]
I --> J["Start optimization"]
J --> K["NullSpaceControl::solve_joint_trajectories() \n Stage 1: Use null-space control for initial guess"]
K --> L["g2o::SparseOptimizer::optimize() \n Stage 1: Loop to optimize joints with DMP starts and goals fixed"]
L --> LL["Check stopping criteria"]
LL --> |Good enough|Q
LL --> |Not good enough| O[Stage 2: Manually adjust DMPs' starts and goals]
O --> P["g2o::SparseOptimizer::optimize() \n Stage 3: Loop to optimize DMP starts and goals with joint trajs fixed"]
P --> L
P --> Q[Store the results and history statistics]
Q --> R[End]
```



### Refactor

* **Goals**

  * Modular

    Load different retargeting or imitation learning methods to obtain results.

  * Multi-modal

    Not restricted to joint trajectories or Cartesian trajectories.

  * More input way

    Easy to load human motion data, either from files or ROS interfaces.

  * ROS independent

    Separate the core from ROS, especially from specific collision checking implementation, and add support for ROS1/ROS2.

* **Framework**

*TODO*

* **UML design**

*TODO*



### Citation

If you find the paper helpful or use our source code, please cite the following:

```latex
@inproceedings{lyw2021dmp,
  title={Dynamic Movement Primitive Based Motion Retargeting for Dual-Arm Sign Language Motions},
  author={Liang, Yuwei and Li, Weijie and Wang, Yue and Xiong, Rong and Mao, Yichao and Zhang, Jiafan},
  booktitle={2021 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={8195--8201},
  year={2021},
  organization={IEEE}
}
```



### FAQ
1. Error “sh: 1: v4l2-ctl: not found” requires the installation of v4l2, run ```sudo apt-get install v4l-utils```
