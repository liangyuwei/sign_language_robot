1.inspire_robot包支持因时机器人公司的灵巧手与机械夹爪在ROS平台上的使用，我们只在ros kinetic环境下进行了测试，其他ros环境需要等待我们后续的测试。
2.为了使程序能够正常运行，需要执行以下环境配置操作：（首次执行的需要，配置好了就不需要了）
    1）安装ros-kinetic环境，具体安装方式如下：
       （1）配置Ubuntu的资源库（系统设置->软件和更新）："restricted"，"universe"和"multiverse"。
       （2）设置Ubuntu的sources.list。终端指令：sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
       （3）设置key。终端指令：sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
       
       hkp://pgp.mit.edu:80     
       
      curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
       （4）安装。终端指令：sudo apt-get update
                           sudo apt-get install ros-kinetic-desktop-full  
       （5）初始化 rosdep。终端指令：sudo rosdep init
                                    rosdep update
       （6）配置环境。终端指令：echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
                              source ~/.bashrc
       （7）安装build依赖。终端指令：sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential


    2）创建catkin工作目录
       终端指令：mkdir -p ~/catkin_ws/src
                cd ~/catkin_ws/src
                catkin_init_workspace
                cd ~/catkin_ws/
                catkin_make
                source devel/setup.bash  (在每一个终端启动时使用，帮助你找到ROS安装目录)
    3）将inspire_robot.zip 放到catkin_ws目录下的/src文件夹下，解压
       终端指令：cd ~/catkin_ws/src
                 unzip inspire_robot
    4) 安装本安装包所需要的依赖
       终端指令：cd ~/catkin_ws
                rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
    5）对安装包进行重新编译
       终端指令：catkin_make
3.1 灵巧手使用
    1）将灵巧手与电脑连接，查看连接端口名称。
       终端指令：dmesg|grep tty*（端口名称应该会显示ttyUSB0或者是ttyUSB1、ttyUSB2等）
    2）对端口进行权限设置，可以选择临时方案或者是永久方案
        (1) 临时方案（每次使用都需要设置）。终端指令：sudo chmod a+rw /dev/ttyUSB*（* 是串口号）
        (2) 永久方案。终端指令：sudo vim -p /etc/udev/rules.d/70-ttyUSB.rules 在打开的文件中输入：KERNEL=="ttyUSB*", OWNER="root", GROUP="root", MODE="0666" 保存关闭之后，重启电脑即可。（注意KERNEL=="ttyUSB*",这里的*不是串口号就是*）
    3) 下面就是正式启动命令。使用launch命令来启动程序：
        (1) 如果串口名称是默认的ttyUSB0。终端指令：cd ~/catkin_ws
                                                 source devel/setup.bash
         
                                                 roslaunch inspire_hand hand_control.launch test_flag:=1
        (2) 如果串口名称非默认的ttyUSB0。终端指令：cd ~/catkin_ws
                                                 source devel/setup.bash
                                                 
                                                 roslaunch inspire_hand hand_control.launch port:=/dev/ttyUSB* test_flag:=1 （* 是串口号）

    4) 如果上一步启动成功，那么就可以使用后续的命令了：
       注意： 需要重新打开一个新的终端，并执行终端指令：cd ~/catkin_ws
                                                     source devel/setup.bash

         
      (1)rosservice call /inspire_hand/set_pos pos1 pos2 pos3 pos4 pos5 pos6 
      设置六个驱动器位置------参数pos范围0-2000 

      (2)rosservice call /inspire_hand/set_angle angle1 angle2 angle3 angle4 angle5 angle6 
      设置灵巧手角度------参数angle范围-1-1000

      (3)rosservice call /inspire_hand/set_force force1 force2 force3 force4 force5 force6 
      设置力控阈值------参数force范围0-1000

      (4)rosservice call /inspire_hand/set_speed speed1 speed2 speed3 speed4 speed5 speed6 
      设置速度------参数speed范围0-1000

      (5)rosservice call /inspire_hand/get_pos_act
      读取驱动器实际的位置值

      (6)rosservice call /inspire_hand/get_angle_act
      读取实际的角度值

      (7)rosservice call /inspire_hand/get_force_act
      读取实际的受力

      (8)rosservice call /inspire_hand/get_pos_set
      读取驱动器设置的位置值

      (9)rosservice call /inspire_hand/get_angle_set
      读取设置的角度值

      (10)rosservice call /inspire_hand/get_force_set
      读取设置的力控阈值
      
      (11)rosservice call /inspire_hand/get_error
      读取故障信息

      (12)rosservice call /inspire_hand/get_status
      读取状态信息

      (13)rosservice call /inspire_hand/get_temp
      读取温度信息

      (14)rosservice call /inspire_hand/get_current
      读取电流

      (15)rosservice call /inspire_hand/set_clear_error
      清除错误
     
      (16)rosservice call /inspire_hand/set_default_speed speed1 speed2 speed3 speed4 speed5 speed6
      设置上电速度------参数speedk范围0-1000

      (17)rosservice call /inspire_hand/set_default_force force1 force2 force3 force4 force5 force6
      设置上电力控阈值------参数forcek范围0-1000

      (18)rosservice call /inspire_hand/set_save_flash
      保存参数到FLASH

      (19)rosservice call /inspire_hand/set_force_clb
      校准力传感器

    5)可以采用下面指令实时监控灵巧手的实际角度和力
     需要重新打开一个新的终端，并执行终端指令：cd ~/catkin_ws
                                            source devel/setup.bash
                                            rosrun inspire_hand handcontroltopicpublisher

     需要重新打开一个新的终端，并执行终端指令：cd ~/catkin_ws
                                            source devel/setup.bash
                                            rosrun inspire_hand handcontroltopicsubscriber




     

