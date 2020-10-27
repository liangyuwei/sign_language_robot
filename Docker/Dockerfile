### Multi-stage build

# Using Nvidia Cuda+OpenGL(for RViz and Gazebo)
FROM nvidia/cudagl:10.2-devel-ubuntu16.04


# Meta data
ENV TZ=Asia/Shanghai
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute
LABEL MAINTAINER="Yuwei Liang lyw.liangyuwei@gmail.com"
LABEL Description="For sign language robot package."


# Build and Debug tools
RUN apt-get update && apt-get install -y \
    g++ \
    build-essential \
    cmake 
RUN apt-get update && apt-get install -y \
    gcc \
    gdb \
    gdbserver \
 && rm -rf /var/lib/apt/lists/*


# Install Ubuntu tools.
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone \
    && rm /etc/apt/sources.list.d/cuda.list \
    && rm /etc/apt/sources.list.d/nvidia-ml.list \
    && apt-get update \
    && apt-get install -y lsb-release psmisc tree vim net-tools iputils-ping wget git python3-pip libglm-dev 


# Install ROS.
SHELL ["/bin/bash", "-c"]
RUN sh -c '. /etc/lsb-release && echo "deb http://mirrors.sjtug.sjtu.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list' \
    && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    && apt-get update \
    && apt-get install -y ros-kinetic-desktop-full \
    && echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc \
    && source ~/.bashrc \
    && apt-get install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential \
    && rosdep init \
    && rosdep update


# Install ROS dependencies.
RUN pip3 install catkin_pkg transforms3d rospkg 
RUN apt-get install -y ros-kinetic-moveit \
                       ros-kinetic-ros-control \
                       ros-kinetic-ros-controllers 


# ROS Serial
RUN apt-get install -y ros-kinetic-serial

# NLopt (a bit slow to use github, so modify hosts)
#RUN wget https://github.com/stevengj/nlopt/archive/v2.6.2.tar.gz \
#    tar -xzf rebol.tar.gz 
RUN echo "151.101.84.133  raw.githubusercontent.com" >> /etc/hosts 
RUN git clone https://github.com/stevengj/nlopt \
 && cd nlopt \
 && mkdir build \
 && cd build \
 && cmake .. \
 && make \
 && make install


# Eigen (also a dependency for g2o)
RUN wget https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.tar.gz \
 && tar -xzvf eigen-3.3.7.tar.gz \
 && cd eigen-3.3.7 \
 && mkdir build \
 && cd build \
 && cmake .. \
 && make \
 && make install


# g2o
RUN apt-get install -y libsuitesparse-dev
RUN git clone https://github.com/RainerKuemmerle/g2o /usr/local/g2o \
 && cd /usr/local/g2o \
 && mkdir build \
 && cd build \
 && cmake .. \
 && make \
 && make install


# TRAC IK
RUN apt-get install -y ros-kinetic-trac-ik-kinematics-plugin ros-kinetic-trac-ik-lib 


# Gazebo ros control
RUN apt-get install -y ros-kinetic-gazebo-ros-control


# SSH server (set 'username:userpassword' with chpasswd)
RUN apt-get update && apt-get install -y openssh-server 

# change default listen port of 22 to other port number for security
ARG sshport=4399
RUN sed -i 's/Port 22/Port '${sshport}'/g' /etc/ssh/sshd_config
EXPOSE ${sshport}

# allow root login
RUN sed -i 's/PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config

# change password (must be!) after allowing root login
ARG userpasswd='123'
# RUN useradd -rm -d /home/${username} -s /bin/bash -g root -G sudo -u 1000 ${username}
RUN echo 'root:'${userpasswd} | chpasswd  
RUN service ssh start

# restart ssh service to update the settings
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["/usr/sbin/sshd", "-D"]


# Make Directory for storing code
RUN mkdir -p /root \
 && mkdir -p /root/vscode-workspace \
 && mkdir -p /root/vscode-workspace/sign_language_robot_ws

RUN apt-get install -y xarclock