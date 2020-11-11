cmake_minimum_required(VERSION 2.8)
project(realtime_motion_retargeting)

set( CMAKE_BUILD_TYPE "Debug" )
set( CMAKE_CXX_FLAGS "-std=c++14 -O3" )

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
)

# Find Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# Find HDF5
find_package(HDF5 REQUIRED COMPONENTS C CXX HL) # this works...
include_directories(${HDF5_INCLUDE_DIRS})

add_message_files(
  FILES
  MotionMsg.msg
  ControlMsg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES realtime_motion_retargeting
 CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable( ControllerNode src/ControllerNode.cpp )
target_link_libraries( ControllerNode ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} )

add_executable( DataSamplerNode src/DataSamplerNode.cpp )
target_link_libraries( DataSamplerNode ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${HDF5_CXX_LIBRARIES})


