cmake_minimum_required(VERSION 2.8.3)
project(arm_hand_capture)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  std_msgs
  geometry_msgs
  trajectory_msgs
  message_generation
  message_filters
)

add_message_files(
  FILES
  GloveState.msg
  DualArmDualHandState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  geometry_msgs
  trajectory_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})



## For sync-ing arm and hand states and re-publish them
add_executable(sync_message_topic src/sync_message_topic.cpp)
target_link_libraries(sync_message_topic ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})


## Testing of message_filters using ordinary message types
add_executable(topic_to_sync_1 src/topic_to_sync_1.cpp)
target_link_libraries(topic_to_sync_1 ${catkin_LIBRARIES})
add_executable(topic_to_sync_2 src/topic_to_sync_2.cpp)
target_link_libraries(topic_to_sync_2 ${catkin_LIBRARIES})
add_executable(topic_to_sync_3 src/topic_to_sync_3.cpp)
target_link_libraries(topic_to_sync_3 ${catkin_LIBRARIES})

add_executable(test_message_filter src/test_message_filter.cpp)
target_link_libraries(test_message_filter ${catkin_LIBRARIES})

add_executable(test_message_filter_class src/test_message_filter_class.cpp)
target_link_libraries(test_message_filter_class ${catkin_LIBRARIES})


## Unpack the sync-ed message and broadcast the transforms of each arm
add_executable(unpack_synced_message src/unpack_synced_message.cpp)
target_link_libraries(unpack_synced_message ${catkin_LIBRARIES})


## Transform the local frames(upperarm, forearm, hand) to match UR5 configuration
add_executable(synced_message_to_ur5 src/synced_message_to_ur5.cpp)
target_link_libraries(synced_message_to_ur5 ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})






