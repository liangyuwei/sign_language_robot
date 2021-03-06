cmake_minimum_required(VERSION 2.8.3)
project(arm_hand_capture)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS
  genmsg
  roscpp
  rospy
  tf
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  visualization_msgs
  message_generation
  message_filters
)

# Find HDF5
find_package(HDF5 REQUIRED COMPONENTS C CXX HL) # this works...
include_directories(${HDF5_INCLUDE_DIRS})

add_message_files(
  FILES
  GloveState.msg
  DualArmDualHandState.msg
  DualArmDualHandStateWithImage.msg # good for visualization, should always use this to get a well-formatted dataset
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  trajectory_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})






## For sync-ing arm and hand states and re-publish them
add_executable(sync_message_topic src/sync_message_topic.cpp)
# make sure messages are generated before compiling nodes
target_link_libraries(sync_message_topic ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
add_dependencies(sync_message_topic ${PROJECT_NAME}_generate_messages_cpp)




## Testing of message_filters using ordinary message types
#add_executable(topic_to_sync_1 src/topic_to_sync_1.cpp)
#target_link_libraries(topic_to_sync_1 ${catkin_LIBRARIES})
#add_executable(topic_to_sync_2 src/topic_to_sync_2.cpp)
#target_link_libraries(topic_to_sync_2 ${catkin_LIBRARIES})
#add_executable(topic_to_sync_3 src/topic_to_sync_3.cpp)
#target_link_libraries(topic_to_sync_3 ${catkin_LIBRARIES})

#add_executable(test_message_filter src/test_message_filter.cpp)
#target_link_libraries(test_message_filter ${catkin_LIBRARIES})

#add_executable(test_message_filter_class src/test_message_filter_class.cpp)
#target_link_libraries(test_message_filter_class ${catkin_LIBRARIES})


## Unpack the sync-ed message and broadcast the transforms of each arm
add_executable(unpack_synced_message src/unpack_synced_message.cpp)
target_link_libraries(unpack_synced_message ${catkin_LIBRARIES})
add_dependencies(unpack_synced_message ${PROJECT_NAME}_generate_messages_cpp)

## Transform the local frames(upperarm, forearm, hand) to match UR5 configuration
# for real-time synchronization experiment in the future
add_executable(synced_message_to_ur5 src/synced_message_to_ur5.cpp)
target_link_libraries(synced_message_to_ur5 ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
add_dependencies(synced_message_to_ur5 ${PROJECT_NAME}_generate_messages_cpp)

## Transform the local frames(upperarm, forearm, hand) to match UR5 configuration
# for real-time synchronization experiment in the future
add_executable(synced_message_to_yumi src/synced_message_to_yumi.cpp)
target_link_libraries(synced_message_to_yumi ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
add_dependencies(synced_message_to_yumi ${PROJECT_NAME}_generate_messages_cpp)


## For visualization markers
add_executable(load_visualization_markers src/load_visualization_markers.cpp)
target_link_libraries(load_visualization_markers ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
add_dependencies(load_visualization_markers ${PROJECT_NAME}_generate_messages_cpp)




## Visualization directly using h5 file (used during data collection)
add_executable(visualize_recorded_movements src/visualize_recorded_movements.cpp)
target_link_libraries(visualize_recorded_movements ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${HDF5_CXX_LIBRARIES})
add_dependencies(visualize_recorded_movements ${PROJECT_NAME}_generate_messages_cpp)






## Sync the arm-hand state with the corresponding image
add_executable(sync_state_with_image src/sync_state_with_image.cpp)
target_link_libraries(sync_state_with_image ${catkin_LIBRARIES})
add_dependencies(sync_state_with_image ${PROJECT_NAME}_generate_messages_cpp)

add_executable(unpack_synced_message_with_image src/unpack_synced_message_with_image.cpp)
target_link_libraries(unpack_synced_message_with_image ${catkin_LIBRARIES})
add_dependencies(unpack_synced_message_with_image ${PROJECT_NAME}_generate_messages_cpp)







## Sync the arm-hand state without image (for robot teleoperation)
add_executable(sync_state_without_image src/sync_state_without_image.cpp)
target_link_libraries(sync_state_without_image ${catkin_LIBRARIES})
add_dependencies(sync_state_without_image ${PROJECT_NAME}_generate_messages_cpp)

add_executable(unpack_synced_message_without_image src/unpack_synced_message_without_image.cpp)
target_link_libraries(unpack_synced_message_without_image ${catkin_LIBRARIES})
add_dependencies(unpack_synced_message_without_image ${PROJECT_NAME}_generate_messages_cpp)

add_executable(load_visualization_markers_without_image src/load_visualization_markers_without_image.cpp)
target_link_libraries(load_visualization_markers_without_image ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
add_dependencies(load_visualization_markers_without_image ${PROJECT_NAME}_generate_messages_cpp)




## For displaying original and adjusted movements (hand tip as the base, and adjust according to human hand length and robot hand length)
add_executable(unpack_original_movement_message src/unpack_original_movement_message.cpp)
target_link_libraries(unpack_original_movement_message ${catkin_LIBRARIES})
add_dependencies(unpack_original_movement_message ${PROJECT_NAME}_generate_messages_cpp)

add_executable(unpack_adjusted_movement_message src/unpack_adjusted_movement_message.cpp)
target_link_libraries(unpack_adjusted_movement_message ${catkin_LIBRARIES})
add_dependencies(unpack_adjusted_movement_message ${PROJECT_NAME}_generate_messages_cpp)

add_executable(load_original_visualization_markers src/load_original_visualization_markers.cpp)
target_link_libraries(load_original_visualization_markers ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
add_dependencies(load_original_visualization_markers ${PROJECT_NAME}_generate_messages_cpp)

add_executable(load_adjusted_visualization_markers src/load_adjusted_visualization_markers.cpp)
target_link_libraries(load_adjusted_visualization_markers ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
add_dependencies(load_adjusted_visualization_markers ${PROJECT_NAME}_generate_messages_cpp)