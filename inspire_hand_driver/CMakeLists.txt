cmake_minimum_required(VERSION 2.8.3)
project(inspire_hand)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  message_generation
  tf
  std_msgs
  genmsg
  )

include_directories(${catkin_INCLUDE_DIRS}
                    ${PROJECT_SOURCE_DIR}/include/)
                   
#SET(SOURCES ${PROJECT_SOURCE_DIR}/src/hand_control.cpp
            #${PROJECT_SOURCE_DIR}/src/hand_control_lib.cpp
            #${PROJECT_SOURCE_DIR}/src/hand_control_client.cpp
	    #${PROJECT_SOURCE_DIR}/src/hand_control_topic.cpp)
   
#SET(HEADERS ${PROJECT_SOURCE_DIR}/include/hand_control.h)


add_service_files(FILES set_id.srv
				set_redu_ratio.srv
				set_clear_error.srv
				set_save_flash.srv
				set_reset_para.srv
				set_force_clb.srv
				set_gesture_no.srv
				set_current_limit.srv
				set_default_speed.srv
				set_default_force.srv
				set_user_def_angle.srv
				set_pos.srv
				set_angle.srv
				set_force.srv
				set_speed.srv
				get_pos_act.srv
				get_angle_act.srv
				get_force_act.srv
				get_current.srv
				get_error.srv
				get_status.srv
				get_temp.srv
				get_pos_set.srv
				get_angle_set.srv
				get_force_set.srv)

generate_messages(DEPENDENCIES
    std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  DEPENDS roscpp serial tf
  )

add_executable(${PROJECT_NAME} src/hand_control.cpp src/hand_control_lib.cpp include/hand_control.h)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} inspire_hand_gencpp)
target_link_libraries(${PROJECT_NAME} ${ROS_LIBRARIES} ${catkin_LIBRARIES})

add_executable(hand_control_client src/hand_control_client.cpp)
target_link_libraries(hand_control_client ${ROS_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(hand_control_client inspire_hand_gencpp)

add_executable(handcontroltopicpublisher src/handcontroltopicpublisher.cpp)
target_link_libraries(handcontroltopicpublisher ${ROS_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(handcontroltopicpublisher inspire_hand_gencpp)

add_executable(handcontroltopicsubscriber src/handcontroltopicsubscriber.cpp)
target_link_libraries(handcontroltopicsubscriber ${ROS_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(handcontroltopicsubscriber inspire_hand_gencpp)

add_executable(handcontroltopicpublisher1 src/handcontroltopicpublisher1.cpp)
target_link_libraries(handcontroltopicpublisher1 ${ROS_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(handcontroltopicpublisher1 inspire_hand_gencpp)

add_executable(handcontroltopicsubscriber1 src/handcontroltopicsubscriber1.cpp)
target_link_libraries(handcontroltopicsubscriber1 ${ROS_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(handcontroltopicsubscriber1 inspire_hand_gencpp)
