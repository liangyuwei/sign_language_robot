cmake_minimum_required(VERSION 2.8.3)
project(sr_description)
find_package(catkin REQUIRED COMPONENTS urdf xacro)
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

catkin_package(
    DEPENDS
    CATKIN_DEPENDS urdf xacro position_controllers joint_trajectory_controller
    INCLUDE_DIRS
    LIBRARIES
)

if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_sr_description_urdf test/test_sr_description_urdf.cpp)
  add_dependencies(test_sr_description_urdf ${catkin_EXPORTED_TARGETS})
  target_link_libraries(test_sr_description_urdf ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${GTEST_LIBRARIES} pthread)
endif()

install(DIRECTORY arm/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/arm)

install(DIRECTORY hand/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/hand)

install(DIRECTORY loaders/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/loaders)

install(DIRECTORY other/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/other)

install(DIRECTORY robots/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/robots)

install(FILES materials.urdf.xacro
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
