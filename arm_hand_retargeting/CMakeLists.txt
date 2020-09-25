cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(arm_hand_retargeting)

#SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_BUILD_TYPE "Debug")

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)#(-std=c++11)#


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages


# For LibTorch (PyTorch)
# set(CMAKE_PREFIX_PATH "/home/liangyuwei/libtorch")
# find_package(Torch REQUIRED)
# include_directories(
#   include ${catkin_INCLUDE_DIRS}${TORCH_INCLUDE_DIRS})#(include/)#


# Eigen 3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# Find HDF5
find_package(HDF5 REQUIRED COMPONENTS C CXX HL) # this works...
include_directories(${HDF5_INCLUDE_DIRS})

# For NLopt
find_package(NLopt REQUIRED)
if(NLopt_FOUND)
  message("NLopt lib is found: " ${NLOPT_LIBRARIES})
endif(NLopt_FOUND)


# Find G2O
list( APPEND CMAKE_MODULE_PATH /home/liangyuwei/g2o/cmake_modules )
set(G2O_DIR /home/liangyuwei/g2o)
find_package(G2O REQUIRED)
if(G2O_FOUND)
  include_directories(${G2O_INCLUDE_DIR})
  link_directories(${G2O_LIBRARY_DIRS})
  link_libraries(${G2O_LIBRARIES})
  message("G2O lib is found: " ${G2O_LIBRARIES})
  message("G2O dir is found: " ${G2O_INCLUDE_DIR})
endif(G2O_FOUND)

# G2O-related (cholmod, csparse)
include_directories(/usr/include/suitesparse)
include_directories(include())

list(APPEND G2O_LIBRARIES
  cholmod
  cxsparse
  g2o_core g2o_stuff g2o_csparse_extension g2o_incremental
  g2o_solver_csparse g2o_solver_pcg g2o_solver_cholmod
)

#link_directories(${G2O_LIBRARY_DIRS})
#message(">>>> G2O-libraries: " ${G2O_LIBRARIES})
#message(">>>> G2O-Inclu-dirs: " ${G2O_INCLUDE_DIRS})
# remember to sudo apt-get install ros-kinetic-libg2o	

## Specify additional locations for library files
#link_directories(
#  /usr/local/lib # Location when using 'make system_install'
#  /usr/lib       # More usual location (e.g. when installing using a package)
#)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  kdl_parser
  moveit_ros_planning_interface
  moveit_core
  moveit_ros_planning
  geometric_shapes
  # trac_ik_lib # Use TRAC-IK to compute
)

set(trac_ik_lib_DIR /home/liangyuwei/trac_ik/trac_ik_lib)
find_package(trac_ik_lib REQUIRED)
if(trac_ik_lib_FOUND)
  include_directories(${trac_ik_lib_INCLUDE_DIRS})
  link_directories(${trac_ik_lib_LIBRARY_DIRS})
  link_libraries(${trac_ik_lib_LIBRARIES})
  message("trac_ik_lib lib is found: " ${trac_ik_lib_LIBRARIES})
  message("trac_ik_lib dir is found: " ${trac_ik_lib_INCLUDE_DIRS})
endif(trac_ik_lib_FOUND)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS
    trac_ik_lib
#  INCLUDE_DIRS include
#  LIBRARIES arm_hand_retargeting
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)


### tool libraries
add_library(collision_checking_yumi src/collision_checking_yumi.cpp)
target_link_libraries(collision_checking_yumi ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# add_library(generate_trajectory_from_viewpoints src/tools/generate_trajectory_from_viewpoints.cpp)
# target_link_libraries(generate_trajectory_from_viewpoints ${EIGEN3_LIBRARIES} ${HDF5_CXX_LIBRARIES})

add_library(generate_trajectory_using_DMP src/generate_trajectory_using_DMP.cpp)
target_link_libraries(generate_trajectory_using_DMP ${EIGEN3_LIBRARIES} ${HDF5_CXX_LIBRARIES})

# add_library(similarity_network_pytorch STATIC src/tools/similarity_network_pytorch.cpp)
# target_link_libraries(similarity_network_pytorch PRIVATE ${TORCH_LIBRARIES} ${HDF5_CXX_LIBRARIES}) # link HDF5 together with TORCH fixes the linking error... i.e. undefined reference to symbol 'GOMP_parallel@@GOMP_4.0


### retargeting code for UR5 configuration
#add_executable(ur5_arm_retarget_nlopt src/ur5_arm_retarget_nlopt.cpp)
#target_link_libraries(ur5_arm_retarget_nlopt ${catkin_LIBRARIES} ${NLOPT_LIBRARIES} ${EIGEN3_LIBRARIES} ${HDF5_CXX_LIBRARIES})
#target_link_libraries(ur5_arm_retarget_nlopt distance_computation)
#target_link_libraries(ur5_arm_retarget_nlopt collision_checking)


### retargeting code for YuMi configuration
#add_executable(yumi_arm_retarget_nlopt src/yumi_arm_retarget_nlopt.cpp)
#target_link_libraries(yumi_arm_retarget_nlopt ${catkin_LIBRARIES} ${NLOPT_LIBRARIES} ${EIGEN3_LIBRARIES} ${HDF5_CXX_LIBRARIES})
#target_link_libraries(yumi_arm_retarget_nlopt distance_computation)
#target_link_libraries(yumi_arm_retarget_nlopt collision_checking_yumi)

#add_executable(yumi_arm_retarget_g2o src/yumi_arm_retarget_g2o.cpp)
#target_link_libraries(yumi_arm_retarget_g2o ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${HDF5_CXX_LIBRARIES})
#target_link_libraries(yumi_arm_retarget_g2o collision_checking_yumi)
#target_link_libraries(yumi_arm_retarget_g2o ${G2O_LIBRARIES} ) # g2o-related 


add_executable(yumi_arm_retarget_g2o_similarity src/yumi_arm_retarget_g2o_similarity.cpp)
target_link_libraries(yumi_arm_retarget_g2o_similarity ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${HDF5_CXX_LIBRARIES} ${G2O_LIBRARIES} collision_checking_yumi generate_trajectory_using_DMP) #generate_trajectory_from_viewpoints)
#target_link_libraries(yumi_arm_retarget_g2o_similarity collision_checking_yumi)
#target_link_libraries(yumi_arm_retarget_g2o_similarity ${G2O_LIBRARIES} ) # use g2o with pytorch similarity network
# target_link_libraries(yumi_arm_retarget_g2o_similarity similarity_network_pytorch)
#target_link_libraries(yumi_arm_retarget_g2o_similarity ${TORCH_LIBRARIES}) # libTorch
#set_property(TARGET yumi_arm_retarget_g2o_similarity PROPERTY CXX_STANDARD 14)#14)


### Optimization code for Hujin's method, for comparison with ours
add_executable(yumi_arm_retarget_hujin_nlopt src/yumi_arm_retarget_hujin_nlopt.cpp)
target_link_libraries(yumi_arm_retarget_hujin_nlopt ${catkin_LIBRARIES} ${NLOPT_LIBRARIES} ${EIGEN3_LIBRARIES} ${HDF5_CXX_LIBRARIES})
# target_link_libraries(yumi_arm_retarget_hujin_nlopt collision_checking_yumi)


### Test modules
#add_executable(TEST_collision_checking src/TEST_collision_checking.cpp)
#target_link_libraries(TEST_collision_checking ${catkin_LIBRARIES} collision_checking)

add_executable(TEST_collision_checking_yumi src/collision_checking_yumi.cpp)
target_link_libraries(TEST_collision_checking_yumi ${catkin_LIBRARIES} collision_checking_yumi ${HDF5_CXX_LIBRARIES})

#add_executable(TEST_generate_trajectory_from_viewpoints src/generate_trajectory_from_viewpoints.cpp)
#target_link_libraries(TEST_generate_trajectory_from_viewpoints ${EIGEN3_LIBRARIES} ${HDF5_CXX_LIBRARIES})

#add_executable(TEST_generate_trajectory_using_DMP src/generate_trajectory_using_DMP.cpp)
#target_link_libraries(TEST_generate_trajectory_using_DMP ${EIGEN3_LIBRARIES} ${HDF5_CXX_LIBRARIES})

#add_executable(distance_computation src/distance_computation.cpp)
#target_link_libraries(distance_computation ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#add_executable(TEST_similarity_network_pytorch src/TEST_similarity_network_pytorch.cpp)
#target_link_libraries(TEST_similarity_network_pytorch ${EIGEN3_LIBRARIES} similarity_network_pytorch)

# add_executable(TEST_KDL_FK_Solver src/KDL_FK_Solver.cpp)
# target_link_libraries(TEST_KDL_FK_Solver ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
