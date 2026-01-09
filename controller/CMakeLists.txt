cmake_minimum_required(VERSION 3.0.2)
project(controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros
  roscpp
  std_msgs
  dynamic_reconfigure
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(Eigen3 REQUIRED)
#find_package(catkin_simple REQUIRED)

# generate_dynamic_reconfigure_options(
  # cfg/param.cfg
# )

# catkin_simple()
# cs_add_library(mpc_solver
  
#   # lib/acado_solver_mex.c
#   # lib/acado_solver_sfunction.c
# )

# target_include_directories(mpc_solver PUBLIC
#   # model/quadrotor_mpc_codegen/
#   include/controller

# )

# # make an executable
# cs_install()
# cs_export()

include_directories(${EIGEN3_INCLUDE_DIRS})
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
#   geometry_msgs#   std_msgs
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
  INCLUDE_DIRS include
  LIBRARIES controller
  CATKIN_DEPENDS geometry_msgs mavros roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  "/home/kd/ecos"
  "/home/kd/ecos/include"
  "/home/kd/ecos/external/amd/include"
  "/home/kd/ecos/external/ldl/include"
  "/home/kd/ecos/external/SuiteSparse_config"
)

link_directories(
  "/home/kd/ecos/build"
  # "/home/kd/ecos"
)

## Declare a C++ library
add_library(${PROJECT_NAME}
   src/param.cpp
  #  src/controller_star_ava_param.cpp
   src/linear_ava_plant.cpp
   src/uav_normal_function.cpp

  #  src/impedance_control_param.cpp
   #"/home/kd/ecos/src"
   #src/param.cpp
 )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(star_controller_node src/controller_star_ava.cpp src/controller_star_ava_param.cpp)
target_link_libraries(star_controller_node
  ${catkin_LIBRARIES}
  # libecos.so
  #src/param.cpp
)

add_executable(four_controller_node src/four_controller_node.cpp src/four_param.cpp)
target_link_libraries(four_controller_node
  ${catkin_LIBRARIES}
  # libecos.so
  #src/param.cpp
)

add_executable(${PROJECT_NAME}_node src/controller_node.cpp src/param.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  # libecos.so
  #src/param.cpp
)


#add_executable(impedance_control_node src/impedance_control_node.cpp src/impedance_control_param.cpp)
#target_link_libraries(impedance_control_node
 # ${catkin_LIBRARIES}
  # libecos.so
  #src/param.cpp
#)


add_executable(linear_ava_controller_node src/linear_ava_controller.cpp src/linear_ava_plant.cpp src/uav_normal_function.cpp)
target_link_libraries(linear_ava_controller_node
  ${catkin_LIBRARIES}
  #libecos.so
)

add_executable(two_linear_ava_controller_node src/two_subcraft_linear_controller.cpp src/two_subcraft_linear_plant.cpp src/uav_normal_function.cpp)
target_link_libraries(two_linear_ava_controller_node
  ${catkin_LIBRARIES}
  #libecos.so
)

add_executable(impedance_control_node src/imp_four_controller_node.cpp src/imp_four_param.cpp)
target_link_libraries(impedance_control_node
  ${catkin_LIBRARIES}
  # libecos.so
  #src/param.cpp
)
