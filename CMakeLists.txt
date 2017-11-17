cmake_minimum_required(VERSION 2.8.3)
project(puma260_driver)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  tf
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   XYZOAT.msg
   JointAngles.msg
 )


generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
#  CATKIN_DEPENDS roscpp
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
)

## Declare a C++ executable
 add_executable(puma_driver_node src/puma_driver_node.cpp src/Puma_OP.cpp src/serial_talk.c)

## Specify libraries to link a library or executable target against
 target_link_libraries(puma_driver_node
   ${catkin_LIBRARIES}
 )

