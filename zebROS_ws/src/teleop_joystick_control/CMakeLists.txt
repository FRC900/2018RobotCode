cmake_minimum_required(VERSION 2.8.3)
project(teleop_joystick_control)

include ("../cmake_modules/CMakeOpt.cmake")

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  hardware_interface
  controller_manager
  roscpp
  control_msgs
  trajectory_msgs
  actionlib
  urdf
  joint_limits_interface
  transmission_interface
  control_toolbox
  std_msgs
  sensor_msgs
  rosparam_shortcuts
  talon_interface
  message_generation
  ros_control_boilerplate
)

add_message_files (
  FILES
  RobotState.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)
find_package(Gflags REQUIRED)
find_package(Eigen3 REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)



catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    hardware_interface
    controller_manager
    roscpp
    control_msgs
    trajectory_msgs
    urdf
    joint_limits_interface
    transmission_interface
    control_toolbox
    std_msgs
    sensor_msgs
    rosparam_shortcuts
    message_runtime
	ros_control_boilerplate
  LIBRARIES
)

###########
## Build ##
###########

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
  ${Gflags_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_joystickSubscriber src/joystickSubscriber.cpp)
set_target_properties(${PROJECT_NAME}_joystickSubscriber PROPERTIES OUTPUT_NAME
joystickSubscriber PREFIX "")
target_link_libraries(${PROJECT_NAME}_joystickSubscriber
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_teleopJoystickCommands src/teleopJoystickCommands.cpp)
set_target_properties(${PROJECT_NAME}_teleopJoystickCommands PROPERTIES OUTPUT_NAME 
teleopJoystickCommands PREFIX "")
target_link_libraries(${PROJECT_NAME}_teleopJoystickCommands
  ${catkin_LIBRARIES}
)
## Add cmake target dependencies of the executable
## same as for the library above. This is needed to force
## talon controller messages to be built before the
## talon controller code tries to use them
add_dependencies(${PROJECT_NAME}_teleopJoystickCommands
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## TOOLS ------------------------------------------------------




## Install ------------------------------------------------------------
# Install header files
#install(DIRECTORY 
#include/${PROJECT_NAME}
#DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)

#install(DIRECTORY
#launch
#config
#DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
