cmake_minimum_required(VERSION 2.8.3)
project(heartbeat_node)

include ("../cmake_modules/CMakeOpt.cmake")

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  controller_manager
  roscpp
  std_msgs
  ros_control_boilerplate
)

#add_message_files (
#  FILES
#)
#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    controller_manager
    roscpp
    std_msgs
    ros_control_boilerplate
  LIBRARIES
  DEPENDS 
)

###########
## Build ##
###########

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
)

## Add cmake target dependencies of the executable
## same as for the library above. This is needed to force
## talon controller messages to be built before the
## talon controller code tries to use them
add_dependencies(
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(heartbeat_jetson src/heartbeat_jetson.cpp)
set_target_properties(heartbeat_jetson PROPERTIES OUTPUT_NAME 
heartbeat_jetson PREFIX "")
target_link_libraries(heartbeat_jetson
  ${catkin_LIBRARIES}
)
add_executable(heartbeat_rio src/heartbeat_rio.cpp)
set_target_properties(heartbeat_rio PROPERTIES OUTPUT_NAME 
heartbeat_rio PREFIX "")
target_link_libraries(heartbeat_rio
  ${catkin_LIBRARIES}
)
## Add cmake target dependencies of the executable
## same as for the library above. This is needed to force
## talon controller messages to be built before the
## talon controller code tries to use them

## TOOLS ------------------------------------------------------

# Install ------------------------------------------------------------

#install(DIRECTORY
#launch
#config
#DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
install(DIRECTORY
	config
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
