cmake_minimum_required(VERSION 2.8.3)
project(robot_visualizer)

include ("../cmake_modules/CMakeOpt.cmake")

find_package(catkin REQUIRED COMPONENTS
    cmake_modules
	roscpp
	std_msgs
	geometry_msgs
	message_generation
)

find_package(Eigen3 REQUIRED)

add_message_files (
  FILES
  RobotVisualizeState.msg

)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
	roscpp
  INCLUDE_DIRS include
  #LIBRARIES ${PROJECT_NAME}
  DEPENDS
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Add cmake target dependencies of the executable
## same as for the library above. This is needed to force
## talon controller messages to be built before the
## talon controller code tries to use them
add_dependencies(
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(robot_visualize src/robot_visualize.cpp)
set_target_properties(robot_visualize PROPERTIES OUTPUT_NAME 
robot_visualize PREFIX "")
target_link_libraries(robot_visualize
  ${catkin_LIBRARIES}
)
## Add cmake target dependencies of the executable
## same as for the library above. This is needed to force
## talon controller messages to be built before the
## talon controller code tries to use them
add_dependencies(robot_visualize
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## TOOLS ------------------------------------------------------

# Install ------------------------------------------------------------
#Install header files
install(TARGETS robot_visualize
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
    config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)











