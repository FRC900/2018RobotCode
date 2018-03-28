cmake_minimum_required(VERSION 2.8.3)
project(elevator_controller)

include ("../cmake_modules/CMakeOpt.cmake")

find_package(catkin REQUIRED COMPONENTS
    cmake_modules
	roscpp
	actionlib
	controller_interface
    talon_interface
    talon_controllers
    message_generation
    std_msgs
    std_srvs
)

find_package(Eigen3 REQUIRED)

add_message_files (
  FILES
  ElevatorControl.msg
  ReturnElevatorCmd.msg
  CubeState.msg
)

add_service_files (
  FILES
  ElevatorControlS.srv
  Intake.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
	roscpp
	actionlib
	controller_interface
    talon_interface
    talon_controllers
    message_runtime
    std_msgs
    std_srvs
  INCLUDE_DIRS include
  #LIBRARIES ${PROJECT_NAME}
  DEPENDS
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME} 
src/elevator_control.cpp 
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Add cmake target dependencies of the executable
# same as for the library above.
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES ${PROJECT_NAME}_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
