cmake_minimum_required(VERSION 2.8.3)
project(ros_control_boilerplate)

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
)

add_message_files (
  FILES
  JoystickState.msg
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

# Sim version of main control
# executable.  For now this just prints debugging
# info for testing code which doens't actually hit
# RoboRIO or CAN hardware. Eventually expand into
# controlling some sort of sim?
add_executable(frcrobot_sim_main 
	src/frcrobot_sim_main.cpp
	src/frcrobot_sim_interface.cpp
	src/frc_robot_interface.cpp
	src/generic_hw_control_loop.cpp
)

target_link_libraries(frcrobot_sim_main
	${catkin_LIBRARIES}
)

if (DEFINED CMAKE_TOOLCHAIN_FILE)
    # FRCRobot Hardware Interface only
    # should be built for the RoboRIO since
    # it direcly interfaces with the RIO
    # hardware & attached devices

    # Main control executable
	add_executable(frcrobot_hw_main 
		src/frcrobot_hw_main.cpp
		src/frcrobot_hw_interface.cpp
		src/frc_robot_interface.cpp
		src/generic_hw_control_loop.cpp
	)

    include_directories(
      $ENV{HOME}/wpilib/user/cpp/include
      $ENV{HOME}/wpilib/cpp/current/include
    )
    
    set (WPI_LIB_DIR        $ENV{HOME}/wpilib/cpp/current/reflib/linux/athena/shared)
    set (WPI_COMMON_LIB_DIR $ENV{HOME}/wpilib/common/current/lib/linux/athena/shared)
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -L ${WPI_LIB_DIR}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -L ${WPI_COMMON_LIB_DIR}")
  
    set (CTRE_LIB_DIR $ENV{HOME}/wpilib/user/cpp/lib)
    set (CTRE_LIBS
        ${CTRE_LIB_DIR}/libCTRE_PhoenixCCI.a
        ${CTRE_LIB_DIR}/libCTRE_Phoenix.a
    )

	# Link against CTRE for CAN talon code and wpilib
	# for HAL, driver station, and so on
	target_link_libraries(frcrobot_hw_main
		${catkin_LIBRARIES}
		${WPI_LIB_DIR}/libwpi.so
		${WPI_LIB_DIR}/libFRC_NetworkCommunication.so.18.0.0
		${WPI_LIB_DIR}/libRoboRIO_FRC_ChipObject.so.18.0.0
		${CTRE_LIBS}
	)

## Add cmake target dependencies of the executable
## same as for the library above. This is needed to force
## talon controller messages to be built before the
## talon controller code tries to use them
add_dependencies(frcrobot_hw_main
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
    ## Install ------------------------------------------------------------

    # Install executables
	install(TARGETS
	  frcrobot_hw_main
	  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)
endif()


# Test trajectory generator node
add_executable(${PROJECT_NAME}_test_trajectory src/tools/test_trajectory.cpp)
set_target_properties(${PROJECT_NAME}_test_trajectory PROPERTIES OUTPUT_NAME test_trajectory PREFIX "")
target_link_libraries(${PROJECT_NAME}_test_trajectory
  ${catkin_LIBRARIES}
)

## TOOLS ------------------------------------------------------

# Tool for analyzing controller performance
add_executable(${PROJECT_NAME}_controller_to_csv_main 
	src/tools/controller_to_csv_main.cpp
	src/tools/controller_to_csv.cpp)
set_target_properties(${PROJECT_NAME}_controller_to_csv_main PROPERTIES OUTPUT_NAME controller_to_csv_main PREFIX "")
target_link_libraries(${PROJECT_NAME}_controller_to_csv_main
  ${Gflags_LIBRARIES}
  ${catkin_LIBRARIES}
)

# Tool for analyzing controller performance
add_library(csv_to_controller src/tools/csv_to_controller.cpp)
target_link_libraries(csv_to_controller
  ${catkin_LIBRARIES}
)
add_executable(${PROJECT_NAME}_csv_to_controller_main 
	src/tools/csv_to_controller_main.cpp
	src/tools/csv_to_controller.cpp)
set_target_properties(${PROJECT_NAME}_csv_to_controller_main PROPERTIES OUTPUT_NAME csv_to_controller_main PREFIX "")
target_link_libraries(${PROJECT_NAME}_csv_to_controller_main
  ${Gflags_LIBRARIES}
  ${catkin_LIBRARIES}
)

# Tool for controlling a robot from keyboard
add_executable(${PROJECT_NAME}_keyboard_teleop src/tools/keyboard_teleop.cpp)
set_target_properties(${PROJECT_NAME}_keyboard_teleop PROPERTIES OUTPUT_NAME keyboard_teleop PREFIX "")
target_link_libraries(${PROJECT_NAME}_keyboard_teleop
  ${catkin_LIBRARIES}
)



## Install ------------------------------------------------------------
install(TARGETS
  frcrobot_sim_main
    ${PROJECT_NAME}_test_trajectory
    ${PROJECT_NAME}_controller_to_csv_main
    ${PROJECT_NAME}_csv_to_controller_main
    ${PROJECT_NAME}_keyboard_teleop
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(DIRECTORY 
	include/${PROJECT_NAME}
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY
	launch
	config
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
