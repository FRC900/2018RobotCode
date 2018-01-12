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
)

add_message_files (
  FILES
  JoystickState.msg
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
if (DEFINED CMAKE_TOOLCHAIN_FILE)
    # FRCRobot Hardware Interface only
    # should be built for the RoboRIO since
    # it direcly interfaces with the RIO
    # hardware & attached devices

    # Main control executable

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

endif()



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

## TOOLS ------------------------------------------------------




## Install ------------------------------------------------------------
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
