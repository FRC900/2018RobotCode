cmake_minimum_required(VERSION 2.8.3)
project(talon_controllers)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  angles
  control_msgs
  control_toolbox
  controller_interface
  forward_command_controller
  realtime_tools
  talon_interface
  urdf
)

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS
    angles
    control_msgs
    controller_interface
    forward_command_controller
    realtime_tools
	talon_interface
    urdf
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/talon_controller.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES talon_controllers_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
