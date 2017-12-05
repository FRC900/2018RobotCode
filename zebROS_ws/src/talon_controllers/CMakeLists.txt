cmake_minimum_required(VERSION 2.8.3)
project(talon_controllers)

include ("../cmake_modules/CMakeOpt.cmake")

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  control_msgs
  controller_interface
  message_generation
  realtime_tools
  talon_interface
  urdf
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options( cfg/TalonConfig.cfg)


add_message_files(FILES CloseLoopControllerMsg.msg)
generate_messages(DEPENDENCIES talon_controllers)


# Declare catkin package
catkin_package(
  CATKIN_DEPENDS
    control_msgs
    controller_interface
	message_runtime
    realtime_tools
	talon_interface
    urdf
	dynamic_reconfigure
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
#include_directories(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
include_directories($ENV{HOME}/2017Preseason/zebROS_ws/src/talon_controllers/cfg/cpp)


add_executable(talon_reconfigure_node src/talon_reconfigure_node.cpp)
add_dependencies(talon_reconfigure_node ${PROJECT_NAME}_gencfg)
target_link_libraries(talon_reconfigure_node ${catkin_LIBRARIES})


add_library(${PROJECT_NAME}
  src/talon_controller.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above. This is needed to force
## talon controller messages to be built before the
## talon controller code tries to use them
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
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
