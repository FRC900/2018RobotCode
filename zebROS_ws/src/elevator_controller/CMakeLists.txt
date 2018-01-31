cmake_minimum_required(VERSION 2.8.3)
project(elevator_controller)

include ("../cmake_modules/CMakeOpt.cmake")

find_package(catkin REQUIRED COMPONENTS
    cmake_modules
    controller_interface
    nav_msgs
    realtime_tools
    talon_interface
    talon_controllers   
    tf
    message_generation
    urdf
    std_msgs
    sensor_msgs
    geometry_msgs
)
find_package(Eigen REQUIRED)

add_message_files (
  FILES
  ElevatorControl.msg
  Intake.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)



catkin_package(
  CATKIN_DEPENDS
	  controller_interface
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  DEPENDS Eigen
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
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

