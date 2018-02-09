cmake_minimum_required(VERSION 2.8.3)
project(talon_swerve_drive_controller)

include ("../cmake_modules/CMakeOpt.cmake")

find_package(catkin REQUIRED COMPONENTS
    cmake_modules
    controller_interface
    std_msgs
    nav_msgs
    geometry_msgs
    trajectory_msgs
    realtime_tools
    talon_interface
    talon_controllers
    tf
    urdf
    message_generation
)
find_package(Eigen REQUIRED)

add_message_files (
  FILES
  CompleteCmd.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  trajectory_msgs
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



add_executable(swerve_command_combiner src/swerve_command_combiner.cpp)
set_target_properties(swerve_command_combiner PROPERTIES OUTPUT_NAME
swerve_command_combiner PREFIX "")
target_link_libraries(swerve_command_combiner
  ${catkin_LIBRARIES}
)

add_dependencies(swerve_command_combiner
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_library(${PROJECT_NAME} 
src/swerve_drive_controller.cpp 
src/odometry.cpp 
src/speed_limiter.cpp
src/900Math.cpp
src/SwerveMath.cpp
src/Swerve.cpp
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES ${PROJECT_NAME}_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

