cmake_minimum_required(VERSION 2.8.3)
project(thermal_modeling)

include ("../cmake_modules/CMakeOpt.cmake")

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  std_msgs
  swerve_point_generator
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
    roscpp
    std_msgs
    swerve_point_generator
  LIBRARIES
  DEPENDS 
)

###########
## Build ##
###########

include_directories(SYSTEM)

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
)

## Add cmake target dependencies of the executable
## same as for the library above. This is needed to force
## talon controller messages to be built before the
## talon controller code tries to use them

add_executable(process_thermal_model src/process_thermal_model.cpp src/thermal_model.cpp)
set_target_properties(process_thermal_model PROPERTIES OUTPUT_NAME 
process_thermal_model PREFIX "")
target_link_libraries(process_thermal_model
  ${catkin_LIBRARIES}
)
## Add cmake target dependencies of the executable
## same as for the library above. This is needed to force
## talon controller messages to be built before the
## talon controller code tries to use them
add_dependencies(process_thermal_model
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## TOOLS ------------------------------------------------------

# Install ------------------------------------------------------------
#Install header files
install(TARGETS process_thermal_model
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#install(DIRECTORY
#launch
#config
#DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
install(DIRECTORY
	config
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
