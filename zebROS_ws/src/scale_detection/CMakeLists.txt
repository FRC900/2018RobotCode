cmake_minimum_required(VERSION 2.8.3)
project(scale_detection)

include ("../cmake_modules/CMakeOpt.cmake")

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
if (NOT DEFINED CMAKE_TOOLCHAIN_FILE)
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    message_filters
    std_msgs
    sensor_msgs
    cv_bridge
  )
  find_package(PCL REQUIRED COMPONENTS common segmentation visualization)
  find_package(ZED REQUIRED)
  find_package(Boost COMPONENTS filesystem system serialization iostreams thread REQUIRED)
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})

  list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

  link_directories(${ZED_LIBRARY_DIR})

  catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES scale_detection
  #  CATKIN_DEPENDS other_catkin_pkg
  #  DEPENDS system_lib
  )

  include_directories(
  # include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
  )

  ## Declare a C++ executable
  ## With catkin_make all packages are built within a single CMake context
  ## The recommended prefix ensures that target names across packages don't collide
  # add_executable(${PROJECT_NAME}_node src/scale_detection_node.cpp)
  add_executable (planar_seg_node src/planar_segmentation.cpp)

  ## Specify libraries to link a library or executable target against
  target_link_libraries(planar_seg_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS} 
    ${ZED_LIBRARIES} 
    ${Boost_LIBRARIES} 
    ${LibTinyXML2} 
    ${PCL_LIBRARIES}
  )

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(planar_seg_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS} 
   ${ZED_LIBRARIES} 
   ${Boost_LIBRARIES} 
   ${LibTinyXML2} 
   ${PCL_LIBRARIES}
 )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

  # Mark executables and/or libraries for installation
  install(TARGETS planar_seg_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION} # Might need to be ${CATKIN_GLOBAL_BIN_DESTINATION} ?
  )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_scale_detection.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
endif()
