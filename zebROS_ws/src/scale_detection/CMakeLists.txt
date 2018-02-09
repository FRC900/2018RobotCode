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

  add_executable (planar_segmentation src/point_extraction/planar_segmentation.cpp)

  ## Specify libraries to link a library or executable target against
  target_link_libraries(planar_segmentation
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS} 
    ${ZED_LIBRARIES} 
    ${Boost_LIBRARIES} 
    ${LibTinyXML2} 
    ${PCL_LIBRARIES}
  )
endif()

