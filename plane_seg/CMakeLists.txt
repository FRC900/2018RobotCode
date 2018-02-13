cmake_minimum_required(VERSION 2.8)
set(CMAKE_BUILD_TYPE Release)
project( plane_segment )
add_definitions(-std=c++11 -DZED_SUPPORT -Wall -Wno-switch -ftrack-macro-expansion=0)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};../zebravision/cmake")
find_package(OpenCV REQUIRED )
find_package(CUDA REQUIRED)
find_package(ZED REQUIRED)
find_package(Boost COMPONENTS filesystem system serialization iostreams thread REQUIRED)
find_package(PCL REQUIRED COMPONENTS common segmentation visualization)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
find_library (LibTinyXML2 tinyxml2)

include_directories( ${OpenCV_INCLUDE_DIRS} ${ZED_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${Boost_INCLUDE_DIR} ../common .. ../navXTimeSync)

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS} /usr/local/cuda/lib64/stubs)
set (NAVX_SRCS
	../navXTimeSync/AHRS.cpp
	../navXTimeSync/ContinuousAngleTracker.cpp
	../navXTimeSync/InertialDataIntegrator.cpp
	../navXTimeSync/OffsetTracker.cpp
	../navXTimeSync/SerialIO.cpp
	../navXTimeSync/SerialPort.cpp )


list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

add_executable( plane_segment main.cpp ../common/mediain.cpp ../common/syncin.cpp ../common/asyncin.cpp ../common/zedcamerain.cpp ../common/zedsvoin.cpp ../common/zedcamerain.cpp ../common/ZvSettings.cpp ../common/zmsin.cpp ../common/cameraparams.cpp ../common/zedparams.cpp  ../common/portable_binary_iarchive.cpp ../common/portable_binary_oarchive.cpp ../common/ZvSettings.cpp ${NAVX_SRCS})
target_link_libraries( plane_segment ${OpenCV_LIBS} ${ZED_LIBRARIES} ${Boost_LIBRARIES} ${LibTinyXML2} ${PCL_LIBRARIES})
