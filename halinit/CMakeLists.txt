cmake_minimum_required(VERSION 2.8)
project(C-Control)

add_definitions(-std=c++11 -Wall -Wextra)

find_package(OpenCV REQUIRED)

#include_directories(SYSTEM /usr/include)
include_directories(SYSTEM /home/eblau/git/allwpilib/hal/include)
include_directories(/home/eblau/git/allwpilib/ni-libraries/include)
link_directories(/home/eblau/git/allwpilib/ni-libraries/lib)
link_directories(/home/eblau/git/allwpilib/myRobotCpp/build/output)

set(WPI_DEP_LIBS
    wpi
    HALAthena
    ntcore
    wpiutil
    cscore
    nilibraries
    ${OpenCV_LIBS}
    )

add_executable(halinit halinit.cpp)
target_link_libraries(halinit ${WPI_DEP_LIBS})
