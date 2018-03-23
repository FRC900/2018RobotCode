cmake_minimum_required(VERSION 2.8)
set(ARM_PREFIX arm-frc-linux-gnueabi)

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSROOT /usr/${ARM_PREFIX})

set(CMAKE_C_COMPILER ${ARM_PREFIX}-gcc)
set(CMAKE_CXX_COMPILER ${ARM_PREFIX}-g++)

set(CMAKE_FIND_ROOT_PATH ${SYSROOT_PATH};$ENV{HOME}/2018RobotCode/zebROS_ws/install_isolated)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

set(BOOST_ROOT ${ARM_PREFIX})
set(Boost_NO_SYSTEM_PATHS=ON)

add_definitions(-std=c++11)

find_program(CMAKE_RANLIB ${ARM_PREFIX}-gcc-ranlib-5.5)
find_program(CMAKE_AR ${ARM_PREFIX}-gcc-ar-5.5)
set(OPT_FLAGS "-O3 -flto=4 -mcpu=cortex-a9 -mfpu=neon")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${OPT_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} ${OPT_FLAGS}")
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath,/usr/arm-frc-linux-gnueabi/opt/ros/kinetic/lib")
set(CMAKE_INSTALL_RPATH "/usr/arm-frc-linux-gnueabi/opt/ros/kinetic/lib")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

