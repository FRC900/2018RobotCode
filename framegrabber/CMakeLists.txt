cmake_minimum_required(VERSION 2.8)
project( framegrabber )
set(CMAKE_LEGACY_CYGWIN_WIN32 0) # Remove when CMake >= 2.8.4 is required
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Ofast -flto --std=c++11")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -Ofast -flto --std=c++11")
find_package( OpenCV REQUIRED )
add_executable( framegrabber framegrabber.cpp)
target_link_libraries( framegrabber ${OpenCV_LIBS} )

