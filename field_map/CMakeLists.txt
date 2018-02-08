cmake_minimum_required(VERSION 2.8)
set(CMAKE_LEGACY_CYGWIN_WIN32 0) # Remove when CMake >= 2.8.4 is required
project( grab_chroma )
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -std=c++11 -Wall -Wextra")

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Ofast -flto")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -Ofast -flto")

find_package( CUDA REQUIRED )
find_package( OpenCV REQUIRED )
find_package( Boost COMPONENTS filesystem system thread program_options serialization iostreams REQUIRED )

include_directories(/home/ubuntu/opencv-2.4.13/build/include)
include_directories( ${Boost_INCLUDE_DIR} )

link_directories(/home/ubuntu/opencv-2.4.13/build/lib)
add_executable( main main.cpp )
target_link_libraries( main ${OpenCV_LIBS} )
