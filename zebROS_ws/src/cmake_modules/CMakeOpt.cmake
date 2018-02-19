# Various compiler optimizations to apply to all nodes

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release)
endif()

add_definitions(-std=c++11 -Wno-deprecated-declarations -Wno-switch -ftrack-macro-expansion=0 -DPCL_ONLY_CORE_POINT_TYPES=ON -DNO_EXPLICIT_INSTANTIATIONS)
#add_definitions(-Wall -Wextra -Wno-switch)

if (DEFINED CMAKE_TOOLCHAIN_FILE)  # Cross-build for Rio
  # Without -fno-finite-math-only, NaN checks are optimized away
  # This is bad since the Zed outputs NaN for invalid depth data
  set (OPT_FLAGS "-Ofast -flto=4 -fno-finite-math-only -mcpu=cortex-a9 -mfpu=neon")
  set (CMAKE_RANLIB "arm-frc-linux-gnueabi-gcc-ranlib-5.5")
  set (CMAKE_AR     "arm-frc-linux-gnueabi-gcc-ar-5.5")

  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath-link,/usr/arm-frc-linux-gnueabi/opt/ros/kinetic/lib")
else() # Native builds
  set (CMAKE_RANLIB "gcc-ranlib")
  set (CMAKE_AR     "gcc-ar")
  
  if (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "arm-linux-gnueabihf") # Jetson TK1
	set (OPT_FLAGS "-Ofast -flto=4 -fno-finite-math-only -mcpu=cortex-a15 -mfpu=neon-vfpv4")
    unset(CUDA_USE_STATIC_CUDA_RUNTIME CACHE)
    option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
  elseif (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "aarch64-linux-gnu") # Jetson TX1/TX2
	set (OPT_FLAGS "-Ofast -flto=4 -fno-finite-math-only -march=native -mtune=native")
    unset(CUDA_USE_STATIC_CUDA_RUNTIME CACHE)
    option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
  else() # x86? Mac?
	set (OPT_FLAGS "-Ofast -flto=4 -fno-finite-math-only -march=native -mtune=native")
  endif()
endif()

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${OPT_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} ${OPT_FLAGS}")
