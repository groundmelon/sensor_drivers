cmake_minimum_required(VERSION 2.8.3)
project(bluefox2)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-std=c++11 -O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    )

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
    /opt/mvIMPACT_acquire
    )

add_executable(bluefox2 
    src/camera_node.cpp
    src/camera.cpp
    )

target_link_libraries(bluefox2
    ${catkin_LIBRARIES}
    /opt/mvIMPACT_acquire/lib/x86_64/libmvDeviceManager.so
    )

