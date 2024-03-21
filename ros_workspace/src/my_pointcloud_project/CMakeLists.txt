cmake_minimum_required(VERSION 3.0)
project(MyPointCloudProject)

set(CMAKE_CXX_STANDARD 14)

find_package(PCL 1.12 REQUIRED)

include_directories(
    include
    ${PCL_INCLUDE_DIRS}
)

link_directories(
    ${PCL_LIBRARY_DIRS}
)

add_definitions(
    ${PCL_DEFINITIONS}
)

add_executable(
    pointcloud_processor
    src/main.cpp
    src/PointCloudProcessor.cpp
)

target_link_libraries(
    pointcloud_processor
    ${PCL_LIBRARIES}
)

target_include_directories(
    pointcloud_processor 
    PRIVATE include/my_pointcloud_project
)