cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(pcl_example)

message(${PROJECT_SOURCE_DIR})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../bin)

find_package(PCL 1.12 REQUIRED)

include_directories(
    ${PCL_INCLUDE_DIRS}
    include
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(pcl_reader pcl_reader.cpp)

add_executable (downsample downsample.cpp)
target_link_libraries (downsample ${PCL_LIBRARIES})

add_executable (euclidean_cluster euclidean_cluster.cpp)
target_link_libraries (euclidean_cluster 
    pcl_reader
    ${PCL_LIBRARIES})

add_executable (environmentsetting environmentsetting.cpp)
