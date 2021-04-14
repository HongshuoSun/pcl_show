cmake_minimum_required(VERSION 3.10)
project(pcl_show)
set(CMAKE_CXX_STANDARD 14)
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Eigen3)

add_executable(pcl_show main.cpp CommonUtils.h)
target_link_libraries(pcl_show ${PCL_LIBRARIES})
target_link_libraries (pcl_show Eigen3::Eigen)
