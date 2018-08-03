cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(sift-keypoint)

find_package(PCL 1.3 REQUIRED)

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (filter-cloud filter-cloud.cpp)
target_link_libraries (filter-cloud ${PCL_LIBRARIES})