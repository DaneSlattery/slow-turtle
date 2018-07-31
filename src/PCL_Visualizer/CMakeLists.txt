cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(visual)

find_package(PCL 1.7 REQUIRED)

if(NOT "${PCL_LIBRARIES}" STREQUAL "")
  list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
endif()

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (visual visual.cpp)
target_link_libraries (visual ${PCL_LIBRARIES})
