cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(OOP_PROJECT)

find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (main main.cpp CommonProcesses.cpp Region_Growing.cpp RAN_SA_C.cpp Segmentation.cpp)
target_link_libraries (main ${PCL_LIBRARIES})

