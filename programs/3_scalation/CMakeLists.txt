cmake_minimum_required(VERSION 2.8)
set(CMAKE_VERBOSE_MAKEFILE 1) # to see all make process

project(3_rescaler)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (3_rescalerCOLOR_ransac 3_rescalerCOLOR_ransac.cc)
target_link_libraries (3_rescalerCOLOR_ransac ${PCL_LIBRARIES})
