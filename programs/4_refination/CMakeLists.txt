cmake_minimum_required(VERSION 2.8)
set(CMAKE_VERBOSE_MAKEFILE 1) # to see all make process
SET(GCC_COVERAGE_COMPILE_FLAGS "-std=c++11 -Os -Wall -pedantic -lboost_system -lboost_filesystem") #Compilation flags for filesystem


project(refinator)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS} ${GCC_COVERAGE_COMPILE_FLAGS})

add_executable (4_refinator 4_refinator.cc)
target_link_libraries (4_refinator ${PCL_LIBRARIES})
