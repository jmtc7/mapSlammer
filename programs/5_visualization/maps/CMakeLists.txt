cmake_minimum_required(VERSION 2.8)
set(CMAKE_VERBOSE_MAKEFILE 1) # to see all make process
SET(GCC_COVERAGE_COMPILE_FLAGS "-std=c++11 -Os -Wall -pedantic -lboost_system -lboost_filesystem") #Compilation flags for filesystem


project(sparseMap_visualizer)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS} ${GCC_COVERAGE_COMPILE_FLAGS})

add_executable (visualize_denseMap visualize_denseMap.cc)
target_link_libraries (visualize_denseMap ${PCL_LIBRARIES})

add_executable (visualize_sparseMap visualize_sparseMap.cc)
target_link_libraries (visualize_sparseMap ${PCL_LIBRARIES})

add_executable (visualize_fineMap visualize_fineMap.cc)
target_link_libraries (visualize_fineMap ${PCL_LIBRARIES})

add_executable (visualize_demon visualize_demon.cc)
target_link_libraries (visualize_demon ${PCL_LIBRARIES})
