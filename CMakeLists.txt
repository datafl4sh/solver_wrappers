cmake_minimum_required (VERSION 2.8)
project (solver_wrappers)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")

set (CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(Eigen3)
if (EIGEN3_FOUND)
    include_directories(${EIGEN3_INCLUDE_DIR})
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DENABLE_EIGEN")
endif()

add_subdirectory(agmg)
add_subdirectory(mumps)

configure_file (
  "${PROJECT_SOURCE_DIR}/config.h.in"
  "${PROJECT_BINARY_DIR}/_config_/config.h"
  )
include_directories("${PROJECT_BINARY_DIR}/_config_/")


add_subdirectory(tests)
