cmake_minimum_required (VERSION 2.8)
project (MUMPSLibs)

set (CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -g")

find_package(Eigen3)
if (EIGEN3_FOUND)
    include_directories(${EIGEN3_INCLUDE_DIR})
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DENABLE_EIGEN")
endif()

find_package(MUMPS)
if (MUMPS_FOUND)
    include_directories(${MUMPS_INCLUDE_DIR})
endif()
    

add_executable(mumps_test "mumps_test.cpp")
target_link_libraries(mumps_test ${MUMPS_LIBRARIES} -lmpi -lmpi_mpifh -lblas)
