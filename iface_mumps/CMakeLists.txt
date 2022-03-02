cmake_minimum_required(VERSION 3.18 FATAL_ERROR)
project("mumps_interface")

set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(MUMPS)

if (MUMPS_FOUND)
    #include_directories(${MUMPS_INCLUDE_DIR})
    #list(APPEND LINK_LIBS ${MUMPS_LIBRARIES})
    target_include_directories(MUMPS INTERFACE "include")
    target_compile_definitions(MUMPS INTERFACE -DHAVE_MUMPS)
endif()

install(FILES "include/mumps.hpp" DESTINATION include)