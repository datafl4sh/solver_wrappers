cmake_minimum_required(VERSION 3.18 FATAL_ERROR)
project("solver_wrappers")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(iface_agmg)
add_subdirectory(iface_mumps)


set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

option(OPT_BUILD_AGMG_TEST "Build AGMG interface test" OFF)
if (OPT_BUILD_AGMG_TEST)
    find_package(Eigen3 REQUIRED)
    add_executable(test_agmg tests/test_agmg.cpp)
    target_link_libraries(test_agmg AGMG Eigen3::Eigen)
    install(TARGETS test_agmg DESTINATION bin)
endif()

option(OPT_BUILD_MUMPS_TEST "Build MUMPS interface test" OFF)
if (OPT_BUILD_MUMPS_TEST)
    find_package(Eigen3 REQUIRED)
    add_executable(test_mumps tests/test_mumps.cpp)
    target_link_libraries(test_mumps MUMPS Eigen3::Eigen)
    install(TARGETS test_mumps DESTINATION bin)
endif()

