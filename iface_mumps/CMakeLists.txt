cmake_minimum_required(VERSION 3.18 FATAL_ERROR)
project("mumps_interface")

include(FindPackageHandleStandardArgs)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT MUMPS_SEQUENTIAL_OR_PARALLEL)
  set(MUMPS_FIND_COMPONENTS SEQ)
endif()

set(MUMPS_SEQUENTIAL_OR_PARALLEL "Sequential"
    CACHE STRING "Find sequential or parallel MUMPS.")

set_property(CACHE MUMPS_SEQUENTIAL_OR_PARALLEL
    PROPERTY STRINGS "Sequential" "Parallel")

if(${MUMPS_SEQUENTIAL_OR_PARALLEL} STREQUAL "Sequential")
    set(MUMPSS_NAMES smumps_seq smumps)
    set(MUMPSD_NAMES dmumps_seq dmumps)
    set(MUMPSC_NAMES cmumps_seq cmumps)
    set(MUMPSZ_NAMES zmumps_seq zmumps)
    set(MUMPS_COMMON_NAMES mumps_common_seq mumps_common)
elseif(${MUMPS_SEQUENTIAL_OR_PARALLEL} STREQUAL "Parallel")
  set(MUMPSS_NAMES smumps_ptscotch smumps_scotch smumps)
  set(MUMPSD_NAMES dmumps_ptscotch dmumps_scotch dmumps)
  set(MUMPSC_NAMES cmumps_ptscotch cmumps_scotch cmumps)
  set(MUMPSZ_NAMES zmumps_ptscotch zmumps_scotch zmumps)
  set(MUMPS_COMMON_NAMES mumps_common_ptscotch mumps_common_scotch mumps_common)
else()
  message(FATAL_ERROR "Invalid value \"${MUMPS_FIND_COMPONENTS}\" for MUMPS_SEQUENTIAL_OR_PARALLEL.")
endif()


######################################################################
## Libraries common to all MUMPS versions
find_library(MUMPS_COMMON_LIBRARIES
    NAMES ${MUMPS_COMMON_NAMES}
    HINTS ENV MUMPS_ROOT
    PATH_SUFFIXES lib lib64)
if(MUMPS_COMMON_LIBRARIES)
  list(APPEND MUMPS_LIBRARIES ${MUMPS_COMMON_LIBRARIES})
endif()

######################################################################
## mpiseq
if(${MUMPS_SEQUENTIAL_OR_PARALLEL} STREQUAL "Sequential")
    find_library(MUMPS_MPISEQ_LIBRARY
        NAMES mpiseq
        HINTS ENV MUMPS_ROOT
        PATH_SUFFIXES lib lib64)
    if(MUMPS_MPISEQ_LIBRARY)
        message(STATUS "MUMPS mpiseq found.")
        list(APPEND MUMPS_LIBRARIES ${MUMPS_MPISEQ_LIBRARY})
    endif()
endif()

######################################################################
## Extras
set(extras_libs ptscotch scotch metis pord pord_seq)
foreach(extra IN LISTS extras_libs)
    find_library(MUMPS_${extra}_LIBRARY
        NAMES ${extra}
        HINTS ENV MUMPS_ROOT
        PATH_SUFFIXES lib lib64)
    if(MUMPS_${extra}_LIBRARY)
        message(STATUS "MUMPS extra: " ${extra})
        list(APPEND ${MUMPS_LIBRARIES} ${MUMPS_${extra}_LIBRARY})
    endif()
endforeach()

######################################################################
## Real single precision MUMPS
find_path(MUMPSS_INCLUDE_DIR
    NAMES smumps_c.h
    HINTS ENV MUMPS_ROOT
    PATH_SUFFIXES include
  )

find_library(MUMPSS_LIBRARIES
    NAMES ${MUMPSS_NAMES}
    HINTS ENV MUMPS_ROOT
    PATH_SUFFIXES lib lib64)

find_package_handle_standard_args(MUMPSS DEFAULT_MSG MUMPSS_LIBRARIES MUMPSS_INCLUDE_DIR)

if (MUMPSS_FOUND)
    add_library(MUMPSS INTERFACE)
    target_link_libraries(MUMPSS INTERFACE ${MUMPSS_LIBRARIES} ${MUMPS_LIBRARIES})
    target_include_directories(MUMPSS INTERFACE "${MUMPSS_INCLUDE_DIR}")
    target_compile_definitions(MUMPSS INTERFACE -DHAVE_MUMPSS)
    list(APPEND MUMPS_VERSIONS MUMPSS)
endif()

######################################################################
## Real double precision MUMPS
find_path(MUMPSD_INCLUDE_DIR
    NAMES dmumps_c.h
    HINTS ENV MUMPS_ROOT
    PATH_SUFFIXES include
  )

find_library(MUMPSD_LIBRARIES
    NAMES ${MUMPSD_NAMES}
    HINTS ENV MUMPS_ROOT
    PATH_SUFFIXES lib lib64)

find_package_handle_standard_args(MUMPSD DEFAULT_MSG MUMPSD_LIBRARIES MUMPSD_INCLUDE_DIR)

if (MUMPSD_FOUND)
    add_library(MUMPSD INTERFACE)
    target_link_libraries(MUMPSD INTERFACE "${MUMPSD_LIBRARIES}")
    target_include_directories(MUMPSD INTERFACE ${MUMPSD_INCLUDE_DIR} ${MUMPS_LIBRARIES})
    target_compile_definitions(MUMPSD INTERFACE -DHAVE_MUMPSD)
    list(APPEND MUMPS_VERSIONS MUMPSD)
endif()

######################################################################
## Complex single precision MUMPS
find_path(MUMPSC_INCLUDE_DIR
    NAMES cmumps_c.h
    HINTS ENV MUMPS_ROOT
    PATH_SUFFIXES include
  )

find_library(MUMPSC_LIBRARIES
    NAMES ${MUMPSC_NAMES}
    HINTS ENV MUMPS_ROOT
    PATH_SUFFIXES lib lib64)

find_package_handle_standard_args(MUMPSC DEFAULT_MSG MUMPSC_LIBRARIES MUMPSC_INCLUDE_DIR)

if (MUMPSC_FOUND)
    add_library(MUMPSC INTERFACE)
    target_link_libraries(MUMPSC INTERFACE "${MUMPSC_LIBRARIES}")
    target_include_directories(MUMPSC INTERFACE ${MUMPSC_INCLUDE_DIR} ${MUMPS_LIBRARIES})
    target_compile_definitions(MUMPSC INTERFACE -DHAVE_MUMPSC)
    list(APPEND MUMPS_VERSIONS MUMPSC)
endif()

######################################################################
## Complex double precision MUMPS
find_path(MUMPSZ_INCLUDE_DIR
    NAMES zmumps_c.h
    HINTS ENV MUMPS_ROOT
    PATH_SUFFIXES include
  )

find_library(MUMPSZ_LIBRARIES
    NAMES ${MUMPSZ_NAMES}
    HINTS ENV MUMPS_ROOT
    PATH_SUFFIXES lib lib64)

find_package_handle_standard_args(MUMPSZ DEFAULT_MSG MUMPSZ_LIBRARIES MUMPSZ_INCLUDE_DIR)

if (MUMPSZ_FOUND)
    add_library(MUMPSZ INTERFACE)
    target_link_libraries(MUMPSZ INTERFACE "${MUMPSZ_LIBRARIES}")
    target_include_directories(MUMPSZ INTERFACE ${MUMPSZ_INCLUDE_DIR} ${MUMPS_LIBRARIES})
    target_compile_definitions(MUMPSZ INTERFACE -DHAVE_MUMPSZ)
    list(APPEND MUMPS_VERSIONS MUMPSZ)
endif()

######################################################################
## Interface
add_library(MUMPS INTERFACE)
target_link_libraries(MUMPS INTERFACE ${MUMPS_VERSIONS})
target_include_directories(MUMPS INTERFACE include)

list(LENGTH MUMPS_VERSIONS num_vers)
if(num_vers GREATER 0)
    message(STATUS "Solver wrappers: MUMPS interface available.")
    target_compile_definitions(MUMPS INTERFACE -DHAVE_MUMPS)
    set(HAVE_MUMPS TRUE PARENT_SCOPE)
endif()

install(FILES "include/mumps.hpp" DESTINATION include)