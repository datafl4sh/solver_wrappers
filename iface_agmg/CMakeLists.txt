cmake_minimum_required(VERSION 3.18 FATAL_ERROR)
project("agmg" LANGUAGES Fortran)

set (AGMG_SOURCE_DIR "${PROJECT_SOURCE_DIR}/agmg_source")
set (AGMG_LDSCRIPT_DIR "${PROJECT_SOURCE_DIR}/ldscripts")

if ( EXISTS "${AGMG_SOURCE_DIR}" AND IS_DIRECTORY "${AGMG_SOURCE_DIR}" )
    get_filename_component (Fortran_COMPILER_NAME ${CMAKE_Fortran_COMPILER} NAME)

    if (Fortran_COMPILER_NAME MATCHES "gfortran.*")
        set (CMAKE_Fortran_FLAGS " -w")
    endif (Fortran_COMPILER_NAME MATCHES "gfortran.*")

    set (COMMON_SOURCES "${AGMG_SOURCE_DIR}/blas_agmg.f" "${AGMG_SOURCE_DIR}/lapack_agmg.f")
    set (SAGMG_SOURCES  "${AGMG_SOURCE_DIR}/sagmg.f90" "${AGMG_SOURCE_DIR}/sagmg_mumps.f90")
    set (DAGMG_SOURCES  "${AGMG_SOURCE_DIR}/dagmg.f90" "${AGMG_SOURCE_DIR}/dagmg_mumps.f90")
    set (CAGMG_SOURCES  "${AGMG_SOURCE_DIR}/cagmg.f90" "${AGMG_SOURCE_DIR}/cagmg_mumps.f90")
    set (ZAGMG_SOURCES  "${AGMG_SOURCE_DIR}/zagmg.f90" "${AGMG_SOURCE_DIR}/zagmg_mumps.f90")

    add_library(agmg_common STATIC ${COMMON_SOURCES})

    add_library(sagmg SHARED ${SAGMG_SOURCES})
    target_link_libraries(sagmg agmg_common)
    add_library(dagmg SHARED ${DAGMG_SOURCES})
    target_link_libraries(dagmg agmg_common)
    add_library(cagmg SHARED ${CAGMG_SOURCES})
    target_link_libraries(cagmg agmg_common)
    add_library(zagmg SHARED ${ZAGMG_SOURCES})
    target_link_libraries(zagmg agmg_common)

    if ( "${CMAKE_SYSTEM_NAME}" MATCHES "Linux" )
        set_target_properties (sagmg PROPERTIES LINK_FLAGS
            "-Wl,--version-script=\"${AGMG_LDSCRIPT_DIR}/libsagmg.version\"")

        set_target_properties (dagmg PROPERTIES LINK_FLAGS
            "-Wl,--version-script=\"${AGMG_LDSCRIPT_DIR}/libdagmg.version\"")

        set_target_properties (cagmg PROPERTIES LINK_FLAGS
            "-Wl,--version-script=\"${AGMG_LDSCRIPT_DIR}/libcagmg.version\"")

        set_target_properties (zagmg PROPERTIES LINK_FLAGS
            "-Wl,--version-script=\"${AGMG_LDSCRIPT_DIR}/libzagmg.version\"")
    endif()

    if ( APPLE )
        set_target_properties (sagmg PROPERTIES LINK_FLAGS "-Wl,-exported_symbols_list \
                               \"${AGMG_LDSCRIPT_DIR}/libsagmg.map\"")

        set_target_properties (dagmg PROPERTIES LINK_FLAGS "-Wl,-exported_symbols_list \
                               \"${AGMG_LDSCRIPT_DIR}/libdagmg.map\"")

        set_target_properties (cagmg PROPERTIES LINK_FLAGS "-Wl,-exported_symbols_list \
                               \"${AGMG_LDSCRIPT_DIR}/libcagmg.map\"")

        set_target_properties (zagmg PROPERTIES LINK_FLAGS "-Wl,-exported_symbols_list \
                               \"${AGMG_LDSCRIPT_DIR}/libzagmg.map\"")
    endif()

    add_library(AGMG INTERFACE)
    target_include_directories(AGMG INTERFACE include)
    target_link_libraries(AGMG INTERFACE sagmg dagmg cagmg zagmg)
    target_compile_definitions(AGMG INTERFACE -DHAVE_AGMG)

    install(TARGETS sagmg dagmg cagmg zagmg DESTINATION lib)
    message(STATUS "AGMG Interface available")
endif()

install(FILES "include/agmg.hpp" DESTINATION include)
