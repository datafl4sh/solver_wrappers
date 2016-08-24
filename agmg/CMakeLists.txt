cmake_minimum_required (VERSION 2.8)
project (AGMGLibs)
enable_language (Fortran)

set (CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")

find_package(Eigen3)
if (EIGEN3_FOUND)
    include_directories(${EIGEN3_INCLUDE_DIR})
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DENABLE_EIGEN")
endif()
    

set (AGMG_SOURCE_DIR "${PROJECT_SOURCE_DIR}/agmg_source")

if ( EXISTS "${AGMG_SOURCE_DIR}" AND IS_DIRECTORY "${AGMG_SOURCE_DIR}" ) 
    
    set (AGMG_FOUND TRUE PARENT_SCOPE)

    get_filename_component (Fortran_COMPILER_NAME ${CMAKE_Fortran_COMPILER} NAME)

    if (Fortran_COMPILER_NAME MATCHES "gfortran.*")
        set (CMAKE_Fortran_FLAGS " -w")
    endif (Fortran_COMPILER_NAME MATCHES "gfortran.*")

    set (COMMON_SOURCES "${AGMG_SOURCE_DIR}/blas_agmg.f" "${AGMG_SOURCE_DIR}/lapack_agmg.f")
    #set (COMMON_SOURCES "")
    set (SAGMG_SOURCES  "${AGMG_SOURCE_DIR}/sagmg.f90" "${AGMG_SOURCE_DIR}/sagmg_mumps.f90")
    set (DAGMG_SOURCES  "${AGMG_SOURCE_DIR}/dagmg.f90" "${AGMG_SOURCE_DIR}/dagmg_mumps.f90")
    set (CAGMG_SOURCES  "${AGMG_SOURCE_DIR}/cagmg.f90" "${AGMG_SOURCE_DIR}/cagmg_mumps.f90")
    set (ZAGMG_SOURCES  "${AGMG_SOURCE_DIR}/zagmg.f90" "${AGMG_SOURCE_DIR}/zagmg_mumps.f90")

    add_library (sagmg SHARED ${SAGMG_SOURCES} ${COMMON_SOURCES})
    add_library (dagmg SHARED ${DAGMG_SOURCES} ${COMMON_SOURCES})
    add_library (cagmg SHARED ${CAGMG_SOURCES} ${COMMON_SOURCES})
    add_library (zagmg SHARED ${ZAGMG_SOURCES} ${COMMON_SOURCES})

	if ( CYGWIN )
		set_target_properties (sagmg PROPERTIES LINK_FLAGS 
			"-Wl,--version-script=\"${CMAKE_CURRENT_SOURCE_DIR}/libsagmg.version\"")
		
		set_target_properties (dagmg PROPERTIES LINK_FLAGS 
			"-Wl,--version-script=\"${CMAKE_CURRENT_SOURCE_DIR}/libdagmg.version\"")
							   
		set_target_properties (cagmg PROPERTIES LINK_FLAGS 
			"-Wl,--version-script=\"${CMAKE_CURRENT_SOURCE_DIR}/libcagmg.version\"")
							   
		set_target_properties (zagmg PROPERTIES LINK_FLAGS 
			"-Wl,--version-script=\"${CMAKE_CURRENT_SOURCE_DIR}/libzagmg.version\"")
	endif (CYGWIN)
	
	if ( APPLE )
		set_target_properties (sagmg PROPERTIES LINK_FLAGS "-Wl,-exported_symbols_list \
                               \"${CMAKE_CURRENT_SOURCE_DIR}/sagmg_export.map\"")
                               #target_link_libraries(sagmg "-lblas -llapack")

		set_target_properties (dagmg PROPERTIES LINK_FLAGS "-Wl,-exported_symbols_list \
                               \"${CMAKE_CURRENT_SOURCE_DIR}/dagmg_export.map\"")
                               #target_link_libraries(dagmg "-lblas -llapack")

		set_target_properties (cagmg PROPERTIES LINK_FLAGS "-Wl,-exported_symbols_list \
							   \"${CMAKE_CURRENT_SOURCE_DIR}/cagmg_export.map\"")
                               #target_link_libraries(cagmg "-lblas -llapack")

		set_target_properties (zagmg PROPERTIES LINK_FLAGS "-Wl,-exported_symbols_list \
							   \"${CMAKE_CURRENT_SOURCE_DIR}/zagmg_export.map\"")
                               #target_link_libraries(zagmg "-lblas -llapack")

	endif ( APPLE )

    set(AGMG_LIBRARIES "${AGMG_LIBRARIES}" sagmg dagmg cagmg zagmg)

    add_executable(agmg_test "agmg_test.cpp")
    target_link_libraries(agmg_test ${AGMG_LIBRARIES})

    
    

endif ()
