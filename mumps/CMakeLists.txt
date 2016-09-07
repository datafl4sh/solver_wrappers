
find_package(MUMPS)
if (MUMPS_FOUND)
    include_directories(${MUMPS_INCLUDE_DIR})
    set(HAVE_MUMPS TRUE PARENT_SCOPE)
    set(SOLVER_WRAPPERS_LIBRARIES ${SOLVER_WRAPPERS_LIBRARIES} ${MUMPS_LIBRARIES} -lmpi -lmpi_mpifh -lblas PARENT_SCOPE)
endif()
