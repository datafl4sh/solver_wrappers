if (HAVE_AGMG)
    add_executable(agmg_test "agmg_test.cpp")
    target_link_libraries(agmg_test ${SOLVER_WRAPPERS_LIBRARIES})
endif()

if (HAVE_MUMPS)
    add_executable(mumps_test "mumps_test.cpp")
    target_link_libraries(mumps_test ${SOLVER_WRAPPERS_LIBRARIES})
endif()
