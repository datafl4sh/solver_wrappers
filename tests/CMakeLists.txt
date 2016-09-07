add_executable(agmg_test "agmg_test.cpp")
target_link_libraries(agmg_test ${SOLVER_WRAPPERS_LIBRARIES})

add_executable(mumps_test "mumps_test.cpp")
target_link_libraries(mumps_test ${SOLVER_WRAPPERS_LIBRARIES})
