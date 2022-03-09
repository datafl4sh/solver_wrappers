
add_library(CG INTERFACE)
target_include_directories(CG INTERFACE include)

install(FILES "include/cg.hpp" DESTINATION include)