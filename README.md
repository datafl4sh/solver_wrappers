# C++ Numerical solver wrappers

Numerical solvers are a very important part of most numerical codes, however solver developers insist in writing their stuff in Fortran, which is the worst crap ever conceived in terms of programming languages.

Here there are C++ wrappers for some famous solvers. The interface is templated and calling a solver is just a matter of instantiating the corresponding class and calling `x = solve(A, b)`. Solver parameters are already set to sensible defaults, however they can be modified by using the specific functions.

### AGMG Wrapper
The AGMG wrapper requires that you have obtained the AGMG source code from its author. Once you have it, put the contents of the `SRC` folder (except the makefile) in the folder `agmg/agmg_source`. An example on how to call AGMG from C++ is contained in `agmg/agmg_test.cpp`. The compilation of the code produces 4 shared libraries named `lib[sdcz]agmg.dylib` (on OS X) or `lib[sdcz]agmg.so` (on other Unices). The libraries are linked in a way such that only the corresponding `[sdcz]agmg()` function is exported. This is needed because otherwise it is impossible to use the solver for two different types at the same time in the same code, because of duplicated symbols.

### MUMPS Wrapper
Work in progress.