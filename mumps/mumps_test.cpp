#include <iostream>

#include <mpi.h>

#include "mumps.hpp"

int main(int argc, char **argv)
{
    MPI_Init(&argc, &argv);
    
    Eigen::SparseMatrix<double> A(3,3);
    
    A.insert(0,0) = 10;
    A.insert(1,1) = 10;
    A.insert(2,2) = 10;
    A.insert(0,2) = 5;
    A.insert(2,0) = 5;
    
    
    Eigen::Matrix<double, Eigen::Dynamic, 1> b(3);
    b(0) = 1;
    b(1) = 2;
    b(2) = 1;
    
    mumps_solver<double> mumps;
    
    std::cout << mumps.solve(A, b) << std::endl;
    
    MPI_Finalize();
    
}