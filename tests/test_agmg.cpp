#include <iostream>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "agmg.hpp"

int main(void)
{
    using T = double;
    size_t ms = 10;
    
    using trip = Eigen::Triplet<T>;
    
    agmg_solver<T> solver;
    
    Eigen::SparseMatrix<T>              A(ms, ms);
    Eigen::Matrix<T, Eigen::Dynamic, 1> b(ms), x(ms);
    std::vector<trip>                   triplets;
    
    for (size_t i = 0; i < ms; i++)
    {
        triplets.push_back( trip(i, i, i+1) );
        b(i) = 1;
    }
    
    A.setFromTriplets(triplets.begin(), triplets.end());
    
    x = solver.solve(A, b);
    
    std::cout << x.transpose() << std::endl;
    
    std::cout << A << std::endl;
    
    return 0;
}