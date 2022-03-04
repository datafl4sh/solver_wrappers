#include <iostream>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "../solver_cg/cg.hpp"

int main(void)
{
    using T = double;
    size_t ms = 10;
    
    using trip = Eigen::Triplet<T>;
    
    Eigen::SparseMatrix<T>              A(ms, ms);
    Eigen::Matrix<T, Eigen::Dynamic, 1> b(ms), x(ms);
    std::vector<trip>                   triplets;
    
    for (size_t i = 0; i < ms; i++)
    {
        triplets.push_back( trip(i, i, i+1) );
        b(i) = 1;
    }
    
    A.setFromTriplets(triplets.begin(), triplets.end());
    
    conjugated_gradient_params<T> cgp;
    cgp.verbose = true;
    cgp.max_iter = 100;
    auto err = conjugated_gradient(cgp, A, b, x);
    
    if (err != cg_status::CONVERGED)
        std::cout << "CG solver not converged :(" << std::endl;

    std::cout << x.transpose() << std::endl;
    
    std::cout << A << std::endl;
    
    return 0;
}

