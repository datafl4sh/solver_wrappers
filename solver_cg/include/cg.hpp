/*
 * DISK++, a template library for DIscontinuous SKeletal methods.
 *  
 * Matteo Cicuttin (C) 2020-2022
 * matteo.cicuttin@uliege.be
 *
 * University of Liège - Montefiore Institute
 * Applied and Computational Electromagnetics group  
 */
/*
 * Copyright (C) 2013-2016, Matteo Cicuttin - matteo.cicuttin@uniud.it
 * Department of Electrical Engineering, University of Udine
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Udine nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR(s) ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE AUTHOR(s) BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

enum class cg_status {
    CONVERGED,
    DIVERGED,
    NOT_CONVERGED,
    WRONG_MATRIX_SIZE,
    WRONG_RHS_SIZE,
    WRONG_SOLUTION_SIZE,
};

#include <iostream>
#include <fstream>

template<typename T>
struct conjugated_gradient_params
{
    T               rr_tol;
    T               rr_max;
    size_t          max_iter;
    bool            verbose;
    bool            save_iteration_history;
    bool            use_initial_guess;
    std::string     history_filename;

    conjugated_gradient_params() : rr_tol(1e-8),
                                   rr_max(20),
                                   max_iter(0),
                                   verbose(false),
                                   save_iteration_history(false),
                                   use_initial_guess(false) {}
};

// TODO: return false and some kind of error in case of non convergence.
template<typename T>
cg_status
conjugated_gradient(const conjugated_gradient_params<T>& cgp,
                    const Eigen::SparseMatrix<T>& A,
                    const Eigen::Matrix<T, Eigen::Dynamic, 1>& b,
                    Eigen::Matrix<T, Eigen::Dynamic, 1>& x)
{
    if ( A.rows() != A.cols() )
    {
        if (cgp.verbose)
            std::cout << "[CG solver] A square matrix is required" << std::endl;

        return cg_status::WRONG_MATRIX_SIZE;
    }

    auto N = A.cols();

    if (b.size() != N)
    {
        if (cgp.verbose)
            std::cout << "[CG solver] Wrong size of RHS vector" << std::endl;

        return cg_status::WRONG_RHS_SIZE;
    }

    if (!cgp.use_initial_guess)
        x = Eigen::Matrix<T, Eigen::Dynamic, 1>::Zero(N);

    if (x.size() != N)
    {
        if (cgp.verbose)
            std::cout << "[CG solver] Wrong size of solution vector" << std::endl;

        return cg_status::WRONG_SOLUTION_SIZE;
    }

    size_t  iter = 0;
    T       nr, nr0, alpha, beta, rho;
    T       rr_ratio = 1.0;

    Eigen::Matrix<T, Eigen::Dynamic, 1> d(N), r(N), r0(N), y(N);

    r0 = d = r = b - A*x;
    nr = nr0 = r.norm();

    std::ofstream iter_hist_ofs;
    if (cgp.save_iteration_history)
    {
        iter_hist_ofs.open(cgp.history_filename);
        if( not iter_hist_ofs.is_open() )
            std::cout << "[CG solver] Can't open iteration history file" << std::endl;
    }

    size_t max_iter = (cgp.max_iter == 0) ? N : cgp.max_iter;
    while ( true )
    {
        y = A*d;
        rho = r.dot(r);
        alpha = rho/d.dot(y);
        x = x + alpha * d;
        r = r - alpha * y;
        beta = r.dot(r)/rho;
        d = r + beta * d;
        nr = r.norm();
        iter++;

        rr_ratio = nr/nr0;

        if (cgp.verbose)
            std::cout << "\r -> CG Iteration " << iter << "/" << max_iter << ", rr = " << rr_ratio << std::flush;

        if (cgp.save_iteration_history and iter_hist_ofs.is_open())
            iter_hist_ofs << rr_ratio << std::endl;

        if (rr_ratio > cgp.rr_max)
            return cg_status::DIVERGED;
        
        if (rr_ratio < cgp.rr_tol)
            break;
        
        if (iter >= max_iter)
            return cg_status::NOT_CONVERGED;
    }

    if (cgp.verbose)
        std::cout << std::endl;

    if (cgp.save_iteration_history and iter_hist_ofs.is_open())
        iter_hist_ofs << rr_ratio << std::endl;

    return cg_status::CONVERGED;
}