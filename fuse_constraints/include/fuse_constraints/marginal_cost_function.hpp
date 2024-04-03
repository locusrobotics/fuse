/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef FUSE_CONSTRAINTS__MARGINAL_COST_FUNCTION_HPP_
#define FUSE_CONSTRAINTS__MARGINAL_COST_FUNCTION_HPP_

#include <ceres/cost_function.h>

#include <vector>

#include <fuse_core/ceres_macros.hpp>
#include <fuse_core/eigen.hpp>
#include <fuse_core/local_parameterization.hpp>
#include <fuse_core/manifold.hpp>

namespace fuse_constraints
{

/**
 * @brief Implements a cost function designed for precomputed marginal distributions
 *
 * The cost function is of the form:
 *
 *             ||                                                                        ||^2
 *   cost(x) = || A1 * (x1 - x1_bar) + A2 * (x2 - x2_bar) + ... + An * (xn - xn_bar) + b ||
 *             ||                                                                        ||
 *
 * where, the A matrices and the b vector are fixed, x_bar is the linearization point used when
 * calculating the A matrices and b vector, and the minus operator in (x - x_bar) is provided by the
 * variable's local parameterization.
 *
 * The A matrices can have any number of rows, but they must all be the same. The number of columns
 * of each A matrix must match the associated variable's local parameterization size, and the number
 * of rows of each x_bar must match the associated variable's global size. The cost function will
 * have the same number of residuals as the rows of A.
 */
class MarginalCostFunction : public ceres::CostFunction
{
public:
#if !CERES_SUPPORTS_MANIFOLDS
  /**
   * @brief Construct a cost function instance
   *
   * @param[in] A                       The A matrix of the marginal cost
   *                                    (of the form A*(x - x_bar) + b)
   * @param[in] b                       The b vector of the marginal cost
   *                                    (of the form A*(x - x_bar) + b)
   * @param[in] x_bar                   The linearization point of the involved variables
   * @param[in] local_parameterizations The local parameterization associated with the variable
   */
  MarginalCostFunction(
    const std::vector<fuse_core::MatrixXd> & A,
    const fuse_core::VectorXd & b,
    const std::vector<fuse_core::VectorXd> & x_bar,
    const std::vector<fuse_core::LocalParameterization::SharedPtr> & local_parameterizations);
#else
  /**
   * @brief Construct a cost function instance
   *
   * @param[in] A         The A matrix of the marginal cost (of the form A*(x - x_bar) + b)
   * @param[in] b         The b vector of the marginal cost (of the form A*(x - x_bar) + b)
   * @param[in] x_bar     The linearization point of the involved variables
   * @param[in] manifolds The manifold associated with the variable
   */
  MarginalCostFunction(
    const std::vector<fuse_core::MatrixXd> & A,
    const fuse_core::VectorXd & b,
    const std::vector<fuse_core::VectorXd> & x_bar,
    const std::vector<fuse_core::Manifold::SharedPtr> & manifolds);
#endif

  /**
   * @brief Destructor
   */
  virtual ~MarginalCostFunction() = default;

  /**
   * @brief Compute the cost values/residuals, and optionally the Jacobians, using the provided
   *        variable/parameter values
   */
  bool Evaluate(
    double const * const * parameters,
    double * residuals,
    double ** jacobians) const override;

private:
  const std::vector<fuse_core::MatrixXd> & A_;  //!< The A matrices of the marginal cost
  const fuse_core::VectorXd & b_;  //!< The b vector of the marginal cost
#if !CERES_SUPPORTS_MANIFOLDS
  //!< Parameterizations
  const std::vector<fuse_core::LocalParameterization::SharedPtr> & local_parameterizations_;
#else
  const std::vector<fuse_core::Manifold::SharedPtr> & manifolds_;  //!< Manifolds
#endif
  const std::vector<fuse_core::VectorXd> & x_bar_;  //!< The linearization point of each variable
};

}  // namespace fuse_constraints

#endif  // FUSE_CONSTRAINTS__MARGINAL_COST_FUNCTION_HPP_
