/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Locus Robotics
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
#ifndef FUSE_CONSTRAINTS__NORMAL_DELTA_ORIENTATION_2D_HPP_
#define FUSE_CONSTRAINTS__NORMAL_DELTA_ORIENTATION_2D_HPP_

#include <ceres/sized_cost_function.h>


namespace fuse_constraints
{

/**
 * @brief Implements a cost function that models a difference between two 2D orientation variables
 *
 * The cost function is of the form:
 *
 *   cost(x) = ||A( (x1 - x0) - b)||^2
 *
 * where, the matrix A and the vector b are fixed and x0 and x1 are the 2D orientation variables.
 * In case the user is interested in implementing a cost function of the form
 *
 *   cost(x) = ((x1 - x0) - mu)^T S^{-1} ((x1 - x0) - mu)
 *
 * where, mu is a vector and S is a covariance matrix, then, A = S^{-1/2}, i.e the matrix A is the
 * square root information matrix (the inverse of the covariance). This is a specialization of the
 * generic "normal delta" cost function that handles the 2*pi roll-over that occurs with rotations.
 */
class NormalDeltaOrientation2D : public ceres::SizedCostFunction<1, 1, 1>
{
public:
  /**
   * @brief Constructor
   *
   * The number of rows in vector b must be the same as the number of columns of matrix A.
   *
   * @param[in] A The residual weighting matrix, most likely the square root information matrix
   * @param[in] b The measured difference between variable x0 and variable x1. It is assumed that
   *              these are the same type of variable. At a minimum, they must have the same
   *              dimensions and the per-element subtraction operator must be valid.
   */
  NormalDeltaOrientation2D(const double A, const double b);

  /**
   * @brief Destructor
   */
  virtual ~NormalDeltaOrientation2D() = default;

  /**
   * @brief Compute the cost values/residuals, and optionally the Jacobians, using the provided
   *        variable/parameter values
   */
  virtual bool Evaluate(
    double const * const * parameters,
    double * residuals,
    double ** jacobians) const;

private:
  double A_;  //!< The residual weighting matrix, most likely the square root information matrix
  double b_;  //!< The measured difference between variable x0 and variable x1
};

}  // namespace fuse_constraints

#endif  // FUSE_CONSTRAINTS__NORMAL_DELTA_ORIENTATION_2D_HPP_
