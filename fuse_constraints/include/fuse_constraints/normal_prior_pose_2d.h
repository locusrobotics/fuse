/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Locus Robotics
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
#ifndef FUSE_CONSTRAINTS_NORMAL_PRIOR_POSE_2D_H
#define FUSE_CONSTRAINTS_NORMAL_PRIOR_POSE_2D_H

#include <fuse_core/eigen.h>

#include <ceres/sized_cost_function.h>


namespace fuse_constraints
{

/**
 * @brief Create a prior cost function on both the position and orientation variables at once.
 *
 * The Ceres::NormalPrior cost function only supports a single variable. This is a convenience cost function that
 * applies a prior constraint on both the position and orientation variables at once.
 *
 * The cost function is of the form:
 *
 *             ||    [  x - b(0)] ||^2
 *   cost(x) = ||A * [  y - b(1)] ||
 *             ||    [yaw - b(2)] ||
 *
 * Here, the matrix A can be of variable size, thereby permitting the computation of errors for partial measurements.
 * The vector b is a fixed-size 3x1. In case the user is interested in implementing a cost function of the form:
 *
 *   cost(X) = (X - mu)^T S^{-1} (X - mu)
 *
 * where, mu is a vector and S is a covariance matrix, then, A = S^{-1/2}, i.e the matrix A is the square root
 * information matrix (the inverse of the covariance).
 */
class NormalPriorPose2D : public ceres::SizedCostFunction<ceres::DYNAMIC, 2, 1>
{
public:
  /**
   * @brief Construct a cost function instance
   *
   * The residual weighting matrix can vary in size, as this cost functor can be used to compute costs for partial
   * vectors. The number of rows of A will be the number of dimensions for which you want to compute the error, and the
   * number of columns in A will be fixed at 3. For example, if we just want to use the values of x and yaw, then \p A
   * will be of size 2x3.
   *
   * @param[in] A The residual weighting matrix, most likely the square root information matrix in order (x, y, yaw)
   * @param[in] b The pose measurement or prior in order (x, y, yaw)
   */
  NormalPriorPose2D(const fuse_core::MatrixXd& A, const fuse_core::Vector3d& b);

  /**
   * @brief Destructor
   */
  virtual ~NormalPriorPose2D() = default;

  /**
   * @brief Compute the cost values/residuals, and optionally the Jacobians, using the provided variable/parameter
   *        values
   */
  virtual bool Evaluate(
    double const* const* parameters,
    double* residuals,
    double** jacobians) const;

private:
  fuse_core::MatrixXd A_;  //!< The residual weighting matrix, most likely the square root information matrix
  fuse_core::Vector3d b_;  //!< The measured 2D pose value
};

}  // namespace fuse_constraints

#endif  // FUSE_CONSTRAINTS_NORMAL_PRIOR_POSE_2D_H
