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
#ifndef FUSE_CONSTRAINTS_NORMAL_PRIOR_POSE_2D_COST_FUNCTOR_H
#define FUSE_CONSTRAINTS_NORMAL_PRIOR_POSE_2D_COST_FUNCTOR_H

#include <fuse_constraints/util.h>
#include <fuse_core/eigen.h>

#include <ceres/internal/disable_warnings.h>
#include <ceres/internal/eigen.h>
#include <Eigen/Core>


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
 * where, the matrix A and the vector b are fixed and (x, y, yaw) are the components of the position and orientation
 * variables. In case the user is interested in implementing a cost function of the form
 *
 *   cost(X) = (X - mu)^T S^{-1} (X - mu)
 *
 * where, mu is a vector and S is a covariance matrix, then, A = S^{-1/2}, i.e the matrix A is the square root
 * information matrix (the inverse of the covariance).
 */
class NormalPriorPose2DCostFunctor
{
public:
  /**
   * @brief Construct a cost function instance
   *
   * @param[in] A The residual weighting matrix, most likely the square root information matrix in order (x, y, yaw)
   * @param[in] b The pose measurement or prior in order (x, y, yaw)
   */
  NormalPriorPose2DCostFunctor(const fuse_core::Matrix3d& A, const fuse_core::Vector3d& b);

  /**
   * @brief Evaluate the cost function. Used by the Ceres optimization engine.
   */
  template <typename T>
  bool operator()(const T* const position, const T* const orientation, T* residual) const;

private:
  fuse_core::Matrix3d A_;  //!< The residual weighting matrix, most likely the square root information matrix
  fuse_core::Vector3d b_;  //!< The measured 2D pose value
};

NormalPriorPose2DCostFunctor::NormalPriorPose2DCostFunctor(const fuse_core::Matrix3d& A, const fuse_core::Vector3d& b) :
  A_(A),
  b_(b)
{
}

template <typename T>
bool NormalPriorPose2DCostFunctor::operator()(const T* const position, const T* const orientation, T* residual) const
{
  Eigen::Map<Eigen::Matrix<T, 3, 1>> residuals_map(residual);
  residuals_map(0) = position[0] - T(b_(0));
  residuals_map(1) = position[1] - T(b_(1));
  residuals_map(2) = orientation[0] - T(b_(2));
  wrapAngle2D(residuals_map(2));
  // Scale the residuals by the square root information matrix to account for
  // the measurement uncertainty.
  residuals_map.applyOnTheLeft(A_.template cast<T>());
  return true;
}

}  // namespace fuse_constraints

#endif  // FUSE_CONSTRAINTS_NORMAL_PRIOR_POSE_2D_COST_FUNCTOR_H
