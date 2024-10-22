/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, Giacomo Franchini
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
#ifndef FUSE_CONSTRAINTS__NORMAL_PRIOR_POSE_3D_EULER_COST_FUNCTOR_HPP_
#define FUSE_CONSTRAINTS__NORMAL_PRIOR_POSE_3D_EULER_COST_FUNCTOR_HPP_

#include <Eigen/Core>

#include <fuse_core/eigen.hpp>
#include <fuse_core/fuse_macros.hpp>
#include <fuse_core/util.hpp>


namespace fuse_constraints
{

/**
 * @brief Create a prior cost function on both the 3D position and orientation variables at once.
 *
 * The Ceres::NormalPrior cost function only supports a single variable. This is a convenience cost
 * function that applies a prior constraint on both the 3D position and orientation variables at
 * once.
 *
 * The cost function is of the form:
 *
 *   cost(x) = || A *       [  p - b(0:2)  ]      ||^2
 *             ||     [  quat2eul(q) - b(3:5)  ]  ||
 *
 * where, the matrix A and the vector b are fixed, p is the position variable, and q is the
 * orientation variable. In case the user is interested in implementing a cost function of the form
 *
 *   cost(X) = (X - mu)^T S^{-1} (X - mu)
 *
 * where, mu is a vector and S is a covariance matrix, then, A = S^{-1/2}, i.e the matrix A is the
 * square root information matrix (the inverse of the covariance).
 */
class NormalPriorPose3DEulerCostFunctor
{
public:
  FUSE_MAKE_ALIGNED_OPERATOR_NEW()

  /**
   * @brief Construct a cost function instance
   *
   * @param[in] A The residual weighting matrix, most likely the square root information matrix in
   *              order (x, y, z, roll, pitch, yaw)
   * @param[in] b The 3D pose measurement or prior in order (x, y, z, roll, pitch, yaw)
   */
  NormalPriorPose3DEulerCostFunctor(const fuse_core::MatrixXd & A, const fuse_core::Vector6d & b);

  /**
   * @brief Evaluate the cost function. Used by the Ceres optimization engine.
   */
  template<typename T>
  bool operator()(const T * const position, const T * const orientation, T * residual) const;

private:
  fuse_core::MatrixXd A_;
  fuse_core::Vector6d b_;
};

NormalPriorPose3DEulerCostFunctor::NormalPriorPose3DEulerCostFunctor(
  const fuse_core::MatrixXd & A,
  const fuse_core::Vector6d & b)
: A_(A),
  b_(b)
{
  CHECK_GT(A_.rows(), 0);
  CHECK_EQ(A_.cols(), 6);
}

template<typename T>
bool NormalPriorPose3DEulerCostFunctor::operator()(
  const T * const position, const T * const orientation,
  T * residual) const
{
  T full_residuals[6];
  T orientation_rpy[3] = {
    fuse_core::getRoll(orientation[0], orientation[1], orientation[2], orientation[3]),
    fuse_core::getPitch(orientation[0], orientation[1], orientation[2], orientation[3]),
    fuse_core::getYaw(orientation[0], orientation[1], orientation[2], orientation[3])
  };

  // Compute the position residual
  full_residuals[0] = position[0] - T(b_(0));
  full_residuals[1] = position[1] - T(b_(1));
  full_residuals[2] = position[2] - T(b_(2));
  // Compute the orientation residual
  full_residuals[3] = orientation_rpy[0] - T(b_(3));
  full_residuals[4] = orientation_rpy[1] - T(b_(4));
  full_residuals[5] = orientation_rpy[2] - T(b_(5));

  // Scale the residuals by the square root information matrix to account for
  // the measurement uncertainty.
  Eigen::Map<Eigen::Vector<T, 6>> full_residuals_map(full_residuals);
  Eigen::Map<Eigen::Vector<T, Eigen::Dynamic>> residuals_map(residual, A_.rows());
  residuals_map = A_.template cast<T>() * full_residuals_map;
  return true;
}

}  // namespace fuse_constraints

#endif  // FUSE_CONSTRAINTS__NORMAL_PRIOR_POSE_3D_COST_FUNCTOR_HPP_
