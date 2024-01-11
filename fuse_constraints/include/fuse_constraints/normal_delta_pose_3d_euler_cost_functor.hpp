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
#ifndef FUSE_CONSTRAINTS__NORMAL_DELTA_POSE_3D_EULER_COST_FUNCTOR_HPP_
#define FUSE_CONSTRAINTS__NORMAL_DELTA_POSE_3D_EULER_COST_FUNCTOR_HPP_

#include <ceres/rotation.h>
#include <glog/logging.h>

#include <fuse_core/eigen.hpp>
#include <fuse_core/fuse_macros.hpp>
#include <fuse_core/util.hpp>


namespace fuse_constraints
{

/**
 * @brief Implements a cost function that models a difference between 3D pose variables.
 *
 * A single pose involves two variables: a 3D position and a 3D orientation. This cost function
 * computes the difference using standard 3D transformation math:
 *
 *   cost(x) = || A * [ (q1^-1 * (p2 - p1)) - b(0:2)  ] ||^2
 *             ||     [ quat2rpy(q1^-1 * q2) - b(3:5) ] ||
 *
 * where p1 and p2 are the position variables, q1 and q2 are the quaternion orientation variables,
 * and the matrix A and the vector b are fixed. In case the user is interested in implementing a
 * cost function of the form:
 *
 *   cost(X) = (X - mu)^T S^{-1} (X - mu)
 *
 * where, mu is a vector and S is a covariance matrix, then, A = S^{-1/2}, i.e the matrix A is the
 * square root information matrix (the inverse of the covariance).
 *
 */
class NormalDeltaPose3DEulerCostFunctor
{
public:
  FUSE_MAKE_ALIGNED_OPERATOR_NEW()

  /**
   * @brief Constructor
   *
   * @param[in] A The residual weighting matrix, most likely the square root information matrix in
   *              order (dx, dy, dz, droll, dpitch, dyaw)
   * @param[in] b The exposed pose difference in order (dx, dy, dz, droll, dpitch, dyaw)
   */
  NormalDeltaPose3DEulerCostFunctor(const fuse_core::MatrixXd & A, const fuse_core::Vector6d & b);

  /**
   * @brief Compute the cost values/residuals using the provided variable/parameter values
   */
  template<typename T>
  bool operator()(
    const T * const position1,
    const T * const orientation1,
    const T * const position2,
    const T * const orientation2,
    T * residual) const;

private:
  fuse_core::MatrixXd A_;
  fuse_core::Vector6d b_;
};

NormalDeltaPose3DEulerCostFunctor::NormalDeltaPose3DEulerCostFunctor(
  const fuse_core::MatrixXd & A,
  const fuse_core::Vector6d & b)
: A_(A),
  b_(b)
{
  CHECK_GT(A_.rows(), 0);
  CHECK_EQ(A_.cols(), 6);
}

template<typename T>
bool NormalDeltaPose3DEulerCostFunctor::operator()(
  const T * const position1,
  const T * const orientation1,
  const T * const position2,
  const T * const orientation2,
  T * residual) const
{
  T full_residuals[6];
  T position_delta_rotated[3];
  T orientation_delta[4];
  T orientation_delta_rpy[3];

  T orientation1_inverse[4] 
  {
    orientation1[0],
    -orientation1[1],
    -orientation1[2],
    -orientation1[3]
  };

  T position_delta[3] 
  {
    position2[0] - position1[0],
    position2[1] - position1[1],
    position2[2] - position1[2]
  };
  
  // Compute the position residual
  ceres::QuaternionRotatePoint(orientation1_inverse, position_delta, position_delta_rotated);
  full_residuals[0] = position_delta_rotated[0] - T(b_(0));
  full_residuals[1] = position_delta_rotated[1] - T(b_(1));
  full_residuals[2] = position_delta_rotated[2] - T(b_(2));

  // Compute the orientation residual
  ceres::QuaternionProduct(orientation1_inverse, orientation2, orientation_delta);
  orientation_delta_rpy[0] = fuse_core::getRoll(orientation_delta[0], orientation_delta[1], orientation_delta[2], orientation_delta[3]);
  orientation_delta_rpy[1] = fuse_core::getPitch(orientation_delta[0], orientation_delta[1], orientation_delta[2], orientation_delta[3]);
  orientation_delta_rpy[2] = fuse_core::getYaw(orientation_delta[0], orientation_delta[1], orientation_delta[2], orientation_delta[3]);
  full_residuals[3] = orientation_delta_rpy[0] - T(b_(3));
  full_residuals[4] = orientation_delta_rpy[1] - T(b_(4));
  full_residuals[5] = orientation_delta_rpy[2] - T(b_(5));

  // Scale the residuals by the square root information matrix to account for
  // the measurement uncertainty.
  Eigen::Map<Eigen::Vector<T, 6>> full_residuals_map(full_residuals);
  Eigen::Map<Eigen::Vector<T, Eigen::Dynamic>> residuals_map(residual, A_.rows());
  residuals_map = A_.template cast<T>() * full_residuals_map;

  return true;
}

}  // namespace fuse_constraints

#endif  // FUSE_CONSTRAINTS__NORMAL_DELTA_POSE_3D_EULER_COST_FUNCTOR_HPP_