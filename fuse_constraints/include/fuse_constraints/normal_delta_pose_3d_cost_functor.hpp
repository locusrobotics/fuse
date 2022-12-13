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
#ifndef FUSE_CONSTRAINTS__NORMAL_DELTA_POSE_3D_COST_FUNCTOR_HPP_
#define FUSE_CONSTRAINTS__NORMAL_DELTA_POSE_3D_COST_FUNCTOR_HPP_

#include <ceres/rotation.h>

#include <fuse_constraints/normal_delta_orientation_3d_cost_functor.hpp>
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
 *   cost(x) = || A * [ q1^-1 * (p2 - p1) - b(0:2)        ] ||^2
 *             ||     [ AngleAxis(b(3:6)^-1 * q1^-1 * q2) ] ||
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
 * Note that the cost function's quaternion components are only concerned with the imaginary
 * components (qx, qy, qz).
 */
class NormalDeltaPose3DCostFunctor
{
public:
  FUSE_MAKE_ALIGNED_OPERATOR_NEW()

  /**
   * @brief Constructor
   *
   * @param[in] A The residual weighting matrix, most likely the square root information matrix in
   *              order (dx, dy, dz, dqx, dqy, dqz)
   * @param[in] b The exposed pose difference in order (dx, dy, dz, dqw, dqx, dqy, dqz)
   */
  NormalDeltaPose3DCostFunctor(const fuse_core::Matrix6d & A, const fuse_core::Vector7d & b);

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
  fuse_core::Matrix6d A_;  //!< The residual weighting matrix, most likely the square root
                           //!< information matrix
  fuse_core::Vector7d b_;  //!< The measured difference between variable pose1 and variable pose2

  NormalDeltaOrientation3DCostFunctor orientation_functor_;
};

NormalDeltaPose3DCostFunctor::NormalDeltaPose3DCostFunctor(
  const fuse_core::Matrix6d & A,
  const fuse_core::Vector7d & b)
: A_(A),
  b_(b),
  orientation_functor_(fuse_core::Matrix3d::Identity(), b_.tail<4>())  // Orientation residuals will
                                                                       // not be scaled
{
}

template<typename T>
bool NormalDeltaPose3DCostFunctor::operator()(
  const T * const position1,
  const T * const orientation1,
  const T * const position2,
  const T * const orientation2,
  T * residual) const
{
  // Compute the position delta between pose1 and pose2
  T orientation1_inverse[4] =
  {
    orientation1[0],
    -orientation1[1],
    -orientation1[2],
    -orientation1[3]
  };
  T position_delta[3] =
  {
    position2[0] - position1[0],
    position2[1] - position1[1],
    position2[2] - position1[2]
  };
  T position_delta_rotated[3];
  ceres::QuaternionRotatePoint(
    orientation1_inverse,
    position_delta,
    position_delta_rotated);

  // Compute the first three residual terms as (position_delta - b)
  residual[0] = position_delta_rotated[0] - T(b_[0]);
  residual[1] = position_delta_rotated[1] - T(b_[1]);
  residual[2] = position_delta_rotated[2] - T(b_[2]);

  // Use the 3D orientation cost functor to compute the orientation delta
  orientation_functor_(orientation1, orientation2, &residual[3]);

  // Map it to Eigen, and weight it
  Eigen::Map<Eigen::Matrix<T, 6, 1>> residual_map(residual);
  residual_map.applyOnTheLeft(A_.template cast<T>());

  return true;
}

}  // namespace fuse_constraints

#endif  // FUSE_CONSTRAINTS__NORMAL_DELTA_POSE_3D_COST_FUNCTOR_HPP_
