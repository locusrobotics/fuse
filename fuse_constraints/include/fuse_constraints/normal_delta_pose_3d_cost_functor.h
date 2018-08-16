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
#ifndef FUSE_CONSTRAINTS_NORMAL_DELTA_POSE_3D_COST_FUNCTOR_H
#define FUSE_CONSTRAINTS_NORMAL_DELTA_POSE_3D_COST_FUNCTOR_H

#include <fuse_constraints/util.h>
#include <fuse_core/eigen.h>

#include <ceres/rotation.h>

namespace fuse_constraints
{

/**
 * @brief Implements a cost function that models a difference between 3D pose variables.
 *
 * A single pose involves two variables: a 3D position and a 3D orientation. This cost function computes the difference
 * using standard 3D transformation math:
 *
 *   delta = q1^-1 * [position2 - position1]
 *                   [          q2         ]
 *
 * where q1 and q2 are the orientations of the two poses, given as quaternions. Once the delta is computed, the
 * difference between the computed delta and the expected delta is given as follows:
 *
 *             ||    [       delta(0) - b(0)       ] ||^2
 *   cost(x) = ||    [       delta(1) - b(1)       ] ||
 *             ||A * [       delta(2) - b(2)       ] ||
 *             ||    [ (delta(3:6) * b(3:6)^-1)(1) ] ||
 *             ||    [ (delta(3:6) * b(3:6)^-1)(2) ] ||
 *             ||    [ (delta(3:6) * b(3:6)^-1)(3) ] ||
 *
 * where, the matrix A and the vector b are fixed. In case the user is interested in implementing a cost function of
 * the form:
 *
 *   cost(X) = (X - mu)^T S^{-1} (X - mu)
 *
 * where, mu is a vector and S is a covariance matrix, then, A = S^{-1/2}, i.e the matrix A is the square root
 * information matrix (the inverse of the covariance).
 *
 * Note that the cost function's quaternion components are only concerned with the imaginary components (qx, qy, qz).
 */
class NormalDeltaPose3DCostFunctor
{
public:
  /**
   * @brief Constructor
   *
   * @param[in] A The residual weighting matrix, most likely the square root information matrix in order
   *              (dx, dy, dz, dqx, dqy, dqz)
   * @param[in] b The exposed pose difference in order (dx, dy, dz, dqw, dqx, dqy, dqz)
   */
  NormalDeltaPose3DCostFunctor(const fuse_core::Matrix6d& A, const fuse_core::Vector7d& b);

  /**
   * @brief Compute the cost values/residuals using the provided variable/parameter values
   */
  template <typename T>
  bool operator()(
    const T* const position1,
    const T* const orientation1,
    const T* const position2,
    const T* const orientation2,
    T* residual) const;

private:
  fuse_core::Matrix6d A_;  //!< The residual weighting matrix, most likely the square root information matrix
  fuse_core::Vector7d b_;  //!< The measured difference between variable pose1 and variable pose2
};

NormalDeltaPose3DCostFunctor::NormalDeltaPose3DCostFunctor(const fuse_core::Matrix6d& A, const fuse_core::Vector7d& b) :
  A_(A),
  b_(b)
{
}

template <typename T>
bool NormalDeltaPose3DCostFunctor::operator()(
  const T* const position1,
  const T* const orientation1,
  const T* const position2,
  const T* const orientation2,
  T* residual) const
{
  // Based on the Ceres SLAM pose graph error calculation here:
  // https://github.com/ceres-solver/ceres-solver/blob/4fc5d25f9cbfe2aa333425ddad03bdc651335c24/examples/slam/pose_graph_3d/pose_graph_3d_error_term.h#L73

  // 1. Get the relative orientation change from the variable's orientation1 to orientation2
  T variable_orientation1_inverse[4] =
  {
    orientation1[0],
    -orientation1[1],
    -orientation1[2],
    -orientation1[3]
  };

  T variable_orientation_delta[4];
  ceres::QuaternionProduct(variable_orientation1_inverse, orientation2, variable_orientation_delta);

  // 2. Get the position change from pose1 to pose2, then rotate it into the frame of pose1
  T variable_position_delta[3] =
  {
    position2[0] - position1[0],
    position2[1] - position1[1],
    position2[2] - position1[2]
  };

  T variable_position_delta_rotated[3];
  ceres::QuaternionRotatePoint(
    variable_orientation1_inverse,
    variable_position_delta,
    variable_position_delta_rotated);

  // 3. Get the difference between the orientation delta that we just computed for orientation1 and orientation2,
  // and the measurement's orientation delta
  T observation_inverse[4] =
  {
    T(b_(3)),
    T(-b_(4)),
    T(-b_(5)),
    T(-b_(6))
  };

  T delta_difference_orientation[4];
  ceres::QuaternionProduct(
    variable_orientation_delta,
    observation_inverse,
    delta_difference_orientation);

  // 4. Compute the position delta, and throw everything into a residual at the same time
  residual[0] = variable_position_delta_rotated[0] - b_[0];
  residual[1] = variable_position_delta_rotated[1] - b_[1];
  residual[2] = variable_position_delta_rotated[2] - b_[2];
  residual[3] = T(2.0) * delta_difference_orientation[1];
  residual[4] = T(2.0) * delta_difference_orientation[2];
  residual[5] = T(2.0) * delta_difference_orientation[3];

  // 5. Map it to Eigen, and weight it
  Eigen::Map<Eigen::Matrix<T, 6, 1> > residual_map(residual);

  residual_map.applyOnTheLeft(A_.template cast<T>());

  return true;
}

}  // namespace fuse_constraints

#endif  // FUSE_CONSTRAINTS_NORMAL_DELTA_POSE_3D_COST_FUNCTOR_H
