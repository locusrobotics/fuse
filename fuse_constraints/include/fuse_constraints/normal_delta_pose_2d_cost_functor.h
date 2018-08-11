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
#ifndef FUSE_CONSTRAINTS_NORMAL_DELTA_POSE_2D_COST_FUNCTOR_H
#define FUSE_CONSTRAINTS_NORMAL_DELTA_POSE_2D_COST_FUNCTOR_H

#include <fuse_constraints/util.h>
#include <fuse_core/eigen.h>

#include <Eigen/Core>



namespace fuse_constraints
{

/**
 * @brief Implements a cost function that models a difference between pose variables.
 *
 * A single pose involves two variables: a 2D position and a 2D orientation. The generic NormalDelta cost function
 * only supports a single variable type, and computes the difference using per-element subtraction. This cost function
 * computes the difference using standard 2D transformation math:
 *
 *   delta = [R1 | t1]^-1 * [R2 | t2]
 *
 * where R1 and R2 are 2D rotation matrices formed from the 2D orientation variables, and t1 and t2 are the position
 * variables in vector form. Once the delta is computed, the difference between the computed delta and the expected
 * delta uses simple per-element subtraction.
 *
 *             ||    [ delta.x   - b(0)] ||^2
 *   cost(x) = ||A * [ delta.y   - b(1)] ||
 *             ||    [ delta.yaw - b(2)] ||
 *
 * where, the matrix A and the vector b are fixed. In case the user is interested in implementing a cost function of
 * the form:
 *
 *   cost(X) = (X - mu)^T S^{-1} (X - mu)
 *
 * where, mu is a vector and S is a covariance matrix, then, A = S^{-1/2}, i.e the matrix A is the square root
 * information matrix (the inverse of the covariance).
 */
class NormalDeltaPose2DCostFunctor
{
public:
  /**
   * @brief Constructor
   *
   * @param[in] A The residual weighting matrix, most likely the square root information matrix in order (x, y, yaw)
   * @param[in] b The exposed pose difference in order (x, y, yaw)
   */
  NormalDeltaPose2DCostFunctor(const fuse_core::Matrix3d& A, const fuse_core::Vector3d& b);

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
  fuse_core::Matrix3d A_;  //!< The residual weighting matrix, most likely the square root information matrix
  fuse_core::Vector3d b_;  //!< The measured difference between variable x0 and variable x1
};

NormalDeltaPose2DCostFunctor::NormalDeltaPose2DCostFunctor(const fuse_core::Matrix3d& A, const fuse_core::Vector3d& b) :
  A_(A),
  b_(b)
{
}

template <typename T>
bool NormalDeltaPose2DCostFunctor::operator()(
  const T* const position1,
  const T* const orientation1,
  const T* const position2,
  const T* const orientation2,
  T* residual) const
{
  Eigen::Map<const Eigen::Matrix<T, 2, 1>> position1_vector(position1);
  Eigen::Map<const Eigen::Matrix<T, 2, 1>> position2_vector(position2);
  Eigen::Map<Eigen::Matrix<T, 3, 1>> residuals_matrix(residual);
  residuals_matrix.template head<2>() =
    RotationMatrix2D(orientation1[0]).transpose() * (position2_vector - position1_vector) -
    b_.head<2>().template cast<T>();
  residuals_matrix(2) = (orientation2[0] - orientation1[0]) - T(b_(2));
  wrapAngle2D(residuals_matrix(2));
  // Scale the residuals by the square root information matrix to account for
  // the measurement uncertainty.
  residuals_matrix.applyOnTheLeft(A_.template cast<T>());
  return true;
}

}  // namespace fuse_constraints

#endif  // FUSE_CONSTRAINTS_NORMAL_DELTA_POSE_2D_COST_FUNCTOR_H
