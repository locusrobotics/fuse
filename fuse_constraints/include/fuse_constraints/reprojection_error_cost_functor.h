/*
 * Software License Agreement (BSD License)
 *
 *  Author: Oscar Mendez
 *  Created on Mon Dec 12 2023
 *
 *  Copyright (c) 2023, Locus Robotics
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
#ifndef FUSE_CONSTRAINTS_REPROJECTION_ERROR_COST_FUNCTOR_H
#define FUSE_CONSTRAINTS_REPROJECTION_ERROR_COST_FUNCTOR_H

#include <fuse_constraints/normal_prior_orientation_3d_cost_functor.h>
#include <fuse_core/eigen.h>
#include <fuse_core/fuse_macros.h>
#include <fuse_core/util.h>

#include <Eigen/Core>

namespace fuse_constraints
{

/**
 * @brief Create a prior cost function on the marker position, minimising reprojection error.
 *
 * The Ceres::NormalPrior cost function only supports a single variable. This is a convenience cost function that
 * applies a prior constraint on the 3D position, orientation and calibration variables at once.
 *
 * The cost function is of the form:
 *
 *   cost(x) = || A * (K * [R_q | p] * [R_{b(3:6)} | b(0:2))] * X - x) ||
 *
 * where, the matrix A and the vector b are fixed, p is the camera position variable, and q is the camera orientation
 * variable, K is the calibration matrix created from the calibration variable, X is the set of marker 3D points,
 * R_b(0:3) is the Rotation matrix from the fixed landmark orentation (b(3:6)), b(0:2) is the fixed landmark position
 * and x is the 2D ovservations.
 *
 * Note that the covariance submatrix for the quaternion is 3x3, representing errors in the orientation local
 * parameterization tangent space. In case the user is interested in implementing a cost function of the form
 *
 *   cost(X) = (X - mu)^T S^{-1} (X - mu)
 *
 * where, mu is a vector and S is a covariance matrix, then, A = S^{-1/2}, i.e the matrix A is the square root
 * information matrix (the inverse of the covariance).
 */
class ReprojectionErrorCostFunctor
{
public:
  FUSE_MAKE_ALIGNED_OPERATOR_NEW();

  /**
   * @brief Construct a cost function instance
   *
   * @param[in] A The residual weighting matrix, most likely derived from the square root information
   *              matrix in order (u, v)
   * @param[in] b The 2D pose measurement or prior in order (u, v
   *
   **/
  ReprojectionErrorCostFunctor(const fuse_core::Matrix2d& A, const fuse_core::Vector2d& b);

  /**
   * @brief Evaluate the cost function. Used by the Ceres optimization engine.
   */
  template <typename T>
  bool operator()(const T* const position, const T* const orientation, const T* const calibration, const T* const point,
                  T* residual) const;

private:
  fuse_core::Matrix2d A_;
  fuse_core::Vector2d b_;
};

ReprojectionErrorCostFunctor::ReprojectionErrorCostFunctor(const fuse_core::Matrix2d& A, const fuse_core::Vector2d& b)
  : A_(A), b_(b)
{
}

template <typename T>
bool ReprojectionErrorCostFunctor::operator()(const T* const position, const T* const orientation,
                                              const T* const calibration, const T* const point, T* residual) const
{

  // Point to Camera CF ( X' = [R|t] X = RX + t )
  // Rotate Point (RX)
  T p[3];
  ceres::QuaternionRotatePoint(orientation, point, p);

  // Ofset (+t)
  p[0] += position[0];
  p[1] += position[1];
  p[2] += position[2];

  // Project to camera ([u,v] = KX)
  // u = (x'fx + z'cx)/z'
  // v = (y'fy + z'cy)/z'
  T uv[2];
  uv[0] = (p[0] * calibration[0] + p[2] * calibration[2]) / p[2];
  uv[1] = (p[1] * calibration[1] + p[2] * calibration[3]) / p[2];

  // Get Residuals
  residual[0] = uv[0] - b_[0];
  residual[1] = uv[1] - b_[1];

  // Weight Residuals
  Eigen::Map<Eigen::Matrix<T, 2, 1>> residual_map(residual);
  residual_map.applyOnTheLeft(A_.template cast<T>());

  return true;
}

}  // namespace fuse_constraints

#endif  // FUSE_CONSTRAINTS_REPROJECTION_ERROR_COST_FUNCTOR_H
