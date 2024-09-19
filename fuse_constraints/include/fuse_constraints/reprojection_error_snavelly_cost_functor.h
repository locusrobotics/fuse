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
#ifndef FUSE_CONSTRAINTS_REPROJECTION_ERROR_SNAVELLY_COST_FUNCTOR_H
#define FUSE_CONSTRAINTS_REPROJECTION_ERROR_SNAVELLY_COST_FUNCTOR_H

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
 * variable, K is the calibration matrix created from the calibration variable (f, r1, r2), X is the set of marker 3D points,
 * R_b(0:3) is the Rotation matrix from the fixed landmark orentation (b(3:6)), b(0:2) is the fixed landmark position
 * and x is the 2D ovservations.
 *
 * Note that the covariance matrix is defined as a 2x2 error in pixel space, most likely a diagonal matrix.
 */
class ReprojectionErrorSnavellyCostFunctor
{
public:
  FUSE_MAKE_ALIGNED_OPERATOR_NEW();

  /**
   * @brief Construct a cost function instance
   *
   * @param[in] A The residual weighting matrix, most likely derived from the square root information
   *              matrix in order (u, v)
   * @param[in] b The 2D measurement or prior in order (u, v)
   * 
   **/
  ReprojectionErrorSnavellyCostFunctor(const fuse_core::Matrix2d& A, const fuse_core::Vector2d& b);

  /**
   * @brief Evaluate the cost function. Used by the Ceres optimization engine.
   */
  template <typename T>
  bool operator()(const T* const position, const T* const orientation, const T* const calibration,
                  const T* const point, T* residuals) const;

private:
  fuse_core::Matrix2d A_;
  fuse_core::Vector2d b_;
};

ReprojectionErrorSnavellyCostFunctor::ReprojectionErrorSnavellyCostFunctor(const fuse_core::Matrix2d& A,
                                                                          const fuse_core::Vector2d& b)
  : A_(A)
  , b_(b)
{
}

template <typename T>
bool ReprojectionErrorSnavellyCostFunctor::operator()(const T* const position, const T* const orientation,
                                              const T* const calibration,
                                              const T* const point,
                                              T* residuals) const
{
  // Point to Camera CF ( X' = [R|t] X = RX + t )
  // Rotate Point (RX)
  T p[3];
  ceres::QuaternionRotatePoint(orientation, point, p);

  // Ofset (+t)
  p[0]+=position[0];
  p[1]+=position[1];
  p[2]+=position[2];

  // Compute the center of distortion. The sign change comes from
  // the camera model that Noah Snavely's Bundler assumes, whereby
  // the camera coordinate system has a negative z axis.
  T uv[2];
  uv[0] = -p[0]/p[2];
  uv[1] = -p[1]/p[2];

  // Apply second and fourth order radial distortion.
  const T& l1 = calibration[1];
  const T& l2 = calibration[2];
  const T r2 = uv[0] * uv[0] + uv[1] * uv[1];
  const T dist = 1.0 + r2 * (l1 + l2 * r2);

  // Compute final projected point position.
  const T& f = calibration[0];
  const T predicted_x = f * dist * uv[0];
  const T predicted_y = f * dist * uv[1];

  // The error is the difference between the predicted and observed position.
  residuals[0] = predicted_x - b_[0];
  residuals[1] = predicted_y - b_[1];

  // Weight Residuals
  Eigen::Map<Eigen::Matrix<T, 2, 1>> residual_map(residuals);
  residual_map.applyOnTheLeft(A_.template cast<T>());

  return true;
}

}  // namespace fuse_constraints

#endif  // FUSE_CONSTRAINTS_REPROJECTION_ERROR_SNAVELLY_COST_FUNCTOR_H
