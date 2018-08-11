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
#ifndef FUSE_CONSTRAINTS_NORMAL_PRIOR_POSE_3D_COST_FUNCTOR_H
#define FUSE_CONSTRAINTS_NORMAL_PRIOR_POSE_3D_COST_FUNCTOR_H

#include <fuse_constraints/util.h>
#include <fuse_constraints/normal_prior_orientation_3d_cost_functor.h>
#include <fuse_core/eigen.h>

#include <Eigen/Core>


namespace fuse_constraints
{

/**
 * @brief Create a prior cost function on both the 3D position and orientation variables at once.
 *
 * The Ceres::NormalPrior cost function only supports a single variable. This is a convenience cost function that
 * applies a prior constraint on both the 3D position and orientation variables at once.
 *
 * The cost function is of the form:
 *
 *             ||    [  x       -     b(0)  ] ||^2
 *             ||    [  y       -     b(1)  ] ||
 *             ||    [  z       -     b(2)  ] ||
 *             ||A * [  (b(3:6) * q^-1)(1)  ] ||
 *   cost(x) = ||    [  (b(3:6) * q^-1)(2)  ] ||
 *             ||    [  (b(3:6) * q^-1)(3)  ] ||
 *
 * where, the matrix A and the vector b are fixed and (x, y, z, qw, qx, qy, qz) are the components of the position and
 * orientation variables. Note that the covariance submatrix for the quaternion is 3x3, representing errors around each
 * imaginary component axis. In case the user is interested in implementing a cost function of the form
 *
 *   cost(X) = (X - mu)^T S^{-1} (X - mu)
 *
 * where, mu is a vector and S is a covariance matrix, then, A = S^{-1/2}, i.e the matrix A is the square root
 * information matrix (the inverse of the covariance).
 */
class NormalPriorPose3DCostFunctor
{
public:
  /**
   * @brief Construct a cost function instance
   *
   * @param[in] A The residual weighting matrix, most likely the square root information matrix in order
   *              (x, y, z, qw, qx, qy, qz)
   * @param[in] b The 3D pose measurement or prior in order (x, y, z, qw, qx, qy, qz)
   */
  NormalPriorPose3DCostFunctor(const fuse_core::Matrix6d& A, const fuse_core::Vector7d& b);

  /**
   * @brief Evaluate the cost function. Used by the Ceres optimization engine.
   */
  template <typename T>
  bool operator()(const T* const position, const T* const orientation, T* residual) const;

private:
  fuse_core::Matrix6d A_;
  fuse_core::Vector7d b_;

  NormalPriorOrientation3DCostFunctor orientation_functor_;
};

NormalPriorPose3DCostFunctor::NormalPriorPose3DCostFunctor(const fuse_core::Matrix6d& A, const fuse_core::Vector7d& b) :
  A_(A),
  b_(b),
  orientation_functor_(fuse_core::Matrix3d::Identity(), b_.tail<4>())  // Delta will not be scaled
{
}

template <typename T>
bool NormalPriorPose3DCostFunctor::operator()(const T* const position, const T* const orientation, T* residual) const
{
  // Use the 3D orientation cost functor to compute the orientation delta
  orientation_functor_(orientation, &residual[3]);

  Eigen::Map<Eigen::Matrix<T, 6, 1> > residuals_map(residual);
  residuals_map(0) = position[0] - T(b_(0));
  residuals_map(1) = position[1] - T(b_(1));
  residuals_map(2) = position[2] - T(b_(2));

  // Scale the residuals by the square root information matrix to account for
  // the measurement uncertainty.
  residuals_map.applyOnTheLeft(A_.template cast<T>());

  return true;
}

}  // namespace fuse_constraints

#endif  // FUSE_CONSTRAINTS_NORMAL_PRIOR_POSE_3D_COST_FUNCTOR_H
