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
#ifndef FUSE_CONSTRAINTS_NORMAL_PRIOR_ORIENTATION_3D_COST_FUNCTOR_H
#define FUSE_CONSTRAINTS_NORMAL_PRIOR_ORIENTATION_3D_COST_FUNCTOR_H

#include <fuse_constraints/util.h>
#include <fuse_core/eigen.h>
#include <fuse_variables/orientation_3d_stamped.h>

#include <ceres/internal/disable_warnings.h>
#include <ceres/rotation.h>
#include <Eigen/Core>


namespace fuse_constraints
{

/**
 * @brief Create a prior cost function on a 3D orientation variable (quaternion)
 *
 * The cost function is of the form:
 *
 *             ||    [  (b * q^-1)(1)  ] ||^2
 *   cost(x) = ||A * [  (b * q^-1)(2)  ] ||
 *             ||    [  (b * q^-1)(3)  ] ||
 *
 * where the matrix A and the vector b are fixed and (w, x, y, z) are the components of the 3D orientation
 * (quaternion) variable, whose indices are (0, 1, 2, 3). Note that the cost function does not include the
 * real-valued component of the quaternion, but only its imaginary components.
 *
 * In case the user is interested in implementing a cost function of the form
 *
 *   cost(X) = (X - mu)^T S^{-1} (X - mu)
 *
 * where, mu is a vector and S is a covariance matrix, then, A = S^{-1/2}, i.e the matrix A is the square root
 * information matrix (the inverse of the covariance).
 */
class NormalPriorOrientation3DCostFunctor
{
public:
  /**
   * @brief Construct a cost function instance
   *
   * @param[in] A The residual weighting matrix, most likely the square root information matrix in order
   *              (quaternion_x, quaternion_y, quaternion_z)
   * @param[in] b The orientation measurement or prior in order (w, x, y, z)
   */
  NormalPriorOrientation3DCostFunctor(
    const fuse_core::Matrix3d& A,
    const fuse_core::Vector4d& b) :
      A_(A),
      b_(b)
  {
  }

  /**
   * @brief Evaluate the cost function. Used by the Ceres optimization engine.
   */
  template <typename T>
  bool operator()(const T* const orientation, T* residuals) const
  {
    using fuse_variables::Orientation3DStamped;

    // 1. Compute the delta quaternion
    T inverse_quaternion[4] =
    {
      orientation[0],
      -orientation[1],
      -orientation[2],
      -orientation[3]
    };

    T observation[4] =
    {
      T(b_(0)),
      T(b_(1)),
      T(b_(2)),
      T(b_(3))
    };

    T output[4];

    ceres::QuaternionProduct(observation, inverse_quaternion, output);

    // 2. Can use just the imaginary coefficients as the residual
    residuals[0] = output[1];
    residuals[1] = output[2];
    residuals[2] = output[3];

    // 3. Scale the residuals by the square root information matrix to account for the measurement uncertainty.
    Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, 1>> residuals_map(residuals, A_.rows());
    residuals_map.applyOnTheLeft(A_.template cast<T>());

    return true;
  }

private:
  fuse_core::Matrix3d A_;  //!< The residual weighting matrix, most likely the square root information matrix
  fuse_core::Vector4d b_;  //!< The measured 3D orientation (quaternion) value
};

}  // namespace fuse_constraints

#endif  // FUSE_CONSTRAINTS_NORMAL_PRIOR_ORIENTATION_3D_COST_FUNCTOR_H
