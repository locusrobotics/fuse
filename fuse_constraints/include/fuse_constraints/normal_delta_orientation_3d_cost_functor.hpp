/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Locus Robotics
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
#ifndef FUSE_CONSTRAINTS__NORMAL_DELTA_ORIENTATION_3D_COST_FUNCTOR_HPP_
#define FUSE_CONSTRAINTS__NORMAL_DELTA_ORIENTATION_3D_COST_FUNCTOR_HPP_

#include <ceres/rotation.h>
#include <Eigen/Core>

#include <fuse_core/eigen.hpp>
#include <fuse_core/fuse_macros.hpp>
#include <fuse_core/util.hpp>
#include <fuse_variables/orientation_3d_stamped.hpp>


namespace fuse_constraints
{

/**
 * @brief Implements a cost function that models a difference between 3D orientation variables
 *        (quaternion)
 *
 * The cost function is of the form:
 *
 *             ||                                  ||^2
 *   cost(x) = || A * AngleAxis(b^-1 * q1^-1 * q2) ||
 *             ||                                  ||
 *
 * where the matrix A and the vector b are fixed, and q1 and q2 are the variables, represented as
 * quaternions. The AngleAxis function converts a quaternion into a 3-vector of the form theta*k,
 * where k is the unit vector axis of rotation and theta is the angle of rotation. The A matrix is
 * applied to the angle-axis 3-vector.
 *
 * In case the user is interested in implementing a cost function of the form
 *
 *   cost(X) = (X - mu)^T S^{-1} (X - mu)
 *
 * where, mu is a vector and S is a covariance matrix, then, A = S^{-1/2}, i.e the matrix A is the
 * square root information matrix (the inverse of the covariance).
 */
class NormalDeltaOrientation3DCostFunctor
{
public:
  FUSE_MAKE_ALIGNED_OPERATOR_NEW()

  /**
   * @brief Construct a cost function instance
   *
   * @param[in] A The residual weighting matrix, most likely the square root information matrix in
   *              order (x, y, z)
   * @param[in] b The measured change between the two orientation variables
   */
  NormalDeltaOrientation3DCostFunctor(
    const fuse_core::Matrix3d & A,
    const fuse_core::Vector4d & b)
  : A_(A),
    b_(b)
  {
  }

  /**
   * @brief Evaluate the cost function. Used by the Ceres optimization engine.
   */
  template<typename T>
  bool operator()(const T * const orientation1, const T * const orientation2, T * residuals) const
  {
    using fuse_variables::Orientation3DStamped;

    T orientation1_inverse[4] =
    {
      orientation1[0],
      -orientation1[1],
      -orientation1[2],
      -orientation1[3]
    };

    T observation_inverse[4] =
    {
      T(b_(0)),
      T(-b_(1)),
      T(-b_(2)),
      T(-b_(3))
    };

    T difference[4];
    ceres::QuaternionProduct(orientation1_inverse, orientation2, difference);
    T error[4];
    ceres::QuaternionProduct(observation_inverse, difference, error);
    ceres::QuaternionToAngleAxis(error, residuals);

    // Scale the residuals by the square root information matrix to account for the measurement
    // uncertainty.
    Eigen::Map<Eigen::Matrix<T, 3, 1>> residuals_map(residuals);
    residuals_map.applyOnTheLeft(A_.template cast<T>());

    return true;
  }

private:
  fuse_core::Matrix3d A_;  //!< The residual weighting matrix, most likely the square root
                           //!< information matrix
  fuse_core::Vector4d b_;  //!< The measured difference between orientation1 and orientation2
};

}  // namespace fuse_constraints

#endif  // FUSE_CONSTRAINTS__NORMAL_DELTA_ORIENTATION_3D_COST_FUNCTOR_HPP_
