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
#ifndef FUSE_CONSTRAINTS__NORMAL_DELTA_ORIENTATION_3D_EULER_COST_FUNCTOR_HPP_
#define FUSE_CONSTRAINTS__NORMAL_DELTA_ORIENTATION_3D_EULER_COST_FUNCTOR_HPP_

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
 *        (quaternion, but converted in RPY for residual computation)
 *
 * The cost function is of the form:
 *
 *             ||                                  ||^2
 *   cost(x) = || A * quat2rpy(b^-1 * q1^-1 * q2) ||
 *             ||                                  ||
 *
 * where the matrix A and the vector b are fixed, and q1 and q2 are the variables, represented as
 * quaternions. The quat2rpy function converts a quaternion into a euler angles vector. 
 * The A matrix is applied to the euler angles vector.
 *
 * In case the user is interested in implementing a cost function of the form
 *
 *   cost(X) = (X - mu)^T S^{-1} (X - mu)
 *
 * where, mu is a vector and S is a covariance matrix, then, A = S^{-1/2}, i.e the matrix A is the
 * square root information matrix (the inverse of the covariance).
 */
class NormalDeltaOrientation3DEulerCostFunctor
{
public:
  FUSE_MAKE_ALIGNED_OPERATOR_NEW()
  using Euler = fuse_variables::Orientation3DStamped::Euler;

  /**
   * @brief Construct a cost function instance
   *
   * @param[in] A The residual weighting matrix, most likely the square root information matrix in
   *              order (x, y, z)
   * @param[in] b The measured change between the two orientation variables (quaternion)
   * @param[in] axes The axes to be considered in the cost function
   */
  NormalDeltaOrientation3DEulerCostFunctor(
    const fuse_core::Matrix3d & A,
    const fuse_core::Vector4d & b,
    std::vector<Euler> axes)
  : A_(A),
    b_(b),
    axes_(axes)
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
    // instead of QuaternionToAngleAxis we implement QuaternionToRPY 
    // and filter the result based on the requested measurement axes 
    // ceres::QuaternionToAngleAxis(error, residuals);
    
    for (size_t i = 0; i < axes_.size(); ++i) {
      T angle;
      switch (axes_[i]) {
        case Euler::ROLL:
          {
            angle = fuse_core::getRoll(
              error[0], error[1], error[2], error[3]);
            break;
          }
        case Euler::PITCH:
          {
            angle =
              fuse_core::getPitch(
                error[0], error[1], error[2], error[3]);
            break;
          }
        case Euler::YAW:
          {
            angle =
              fuse_core::getYaw(
                error[0], error[1], error[2], error[3]);
            break;
          }
        default:
          {
            throw std::runtime_error(
                    "The provided axis specified is unknown. "
                    "I should probably be more informative here");
          }
      }
      residuals[i] = angle;
    }
    // Scale the residuals by the square root information matrix to account for the measurement
    // uncertainty.
    Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, 1>> residuals_map(residuals, A_.rows());
    residuals_map.applyOnTheLeft(A_.template cast<T>());

    return true;
  }

private:
  fuse_core::Matrix3d A_;  //!< The residual weighting matrix, most likely the square root
                           //!< information matrix
  fuse_core::Vector4d b_;  //!< The measured difference between orientation1 and orientation2
  std::vector<Euler> axes_; //!< The axes to use for the orientation error.
};

}  // namespace fuse_constraints

#endif  // FUSE_CONSTRAINTS__NORMAL_DELTA_ORIENTATION_3D_COST_FUNCTOR_HPP_
