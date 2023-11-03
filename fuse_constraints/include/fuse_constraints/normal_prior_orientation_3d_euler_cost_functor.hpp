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
#ifndef FUSE_CONSTRAINTS__NORMAL_PRIOR_ORIENTATION_3D_EULER_COST_FUNCTOR_HPP_
#define FUSE_CONSTRAINTS__NORMAL_PRIOR_ORIENTATION_3D_EULER_COST_FUNCTOR_HPP_

#include <ceres/rotation.h>
#include <Eigen/Core>

#include <vector>

#include <fuse_core/eigen.hpp>
#include <fuse_core/util.hpp>
#include <fuse_variables/orientation_3d_stamped.hpp>


namespace fuse_constraints
{

/**
 * @brief Create a prior cost function on a 3D orientation variable using Euler roll, pitch, and yaw
 *        measurements
 *
 * The functor can compute the cost of a subset of the axes, in the event that we are not interested
 * in all the Euler angles in the variable.
 *
 * So, for example, if
 * b_ = [ measured_yaw  ]
 *      [ measured_roll ]
 *
 * then the cost function is of the form:
 *
 *   cost(x) = || A * [ yaw(x)  - b_(0) ] ||^2
 *             ||     [ roll(x) - b_(1) ] ||
 *
 * where the matrix A and the vector b are fixed and (roll, pitch, yaw) are the components of the 3D
 * orientation variable.
 *
 * In case the user is interested in implementing a cost function of the form
 *
 *   cost(X) = (X - mu)^T S^{-1} (X - mu)
 *
 * where, mu is a vector and S is a covariance matrix, then, A = S^{-1/2}, i.e the matrix A is the
 * square root information matrix (the inverse of the covariance).
 */
class NormalPriorOrientation3DEulerCostFunctor
{
public:
  using Euler = fuse_variables::Orientation3DStamped::Euler;

  /**
   * @brief Construct a cost function instance
   *
   * @param[in] A The residual weighting matrix, most likely the square root information matrix. Its
   *              order must match the values in \p axes.
   * @param[in] b The orientation measurement or prior.
   * @param[in] axes The Euler angle axes for which we want to compute errors. Defaults to all axes.
   */
  NormalPriorOrientation3DEulerCostFunctor(
    const fuse_core::Matrix3d & A,
    const fuse_core::Vector4d & b,
    const std::vector<Euler> & axes = {Euler::ROLL, Euler::PITCH, Euler::YAW}) :  //NOLINT
  A_(A),
  b_(b),
  axes_(axes)
  {
  }

  /**
   * @brief Evaluate the cost function. Used by the Ceres optimization engine.
   */
  template<typename T>
  bool operator()(const T * const orientation, T * residuals) const
  {
    using fuse_variables::Orientation3DStamped;

    // Compute the delta quaternion
    // T variable[4] =
    // {
    //   orientation[0],
    //   orientation[1],
    //   orientation[2],
    //   orientation[3]
    // };

    T observation_inverse[4] =
    {
      T(b_(0)),
      T(-b_(1)),
      T(-b_(2)),
      T(-b_(3))
    };

    T difference[4];
    ceres::QuaternionProduct(observation_inverse, orientation, difference);

    T angle[3] = {T(0.0), T(0.0), T(0.0)};

    for (size_t i = 0; i < axes_.size(); ++i) {
      switch (axes_[i]) {
        case Euler::ROLL:
          {
            angle[0] = fuse_core::getRoll(
              difference[0], difference[1], difference[2], difference[3]);
            break;
          }
        case Euler::PITCH:
          {
            angle[1] = fuse_core::getPitch(
              difference[0], difference[1], difference[2], difference[3]);
            break;
          }
        case Euler::YAW:
          {
            angle[2] = fuse_core::getYaw(
              difference[0], difference[1], difference[2], difference[3]);
            break;
          }
        default:
          {
            throw std::runtime_error(
                    "The provided axis specified is unknown. "
                    "I should probably be more informative here");
          }
      }
    }

    residuals[0] = angle[0];
    residuals[1] = angle[1];
    residuals[2] = angle[2];

    Eigen::Map<Eigen::Matrix<T, 3, 1>> residuals_map(residuals, A_.rows());
    residuals_map.applyOnTheLeft(A_.template cast<T>());

    return true;
  }

private:
  fuse_core::Matrix3d A_;  //!< The residual weighting matrix, most likely the square root
                           //!< information matrix
  fuse_core::Vector4d b_;  //!< The measured 3D orientation (quaternion) value
  std::vector<Euler> axes_;  //!< The Euler angle axes that we're measuring
};

}  // namespace fuse_constraints

#endif  // FUSE_CONSTRAINTS__NORMAL_PRIOR_ORIENTATION_3D_EULER_COST_FUNCTOR_HPP_
