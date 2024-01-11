/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, Giacomo Franchini
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
#ifndef FUSE_MODELS__UNICYCLE_3D_STATE_COST_FUNCTOR_HPP_
#define FUSE_MODELS__UNICYCLE_3D_STATE_COST_FUNCTOR_HPP_

#include <fuse_models/omnidirectional_3d_predict.hpp>

#include <fuse_core/eigen.hpp>
#include <fuse_core/fuse_macros.hpp>
#include <fuse_core/util.hpp>


namespace fuse_models
{

/**
 * @brief Create a cost function for a 3D state vector
 *
 * The state vector includes the following quantities, given in this order:
 *   position (x, y, z)
 *   orientation (roll, pitch, yaw)
 *   linear velocity (x, y, z)
 *   angular velocity (roll, pitch, yaw)
 *   linear acceleration (x, y, z)
 *
 * The Ceres::NormalPrior cost function only supports a single variable. This is a convenience cost
 * function that applies a prior constraint on both the entire state vector.
 *
 * The cost function is of the form:
 *
 *             ||    [              x_t2 - proj(x_t1)             ]  ||^2
 *             ||    [              y_t2 - proj(y_t1)             ]  ||
 *             ||    [              z_t2 - proj(z_t1)             ]  ||
 *             ||    [ quat2eul(rpy_t2) - proj(quat2eul(rpy_t1))  ]  ||
 *  cost(x) =  || A *[         x_vel_t2 - proj(x_vel_t1)          ]  ||
 *             ||    [         y_vel_t2 - proj(y_vel_t1)          ]  ||
 *             ||    [         z_vel_t2 - proj(z_vel_t1)          ]  ||
 *             ||    [      roll_vel_t2 - proj(roll_vel_t1)       ]  ||
 *             ||    [      pitch_vel_t2 - proj(pitch_vel_t1)     ]  ||
 *             ||    [      yaw_vel_t2 - proj(yaw_vel_t1)         ]  ||
 *             ||    [         x_acc_t2 - proj(x_acc_t1)          ]  ||
 *             ||    [         y_acc_t2 - proj(y_acc_t1)          ]  ||
 *             ||    [         z_acc_t2 - proj(z_acc_t1)          ]  ||
 *
 * where, the matrix A is fixed, the state variables are provided at two discrete time steps, and
 * proj is a function that projects the state variables from time t1 to time t2. In case the user is
 * interested in implementing a cost function of the form
 *
 *   cost(X) = (X - mu)^T S^{-1} (X - mu)
 *
 * where, mu is a vector and S is a covariance matrix, then, A = S^{-1/2}, i.e the matrix A is the
 * square root information matrix (the inverse of the covariance).
 */
class Omnidirectional3DStateCostFunctor
{
public:
  FUSE_MAKE_ALIGNED_OPERATOR_NEW()

  /**
   * @brief Construct a cost function instance
   *
   * @param[in] dt The time delta across which to generate the kinematic model cost
   * @param[in] A The residual weighting matrix, most likely the square root information matrix in
   *              order (x, y, z, qx, qy, qz, qw, x_vel, y_vel, z_vel, roll_vel, pitch_vel, yaw_vel, 
   *                      x_acc, y_acc, z_acc)
   */
  Omnidirectional3DStateCostFunctor(const double dt, const fuse_core::Matrix15d & A);

  /**
   * @brief Evaluate the cost function. Used by the Ceres optimization engine.
   * @param[in] position1    - First position (array with x at index 0, y at index 1, z at index 2)
   * @param[in] orientation1 - First orientation (array with w at index 0, x at index 1, y at index 2, z at index 3) check this order
   * @param[in] vel_linear1  - First linear velocity (array with x at index 0, y at index 1, z at index 2)
   * @param[in] vel_angular1 - First angular velocity (array with vroll at index 0, vpitch at index 1, vyaw at index 2)
   * @param[in] acc_linear1  - First linear acceleration (array with x at index 0, y at index 1, z at index 2)
   * @param[in] dt - The time delta across which to predict the state
   * @param[out] position2 - Second position (array with x at index 0, y at index 1, z at index 2)
   * @param[out] orientation2 - Second orientation (array with w at index 0, x at index 1, y at index 2, z at index 3) check this order
   * @param[out] vel_linear2  - Second velocity (array with x at index 0, y at index 1, z at index 2)
   * @param[out] vel_angular2 - Second angular velocity (array with vroll at index 0, vpitch at index 1, vyaw at index 2)
   * @param[out] acc_linear2  - Second linear acceleration (array with x at index 0, y at index 1, z at index 2)
   */
  template<typename T>
  bool operator()(
    const T * const position1,
    const T * const orientation1,
    const T * const vel_linear1,
    const T * const vel_angular1,
    const T * const acc_linear1,
    const T * const position2,
    const T * const orientation2,
    const T * const vel_linear2,
    const T * const vel_angular2,
    const T * const acc_linear2,
    T * residual) const;

private:
  double dt_;
  fuse_core::Matrix15d A_;  //!< The residual weighting matrix, most likely the square root
                           //!< information matrix
};

Omnidirectional3DStateCostFunctor::Omnidirectional3DStateCostFunctor(
  const double dt,
  const fuse_core::Matrix15d & A)
: dt_(dt),
  A_(A)
{
}

template<typename T>
bool Omnidirectional3DStateCostFunctor::operator()(
  const T * const position1,
  const T * const orientation1,
  const T * const vel_linear1,
  const T * const vel_angular1,
  const T * const acc_linear1,
  const T * const position2,
  const T * const orientation2,
  const T * const vel_linear2,
  const T * const vel_angular2,
  const T * const acc_linear2,
  T * residual) const
{
  T position_pred[3];
  T orientation_pred[3];
  T vel_linear_pred[3];
  T vel_angular_pred[3];
  T acc_linear_pred[3];

  // Convert orientation variables from quaternion to roll-pitch-yaw
  const T orientation1_rpy[3] {
    fuse_core::getRoll(orientation1[0], orientation1[1], orientation1[2], orientation1[3]),
    fuse_core::getPitch(orientation1[0], orientation1[1], orientation1[2], orientation1[3]),
    fuse_core::getYaw(orientation1[0], orientation1[1], orientation1[2], orientation1[3])
  };
  const T orientation2_rpy[3] {
    fuse_core::getRoll(orientation2[0], orientation2[1], orientation2[2], orientation2[3]),
    fuse_core::getPitch(orientation2[0], orientation2[1], orientation2[2], orientation2[3]),
    fuse_core::getYaw(orientation2[0], orientation2[1], orientation2[2], orientation2[3])
  };

  predict(
    position1,
    orientation1_rpy,
    vel_linear1,
    vel_angular1,
    acc_linear1,
    T(dt_),
    position_pred,
    orientation_pred,
    vel_linear_pred,
    vel_angular_pred,
    acc_linear_pred);
  
  Eigen::Map<Eigen::Matrix<T, 15, 1>> residuals_map(residual);
  residuals_map(0) = position2[0] - position_pred[0];
  residuals_map(1) = position2[1] - position_pred[1];
  residuals_map(2) = position2[2] - position_pred[2];
  residuals_map(3) = orientation2_rpy[0] - orientation_pred[0];
  residuals_map(4) = orientation2_rpy[1] - orientation_pred[1];
  residuals_map(5) = orientation2_rpy[2] - orientation_pred[2];
  residuals_map(6) = vel_linear2[0] - vel_linear_pred[0];
  residuals_map(7) = vel_linear2[1] - vel_linear_pred[1];
  residuals_map(8) = vel_linear2[2] - vel_linear_pred[2];
  residuals_map(9) = vel_angular2[0] - vel_angular_pred[0];
  residuals_map(10) = vel_angular2[1] - vel_angular_pred[1];
  residuals_map(11) = vel_angular2[2] - vel_angular_pred[2];
  residuals_map(12) = acc_linear2[0] - acc_linear_pred[0];
  residuals_map(13) = acc_linear2[1] - acc_linear_pred[1];
  residuals_map(14) = acc_linear2[2] - acc_linear_pred[2];

  fuse_core::wrapAngle2D(residuals_map(3));
  fuse_core::wrapAngle2D(residuals_map(4));
  fuse_core::wrapAngle2D(residuals_map(5));

  // Scale the residuals by the square root information matrix to account for
  // the measurement uncertainty.
  residuals_map.applyOnTheLeft(A_.template cast<T>());

  return true;
}

}  // namespace fuse_models

#endif  // FUSE_MODELS__UNICYCLE_3D_STATE_COST_FUNCTOR_HPP_
