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
#ifndef FUSE_MODELS__UNICYCLE_3D_PREDICT_HPP_
#define FUSE_MODELS__UNICYCLE_3D_PREDICT_HPP_

#include <ceres/jet.h>
#include <Eigen/Core>
#include <array>

#include <fuse_core/util.hpp>
#include <fuse_core/eigen.hpp>

namespace fuse_models
{
/**
 * @brief Given a state and time delta, predicts a new state + computes the Jacobians
 * @param[in] position            - First position
 * @param[in] orientation         - First orientation
 * @param[in] vel_linear          - First linear velocity
 * @param[in] vel_angular         - First angular velocity
 * @param[in] acc_linear          - First linear acceleration
 * @param[in] dt - The time delta across which to predict the state
 * @param[out] position_out       - Second position
 * @param[out] orientation_out    - Second orientation
 * @param[out] vel_linear_out     - Second linear velocity
 * @param[out] vel_angular_out    - Second angular velocity
 * @param[out] acc_linear_out     - Second linear acceleration
 * @param[out] state_tf_jacobian  - Jacobian of the motion model
 */
template <typename T>
inline void predict(
  const Eigen::Matrix<T, 3, 1> & position,
  const Eigen::Quaternion<T> & orientation,
  const Eigen::Matrix<T, 3, 1> & vel_linear,
  const Eigen::Matrix<T, 3, 1> & vel_angular,
  const Eigen::Matrix<T, 3, 1> & acc_linear,
  const T dt,
  Eigen::Matrix<T, 3, 1> & position_out,
  Eigen::Quaternion<T> & orientation_out,
  Eigen::Matrix<T, 3, 1> & vel_linear_out,
  Eigen::Matrix<T, 3, 1> & vel_angular_out,
  Eigen::Matrix<T, 3, 1> & acc_linear_out,
  Eigen::Matrix<T, 15, 15>* state_tf_jacobian = nullptr)
{
  // Convert quaternion to eigen
  T roll = fuse_core::getRoll(orientation.w(), orientation.x(), orientation.y(), orientation.z());
  T pitch = fuse_core::getPitch(orientation.w(), orientation.x(), orientation.y(), orientation.z());
  T yaw = fuse_core::getYaw(orientation.w(), orientation.x(), orientation.y(), orientation.z());
  // 3D material point projection model which matches the one used by r_l.
  T sr = ceres::sin(roll);
  T cr = ceres::cos(roll);
  T sp = ceres::sin(pitch);
  T cp = ceres::cos(pitch);
  T sy = ceres::sin(yaw);  // Should probably be sin((yaw1 + yaw2) / 2), but r_l uses this model
  T cy = ceres::cos(yaw);
  T cpi = 1.0 / cp;
  T tp = ceres::tan(pitch);

  Eigen::Matrix<T, 3, 3> tf_pos, tf_rot, tf_vel, tf_acc;
  Eigen::Matrix<T, 15, 15> state_tf;

  tf_pos(0, 0) = (cy * cp) * dt;
  tf_pos(0, 1) = (cy * sp * sr - sy * cr) * dt;
  tf_pos(0, 2) = (cy * sp * cr + sy * sr) * dt;
  tf_pos(1, 0) = (sy * cp) * dt;
  tf_pos(1, 1) = (sy * sp * sr + cy * cr) * dt;
  tf_pos(1, 2) = (sy * sp * cr - cy * sr) * dt;
  tf_pos(2, 0) = (-sp) * dt;
  tf_pos(2, 1) = (cp * sr) * dt;
  tf_pos(2, 2) = (cp * cr) * dt;

  tf_rot(0, 0) = dt;
  tf_rot(0, 1) = sr * sp * cpi * dt;
  tf_rot(0, 2) = cr * sp * cpi * dt;
  tf_rot(1, 0) = T(0);
  tf_rot(1, 1) = cr * dt;
  tf_rot(1, 2) = -sr * dt;
  tf_rot(2, 0) = T(0);
  tf_rot(2, 1) = sr * cpi * dt;
  tf_rot(2, 2) = cr * cpi * dt;

  // Compute the transfer function matrix
  state_tf.setZero();
  // position
  // state_tf.block<T, 3, 3>(0, 0) = Eigen::Matrix<T, 3, 3>::Identity();
  // state_tf.block<3, 3>(0, 6) = tf_pos;
  // state_tf.block<3, 3>(0, 12) = 0.5 * tf_pos * dt;
  // // orientation
  // state_tf.block<3, 3>(3, 3) = Eigen::Matrix<T, 3, 3>::Identity();
  // state_tf.block<3, 3>(3, 9) = tf_rot;
  // // linear velocity
  // state_tf.block<3, 3>(6, 6) = Eigen::Matrix<T, 3, 3>::Identity();
  // state_tf.block<3, 3>(6, 12) = dt * Eigen::Matrix<T, 3, 3>::Identity();
  // // angular velocity
  // state_tf.block<3, 3>(9, 9) = Eigen::Matrix<T, 3, 3>::Identity();
  // // linear acceleration
  // state_tf.block<3, 3>(12, 12) = Eigen::Matrix<T, 3, 3>::Identity();

  if (state_tf_jacobian)
  {
    // TODO: compute the jacobian of the motion model transfer function
    T x_coeff = T(0.0);
    T y_coeff = T(0.0);
    T z_coeff = T(0.0);
    T one_half_at_squared = 0.5 * dt * dt;

    y_coeff = cy * sp * cr + sy * sr;
    z_coeff = -cy * sp * sr + sy * cr;
    T dFx_dR = (y_coeff * vel_linear.y() + z_coeff * vel_linear.z()) * dt +
      (y_coeff * acc_linear.y() + z_coeff * acc_linear.z()) * one_half_at_squared;
    T dFR_dR = 1.0 + (cr * tp * vel_angular.y() - sr * tp * vel_angular.z()) * dt;

    x_coeff = -cy * sp;
    y_coeff = cy * cp * sr;
    z_coeff = cy * cp * cr;
    T dFx_dP =
      (x_coeff * vel_linear.x() + y_coeff * vel_linear.y() + z_coeff * vel_linear.z()) * dt +
      (x_coeff * acc_linear.x() + y_coeff * acc_linear.y() + z_coeff * acc_linear.z()) *
      one_half_at_squared;
    T dFR_dP =
      (cpi * cpi * sr * vel_angular.y() + cpi * cpi * cr * vel_angular.z()) * dt;

    x_coeff = -sy * cp;
    y_coeff = -sy * sp * sr - cy * cr;
    z_coeff = -sy * sp * cr + cy * sr;
    T dFx_dY =
      (x_coeff * vel_linear.x() + y_coeff * vel_linear.y() + z_coeff * vel_linear.z()) * dt +
      (x_coeff * acc_linear.x() + y_coeff * acc_linear.y() + z_coeff * acc_linear.z()) *
      one_half_at_squared;

    y_coeff = sy * sp * cr - cy * sr;
    z_coeff = -sy * sp * sr - cy * cr;
    T dFy_dR = (y_coeff * vel_linear.y() + z_coeff * vel_linear.z()) * dt +
      (y_coeff * acc_linear.y() + z_coeff * acc_linear.z()) * one_half_at_squared;
    T dFP_dR = (-sr * vel_angular.y() - cr * vel_angular.z()) * dt;

    x_coeff = -sy * sp;
    y_coeff = sy * cp * sr;
    z_coeff = sy * cp * cr;
    T dFy_dP =
      (x_coeff * vel_linear.x() + y_coeff * vel_linear.y() + z_coeff * vel_linear.z()) * dt +
      (x_coeff * acc_linear.x() + y_coeff * acc_linear.y() + z_coeff * acc_linear.z()) *
      one_half_at_squared;

    x_coeff = cy * cp;
    y_coeff = cy * sp * sr - sy * cr;
    z_coeff = cy * sp * cr + sy * sr;
    T dFy_dY =
      (x_coeff * vel_linear.x() + y_coeff * vel_linear.y() + z_coeff * vel_linear.z()) * dt +
      (x_coeff * acc_linear.x() + y_coeff * acc_linear.y() + z_coeff * acc_linear.z()) *
      one_half_at_squared;

    y_coeff = cp * cr;
    z_coeff = -cp * sr;
    T dFz_dR = (y_coeff * vel_linear.y() + z_coeff * vel_linear.z()) * dt +
      (y_coeff * acc_linear.y() + z_coeff * acc_linear.z()) * one_half_at_squared;
    T dFY_dR = (cr * cpi * vel_angular.y() - sr * cpi * vel_angular.z()) * dt;

    x_coeff = -cp;
    y_coeff = -sp * sr;
    z_coeff = -sp * cr;
    T dFz_dP =
      (x_coeff * vel_linear.x() + y_coeff * vel_linear.y() + z_coeff * vel_linear.z()) * dt +
      (x_coeff * acc_linear.x() + y_coeff * acc_linear.y() + z_coeff * acc_linear.z()) *
      one_half_at_squared;
    T dFY_dP =
      (sr * tp * cpi * vel_angular.y() + cr * tp * cpi * vel_angular.z()) * dt;

    *state_tf_jacobian = state_tf;
    (*state_tf_jacobian)(0, 3) = dFx_dR;
    (*state_tf_jacobian)(0, 4) = dFx_dP;
    (*state_tf_jacobian)(0, 5) = dFx_dY;
    (*state_tf_jacobian)(1, 3) = dFy_dR;
    (*state_tf_jacobian)(1, 4) = dFy_dP;
    (*state_tf_jacobian)(1, 5) = dFy_dY;
    (*state_tf_jacobian)(2, 3) = dFz_dR;
    (*state_tf_jacobian)(2, 4) = dFz_dP;
    (*state_tf_jacobian)(3, 3) = dFR_dR;
    (*state_tf_jacobian)(3, 4) = dFR_dP;
    (*state_tf_jacobian)(4, 3) = dFP_dR;
    (*state_tf_jacobian)(5, 3) = dFY_dR;
    (*state_tf_jacobian)(5, 4) = dFY_dP;
  }

  // Predict the new state
  Eigen::Matrix<T, 15, 1> state;
  state.head(3)      = position;
  state.segment(3,3) = Eigen::Matrix<T, 3, 1>(roll, pitch, yaw);
  state.segment(6,3) = vel_linear;
  state.segment(9,3) = vel_angular;
  state.tail(3)      = acc_linear;
  
  state = state_tf * state;

  // Convert back
  position_out    = state.head(3);
  fuse_core::wrapAngle2D(state(5));
  fuse_core::wrapAngle2D(state(4));
  fuse_core::wrapAngle2D(state(3));
  Eigen::Quaterniond q =
  Eigen::AngleAxis<T>(state(5), Eigen::Matrix<T, 3, 1>::UnitZ()) *
  Eigen::AngleAxis<T>(state(4), Eigen::Matrix<T, 3, 1>::UnitY()) *
  Eigen::AngleAxis<T>(state(3), Eigen::Matrix<T, 3, 1>::UnitX());
  orientation_out = q;
  vel_linear_out  = state.segment(6,3);
  vel_angular_out = state.segment(9,3);
  acc_linear_out  = state.tail(3);
}

/**
 * @brief Given a state and time delta, predicts a new state + computes the Jacobians
 * @param[in] position            - First position
 * @param[in] orientation         - First orientation
 * @param[in] vel_linear          - First linear velocity
 * @param[in] vel_angular         - First angular velocity
 * @param[in] acc_linear          - First linear acceleration
 * @param[in] dt - The time delta across which to predict the state
 * @param[out] position_out       - Second position
 * @param[out] orientation_out    - Second orientation
 * @param[out] vel_linear_out     - Second linear velocity
 * @param[out] vel_angular_out    - Second angular velocity
 * @param[out] acc_linear_out     - Second linear acceleration
 * @param[out] state_tf_jacobian  - Jacobian of the motion model
 */
inline void predict(
  const fuse_core::Vector3d & position,
  // const fuse_core::Vector3d & orientation,
  const fuse_core::Quaternion & orientation,
  const fuse_core::Vector3d & vel_linear,
  const fuse_core::Vector3d & vel_angular,
  const fuse_core::Vector3d & acc_linear,
  const double dt,
  fuse_core::Vector3d & position_out,
  // fuse_core::Vector3d & orientation_out,
  fuse_core::Quaternion & orientation_out,
  fuse_core::Vector3d & vel_linear_out,
  fuse_core::Vector3d & vel_angular_out,
  fuse_core::Vector3d & acc_linear_out,
  fuse_core::Matrix15d* state_tf_jacobian = nullptr)
{
  // Convert quaternion to eigen
  double roll = fuse_core::getRoll(orientation.w(), orientation.x(), orientation.y(), orientation.z());
  double pitch = fuse_core::getPitch(orientation.w(), orientation.x(), orientation.y(), orientation.z());
  double yaw = fuse_core::getYaw(orientation.w(), orientation.x(), orientation.y(), orientation.z());
  // 3D material point projection model which matches the one used by r_l.
  double sr = ceres::sin(roll);
  double cr = ceres::cos(roll);
  double sp = ceres::sin(pitch);
  double cp = ceres::cos(pitch);
  double sy = ceres::sin(yaw);  // Should probably be sin((yaw1 + yaw2) / 2), but r_l uses this model
  double cy = ceres::cos(yaw);
  double cpi = 1.0 / cp;
  double tp = ceres::tan(pitch);

  fuse_core::Matrix3d tf_pos, tf_rot, tf_vel, tf_acc;
  fuse_core::Matrix15d state_tf;

  tf_pos(0, 0) = (cy * cp) * dt;
  tf_pos(0, 1) = (cy * sp * sr - sy * cr) * dt;
  tf_pos(0, 2) = (cy * sp * cr + sy * sr) * dt;
  tf_pos(1, 0) = (sy * cp) * dt;
  tf_pos(1, 1) = (sy * sp * sr + cy * cr) * dt;
  tf_pos(1, 2) = (sy * sp * cr - cy * sr) * dt;
  tf_pos(2, 0) = (-sp) * dt;
  tf_pos(2, 1) = (cp * sr) * dt;
  tf_pos(2, 2) = (cp * cr) * dt;

  tf_rot(0, 0) = dt;
  tf_rot(0, 1) = sr * sp * cpi * dt;
  tf_rot(0, 2) = cr * sp * cpi * dt;
  tf_rot(1, 0) = 0;
  tf_rot(1, 1) = cr * dt;
  tf_rot(1, 2) = -sr * dt;
  tf_rot(2, 0) = 0;
  tf_rot(2, 1) = sr * cpi * dt;
  tf_rot(2, 2) = cr * cpi * dt;

  // Compute the transfer function matrix
  state_tf.setZero();
  // position
  state_tf.block<3, 3>(0, 0) = fuse_core::Matrix3d::Identity();
  state_tf.block<3, 3>(0, 6) = tf_pos;
  state_tf.block<3, 3>(0, 12) = 0.5 * tf_pos * dt;
  // orientation
  state_tf.block<3, 3>(3, 3) = fuse_core::Matrix3d::Identity();
  state_tf.block<3, 3>(3, 9) = tf_rot;
  // linear velocity
  state_tf.block<3, 3>(6, 6) = fuse_core::Matrix3d::Identity();
  state_tf.block<3, 3>(6, 12) = dt * fuse_core::Matrix3d::Identity();
  // angular velocity
  state_tf.block<3, 3>(9, 9) = fuse_core::Matrix3d::Identity();
  // linear acceleration
  state_tf.block<3, 3>(12, 12) = fuse_core::Matrix3d::Identity();

  if (state_tf_jacobian)
  {
    // TODO: compute the jacobian of the motion model transfer function
    double x_coeff = 0.0;
    double y_coeff = 0.0;
    double z_coeff = 0.0;
    double one_half_at_squared = 0.5 * dt * dt;

    y_coeff = cy * sp * cr + sy * sr;
    z_coeff = -cy * sp * sr + sy * cr;
    double dFx_dR = (y_coeff * vel_linear.y() + z_coeff * vel_linear.z()) * dt +
      (y_coeff * acc_linear.y() + z_coeff * acc_linear.z()) * one_half_at_squared;
    double dFR_dR = 1.0 + (cr * tp * vel_angular.y() - sr * tp * vel_angular.z()) * dt;

    x_coeff = -cy * sp;
    y_coeff = cy * cp * sr;
    z_coeff = cy * cp * cr;
    double dFx_dP =
      (x_coeff * vel_linear.x() + y_coeff * vel_linear.y() + z_coeff * vel_linear.z()) * dt +
      (x_coeff * acc_linear.x() + y_coeff * acc_linear.y() + z_coeff * acc_linear.z()) *
      one_half_at_squared;
    double dFR_dP =
      (cpi * cpi * sr * vel_angular.y() + cpi * cpi * cr * vel_angular.z()) * dt;

    x_coeff = -sy * cp;
    y_coeff = -sy * sp * sr - cy * cr;
    z_coeff = -sy * sp * cr + cy * sr;
    double dFx_dY =
      (x_coeff * vel_linear.x() + y_coeff * vel_linear.y() + z_coeff * vel_linear.z()) * dt +
      (x_coeff * acc_linear.x() + y_coeff * acc_linear.y() + z_coeff * acc_linear.z()) *
      one_half_at_squared;

    y_coeff = sy * sp * cr - cy * sr;
    z_coeff = -sy * sp * sr - cy * cr;
    double dFy_dR = (y_coeff * vel_linear.y() + z_coeff * vel_linear.z()) * dt +
      (y_coeff * acc_linear.y() + z_coeff * acc_linear.z()) * one_half_at_squared;
    double dFP_dR = (-sr * vel_angular.y() - cr * vel_angular.z()) * dt;

    x_coeff = -sy * sp;
    y_coeff = sy * cp * sr;
    z_coeff = sy * cp * cr;
    double dFy_dP =
      (x_coeff * vel_linear.x() + y_coeff * vel_linear.y() + z_coeff * vel_linear.z()) * dt +
      (x_coeff * acc_linear.x() + y_coeff * acc_linear.y() + z_coeff * acc_linear.z()) *
      one_half_at_squared;

    x_coeff = cy * cp;
    y_coeff = cy * sp * sr - sy * cr;
    z_coeff = cy * sp * cr + sy * sr;
    double dFy_dY =
      (x_coeff * vel_linear.x() + y_coeff * vel_linear.y() + z_coeff * vel_linear.z()) * dt +
      (x_coeff * acc_linear.x() + y_coeff * acc_linear.y() + z_coeff * acc_linear.z()) *
      one_half_at_squared;

    y_coeff = cp * cr;
    z_coeff = -cp * sr;
    double dFz_dR = (y_coeff * vel_linear.y() + z_coeff * vel_linear.z()) * dt +
      (y_coeff * acc_linear.y() + z_coeff * acc_linear.z()) * one_half_at_squared;
    double dFY_dR = (cr * cpi * vel_angular.y() - sr * cpi * vel_angular.z()) * dt;

    x_coeff = -cp;
    y_coeff = -sp * sr;
    z_coeff = -sp * cr;
    double dFz_dP =
      (x_coeff * vel_linear.x() + y_coeff * vel_linear.y() + z_coeff * vel_linear.z()) * dt +
      (x_coeff * acc_linear.x() + y_coeff * acc_linear.y() + z_coeff * acc_linear.z()) *
      one_half_at_squared;
    double dFY_dP =
      (sr * tp * cpi * vel_angular.y() + cr * tp * cpi * vel_angular.z()) * dt;

    *state_tf_jacobian = state_tf;
    (*state_tf_jacobian)(0, 3) = dFx_dR;
    (*state_tf_jacobian)(0, 4) = dFx_dP;
    (*state_tf_jacobian)(0, 5) = dFx_dY;
    (*state_tf_jacobian)(1, 3) = dFy_dR;
    (*state_tf_jacobian)(1, 4) = dFy_dP;
    (*state_tf_jacobian)(1, 5) = dFy_dY;
    (*state_tf_jacobian)(2, 3) = dFz_dR;
    (*state_tf_jacobian)(2, 4) = dFz_dP;
    (*state_tf_jacobian)(3, 3) = dFR_dR;
    (*state_tf_jacobian)(3, 4) = dFR_dP;
    (*state_tf_jacobian)(4, 3) = dFP_dR;
    (*state_tf_jacobian)(5, 3) = dFY_dR;
    (*state_tf_jacobian)(5, 4) = dFY_dP;
  }

  // Predict the new state
  fuse_core::Vector15d state;
  state.head(3)      = position;
  state.segment(3,3) = fuse_core::Vector3d(roll, pitch, yaw);
  state.segment(6,3) = vel_linear;
  state.segment(9,3) = vel_angular;
  state.tail(3)      = acc_linear;
  
  state = state_tf * state;

  // Convert back
  position_out    = state.head(3);
  fuse_core::wrapAngle2D(state(5));
  fuse_core::wrapAngle2D(state(4));
  fuse_core::wrapAngle2D(state(3));
  Eigen::Quaterniond q =
  Eigen::AngleAxisd(state(5), Eigen::Vector3d::UnitZ()) *
  Eigen::AngleAxisd(state(4), Eigen::Vector3d::UnitY()) *
  Eigen::AngleAxisd(state(3), Eigen::Vector3d::UnitX());
  orientation_out = q;
  vel_linear_out  = state.segment(6,3);
  vel_angular_out = state.segment(9,3);
  acc_linear_out  = state.tail(3);
}

template<typename T>
inline void predict(
  const T * const position1,
  const T * const orientation1,
  const T * const vel_linear1,
  const T * const vel_angular1,
  const T * const acc_linear1,
  const T dt,
  T * const position2,
  T * const orientation2,
  T * const vel_linear2,
  T * const vel_angular2,
  T * const acc_linear2)
{
  // Creates some Eigen maps and pass them to the predict function
  // First test just to try if this works, create new eigen vector and pass these to the predict function
  Eigen::Matrix<T, 3, 1> position_in(position1[0], position1[1], position1[2]);
  Eigen::Quaternion<T> orientation_in(orientation1[3], orientation1[0], orientation1[1], orientation1[2]);
  Eigen::Matrix<T, 3, 1> vel_linear_in(vel_linear1[0], vel_linear1[1], vel_linear1[2]);
  Eigen::Matrix<T, 3, 1> vel_angular_in(vel_angular1[0], vel_angular1[1], vel_angular1[2]);
  Eigen::Matrix<T, 3, 1> acc_linear_in(acc_linear1[0], acc_linear1[1], acc_linear1[2]);
  Eigen::Matrix<T, 3, 1> position_out, vel_linear_out, vel_angular_out, acc_linear_out;
  Eigen::Quaternion<T> orientation_out;
  predict(
    position_in, 
    orientation_in, 
    vel_linear_in, 
    vel_angular_in, 
    acc_linear_in, 
    dt,
    position_out, 
    orientation_out, 
    vel_linear_out, 
    vel_angular_out, 
    acc_linear_out);
  
  // Convert back
  position2[0] = position_out.x();
  position2[1] = position_out.y();
  position2[2] = position_out.z();
  orientation2[0] = orientation_out.x();
  orientation2[1] = orientation_out.y();
  orientation2[2] = orientation_out.z();
  orientation2[3] = orientation_out.w();
  vel_linear2[0] = vel_linear_out.x();
  vel_linear2[1] = vel_linear_out.y();
  vel_linear2[2] = vel_linear_out.z();
  vel_angular2[0] = vel_angular_out.x();
  vel_angular2[1] = vel_angular_out.y();
  vel_angular2[2] = vel_angular_out.z();
  acc_linear2[0] = acc_linear_out.x();
  acc_linear2[1] = acc_linear_out.y();
  acc_linear2[2] = acc_linear_out.z();
}

// /**
//  * @brief Given a state and time delta, predicts a new state
//  * @param[in] position1_x   - First X position
//  * @param[in] position1_y   - First Y position
//  * @param[in] position1_z   - First Z position
//  * @param[in] roll1         - First roll
//  * @param[in] pitch1        - First pitch
//  * @param[in] yaw1          - First yaw
//  * @param[in] vel_linear1_x - First X velocity
//  * @param[in] vel_linear1_y - First Y velocity
//  * @param[in] vel_linear1_z - First Z velocity
//  * @param[in] vel_roll1     - First roll velocity
//  * @param[in] vel_pitch1    - First pitch velocity
//  * @param[in] vel_yaw1      - First yaw velocity
//  * @param[in] acc_linear1_x - First X acceleration
//  * @param[in] acc_linear1_y - First Y acceleration
//  * @param[in] acc_linear1_z - First Z acceleration
//  * @param[in] dt - The time delta across which to predict the state
//  * @param[out] position2_x   - Second X position
//  * @param[out] position2_y   - Second Y position
//  * @param[out] position2_z   - Second Z position
//  * @param[out] roll2         - Second roll
//  * @param[out] pitch2        - Second pitch
//  * @param[out] yaw2          - Second orientation
//  * @param[out] vel_linear2_x - Second X velocity
//  * @param[out] vel_linear2_y - Second Y velocity
//  * @param[out] vel_linear2_z - Second Z velocity
//  * @param[out] vel_roll2     - Second roll velocity
//  * @param[out] vel_pitch2    - Second pitch velocity
//  * @param[out] vel_yaw2      - Second yaw velocity
//  * @param[out] acc_linear2_x - Second X acceleration
//  * @param[out] acc_linear2_y - Second Y acceleration
//  * @param[out] acc_linear2_z - Second Z acceleration
//  * @param[out] jacobians - Jacobians wrt the state
//  */
// inline void predict(
//   const double position1_x,
//   const double position1_y,
//   const double position1_z,
//   // const double roll1,
//   // const double pitch1,
//   // const double yaw1,
//   const double q1_x,
//   const double q1_y,
//   const double q1_z,
//   const double q1_w,
//   const double vel_linear1_x,
//   const double vel_linear1_y,
//   const double vel_linear1_z,
//   const double vel_roll1,
//   const double vel_pitch1,
//   const double vel_yaw1,
//   const double acc_linear1_x,
//   const double acc_linear1_y,
//   const double acc_linear1_z,
//   const double dt,
//   double & position2_x,
//   double & position2_y,
//   double & position2_z,
//   // double & roll2,
//   // double & pitch2,
//   // double & yaw2,
//   const double q2_x,
//   const double q2_y,
//   const double q2_z,
//   const double q2_w,
//   double & vel_linear2_x,
//   double & vel_linear2_y,
//   double & vel_linear2_z,
//   double & vel_roll2,
//   double & vel_pitch2,
//   double & vel_yaw2,
//   double & acc_linear2_x,
//   double & acc_linear2_y,
//   double & acc_linear2_z,
//   double ** jacobians)
// {
//   fuse_core::Vector3d position1(position1_x, position1_y, position1_z);
//   // fuse_core::Vector3d orientation1(roll1, pitch1, yaw1);
//   fuse_core::Quaternion orientation1(q1_w, q1_x, q1_y, q1_z);  
//   fuse_core::Vector3d vel_linear1(vel_linear1_x, vel_linear1_y, vel_linear1_z);
//   fuse_core::Vector3d vel_angular1(vel_roll1, vel_pitch1, vel_yaw1);
//   fuse_core::Vector3d acc_linear1(acc_linear1_x, acc_linear1_y, acc_linear1_z);
//   fuse_core::Vector3d position2, orientation2, vel_linear2, vel_angular2, acc_linear2;
//   fuse_core::Quaternion orientation2;
//   fuse_core::Matrix15d* jacobian; 

//   predict(position1, orientation1, vel_linear1, vel_angular1, acc_linear1, dt,
//       position2, orientation2, vel_linear2, vel_angular2, acc_linear2, jacobian);
  
//   position2_x = position2.x();
//   position2_y = position2.y();
//   position2_z = position2.z();
//   // roll2 = orientation2.x();
//   // pitch2 = orientation2.y();
//   // yaw2 = orientation2.z();
//   q2_x = orientation2.x();
//   q2_y = orientation2.y();
//   q2_z = orientation2.z();
//   q2_w = orientation2.w();
//   vel_linear2_x = vel_linear2.x();
//   vel_linear2_y = vel_linear2.y();
//   vel_linear2_z = vel_linear2.z();
//   vel_roll2 = vel_angular2.x();
//   vel_pitch2 = vel_angular2.y();
//   vel_yaw2 = vel_angular2.z();
//   acc_linear2_x = acc_linear2.x();
//   acc_linear2_y = acc_linear2.y();
//   acc_linear2_z = acc_linear2.z();

//   if (jacobians) 
//   {
//     // Jacobian wrt position1
//     if (jacobians[0]) {
//       Eigen::Map<fuse_core::Matrix<double, 15, 3>> j_temp(jacobians[0]);
//       j_temp = jacobian->block<15, 3>(0, 0);
//     }
//     // Jacobian wrt orientation1
//     // if (jacobians[1]) {
//     //   Eigen::Map<fuse_core::Matrix<double, 15, 3>> j_temp(jacobians[1]);
//     //   j_temp = jacobian->block<15, 3>(0, 3);
//     // }
//     if (jacobians[1]) {
//       Eigen::Map<fuse_core::Matrix<double, 15, 3>> j_temp(jacobians[1]);
//       j_temp = jacobian->block<15, 3>(0, 3);
//     }
//     // Jacobian wrt vel_linear1
//     if (jacobians[2]) {
//       Eigen::Map<fuse_core::Matrix<double, 15, 3>> j_temp(jacobians[2]);
//       j_temp = jacobian->block<15, 3>(0, 6);
//     }
//     // Jacobian wrt vel_angular1
//     if (jacobians[3]) {
//       Eigen::Map<fuse_core::Matrix<double, 15, 3>> j_temp(jacobians[3]);
//       j_temp = jacobian->block<15, 3>(0, 9);
//     }
//     // Jacobian wrt acc_linear1
//     if (jacobians[4]) {
//       Eigen::Map<fuse_core::Matrix<double, 15, 3>> j_temp(jacobians[4]);
//       j_temp = jacobian->block<15, 3>(0, 12);
//     }
//   }
// }
}  // namespace fuse_models

#endif  // FUSE_MODELS__UNICYCLE_3D_PREDICT_HPP_
