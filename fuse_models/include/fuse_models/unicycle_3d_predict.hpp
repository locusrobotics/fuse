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
  using Jet = ceres::Jet<double, 32>;
// /**
//  * @brief Given a state and time delta, predicts a new state - 
//  *        inner templated version for ceres autodiff
//  * @param[in] position1      - First position
//  * @param[in] orientation1   - First orientation
//  * @param[in] vel_linear1    - First linear velocity
//  * @param[in] vel_angular1   - First angular velocity
//  * @param[in] acc_linear1    - First linear acceleration
//  * @param[in] dt - The time delta across which to predict the state
//  * @param[out] position2     - Second position
//  * @param[out] orientation2  - Second orientation
//  * @param[out] vel_linear2   - Second linear velocity
//  * @param[out] vel_angular2  - Second angular velocity
//  * @param[out] acc_linear2   - Second linear acceleration
//  */
// template <typename T>
// inline void predict_eigen(
//   // const Eigen::Ref<const Eigen::Matrix<T, 3, 1>>& position1,
//   // const Eigen::Ref<const Eigen::Quaternion<T>>& orientation1,
//   // const Eigen::Ref<const Eigen::Matrix<T, 3, 1>>& vel_linear1,
//   // const Eigen::Ref<const Eigen::Matrix<T, 3, 1>>& vel_angular1,
//   // const Eigen::Ref<const Eigen::Matrix<T, 3, 1>>& acc_linear1,
//   // const T dt,
//   // Eigen::Ref<Eigen::Matrix<T, 3, 1>> position2,
//   // Eigen::Ref<Eigen::Quaternion<T>> orientation2,
//   // Eigen::Ref<Eigen::Matrix<T, 3, 1>> vel_linear2,
//   // Eigen::Ref<Eigen::Matrix<T, 3, 1>> vel_angular2,
//   // Eigen::Ref<Eigen::Matrix<T, 3, 1>> acc_linear2)
//   const Eigen::Matrix<T, 3, 1>& position1,
//   const Eigen::Quaternion<T>& orientation1,
//   const Eigen::Matrix<T, 3, 1>& vel_linear1,
//   const Eigen::Matrix<T, 3, 1>& vel_angular1,
//   const Eigen::Matrix<T, 3, 1>& acc_linear1,
//   const T dt,
//   Eigen::Matrix<T, 3, 1> position2,
//   Eigen::Quaternion<T> orientation2,
//   Eigen::Matrix<T, 3, 1> vel_linear2,
//   Eigen::Matrix<T, 3, 1> vel_angular2,
//   Eigen::Matrix<T, 3, 1> acc_linear2)
// {
//   // Convert quaternion to eigen
//   Eigen::Vector<T, 3> rpy(
//     fuse_core::getRoll(orientation1.w(), orientation1.x(), orientation1.y(), orientation1.z()),
//     fuse_core::getPitch(orientation1.w(), orientation1.x(), orientation1.y(), orientation1.z()),
//     fuse_core::getYaw(orientation1.w(), orientation1.x(), orientation1.y(), orientation1.z())
//   );
//   // 3D material point projection model which matches the one used by r_l.
//   T sr = ceres::sin(rpy.x());
//   T cr = ceres::cos(rpy.x());
//   T sp = ceres::sin(rpy.y());
//   T cp = ceres::cos(rpy.y());
//   T sy = ceres::sin(rpy.z());  // Should probably be sin((yaw1 + yaw2) / 2), but r_l uses this model
//   T cy = ceres::cos(rpy.z());
//   T cpi = 1.0 / cp;
//   T tp = ceres::tan(rpy.y());

//   Eigen::Matrix<T, 3, 3> tf_pos, tf_rot, tf_vel, tf_acc;
//   Eigen::Matrix<T, 15, 15> state_tf;

//   tf_pos(0, 0) = (cy * cp);
//   tf_pos(0, 1) = (cy * sp * sr - sy * cr);
//   tf_pos(0, 2) = (cy * sp * cr + sy * sr);
//   tf_pos(1, 0) = (sy * cp);
//   tf_pos(1, 1) = (sy * sp * sr + cy * cr);
//   tf_pos(1, 2) = (sy * sp * cr - cy * sr);
//   tf_pos(2, 0) = (-sp);
//   tf_pos(2, 1) = (cp * sr);
//   tf_pos(2, 2) = (cp * cr);

//   tf_rot(0, 0) = 1;
//   tf_rot(0, 1) = sr * sp * cpi;
//   tf_rot(0, 2) = cr * sp * cpi;
//   tf_rot(1, 0) = T(0);
//   tf_rot(1, 1) = cr;
//   tf_rot(1, 2) = -sr;
//   tf_rot(2, 0) = T(0);
//   tf_rot(2, 1) = sr * cpi;
//   tf_rot(2, 2) = cr * cpi;

//   // Project the state
//   position2 = position1 + tf_pos * vel_linear1 * dt + 0.5 * tf_pos * acc_linear1 * dt * dt;
//   rpy = rpy + tf_rot * vel_angular1 * dt;
//   vel_linear2 = vel_linear1 + acc_linear1 * dt;
//   vel_angular2 = vel_angular1;
//   acc_linear2 = acc_linear1;

//   fuse_core::wrapAngle2D(rpy.x());
//   fuse_core::wrapAngle2D(rpy.y());
//   fuse_core::wrapAngle2D(rpy.z());

//   // Convert back to quaternion
//   orientation2 = Eigen::AngleAxis<T>(rpy.z(), Eigen::Matrix<T, 3, 1>::UnitZ()) *
//                  Eigen::AngleAxis<T>(rpy.y(), Eigen::Matrix<T, 3, 1>::UnitY()) *
//                  Eigen::AngleAxis<T>(rpy.x(), Eigen::Matrix<T, 3, 1>::UnitX());
// }
/**
 * @brief Given a state and time delta, predicts a new state - 
 *        inner templated version for ceres autodiff
 * @param[in] position1      - First position
 * @param[in] orientation1   - First orientation
 * @param[in] vel_linear1    - First linear velocity
 * @param[in] vel_angular1   - First angular velocity
 * @param[in] acc_linear1    - First linear acceleration
 * @param[in] dt - The time delta across which to predict the state
 * @param[out] position2     - Second position
 * @param[out] orientation2  - Second orientation
 * @param[out] vel_linear2   - Second linear velocity
 * @param[out] vel_angular2  - Second angular velocity
 * @param[out] acc_linear2   - Second linear acceleration
 */
inline void predict_eigen(
  const Eigen::Matrix<double, 3, 1>& position1,
  const Eigen::Quaternion<double>& orientation1,
  const Eigen::Matrix<double, 3, 1>& vel_linear1,
  const Eigen::Matrix<double, 3, 1>& vel_angular1,
  const Eigen::Matrix<double, 3, 1>& acc_linear1,
  const double dt,
  Eigen::Matrix<double, 3, 1> position2,
  Eigen::Quaternion<double> orientation2,
  Eigen::Matrix<double, 3, 1> vel_linear2,
  Eigen::Matrix<double, 3, 1> vel_angular2,
  Eigen::Matrix<double, 3, 1> acc_linear2)
{
  // Convert quaternion to eigen
  Eigen::Vector<double, 3> rpy(
    fuse_core::getRoll(orientation1.w(), orientation1.x(), orientation1.y(), orientation1.z()),
    fuse_core::getPitch(orientation1.w(), orientation1.x(), orientation1.y(), orientation1.z()),
    fuse_core::getYaw(orientation1.w(), orientation1.x(), orientation1.y(), orientation1.z())
  );
  fuse_core::wrapAngle2D(rpy.x());
  fuse_core::wrapAngle2D(rpy.y());
  fuse_core::wrapAngle2D(rpy.z());
  // 3D material point projection model which matches the one used by r_l.
  double sr = ceres::sin(rpy.x());
  double cr = ceres::cos(rpy.x());
  double sp = ceres::sin(rpy.y());
  double cp = ceres::cos(rpy.y());
  double sy = ceres::sin(rpy.z());  // Should probably be sin((yaw1 + yaw2) / 2), but r_l uses this model
  double cy = ceres::cos(rpy.z());
  double cpi = 1.0 / cp;

  Eigen::Matrix<double, 3, 3> tf_pos, tf_rot, tf_vel, tf_acc;
  Eigen::Matrix<double, 15, 15> state_tf;

  tf_pos(0, 0) = (cy * cp); // vx1 to x2
  tf_pos(0, 1) = (cy * sp * sr - sy * cr); // vy1 to x2
  tf_pos(0, 2) = (cy * sp * cr + sy * sr); // vz1 to x2
  tf_pos(1, 0) = (sy * cp); // vx1 to y2
  tf_pos(1, 1) = (sy * sp * sr + cy * cr); // vy1 to y2
  tf_pos(1, 2) = (sy * sp * cr - cy * sr); // vz1 to y2
  tf_pos(2, 0) = (-sp); // vx1 to z2
  tf_pos(2, 1) = (cp * sr); // vy1 to z2
  tf_pos(2, 2) = (cp * cr); // vz1 to z2

  tf_rot(0, 0) = 1;
  tf_rot(0, 1) = sr * sp * cpi;
  tf_rot(0, 2) = cr * sp * cpi;
  tf_rot(1, 0) = double(0);
  tf_rot(1, 1) = cr;
  tf_rot(1, 2) = -sr;
  tf_rot(2, 0) = double(0);
  tf_rot(2, 1) = sr * cpi;
  tf_rot(2, 2) = cr * cpi;

  // Project the state
  position2 = position1 + tf_pos * vel_linear1 * dt + 0.5 * tf_pos * acc_linear1 * dt * dt;
  rpy = rpy + tf_rot * vel_angular1 * dt;
  vel_linear2 = vel_linear1 + acc_linear1 * dt;
  vel_angular2 = vel_angular1;
  acc_linear2 = acc_linear1;

  fuse_core::wrapAngle2D(rpy.x());
  fuse_core::wrapAngle2D(rpy.y());
  fuse_core::wrapAngle2D(rpy.z());

  // Convert back to quaternion
  orientation2 = Eigen::AngleAxis<double>(rpy.z(), Eigen::Matrix<double, 3, 1>::UnitZ()) *
                 Eigen::AngleAxis<double>(rpy.y(), Eigen::Matrix<double, 3, 1>::UnitY()) *
                 Eigen::AngleAxis<double>(rpy.x(), Eigen::Matrix<double, 3, 1>::UnitX());
}

/**
 * @brief Given a state and time delta, predicts a new state - 
 *        inner templated version for ceres autodiff
 * @param[in] position1      - First position
 * @param[in] orientation1   - First orientation
 * @param[in] vel_linear1    - First linear velocity
 * @param[in] vel_angular1   - First angular velocity
 * @param[in] acc_linear1    - First linear acceleration
 * @param[in] dt - The time delta across which to predict the state
 * @param[out] position2     - Second position
 * @param[out] orientation2  - Second orientation
 * @param[out] vel_linear2   - Second linear velocity
 * @param[out] vel_angular2  - Second angular velocity
 * @param[out] acc_linear2   - Second linear acceleration
 */
inline void predict_eigen(
  const Eigen::Matrix<Jet, 3, 1>& position1,
  const Eigen::Quaternion<Jet>& orientation1,
  const Eigen::Matrix<Jet, 3, 1>& vel_linear1,
  const Eigen::Matrix<Jet, 3, 1>& vel_angular1,
  const Eigen::Matrix<Jet, 3, 1>& acc_linear1,
  const Jet dt,
  Eigen::Matrix<Jet, 3, 1> position2,
  Eigen::Quaternion<Jet> orientation2,
  Eigen::Matrix<Jet, 3, 1> vel_linear2,
  Eigen::Matrix<Jet, 3, 1> vel_angular2,
  Eigen::Matrix<Jet, 3, 1> acc_linear2)
{
  // Convert quaternion to eigen
  Eigen::Vector<Jet, 3> rpy(
    fuse_core::getRoll(orientation1.w(), orientation1.x(), orientation1.y(), orientation1.z()),
    fuse_core::getPitch(orientation1.w(), orientation1.x(), orientation1.y(), orientation1.z()),
    fuse_core::getYaw(orientation1.w(), orientation1.x(), orientation1.y(), orientation1.z())
  );
  fuse_core::wrapAngle2D(rpy.x());
  fuse_core::wrapAngle2D(rpy.y());
  fuse_core::wrapAngle2D(rpy.z());
  // 3D material point projection model which matches the one used by r_l.
  Jet sr = ceres::sin(rpy.x());
  Jet cr = ceres::cos(rpy.x());
  Jet sp = ceres::sin(rpy.y());
  Jet cp = ceres::cos(rpy.y());
  Jet sy = ceres::sin(rpy.z());  // Should probably be sin((yaw1 + yaw2) / 2), but r_l uses this model
  Jet cy = ceres::cos(rpy.z());
  Jet cpi = 1.0 / cp;

  Eigen::Matrix<Jet, 3, 3> tf_pos, tf_rot, tf_vel, tf_acc;
  Eigen::Matrix<Jet, 15, 15> state_tf;

  tf_pos(0, 0) = (cy * cp);
  tf_pos(0, 1) = (cy * sp * sr - sy * cr);
  tf_pos(0, 2) = (cy * sp * cr + sy * sr);
  tf_pos(1, 0) = (sy * cp);
  tf_pos(1, 1) = (sy * sp * sr + cy * cr);
  tf_pos(1, 2) = (sy * sp * cr - cy * sr);
  tf_pos(2, 0) = (-sp);
  tf_pos(2, 1) = (cp * sr);
  tf_pos(2, 2) = (cp * cr);

  tf_rot(0, 0) = Jet(1);
  tf_rot(0, 1) = sr * sp * cpi;
  tf_rot(0, 2) = cr * sp * cpi;
  tf_rot(1, 0) = Jet(0);
  tf_rot(1, 1) = cr;
  tf_rot(1, 2) = -sr;
  tf_rot(2, 0) = Jet(0);
  tf_rot(2, 1) = sr * cpi;
  tf_rot(2, 2) = cr * cpi;

  // Project the state
  position2 = position1 + tf_pos * vel_linear1 * dt + 0.5 * tf_pos * acc_linear1 * dt * dt;
  rpy = rpy + tf_rot * vel_angular1 * dt;
  vel_linear2 = vel_linear1 + acc_linear1 * dt;
  vel_angular2 = vel_angular1;
  acc_linear2 = acc_linear1;

  fuse_core::wrapAngle2D(rpy.x());
  fuse_core::wrapAngle2D(rpy.y());
  fuse_core::wrapAngle2D(rpy.z());

  // Convert back to quaternion
  orientation2 = Eigen::AngleAxis<Jet>(rpy.z(), Eigen::Matrix<Jet, 3, 1>::UnitZ()) *
                 Eigen::AngleAxis<Jet>(rpy.y(), Eigen::Matrix<Jet, 3, 1>::UnitY()) *
                 Eigen::AngleAxis<Jet>(rpy.x(), Eigen::Matrix<Jet, 3, 1>::UnitX());

    // Project the state - dummy versions
  position2 = position1;
  orientation2 = orientation1;
  vel_linear2 = vel_linear1;
  vel_angular2 = vel_angular1;
  acc_linear2 = acc_linear1;
}

/**
 * @brief Given a state and time delta, predicts a new state -
 *        Templated version to be called from ceres autodiff costfunction
 * @param[in] position1    - First position (array with x at index 0, y at index 1, z at index 2)
 * @param[in] orientation1 - First orientation (array with w at index 0, x at index 1, y at index 2, z at index 3) check this order
 * @param[in] vel_linear1  - First linear velocity (array with x at index 0, y at index 1, z at index 2)
 * @param[in] vel_angular1 - First angular velocity (array with vroll at index 0, vpitch at index 1, vyaw at index 2)
 * @param[in] acc_linear1  - First linear acceleration (array with x at index 0, y at index 1, z at index 2)
 * @param[in] dt - The time delta across which to predict the state
 * @param[out] position2 - Second position (array with x at index 0, y at index 1, z at index 2)
 * @param[out] orientation2 - Second orientation (array with w at index 0, x at index 1, y at index 2, z at index 3) check this order
 * @param[out] vel_linear2  - Second velocity (array with x at index 0, y at index 1, z at index 2)
 * @param[out] vel_angular2 - Second yaw velocity (array with vroll at index 0, vpitch at index 1, vyaw at index 2)
 * @param[out] acc_linear2  - Second linear acceleration (array with x at index 0, y at index 1, z at index 2)
 */
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
  // conversion from array to eigen
  // TODO: check how to map quaternion for rowmajor arrays
  const Eigen::Matrix<T, 3, 1> position1_eigen = 
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> (position1);
  const Eigen::Quaternion<T> orientation1_eigen = 
    Eigen::Map<const Eigen::Quaternion<T>> (orientation1);
  const Eigen::Matrix<T, 3, 1> vel_linear1_eigen = 
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> (vel_linear1);
  const Eigen::Matrix<T, 3, 1> vel_angular1_eigen = 
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> (vel_angular1);
  const Eigen::Matrix<T, 3, 1> acc_linear1_eigen = 
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> (acc_linear1);

  Eigen::Matrix<T, 3, 1> position2_eigen = 
    Eigen::Map<Eigen::Matrix<T, 3, 1>> (position2);
  Eigen::Quaternion<T> orientation2_eigen =
    Eigen::Map<Eigen::Quaternion<T>> (orientation2);
  Eigen::Matrix<T, 3, 1> vel_linear2_eigen =
    Eigen::Map<Eigen::Matrix<T, 3, 1>> (vel_linear2);
  Eigen::Matrix<T, 3, 1> vel_angular2_eigen =
    Eigen::Map<Eigen::Matrix<T, 3, 1>> (vel_angular2);
  Eigen::Matrix<T, 3, 1> acc_linear2_eigen =
    Eigen::Map<Eigen::Matrix<T, 3, 1>> (acc_linear2);
 
  predict_eigen(
    position1_eigen,
    orientation1_eigen,
    vel_linear1_eigen,
    vel_angular1_eigen,
    acc_linear1_eigen,
    dt,
    position2_eigen,
    orientation2_eigen,
    vel_linear2_eigen,
    vel_angular2_eigen,
    acc_linear2_eigen);
}
/**
 * @brief Given a state and time delta, predicts a new state + computes the Jacobians - 
 *        version for predicting new states inside the plugin
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
  const fuse_core::Vector3d & position1,
  const fuse_core::Quaternion & orientation1,
  const fuse_core::Vector3d & vel_linear1,
  const fuse_core::Vector3d & vel_angular1,
  const fuse_core::Vector3d & acc_linear1,
  const double dt,
  fuse_core::Vector3d & position2,
  fuse_core::Quaternion & orientation2,
  fuse_core::Vector3d & vel_linear2,
  fuse_core::Vector3d & vel_angular2,
  fuse_core::Vector3d & acc_linear2)
{
  // This is all double code - keep it like this if we want to use also analytic diff, otherwise we
  // can call the templated version above from here
  // Convert quaternion to eigen
  fuse_core::Vector3d rpy(
    fuse_core::getRoll(orientation1.w(), orientation1.x(), orientation1.y(), orientation1.z()),
    fuse_core::getPitch(orientation1.w(), orientation1.x(), orientation1.y(), orientation1.z()),
    fuse_core::getYaw(orientation1.w(), orientation1.x(), orientation1.y(), orientation1.z())
  );
  fuse_core::wrapAngle2D(rpy.x());
  fuse_core::wrapAngle2D(rpy.y());
  fuse_core::wrapAngle2D(rpy.z());
  // 3D material point projection model which matches the one used by r_l.
  double sr = ceres::sin(rpy.x());
  double cr = ceres::cos(rpy.x());
  double sp = ceres::sin(rpy.y());
  double cp = ceres::cos(rpy.y());
  double sy = ceres::sin(rpy.z());  // Should probably be sin((yaw1 + yaw2) / 2), but r_l uses this model
  double cy = ceres::cos(rpy.z());
  double cpi = 1.0 / cp;

  fuse_core::Matrix3d tf_pos, tf_rot, tf_vel, tf_acc;
  fuse_core::Matrix15d state_tf;

  tf_pos(0, 0) = (cy * cp);
  tf_pos(0, 1) = (cy * sp * sr - sy * cr);
  tf_pos(0, 2) = (cy * sp * cr + sy * sr);
  tf_pos(1, 0) = (sy * cp);
  tf_pos(1, 1) = (sy * sp * sr + cy * cr);
  tf_pos(1, 2) = (sy * sp * cr - cy * sr);
  tf_pos(2, 0) = (-sp);
  tf_pos(2, 1) = (cp * sr);
  tf_pos(2, 2) = (cp * cr);

  tf_rot(0, 0) = 1.0;
  tf_rot(0, 1) = sr * sp * cpi;
  tf_rot(0, 2) = cr * sp * cpi;
  tf_rot(1, 0) = 0.0;
  tf_rot(1, 1) = cr;
  tf_rot(1, 2) = -sr;
  tf_rot(2, 0) = 0.0;
  tf_rot(2, 1) = sr * cpi;
  tf_rot(2, 2) = cr * cpi;

  // Project the state
  position2 = position1 + tf_pos * vel_linear1 * dt + 0.5 * tf_pos * acc_linear1 * dt * dt;
  rpy = rpy + tf_rot * vel_angular1 * dt;
  vel_linear2 = vel_linear1 + acc_linear1 * dt;
  vel_angular2 = vel_angular1;
  acc_linear2 = acc_linear1;

  fuse_core::wrapAngle2D(rpy.x());
  fuse_core::wrapAngle2D(rpy.y());
  fuse_core::wrapAngle2D(rpy.z());

    // Convert back to quaternion
  orientation2 = Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
                 Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
                 Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX());

  // // Compute the transfer function matrix
  // state_tf.setZero();
  // // position
  // state_tf.block<3, 3>(0, 0) = fuse_core::Matrix3d::Identity();
  // state_tf.block<3, 3>(0, 6) = tf_pos;
  // state_tf.block<3, 3>(0, 12) = 0.5 * tf_pos * dt;
  // // orientation
  // state_tf.block<3, 3>(3, 3) = fuse_core::Matrix3d::Identity();
  // state_tf.block<3, 3>(3, 9) = tf_rot;
  // // linear velocity
  // state_tf.block<3, 3>(6, 6) = fuse_core::Matrix3d::Identity();
  // state_tf.block<3, 3>(6, 12) = dt * fuse_core::Matrix3d::Identity();
  // // angular velocity
  // state_tf.block<3, 3>(9, 9) = fuse_core::Matrix3d::Identity();
  // // linear acceleration
  // state_tf.block<3, 3>(12, 12) = fuse_core::Matrix3d::Identity();

  // if (state_tf_jacobian)
  // {
  //   // TODO: compute the jacobian of the motion model transfer function
  //   double x_coeff = 0.0;
  //   double y_coeff = 0.0;
  //   double z_coeff = 0.0;
  //   double one_half_at_squared = 0.5 * dt * dt;

  //   y_coeff = cy * sp * cr + sy * sr;
  //   z_coeff = -cy * sp * sr + sy * cr;
  //   double dFx_dR = (y_coeff * vel_linear.y() + z_coeff * vel_linear.z()) * dt +
  //     (y_coeff * acc_linear.y() + z_coeff * acc_linear.z()) * one_half_at_squared;
  //   double dFR_dR = 1.0 + (cr * tp * vel_angular.y() - sr * tp * vel_angular.z()) * dt;

  //   x_coeff = -cy * sp;
  //   y_coeff = cy * cp * sr;
  //   z_coeff = cy * cp * cr;
  //   double dFx_dP =
  //     (x_coeff * vel_linear.x() + y_coeff * vel_linear.y() + z_coeff * vel_linear.z()) * dt +
  //     (x_coeff * acc_linear.x() + y_coeff * acc_linear.y() + z_coeff * acc_linear.z()) *
  //     one_half_at_squared;
  //   double dFR_dP =
  //     (cpi * cpi * sr * vel_angular.y() + cpi * cpi * cr * vel_angular.z()) * dt;

  //   x_coeff = -sy * cp;
  //   y_coeff = -sy * sp * sr - cy * cr;
  //   z_coeff = -sy * sp * cr + cy * sr;
  //   double dFx_dY =
  //     (x_coeff * vel_linear.x() + y_coeff * vel_linear.y() + z_coeff * vel_linear.z()) * dt +
  //     (x_coeff * acc_linear.x() + y_coeff * acc_linear.y() + z_coeff * acc_linear.z()) *
  //     one_half_at_squared;

  //   y_coeff = sy * sp * cr - cy * sr;
  //   z_coeff = -sy * sp * sr - cy * cr;
  //   double dFy_dR = (y_coeff * vel_linear.y() + z_coeff * vel_linear.z()) * dt +
  //     (y_coeff * acc_linear.y() + z_coeff * acc_linear.z()) * one_half_at_squared;
  //   double dFP_dR = (-sr * vel_angular.y() - cr * vel_angular.z()) * dt;

  //   x_coeff = -sy * sp;
  //   y_coeff = sy * cp * sr;
  //   z_coeff = sy * cp * cr;
  //   double dFy_dP =
  //     (x_coeff * vel_linear.x() + y_coeff * vel_linear.y() + z_coeff * vel_linear.z()) * dt +
  //     (x_coeff * acc_linear.x() + y_coeff * acc_linear.y() + z_coeff * acc_linear.z()) *
  //     one_half_at_squared;

  //   x_coeff = cy * cp;
  //   y_coeff = cy * sp * sr - sy * cr;
  //   z_coeff = cy * sp * cr + sy * sr;
  //   double dFy_dY =
  //     (x_coeff * vel_linear.x() + y_coeff * vel_linear.y() + z_coeff * vel_linear.z()) * dt +
  //     (x_coeff * acc_linear.x() + y_coeff * acc_linear.y() + z_coeff * acc_linear.z()) *
  //     one_half_at_squared;

  //   y_coeff = cp * cr;
  //   z_coeff = -cp * sr;
  //   double dFz_dR = (y_coeff * vel_linear.y() + z_coeff * vel_linear.z()) * dt +
  //     (y_coeff * acc_linear.y() + z_coeff * acc_linear.z()) * one_half_at_squared;
  //   double dFY_dR = (cr * cpi * vel_angular.y() - sr * cpi * vel_angular.z()) * dt;

  //   x_coeff = -cp;
  //   y_coeff = -sp * sr;
  //   z_coeff = -sp * cr;
  //   double dFz_dP =
  //     (x_coeff * vel_linear.x() + y_coeff * vel_linear.y() + z_coeff * vel_linear.z()) * dt +
  //     (x_coeff * acc_linear.x() + y_coeff * acc_linear.y() + z_coeff * acc_linear.z()) *
  //     one_half_at_squared;
  //   double dFY_dP =
  //     (sr * tp * cpi * vel_angular.y() + cr * tp * cpi * vel_angular.z()) * dt;

  //   *state_tf_jacobian = state_tf;
  //   (*state_tf_jacobian)(0, 3) = dFx_dR;
  //   (*state_tf_jacobian)(0, 4) = dFx_dP;
  //   (*state_tf_jacobian)(0, 5) = dFx_dY;
  //   (*state_tf_jacobian)(1, 3) = dFy_dR;
  //   (*state_tf_jacobian)(1, 4) = dFy_dP;
  //   (*state_tf_jacobian)(1, 5) = dFy_dY;
  //   (*state_tf_jacobian)(2, 3) = dFz_dR;
  //   (*state_tf_jacobian)(2, 4) = dFz_dP;
  //   (*state_tf_jacobian)(3, 3) = dFR_dR;
  //   (*state_tf_jacobian)(3, 4) = dFR_dP;
  //   (*state_tf_jacobian)(4, 3) = dFP_dR;
  //   (*state_tf_jacobian)(5, 3) = dFY_dR;
  //   (*state_tf_jacobian)(5, 4) = dFY_dP;
  // }

  // // Predict the new state
  // fuse_core::Vector15d state;
  // state.head(3)      = position;
  // state.segment(3,3) = fuse_core::Vector3d(roll, pitch, yaw);
  // state.segment(6,3) = vel_linear;
  // state.segment(9,3) = vel_angular;
  // state.tail(3)      = acc_linear;
  
  // state = state_tf * state;

  // // Convert back
  // position_out    = state.head(3);
  // fuse_core::wrapAngle2D(state(5));
  // fuse_core::wrapAngle2D(state(4));
  // fuse_core::wrapAngle2D(state(3));
  // Eigen::Quaterniond q =
  // Eigen::AngleAxisd(state(5), Eigen::Vector3d::UnitZ()) *
  // Eigen::AngleAxisd(state(4), Eigen::Vector3d::UnitY()) *
  // Eigen::AngleAxisd(state(3), Eigen::Vector3d::UnitX());
  // orientation_out = q;
  // vel_linear_out  = state.segment(6,3);
  // vel_angular_out = state.segment(9,3);
  // acc_linear_out  = state.tail(3);
}
}  // namespace fuse_models

#endif  // FUSE_MODELS__UNICYCLE_3D_PREDICT_HPP_
