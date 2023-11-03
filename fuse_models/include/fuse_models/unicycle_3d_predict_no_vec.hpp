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
#include "covariance_geometry/pose_representation.hpp"

namespace fuse_models
{
  using Jet = ceres::Jet<double, 32>;

template<typename T>
inline void predict(
  const T position1_x,
  const T position1_y,
  const T position1_z,
  const T quat1_w,
  const T quat1_x,
  const T quat1_y,
  const T quat1_z,
  const T vel_linear1_x,
  const T vel_linear1_y,
  const T vel_linear1_z,
  const T vel_roll1,
  const T vel_pitch1,
  const T vel_yaw1,
  const T acc_linear1_x,
  const T acc_linear1_y,
  const T acc_linear1_z,
  const T dt,
  T & position2_x,
  T & position2_y,
  T & position2_z,
  T & quat2_w,
  T & quat2_x,
  T & quat2_y,
  T & quat2_z,
  T & vel_linear2_x,
  T & vel_linear2_y,
  T & vel_linear2_z,
  T & vel_roll2,
  T & vel_pitch2,
  T & vel_yaw2,
  T & acc_linear2_x,
  T & acc_linear2_y,
  T & acc_linear2_z)
{
    // Convert quaternion to eigen
  T roll1 = fuse_core::getRoll(quat1_w, quat1_x, quat1_y, quat1_z);
  T pitch1 = fuse_core::getPitch(quat1_w, quat1_x, quat1_y, quat1_z);
  T yaw1 = fuse_core::getYaw(quat1_w, quat1_x, quat1_y, quat1_z);
  fuse_core::wrapAngle2D(roll1);
  fuse_core::wrapAngle2D(pitch1);
  fuse_core::wrapAngle2D(yaw1);
  // 3D material point projection model which matches the one used by r_l.
  T sr = ceres::sin(roll1);
  T cr = ceres::cos(roll1);
  T sp = ceres::sin(pitch1);
  T cp = ceres::cos(pitch1);
  T sy = ceres::sin(yaw1);  // Should probably be sin((yaw1 + yaw2) / 2), but r_l uses this model
  T cy = ceres::cos(yaw1);
  T cpi = 1.0 / cp;

  // Project the state
  position2_x = position1_x + 
    ((cy * cp) * vel_linear1_x + (cy * sp * sr - sy * cr) * vel_linear1_y + (cy * sp * cr + sy * sr) * vel_linear1_z) * dt +
    0.5 * ((cy * cp) * acc_linear1_x + (cy * sp * sr - sy * cr) * acc_linear1_y + (cy * sp * cr + sy * sr) * acc_linear1_z) * dt * dt;
  
  position2_y = position1_y + 
    ((sy * cp) * vel_linear1_x + (sy * sp * sr + cy * cr) * vel_linear1_y + (sy * sp * cr - cy * sr) * vel_linear1_z) * dt +
    0.5 * ((sy * cp) * acc_linear1_x + (sy * sp * sr + cy * cr) * acc_linear1_y + (sy * sp * cr - cy * sr) * acc_linear1_z) * dt * dt;
  
  position2_z = position1_z + 
    ((-sp) * vel_linear1_x + (cp * sr) * vel_linear1_y + (cp * cr) * vel_linear1_z) * dt +
    0.5 * ((-sp) * acc_linear1_x + (cp * sr) * acc_linear1_y + (cp * cr) * acc_linear1_z) * dt * dt;
  
  T roll2 = roll1 + 
    (vel_roll1 + sr * sp * cpi * vel_pitch1 + cr * sp * cpi * vel_yaw1) * dt;
  T pitch2 = pitch1 +
    (T(0) * vel_roll1 + cr * vel_pitch1 - sr * vel_yaw1) * dt;
  T yaw2 = yaw1 +
    (T(0) * vel_roll1 + sr * cpi * vel_pitch1 + cr * cpi * vel_yaw1) * dt;
  
  fuse_core::wrapAngle2D(roll2);
  fuse_core::wrapAngle2D(pitch2);
  fuse_core::wrapAngle2D(yaw2);

  Eigen::Quaternion<T> q;
  q.w() = cos(roll2 / T(2)) * cos(pitch2 / T(2)) * cos(yaw2 / T(2)) +
    sin(roll2 / T(2)) * sin(pitch2 / T(2)) * sin(yaw2 / T(2));
  q.x() = sin(roll2 / T(2)) * cos(pitch2 / T(2)) * cos(yaw2 / T(2)) -
    cos(roll2 / T(2)) * sin(pitch2 / T(2)) * sin(yaw2 / T(2));
  q.y() = cos(roll2 / T(2)) * sin(pitch2 / T(2)) * cos(yaw2 / T(2)) +
    sin(roll2 / T(2)) * cos(pitch2 / T(2)) * sin(yaw2 / T(2));
  q.z() = cos(roll2 / T(2)) * cos(pitch2 / T(2)) * sin(yaw2 / T(2)) -
    sin(roll2 / T(2)) * sin(pitch2 / T(2)) * cos(yaw2 / T(2));
  q.normalize();

  quat2_w = q.w();
  quat2_x = q.x();
  quat2_y = q.y();
  quat2_z = q.z();

  vel_linear2_x = vel_linear1_x + acc_linear1_x * dt;
  vel_linear2_y = vel_linear1_y + acc_linear1_y * dt;
  vel_linear2_z = vel_linear1_z + acc_linear1_z * dt;

  vel_roll2 = vel_roll1;
  vel_pitch2 = vel_pitch1;
  vel_yaw2 = vel_yaw1;

  acc_linear2_x = acc_linear1_x;
  acc_linear2_y = acc_linear1_y;
  acc_linear2_z = acc_linear1_z;
}

/**
 * @brief Given a state and time delta, predicts a new state
 * @param[in] position1 - First position (array with x at index 0, y at index 1)
 * @param[in] yaw1 - First orientation
 * @param[in] vel_linear1 - First velocity (array with x at index 0, y at index 1)
 * @param[in] vel_yaw1 - First yaw velocity
 * @param[in] acc_linear1 - First linear acceleration (array with x at index 0, y at index 1)
 * @param[in] dt - The time delta across which to predict the state
 * @param[out] position2 - Second position (array with x at index 0, y at index 1)
 * @param[out] yaw2 - Second orientation
 * @param[out] vel_linear2 - Second velocity (array with x at index 0, y at index 1)
 * @param[out] vel_yaw2 - Second yaw velocity
 * @param[out] acc_linear2 - Second linear acceleration (array with x at index 0, y at index 1)
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
  predict(
    position1[0],
    position1[1],
    position1[2],
    orientation1[0],
    orientation1[1],
    orientation1[2],
    orientation1[3],
    vel_linear1[0],
    vel_linear1[1],
    vel_linear1[2],
    vel_angular1[0],
    vel_angular1[1],
    vel_angular1[2],
    acc_linear1[0],
    acc_linear1[1],
    acc_linear1[2],
    dt,
    position2[0],
    position2[1],
    position2[2],
    orientation2[0],
    orientation2[1],
    orientation2[2],
    orientation2[3],
    vel_linear2[0],
    vel_linear2[1],
    vel_linear2[2],
    vel_angular2[0],
    vel_angular2[1],
    vel_angular2[2],
    acc_linear2[0],
    acc_linear2[1],
    acc_linear2[2]);
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

  std::cout << "Predicted position: " << position2.transpose() << std::endl;
  std::cout << "Predicted orientation: w: " << orientation2.w() << " x:" << orientation2.x() << " y:" << orientation2.y() << " z:" << orientation2.z() << std::endl;
}
}  // namespace fuse_models

#endif  // FUSE_MODELS__UNICYCLE_3D_PREDICT_HPP_
