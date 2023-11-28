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

#include <Eigen/Core>

#include <fuse_core/util.hpp>
#include <fuse_core/eigen.hpp>

namespace fuse_models
{
/**
 * @brief Given a state and time delta, predicts a new state
 * @param[in] position1_x - First X position
 * @param[in] position1_y - First Y position
 * @param[in] position1_z - First Z position
 * @param[in] orientation1_r - First roll orientation
 * @param[in] orientation1_p - First pitch orientation
 * @param[in] orientation1_y - First yaw orientation
 * @param[in] vel_linear1_x - First X velocity
 * @param[in] vel_linear1_y - First Y velocity
 * @param[in] vel_linear1_z - First Z velocity
 * @param[in] vel_angular1_r - First roll velocity
 * @param[in] vel_angular1_p - First pitch velocity
 * @param[in] vel_angular1_y - First yaw velocity
 * @param[in] acc_linear1_x - First X acceleration
 * @param[in] acc_linear1_y - First Y acceleration
 * @param[in] acc_linear1_z - First Z acceleration
 * @param[in] dt - The time delta across which to predict the state
 * @param[out] position2_x - Second X position
 * @param[out] position2_y - Second Y position
 * @param[out] position2_z - Second Z position
 * @param[out] orientation2_r - Second roll orientation
 * @param[out] orientation2_p - Second pitch orientation
 * @param[out] orientation2_y - Second yaw orientation
 * @param[out] vel_linear2_x - Second X velocity
 * @param[out] vel_linear2_y - Second Y velocity
 * @param[out] vel_linear2_z - Second Z velocity
 * @param[out] vel_angular_r - Second roll velocity
 * @param[out] vel_angular_p - Second pitch velocity
 * @param[out] vel_angular_y - Second yaw velocity
 * @param[out] acc_linear2_x - Second X acceleration
 * @param[out] acc_linear2_y - Second Y acceleration
 * @param[out] acc_linear2_z - Second Z acceleration
 */
template<typename T>
inline void predict(
  const T position1_x,
  const T position1_y,
  const T position1_z,
  const T orientation1_r,
  const T orientation1_p,
  const T orientation1_y,
  const T vel_linear1_x,
  const T vel_linear1_y,
  const T vel_linear1_z,
  const T vel_angular1_r,
  const T vel_angular1_p,
  const T vel_angular1_y,
  const T acc_linear1_x,
  const T acc_linear1_y,
  const T acc_linear1_z,
  const T dt,
  T & position2_x,  
  T & position2_y,
  T & position2_z,
  T & orientation2_r,
  T & orientation2_p,
  T & orientation2_y,
  T & vel_linear2_x,
  T & vel_linear2_y,
  T & vel_linear2_z,
  T & vel_angular2_r,
  T & vel_angular2_p,
  T & vel_angular2_y,
  T & acc_linear2_x,
  T & acc_linear2_y,
  T & acc_linear2_z)
{
  // 3D material point projection model which matches the one used by r_l.
  T sr = ceres::sin(orientation1_r);
  T cr = ceres::cos(orientation1_r);
  T sp = ceres::sin(orientation1_p);
  T cp = ceres::cos(orientation1_p);
  T sy = ceres::sin(orientation1_y);
  T cy = ceres::cos(orientation1_y);
  T cpi = T(1.0) / cp;

  // Project the state
  position2_x = position1_x + 
    ((cy * cp) * vel_linear1_x + (cy * sp * sr - sy * cr) * vel_linear1_y + (cy * sp * cr + sy * sr) * vel_linear1_z) * dt +
    ((cy * cp) * acc_linear1_x + (cy * sp * sr - sy * cr) * acc_linear1_y + (cy * sp * cr + sy * sr) * acc_linear1_z) * T(0.5) * dt * dt;
  position2_y = position1_y +
    ((sy * cp) * vel_linear1_x + (sy * sp * sr + cy * cr) * vel_linear1_y + (sy * sp * cr - cy * sr) * vel_linear1_z) * dt +
    ((sy * cp) * acc_linear1_x + (sy * sp * sr + cy * cr) * acc_linear1_y + (sy * sp * cr - cy * sr) * acc_linear1_z) * T(0.5) * dt * dt;
  position2_z = position1_z +
    ((-sp) * vel_linear1_x + (cp * sr) + vel_linear1_y + (cp * cr) * vel_linear1_z) * dt +
    ((-sp) * acc_linear1_x + (cp * sr) + acc_linear1_y + (cp * cr) * acc_linear1_z) * T(0.5) * dt * dt;
  
  orientation2_r = orientation1_r + 
    (vel_angular1_r + sr * sp * cpi * vel_angular1_p + cr * sp * cpi * vel_angular1_y) * dt;
  orientation2_p = orientation1_p + 
    (cr * vel_angular1_p - sr * vel_angular1_y) * dt;
  orientation2_y = orientation1_y + 
    (sr * cpi * vel_angular1_p + cr * cpi * vel_angular1_y) * dt;
  
  vel_linear2_x = vel_linear1_x + acc_linear1_x * dt;
  vel_linear2_y = vel_linear1_y + acc_linear1_y * dt;
  vel_linear2_z = vel_linear1_z + acc_linear1_z * dt;

  vel_angular2_r = vel_angular1_r;
  vel_angular2_p = vel_angular1_p;
  vel_angular2_y = vel_angular1_y;

  acc_linear2_x = acc_linear1_x;
  acc_linear2_y = acc_linear1_y;
  acc_linear2_z = acc_linear1_z;

  fuse_core::wrapAngle2D(orientation2_r);
  fuse_core::wrapAngle2D(orientation2_p);
  fuse_core::wrapAngle2D(orientation2_y);
  }

/**
 * @brief Given a state and time delta, predicts a new state -
 * @param[in] position1    - First position (array with x at index 0, y at index 1, z at index 2)
 * @param[in] orientation1 - First orientation (array with roll at index 0, pitch at index 1, yaw at index 2)
 * @param[in] vel_linear1  - First linear velocity (array with x at index 0, y at index 1, z at index 2)
 * @param[in] vel_angular1 - First angular velocity (array with vroll at index 0, vpitch at index 1, vyaw at index 2)
 * @param[in] acc_linear1  - First linear acceleration (array with x at index 0, y at index 1, z at index 2)
 * @param[in] dt - The time delta across which to predict the state
 * @param[out] position2    - Second position (array with x at index 0, y at index 1, z at index 2)
 * @param[out] orientation2 - Second orientation (array with roll at index 0, pitch at index 1, yaw at index 2)
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
  predict(
    position1[0],
    position1[1],
    position1[2],
    orientation1[0],
    orientation1[1],
    orientation1[2],
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
 * @brief Given a state and time delta, predicts a new state
 * @param[in] position1    - First position
 * @param[in] orientation1 - First orientation (quaternion)
 * @param[in] vel_linear1  - First linear velocity
 * @param[in] vel_angular1 - First angular velocity
 * @param[in] acc_linear1  - First linear acceleration
 * @param[in] dt - The time delta across which to predict the state
 * @param[out] position2    - Second position
 * @param[out] orientation2 - Second orientation (quaternion)
 * @param[out] vel_linear2  - Second linear velocity
 * @param[out] vel_angular2 - Second angular velocity
 * @param[out] acc_linear2  - Second linear acceleration
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
  // Convert quaternion to eigen
  fuse_core::Vector3d rpy(
    fuse_core::getRoll(orientation1.w(), orientation1.x(), orientation1.y(), orientation1.z()),
    fuse_core::getPitch(orientation1.w(), orientation1.x(), orientation1.y(), orientation1.z()),
    fuse_core::getYaw(orientation1.w(), orientation1.x(), orientation1.y(), orientation1.z())
  );
  // 3D material point projection model which matches the one used by r_l.
  double sr = ceres::sin(rpy.x());
  double cr = ceres::cos(rpy.x());
  double sp = ceres::sin(rpy.y());
  double cp = ceres::cos(rpy.y());
  double sy = ceres::sin(rpy.z());  // Should probably be sin((yaw1 + yaw2) / 2), but r_l uses this model
  double cy = ceres::cos(rpy.z());
  double cpi = 1.0 / cp;

  fuse_core::Matrix3d tf_pos, tf_rot;
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
  position2 = position1 + tf_pos * vel_linear1 * dt + tf_pos * acc_linear1 * 0.5 * dt * dt;
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
  }
}  // namespace fuse_models

#endif  // FUSE_MODELS__UNICYCLE_3D_PREDICT_HPP_
