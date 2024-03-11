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
  const T sr = ceres::sin(orientation1_r);
  const T cr = ceres::cos(orientation1_r);
  const T sp = ceres::sin(orientation1_p);
  const T cp = ceres::cos(orientation1_p);
  const T sy = ceres::sin(orientation1_y);
  const T cy = ceres::cos(orientation1_y);
  const T cpi = T(1.0) / cp;
  const T dt2 = T(0.5) * dt * dt;

  // Project the state
  position2_x = position1_x + 
    ((cy * cp) * vel_linear1_x + (cy * sp * sr - sy * cr) * vel_linear1_y + (cy * sp * cr + sy * sr) * vel_linear1_z) * dt +
    ((cy * cp) * acc_linear1_x + (cy * sp * sr - sy * cr) * acc_linear1_y + (cy * sp * cr + sy * sr) * acc_linear1_z) * dt2;
  position2_y = position1_y +
    ((sy * cp) * vel_linear1_x + (sy * sp * sr + cy * cr) * vel_linear1_y + (sy * sp * cr - cy * sr) * vel_linear1_z) * dt +
    ((sy * cp) * acc_linear1_x + (sy * sp * sr + cy * cr) * acc_linear1_y + (sy * sp * cr - cy * sr) * acc_linear1_z) * dt2;
   position2_z = position1_z +
    ((-sp) * vel_linear1_x + (cp * sr) * vel_linear1_y + (cp * cr) * vel_linear1_z) * dt +
    ((-sp) * acc_linear1_x + (cp * sr) * acc_linear1_y + (cp * cr) * acc_linear1_z) * dt2;
  
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
 * @param[out] jacobians - Jacobians wrt the state
 */
inline void predict(
  const double position1_x,
  const double position1_y,
  const double position1_z,
  const double orientation1_r,
  const double orientation1_p,
  const double orientation1_y,
  const double vel_linear1_x,
  const double vel_linear1_y,
  const double vel_linear1_z,
  const double vel_angular1_r,
  const double vel_angular1_p,
  const double vel_angular1_y,
  const double acc_linear1_x,
  const double acc_linear1_y,
  const double acc_linear1_z,
  const double dt,
  double & position2_x,  
  double & position2_y,
  double & position2_z,
  double & orientation2_r,
  double & orientation2_p,
  double & orientation2_y,
  double & vel_linear2_x,
  double & vel_linear2_y,
  double & vel_linear2_z,
  double & vel_angular2_r,
  double & vel_angular2_p,
  double & vel_angular2_y,
  double & acc_linear2_x,
  double & acc_linear2_y,
  double & acc_linear2_z,
  double ** jacobians,
  double * jacobian_quat2rpy)
{
  // 3D material point projection model which matches the one used by r_l.
  const double sr = ceres::sin(orientation1_r);
  const double cr = ceres::cos(orientation1_r);
  const double sp = ceres::sin(orientation1_p);
  const double cp = ceres::cos(orientation1_p);
  const double sy = ceres::sin(orientation1_y);
  const double cy = ceres::cos(orientation1_y);
  const double cpi = 1.0 / cp;
  const double dt2 = 0.5 * dt * dt;

  // Project the state
  position2_x = position1_x + 
    ((cp * cy) * vel_linear1_x + (sr * sp * cy - cr * sy) * vel_linear1_y + (cr * sp * cy  + sr * sy) * vel_linear1_z) * dt +
    ((cp * cy) * acc_linear1_x + (sr * sp * cy - cr * sy) * acc_linear1_y + (cr * sp * cy  + sr * sy) * acc_linear1_z) * dt2;
  position2_y = position1_y +
    ((cp * sy) * vel_linear1_x + (sr * sp * sy + cr * cy) * vel_linear1_y + (cr * sp * sy - sr * cy) * vel_linear1_z) * dt +
    ((cp * sy) * acc_linear1_x + (sr * sp * sy + cr * cy) * acc_linear1_y + (cr * sp * sy - sr * cy) * acc_linear1_z) * dt2;
  position2_z = position1_z +
    ((-sp) * vel_linear1_x + (sr * cp) * vel_linear1_y + (cr * cp) * vel_linear1_z) * dt +
    ((-sp) * acc_linear1_x + (sr * cp) * acc_linear1_y + (cr * cp) * acc_linear1_z) * dt2;
  
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

  if (jacobians) {
    // Jacobian wrt position1
    if (jacobians[0]) {
      Eigen::Map<fuse_core::Matrix<double, 15, 3>> jacobian(jacobians[0]);
      jacobian.setZero();
      // partial derivatives of position2 wrt orientation1
      jacobian(0, 0) = 1.0;
      jacobian(1, 1) = 1.0;
      jacobian(2, 2) = 1.0;
    }
    // Jacobian wrt orientation1 
    if (jacobians[1]) {
      Eigen::Map<fuse_core::Matrix<double, 15, 4>> jacobian(jacobians[1]);
      Eigen::Map<fuse_core::Matrix<double, 3, 4>> j_quat2rpy_map(jacobian_quat2rpy);
      fuse_core::Matrix<double, 6, 3> j_tmp;
      jacobian.setZero();
      j_tmp.setZero();
      
      // partial derivatives of position2_x wrt orientation1
      j_tmp(0, 0) = 
        ((cr * sp * cy + sr * sy) * vel_linear1_y + (- sr * sp * cy + cr * sy) * vel_linear1_z) * dt + 
        ((cr * sp * cy + sr * sy) * acc_linear1_y + (- sr * sp * cy + cr * sy) * acc_linear1_z) * dt2;
      j_tmp(0, 1) = 
        ((-sp * cy) * vel_linear1_x + (sr * cp * cy) * vel_linear1_y + (cr * cp * cy) * vel_linear1_z) * dt + 
        ((-sp * cy) * acc_linear1_x + (sr * cp * cy) * acc_linear1_y + (cr * cp * cy) * acc_linear1_z) * dt2; 
      j_tmp(0, 2) = 
        ((-cp * sy) * vel_linear1_x + (- sr * sp * sy - cr * cy) * vel_linear1_y + (- cr * sp * sy + sr * cy) * vel_linear1_z) * dt + 
        ((-cp * sy) * acc_linear1_x + (- sr * sp * sy - cr * cy) * acc_linear1_y + (- cr * sp * sy + sr * cy) * acc_linear1_z) * dt2;
      
       // partial derivatives of position2_y wrt orientation1
      j_tmp(1, 0) =  
        ((- sr * cy + cr * sp * sy) * vel_linear1_y + (- cr * cy - sr * sp * sy) * vel_linear1_z) * dt +
        ((- sr * cy + cr * sp * sy) * acc_linear1_y + (- cr * cy - sr * sp * sy) * acc_linear1_z) * dt2;
      j_tmp(1, 1) = 
        ((-sp * sy) * vel_linear1_x + (sr * cp * sy) * vel_linear1_y + (cr * cp * sy) * vel_linear1_z) * dt + 
        ((-sp * sy) * acc_linear1_x + (sr * cp * sy) * acc_linear1_y + (cr * cp * sy) * acc_linear1_z) * dt2;
      j_tmp(1, 2) = 
        ((cp * cy) * vel_linear1_x + (- cr * sy + sr * sp * cy) * vel_linear1_y + (sr * sy + cr * sp * cy) * vel_linear1_z) * dt + 
        ((cp * cy) * acc_linear1_x + (- cr * sy + sr * sp * cy) * acc_linear1_y + (sr * sy + cr * sp * cy) * acc_linear1_z) * dt2;
      
      // partial derivatives of position2_z wrt orientation1
      j_tmp(2, 0) = 
        ((cr * cp) * vel_linear1_y - (sr * cp) * vel_linear1_z) * dt + 
        ((cr * cp) * acc_linear1_y - (sr * cp) * acc_linear1_z) * dt2;
      j_tmp(2, 1) = 
        (-cp * vel_linear1_x - (sr * sp) * vel_linear1_y - (cr * sp) * vel_linear1_z) * dt +
        (-cp * acc_linear1_x - (sr * sp) * acc_linear1_y - (cr * sp) * acc_linear1_z) * dt2;

      // partial derivatives of orientation2_r wrt orientation1
      j_tmp(3, 0) = 1.0 + 
        ((cr * sp * cpi) * vel_angular1_p - (sr * sp * cpi) * vel_angular1_y) * dt;
      j_tmp(3, 1) = 
        ((sr + sr * sp * sp * cpi * cpi) * vel_angular1_p + (cr + cr *sp * sp * cpi * cpi) * vel_angular1_y) * dt;

      // partial derivatives of orientation2_p wrt orientation1
      j_tmp(4, 0) = (-sr * vel_angular1_p - cr * vel_angular1_y) * dt;
      j_tmp(4, 1) = 1.0;

      // partial derivatives of orientation2_y wrt orientation1
      j_tmp(5, 0) = ((cr * cpi) * vel_angular1_p - (sr * cpi) * vel_angular1_y) * dt;
      j_tmp(5, 1) = ((sp * sr * cpi * cpi) * vel_angular1_p + (cr * sp * cpi * cpi) * vel_angular1_y) * dt;
      j_tmp(5, 2) = 1.0;

      jacobian.block<6, 4>(0, 0) = j_tmp * j_quat2rpy_map;
    }                     
    // Jacobian wrt vel_linear1
    if (jacobians[2]) {
      Eigen::Map<fuse_core::Matrix<double, 15, 3>> jacobian(jacobians[2]);
      jacobian.setZero();

      // partial derivatives of position1_x wrt vel_linear1
      jacobian(0, 0) = cp * cy * dt;
      jacobian(0, 1) = (sr * sp * cy - cr * sy) * dt;
      jacobian(0, 2) = (cr * sp * cy  + sr * sy) * dt;
      // partial derivatives of position1_y wrt vel_linear1
      jacobian(1, 0) = cp * sy * dt;
      jacobian(1, 1) = (sr * sp * sy + cr * cy) * dt;
      jacobian(1, 2) = (cr * sp * sy - sr * cy) * dt;
      // partial derivatives of position1_z wrt vel_linear1
      jacobian(2, 0) = -sp * dt;
      jacobian(2, 1) = sr * cp * dt;
      jacobian(2, 2) = cr * cp * dt;
      // partial derivatives of vel_linear2_x wrt vel_linear1
      jacobian(6, 0) = 1.0;
      // partial derivatives of vel_linear2_y wrt vel_linear1
      jacobian(7, 1) = 1.0;
      // partial derivatives of vel_linear2_z wrt vel_linear1
      jacobian(8, 2) = 1.0;
    }

    // Jacobian wrt vel_angular1
    if (jacobians[3]) {
      Eigen::Map<fuse_core::Matrix<double, 15, 3>> jacobian(jacobians[3]);
      jacobian.setZero();

      // partial derivatives of orientation2_r wrt vel_angular1
      jacobian(3, 0) = dt;
      jacobian(3, 1) = sr * sp * cpi * dt;
      jacobian(3, 2) = cr * sp * cpi * dt;
      // partial derivatives of orientation2_p wrt vel_angular1
      jacobian(4, 1) = cr * dt;
      jacobian(4, 2) = -sr * dt;
      // partial derivatives of orientation2_y wrt vel_angular1
      jacobian(5, 1) = sr * cpi * dt;
      jacobian(5, 2) = cr * cpi * dt;
      // partial derivatives of vel_angular2_r wrt vel_angular1
      jacobian(9, 0) = 1.0;
      // partial derivatives of vel_angular2_p wrt vel_angular1
      jacobian(10, 1) = 1.0;
      // partial derivatives of vel_angular2_y wrt vel_angular1
      jacobian(11, 2) = 1.0;
    }

    // Jacobian wrt acc_linear1
    if (jacobians[4]) {
      Eigen::Map<fuse_core::Matrix<double, 15, 3>> jacobian(jacobians[4]);
      jacobian.setZero();
      // partial derivatives of position1_x wrt acc_linear1
      jacobian(0, 0) = cp * cy * dt2;
      jacobian(0, 1) = (sr * sp * cy - cr * sy) * dt2;
      jacobian(0, 2) = (cr * sp * cy  + sr * sy) * dt2;
      // partial derivatives of position1_y wrt acc_linear1
      jacobian(1, 0) = cp * sy * dt2;
      jacobian(1, 1) = (sr * sp * sy + cr * cy) * dt2;
      jacobian(1, 2) = (cr * sp * sy - sr * cy) * dt2;
      // partial derivatives of position1_z wrt acc_linear1
      jacobian(2, 0) = -sp * dt2;
      jacobian(2, 1) = sr * cp * dt2;
      jacobian(2, 2) = cr * cp * dt2;
      // partial derivatives of vel_linear2_x wrt acc_linear1
      jacobian(6, 0) = dt;
      // partial derivatives of vel_linear2_y wrt acc_linear1
      jacobian(7, 1) = dt;
      // partial derivatives of vel_linear2_z wrt acc_linear1
      jacobian(8, 2) = dt;
      // partial derivatives of acc_linear2_x wrt acc_linear1
      jacobian(12, 0) = 1.0;
      // partial derivatives of acc_linear2_y wrt acc_linear1
      jacobian(13, 1) = 1.0;
      // partial derivatives of acc_linear2_z wrt acc_linear1
      jacobian(14, 2) = 1.0;
    }
  }
}

/**
 * @brief Given a state and time delta, predicts a new state
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
  const Eigen::Quaterniond & orientation1,
  const fuse_core::Vector3d & vel_linear1,
  const fuse_core::Vector3d & vel_angular1,
  const fuse_core::Vector3d & acc_linear1,
  const double dt,
  fuse_core::Vector3d & position2,
  Eigen::Quaterniond & orientation2,
  fuse_core::Vector3d & vel_linear2,
  fuse_core::Vector3d & vel_angular2,
  fuse_core::Vector3d & acc_linear2)
{
  fuse_core::Vector3d rpy(
    fuse_core::getRoll(orientation1.w(), orientation1.x(), orientation1.y(), orientation1.z()),
    fuse_core::getPitch(orientation1.w(), orientation1.x(), orientation1.y(), orientation1.z()),
    fuse_core::getYaw(orientation1.w(), orientation1.x(), orientation1.y(), orientation1.z())
  );

  predict(
    position1.x(),
    position1.y(),
    position1.z(),
    rpy.x(),
    rpy.y(),
    rpy.z(),
    vel_linear1.x(),
    vel_linear1.y(),
    vel_linear1.z(),
    vel_angular1.x(),
    vel_angular1.y(),
    vel_angular1.z(),
    acc_linear1.x(),
    acc_linear1.y(),
    acc_linear1.z(),
    dt,
    position2.x(),
    position2.y(),
    position2.z(),
    rpy.x(),
    rpy.y(),
    rpy.z(),
    vel_linear2.x(),
    vel_linear2.y(),
    vel_linear2.z(),
    vel_angular2.x(),
    vel_angular2.y(),
    vel_angular2.z(),
    acc_linear2.x(),
    acc_linear2.y(),
    acc_linear2.z());

  // Convert back to quaternion
  orientation2 = Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX());
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
 * @param[out] jacobian - Jacobian wrt the state
 */
inline void predict(
  const fuse_core::Vector3d & position1,
  const Eigen::Quaterniond & orientation1,
  const fuse_core::Vector3d & vel_linear1,
  const fuse_core::Vector3d & vel_angular1,
  const fuse_core::Vector3d & acc_linear1,
  const double dt,
  fuse_core::Vector3d & position2,
  Eigen::Quaterniond & orientation2,
  fuse_core::Vector3d & vel_linear2,
  fuse_core::Vector3d & vel_angular2,
  fuse_core::Vector3d & acc_linear2,
  fuse_core::Matrix15d & jacobian)
{

  double quat[4] = {orientation1.w(), orientation1.x(), orientation1.y(), orientation1.z()};
  double rpy[3];
  double jacobian_quat2rpy[12];
  fuse_core::quaternion2rpy(quat, rpy, jacobian_quat2rpy);

  // fuse_core::Matrix15d is Eigen::RowMajor, so we cannot use pointers to the columns where each
  // parameter block starts. Instead, we need to create a vector of Eigen::RowMajor matrices per
  // parameter block and later reconstruct the fuse_core::Matrix15d with the full jacobian. The
  // parameter blocks have the following sizes: {position1: 3, orientation1: 4, vel_linear1: 3, 
  // vel_angular1: 3, acc_linear1: 3}

  static constexpr size_t num_residuals{15};
  static constexpr size_t num_parameter_blocks{5};
  static const std::array<size_t, num_parameter_blocks> block_sizes = {3, 4, 3, 3, 3};

  std::array<fuse_core::MatrixXd, num_parameter_blocks> J;
  std::array<double *, num_parameter_blocks> jacobians;

  for (size_t i = 0; i < num_parameter_blocks; ++i) {
    J[i].resize(num_residuals, block_sizes[i]);
    jacobians[i] = J[i].data();
  }

  predict(
    position1.x(),
    position1.y(),
    position1.z(),
    rpy[0],
    rpy[1],
    rpy[2],
    vel_linear1.x(),
    vel_linear1.y(),
    vel_linear1.z(),
    vel_angular1.x(),
    vel_angular1.y(),
    vel_angular1.z(),
    acc_linear1.x(),
    acc_linear1.y(),
    acc_linear1.z(),
    dt,
    position2.x(),
    position2.y(),
    position2.z(),
    rpy[0],
    rpy[1],
    rpy[2],
    vel_linear2.x(),
    vel_linear2.y(),
    vel_linear2.z(),
    vel_angular2.x(),
    vel_angular2.y(),
    vel_angular2.z(),
    acc_linear2.x(),
    acc_linear2.y(),
    acc_linear2.z(),
    jacobians.data(),
    jacobian_quat2rpy);

  jacobian << J[0], J[1], J[2], J[3], J[4];

  // Convert back to quaternion
  orientation2 = Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX());
}

}  // namespace fuse_models

#endif  // FUSE_MODELS__UNICYCLE_3D_PREDICT_HPP_
