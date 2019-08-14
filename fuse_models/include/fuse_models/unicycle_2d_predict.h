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
#ifndef FUSE_MODELS_UNICYCLE_2D_PREDICT_H
#define FUSE_MODELS_UNICYCLE_2D_PREDICT_H

#include <ceres/jet.h>
#include <fuse_core/util.h>
#include <tf2_2d/tf2_2d.h>


namespace fuse_models
{

/**
 * @brief Given a state and time delta, predicts a new state
 * @param[in] position1_x - First X position
 * @param[in] position1_y - First Y position
 * @param[in] yaw1 - First orientation
 * @param[in] vel_linear1_x - First X velocity
 * @param[in] vel_linear1_y - First Y velocity
 * @param[in] vel_yaw1 - First yaw velocity
 * @param[in] acc_linear1_x - First X acceleration
 * @param[in] acc_linear1_y - First Y acceleration
 * @param[in] dt - The time delta across which to predict the state
 * @param[in] position2_x - Second X position
 * @param[in] position2_y - Second Y position
 * @param[in] yaw2 - Second orientation
 * @param[in] vel_linear2_x - Second X velocity
 * @param[in] vel_linear2_y - Second Y velocity
 * @param[in] vel_yaw2 - Second yaw velocity
 * @param[in] acc_linear2_x - Second X acceleration
 * @param[in] acc_linear2_y - Second Y acceleration
 */
template<typename T>
inline void predict(
  const T position1_x,
  const T position1_y,
  const T yaw1,
  const T vel_linear1_x,
  const T vel_linear1_y,
  const T vel_yaw1,
  const T acc_linear1_x,
  const T acc_linear1_y,
  const T dt,
  T& position2_x,
  T& position2_y,
  T& yaw2,
  T& vel_linear2_x,
  T& vel_linear2_y,
  T& vel_yaw2,
  T& acc_linear2_x,
  T& acc_linear2_y)
{
  // There are better models for this projection, but this matches the one used by r_l.
  T sy = ceres::sin(yaw1);  // Should probably be sin((yaw1 + yaw2) / 2), but r_l uses this model
  T cy = ceres::cos(yaw1);
  T delta_x = vel_linear1_x * dt + T(0.5) * acc_linear1_x * dt * dt;
  T delta_y = vel_linear1_y * dt + T(0.5) * acc_linear1_y * dt * dt;

  position2_x = position1_x + cy * delta_x - sy * delta_y;
  position2_y = position1_y + sy * delta_x + cy * delta_y;
  yaw2 = yaw1 + vel_yaw1 * dt;
  vel_linear2_x = vel_linear1_x + acc_linear1_x * dt;
  vel_linear2_y = vel_linear1_y + acc_linear1_y * dt;
  vel_yaw2 = vel_yaw1;
  acc_linear2_x = acc_linear1_x;
  acc_linear2_y = acc_linear1_y;

  fuse_core::wrapAngle2D(yaw2);
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
  const T* const position1,
  const T* const yaw1,
  const T* const vel_linear1,
  const T* const vel_yaw1,
  const T* const acc_linear1,
  const T dt,
  T* const position2,
  T* const yaw2,
  T* const vel_linear2,
  T* const vel_yaw2,
  T* const acc_linear2)
{
  predict(
    position1[0],
    position1[1],
    *yaw1,
    vel_linear1[0],
    vel_linear1[1],
    *vel_yaw1,
    acc_linear1[0],
    acc_linear1[1],
    dt,
    position2[0],
    position2[1],
    *yaw2,
    vel_linear2[0],
    vel_linear2[1],
    *vel_yaw2,
    acc_linear2[0],
    acc_linear2[1]);
}

/**
 * @brief Given a state and time delta, predicts a new state
 * @param[in] pose1 - The first 2D pose
 * @param[in] vel_linear_1 - The first linear velocity
 * @param[in] vel_yaw1 - The first yaw velocity
 * @param[in] acc_linear1 - The first linear acceleration
 * @param[in] dt - The time delta across which to predict the state
 * @param[in] pose2 - The second 2D pose
 * @param[in] vel_linear_2 - The second linear velocity
 * @param[in] vel_yaw2 - The second yaw velocity
 * @param[in] acc_linear2 - The second linear acceleration
 */
inline void predict(
  const tf2_2d::Transform& pose1,
  const tf2_2d::Vector2& vel_linear1,
  const double vel_yaw1,
  const tf2_2d::Vector2& acc_linear1,
  const double dt,
  tf2_2d::Transform& pose2,
  tf2_2d::Vector2& vel_linear2,
  double& vel_yaw2,
  tf2_2d::Vector2& acc_linear2)
{
  double x_pred {};
  double y_pred {};
  double yaw_pred {};
  double vel_linear_x_pred {};
  double vel_linear_y_pred {};
  double acc_linear_x_pred {};
  double acc_linear_y_pred {};

  predict(
    pose1.x(),
    pose1.y(),
    pose1.yaw(),
    vel_linear1.x(),
    vel_linear1.y(),
    vel_yaw1,
    acc_linear1.x(),
    acc_linear1.y(),
    dt,
    x_pred,
    y_pred,
    yaw_pred,
    vel_linear_x_pred,
    vel_linear_y_pred,
    vel_yaw2,
    acc_linear_x_pred,
    acc_linear_y_pred);

  pose2.setX(x_pred);
  pose2.setY(y_pred);
  pose2.setYaw(yaw_pred);
  vel_linear2.setX(vel_linear_x_pred);
  vel_linear2.setY(vel_linear_y_pred);
  acc_linear2.setX(acc_linear_x_pred);
  acc_linear2.setY(acc_linear_y_pred);
}

}  // namespace fuse_models

#endif  // FUSE_MODELS_UNICYCLE_2D_PREDICT_H
