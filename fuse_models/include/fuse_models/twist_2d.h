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
#ifndef FUSE_MODELS_TWIST_2D_H
#define FUSE_MODELS_TWIST_2D_H

#include <fuse_models/parameters/twist_2d_params.h>
#include <fuse_core/throttled_callback.h>

#include <fuse_core/async_sensor_model.h>
#include <fuse_core/uuid.h>

#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>


namespace fuse_models
{

/**
 * @brief An adapter-type sensor that produces absolute velocity constraints from information published by another node
 *
 * This sensor subscribes to a geometry_msgs::TwistWithCovarianceStamped topic and converts each received message
 * into two absolute velocity constraints (one for linear velocity, and one for angular).
 *
 * Parameters:
 *  - device_id (uuid string, default: 00000000-0000-0000-0000-000000000000) The device/robot ID to publish
 *  - device_name (string) Used to generate the device/robot ID if the device_id is not provided
 *  - queue_size (int, default: 10) The subscriber queue size for the twist messages
 *  - topic (string) The topic to which to subscribe for the twist messages
 *
 * Subscribes:
 *  - \p topic (geometry_msgs::TwistWithCovarianceStamped) Absolute velocity information at a given timestamp
 */
class Twist2D : public fuse_core::AsyncSensorModel
{
public:
  SMART_PTR_DEFINITIONS(Twist2D);
  using ParameterType = parameters::Twist2DParams;

  /**
   * @brief Default constructor
   */
  Twist2D();

  /**
   * @brief Destructor
   */
  virtual ~Twist2D() = default;

  /**
   * @brief Callback for twist messages
   * @param[in] msg - The twist message to process
   */
  void process(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& msg);

protected:
  fuse_core::UUID device_id_;  //!< The UUID of this device

  /**
   * @brief Loads ROS parameters and subscribes to the parameterized topic
   */
  void onInit() override;

  /**
   * @brief Subscribe to the input topic to start sending transactions to the optimizer
   */
  void onStart() override;

  /**
   * @brief Unsubscribe from the input topic to stop sending transactions to the optimizer
   */
  void onStop() override;

  ParameterType params_;

  tf2_ros::Buffer tf_buffer_;

  tf2_ros::TransformListener tf_listener_;

  ros::Subscriber subscriber_;

  using TwistThrottledCallback = fuse_core::ThrottledMessageCallback<geometry_msgs::TwistWithCovarianceStamped>;
  TwistThrottledCallback throttled_callback_;
};

}  // namespace fuse_models

#endif  // FUSE_MODELS_TWIST_2D_H
