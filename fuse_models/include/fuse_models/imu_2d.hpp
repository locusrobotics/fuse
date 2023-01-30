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
#ifndef FUSE_MODELS__IMU_2D_HPP_
#define FUSE_MODELS__IMU_2D_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>

#include <fuse_models/parameters/imu_2d_params.hpp>
#include <fuse_core/throttled_callback.hpp>

#include <fuse_core/async_sensor_model.hpp>
#include <fuse_core/uuid.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>


namespace fuse_models
{

/**
 * @brief An adapter-type sensor that produces orientation (relative or absolute), angular velocity,
 *        and linear acceleration constraints from IMU sensor data published by another node
 *
 * This sensor subscribes to a sensor_msgs::msg::Imu topic and:
 * 1. Creates relative or absolute orientation and constraints. If the \p differential parameter
 *    is set to false (the default), the orientation measurement will be treated as an absolute
 *    constraint. If it is set to true, consecutive measurements will be used to generate relative
 *    orientation constraints.
 * 2. Creates 2D velocity variables and constraints.
 *
 * This sensor really just separates out the orientation, angular velocity, and linear acceleration
 * components of the message, and processes them just like the Pose2D, Twist2D, and Acceleration2D
 * classes.
 *
 * Parameters:
 *  - device_id (uuid string, default: 00000000-0000-0000-0000-000000000000) The device/robot ID to
 *                                                                           publish
 *  - device_name (string) Used to generate the device/robot ID if the device_id is not provided
 *  - queue_size (int, default: 10) The subscriber queue size for the pose messages
 *  - topic (string) The topic to which to subscribe for the pose messages
 *  - differential (bool, default: false) Whether we should fuse orientation measurements
 *                                        absolutely, or to create relative orientation constraints
 *                                        using consecutive measurements.
 *  - remove_gravitational_acceleration (bool, default: false) Whether we should remove acceleration
 *                                                             due to gravity from the acceleration
 *                                                             values produced by the IMU before
 *                                                             fusing
 *  - gravitational_acceleration (double, default: 9.80665) Acceleration due to gravity, in
 *                                                          meters/sec^2. This value is only used if
 *                                                          \p remove_gravitational_acceleration is
 *                                                          true
 *  - orientation_target_frame (string) Orientation data will be transformed into this frame before
 *                                      it is fused.
 *  - twist_target_frame (string) Twist/velocity data will be transformed into this frame before it
 *                                is fused.
 *  - acceleration_target_frame (string) Acceleration data will be transformed into this frame
 *                                       before it is fused.
 *
 * Subscribes:
 *  - \p topic (sensor_msgs::msg::Imu) IMU data at a given timestep
 */
class Imu2D : public fuse_core::AsyncSensorModel
{
public:
  FUSE_SMART_PTR_DEFINITIONS(Imu2D)
  using ParameterType = parameters::Imu2DParams;

  /**
   * @brief Default constructor
   */
  Imu2D();

  /**
   * @brief Destructor
   */
  virtual ~Imu2D() = default;

  /**
   * @brief Shadowing extension to the AsyncSensorModel::initialize call
   */
  void initialize(
    fuse_core::node_interfaces::NodeInterfaces<ALL_FUSE_CORE_NODE_INTERFACES> interfaces,
    const std::string & name,
    fuse_core::TransactionCallback transaction_callback) override;

  /**
   * @brief Callback for pose messages
   * @param[in] msg - The IMU message to process
   */
  void process(const sensor_msgs::msg::Imu & msg);

protected:
  fuse_core::UUID device_id_;  //!< The UUID of this device

  /**
   * @brief Perform any required initialization for the sensor model
   *
   * This could include things like reading from the parameter server or subscribing to topics. The
   * class's node handles will be properly initialized before SensorModel::onInit() is called.
   * Spinning of the callback queue will not begin until after the call to SensorModel::onInit()
   * completes.
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

  /**
   * @brief Process a pose message in differential mode
   *
   * @param[in] pose - The pose message to process in differential mode
   * @param[in] twist - The twist message used in case the twist covariance is used in differential
   *                    mode
   * @param[in] validate - Whether to validate the pose and twist coavriance or not
   * @param[out] transaction - The generated variables and constraints are added to this transaction
   */
  void processDifferential(
    const geometry_msgs::msg::PoseWithCovarianceStamped & pose,
    const geometry_msgs::msg::TwistWithCovarianceStamped & twist, const bool validate,
    fuse_core::Transaction & transaction);

  fuse_core::node_interfaces::NodeInterfaces<
    fuse_core::node_interfaces::Base,
    fuse_core::node_interfaces::Clock,
    fuse_core::node_interfaces::Logging,
    fuse_core::node_interfaces::Parameters,
    fuse_core::node_interfaces::Topics,
    fuse_core::node_interfaces::Waitables
  > interfaces_;  //!< Shadows AsyncSensorModel interfaces_

  rclcpp::Clock::SharedPtr clock_;  //!< The sensor model's clock, for timestamping and logging
  rclcpp::Logger logger_;  //!< The sensor model's logger

  ParameterType params_;

  geometry_msgs::msg::PoseWithCovarianceStamped::UniquePtr previous_pose_;

  // NOTE(CH3): Unique ptr to defer till we have the node interfaces from initialize()
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;

  using ImuThrottledCallback = fuse_core::ThrottledMessageCallback<sensor_msgs::msg::Imu>;
  ImuThrottledCallback throttled_callback_;
};

}  // namespace fuse_models

#endif  // FUSE_MODELS__IMU_2D_HPP_
