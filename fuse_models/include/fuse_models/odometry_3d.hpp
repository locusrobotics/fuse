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
#ifndef FUSE_MODELS__ODOMETRY_3D_HPP_
#define FUSE_MODELS__ODOMETRY_3D_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>

#include <fuse_models/parameters/odometry_3d_params.hpp>
#include <fuse_core/throttled_callback.hpp>

#include <fuse_core/async_sensor_model.hpp>
#include <fuse_core/uuid.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>


namespace fuse_models
{

/**
 * @brief An adapter-type sensor that produces pose (relative or absolute) and velocity constraints
 *        from sensor data published by another node
 *
 * This sensor subscribes to a nav_msgs::msg::Odometry topic and:
 * 1. Creates relative or absolute pose variables and constraints. If the \p differential parameter
 *    is set to false (the default), the  measurement will be treated as an absolute constraint. If
 *    it is set to true, consecutive measurements will be used to generate relative pose
 *    constraints.
 * 2. Creates 3D velocity variables and constraints.
 *
 * This sensor really just separates out the pose and twist components of the message, and processes
 * them just like the Pose3D and Twist3D classes.
 *
 * Parameters:
 *  - device_id (uuid string, default: 00000000-0000-0000-0000-000000000000) The device/robot ID to
 *                                                                           publish
 *  - device_name (string) Used to generate the device/robot ID if the device_id is not provided
 *  - queue_size (int, default: 10) The subscriber queue size for the pose messages
 *  - topic (string) The topic to which to subscribe for the pose messages
 *  - differential (bool, default: false) Whether we should fuse measurements absolutely, or to
 *                                        create relative pose constraints using consecutive
 *                                        measurements.
 *  - pose_target_frame (string) Pose data will be transformed into this frame before it is fused.
 *                               This frame should be a world-fixed frame, typically 'odom' or
 *                               'map'.
 *  - twist_target_frame (string) Twist/velocity data will be transformed into this frame before it
 *                                is fused. This frame should be a body-relative frame, typically
 *                                'base_link'.
 *
 * Subscribes:
 *  - \p topic (nav_msgs::msg::Odometry) Odometry information at a given timestep
 */
class Odometry3D : public fuse_core::AsyncSensorModel
{
public:
  FUSE_SMART_PTR_DEFINITIONS(Odometry3D)
  using ParameterType = parameters::Odometry3DParams;

  /**
   * @brief Default constructor
   */
  Odometry3D();

  /**
   * @brief Destructor
   */
  virtual ~Odometry3D() = default;

  /**
   * @brief Shadowing extension to the AsyncSensorModel::initialize call
   */
  void initialize(
    fuse_core::node_interfaces::NodeInterfaces<ALL_FUSE_CORE_NODE_INTERFACES> interfaces,
    const std::string & name,
    fuse_core::TransactionCallback transaction_callback) override;

  /**
   * @brief Callback for pose messages
   * @param[in] msg - The pose message to process
   */
  void process(const nav_msgs::msg::Odometry & msg);

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
    const geometry_msgs::msg::TwistWithCovarianceStamped & twist,
    const bool validate,
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

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;

  using OdometryThrottledCallback = fuse_core::ThrottledMessageCallback<nav_msgs::msg::Odometry>;
  OdometryThrottledCallback throttled_callback_;
};

}  // namespace fuse_models

#endif  // FUSE_MODELS__ODOMETRY_3D_HPP_
