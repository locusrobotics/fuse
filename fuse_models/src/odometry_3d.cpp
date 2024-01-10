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
#include <memory>
#include <utility>

#include <fuse_core/transaction.hpp>
#include <fuse_core/uuid.hpp>
#include <fuse_models/common/sensor_proc.hpp>
#include <fuse_models/odometry_3d.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(fuse_models::Odometry3D, fuse_core::SensorModel)

namespace fuse_models
{

Odometry3D::Odometry3D()
: fuse_core::AsyncSensorModel(1),
  device_id_(fuse_core::uuid::NIL),
  logger_(rclcpp::get_logger("uninitialized")),
  throttled_callback_(std::bind(&Odometry3D::process, this, std::placeholders::_1))
{
}

void Odometry3D::initialize(
  fuse_core::node_interfaces::NodeInterfaces<ALL_FUSE_CORE_NODE_INTERFACES> interfaces,
  const std::string & name,
  fuse_core::TransactionCallback transaction_callback)
{
  interfaces_ = interfaces;
  fuse_core::AsyncSensorModel::initialize(interfaces, name, transaction_callback);
}

void Odometry3D::onInit()
{
  logger_ = interfaces_.get_node_logging_interface()->get_logger();
  clock_ = interfaces_.get_node_clock_interface()->get_clock();

  // Read settings from the parameter sever
  device_id_ = fuse_variables::loadDeviceId(interfaces_);

  params_.loadFromROS(interfaces_, name_);

  throttled_callback_.setThrottlePeriod(params_.throttle_period);

  if (!params_.throttle_use_wall_time) {
    throttled_callback_.setClock(clock_);
  }

  if (params_.position_indices.empty() && params_.orientation_indices.empty() &&
    params_.linear_velocity_indices.empty() && params_.angular_velocity_indices.empty())
  {
    RCLCPP_WARN_STREAM(
      logger_,
      "No dimensions were specified. Data from topic " << params_.topic
                                                       << " will be ignored.");
  }

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(clock_);
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(
    *tf_buffer_,
    interfaces_.get_node_base_interface(),
    interfaces_.get_node_logging_interface(),
    interfaces_.get_node_parameters_interface(),
    interfaces_.get_node_topics_interface()
  );
}

void Odometry3D::onStart()
{
  if (!params_.position_indices.empty() || !params_.orientation_indices.empty() ||
    !params_.linear_velocity_indices.empty() || !params_.angular_velocity_indices.empty())
  {
  previous_pose_.reset();

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = cb_group_;

  sub_ = rclcpp::create_subscription<nav_msgs::msg::Odometry>(
    interfaces_,
    params_.topic,
    params_.queue_size,
    std::bind(
      &OdometryThrottledCallback::callback<
        const nav_msgs::msg::Odometry &>,
      &throttled_callback_,
      std::placeholders::_1
    ),
    sub_options
  );
  }
}

void Odometry3D::onStop()
{
  sub_.reset();
}

void Odometry3D::process(const nav_msgs::msg::Odometry & msg)
{
  // Create a transaction object
  auto transaction = fuse_core::Transaction::make_shared();
  transaction->stamp(msg.header.stamp);

  // Handle the pose data
  auto pose = std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>();
  pose->header = msg.header;
  pose->pose = msg.pose;

  geometry_msgs::msg::TwistWithCovarianceStamped twist;
  twist.header = msg.header;
  twist.header.frame_id = msg.child_frame_id;
  twist.twist = msg.twist;

  const bool validate = !params_.disable_checks;

  if (params_.differential) {
    processDifferential(*pose, twist, validate, *transaction);
  } else {
    common::processAbsolutePose3DWithCovariance(
      name(),
      device_id_,
      *pose,
      params_.pose_loss,
      params_.pose_target_frame,
      params_.position_indices,
      params_.orientation_indices,
      *tf_buffer_,
      validate,
      *transaction,
      params_.tf_timeout);
  }

  // Handle the twist data
  common::processTwist3DWithCovariance(
    name(),
    device_id_,
    twist,
    params_.linear_velocity_loss,
    params_.angular_velocity_loss,
    params_.twist_target_frame,
    params_.linear_velocity_indices,
    params_.angular_velocity_indices,
    *tf_buffer_,
    validate,
    *transaction,
    params_.tf_timeout);

  // Send the transaction object to the plugin's parent
  sendTransaction(transaction);
}

void Odometry3D::processDifferential(
  const geometry_msgs::msg::PoseWithCovarianceStamped & pose,
  const geometry_msgs::msg::TwistWithCovarianceStamped & twist,
  const bool validate,
  fuse_core::Transaction & transaction)
{
  auto transformed_pose = std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>();
  transformed_pose->header.frame_id =
    params_.pose_target_frame.empty() ? pose.header.frame_id : params_.pose_target_frame;

  if (!common::transformMessage(*tf_buffer_, pose, *transformed_pose)) {
    RCLCPP_WARN_STREAM_THROTTLE(
      logger_, *clock_, 5.0 * 1000,
      "Cannot transform pose message with stamp "
        << rclcpp::Time(
        pose.header.stamp).nanoseconds() << " to pose target frame " << params_.pose_target_frame);
    return;
  }

  if (!previous_pose_) {
    previous_pose_ = std::move(transformed_pose);
    return;
  }

  if (params_.use_twist_covariance) {
    geometry_msgs::msg::TwistWithCovarianceStamped transformed_twist;
    transformed_twist.header.frame_id =
      params_.twist_target_frame.empty() ? twist.header.frame_id : params_.twist_target_frame;

    if (!common::transformMessage(*tf_buffer_, twist, transformed_twist)) {
      RCLCPP_WARN_STREAM_THROTTLE(
        logger_, *clock_, 5.0 * 1000,
        "Cannot transform twist message with stamp " << rclcpp::Time(
          twist.header.stamp).nanoseconds()
                                                     << " to twist target frame " <<
          params_.twist_target_frame);
    } else {
      common::processDifferentialPose3DWithTwistCovariance(
        name(),
        device_id_,
        *previous_pose_,
        *transformed_pose,
        transformed_twist,
        params_.minimum_pose_relative_covariance,
        params_.twist_covariance_offset,
        params_.pose_loss,
        params_.position_indices,
        params_.orientation_indices,
        validate,
        transaction);
    }
  } else {
    common::processDifferentialPose3DWithCovariance(
      name(),
      device_id_,
      *previous_pose_,
      *transformed_pose,
      params_.independent,
      params_.minimum_pose_relative_covariance,
      params_.pose_loss,
      params_.position_indices,
      params_.orientation_indices,
      validate,
      transaction);
  }
  previous_pose_ = std::move(transformed_pose);
}

}  // namespace fuse_models
