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
#include <fuse_core/transaction.hpp>
#include <fuse_core/uuid.hpp>
#include <fuse_models/common/sensor_proc.hpp>
#include <fuse_models/twist_2d.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(fuse_models::Twist2D, fuse_core::SensorModel)

namespace fuse_models
{

Twist2D::Twist2D()
: fuse_core::AsyncSensorModel(1),
  device_id_(fuse_core::uuid::NIL),
  logger_(rclcpp::get_logger("uninitialized")),
  throttled_callback_(std::bind(&Twist2D::process, this, std::placeholders::_1))
{
}

void Twist2D::initialize(
  fuse_core::node_interfaces::NodeInterfaces<ALL_FUSE_CORE_NODE_INTERFACES> interfaces,
  const std::string & name,
  fuse_core::TransactionCallback transaction_callback)
{
  interfaces_ = interfaces;
  fuse_core::AsyncSensorModel::initialize(interfaces, name, transaction_callback);
}

void Twist2D::onInit()
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

  if (params_.linear_indices.empty() &&
    params_.angular_indices.empty())
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

void Twist2D::onStart()
{
  if (!params_.linear_indices.empty() ||
    !params_.angular_indices.empty())
  {
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = cb_group_;

    sub_ = rclcpp::create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      interfaces_,
      params_.topic,
      params_.queue_size,
      std::bind(
        &TwistThrottledCallback::callback<
          const geometry_msgs::msg::TwistWithCovarianceStamped &>,
        &throttled_callback_,
        std::placeholders::_1
      ),
      sub_options
    );
  }
}

void Twist2D::onStop()
{
  sub_.reset();
}

void Twist2D::process(const geometry_msgs::msg::TwistWithCovarianceStamped & msg)
{
  // Create a transaction object
  auto transaction = fuse_core::Transaction::make_shared();
  transaction->stamp(msg.header.stamp);

  common::processTwistWithCovariance(
    name(),
    device_id_,
    msg,
    params_.linear_loss,
    params_.angular_loss,
    params_.target_frame,
    params_.linear_indices,
    params_.angular_indices,
    *tf_buffer_,
    !params_.disable_checks,
    *transaction,
    params_.tf_timeout);

  // Send the transaction object to the plugin's parent
  sendTransaction(transaction);
}

}  // namespace fuse_models
