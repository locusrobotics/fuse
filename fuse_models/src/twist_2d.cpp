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
#include <fuse_models/common/sensor_proc.h>
#include <fuse_models/twist_2d.h>

#include <fuse_core/transaction.hpp>
#include <fuse_core/uuid.hpp>

#include <geometry_msgs/msg/TwistWithCovarianceStamped.hpp>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>


// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(fuse_models::Twist2D, fuse_core::SensorModel)

namespace fuse_models
{

Twist2D::Twist2D() :
  fuse_core::AsyncSensorModel(1),
  device_id_(fuse_core::uuid::NIL),
  tf_listener_(tf_buffer_),
  throttled_callback_(std::bind(&Twist2D::process, this, std::placeholders::_1))
{
}

void Twist2D::onInit()
{
  // Read settings from the parameter sever
  device_id_ = fuse_variables::loadDeviceId(private_node_handle_);

  params_.loadFromROS(private_node_handle_);

  throttled_callback_.setThrottlePeriod(params_.throttle_period);

  if (!params_.throttle_use_wall_time) {
    throttled_callback_.setClock(node_->get_clock());
  }

  if (params_.linear_indices.empty() &&
      params_.angular_indices.empty())
  {
    RCLCPP_WARN_STREAM(node_->get_logger(),
                       "No dimensions were specified. Data from topic " << ros::names::resolve(params_.topic)
                       << " will be ignored.");
  }
}

void Twist2D::onStart()
{
  if (!params_.linear_indices.empty() ||
      !params_.angular_indices.empty())
  {
    subscriber_ = node_handle_.subscribe<geometry_msgs::msg::TwistWithCovarianceStamped>(
        ros::names::resolve(params_.topic), params_.queue_size, &TwistThrottledCallback::callback, &throttled_callback_,
        ros::TransportHints().tcpNoDelay(params_.tcp_no_delay));
  }
}

void Twist2D::onStop()
{
  subscriber_.shutdown();
}

void Twist2D::process(const geometry_msgs::msg::TwistWithCovarianceStamped& msg)
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
    tf_buffer_,
    !params_.disable_checks,
    *transaction,
    params_.tf_timeout);

  // Send the transaction object to the plugin's parent
  sendTransaction(transaction);
}

}  // namespace fuse_models
