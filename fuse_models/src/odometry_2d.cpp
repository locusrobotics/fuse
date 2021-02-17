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
#include <fuse_models/odometry_2d.h>

#include <fuse_core/transaction.h>
#include <fuse_core/uuid.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <memory>
#include <utility>


// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(fuse_models::Odometry2D, fuse_core::SensorModel)

namespace fuse_models
{

Odometry2D::Odometry2D() :
  fuse_core::AsyncSensorModel(1),
  device_id_(fuse_core::uuid::NIL),
  tf_listener_(tf_buffer_),
  throttled_callback_(std::bind(&Odometry2D::process, this, std::placeholders::_1))
{
}

void Odometry2D::onInit()
{
  // Read settings from the parameter sever
  device_id_ = fuse_variables::loadDeviceId(private_node_handle_);

  params_.loadFromROS(private_node_handle_);

  throttled_callback_.setThrottlePeriod(params_.throttle_period);
  throttled_callback_.setUseWallTime(params_.throttle_use_wall_time);

  if (params_.position_indices.empty() &&
      params_.orientation_indices.empty() &&
      params_.linear_velocity_indices.empty() &&
      params_.angular_velocity_indices.empty())
  {
    ROS_WARN_STREAM("No dimensions were specified. Data from topic " << ros::names::resolve(params_.topic) <<
                    " will be ignored.");
  }
}

void Odometry2D::onStart()
{
  if (!params_.position_indices.empty() ||
      !params_.orientation_indices.empty() ||
      !params_.linear_velocity_indices.empty() ||
      !params_.angular_velocity_indices.empty())
  {
    previous_pose_.reset();
    subscriber_ = node_handle_.subscribe<nav_msgs::Odometry>(ros::names::resolve(params_.topic), params_.queue_size,
                                                             &OdometryThrottledCallback::callback, &throttled_callback_,
                                                             ros::TransportHints().tcpNoDelay(params_.tcp_no_delay));
  }
}

void Odometry2D::onStop()
{
  subscriber_.shutdown();
}

void Odometry2D::process(const nav_msgs::Odometry::ConstPtr& msg)
{
  // Create a transaction object
  auto transaction = fuse_core::Transaction::make_shared();
  transaction->stamp(msg->header.stamp);

  // Handle the pose data
  auto pose = std::make_unique<geometry_msgs::PoseWithCovarianceStamped>();
  pose->header = msg->header;
  pose->pose = msg->pose;

  geometry_msgs::TwistWithCovarianceStamped twist;
  twist.header = msg->header;
  twist.header.frame_id = msg->child_frame_id;
  twist.twist = msg->twist;

  const bool validate = !params_.disable_checks;

  if (params_.differential)
  {
    processDifferential(*pose, twist, validate, *transaction);
  }
  else
  {
    common::processAbsolutePoseWithCovariance(
      name(),
      device_id_,
      *pose,
      params_.pose_loss,
      params_.pose_target_frame,
      params_.position_indices,
      params_.orientation_indices,
      tf_buffer_,
      validate,
      *transaction,
      params_.tf_timeout);
  }

  // Handle the twist data
  common::processTwistWithCovariance(
    name(),
    device_id_,
    twist,
    params_.linear_velocity_loss,
    params_.angular_velocity_loss,
    params_.twist_target_frame,
    params_.linear_velocity_indices,
    params_.angular_velocity_indices,
    tf_buffer_,
    validate,
    *transaction,
    params_.tf_timeout);

  // Send the transaction object to the plugin's parent
  sendTransaction(transaction);
}

void Odometry2D::processDifferential(const geometry_msgs::PoseWithCovarianceStamped& pose,
                                     const geometry_msgs::TwistWithCovarianceStamped& twist, const bool validate,
                                     fuse_core::Transaction& transaction)
{
  auto transformed_pose = std::make_unique<geometry_msgs::PoseWithCovarianceStamped>();
  transformed_pose->header.frame_id =
      params_.pose_target_frame.empty() ? pose.header.frame_id : params_.pose_target_frame;

  if (!common::transformMessage(tf_buffer_, pose, *transformed_pose))
  {
    ROS_WARN_STREAM_THROTTLE(5.0, "Cannot transform pose message with stamp "
                                      << pose.header.stamp << " to pose target frame " << params_.pose_target_frame);
    return;
  }

  if (!previous_pose_)
  {
    previous_pose_ = std::move(transformed_pose);
    return;
  }

  if (params_.use_twist_covariance)
  {
    geometry_msgs::TwistWithCovarianceStamped transformed_twist;
    transformed_twist.header.frame_id =
        params_.twist_target_frame.empty() ? twist.header.frame_id : params_.twist_target_frame;

    if (!common::transformMessage(tf_buffer_, twist, transformed_twist))
    {
      ROS_WARN_STREAM_THROTTLE(5.0, "Cannot transform twist message with stamp " << twist.header.stamp
                                                                                 << " to twist target frame "
                                                                                 << params_.twist_target_frame);
    }
    else
    {
      common::processDifferentialPoseWithTwistCovariance(
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
  }
  else
  {
    common::processDifferentialPoseWithCovariance(
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
