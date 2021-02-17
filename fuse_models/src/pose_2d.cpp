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
#include <fuse_models/pose_2d.h>

#include <fuse_core/transaction.h>
#include <fuse_core/uuid.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <memory>
#include <utility>


// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(fuse_models::Pose2D, fuse_core::SensorModel)

namespace fuse_models
{

Pose2D::Pose2D() :
  fuse_core::AsyncSensorModel(1),
  device_id_(fuse_core::uuid::NIL),
  tf_listener_(tf_buffer_),
  throttled_callback_(std::bind(&Pose2D::process, this, std::placeholders::_1))
{
}

void Pose2D::onInit()
{
  // Read settings from the parameter sever
  device_id_ = fuse_variables::loadDeviceId(private_node_handle_);

  params_.loadFromROS(private_node_handle_);

  throttled_callback_.setThrottlePeriod(params_.throttle_period);
  throttled_callback_.setUseWallTime(params_.throttle_use_wall_time);

  if (params_.position_indices.empty() &&
      params_.orientation_indices.empty())
  {
    ROS_WARN_STREAM("No dimensions were specified. Data from topic " << ros::names::resolve(params_.topic) <<
                    " will be ignored.");
  }
}

void Pose2D::onStart()
{
  if (!params_.position_indices.empty() ||
      !params_.orientation_indices.empty())
  {
    subscriber_ = node_handle_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
        ros::names::resolve(params_.topic), params_.queue_size, &PoseThrottledCallback::callback, &throttled_callback_,
        ros::TransportHints().tcpNoDelay(params_.tcp_no_delay));
  }
}

void Pose2D::onStop()
{
  subscriber_.shutdown();
}

void Pose2D::process(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  // Create a transaction object
  auto transaction = fuse_core::Transaction::make_shared();
  transaction->stamp(msg->header.stamp);

  const bool validate = !params_.disable_checks;

  if (params_.differential)
  {
    processDifferential(*msg, validate, *transaction);
  }
  else
  {
    common::processAbsolutePoseWithCovariance(
      name(),
      device_id_,
      *msg,
      params_.loss,
      params_.target_frame,
      params_.position_indices,
      params_.orientation_indices,
      tf_buffer_,
      validate,
      *transaction,
      params_.tf_timeout);
  }

  // Send the transaction object to the plugin's parent
  sendTransaction(transaction);
}

void Pose2D::processDifferential(const geometry_msgs::PoseWithCovarianceStamped& pose, const bool validate,
                                 fuse_core::Transaction& transaction)
{
  auto transformed_pose = std::make_unique<geometry_msgs::PoseWithCovarianceStamped>();
  transformed_pose->header.frame_id = params_.target_frame.empty() ? pose.header.frame_id : params_.target_frame;

  if (!common::transformMessage(tf_buffer_, pose, *transformed_pose))
  {
    ROS_WARN_STREAM_THROTTLE(5.0, "Cannot transform pose message with stamp "
                                      << pose.header.stamp << " to target frame " << params_.target_frame);
    return;
  }

  if (previous_pose_msg_)
  {
    common::processDifferentialPoseWithCovariance(
      name(),
      device_id_,
      *previous_pose_msg_,
      *transformed_pose,
      params_.independent,
      params_.minimum_pose_relative_covariance,
      params_.loss,
      params_.position_indices,
      params_.orientation_indices,
      validate,
      transaction);
  }

  previous_pose_msg_ = std::move(transformed_pose);
}

}  // namespace fuse_models
