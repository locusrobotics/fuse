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
#include <fuse_core/async_publisher.h>
#include <fuse_core/graph.h>
#include <fuse_core/transaction.h>
#include <fuse_core/uuid.h>
#include <fuse_publishers/pose_2d_publisher.h>
#include <fuse_variables/position_2d_stamped.h>
#include <fuse_variables/orientation_2d_stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <locus_cpp/ros_util.h>
#include <locus_cpp/tf_util.h>

#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <iterator>
#include <memory>
#include <string>
#include <utility>
#include <vector>


// Register this publisher with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(fuse_publishers::Pose2DPublisher, fuse_core::Publisher);

static const ros::Time TIME_ZERO = ros::Time(0, 0);

namespace fuse_publishers
{

Pose2DPublisher::Pose2DPublisher() :
  fuse_core::AsyncPublisher(1),
  device_id_(fuse_core::uuid::NIL),
  orientation_stamp_(0, 0),
  orientation_uuid_(fuse_core::uuid::NIL),
  position_stamp_(0, 0),
  position_uuid_(fuse_core::uuid::NIL),
  publish_to_tf_(false),
  use_tf_lookup_(false)
{
}

void Pose2DPublisher::onInit()
{
  // Read configuration from the parameter server
  private_node_handle_.param("base_frame", base_frame_, std::string("base_link"));
  private_node_handle_.param("map_frame", map_frame_, std::string("map"));
  private_node_handle_.param("odom_frame", odom_frame_, std::string("odom"));
  std::string device_str;
  if (private_node_handle_.getParam("device_id", device_str))
  {
    device_id_ = fuse_core::uuid::from_string(device_str);
  }
  else if (private_node_handle_.getParam("device_name", device_str))
  {
    device_id_ = fuse_core::uuid::generate(device_str);
  }
  private_node_handle_.param("publish_to_tf", publish_to_tf_, false);

  // Configure tf, if requested
  if (publish_to_tf_)
  {
    use_tf_lookup_ = (!odom_frame_.empty() && (odom_frame_ != base_frame_));
    if (use_tf_lookup_)
    {
      double tf_cache_time;
      double default_tf_cache_time = 10.0;
      private_node_handle_.param("tf_cache_time", tf_cache_time, default_tf_cache_time);
      if (tf_cache_time <= 0)
      {
        ROS_WARN_STREAM("The requested tf_cache_time is <= 0. Using the default value (" <<
                        default_tf_cache_time << "s) instead.");
        tf_cache_time = default_tf_cache_time;
      }

      double tf_timeout;
      double default_tf_timeout = 0.1;
      private_node_handle_.param("tf_timeout", tf_timeout, default_tf_timeout);
      if (tf_timeout <= 0)
      {
        ROS_WARN_STREAM("The requested tf_timeout is <= 0. Using the default value (" <<
                        default_tf_timeout << "s) instead.");
        tf_timeout = default_tf_timeout;
      }
      tf_timeout_ = ros::Duration(tf_timeout);

      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(ros::Duration(tf_cache_time));
      tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, node_handle_);
    }

    double tf_publish_frequency;
    double default_tf_publish_frequency = 10.0;
    private_node_handle_.param("tf_publish_frequency", tf_publish_frequency, default_tf_publish_frequency);
    if (tf_publish_frequency <= 0)
    {
      ROS_WARN_STREAM("The requested tf_publish_frequency is <= 0. Using the default value (" <<
                      default_tf_publish_frequency << "hz) instead.");
      tf_publish_frequency = default_tf_publish_frequency;
    }
    tf_publish_timer_ = private_node_handle_.createTimer(
      ros::Duration(1.0 / tf_publish_frequency), &Pose2DPublisher::tfPublishTimerCallback, this);
  }

  // Advertise the topics
  pose_publisher_ = private_node_handle_.advertise<geometry_msgs::PoseStamped>("pose", 1);
  pose_with_covariance_publisher_ = private_node_handle_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
    "pose_with_covariance", 1);
}

void Pose2DPublisher::notifyCallback(
  fuse_core::Transaction::ConstSharedPtr transaction,
  fuse_core::Graph::ConstSharedPtr graph)
{
  // Clear the latest variable if it points to a removed variable
  if (orientation_stamp_ != TIME_ZERO && position_stamp_ != TIME_ZERO)
  {
    for (const auto& removed_variable_uuid : transaction->removedVariables())
    {
      if ((orientation_uuid_ == removed_variable_uuid) || (position_uuid_ == removed_variable_uuid))
      {
        orientation_stamp_ = TIME_ZERO;
        orientation_uuid_ = fuse_core::uuid::NIL;
        position_stamp_ = TIME_ZERO;
        position_uuid_ = fuse_core::uuid::NIL;
        break;
      }
    }
  }
  // Look through the added variables, looking for the most recent timestamp
  auto orientation_type = fuse_variables::Orientation2DStamped(TIME_ZERO).type();
  auto position_type = fuse_variables::Position2DStamped(TIME_ZERO).type();
  for (const auto& added_variable : transaction->addedVariables())
  {
    if (added_variable->type() == orientation_type)
    {
      auto orientation_variable = std::dynamic_pointer_cast<const fuse_variables::Orientation2DStamped>(added_variable);
      if (!static_cast<bool>(orientation_variable))
      {
        ROS_WARN_STREAM_THROTTLE(10.0, "Failed to cast variable to type '" << orientation_type <<
                                       "' even though the variable has the expected type() string.");
        continue;
      }
      if (orientation_variable->deviceId() == device_id_ && orientation_variable->stamp() >= orientation_stamp_)
      {
        orientation_uuid_ = orientation_variable->uuid();
        orientation_stamp_ = orientation_variable->stamp();
      }
    }
    else if (added_variable->type() == position_type)
    {
      auto position_variable = std::dynamic_pointer_cast<const fuse_variables::Position2DStamped>(added_variable);
      if (!static_cast<bool>(position_variable))
      {
        ROS_WARN_STREAM_THROTTLE(10.0, "Failed to cast variable to type '" << position_type <<
                                       "' even though the variable has the expected type() string.");
        continue;
      }
      if (position_variable->deviceId() == device_id_ && position_variable->stamp() >= position_stamp_)
      {
        position_uuid_ = position_variable->uuid();
        position_stamp_ = position_variable->stamp();
      }
    }
  }
  if (orientation_stamp_ == TIME_ZERO ||
      position_stamp_ == TIME_ZERO ||
      orientation_stamp_ != position_stamp_)
  {
    ROS_WARN_STREAM_THROTTLE(10.0, "Failed to find a matching set of position and orientation variables.");
    return;
  }
  // Convert the pose variable into ROS types
  geometry_msgs::Point translation;
  geometry_msgs::Quaternion rotation;
  try
  {
    auto orientation_variable = dynamic_cast<const fuse_variables::Orientation2DStamped&>(
      graph->getVariable(orientation_uuid_));
    rotation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), orientation_variable.yaw()));
  }
  catch (const std::exception& e)
  {
    ROS_WARN_STREAM_THROTTLE(10.0, "Failed to cast variable to type '" << orientation_type <<
                                   "' even though the variable has the expected type() string.");
    return;
  }
  try
  {
    auto position_variable = dynamic_cast<const fuse_variables::Position2DStamped&>(
      graph->getVariable(position_uuid_));
    translation.x = position_variable.x();
    translation.y = position_variable.y();
    translation.z = 0.0;
  }
  catch (const std::exception& e)
  {
    ROS_WARN_STREAM_THROTTLE(10.0, "Failed to cast variable to type '" << position_type <<
                                   "' even though the variable has the expected type() string.");
    return;
  }
  // Publish the various message types
  if (publish_to_tf_)
  {
    // Create a 3D ROS Transform message from the current 2D pose
    geometry_msgs::TransformStamped map_to_base;
    map_to_base.header.stamp = orientation_stamp_;
    map_to_base.header.frame_id = map_frame_;
    map_to_base.child_frame_id = base_frame_;
    map_to_base.transform.translation.x = translation.x;  // Transforms use Vector3 instead of Point (shakes fist)
    map_to_base.transform.translation.y = translation.y;
    map_to_base.transform.translation.z = translation.z;
    map_to_base.transform.rotation = rotation;
    // If we are suppose to publish the map->odom frame instead, do that transformation now
    if (use_tf_lookup_)
    {
      // We need to lookup the base->odom frame first, so we can compute the map->odom transform from the
      // map->base transform
      try
      {
        auto base_to_odom = tf_buffer_->lookupTransform(base_frame_, odom_frame_, orientation_stamp_, tf_timeout_);
        geometry_msgs::TransformStamped map_to_odom;
        tf2::doTransform(base_to_odom, map_to_odom, map_to_base);
        map_to_odom.child_frame_id = odom_frame_;  // The child frame is not populated for some reason
        tf_transform_ = map_to_odom;
      }
      catch (const std::exception& e)
      {
        ROS_WARN_STREAM_THROTTLE(2.0, "Could not lookup the transform " << base_frame_ << "->" << odom_frame_ <<
                                      ". Error: " << e.what());
      }
    }
    else
    {
      // Simple. No intermediate frame. Just use the optimized map->base transform.
      tf_transform_ = map_to_base;
    }
  }
  if (pose_publisher_.getNumSubscribers() > 0)
  {
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = orientation_stamp_;
    msg.header.frame_id = map_frame_;
    msg.pose.position = translation;
    msg.pose.orientation = rotation;
    pose_publisher_.publish(msg);
  }
  if (pose_with_covariance_publisher_.getNumSubscribers() > 0)
  {
    // Get the covariance from the graph
    std::vector<std::pair<fuse_core::UUID, fuse_core::UUID>> requests;
    requests.emplace_back(position_uuid_, position_uuid_);
    requests.emplace_back(position_uuid_, orientation_uuid_);
    requests.emplace_back(orientation_uuid_, orientation_uuid_);
    std::vector<std::vector<double>> covariance_blocks;
    graph->getCovariance(requests, covariance_blocks);
    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.header.stamp = orientation_stamp_;
    msg.header.frame_id = map_frame_;
    msg.pose.pose.position = translation;
    msg.pose.pose.orientation = rotation;
    msg.pose.covariance[0] = covariance_blocks[0][0];
    msg.pose.covariance[1] = covariance_blocks[0][1];
    msg.pose.covariance[6] = covariance_blocks[0][2];
    msg.pose.covariance[7] = covariance_blocks[0][3];
    msg.pose.covariance[5] = covariance_blocks[1][0];
    msg.pose.covariance[11] = covariance_blocks[1][1];
    msg.pose.covariance[30] = covariance_blocks[1][0];
    msg.pose.covariance[31] = covariance_blocks[1][1];
    msg.pose.covariance[35] = covariance_blocks[2][0];
    pose_with_covariance_publisher_.publish(msg);
  }
}

void Pose2DPublisher::tfPublishTimerCallback(const ros::TimerEvent& event)
{
  // The tf_transform_ is updated in a separate thread, so we must guard the read/write operations.
  // Only publish if the tf transform is valid
  if (tf_transform_.header.stamp != TIME_ZERO)
  {
    // Update the timestamp of the transform so the tf tree will continue to be valid
    tf_transform_.header.stamp = event.current_real;
    tf_publisher_.sendTransform(tf_transform_);
  }
}

}  // namespace fuse_publishers
