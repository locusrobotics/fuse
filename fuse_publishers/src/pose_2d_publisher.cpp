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
#include <fuse_publishers/pose_2d_publisher.h>

#include <fuse_core/async_publisher.hpp>
#include <fuse_core/graph.hpp>
#include <fuse_core/node_interfaces/node_interfaces.hpp>
#include <fuse_core/time.hpp>
#include <fuse_core/transaction.hpp>
#include <fuse_core/uuid.hpp>
#include <fuse_variables/orientation_2d_stamped.hpp>
#include <fuse_variables/position_2d_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <fuse_core/parameter.hpp>

#include <chrono>
#include <exception>
#include <utility>
#include <vector>


// Register this publisher with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(fuse_publishers::Pose2DPublisher, fuse_core::Publisher);

// Some file-scope functions in an anonymous namespace
namespace
{

bool findPose(
  rclcpp::Logger logger,
  rclcpp::Clock clock,
  const fuse_core::Graph& graph,
  const rclcpp::Time& stamp,
  const fuse_core::UUID& device_id,
  fuse_core::UUID& orientation_uuid,
  fuse_core::UUID& position_uuid,
  geometry_msgs::msg::Pose& pose
)
{
  try
  {
    orientation_uuid = fuse_variables::Orientation2DStamped(stamp, device_id).uuid();
    auto orientation_variable = dynamic_cast<const fuse_variables::Orientation2DStamped&>(
      graph.getVariable(orientation_uuid));
    position_uuid = fuse_variables::Position2DStamped(stamp, device_id).uuid();
    auto position_variable = dynamic_cast<const fuse_variables::Position2DStamped&>(
      graph.getVariable(position_uuid));
    pose.position.x = position_variable.x();
    pose.position.y = position_variable.y();
    pose.position.z = 0.0;
    pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), orientation_variable.yaw()));
  }
  catch (const std::exception& e)
  {
    RCLCPP_WARN_STREAM_THROTTLE(logger, clock, 10.0 * 1000,
                                "Failed to find a pose at time " << stamp.nanoseconds() << ". Error" << e.what());
    return false;
  }
  catch (...)
  {
    RCLCPP_WARN_STREAM_THROTTLE(logger, clock, 10.0 * 1000,
                                "Failed to find a pose at time " << stamp.nanoseconds() << ". Error: unknown");
    return false;
  }
  return true;
}

}  // namespace

namespace fuse_publishers
{

Pose2DPublisher::Pose2DPublisher() :
  fuse_core::AsyncPublisher(1),
  device_id_(fuse_core::uuid::NIL),
  logger_(rclcpp::get_logger("uninitialized")),
  publish_to_tf_(false),
  tf_timeout_(rclcpp::Duration(0, 0)),
  use_tf_lookup_(false)
{
}

void Pose2DPublisher::initialize(
  fuse_core::node_interfaces::NodeInterfaces<
    fuse_core::node_interfaces::Base,
    fuse_core::node_interfaces::Clock,
    fuse_core::node_interfaces::Graph,
    fuse_core::node_interfaces::Logging,
    fuse_core::node_interfaces::Parameters,
    fuse_core::node_interfaces::Services,
    fuse_core::node_interfaces::TimeSource,
    fuse_core::node_interfaces::Timers,
    fuse_core::node_interfaces::Topics,
    fuse_core::node_interfaces::Waitables
  > interfaces,
  const std::string & name)
{
  interfaces_ = interfaces;
  fuse_core::AsyncPublisher::initialize(interfaces, name);
}

void Pose2DPublisher::onInit()
{
  logger_ = interfaces_.get_node_logging_interface()->get_logger();
  clock_ = interfaces_.get_node_clock_interface()->get_clock();

  tf_publisher_ = std::make_shared<tf2_ros::TransformBroadcaster>(interfaces_);

  // Read configuration from the parameter server
  base_frame_ = fuse_core::getParam(interfaces_, "base_frame", std::string("base_link"));
  map_frame_ = fuse_core::getParam(interfaces_, "map_frame", std::string("map"));
  odom_frame_ = fuse_core::getParam(interfaces_, "odom_frame", std::string("odom"));

  std::string device_str;
  device_str = fuse_core::getParam(interfaces_, "device_id", device_str);
  if (device_str != "")
  {
    device_id_ = fuse_core::uuid::from_string(device_str);
  }
  else{
    device_str = fuse_core::getParam(interfaces_, "device_name", device_str);
    if (device_str != "")
    {
      device_id_ = fuse_core::uuid::generate(device_str);
    }
  }
  publish_to_tf_ = fuse_core::getParam(interfaces_, "publish_to_tf", false);

  // Configure tf, if requested
  if (publish_to_tf_)
  {
    use_tf_lookup_ = (!odom_frame_.empty() && (odom_frame_ != base_frame_));
    if (use_tf_lookup_)
    {
      double tf_cache_time;
      double default_tf_cache_time = 10.0;
      tf_cache_time = fuse_core::getParam(interfaces_, "tf_cache_time", default_tf_cache_time);
      if (tf_cache_time <= 0)
      {
        RCLCPP_WARN_STREAM(
          logger_,
          "The requested tf_cache_time is <= 0. Using the default value ("
          << default_tf_cache_time << "s) instead.");
        tf_cache_time = default_tf_cache_time;
      }

      double tf_timeout;
      double default_tf_timeout = 0.1;
      tf_timeout = fuse_core::getParam(interfaces_, "tf_timeout", default_tf_timeout);
      if (tf_timeout <= 0)
      {
        RCLCPP_WARN_STREAM(
          logger_,
          "The requested tf_timeout is <= 0. Using the default value ("
          << default_tf_timeout << "s) instead.");
        tf_timeout = default_tf_timeout;
      }
      tf_timeout_ = rclcpp::Duration::from_seconds(tf_timeout);

      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(
        clock_,
        rclcpp::Duration::from_seconds(tf_cache_time)
          .to_chrono<std::chrono::nanoseconds>()
        // , interfaces_  // NOTE(methylDragon): This one is pending a change on tf2_ros/buffer.h
                          // TODO(methylDragon): See above ^
      );
      tf_listener_ = std::make_unique<tf2_ros::TransformListener>(
        *tf_buffer_,
        interfaces_.get_node_base_interface(),
        interfaces_.get_node_logging_interface(),
        interfaces_.get_node_parameters_interface(),
        interfaces_.get_node_topics_interface()
      );
    }
  }

  // Advertise the topics
  rclcpp::PublisherOptions pub_options;
  pub_options.callback_group = cb_group_;

  pose_publisher_ = rclcpp::create_publisher<geometry_msgs::msg::PoseStamped>(interfaces_, name_ + "/pose", 1, pub_options);
  pose_with_covariance_publisher_ = rclcpp::create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(interfaces_, name_ + "/pose_with_covariance", 1, pub_options);
}

void Pose2DPublisher::onStart()
{
  // Clear the transform
  tf_transform_ = geometry_msgs::msg::TransformStamped();
  // Clear the synchronizer
  synchronizer_ = Synchronizer::make_unique(device_id_);
  // Start the tf timer
  if (publish_to_tf_)
  {
    double tf_publish_frequency;
    double default_tf_publish_frequency = 10.0;

    // We pull the param again on each start so the publisher can technically get set to a different
    // publish frequency
    tf_publish_frequency = fuse_core::getParam(interfaces_, "tf_publish_frequency", default_tf_publish_frequency);
    if (tf_publish_frequency <= 0)
    {
      RCLCPP_WARN_STREAM(logger_,
                         "The requested tf_publish_frequency is <= 0. Using the default value (" <<
                         default_tf_publish_frequency << "hz) instead.");
      tf_publish_frequency = default_tf_publish_frequency;
    }
    tf_publish_timer_ = rclcpp::create_timer(
      interfaces_,
      clock_,
      std::chrono::duration<double>(1.0 / tf_publish_frequency),
      std::move(std::bind(&Pose2DPublisher::tfPublishTimerCallback, this)),
      cb_group_
    );
  }
}

void Pose2DPublisher::onStop()
{
  // Stop the tf timer
  if (publish_to_tf_)
  {
    tf_publish_timer_->cancel();
  }
}

void Pose2DPublisher::notifyCallback(
  fuse_core::Transaction::ConstSharedPtr transaction,
  fuse_core::Graph::ConstSharedPtr graph)
{
  auto latest_stamp = synchronizer_->findLatestCommonStamp(*transaction, *graph);
  if (!fuse_core::is_valid(latest_stamp))  // If uninitialized
  {
    RCLCPP_WARN_STREAM_THROTTLE(
      logger_, *clock_, 10.0 * 1000,
      "Failed to find a matching set of stamped pose variables with device id '" << device_id_ << "'.");
    return;
  }
  // Get the pose values associated with the selected timestamp
  fuse_core::UUID orientation_uuid;
  fuse_core::UUID position_uuid;
  geometry_msgs::msg::Pose pose;
  if (!findPose(logger_, *clock_, *graph, latest_stamp, device_id_, orientation_uuid, position_uuid, pose))
  {
    return;
  }
  // Publish the various message types
  if (publish_to_tf_)
  {
    // Create a 3D ROS Transform message from the current 2D pose
    geometry_msgs::msg::TransformStamped map_to_base;
    map_to_base.header.stamp = latest_stamp;
    map_to_base.header.frame_id = map_frame_;
    map_to_base.child_frame_id = base_frame_;
    map_to_base.transform.translation.x = pose.position.x;  // Transforms use Vector3 instead of Point (shakes fist)
    map_to_base.transform.translation.y = pose.position.y;
    map_to_base.transform.translation.z = pose.position.z;
    map_to_base.transform.rotation = pose.orientation;
    // If we are suppose to publish the map->odom frame instead, do that transformation now
    if (use_tf_lookup_)
    {
      // We need to lookup the base->odom frame first, so we can compute the map->odom transform from the
      // map->base transform
      try
      {
        auto base_to_odom = tf_buffer_->lookupTransform(base_frame_, odom_frame_, latest_stamp, tf_timeout_);
        geometry_msgs::msg::TransformStamped map_to_odom;
        tf2::doTransform(base_to_odom, map_to_odom, map_to_base);
        map_to_odom.child_frame_id = odom_frame_;  // The child frame is not populated for some reason
        tf_transform_ = map_to_odom;
      }
      catch (const std::exception& e)
      {
        RCLCPP_WARN_STREAM_THROTTLE(
          logger_, *clock_, 2.0 * 1000,
          "Could not lookup the transform " << base_frame_ << "->" << odom_frame_ << ". Error: " << e.what());
      }
    }
    else
    {
      // Simple. No intermediate frame. Just use the optimized map->base transform.
      tf_transform_ = map_to_base;
    }
  }
  if (pose_publisher_->get_subscription_count() > 0)
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = latest_stamp;
    msg.header.frame_id = map_frame_;
    msg.pose = pose;
    pose_publisher_->publish(msg);
  }
  if (pose_with_covariance_publisher_->get_subscription_count() > 0)
  {
    // Get the covariance from the graph
    std::vector<std::pair<fuse_core::UUID, fuse_core::UUID>> requests;
    requests.emplace_back(position_uuid, position_uuid);
    requests.emplace_back(position_uuid, orientation_uuid);
    requests.emplace_back(orientation_uuid, orientation_uuid);
    std::vector<std::vector<double>> covariance_blocks;
    graph->getCovariance(requests, covariance_blocks);
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = latest_stamp;
    msg.header.frame_id = map_frame_;
    msg.pose.pose = pose;
    msg.pose.covariance[0] = covariance_blocks[0][0];
    msg.pose.covariance[1] = covariance_blocks[0][1];
    msg.pose.covariance[6] = covariance_blocks[0][2];
    msg.pose.covariance[7] = covariance_blocks[0][3];
    msg.pose.covariance[5] = covariance_blocks[1][0];
    msg.pose.covariance[11] = covariance_blocks[1][1];
    msg.pose.covariance[30] = covariance_blocks[1][0];
    msg.pose.covariance[31] = covariance_blocks[1][1];
    msg.pose.covariance[35] = covariance_blocks[2][0];
    pose_with_covariance_publisher_->publish(msg);
  }
}

void Pose2DPublisher::tfPublishTimerCallback()
{
  // The tf_transform_ is updated in a separate thread, so we must guard the read/write operations.
  // Only publish if the tf transform is valid
  if (fuse_core::is_valid(tf_transform_.header.stamp))
  {
    // Update the timestamp of the transform so the tf tree will continue to be valid
    tf_transform_.header.stamp = clock_->now();
    tf_publisher_->sendTransform(tf_transform_);
  }
}

}  // namespace fuse_publishers
