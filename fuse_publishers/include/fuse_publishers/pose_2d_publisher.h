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
#ifndef FUSE_PUBLISHERS_POSE_2D_PUBLISHER_H
#define FUSE_PUBLISHERS_POSE_2D_PUBLISHER_H

#include <fuse_publishers/stamped_variable_synchronizer.h>

#include <fuse_core/async_publisher.hpp>
#include <fuse_core/graph.hpp>
#include <fuse_core/fuse_macros.hpp>
#include <fuse_core/transaction.hpp>
#include <fuse_core/uuid.hpp>
#include <fuse_variables/orientation_2d_stamped.hpp>
#include <fuse_variables/position_2d_stamped.hpp>

#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>


#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>



#include <memory>
#include <string>


namespace fuse_publishers
{

/**
 * @brief Publisher plugin that publishes the latest 2D pose (combination of Position2DStamped and Orientation2DStamped)
 *
 * There are several options: the latest pose can be sent to the tf topic, just the pose can be published, or the pose
 * and the covariance can be published.
 *
 * The tf operation is not straight forward. The standard ROS frame conventions are map->odom->base_link. The
 * odom->base_link frame is generally published by a separate odometry system (robot driver node, robot_localization
 * node, robot_pose_ekf node, etc.). If this node published a map->base_link transform, the tf system would no longer
 * form a tree (the base_link node would have multiple parents), which breaks the tf system. Instead, we want to
 * publish the map->odom transform. If the map->base_link transform is desired, the odom frame may be set to an empty
 * string. Additionally, the tf system requires transforms to be available both before and after a requested timestamp
 * before it will compute a transform. If this plugin only published transforms after each successful cycle of the
 * optimizer, the map->odom transform will become old or stale as the optimization time increases. To prevent this,
 * the map->odom transform is published in response to an independent timer, and the timestamp of the transform is
 * updated to the current publication time. Although this is "wrong", it keeps the tf tree populated with recent
 * transform data so that other nodes can execute tf queries.
 *
 * Parameters:
 *  - base_frame (string, default: base_link)  Name for the robot's base frame
 *  - device_id (uuid string, default: 00000000-0000-0000-0000-000000000000) The device/robot ID to publish
 *  - device_name (string) Used to generate the device/robot ID if the device_id is not provided
 *  - map_frame (string, default: map)  Name for the robot's map frame
 *  - odom_frame (string, default: odom)  Name for the robot's odom frame (or {empty} if the frame map->base should be
 *                                        published to tf instead of map->odom)
 *  - publish_to_tf (bool, default: false)  Flag indicating that the optimized pose should be published to tf
 *  - tf_cache_time (seconds, default: 10.0)  How long to keep a history of transforms (for map->odom lookup)
 *  - tf_publish_frequency (Hz, default: 10.0)  How often the latest pose should be published to tf
 *  - tf_timeout (seconds, default: 0.1)  The maximum amount of time to wait for a transform to become available
 *
 * Publishes:
 *  - pose (geometry_msgs::msg::PoseStamped)  The most recent optimized robot pose (i.e. the map->base transform)
 *  - pose_with_covariance (geometry_msgs::msg::PoseWithCovarianceStamped)  The most recent optimized robot pose and
 *                                                                     covariance (i.e. the map->base transform)
 *  - tf (tf2_msgs::msg::TFMessage)  The most recent map->odom transform (or map->base if the odom_frame is empty)
 *
 * Subscribes:
 *  - tf, tf_static (tf2_msgs::msg::TFMessage)  Used to lookup the current odom->base frame, if needed
 */
class Pose2DPublisher : public fuse_core::AsyncPublisher
{
public:
  FUSE_SMART_PTR_DEFINITIONS(Pose2DPublisher)

  /**
   * @brief Constructor
   */
  Pose2DPublisher();

  /**
   * @brief Destructor
   */
  virtual ~Pose2DPublisher() = default;

  /**
   * @brief Shadowing extension to the AsyncPublisher::initialize call
   */
  void initialize(
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
    const std::string & name) override;

  /**
   * @brief Perform any required post-construction initialization, such as advertising publishers or reading from the
   * parameter server.
   */
  void onInit() override;

  /**
   * @brief Perform any required operations before the first call to notify() occurs
   */
  void onStart() override;

  /**
   * @brief Perform any required operations to stop publications
   */
  void onStop() override;

  /**
   * @brief Notify the publisher about variables that have been added or removed
   *
   * This publisher publishes only the most recent pose. By analyzing the added and removed variables, the most
   * recent pose UUID can be maintained without performing an exhaustive search through the entire Graph during
   * the Publisher::publish() call.
   *
   * @param[in] transaction A Transaction object, describing the set of variables that have been added and/or removed
   * @param[in] graph       A read-only pointer to the graph object, allowing queries to be performed whenever needed
   */
  void notifyCallback(
    fuse_core::Transaction::ConstSharedPtr transaction,
    fuse_core::Graph::ConstSharedPtr graph) override;

  /**
   * @brief Timer-based callback that publishes the latest map->odom transform
   *
   * The transform is published with the current timestamp, not the timestamp of the data used to generate the
   * transform. Although this is "wrong", it keeps the tf tree populated with recent transforms so that other nodes
   * can execute tf queries.
   */
  void tfPublishTimerCallback();

protected:
  fuse_core::node_interfaces::NodeInterfaces<
    fuse_core::node_interfaces::Base,
    fuse_core::node_interfaces::Clock,
    fuse_core::node_interfaces::Logging,
    fuse_core::node_interfaces::Parameters,
    fuse_core::node_interfaces::Services,
    fuse_core::node_interfaces::Timers,
    fuse_core::node_interfaces::Topics,
    fuse_core::node_interfaces::Waitables
  > interfaces_;  //!< Shadows AsyncPublisher interfaces_

  using Synchronizer = StampedVariableSynchronizer<fuse_variables::Orientation2DStamped,
                                                   fuse_variables::Position2DStamped>;
  std::string base_frame_;  //!< The name of the robot's base_link frame
  fuse_core::UUID device_id_;  //!< The UUID of the device to be published
  rclcpp::Clock::SharedPtr clock_;  //!< The publisher's clock, for timestamping and logging
  rclcpp::Logger logger_;  //!< The publisher's logger, shared_ptr for deferred init

  std::string map_frame_;  //!< The name of the robot's map frame
  std::string odom_frame_;  //!< The name of the odom frame for this pose (or empty if the odom is not used)
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;  //!< Publish the pose as a geometry_msgs::PoseStamped
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_with_covariance_publisher_;  //!< Publish the pose as a geometry_msgs::PoseWithCovarianceStamped
  bool publish_to_tf_;  //!< Flag indicating the pose should be sent to the tf system as well as the pose topics
  Synchronizer::UniquePtr synchronizer_;  //!< Object that tracks the latest common timestamp of multiple variables
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;  //!< TF2 object that supports querying transforms by time and frame id
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;  //!< TF2 object that subscribes to the tf topics and
                                                             //!< inserts the received transforms into the tf buffer
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_publisher_ = nullptr;  //!< Publish the map->odom or map->base transform to the tf system
  rclcpp::TimerBase::SharedPtr tf_publish_timer_;  //!< Timer that publishes tf messages to ensure the tf transform doesn't get stale
  rclcpp::Duration tf_timeout_;  //!< The max time to wait for a tf transform to become available
  geometry_msgs::msg::TransformStamped tf_transform_;  //!< The transform to be published to tf
  bool use_tf_lookup_;  //!< Internal flag indicating that a tf frame lookup is required
};

}  // namespace fuse_publishers

#endif  // FUSE_PUBLISHERS_POSE_2D_PUBLISHER_H
