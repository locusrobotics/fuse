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
#include <fuse_publishers/path_2d_publisher.h>
#include <fuse_core/async_publisher.hpp>
#include <fuse_core/graph.hpp>
#include <fuse_core/transaction.hpp>
#include <fuse_core/uuid.hpp>
#include <fuse_variables/orientation_2d_stamped.hpp>
#include <fuse_variables/position_2d_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


#include <fuse_core/parameter.hpp>

#include <algorithm>
#include <utility>
#include <vector>


// Register this publisher with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(fuse_publishers::Path2DPublisher, fuse_core::Publisher);

namespace fuse_publishers
{

Path2DPublisher::Path2DPublisher() :
  fuse_core::AsyncPublisher(),
  device_id_(fuse_core::uuid::NIL),
  frame_id_("map")
{
}

void Path2DPublisher::initialize(
  fuse_core::node_interfaces::NodeInterfaces<
    fuse_core::node_interfaces::Base,
    fuse_core::node_interfaces::Parameters,
    fuse_core::node_interfaces::Topics,
    fuse_core::node_interfaces::Waitables
  > interfaces,
  const std::string & name,
  size_t thread_count)
{
  interfaces_ = interfaces;
  fuse_core::AsyncPublisher::initialize(interfaces_, name, thread_count);
}

void Path2DPublisher::onInit()
{
  // Configure the publisher
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
  frame_id_ = fuse_core::getParam(interfaces_, "frame_id", frame_id_);

  // Advertise the topic
  rclcpp::PublisherOptions pub_options;
  pub_options.callback_group = cb_group_;

  path_publisher_ = rclcpp::create_publisher<nav_msgs::msg::Path>(interfaces_, name_ + "/path", 1, pub_options);
  pose_array_publisher_ = rclcpp::create_publisher<geometry_msgs::msg::PoseArray>(interfaces_, name_ + "/pose_array", 1, pub_options);
}

void Path2DPublisher::notifyCallback(
  fuse_core::Transaction::ConstSharedPtr /*transaction*/,
  fuse_core::Graph::ConstSharedPtr graph)
{
  // Exit early if no one is listening
  if ((path_publisher_->get_subscription_count() == 0) && (pose_array_publisher_->get_subscription_count() == 0))
  {
    return;
  }
  // Extract all of the 2D pose variables to the path
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  for (const auto& variable : graph->getVariables())
  {
    auto orientation = dynamic_cast<const fuse_variables::Orientation2DStamped*>(&variable);
    if (orientation &&
       (orientation->deviceId() == device_id_))
    {
      const auto& stamp = orientation->stamp();
      auto position_uuid = fuse_variables::Position2DStamped(stamp, device_id_).uuid();
      if (!graph->variableExists(position_uuid))
      {
        continue;
      }
      auto position = dynamic_cast<const fuse_variables::Position2DStamped*>(&graph->getVariable(position_uuid));
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = stamp;
      pose.header.frame_id = frame_id_;
      pose.pose.position.x = position->x();
      pose.pose.position.y = position->y();
      pose.pose.position.z = 0.0;
      pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), orientation->yaw()));
      poses.push_back(std::move(pose));
    }
  }
  // Exit if there are no poses
  if (poses.empty())
  {
    return;
  }
  // Sort the poses by timestamp
  auto compare_stamps = [](const geometry_msgs::msg::PoseStamped& pose1, const geometry_msgs::msg::PoseStamped& pose2)
  {
    if(pose1.header.stamp.sec == pose2.header.stamp.sec){
      return pose1.header.stamp.nanosec < pose2.header.stamp.nanosec;
    }
    else
    {
      return pose1.header.stamp.sec < pose2.header.stamp.sec;
    }


  };
  std::sort(poses.begin(), poses.end(), compare_stamps);
  // Define the header for the aggregate message
  std_msgs::msg::Header header;
  header.stamp = poses.back().header.stamp;
  header.frame_id = frame_id_;
  // Convert the sorted poses into a Path msg
  if (path_publisher_->get_subscription_count() > 0)
  {
    nav_msgs::msg::Path path_msg;
    path_msg.header = header;
    path_msg.poses = poses;
    path_publisher_->publish(path_msg);
  }
  // Convert the sorted poses into a PoseArray msg
  if (pose_array_publisher_->get_subscription_count() > 0)
  {
    geometry_msgs::msg::PoseArray pose_array_msg;
    pose_array_msg.header = header;
    std::transform(poses.begin(),
                   poses.end(),
                   std::back_inserter(pose_array_msg.poses),
                   [](const geometry_msgs::msg::PoseStamped& pose)
                   {
                     return pose.pose;
                   });  // NOLINT(whitespace/braces)
    pose_array_publisher_->publish(pose_array_msg);
  }
}

}  // namespace fuse_publishers
