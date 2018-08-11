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
#include <fuse_core/async_publisher.h>
#include <fuse_core/graph.h>
#include <fuse_core/transaction.h>
#include <fuse_core/uuid.h>
#include <fuse_variables/orientation_2d_stamped.h>
#include <fuse_variables/position_2d_stamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <string>
#include <utility>
#include <vector>


// Register this publisher with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(fuse_publishers::Path2DPublisher, fuse_core::Publisher);


// Some file-scope functions in an anonymous namespace
namespace
{

bool checkVariable(
  const fuse_core::Variable& variable,
  const std::string& requested_type,
  const fuse_core::UUID& requested_device,
  ros::Time& output_stamp)
{
  if (variable.type() != requested_type)
  {
    return false;
  }
  try
  {
    auto stamped_variable = dynamic_cast<const fuse_variables::Stamped&>(variable);
    if (stamped_variable.deviceId() != requested_device)
    {
      return false;
    }
    output_stamp = stamped_variable.stamp();
  }
  catch (const std::exception& e)
  {
    ROS_WARN_STREAM_THROTTLE(10.0, "Failed to convert variable to a stamped type. Error" << e.what());
    return false;
  }
  catch (...)
  {
    ROS_WARN_STREAM_THROTTLE(10.0, "Failed to convert variable to a stamped type. Error: unknown");
    return false;
  }
  return true;
}

bool findPose(
  const fuse_core::Graph& graph,
  const ros::Time& stamp,
  const fuse_core::UUID& device_id,
  geometry_msgs::Pose& pose)
{
  try
  {
    auto orientation_variable = dynamic_cast<const fuse_variables::Orientation2DStamped&>(
      graph.getVariable(fuse_variables::Orientation2DStamped(stamp, device_id).uuid()));
    auto position_variable = dynamic_cast<const fuse_variables::Position2DStamped&>(
      graph.getVariable(fuse_variables::Position2DStamped(stamp, device_id).uuid()));
    pose.position.x = position_variable.x();
    pose.position.y = position_variable.y();
    pose.position.z = 0.0;
    pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), orientation_variable.yaw()));
  }
  catch (const std::exception& e)
  {
    ROS_WARN_STREAM_THROTTLE(10.0, "Failed to find a pose at time " << stamp << ". Error" << e.what());
    return false;
  }
  catch (...)
  {
    ROS_WARN_STREAM_THROTTLE(10.0, "Failed to find a pose at time " << stamp << ". Error: unknown");
    return false;
  }
  return true;
}

}  // namespace

namespace fuse_publishers
{

Path2DPublisher::Path2DPublisher() :
  fuse_core::AsyncPublisher(1),
  device_id_(fuse_core::uuid::NIL),
  frame_id_("map")
{
}

void Path2DPublisher::onInit()
{
  // Configure the publisher
  std::string device_str;
  if (private_node_handle_.getParam("device_id", device_str))
  {
    device_id_ = fuse_core::uuid::from_string(device_str);
  }
  else if (private_node_handle_.getParam("device_name", device_str))
  {
    device_id_ = fuse_core::uuid::generate(device_str);
  }
  private_node_handle_.getParam("frame_id", frame_id_);

  // Advertise the topic
  path_publisher_ = private_node_handle_.advertise<nav_msgs::Path>("path", 1);
  pose_array_publisher_ = private_node_handle_.advertise<geometry_msgs::PoseArray>("pose_array", 1);
}

void Path2DPublisher::notifyCallback(
  fuse_core::Transaction::ConstSharedPtr transaction,
  fuse_core::Graph::ConstSharedPtr graph)
{
  // Exit early if no one is listening
  if ((path_publisher_.getNumSubscribers() == 0) && (pose_array_publisher_.getNumSubscribers() == 0))
  {
    return;
  }
  // Extract all of the 2D pose variables to the path
  std::vector<geometry_msgs::PoseStamped> poses;
  for (const auto& variable : graph->getVariables())
  {
    // Use the orientation variable as the "reference" variable
    ros::Time stamp;
    if (checkVariable(variable, fuse_variables::Orientation2DStamped::TYPE, device_id_, stamp))
    {
      geometry_msgs::PoseStamped pose;
      if (findPose(*graph, stamp, device_id_, pose.pose))
      {
        pose.header.stamp = stamp;
        pose.header.frame_id = frame_id_;
        poses.push_back(pose);
      }
    }
  }
  // Exit if there are no poses
  if (poses.empty())
  {
    return;
  }
  // Sort the poses by timestamp
  auto compare_stamps = [](const geometry_msgs::PoseStamped& pose1, const geometry_msgs::PoseStamped& pose2)
  {
    return pose1.header.stamp < pose2.header.stamp;
  };
  std::sort(poses.begin(), poses.end(), compare_stamps);
  // Define the header for the aggregate message
  std_msgs::Header header;
  header.stamp = poses.back().header.stamp;
  header.frame_id = frame_id_;
  // Convert the sorted poses into a Path msg
  if (path_publisher_.getNumSubscribers() > 0)
  {
    nav_msgs::Path path_msg;
    path_msg.header = header;
    path_msg.poses = poses;
    path_publisher_.publish(path_msg);
  }
  // Convert the sorted poses into a PoseArray msg
  if (pose_array_publisher_.getNumSubscribers() > 0)
  {
    geometry_msgs::PoseArray pose_array_msg;
    pose_array_msg.header = header;
    std::transform(poses.begin(),
                   poses.end(),
                   std::back_inserter(pose_array_msg.poses),
                   [](const geometry_msgs::PoseStamped& pose)
                   {
                     return pose.pose;
                   });  // NOLINT(whitespace/braces)
    pose_array_publisher_.publish(pose_array_msg);
  }
}

}  // namespace fuse_publishers
