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
}

void Path2DPublisher::notifyCallback(
  fuse_core::Transaction::ConstSharedPtr transaction,
  fuse_core::Graph::ConstSharedPtr graph)
{
  // Exit early if no one is listening
  if (path_publisher_.getNumSubscribers() == 0)
  {
    return;
  }

  // Get the type strings for the variables of interest
  auto orientation_type = fuse_variables::Orientation2DStamped(ros::Time()).type();
  auto position_type = fuse_variables::Position2DStamped(ros::Time()).type();

  // Extract and sort all of the 2D pose variables to the path
  std::map<ros::Time, geometry_msgs::PoseStamped> poses;
  for (const auto& variable : graph->getVariables())
  {
    if (variable.type() == orientation_type)
    {
      try
      {
        auto orientation_variable = dynamic_cast<const fuse_variables::Orientation2DStamped&>(variable);
        if (orientation_variable.deviceId() == device_id_)
        {
          auto& pose = poses[orientation_variable.stamp()];
          pose.header.stamp = orientation_variable.stamp();
          pose.header.frame_id = frame_id_;    
          pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), orientation_variable.yaw()));
        }
      }
      catch (const std::exception& e)
      {
        ROS_WARN_STREAM_THROTTLE(10.0, "Failed to cast variable to type '" << orientation_type <<
                                      "' even though the variable has the expected type() string.");
        return;
      }
    }
    else if (variable.type() == position_type)
    {
      try
      {
        auto position_variable = dynamic_cast<const fuse_variables::Position2DStamped&>(variable);
        if (position_variable.deviceId() == device_id_)
        {
          auto& pose = poses[position_variable.stamp()];
          pose.header.stamp = position_variable.stamp();
          pose.header.frame_id = frame_id_;    
          pose.pose.position.x = position_variable.x();
          pose.pose.position.y = position_variable.y();
          pose.pose.position.z = 0.0;
        }
      }
      catch (const std::exception& e)
      {
        ROS_WARN_STREAM_THROTTLE(10.0, "Failed to cast variable to type '" << position_type <<
                                      "' even though the variable has the expected type() string.");
        return;
      }
    }
  }
  // Convert the sorted poses into a Path msg
  nav_msgs::Path path_msg;
  std::transform(poses.begin(),
                 poses.end(),
                 std::back_inserter(path_msg.poses),
                 [](const std::pair<ros::Time, geometry_msgs::PoseStamped>& stamp__pose)
                 {
                   return stamp__pose.second;
                 });
  if (path_msg.poses.empty())
  {
    path_msg.header.stamp = ros::Time::now();
  }
  else
  {
    path_msg.header.stamp = path_msg.poses.back().header.stamp;
  }
  path_msg.header.frame_id = frame_id_;

  // Publish the message
  path_publisher_.publish(path_msg);
}

}  // namespace fuse_publishers
