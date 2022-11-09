/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Locus Robotics
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
#include <fuse_publishers/serialized_publisher.h>

#include <fuse_core/async_publisher.h>
#include <fuse_core/graph.h>
#include <fuse_core/graph_deserializer.h>
#include <fuse_core/parameter.h>
#include <fuse_core/transaction.h>
#include <fuse_core/transaction_deserializer.h>
#include <fuse_msgs/SerializedGraph.h>
#include <fuse_msgs/SerializedTransaction.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>


// Register this publisher with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(fuse_publishers::SerializedPublisher, fuse_core::Publisher);

namespace fuse_publishers
{

SerializedPublisher::SerializedPublisher() :
  fuse_core::AsyncPublisher(1),
  frame_id_("map"),
  graph_publisher_throttled_callback_(
      std::bind(&SerializedPublisher::graphPublisherCallback, this, std::placeholders::_1, std::placeholders::_2))
{
}

void SerializedPublisher::onInit()
{
  // Configure the publisher
  private_node_handle_.getParam("frame_id", frame_id_);

  bool latch = false;
  private_node_handle_.getParam("latch", latch);

  rclcpp::Duration graph_throttle_period{ 0 };
  fuse_core::getPositiveParam(private_node_handle_, "graph_throttle_period", graph_throttle_period, false);

  bool graph_throttle_use_wall_time{ false };
  private_node_handle_.getParam("graph_throttle_use_wall_time", graph_throttle_use_wall_time);

  graph_publisher_throttled_callback_.setThrottlePeriod(graph_throttle_period);
  graph_publisher_throttled_callback_.setUseWallTime(graph_throttle_use_wall_time);

  // Advertise the topics
  graph_publisher_ = private_node_handle_.advertise<fuse_msgs::SerializedGraph>("graph", 1, latch);
  transaction_publisher_ = private_node_handle_.advertise<fuse_msgs::SerializedTransaction>("transaction", 1, latch);
}

void SerializedPublisher::notifyCallback(
  fuse_core::Transaction::ConstSharedPtr transaction,
  fuse_core::Graph::ConstSharedPtr graph)
{
  const auto& stamp = transaction->stamp();
  if (graph_publisher_.getNumSubscribers() > 0)
  {
    graph_publisher_throttled_callback_(graph, stamp);
  }

  if (transaction_publisher_.getNumSubscribers() > 0)
  {
    fuse_msgs::SerializedTransaction msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;
    fuse_core::serializeTransaction(*transaction, msg);
    transaction_publisher_.publish(msg);
  }
}

void SerializedPublisher::graphPublisherCallback(fuse_core::Graph::ConstSharedPtr graph, const rclcpp::Time& stamp) const
{
  fuse_msgs::SerializedGraph msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id_;
  fuse_core::serializeGraph(*graph, msg);
  graph_publisher_.publish(msg);
}

}  // namespace fuse_publishers
