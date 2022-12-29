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

#include <fuse_core/async_publisher.hpp>
#include <fuse_core/graph.hpp>
#include <fuse_core/graph_deserializer.hpp>
#include <fuse_core/parameter.hpp>
#include <fuse_core/transaction.hpp>
#include <fuse_core/transaction_deserializer.hpp>
#include <fuse_msgs/msg/serialized_graph.hpp>
#include <fuse_msgs/msg/serialized_transaction.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>


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

void SerializedPublisher::initialize(
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

void SerializedPublisher::onInit()
{
  // Configure the publisher
  frame_id_ = fuse_core::getParam(interfaces_, "frame_id", frame_id_);

  bool latch = false;
  latch = fuse_core::getParam(interfaces_, "latch", latch);

  rclcpp::Duration graph_throttle_period{ 0, 0 };
  fuse_core::getPositiveParam(interfaces_, "graph_throttle_period", graph_throttle_period, false);

  bool graph_throttle_use_wall_time{ false };
  graph_throttle_use_wall_time =
    fuse_core::getParam(
      interfaces_, "graph_throttle_use_wall_time", graph_throttle_use_wall_time);

  graph_publisher_throttled_callback_.setThrottlePeriod(graph_throttle_period);

  if (!graph_throttle_use_wall_time) {
    graph_publisher_throttled_callback_.setClock(interfaces_.get_node_clock_interface()->get_clock());
  }

  // Advertise the topics
  rclcpp::QoS qos(1);  // Queue size of 1
  if (latch) {
    qos.transient_local();
  }

  rclcpp::PublisherOptions pub_options;
  pub_options.callback_group = cb_group_;

  graph_publisher_ =
    rclcpp::create_publisher<fuse_msgs::msg::SerializedGraph>(interfaces_, "graph", qos, pub_options);
  transaction_publisher_ =
    rclcpp::create_publisher<fuse_msgs::msg::SerializedTransaction>(interfaces_, "transaction", qos, pub_options);
}

void SerializedPublisher::notifyCallback(
  fuse_core::Transaction::ConstSharedPtr transaction,
  fuse_core::Graph::ConstSharedPtr graph)
{
  const auto& stamp = transaction->stamp();
  if (graph_publisher_->get_subscription_count() > 0)
  {
    graph_publisher_throttled_callback_(graph, stamp);
  }

  if (transaction_publisher_->get_subscription_count() > 0)
  {
    fuse_msgs::msg::SerializedTransaction msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;
    fuse_core::serializeTransaction(*transaction, msg);
    transaction_publisher_->publish(msg);
  }
}

void SerializedPublisher::graphPublisherCallback(
  fuse_core::Graph::ConstSharedPtr graph, const rclcpp::Time& stamp) const
{
  fuse_msgs::msg::SerializedGraph msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id_;
  fuse_core::serializeGraph(*graph, msg);
  graph_publisher_->publish(msg);
}

}  // namespace fuse_publishers
