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
#ifndef FUSE_PUBLISHERS_SERIALIZED_PUBLISHER_H
#define FUSE_PUBLISHERS_SERIALIZED_PUBLISHER_H

#include <fuse_core/async_publisher.hpp>
#include <fuse_core/graph.hpp>
#include <fuse_core/fuse_macros.hpp>
#include <fuse_core/throttled_callback.hpp>
#include <fuse_core/transaction.hpp>
#include <rclcpp/rclcpp.hpp>

#include <fuse_msgs/msg/serialized_graph.hpp>
#include <fuse_msgs/msg/serialized_transaction.hpp>


#include <string>


namespace fuse_publishers
{

/**
 * @brief Publisher plugin that publishes the transaction and graph as serialized messages
 */
class SerializedPublisher : public fuse_core::AsyncPublisher
{
public:
  FUSE_SMART_PTR_DEFINITIONS(SerializedPublisher)

  /**
   * @brief Constructor
   */
  SerializedPublisher();

  /**
   * @brief Destructor
   */
  virtual ~SerializedPublisher() = default;

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
   * @brief Notify the publisher about variables that have been added or removed
   *
   * @param[in] transaction A Transaction object, describing the set of variables that have been added and/or removed
   * @param[in] graph       A read-only pointer to the graph object, allowing queries to be performed whenever needed
   */
  void notifyCallback(
    fuse_core::Transaction::ConstSharedPtr transaction,
    fuse_core::Graph::ConstSharedPtr graph) override;

protected:
  fuse_core::node_interfaces::NodeInterfaces<
    fuse_core::node_interfaces::Base,
    fuse_core::node_interfaces::Clock,
    fuse_core::node_interfaces::Logging,
    fuse_core::node_interfaces::Parameters,
    fuse_core::node_interfaces::Topics,
    fuse_core::node_interfaces::Waitables
  > interfaces_;  //!< Shadows AsyncPublisher interfaces_

  /**
   * @brief Publish the serialized graph
   *
   * @param[in] graph A read-only pointer to the graph object, allowing queries to be performed whenever needed
   * @param[in] stamp A rclcpp::Time stamp used for the serialized graph message published
   */
  void graphPublisherCallback(fuse_core::Graph::ConstSharedPtr graph, const rclcpp::Time& stamp) const;

  std::string frame_id_;  //!< The name of the frame for the serialized graph and transaction messages published
  rclcpp::Publisher<fuse_msgs::msg::SerializedGraph>::SharedPtr graph_publisher_;
  rclcpp::Publisher<fuse_msgs::msg::SerializedTransaction>::SharedPtr transaction_publisher_;

  using GraphPublisherCallback = std::function<void(fuse_core::Graph::ConstSharedPtr, const rclcpp::Time&)>;
  using GraphPublisherThrottledCallback = fuse_core::ThrottledCallback<GraphPublisherCallback>;
  GraphPublisherThrottledCallback graph_publisher_throttled_callback_;  //!< The graph publisher throttled callback
};

}  // namespace fuse_publishers

#endif  // FUSE_PUBLISHERS_SERIALIZED_PUBLISHER_H
