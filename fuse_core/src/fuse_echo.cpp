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
#include <fuse_msgs/msg/serialized_graph.h>
#include <fuse_msgs/msg/serialized_transaction.h>

#include <fuse_core/graph.hpp>
#include <fuse_core/graph_deserializer.hpp>
#include <fuse_core/transaction.hpp>
#include <fuse_core/transaction_deserializer.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>

namespace fuse_core
{

/**
 * Class that subscribes to the 'graph' and 'transaction' topics and prints the objects to stdout
 */
class FuseEcho : public rclcpp::Node
{
public:
  explicit FuseEcho(rclcpp::NodeOptions options)
  : Node("fuse_echo", options)
  {
    // Subscribe to the constraint topic
    graph_sub_ = this->create_subscription<fuse_msgs::msg::SerializedGraph>(
      "graph",
      rclcpp::QoS(100),
      std::bind(&FuseEcho::graphCallback, this, std::placeholders::_1)
    );
    transaction_sub_ = this->create_subscription<fuse_msgs::msg::SerializedTransaction>(
      "transaction",
      rclcpp::QoS(100),
      std::bind(&FuseEcho::transactionCallback, this, std::placeholders::_1)
    );
  }

private:
  fuse_core::GraphDeserializer graph_deserializer_;
  fuse_core::TransactionDeserializer transaction_deserializer_;
  rclcpp::Subscription<fuse_msgs::msg::SerializedGraph>::SharedPtr graph_sub_;
  rclcpp::Subscription<fuse_msgs::msg::SerializedTransaction>::SharedPtr transaction_sub_;

  void graphCallback(const fuse_msgs::msg::SerializedGraph & msg)
  {
    std::cout << "-------------------------" << std::endl;
    std::cout << "GRAPH:" << std::endl;
    std::cout << "received at: " << this->now().seconds() << std::endl;
    auto graph = graph_deserializer_.deserialize(msg);
    graph->print();
  }

  void transactionCallback(const fuse_msgs::msg::SerializedTransaction & msg)
  {
    std::cout << "-------------------------" << std::endl;
    std::cout << "TRANSACTION:" << std::endl;
    std::cout << "received at: " << this->now().seconds() << std::endl;
    auto transaction = transaction_deserializer_.deserialize(msg);
    transaction->print();
  }
};

}  // namespace fuse_core


#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(fuse_core::FuseEcho)
