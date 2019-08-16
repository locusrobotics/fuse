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
#include <fuse_msgs/SerializedGraph.h>
#include <fuse_msgs/SerializedTransaction.h>
#include <fuse_core/graph.h>
#include <fuse_core/graph_deserializer.h>
#include <fuse_core/transaction.h>
#include <fuse_core/transaction_deserializer.h>
#include <ros/ros.h>


/**
 * Class that subscribes to the 'graph' and 'transaction' topics and prints the objects to stdout
 */
class FuseEcho
{
public:
  explicit FuseEcho(const ros::NodeHandle& node_handle = ros::NodeHandle()) :
    node_handle_(node_handle)
  {
    // Subscribe to the constraint topic
    graph_subscriber_ = node_handle_.subscribe("graph", 100, &FuseEcho::graphCallback, this);
    transaction_subscriber_ = node_handle_.subscribe("transaction", 100, &FuseEcho::transactionCallback, this);
  }

private:
  fuse_core::GraphDeserializer graph_deserializer_;
  fuse_core::TransactionDeserializer transaction_deserializer_;
  ros::NodeHandle node_handle_;
  ros::Subscriber graph_subscriber_;
  ros::Subscriber transaction_subscriber_;

  void graphCallback(const fuse_msgs::SerializedGraph::ConstPtr& msg)
  {
    std::cout << "-------------------------" << std::endl;
    std::cout << "GRAPH:" << std::endl;
    std::cout << "received at: " << ros::Time::now() << std::endl;
    auto graph = graph_deserializer_.deserialize(msg);
    graph->print();
  }

  void transactionCallback(const fuse_msgs::SerializedTransaction::ConstPtr& msg)
  {
    std::cout << "-------------------------" << std::endl;
    std::cout << "TRANSACTION:" << std::endl;
    std::cout << "received at: " << ros::Time::now() << std::endl;
    auto transaction = transaction_deserializer_.deserialize(msg);
    transaction.print();
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fuse_echo", ros::init_options::AnonymousName);

  // Object that subscribes to the 'graph' and 'transaction' topics and prints the objects to stdout
  FuseEcho echoer;

  // Wait for an exit signal
  ros::spin();

  return 0;
}
