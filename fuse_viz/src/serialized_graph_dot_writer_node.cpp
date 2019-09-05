/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Clearpath Robotics
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

#include <fuse_viz/serialized_graph_dot_writer.h>
#include <ros/ros.h>

#include <fstream>
#include <string>

namespace fuse_viz
{

/**
 * @brief A fuse_msgs::SerializedGraph to DOT format writer node.
 *
 * This subscribes to a fuse_msgs::SerializedGraph topic and saves each message into a file using Graphviz DOT format.
 */
class SerializedGraphDOTWriterNode
{
public:
  /**
   * @brief Constructor.
   * This reads ROS parameters and subscribes to the fuse_msgs::SerializedGraph topic.
   */
  SerializedGraphDOTWriterNode();

  /**
   * @brief A callback to process fuse_msgs::SerializedGraph messages.
   *
   * @param[in] msg A fuse_msgs::SerializedGraph message.
   */
  void graphCallback(const fuse_msgs::SerializedGraphConstPtr& msg);

private:
  ros::NodeHandle nh_;          //< ROS node handle
  ros::Subscriber subscriber_;  //< ROS subscriber to the fuse_msgs::SerializedGraph topic

  std::string filename_prefix_{ "" };  //< Filename prefix to save the graph in Graphviz DOT format
  int queue_size_{ 100 };              //< Queue size for the ROS subscriber
};

SerializedGraphDOTWriterNode::SerializedGraphDOTWriterNode()
{
  ros::NodeHandle private_node_handle("~");

  private_node_handle.param("filename_prefix", filename_prefix_, filename_prefix_);
  private_node_handle.param("queue_size", queue_size_, queue_size_);

  subscriber_ = nh_.subscribe("graph", queue_size_, &SerializedGraphDOTWriterNode::graphCallback, this);
}

void SerializedGraphDOTWriterNode::graphCallback(const fuse_msgs::SerializedGraphConstPtr& msg)
{
  std::ofstream ofs(filename_prefix_ + std::to_string(msg->header.seq) + ".dot");
  SerializedGraphDOTWriter::write(ofs, msg);
}

}  // namespace fuse_viz

int main(int argc, char** argv)
{
  ros::init(argc, argv, "serialized_graph_dot_writer_node");
  fuse_viz::SerializedGraphDOTWriterNode node;
  ros::spin();

  return EXIT_SUCCESS;
}
