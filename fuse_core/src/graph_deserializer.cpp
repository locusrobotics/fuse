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
#include <fuse_core/graph_deserializer.h>

#include <sstream>


namespace fuse_core
{

void serializeGraph(const fuse_core::Graph& graph, fuse_msgs::SerializedGraph& msg)
{
  std::stringstream stream;
  {
    cereal::JSONOutputArchive archive(stream);
    graph.serializeGraph(archive);
  }
  msg.plugin_name = graph.type();
  msg.data = stream.str();
}

GraphDeserializer::GraphDeserializer() :
  constraint_loader_("fuse_core", "fuse_core::Constraint"),
  graph_loader_("fuse_core", "fuse_core::Graph"),
  variable_loader_("fuse_core", "fuse_core::Variable")
{
  // Load all known plugin libraries
  // I believe the library containing a given Variable or Constraint must be loaded in order to deserialize
  // an object of that type. But I haven't actually tested that theory.
  for (const auto& class_name : constraint_loader_.getDeclaredClasses())
  {
    constraint_loader_.loadLibraryForClass(class_name);
  }
  for (const auto& class_name : variable_loader_.getDeclaredClasses())
  {
    variable_loader_.loadLibraryForClass(class_name);
  }
}

fuse_core::Graph::SharedPtr GraphDeserializer::deserialize(const fuse_msgs::SerializedGraph::ConstPtr& msg)
{
  return deserialize(*msg);
}

fuse_core::Graph::SharedPtr GraphDeserializer::deserialize(const fuse_msgs::SerializedGraph& msg)
{
  // Create a Variable object using pluginlib. This will throw if the plugin name is not found.
  // The unique ptr returned by pluginlib has a custom deleter. This makes it annoying to return
  // back to the user as the output is not equivalent to fuse_ros::Variable::UniquePtr. A shared_ptr
  // on the other hand is able to capture the custom deleter without modifying the datatype.
  fuse_core::Graph::SharedPtr graph = graph_loader_.createUniqueInstance(msg.plugin_name);
  // Deserialize the message into the Variable. This will throw if something goes wrong in the deserialization.
  std::stringstream stream(msg.data);
  {
    cereal::JSONInputArchive archive(stream);
    graph->deserializeGraph(archive);
  }
  // Return the populated variable. UniquePtrs are automatically promoted to SharedPtrs.
  return graph;
}

}  // namespace fuse_core
