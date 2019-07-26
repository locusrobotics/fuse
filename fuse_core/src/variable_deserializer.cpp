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
#include <fuse_core/variable_deserializer.h>

#include <sstream>


namespace fuse_core
{

void serializeVariable(const fuse_core::Variable& variable, fuse_msgs::SerializedVariable& msg)
{
  std::stringstream stream;
  {
    cereal::JSONOutputArchive archive(stream);
    variable.serializeVariable(archive);
  }
  msg.plugin_name = variable.type();
  msg.data = stream.str();
}

VariableDeserializer::VariableDeserializer() :
  plugin_loader_("fuse_core", "fuse_core::Variable")
{
}

fuse_core::Variable::SharedPtr VariableDeserializer::deserialize(const fuse_msgs::SerializedVariable::ConstPtr& msg)
{
  return deserialize(*msg);
}

fuse_core::Variable::SharedPtr VariableDeserializer::deserialize(const fuse_msgs::SerializedVariable& msg)
{
  // Create a Variable object using pluginlib. This will throw if the plugin name is not found.
  // The unique ptr returned by pluginlib has a custom deleter. This makes it annoying to return
  // back to the user as the output is not equivalent to fuse_ros::Variable::UniquePtr. A shared_ptr
  // on the other hand is able to capture the custom deleter without modifying the datatype.
  fuse_core::Variable::SharedPtr variable = plugin_loader_.createUniqueInstance(msg.plugin_name);
  // Deserialize the message into the Variable. This will throw if something goes wrong in the deserialization.
  std::stringstream stream(msg.data);
  {
    cereal::JSONInputArchive archive(stream);
    // cereal::XMLInputArchive archive(stream);
    variable->deserializeVariable(archive);
  }
  // Return the populated variable. UniquePtrs are automatically promoted to SharedPtrs.
  return variable;
}

}  // namespace fuse_core
