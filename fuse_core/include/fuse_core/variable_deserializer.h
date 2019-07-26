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
#ifndef FUSE_CORE_VARIABLE_DESERIALIZER_H
#define FUSE_CORE_VARIABLE_DESERIALIZER_H

#include <fuse_msgs/SerializedVariable.h>
#include <fuse_core/variable.h>

#include <pluginlib/class_loader.h>


namespace fuse_core
{

/**
 * @brief Serialize a variable into a message
 *
 * @param variable
 * @param msg
 */
void serializeVariable(const fuse_core::Variable& variable, fuse_msgs::SerializedVariable& msg);

/**
 * @brief Object that loads all known Variable plugins, and deserializes a SerializedVariable message
 *        using the correct plugin.
 */
class VariableDeserializer
{
public:
  /**
   * @brief Constructor
   */
  VariableDeserializer();

  /**
   * @brief Destructor
   */
  ~VariableDeserializer() = default;

  /**
   * @brief Deserialize a SerializedVariable message into a fuse Variable object.
   *
   * A derived fuse Variable object is allocated, and a shared pointer to the variable is returned. This uses the
   * registered Variable classes to deserialize the message as the correct derived type. If no plugin is available
   * or an error occurs during deserialization, an exception is thrown.
   *
   * @param[IN]  msg  The SerializedVariable message to be deserialized into a fuse Variable derived object.
   * @return          A shared pointer to the derived fuse Variable object
   */
  fuse_core::Variable::SharedPtr deserialize(const fuse_msgs::SerializedVariable::ConstPtr& msg);

  /**
   * @brief Deserialize a SerializedVariable message into a fuse Variable object.
   *
   * A derived fuse Variable object is allocated, and a shared pointer to the variable is returned. This uses the
   * registered Variable classes to deserialize the message as the correct derived type. If no plugin is available
   * or an error occurs during deserialization, an exception is thrown.
   *
   * @param[IN]  msg  The SerializedVariable message to be deserialized into a fuse Variable derived object.
   * @return          A shared pointer to the derived fuse Variable object
   */
  fuse_core::Variable::SharedPtr deserialize(const fuse_msgs::SerializedVariable& msg);

private:
  pluginlib::ClassLoader<fuse_core::Variable> plugin_loader_;  //!< Pluginlib class loader
};

}  // namespace fuse_core

#endif  // FUSE_CORE_VARIABLE_DESERIALIZER_H
