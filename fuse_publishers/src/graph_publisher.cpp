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
#include <fuse_publishers/graph_publisher.h>

#include <fuse_core/async_publisher.h>
#include <fuse_core/graph.h>
#include <fuse_core/transaction.h>
#include <fuse_core/uuid.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <exception>
#include <memory>
#include <string>
#include <utility>
#include <vector>


// Register this publisher with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(fuse_publishers::GraphPublisher, fuse_core::Publisher);

namespace fuse_publishers
{

GraphPublisher::GraphPublisher() :
  fuse_core::AsyncPublisher(1)
{
}

void GraphPublisher::onInit()
{
  // Read configuration from the parameter server
  std::string device_str;
  if (private_node_handle_.getParam("device_id", device_str))
  {
    device_id_ = fuse_core::uuid::from_string(device_str);
  }
  else if (private_node_handle_.getParam("device_name", device_str))
  {
    device_id_ = fuse_core::uuid::generate(device_str);
  }
  private_node_handle_.param("publish_optimized_graph", publish_optimized_graph_, publish_optimized_graph_);
  private_node_handle_.param("queue_size", queue_size_, queue_size_);
  private_node_handle_.param("frame_id", frame_id_, frame_id_);

  // The graph doesn't have a frame ID
  graph_msg_.header.frame_id = frame_id_;

  // Advertise the topics
  graph_publisher_ = private_node_handle_.advertise<fuse_msgs::Graph>("graph", queue_size_);
}

void GraphPublisher::onStart()
{
  graph_msg_.variables.clear();
  graph_msg_.constraints.clear();

  index_by_variable_uuid_.clear();
  index_by_constraint_uuid_.clear();
}

void GraphPublisher::notifyCallback(
  fuse_core::Transaction::ConstSharedPtr transaction,
  fuse_core::Graph::ConstSharedPtr graph)
{
  updateGraphMessage(transaction);

  if (graph_publisher_.getNumSubscribers() > 0)
  {
    // Unfortunately, the graph doesn't have a stamp and the transaction stamp is zero for merged transactions.
    // Alternatively, we could use the most recent involved stamp in the transaction, but that wouldn't work if the
    // transaction is empty
    graph_msg_.header.stamp = ros::Time::now();

    if (publish_optimized_graph_ && transaction->empty())
    {
      for (const auto& variable : graph->getVariables())
      {
        const auto variable_iter = index_by_variable_uuid_.find(variable.uuid());
        if (variable_iter == index_by_variable_uuid_.end())
        {
          ROS_WARN_STREAM("Could not find variable " << fuse_core::uuid::to_string(variable.uuid())
                                                     << " in the graph message.");
          continue;
        }

        const auto& variable_index = variable_iter->second;
        if (variable_index >= graph_msg_.variables.size())
        {
          ROS_WARN_STREAM("Variable " << fuse_core::uuid::to_string(variable.uuid()) << " index (" << variable_index
                                      << ") is out of range. Graph message only has " << graph_msg_.variables.size()
                                      << " variables.");
          continue;
        }

        auto& variable_data = graph_msg_.variables[variable_index].data;
        if (variable_data.size() != variable.size())
        {
          ROS_WARN_STREAM("Variable " << fuse_core::uuid::to_string(variable.uuid()) << " data size ("
                                      << variable.size() << ") is different that the graph message variable size ("
                                      << variable_data.size() << ").");
          continue;
        }

        std::copy_n(variable.data(), variable.size(), variable_data.begin());
      }
    }

    graph_publisher_.publish(graph_msg_);
  }
}

void GraphPublisher::updateGraphMessage(fuse_core::Transaction::ConstSharedPtr transaction)
{
  for (const auto& variable : transaction->addedVariables())
  {
    addVariable(variable);
  }

  for (const auto& constraint : transaction->addedConstraints())
  {
    addConstraint(constraint);
  }

  for (const auto& constraint_uuid : transaction->removedConstraints())
  {
    removeConstraint(constraint_uuid);
  }

  for (const auto& variable_uuid : transaction->removedVariables())
  {
    removeVariable(variable_uuid);
  }
}

fuse_msgs::Variable toMsg(const fuse_core::Variable& variable)
{
  fuse_msgs::Variable variable_msg;

  variable_msg.uuid = fuse_core::uuid::to_string(variable.uuid());
  variable_msg.type = variable.type();
  std::copy_n(variable.data(), variable.size(), std::back_inserter(variable_msg.data));

  return variable_msg;
}

fuse_msgs::Constraint toMsg(const fuse_core::Constraint& constraint)
{
  fuse_msgs::Constraint constraint_msg;

  constraint_msg.uuid = fuse_core::uuid::to_string(constraint.uuid());
  constraint_msg.type = constraint.type();

  const auto& constraint_variables = constraint.variables();
  constraint_msg.variables.reserve(constraint_variables.size());
  std::transform(constraint_variables.begin(), constraint_variables.end(), std::back_inserter(constraint_msg.variables),
                 [](const auto& variable_uuid) -> std::string { return fuse_core::uuid::to_string(variable_uuid); });

  // FIXME populate the properties vector somehow from the cost function or maybe extend the API to support this

  return constraint_msg;
}

bool GraphPublisher::variableExists(const fuse_core::UUID& variable_uuid) const
{
  return index_by_variable_uuid_.find(variable_uuid) != index_by_variable_uuid_.end();
}

bool GraphPublisher::constraintExists(const fuse_core::UUID& constraint_uuid) const
{
  return index_by_constraint_uuid_.find(constraint_uuid) != index_by_constraint_uuid_.end();
}

bool GraphPublisher::addVariable(const fuse_core::Variable& variable)
{
  if (variableExists(variable.uuid()))
  {
    return false;
  }

  const auto variable_index = graph_msg_.variables.size();

  graph_msg_.variables.push_back(toMsg(variable));

  index_by_variable_uuid_[variable.uuid()] = variable_index;

  return true;
}

bool GraphPublisher::addConstraint(const fuse_core::Constraint& constraint)
{
  if (constraintExists(constraint.uuid()))
  {
    return false;
  }

  const auto constraint_index = graph_msg_.constraints.size();

  graph_msg_.constraints.push_back(toMsg(constraint));

  index_by_constraint_uuid_[constraint.uuid()] = constraint_index;

  return true;
}

bool GraphPublisher::removeVariable(const fuse_core::UUID& variable_uuid)
{
  // Confirm the variable exists
  const auto variable_iter = index_by_variable_uuid_.find(variable_uuid);
  if (variable_iter == index_by_variable_uuid_.end())
  {
    return false;
  }

  // Swap with the last variable, since we don't care about the order and this is more efficient
  const auto& variable_index = variable_iter->second;
  if (variable_index < graph_msg_.variables.size() - 1)
  {
    const auto& variable_back = graph_msg_.variables.back();
    graph_msg_.variables[variable_index] = variable_back;
    index_by_variable_uuid_[fuse_core::uuid::from_string(variable_back.uuid)] = variable_index;
  }

  graph_msg_.variables.pop_back();
  index_by_variable_uuid_.erase(variable_iter);

  return true;
}

bool GraphPublisher::removeConstraint(const fuse_core::UUID& constraint_uuid)
{
  // Confirm the constraint exists
  const auto constraint_iter = index_by_constraint_uuid_.find(constraint_uuid);
  if (constraint_iter == index_by_constraint_uuid_.end())
  {
    return false;
  }

  // Swap with the last constraint, since we don't care about the order and this is more efficient
  const auto& constraint_index = constraint_iter->second;
  if (constraint_index < graph_msg_.constraints.size() - 1)
  {
    const auto& constraint_back = graph_msg_.constraints.back();
    graph_msg_.constraints[constraint_index] = constraint_back;
    index_by_constraint_uuid_[fuse_core::uuid::from_string(constraint_back.uuid)] = constraint_index;
  }

  graph_msg_.constraints.pop_back();
  index_by_constraint_uuid_.erase(constraint_iter);

  return true;
}

}  // namespace fuse_publishers
