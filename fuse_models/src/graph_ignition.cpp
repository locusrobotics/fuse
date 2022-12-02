/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Clearpath Robotics
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

#include <fuse_models/graph_ignition.h>

#include <std_srvs/Empty.h>

#include <pluginlib/class_list_macros.h>

#include <boost/range/algorithm.hpp>
#include <boost/range/empty.hpp>
#include <boost/range/size.hpp>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(fuse_models::GraphIgnition, fuse_core::SensorModel);

namespace fuse_models
{

GraphIgnition::GraphIgnition() : fuse_core::AsyncSensorModel(1), started_(false)
{
}

void GraphIgnition::onInit()
{
  // Read settings from the parameter sever
  params_.loadFromROS(private_node_handle_);

  // Connect to the reset service
  if (!params_.reset_service.empty())
  {
    reset_client_ = node_handle_.serviceClient<std_srvs::Empty>(ros::names::resolve(params_.reset_service));
  }

  // Advertise
  subscriber_ = node_handle_.subscribe(ros::names::resolve(params_.topic), params_.queue_size,
                                       &GraphIgnition::subscriberCallback, this);
  set_graph_service_ = node_handle_.advertiseService(ros::names::resolve(params_.set_graph_service),
                                                     &GraphIgnition::setGraphServiceCallback, this);
}

void GraphIgnition::start()
{
  started_ = true;
}

void GraphIgnition::stop()
{
  started_ = false;
}

void GraphIgnition::subscriberCallback(const fuse_msgs::SerializedGraph::ConstPtr& msg)
{
  try
  {
    process(*msg);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), e.what() << " Ignoring message.");
  }
}

bool GraphIgnition::setGraphServiceCallback(fuse_models::SetGraph::Request& req, fuse_models::SetGraph::Response& res)
{
  try
  {
    process(req.graph);
    res.success = true;
  }
  catch (const std::exception& e)
  {
    res.success = false;
    res.message = e.what();
    RCLCPP_ERROR_STREAM(node_->get_logger(), e.what() << " Ignoring request.");
  }
  return true;
}

void GraphIgnition::process(const fuse_msgs::SerializedGraph& msg)
{
  // Verify we are in the correct state to process set graph requests
  if (!started_)
  {
    throw std::runtime_error("Attempting to set the graph while the sensor is stopped.");
  }

  // Deserialize the graph message
  const auto graph = graph_deserializer_.deserialize(msg);

  // Validate the requested graph before we do anything
  if (boost::empty(graph->getConstraints()))
  {
    throw std::runtime_error("Attempting to set a graph with no constraints.");
  }

  if (boost::empty(graph->getVariables()))
  {
    throw std::runtime_error("Attempting to set a graph with no variables.");
  }

  // Tell the optimizer to reset before providing the initial state
  if (!params_.reset_service.empty())
  {
    // Wait for the reset service
    while (!reset_client_.waitForExistence(rclcpp::Duration::from_seconds(10.0)) && ros::ok())
    {
      RCLCPP_WARN_STREAM(node_->get_logger(),
                         "Waiting for '" << reset_client_.getService() << "' service to become avaiable.");
    }

    auto srv = std_srvs::Empty();
    if (!reset_client_.call(srv))
    {
      // The reset() service failed. Propagate that failure to the caller of this service.
      throw std::runtime_error("Failed to call the '" + reset_client_.getService() + "' service.");
    }
  }

  // Now that the optimizer has been reset, actually send the initial state constraints to the optimizer
  sendGraph(*graph, msg.header.stamp);
}

void GraphIgnition::sendGraph(const fuse_core::Graph& graph, const rclcpp::Time& stamp)
{
  // Create a transaction equivalent to the graph
  auto transaction = fuse_core::Transaction::make_shared();
  transaction->stamp(stamp);

  // Add variables
  for (const auto& variable : graph.getVariables())
  {
    transaction->addVariable(variable.clone());

    // If the variable is a fuse_variables::Stamped variable, set the involved stamp
    const auto stamped_variable = dynamic_cast<const fuse_variables::Stamped*>(&variable);
    if (stamped_variable)
    {
      transaction->addInvolvedStamp(stamped_variable->stamp());
    }
  }

  // If the transaction ended up with no involved stamps, we use a single involved stamped equal to the
  // transaction/graph stamp
  if (boost::empty(transaction->involvedStamps()))
  {
    transaction->addInvolvedStamp(stamp);
  }

  // Add constraints
  for (const auto& constraint : graph.getConstraints())
  {
    transaction->addConstraint(constraint.clone());
  }

  // Send the transaction to the optimizer.
  sendTransaction(transaction);

  RCLCPP_INFO_STREAM(node_->get_logger(),
                     "Received a set_graph request (stamp: " << transaction->stamp() << ", constraints: "
                     << boost::size(transaction->addedConstraints()) << ", variables: "
                     << boost::size(transaction->addedVariables()) << ")");
}

}  // namespace fuse_models
