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

#include <boost/range/algorithm.hpp>
#include <boost/range/empty.hpp>
#include <boost/range/size.hpp>
#include <fuse_models/graph_ignition.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <std_srvs/srv/empty.hpp>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(fuse_models::GraphIgnition, fuse_core::SensorModel);

namespace fuse_models
{

GraphIgnition::GraphIgnition()
: fuse_core::AsyncSensorModel(1),
  started_(false),
  logger_(rclcpp::get_logger("uninitialized"))
{
}

void GraphIgnition::initialize(
  fuse_core::node_interfaces::NodeInterfaces<ALL_FUSE_CORE_NODE_INTERFACES> interfaces,
  const std::string & name,
  fuse_core::TransactionCallback transaction_callback)
{
  interfaces_ = interfaces;
  fuse_core::AsyncSensorModel::initialize(interfaces, name, transaction_callback);
}

void GraphIgnition::onInit()
{
  logger_ = interfaces_.get_node_logging_interface()->get_logger();

  // Read settings from the parameter sever
  params_.loadFromROS(interfaces_, name_);

  // Connect to the reset service
  if (!params_.reset_service.empty()) {
    reset_client_ = rclcpp::create_client<std_srvs::srv::Empty>(
      interfaces_.get_node_base_interface(),
      interfaces_.get_node_graph_interface(),
      interfaces_.get_node_services_interface(),
      params_.reset_service,
      rclcpp::ServicesQoS(),
      cb_group_
    );
  }

  // Advertise
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = cb_group_;
  sub_ = rclcpp::create_subscription<fuse_msgs::msg::SerializedGraph>(
    interfaces_,
    fuse_core::joinTopicName(name_, params_.topic),
    params_.queue_size,
    std::bind(&GraphIgnition::subscriberCallback, this, std::placeholders::_1),
    sub_options
  );

  set_graph_service_ = rclcpp::create_service<fuse_msgs::srv::SetGraph>(
    interfaces_.get_node_base_interface(),
    interfaces_.get_node_services_interface(),
    fuse_core::joinTopicName(
      interfaces_.get_node_base_interface()->get_name(),
      params_.set_graph_service),
    std::bind(
      &GraphIgnition::setGraphServiceCallback, this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3),
    rclcpp::ServicesQoS(),
    cb_group_
  );
}

void GraphIgnition::start()
{
  started_ = true;
}

void GraphIgnition::stop()
{
  started_ = false;
}

void GraphIgnition::subscriberCallback(const fuse_msgs::msg::SerializedGraph & msg)
{
  try {
    process(msg);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(logger_, e.what() << " Ignoring message.");
  }
}

bool GraphIgnition::setGraphServiceCallback(
  rclcpp::Service<fuse_msgs::srv::SetGraph>::SharedPtr service,
  std::shared_ptr<rmw_request_id_t> request_id,
  const fuse_msgs::srv::SetGraph::Request::SharedPtr req)
{
  try {
    process(
      req->graph,
      [service, request_id]() {
        fuse_msgs::srv::SetGraph::Response response;
        response.success = true;
        service->send_response(*request_id, response);
      });
  } catch (const std::exception & e) {
    fuse_msgs::srv::SetGraph::Response response;
    response.success = false;
    response.message = e.what();
    RCLCPP_ERROR_STREAM(logger_, e.what() << " Ignoring request.");
    service->send_response(*request_id, response);
  }
  return true;
}

void GraphIgnition::process(
  const fuse_msgs::msg::SerializedGraph & msg, std::function<void()> post_process)
{
  // Verify we are in the correct state to process set graph requests
  if (!started_) {
    throw std::runtime_error("Attempting to set the graph while the sensor is stopped.");
  }

  // Deserialize the graph message
  // NOTE(methylDragon): We convert the Graph::UniquePtr to a shared pointer so it can be passed as
  //                     a copyable object to the deferred service call's std::function<> arg to
  //                     satisfy the requirement that std::function<> arguments are copyable.
  const auto graph =
    std::shared_ptr<fuse_core::Graph>(std::move(graph_deserializer_.deserialize(msg)));

  // Validate the requested graph before we do anything
  if (boost::empty(graph->getConstraints())) {
    throw std::runtime_error("Attempting to set a graph with no constraints.");
  }

  if (boost::empty(graph->getVariables())) {
    throw std::runtime_error("Attempting to set a graph with no variables.");
  }

  // Tell the optimizer to reset before providing the initial state
  if (!params_.reset_service.empty()) {
    // Wait for the reset service
    while (!reset_client_->wait_for_service(std::chrono::seconds(10)) &&
      interfaces_.get_node_base_interface()->get_context()->is_valid())
    {
      RCLCPP_WARN_STREAM(
        logger_,
        "Waiting for '" << reset_client_->wait_for_service() << "' service to become avaiable.");
    }

    auto srv = std::make_shared<std_srvs::srv::Empty::Request>();
    // Don't block the executor.
    // It needs to be free to handle the response to this service call.
    // Have a callback do the rest of the work when a response comes.
    auto result_future = reset_client_->async_send_request(
      srv,
      [this, post_process, captured_graph = std::move(graph),
      msg](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture result) {
        (void)result;
        // Now that the optimizer has been reset, actually send the initial state constraints to the
        // optimizer
        sendGraph(*captured_graph, msg.header.stamp);
        if (post_process) {
          post_process();
        }
      });
  } else {
    sendGraph(*graph, msg.header.stamp);
    if (post_process) {
      post_process();
    }
  }
}

void GraphIgnition::sendGraph(const fuse_core::Graph & graph, const rclcpp::Time & stamp)
{
  // Create a transaction equivalent to the graph
  auto transaction = fuse_core::Transaction::make_shared();
  transaction->stamp(stamp);

  // Add variables
  for (const auto & variable : graph.getVariables()) {
    transaction->addVariable(variable.clone());

    // If the variable is a fuse_variables::Stamped variable, set the involved stamp
    const auto stamped_variable = dynamic_cast<const fuse_variables::Stamped *>(&variable);
    if (stamped_variable) {
      transaction->addInvolvedStamp(stamped_variable->stamp());
    }
  }

  // If the transaction ended up with no involved stamps, we use a single involved stamped equal to
  // the transaction/graph stamp
  if (boost::empty(transaction->involvedStamps())) {
    transaction->addInvolvedStamp(stamp);
  }

  // Add constraints
  for (const auto & constraint : graph.getConstraints()) {
    transaction->addConstraint(constraint.clone());
  }

  // Send the transaction to the optimizer.
  sendTransaction(transaction);

  RCLCPP_INFO_STREAM(
    logger_,
    "Received a set_graph request (stamp: "
      << transaction->stamp().nanoseconds() << ", constraints: "
      << boost::size(transaction->addedConstraints()) << ", variables: "
      << boost::size(transaction->addedVariables()) << ")");
}

}  // namespace fuse_models
