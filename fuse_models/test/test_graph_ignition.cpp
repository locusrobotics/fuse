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

#include <fuse_core/constraint.h>
#include <fuse_core/graph_deserializer.h>
#include <fuse_core/transaction.h>
#include <fuse_core/variable.h>
#include <fuse_graphs/hash_graph.h>
#include <fuse_models/SetGraph.h>
#include <fuse_models/graph_ignition.h>
#include <ros/ros.h>

#include <test/example_constraint.h>
#include <test/example_variable.h>
#include <test/example_variable_stamped.h>

#include <gtest/gtest.h>

#include <boost/range/algorithm.hpp>
#include <boost/range/size.hpp>

#include <chrono>
#include <future>
#include <utility>
#include <string>

/**
 * @brief Promise used to communicate between the tests and the callback
 */
std::promise<fuse_core::Transaction::SharedPtr> callback_promise;

/**
 * @brief Transaction callback that forwards the transaction into the promise result
 */
void transactionCallback(fuse_core::Transaction::SharedPtr transaction)
{
  callback_promise.set_value(std::move(transaction));
}

/**
 * @brief Static variable to hold the last unit test error description
 */
static std::string failure_description;  // NOLINT(runtime/string)

/**
 * @brief Compare all the properties of two Variable objects
 * @return True if all the properties match, false otherwise
 */
bool compareVariables(const fuse_core::Variable& expected, const fuse_core::Variable& actual)
{
  failure_description = "";
  bool variables_equal = true;
  if (expected.type() != actual.type())
  {
    variables_equal = false;
    failure_description += "The variables have different types.\n  expected type is '" + expected.type() +
                           "'\n    actual type is '" + actual.type() + "'\n";
  }
  if (expected.size() != actual.size())
  {
    variables_equal = false;
    failure_description += "The variables have different sizes.\n  expected size is '" +
                           std::to_string(expected.size()) + "'\n    actual size is '" + std::to_string(actual.size()) +
                           "'\n";
  }
  if (expected.uuid() != actual.uuid())
  {
    variables_equal = false;
    failure_description += "The variables have different UUIDs.\n  expected UUID is '" +
                           fuse_core::uuid::to_string(expected.uuid()) + "'\n    actual UUID is '" +
                           fuse_core::uuid::to_string(actual.uuid()) + "'\n";
  }
  for (size_t i = 0; i < expected.size(); ++i)
  {
    if (expected.data()[i] != actual.data()[i])
    {
      variables_equal = false;
      failure_description += "The variables have different values.\n  expected data(" + std::to_string(i) + ") is '" +
                             std::to_string(expected.data()[i]) + "'\n    actual data(" + std::to_string(i) + ") is '" +
                             std::to_string(actual.data()[i]) + "'\n";
    }
  }
  return variables_equal;
}

/**
 * @brief Compare all the properties of two Constraint objects
 * @return True if all the properties match, false otherwise
 */
bool compareConstraints(const fuse_core::Constraint& expected, const fuse_core::Constraint& actual)
{
  failure_description = "";
  bool constraints_equal = true;
  if (expected.type() != actual.type())
  {
    constraints_equal = false;
    failure_description += "The constraints have different types.\n  expected type is '" + expected.type() +
                           "'\n    actual type is '" + actual.type() + "'\n";
  }
  if (expected.uuid() != actual.uuid())
  {
    constraints_equal = false;
    failure_description += "The constraints have different UUIDs.\n  expected UUID is '" +
                           fuse_core::uuid::to_string(expected.uuid()) + "'\n    actual UUID is '" +
                           fuse_core::uuid::to_string(actual.uuid()) + "'\n";
  }
  if (expected.variables().size() != actual.variables().size())
  {
    constraints_equal = false;
    failure_description += "The constraints involve a different number of variables.\n  expected variable count is '" +
                           std::to_string(expected.variables().size()) + "'\n    actual variable count is '" +
                           std::to_string(actual.variables().size()) + "'\n";
  }
  for (size_t i = 0; i < expected.variables().size(); ++i)
  {
    if (expected.variables().at(i) != actual.variables().at(i))
    {
      constraints_equal = false;
      std::string i_str = std::to_string(i);
      failure_description += "The constraints involve different variable UUIDs.\n  expected variables(" + i_str +
                             ") is '" + fuse_core::uuid::to_string(expected.variables()[i]) +
                             "'\n    actual variables(" + i_str + ") is '" +
                             fuse_core::uuid::to_string(actual.variables()[i]) + "'\n";
    }
  }
  return constraints_equal;
}

namespace fuse_core
{

bool operator==(const fuse_core::Variable& rhs, const fuse_core::Variable& lhs)
{
  return compareVariables(rhs, lhs);
}

bool operator!=(const fuse_core::Variable& rhs, const fuse_core::Variable& lhs)
{
  return !(rhs == lhs);
}

bool operator==(const fuse_core::Constraint& rhs, const fuse_core::Constraint& lhs)
{
  return compareConstraints(rhs, lhs);
}

bool operator!=(const fuse_core::Constraint& rhs, const fuse_core::Constraint& lhs)
{
  return !(rhs == lhs);
}

}  // namespace fuse_core

TEST(Unicycle2DIgnition, SetGraphService)
{
  // Set some configuration
  ros::param::set("/graph_ignition_test/ignition_sensor/set_graph_service", "/set_graph");
  ros::param::set("/graph_ignition_test/ignition_sensor/reset_service", "");

  // Initialize the callback promise. Promises are single-use.
  callback_promise = std::promise<fuse_core::Transaction::SharedPtr>();
  auto callback_future = callback_promise.get_future();

  // Create an ignition sensor and register the callback
  fuse_models::GraphIgnition ignition_sensor;
  ignition_sensor.initialize("ignition_sensor", &transactionCallback);
  ignition_sensor.start();

  // Create graph
  fuse_graphs::HashGraph graph;

  auto variable1 = ExampleVariable::make_shared();
  variable1->data()[0] = 1.0;
  graph.addVariable(variable1);

  auto variable2 = ExampleVariable::make_shared();
  variable2->data()[0] = 2.5;
  graph.addVariable(variable2);

  auto variable3 = ExampleVariable::make_shared();
  variable3->data()[0] = -1.2;
  graph.addVariable(variable3);

  auto constraint1 = ExampleConstraint::make_shared(
      "test",
      std::initializer_list<fuse_core::UUID>{ variable1->uuid(), variable2->uuid() });  // NOLINT(whitespace/braces)
  constraint1->data = 1.5;
  graph.addConstraint(constraint1);

  auto constraint2 = ExampleConstraint::make_shared(
      "test",
      std::initializer_list<fuse_core::UUID>{ variable2->uuid(), variable3->uuid() });  // NOLINT(whitespace/braces)
  constraint2->data = -3.7;
  graph.addConstraint(constraint2);

  // Call the SetGraph service
  fuse_models::SetGraph srv;
  fuse_core::serializeGraph(graph, srv.request.graph);
  const bool success = ros::service::call("/set_graph", srv);
  ASSERT_TRUE(success);
  EXPECT_TRUE(srv.response.success);

  // The ignition sensor should publish a transaction in response to the service call. Wait for the callback to fire.
  auto status = callback_future.wait_for(std::chrono::seconds(5));
  ASSERT_TRUE(status == std::future_status::ready);

  // Check the transaction is equivalent to the graph, i.e. it has the same constraints and transactions
  const auto transaction = callback_future.get();

  ASSERT_EQ(boost::size(graph.getConstraints()), boost::size(transaction->addedConstraints()));
  ASSERT_EQ(boost::size(graph.getVariables()), boost::size(transaction->addedVariables()));

  // We cannot compare the constraints or variables of the graph with the added constraints or variables of the
  // transaction with:
  //
  //    graph.getConstraints() == transaction->addedConstraints()
  //
  // and
  //
  //    graph.getVariables() == transaction->addedVariables()
  //
  // because the graph could stored the constraints and variables in unordered containers. Indeed, the
  // fuse_graphs::HashGraph uses unordered containers for both the constraints and variables.
  //
  // So even if the added constraints and variables are stored in std::vector containers in the transaction, we cannot
  // compare them with the straightforward approach mentioned above. Instead, we need to check that all added
  // constraints and varaibles are in the graph, and check they are the same.
  for (const auto& added_constraint : transaction->addedConstraints())
  {
    try
    {
      const auto& constraint = graph.getConstraint(added_constraint.uuid());

      EXPECT_EQ(constraint, added_constraint) << failure_description;
    }
    catch (const std::out_of_range& ex)
    {
      ADD_FAILURE() << ex.what();
    }
  }

  for (const auto& added_variable : transaction->addedVariables())
  {
    try
    {
      const auto& variable = graph.getVariable(added_variable.uuid());

      EXPECT_EQ(variable, added_variable) << failure_description;
    }
    catch (const std::out_of_range& ex)
    {
      ADD_FAILURE() << ex.what();
    }
  }

  // Since the variables in the graph do not have a stamp, the transaction should have a single involved stamp, equal to
  // the transaction stamp, that should also be equal to the requested graph message stamp
  ASSERT_EQ(1u, boost::size(transaction->involvedStamps()));
  EXPECT_EQ(transaction->stamp(), transaction->involvedStamps().front());
  EXPECT_EQ(srv.request.graph.header.stamp, transaction->stamp());
}

TEST(Unicycle2DIgnition, SetGraphServiceWithStampedVariables)
{
  // Set some configuration
  ros::param::set("/graph_ignition_test/ignition_sensor/set_graph_service", "/set_graph");
  ros::param::set("/graph_ignition_test/ignition_sensor/reset_service", "");

  // Initialize the callback promise. Promises are single-use.
  callback_promise = std::promise<fuse_core::Transaction::SharedPtr>();
  auto callback_future = callback_promise.get_future();

  // Create an ignition sensor and register the callback
  fuse_models::GraphIgnition ignition_sensor;
  ignition_sensor.initialize("ignition_sensor", &transactionCallback);
  ignition_sensor.start();

  // Create graph
  fuse_graphs::HashGraph graph;

  auto variable1 = ExampleVariableStamped::make_shared(rclcpp::Time(101.0));
  variable1->data()[0] = 1.0;
  graph.addVariable(variable1);

  auto variable2 = ExampleVariableStamped::make_shared(rclcpp::Time(102.0));
  variable2->data()[0] = 2.5;
  graph.addVariable(variable2);

  auto variable3 = ExampleVariableStamped::make_shared(rclcpp::Time(103.0));
  variable3->data()[0] = -1.2;
  graph.addVariable(variable3);

  auto constraint1 = ExampleConstraint::make_shared(
      "test",
      std::initializer_list<fuse_core::UUID>{ variable1->uuid(), variable2->uuid() });  // NOLINT(whitespace/braces)
  constraint1->data = 1.5;
  graph.addConstraint(constraint1);

  auto constraint2 = ExampleConstraint::make_shared(
      "test",
      std::initializer_list<fuse_core::UUID>{ variable2->uuid(), variable3->uuid() });  // NOLINT(whitespace/braces)
  constraint2->data = -3.7;
  graph.addConstraint(constraint2);

  // Call the SetGraph service
  fuse_models::SetGraph srv;
  fuse_core::serializeGraph(graph, srv.request.graph);
  const bool success = ros::service::call("/set_graph", srv);
  ASSERT_TRUE(success);
  EXPECT_TRUE(srv.response.success);

  // The ignition sensor should publish a transaction in response to the service call. Wait for the callback to fire.
  auto status = callback_future.wait_for(std::chrono::seconds(5));
  ASSERT_TRUE(status == std::future_status::ready);

  // Check the transaction is equivalent to the graph, i.e. it has the same constraints and transactions
  const auto transaction = callback_future.get();

  ASSERT_EQ(boost::size(graph.getConstraints()), boost::size(transaction->addedConstraints()));
  ASSERT_EQ(boost::size(graph.getVariables()), boost::size(transaction->addedVariables()));

  // We cannot compare the constraints or variables of the graph with the added constraints or variables of the
  // transaction with:
  //
  //    graph.getConstraints() == transaction->addedConstraints()
  //
  // and
  //
  //    graph.getVariables() == transaction->addedVariables()
  //
  // because the graph could stored the constraints and variables in unordered containers. Indeed, the
  // fuse_graphs::HashGraph uses unordered containers for both the constraints and variables.
  //
  // So even if the added constraints and variables are stored in std::vector containers in the transaction, we cannot
  // compare them with the straightforward approach mentioned above. Instead, we need to check that all added
  // constraints and varaibles are in the graph, and check they are the same.
  for (const auto& added_constraint : transaction->addedConstraints())
  {
    try
    {
      const auto& constraint = graph.getConstraint(added_constraint.uuid());

      EXPECT_EQ(constraint, added_constraint) << failure_description;
    }
    catch (const std::out_of_range& ex)
    {
      ADD_FAILURE() << ex.what();
    }
  }

  for (const auto& added_variable : transaction->addedVariables())
  {
    try
    {
      const auto& variable = graph.getVariable(added_variable.uuid());

      EXPECT_EQ(variable, added_variable) << failure_description;
    }
    catch (const std::out_of_range& ex)
    {
      ADD_FAILURE() << ex.what();
    }
  }

  // Since the variables in the graph have a stamp, the transaction should have one involved stamp per variable, and the
  // transaction stamp should be equal to the requested graph message stamp
  ASSERT_EQ(boost::size(graph.getVariables()), boost::size(transaction->involvedStamps()));
  EXPECT_EQ(srv.request.graph.header.stamp, transaction->stamp());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "graph_ignition_test");
  auto spinner = ros::AsyncSpinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
