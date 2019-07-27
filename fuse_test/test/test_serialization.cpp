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
#include <fuse_constraints/dummy_constraint.h>
#include <fuse_core/constraint.h>
#include <fuse_core/constraint_deserializer.h>
#include <fuse_core/eigen.h>
#include <fuse_core/graph.h>
#include <fuse_core/graph_deserializer.h>
#include <fuse_core/serialization.h>
#include <fuse_core/transaction.h>
#include <fuse_core/transaction_deserializer.h>
#include <fuse_core/variable.h>
#include <fuse_core/variable_deserializer.h>
#include <fuse_graphs/hash_graph.h>
#include <fuse_msgs/SerializedConstraint.h>
#include <fuse_msgs/SerializedGraph.h>
#include <fuse_msgs/SerializedTransaction.h>
#include <fuse_msgs/SerializedVariable.h>
#include <fuse_variables/dummy_variable.h>

#include <pluginlib/class_loader.h>

#include <cereal/archives/json.hpp>
#include <gtest/gtest.h>

// NOLINTNEXTLINE(build/namespaces): using operator""s issues a warning.
using namespace std::string_literals;


TEST(Serialization, VariableInstance)
{
  // Create a variable instance
  auto stamp = ros::Time(12345678, 910111213);
  auto quest = "Rescue Queen Guenever"s;
  auto variable = fuse_variables::DummyVariable(stamp, quest);
  variable.a() = 5.1;
  variable.b() = -0.03;

  // Serialize the variable
  // There are several different serialization formats. Using JSON here so the results are somewhat readable.
  std::stringstream stream;
  {
    cereal::JSONOutputArchive archive(stream);
    archive(CEREAL_NVP(variable));
  }
  std::cout << stream.str() << std::endl;
}

TEST(Serialization, DerivedPointer)
{
  // Create a variable instance
  auto stamp = ros::Time(12345678, 910111213);
  auto quest = "Seek the Holy Grail"s;
  auto variable = fuse_variables::DummyVariable::make_shared(stamp, quest);
  variable->a() = 5.1;
  variable->b() = -0.03;

  // Serialize the variable
  std::stringstream stream;
  {
    cereal::JSONOutputArchive archive(stream);
    archive(CEREAL_NVP(variable));
  }
  std::cout << stream.str() << std::endl;
}

TEST(Serialization, BaseClassPointer)
{
  // Create a variable instance
  auto stamp = ros::Time(12345678, 910111213);
  auto quest = "Draw the sword from the stone"s;
  fuse_core::Variable::SharedPtr variable = fuse_variables::DummyVariable::make_shared(stamp, quest);
  variable->data()[0] = 5.1;
  variable->data()[1] = -0.03;

  // Serialize the variable
  std::stringstream stream;
  {
    cereal::JSONOutputArchive archive(stream);
    archive(CEREAL_NVP(variable));
  }
  std::cout << stream.str() << std::endl;
}

TEST(Serialization, Plugin)
{
  // Create a variable instance
  pluginlib::ClassLoader<fuse_core::Variable> plugin_loader("fuse_core", "fuse_core::Variable");
  fuse_core::Variable::SharedPtr variable = plugin_loader.createUniqueInstance("fuse_variables::DummyVariable");
  // Slight issue here -- no way to initialize the variable this way
  // May actually be fine, as this is only intended for deserialization anyway

  // Serialize the variable
  std::stringstream stream;
  {
    cereal::JSONOutputArchive archive(stream);
    archive(CEREAL_NVP(variable));
  }
  std::cout << stream.str() << std::endl;
}

TEST(Serialization, Message)
{
  // Create a variable instance
  auto stamp = ros::Time(12345678, 910111213);
  auto quest = "Draw the sword from the stone"s;
  fuse_core::Variable::SharedPtr variable = fuse_variables::DummyVariable::make_shared(stamp, quest);
  variable->data()[0] = 5.1;
  variable->data()[1] = -0.03;

  // Create a message
  fuse_msgs::SerializedVariable msg;

  // Serialize the variable into the message
  fuse_core::serializeVariable(*variable, msg);
  std::cout << msg.data << std::endl;
}

TEST(Serialization, ConstraintInstance)
{
  // Create a variable instance
  auto stamp = ros::Time(12345678, 910111213);
  auto quest = "Rescue Queen Guenever"s;
  auto variable = fuse_variables::DummyVariable(stamp, quest);
  variable.a() = 5.1;
  variable.b() = -0.03;

  // Create a constraint instance
  auto mean = fuse_core::Vector2d();
  mean << 1.0, 2.0;
  auto cov = fuse_core::Matrix2d();
  cov << 1.0, 0.0, 0.0, 2.0;
  auto constraint = fuse_constraints::DummyConstraint(variable, mean, cov);

  // Serialize the variable
  // There are several different serialization formats. Using JSON here so the results are somewhat readable.
  std::stringstream stream;
  {
    cereal::JSONOutputArchive archive(stream);
    archive(CEREAL_NVP(constraint));
  }
  std::cout << stream.str() << std::endl;
}

TEST(Serialization, Transaction)
{
  // Create a variable instance
  auto stamp = ros::Time(12345678, 910111213);
  auto quest = "Rescue Queen Guenever"s;
  auto added_variable = fuse_variables::DummyVariable::make_shared(stamp, quest);
  added_variable->a() = 5.1;
  added_variable->b() = -0.03;

  // Create a constraint instance
  auto mean = fuse_core::Vector2d();
  mean << 1.0, 2.0;
  auto cov = fuse_core::Matrix2d();
  cov << 1.0, 0.0, 0.0, 2.0;
  auto added_constraint = fuse_constraints::DummyConstraint::make_shared(*added_variable, mean, cov);

  // Create some fake variables and constraints to remove
  auto removed_variable = fuse_core::uuid::generate();
  auto removed_constraint = fuse_core::uuid::generate();

  // Create a transaction
  auto transaction = fuse_core::Transaction();
  transaction.stamp(ros::Time(1, 2));
  transaction.addInvolvedStamp(ros::Time(3, 4));
  transaction.addVariable(added_variable);
  transaction.addConstraint(added_constraint);
  transaction.removeVariable(removed_variable);
  transaction.removeConstraint(removed_constraint);

  // Serialize
  std::stringstream stream;
  {
    cereal::JSONOutputArchive archive(stream);
    archive(CEREAL_NVP(transaction));
  }
  std::cout << stream.str() << std::endl;
}

TEST(Serialization, Graph)
{
  // Create a variable instance
  auto stamp = ros::Time(12345678, 910111213);
  auto quest = "Rescue Queen Guenever"s;
  auto added_variable = fuse_variables::DummyVariable::make_shared(stamp, quest);
  added_variable->a() = 5.1;
  added_variable->b() = -0.03;

  // Create a constraint instance
  auto mean = fuse_core::Vector2d();
  mean << 1.0, 2.0;
  auto cov = fuse_core::Matrix2d();
  cov << 1.0, 0.0, 0.0, 2.0;
  auto added_constraint = fuse_constraints::DummyConstraint::make_shared(*added_variable, mean, cov);

  // Create a graph
  auto graph = fuse_graphs::HashGraph();
  graph.addVariable(added_variable);
  graph.addConstraint(added_constraint);

  // Serialize
  std::stringstream stream;
  {
    cereal::JSONOutputArchive archive(stream);
    archive(CEREAL_NVP(graph));
  }
  std::cout << stream.str() << std::endl;
}

TEST(Deserialization, VariableMessage)
{
  // Create a variable instance
  auto stamp = ros::Time(12345678, 910111213);
  auto quest = "Draw the sword from the stone"s;
  fuse_core::Variable::SharedPtr input = fuse_variables::DummyVariable::make_shared(stamp, quest);
  input->data()[0] = 5.1;
  input->data()[1] = -0.03;

  // Create a message
  fuse_msgs::SerializedVariable msg;

  // Serialize the variable into the message
  fuse_core::serializeVariable(*input, msg);

  fuse_core::VariableDeserializer deserializer;
  fuse_core::Variable::SharedPtr output = deserializer.deserialize(msg);
  output->print();
}

TEST(Deserialization, ConstraintMessage)
{
  // Create a variable instance
  auto stamp = ros::Time(12345678, 910111213);
  auto quest = "Rescue Queen Guenever"s;
  auto variable = fuse_variables::DummyVariable(stamp, quest);
  variable.a() = 5.1;
  variable.b() = -0.03;

  // Create a constraint instance
  auto mean = fuse_core::Vector2d();
  mean << 1.0, 2.0;
  auto cov = fuse_core::Matrix2d();
  cov << 1.0, 0.0, 0.0, 2.0;
  auto input = fuse_constraints::DummyConstraint::make_shared(variable, mean, cov);

  // Create a message
  fuse_msgs::SerializedConstraint msg;

  // Serialize the constraint into the message
  fuse_core::serializeConstraint(*input, msg);

  // Deserialize the constraint from the message
  fuse_core::ConstraintDeserializer deserializer;
  fuse_core::Constraint::SharedPtr output = deserializer.deserialize(msg);
  output->print();
}

TEST(Deserialization, TransactionMessage)
{
  // Create a variable instance
  auto stamp = ros::Time(12345678, 910111213);
  auto quest = "Find a shrubbery, a nice one that's not too expensive"s;
  auto added_variable = fuse_variables::DummyVariable::make_shared(stamp, quest);
  added_variable->a() = 5.1;
  added_variable->b() = -0.03;

  // Create a constraint instance
  auto mean = fuse_core::Vector2d();
  mean << 1.0, 2.0;
  auto cov = fuse_core::Matrix2d();
  cov << 1.0, 0.0, 0.0, 2.0;
  auto added_constraint = fuse_constraints::DummyConstraint::make_shared(*added_variable, mean, cov);

  // Create some fake variables and constraints to remove
  auto removed_variable = fuse_core::uuid::generate();
  auto removed_constraint = fuse_core::uuid::generate();

  // Create a transaction
  auto input = fuse_core::Transaction();
  input.stamp(ros::Time(1, 2));
  input.addInvolvedStamp(ros::Time(3, 4));
  input.addVariable(added_variable);
  input.addConstraint(added_constraint);
  input.removeVariable(removed_variable);
  input.removeConstraint(removed_constraint);

  // Create a message
  fuse_msgs::SerializedTransaction msg;

  // Serialize the constraint into the message
  fuse_core::serializeTransaction(input, msg);

  // Deserialize the constraint from the message
  fuse_core::TransactionDeserializer deserializer;
  fuse_core::Transaction output = deserializer.deserialize(msg);
  output.print();
}

TEST(Deserialization, GraphMessage)
{
  // Create a variable instance
  auto stamp = ros::Time(12345678, 910111213);
  auto quest = "Cut down the mightiest tree with a herring"s;
  auto added_variable = fuse_variables::DummyVariable::make_shared(stamp, quest);
  added_variable->a() = 5.1;
  added_variable->b() = -0.03;

  // Create a constraint instance
  auto mean = fuse_core::Vector2d();
  mean << 1.0, 2.0;
  auto cov = fuse_core::Matrix2d();
  cov << 1.0, 0.0, 0.0, 2.0;
  auto added_constraint = fuse_constraints::DummyConstraint::make_shared(*added_variable, mean, cov);

  // Create a graph
  auto input = fuse_graphs::HashGraph::make_shared();
  input->addVariable(added_variable);
  input->addConstraint(added_constraint);

  // Create a message
  fuse_msgs::SerializedGraph msg;

  // Serialize the graph into the message
  fuse_core::serializeGraph(*input, msg);

  // Deserialize the graph
  fuse_core::GraphDeserializer deserializer;
  fuse_core::Graph::SharedPtr output = deserializer.deserialize(msg);
  output->print();
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
