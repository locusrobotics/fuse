/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Locus Robotics
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
#include <fuse_publishers/stamped_variable_synchronizer.h>

#include <fuse_core/uuid.h>
#include <fuse_graphs/hash_graph.h>
#include <fuse_variables/orientation_2d_stamped.h>
#include <fuse_variables/position_2d_stamped.h>

#include <gtest/gtest.h>


using fuse_core::uuid::generate;
using fuse_publishers::StampedVariableSynchronizer;
using fuse_variables::Orientation2DStamped;
using fuse_variables::Position2DStamped;

TEST(StampedVariableSynchronizer, Constructor)
{
  StampedVariableSynchronizer<Orientation2DStamped> sync1(generate("I am Gun Robot"));

  StampedVariableSynchronizer<Orientation2DStamped, Position2DStamped> sync2;

  // StampedVariableSynchronizer<int, double, size_t> sync3;
  // This _correctly_ does not compile:
  // error: static assertion failed: All synchronized types must be derived from both fuse_core::Variable
  //                                 and fuse_variable::Stamped.

  // StampedVariableSynchronizer<> sync4;
  // This _correctly_ does not compile:
  // error: static assertion failed: At least one type must be specified.

  SUCCEED();
}

TEST(StampedVariableSynchronizer, FullSearch)
{
  // Don't provide an incremental transaction. This will force a full search of the graph since no timestamp has
  // been found before

  // Create the synchronizer
  auto sync = StampedVariableSynchronizer<Orientation2DStamped, Position2DStamped>(generate("blank"));

  // Define the transaction and graph
  auto transaction = fuse_core::Transaction();
  auto graph = fuse_graphs::HashGraph();
  graph.addVariable(fuse_variables::Orientation2DStamped::make_shared(rclcpp::Time(10, 0), generate("blank")));
  graph.addVariable(fuse_variables::Position2DStamped::make_shared(rclcpp::Time(10, 0), generate("blank")));
  graph.addVariable(fuse_variables::Orientation2DStamped::make_shared(rclcpp::Time(20, 0), generate("blank")));
  graph.addVariable(fuse_variables::Position2DStamped::make_shared(rclcpp::Time(20, 0), generate("blank")));
  graph.addVariable(fuse_variables::Orientation2DStamped::make_shared(rclcpp::Time(30, 0), generate("Dadblank")));
  graph.addVariable(fuse_variables::Position2DStamped::make_shared(rclcpp::Time(30, 0), generate("Dadblank")));
  graph.addVariable(fuse_variables::Orientation2DStamped::make_shared(rclcpp::Time(40, 0), generate("blank")));

  // Use the synchronizer
  auto actual = sync.findLatestCommonStamp(transaction, graph);

  // Expect Time(20, 0).
  // Time(30, 0) entries have a different device_id
  // Time(40, 0) entries don't have a whole set
  EXPECT_EQ(rclcpp::Time(20, 0), actual);
}

TEST(StampedVariableSynchronizer, Update)
{
  // Perform an initial search, then use the transaction to perform an incremental update

  // Create the synchronizer
  auto sync = StampedVariableSynchronizer<Orientation2DStamped, Position2DStamped>(generate("blank"));

  // Define the first transaction and graph
  auto transaction1 = fuse_core::Transaction();
  auto graph = fuse_graphs::HashGraph();
  graph.addVariable(fuse_variables::Orientation2DStamped::make_shared(rclcpp::Time(10, 0), generate("blank")));
  graph.addVariable(fuse_variables::Position2DStamped::make_shared(rclcpp::Time(10, 0), generate("blank")));
  graph.addVariable(fuse_variables::Orientation2DStamped::make_shared(rclcpp::Time(20, 0), generate("blank")));
  graph.addVariable(fuse_variables::Position2DStamped::make_shared(rclcpp::Time(20, 0), generate("blank")));

  // Use the synchronizer
  auto actual1 = sync.findLatestCommonStamp(transaction1, graph);
  EXPECT_EQ(rclcpp::Time(20, 0), actual1);

  // Create an incremental transaction update
  auto transaction2 = fuse_core::Transaction();
  transaction2.addVariable(fuse_variables::Orientation2DStamped::make_shared(rclcpp::Time(30, 0), generate("blank")));
  transaction2.addVariable(fuse_variables::Position2DStamped::make_shared(rclcpp::Time(30, 0), generate("blank")));
  transaction2.addVariable(fuse_variables::Orientation2DStamped::make_shared(rclcpp::Time(40, 0), generate("blank")));
  graph.addVariable(fuse_variables::Orientation2DStamped::make_shared(rclcpp::Time(30, 0), generate("blank")));
  graph.addVariable(fuse_variables::Position2DStamped::make_shared(rclcpp::Time(30, 0), generate("blank")));
  graph.addVariable(fuse_variables::Orientation2DStamped::make_shared(rclcpp::Time(40, 0), generate("blank")));

  // Use the synchronizer
  auto actual2 = sync.findLatestCommonStamp(transaction2, graph);
  EXPECT_EQ(rclcpp::Time(30, 0), actual2);
}

TEST(StampedVariableSynchronizer, Remove)
{
  // Perform an initial search, then use the transaction to remove the latest variables
  // Create the synchronizer
  auto sync = StampedVariableSynchronizer<Orientation2DStamped, Position2DStamped>(generate("blank"));

  // Define the first transaction and graph
  auto transaction1 = fuse_core::Transaction();
  auto graph = fuse_graphs::HashGraph();
  graph.addVariable(fuse_variables::Orientation2DStamped::make_shared(rclcpp::Time(10, 0), generate("blank")));
  graph.addVariable(fuse_variables::Position2DStamped::make_shared(rclcpp::Time(10, 0), generate("blank")));
  graph.addVariable(fuse_variables::Orientation2DStamped::make_shared(rclcpp::Time(20, 0), generate("blank")));
  graph.addVariable(fuse_variables::Position2DStamped::make_shared(rclcpp::Time(20, 0), generate("blank")));
  graph.addVariable(fuse_variables::Orientation2DStamped::make_shared(rclcpp::Time(30, 0), generate("blank")));
  graph.addVariable(fuse_variables::Position2DStamped::make_shared(rclcpp::Time(30, 0), generate("blank")));

  // Use the synchronizer
  auto actual1 = sync.findLatestCommonStamp(transaction1, graph);
  EXPECT_EQ(rclcpp::Time(30, 0), actual1);

  // Create an incremental transaction that removes one of the latest variables
  auto transaction2 = fuse_core::Transaction();
  transaction2.removeVariable(fuse_variables::Position2DStamped(rclcpp::Time(30, 0), generate("blank")).uuid());
  graph.removeVariable(fuse_variables::Position2DStamped(rclcpp::Time(30, 0), generate("blank")).uuid());

  // Use the synchronizer
  auto actual2 = sync.findLatestCommonStamp(transaction2, graph);
  EXPECT_EQ(rclcpp::Time(20, 0), actual2);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
