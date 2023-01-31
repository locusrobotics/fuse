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
#include <algorithm>
#include <string>
#include <unordered_map>
#include <vector>

#include "common.hpp"
#include "example_optimizer.hpp"
#include <fuse_graphs/hash_graph.hpp>

#include <gtest/gtest.h>  // NOLINT


TEST(Optimizer, Constructor)
{
  // Create optimizer:
  auto node = std::make_shared<rclcpp::Node>("example_optimizer_node");
  ExampleOptimizer optimizer(*node);

  // Check the motion and sensor models, and publishers were loaded:
  const auto & motion_models = optimizer.getMotionModels();
  const auto & sensor_models = optimizer.getSensorModels();
  const auto & publishers = optimizer.getPublishers();

  EXPECT_FALSE(motion_models.empty());
  EXPECT_FALSE(sensor_models.empty());
  EXPECT_FALSE(publishers.empty());

  // Check the expected motion and sensor models, and publisher were loaded:
  const std::vector<std::string> expected_motion_models = {"noisy_unicycle_2d", "unicycle_2d"};
  const std::vector<std::string> expected_sensor_models = {"imu", "laser_localization",   // NOLINT
    "unicycle_2d_ignition", "wheel_odometry"};
  const std::vector<std::string> expected_publishers =
  {"odometry_publisher", "serialized_publisher"};

  ASSERT_TRUE(std::is_sorted(expected_motion_models.begin(), expected_motion_models.end()))
    << expected_motion_models << " is not sorted.";
  ASSERT_TRUE(std::is_sorted(expected_sensor_models.begin(), expected_sensor_models.end()))
    << expected_sensor_models << " is not sorted.";
  ASSERT_TRUE(std::is_sorted(expected_publishers.begin(), expected_publishers.end()))
    << expected_publishers << " is not sorted.";

  // Compute the symmetric difference between the expected and actual motion and sensor models, and
  // publishers:
  const auto difference_motion_models = set_symmetric_difference(
    expected_motion_models,
    motion_models);
  const auto difference_sensor_models = set_symmetric_difference(
    expected_sensor_models,
    sensor_models);
  const auto difference_publishers = set_symmetric_difference(expected_publishers, publishers);

  // Check the symmetric difference is empty, i.e. the actual motion and sensor models, and
  // publishers are the same as the expected ones:
  EXPECT_TRUE(difference_motion_models.empty())
    << "Actual: " << motion_models << "\nExpected: " << expected_motion_models
    << "\nDifference: " << difference_motion_models;
  EXPECT_TRUE(difference_sensor_models.empty())
    << "Actual: " << sensor_models << "\nExpected: " << expected_sensor_models
    << "\nDifference: " << difference_sensor_models;
  EXPECT_TRUE(difference_publishers.empty())
    << "Actual: " << publishers << "\nExpected: " << expected_publishers << "\nDifference: " <<
    difference_publishers;
}


// NOTE(CH3): This main is required because the test is manually run by a launch test
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
