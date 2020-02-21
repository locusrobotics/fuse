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
#include <fuse_optimizers/optimizer.h>
#include <fuse_graphs/hash_graph.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <unordered_map>
#include <vector>


/**
 * @brief Dummy optimizer that exposes the motion and sensor models, and the publishers, so we can check the expected
 * ones are loaded.
 */
class DummyOptimizer : public fuse_optimizers::Optimizer
{
public:
  SMART_PTR_DEFINITIONS(DummyOptimizer);

  DummyOptimizer(fuse_core::Graph::UniquePtr graph, const ros::NodeHandle& node_handle = ros::NodeHandle(),
                 const ros::NodeHandle& private_node_handle = ros::NodeHandle("~"))
    : fuse_optimizers::Optimizer(std::move(graph), node_handle, private_node_handle)
  {
  }

  const MotionModels& getMotionModels() const
  {
    return motion_models_;
  }

  const SensorModels& getSensorModels() const
  {
    return sensor_models_;
  }

  const Publishers& getPublishers() const
  {
    return publishers_;
  }

  void transactionCallback(
      const std::string& sensor_name,
      fuse_core::Transaction::SharedPtr transaction) override
  {
  }
};

/**
 * @brief Helper function to print the elements of std::vector<T> objects
 *
 * @param[in, out] os An output stream
 * @param[in] v A vector to print into the stream
 * @return The output stream with the vector printed into it
 */
template <typename T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& v)
{
  os << '[';

  if (!v.empty())
  {
    std::copy(v.begin(), v.end() - 1, std::ostream_iterator<T>(os, ", "));
    os << v.back();
  }

  os << ']';

  return os;
}

/**
 * @brief Helper function to print the keys of std::unordered_map<K, V> objects
 *
 * @param[in, out] os An output stream
 * @param[in] m An unordered map with the keys to print into the stream
 * @return The output stream with the vector printed into it
 */
template <typename K, typename V>
std::ostream& operator<<(std::ostream& os, const std::unordered_map<K, V>& m)
{
  os << '[';

  for (const auto& entry : m)
  {
    os << entry.first << ", ";
  }

  os << ']';

  return os;
}

/**
 * @brief Helper function to compute the symmetric difference between a sorted std::vector<std::string> and the keys of
 * an std::unordered_map<std::string, T>
 *
 * @param[in] lhs A sorted vector of strings
 * @param[in] rhs An unordered map of key strings
 * @return A vector with the symmetric difference strings
 */
template <typename T>
std::vector<std::string> set_symmetric_difference(const std::vector<std::string>& lhs,
                                                  const std::unordered_map<std::string, T>& rhs)
{
  // Retrieve the keys:
  std::vector<std::string> rhs_keys;
  std::transform(rhs.begin(), rhs.end(), std::back_inserter(rhs_keys),
                 [](const auto& pair) { return pair.first; });  // NOLINT(whitespace/braces)

  // Sort the keys so we can use std::set_symmetric_difference:
  std::sort(rhs_keys.begin(), rhs_keys.end());

  // Compute the symmetric difference:
  std::vector<std::string> diff;
  std::set_symmetric_difference(lhs.begin(), lhs.end(), rhs_keys.begin(), rhs_keys.end(), std::back_inserter(diff));

  return diff;
}

TEST(Optimizer, Constructor)
{
  // Create dummy optimizer:
  DummyOptimizer optimizer(fuse_graphs::HashGraph::make_unique());

  // Check the motion and sensor models, and publishers were loaded:
  const auto& motion_models = optimizer.getMotionModels();
  const auto& sensor_models = optimizer.getSensorModels();
  const auto& publishers = optimizer.getPublishers();

  EXPECT_FALSE(motion_models.empty());
  EXPECT_FALSE(sensor_models.empty());
  EXPECT_FALSE(publishers.empty());

  // Check the expected motion and sensor models, and publisher were loaded:
  const std::vector<std::string> expected_motion_models = { "noisy_unicycle_2d", "unicycle_2d" };
  const std::vector<std::string> expected_sensor_models = { "imu", "laser_localization",  // NOLINT(whitespace/braces)
                                                            "unicycle_2d_ignition", "wheel_odometry" };
  const std::vector<std::string> expected_publishers = { "odometry_publisher", "serialized_publisher" };

  ASSERT_TRUE(std::is_sorted(expected_motion_models.begin(), expected_motion_models.end()))
      << expected_motion_models << " is not sorted.";
  ASSERT_TRUE(std::is_sorted(expected_sensor_models.begin(), expected_sensor_models.end()))
      << expected_sensor_models << " is not sorted.";
  ASSERT_TRUE(std::is_sorted(expected_publishers.begin(), expected_publishers.end()))
      << expected_publishers << " is not sorted.";

  // Compute the symmetric difference between the expected and actual motion and sensor models, and publishers:
  const auto difference_motion_models = set_symmetric_difference(expected_motion_models, motion_models);
  const auto difference_sensor_models = set_symmetric_difference(expected_sensor_models, sensor_models);
  const auto difference_publishers = set_symmetric_difference(expected_publishers, publishers);

  // Check the symmetric difference is empty, i.e. the actual motion and sensor models, and publishers are the same as
  // the expected ones:
  EXPECT_TRUE(difference_motion_models.empty())
      << "Actual: " << motion_models << "\nExpected: " << expected_motion_models
      << "\nDifference: " << difference_motion_models;
  EXPECT_TRUE(difference_sensor_models.empty())
      << "Actual: " << sensor_models << "\nExpected: " << expected_sensor_models
      << "\nDifference: " << difference_sensor_models;
  EXPECT_TRUE(difference_publishers.empty())
      << "Actual: " << publishers << "\nExpected: " << expected_publishers << "\nDifference: " << difference_publishers;
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_optimizer");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
