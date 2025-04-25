/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, Giacomo Franchini
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
#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <future>
#include <string>
#include <utility>
#include <vector>

#include <fuse_constraints/absolute_constraint.hpp>
#include <fuse_constraints/absolute_orientation_3d_stamped_constraint.hpp>
#include <fuse_core/eigen.hpp>
#include <fuse_core/util.hpp>
#include <fuse_core/eigen_gtest.hpp>
#include <fuse_core/transaction.hpp>
#include <fuse_models/omnidirectional_3d_ignition.hpp>
#include <fuse_msgs/srv/set_pose.hpp>
#include <fuse_msgs/srv/set_pose_deprecated.hpp>
#include <fuse_variables/orientation_3d_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

using fuse_constraints::AbsolutePosition3DStampedConstraint;
using fuse_constraints::AbsoluteOrientation3DStampedConstraint;
using fuse_constraints::AbsoluteVelocityLinear3DStampedConstraint;
using fuse_constraints::AbsoluteVelocityAngular3DStampedConstraint;
using fuse_constraints::AbsoluteAccelerationLinear3DStampedConstraint;


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
 * @brief Helper function for fetching the desired constraint from a transaction
 */
template<typename Derived>
const Derived * getConstraint(const fuse_core::Transaction & transaction)
{
  for (const auto & constraint : transaction.addedConstraints()) {
    auto derived = dynamic_cast<const Derived *>(&constraint);
    if (derived) {
      return derived;
    }
  }
  return nullptr;
}


class Omnidirectional3DIgnitionTestFixture : public ::testing::Test
{
public:
  Omnidirectional3DIgnitionTestFixture()
  {
  }

  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    spinner_ = std::thread(
      [&]() {
        executor_->spin();
      });
  }

  void TearDown() override
  {
    executor_->cancel();
    if (spinner_.joinable()) {
      spinner_.join();
    }
    executor_.reset();
    rclcpp::shutdown();
  }

  std::thread spinner_;   //!< Internal thread for spinning the executor
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
};

TEST_F(Omnidirectional3DIgnitionTestFixture, InitialTransaction)
{
  // Set some configuration
  rclcpp::NodeOptions options;
  options.arguments(
  {
    "--ros-args",
    "-p", "ignition_sensor.initial_state:="
    "[0.1, 1.2, 2.3, 0.1, 0.2, 0.3, 6.7, 7.8, 8.9, 9.1, 10.2, 11.3, 12.4, 13.5, 14.6]",
    "-p", "ignition_sensor.initial_sigma:="
    "[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0]"});
  auto node = rclcpp::Node::make_shared("omnidirectional_3d_ignition_test", options);
  executor_->add_node(node);

  // Initialize the callback promise. Promises are single-use.
  callback_promise = std::promise<fuse_core::Transaction::SharedPtr>();
  auto callback_future = callback_promise.get_future();

  // Create an ignition sensor and register the callback
  fuse_models::Omnidirectional3DIgnition ignition_sensor;
  ignition_sensor.initialize(*node, "ignition_sensor", &transactionCallback);
  ignition_sensor.start();

  // The ignition sensor should publish a transaction immediately. Wait for the callback to fire.
  auto status = callback_future.wait_for(std::chrono::seconds(5));
  ASSERT_TRUE(status == std::future_status::ready);

  // Check the transaction
  auto transaction = callback_future.get();
  {
    fuse_core::Vector3d expected_mean;
    expected_mean << 0.1, 1.2, 2.3;
    fuse_core::Matrix3d expected_cov;
    expected_cov << 1.0, 0.0, 0.0, 0.0, 4.0, 0.0, 0.0, 0.0, 9.0;
    auto actual = getConstraint<AbsolutePosition3DStampedConstraint>(*transaction);
    ASSERT_TRUE(static_cast<bool>(actual));
    EXPECT_MATRIX_NEAR(expected_mean, actual->mean(), 1.0e-9);
    EXPECT_MATRIX_NEAR(expected_cov, actual->covariance(), 1.0e-9);
  }
  {
    fuse_core::Vector3d expected_mean;
    expected_mean << 0.1, 0.2, 0.3;
    fuse_core::Matrix3d expected_cov;
    expected_cov << 16.0, 0.0, 0.0, 0.0, 25.0, 0.0, 0.0, 0.0, 36.0;
    auto actual = getConstraint<AbsoluteOrientation3DStampedConstraint>(*transaction);
    ASSERT_TRUE(static_cast<bool>(actual));
    fuse_variables::Orientation3DStamped orientation_actual;
    orientation_actual.w() = actual->mean()[0];
    orientation_actual.x() = actual->mean()[1];
    orientation_actual.y() = actual->mean()[2];
    orientation_actual.z() = actual->mean()[3];
    EXPECT_NEAR(expected_mean.x(), orientation_actual.roll(), 1.0e-9);
    EXPECT_NEAR(expected_mean.y(), orientation_actual.pitch(), 1.0e-9);
    EXPECT_NEAR(expected_mean.z(), orientation_actual.yaw(), 1.0e-9);
    EXPECT_MATRIX_NEAR(expected_cov, actual->covariance(), 1.0e-9);
  }
  {
    fuse_core::Vector3d expected_mean;
    expected_mean << 6.7, 7.8, 8.9;
    fuse_core::Matrix3d expected_cov;
    expected_cov << 49.0, 0.0, 0.0, 0.0, 64.0, 0.0, 0.0, 0.0, 81.0;
    auto actual = getConstraint<AbsoluteVelocityLinear3DStampedConstraint>(*transaction);
    ASSERT_TRUE(static_cast<bool>(actual));
    EXPECT_MATRIX_NEAR(expected_mean, actual->mean(), 1.0e-9);
    EXPECT_MATRIX_NEAR(expected_cov, actual->covariance(), 1.0e-9);
  }
  {
    fuse_core::Vector3d expected_mean;
    expected_mean << 9.1, 10.2, 11.3;
    fuse_core::Matrix3d expected_cov;
    expected_cov << 100.0, 0.0, 0.0, 0.0, 121.0, 0.0, 0.0, 0.0, 144.0;
    auto actual = getConstraint<AbsoluteVelocityAngular3DStampedConstraint>(*transaction);
    ASSERT_TRUE(static_cast<bool>(actual));
    EXPECT_MATRIX_NEAR(expected_mean, actual->mean(), 1.0e-9);
    EXPECT_MATRIX_NEAR(expected_cov, actual->covariance(), 1.0e-9);
  }
  {
    fuse_core::Vector3d expected_mean;
    expected_mean << 12.4, 13.5, 14.6;
    fuse_core::Matrix3d expected_cov;
    expected_cov << 169.0, 0.0, 0.0, 0.0, 196.0, 0.0, 0.0, 0.0, 225.0;
    auto actual = getConstraint<AbsoluteAccelerationLinear3DStampedConstraint>(*transaction);
    ASSERT_TRUE(static_cast<bool>(actual));
    EXPECT_MATRIX_NEAR(expected_mean, actual->mean(), 1.0e-9);
    EXPECT_MATRIX_NEAR(expected_cov, actual->covariance(), 1.0e-9);
  }
}

TEST_F(Omnidirectional3DIgnitionTestFixture, SkipInitialTransaction)
{
  // Set some configuration
  rclcpp::NodeOptions options;
  options.arguments(
  {
    "--ros-args",
    "-p", "ignition_sensor.initial_state:="
    "[0.1, 1.2, 2.3, 0.1, 0.2, 0.3, 6.7, 7.8, 8.9, 9.1, 10.2, 11.3, 12.4, 13.5, 14.6]",
    "-p", "ignition_sensor.initial_sigma:="
    "[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0]",
    "-p", "ignition_sensor.reset_service:=''",
    "-p", "ignition_sensor.publish_on_startup:=false"});
  auto node = rclcpp::Node::make_shared("omnidirectional_3d_ignition_test", options);
  executor_->add_node(node);

  // Initialize the callback promise. Promises are single-use.
  callback_promise = std::promise<fuse_core::Transaction::SharedPtr>();
  auto callback_future = callback_promise.get_future();

  // Create an ignition sensor and register the callback
  fuse_models::Omnidirectional3DIgnition ignition_sensor;
  ignition_sensor.initialize(*node, "ignition_sensor", &transactionCallback);
  ignition_sensor.start();

  // The ignition sensor should publish a transaction immediately. Wait for the callback to fire.
  auto status = callback_future.wait_for(std::chrono::seconds(5));
  ASSERT_FALSE(status == std::future_status::ready);
}

TEST_F(Omnidirectional3DIgnitionTestFixture, SetPoseService)
{
  // Set some configuration
  rclcpp::NodeOptions options;
  options.arguments(
  {
    "--ros-args",
    "-p", "ignition_sensor.initial_state:="
    "[0.1, 1.2, 2.3, 0.1, 0.2, 0.3, 6.7, 7.8, 8.9, 9.1, 10.2, 11.3, 12.4, 13.5, 14.6]",
    "-p", "ignition_sensor.initial_sigma:="
    "[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0]",
    "-p", "ignition_sensor.reset_service:=''",
    "-p", "ignition_sensor.publish_on_startup:=false"});
  auto node = rclcpp::Node::make_shared("omnidirectional_3d_ignition_test", options);
  executor_->add_node(node);

  // Initialize the callback promise. Promises are single-use.
  callback_promise = std::promise<fuse_core::Transaction::SharedPtr>();
  auto callback_future = callback_promise.get_future();

  // Create an ignition sensor and register the callback
  fuse_models::Omnidirectional3DIgnition ignition_sensor;
  ignition_sensor.initialize(*node, "ignition_sensor", &transactionCallback);
  ignition_sensor.start();

  Eigen::Quaterniond q;
  q = Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ());   // roll, pitch, yaw = 0.1 rad

  // Call the SetPose service
  auto srv = std::make_shared<fuse_msgs::srv::SetPose::Request>();
  srv->pose.header.stamp = rclcpp::Time(12, 345678910);
  srv->pose.pose.pose.position.x = 1.0;
  srv->pose.pose.pose.position.y = 2.0;
  srv->pose.pose.pose.position.z = 3.0;
  srv->pose.pose.pose.orientation.x = q.x();
  srv->pose.pose.pose.orientation.y = q.y();
  srv->pose.pose.pose.orientation.z = q.z();
  srv->pose.pose.pose.orientation.w = q.w();
  srv->pose.pose.covariance[0] = 1.0;
  srv->pose.pose.covariance[7] = 2.0;
  srv->pose.pose.covariance[14] = 3.0;
  srv->pose.pose.covariance[21] = 4.0;
  srv->pose.pose.covariance[28] = 5.0;
  srv->pose.pose.covariance[35] = 6.0;
  auto client = node->create_client<fuse_msgs::srv::SetPose>(
    "/omnidirectional_3d_ignition_test/set_pose");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(1)));
  auto result = client->async_send_request(srv);
  ASSERT_EQ(std::future_status::ready, result.wait_for(std::chrono::seconds(10)));
  EXPECT_TRUE(result.get()->success);

  // The ignition sensor should publish a transaction in response to the service call. Wait for the
  // callback to fire.
  auto status = callback_future.wait_for(std::chrono::seconds(5));
  ASSERT_TRUE(status == std::future_status::ready);

  // Check the transaction
  auto transaction = callback_future.get();
  {
    fuse_core::Vector3d expected_mean;
    expected_mean << 1.0, 2.0, 3.0;
    fuse_core::Matrix3d expected_cov;
    expected_cov << 1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 3.0;
    auto actual = getConstraint<AbsolutePosition3DStampedConstraint>(*transaction);
    ASSERT_TRUE(static_cast<bool>(actual));
    EXPECT_MATRIX_NEAR(expected_mean, actual->mean(), 1.0e-9);
    EXPECT_MATRIX_NEAR(expected_cov, actual->covariance(), 1.0e-9);
  }
  {
    fuse_core::Vector3d expected_mean;
    expected_mean << 0.1, 0.1, 0.1;
    fuse_core::Matrix3d expected_cov;
    expected_cov << 4.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 6.0;
    auto actual = getConstraint<AbsoluteOrientation3DStampedConstraint>(*transaction);
    ASSERT_TRUE(static_cast<bool>(actual));
    fuse_variables::Orientation3DStamped orientation_actual;
    orientation_actual.w() = actual->mean()[0];
    orientation_actual.x() = actual->mean()[1];
    orientation_actual.y() = actual->mean()[2];
    orientation_actual.z() = actual->mean()[3];
    EXPECT_NEAR(expected_mean.x(), orientation_actual.roll(), 1.0e-1);
    EXPECT_NEAR(expected_mean.y(), orientation_actual.pitch(), 1.0e-1);
    EXPECT_NEAR(expected_mean.z(), orientation_actual.yaw(), 1.0e-1);
    EXPECT_MATRIX_NEAR(expected_cov, actual->covariance(), 1.0e-9);
  }
  {
    fuse_core::Vector3d expected_mean;
    expected_mean << 6.7, 7.8, 8.9;
    fuse_core::Matrix3d expected_cov;
    expected_cov << 49.0, 0.0, 0.0, 0.0, 64.0, 0.0, 0.0, 0.0, 81.0;
    auto actual = getConstraint<AbsoluteVelocityLinear3DStampedConstraint>(*transaction);
    ASSERT_TRUE(static_cast<bool>(actual));
    EXPECT_MATRIX_NEAR(expected_mean, actual->mean(), 1.0e-9);
    EXPECT_MATRIX_NEAR(expected_cov, actual->covariance(), 1.0e-9);
  }
  {
    fuse_core::Vector3d expected_mean;
    expected_mean << 9.1, 10.2, 11.3;
    fuse_core::Matrix3d expected_cov;
    expected_cov << 100.0, 0.0, 0.0, 0.0, 121.0, 0.0, 0.0, 0.0, 144.0;
    auto actual = getConstraint<AbsoluteVelocityAngular3DStampedConstraint>(*transaction);
    ASSERT_TRUE(static_cast<bool>(actual));
    EXPECT_MATRIX_NEAR(expected_mean, actual->mean(), 1.0e-9);
    EXPECT_MATRIX_NEAR(expected_cov, actual->covariance(), 1.0e-9);
  }
  {
    fuse_core::Vector3d expected_mean;
    expected_mean << 12.4, 13.5, 14.6;
    fuse_core::Matrix3d expected_cov;
    expected_cov << 169.0, 0.0, 0.0, 0.0, 196.0, 0.0, 0.0, 0.0, 225.0;
    auto actual = getConstraint<AbsoluteAccelerationLinear3DStampedConstraint>(*transaction);
    ASSERT_TRUE(static_cast<bool>(actual));
    EXPECT_MATRIX_NEAR(expected_mean, actual->mean(), 1.0e-9);
    EXPECT_MATRIX_NEAR(expected_cov, actual->covariance(), 1.0e-9);
  }
}

TEST_F(Omnidirectional3DIgnitionTestFixture, SetPoseDeprecatedService)
{
  // Set some configuration
  rclcpp::NodeOptions options;
  options.arguments(
  {
    "--ros-args",
    "-p", "ignition_sensor.initial_state:="
    "[0.1, 1.2, 2.3, 0.1, 0.2, 0.3, 6.7, 7.8, 8.9, 9.1, 10.2, 11.3, 12.4, 13.5, 14.6]",
    "-p", "ignition_sensor.initial_sigma:="
    "[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0]",
    "-p", "ignition_sensor.reset_service:=''",
    "-p", "ignition_sensor.publish_on_startup:=false"});
  auto node = rclcpp::Node::make_shared("omnidirectional_3d_ignition_test", options);
  executor_->add_node(node);

  // Initialize the callback promise. Promises are single-use.
  callback_promise = std::promise<fuse_core::Transaction::SharedPtr>();
  auto callback_future = callback_promise.get_future();

  // Create an ignition sensor and register the callback
  fuse_models::Omnidirectional3DIgnition ignition_sensor;
  ignition_sensor.initialize(*node, "ignition_sensor", &transactionCallback);
  ignition_sensor.start();

  Eigen::Quaterniond q;
  q = Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ());   // roll, pitch, yaw = 0.1 rad

  // Call the SetPose service
  auto srv = std::make_shared<fuse_msgs::srv::SetPoseDeprecated::Request>();
  srv->pose.header.stamp = rclcpp::Time(12, 345678910);
  srv->pose.pose.pose.position.x = 1.0;
  srv->pose.pose.pose.position.y = 2.0;
  srv->pose.pose.pose.position.z = 3.0;
  srv->pose.pose.pose.orientation.x = q.x();
  srv->pose.pose.pose.orientation.y = q.y();
  srv->pose.pose.pose.orientation.z = q.z();
  srv->pose.pose.pose.orientation.w = q.w();
  srv->pose.pose.covariance[0] = 1.0;
  srv->pose.pose.covariance[7] = 2.0;
  srv->pose.pose.covariance[14] = 3.0;
  srv->pose.pose.covariance[21] = 4.0;
  srv->pose.pose.covariance[28] = 5.0;
  srv->pose.pose.covariance[35] = 6.0;
  auto client = node->create_client<fuse_msgs::srv::SetPoseDeprecated>(
    "/omnidirectional_3d_ignition_test/set_pose_deprecated");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(1)));
  auto result = client->async_send_request(srv);
  ASSERT_EQ(std::future_status::ready, result.wait_for(std::chrono::seconds(10)));

  // The ignition sensor should publish a transaction in response to the service call. Wait for the
  // callback to fire.
  auto status = callback_future.wait_for(std::chrono::seconds(5));
  ASSERT_TRUE(status == std::future_status::ready);

  // Check the transaction
  auto transaction = callback_future.get();
  {
    fuse_core::Vector3d expected_mean;
    expected_mean << 1.0, 2.0, 3.0;
    fuse_core::Matrix3d expected_cov;
    expected_cov << 1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 3.0;
    auto actual = getConstraint<AbsolutePosition3DStampedConstraint>(*transaction);
    ASSERT_TRUE(static_cast<bool>(actual));
    EXPECT_MATRIX_NEAR(expected_mean, actual->mean(), 1.0e-9);
    EXPECT_MATRIX_NEAR(expected_cov, actual->covariance(), 1.0e-9);
  }
  {
    fuse_core::Vector3d expected_mean;
    expected_mean << 0.1, 0.1, 0.1;
    fuse_core::Matrix3d expected_cov;
    expected_cov << 4.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 6.0;
    auto actual = getConstraint<AbsoluteOrientation3DStampedConstraint>(*transaction);
    ASSERT_TRUE(static_cast<bool>(actual));
    fuse_variables::Orientation3DStamped orientation_actual;
    orientation_actual.w() = actual->mean()[0];
    orientation_actual.x() = actual->mean()[1];
    orientation_actual.y() = actual->mean()[2];
    orientation_actual.z() = actual->mean()[3];
    EXPECT_NEAR(expected_mean.x(), orientation_actual.roll(), 1.0e-1); // not high precision
    EXPECT_NEAR(expected_mean.y(), orientation_actual.pitch(), 1.0e-1);
    EXPECT_NEAR(expected_mean.z(), orientation_actual.yaw(), 1.0e-1);
    EXPECT_MATRIX_NEAR(expected_cov, actual->covariance(), 1.0e-9);
  }
  {
    fuse_core::Vector3d expected_mean;
    expected_mean << 6.7, 7.8, 8.9;
    fuse_core::Matrix3d expected_cov;
    expected_cov << 49.0, 0.0, 0.0, 0.0, 64.0, 0.0, 0.0, 0.0, 81.0;
    auto actual = getConstraint<AbsoluteVelocityLinear3DStampedConstraint>(*transaction);
    ASSERT_TRUE(static_cast<bool>(actual));
    EXPECT_MATRIX_NEAR(expected_mean, actual->mean(), 1.0e-9);
    EXPECT_MATRIX_NEAR(expected_cov, actual->covariance(), 1.0e-9);
  }
  {
    fuse_core::Vector3d expected_mean;
    expected_mean << 9.1, 10.2, 11.3;
    fuse_core::Matrix3d expected_cov;
    expected_cov << 100.0, 0.0, 0.0, 0.0, 121.0, 0.0, 0.0, 0.0, 144.0;
    auto actual = getConstraint<AbsoluteVelocityAngular3DStampedConstraint>(*transaction);
    ASSERT_TRUE(static_cast<bool>(actual));
    EXPECT_MATRIX_NEAR(expected_mean, actual->mean(), 1.0e-9);
    EXPECT_MATRIX_NEAR(expected_cov, actual->covariance(), 1.0e-9);
  }
  {
    fuse_core::Vector3d expected_mean;
    expected_mean << 12.4, 13.5, 14.6;
    fuse_core::Matrix3d expected_cov;
    expected_cov << 169.0, 0.0, 0.0, 0.0, 196.0, 0.0, 0.0, 0.0, 225.0;
    auto actual = getConstraint<AbsoluteAccelerationLinear3DStampedConstraint>(*transaction);
    ASSERT_TRUE(static_cast<bool>(actual));
    EXPECT_MATRIX_NEAR(expected_mean, actual->mean(), 1.0e-9);
    EXPECT_MATRIX_NEAR(expected_cov, actual->covariance(), 1.0e-9);
  }
}
