/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Locus Robotics
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

#include <memory>
#include <mutex>
#include <thread>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

class FixedLagAutostartFixture : public ::testing::Test
{
public:
  FixedLagAutostartFixture()
  {
  }

  void SetUp() override
  {
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
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard lock(received_odom_mutex_);
    received_odom_msg_ = msg;
  }

  nav_msgs::msg::Odometry::SharedPtr get_last_odom_msg()
  {
    std::lock_guard lock(received_odom_mutex_);
    return received_odom_msg_;
  }

  std::thread spinner_;   //!< Internal thread for spinning the executor
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  nav_msgs::msg::Odometry::SharedPtr received_odom_msg_;
  std::mutex received_odom_mutex_;
};

TEST_F(FixedLagAutostartFixture, AutostartProcessesFirstTransaction)
{
  // No ignition sensors are configured, so the smoother auto-starts and must process the very
  // first sensor transaction. This is a regression test for the optimizer thread terminating on
  // the first optimization cycle because lag_expiration_ was constructed with the wrong clock type.
  auto node = rclcpp::Node::make_shared("fixed_lag_autostart_test");
  executor_->add_node(node);

  auto pose_publisher =
    node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/absolute_pose", 5);

  auto odom_subscriber =
    node->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 5,
    std::bind(&FixedLagAutostartFixture::odom_callback, this, std::placeholders::_1));

  // Time should be valid after rclcpp::init() returns in main(). But it doesn't hurt to verify.
  ASSERT_TRUE(node->get_clock()->wait_until_started(rclcpp::Duration::from_seconds(1.0)));

  // The smoother auto-starts, so the sensors subscribe to their topics on startup.
  // I need to wait for those subscribers to be ready before sending them sensor data.
  rclcpp::Time subscriber_timeout = node->now() + rclcpp::Duration::from_seconds(10.0);
  while ((pose_publisher->get_subscription_count() < 1u) &&
    (node->now() < subscriber_timeout))
  {
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_GE(pose_publisher->get_subscription_count(), 1u);

  // Publish an absolute pose measurement
  auto pose_msg1 = geometry_msgs::msg::PoseWithCovarianceStamped();
  pose_msg1.header.stamp = rclcpp::Time(2, 0, RCL_ROS_TIME);
  pose_msg1.header.frame_id = "map";
  pose_msg1.pose.pose.position.x = 100.1;
  pose_msg1.pose.pose.position.y = 100.2;
  pose_msg1.pose.pose.position.z = 0.0;
  pose_msg1.pose.pose.orientation.x = 0.0;
  pose_msg1.pose.pose.orientation.y = 0.0;
  pose_msg1.pose.pose.orientation.z = 0.8660;
  pose_msg1.pose.pose.orientation.w = 0.5000;
  pose_msg1.pose.covariance[0] = 1.0;
  pose_msg1.pose.covariance[7] = 1.0;
  pose_msg1.pose.covariance[35] = 1.0;
  pose_publisher->publish(pose_msg1);

  // Force a delay between publishing, otherwise the subscriber does not receive all the messages
  rclcpp::sleep_for(std::chrono::milliseconds(100));

  auto pose_msg2 = geometry_msgs::msg::PoseWithCovarianceStamped();
  pose_msg2.header.stamp = rclcpp::Time(3, 0, RCL_ROS_TIME);
  pose_msg2.header.frame_id = "map";
  pose_msg2.pose.pose.position.x = 100.1;
  pose_msg2.pose.pose.position.y = 100.2;
  pose_msg2.pose.pose.position.z = 0.0;
  pose_msg2.pose.pose.orientation.x = 0.0;
  pose_msg2.pose.pose.orientation.y = 0.0;
  pose_msg2.pose.pose.orientation.z = 0.8660;
  pose_msg2.pose.pose.orientation.w = 0.5000;
  pose_msg2.pose.covariance[0] = 1.0;
  pose_msg2.pose.covariance[7] = 1.0;
  pose_msg2.pose.covariance[35] = 1.0;
  pose_publisher->publish(pose_msg2);

  // Wait for the optimizer to process all queued transactions and publish the last odometry msg
  rclcpp::Time result_timeout = node->now() + rclcpp::Duration::from_seconds(5.0);
  auto odom_msg = nav_msgs::msg::Odometry::SharedPtr();
  while ((!odom_msg || odom_msg->header.stamp != rclcpp::Time(3, 0,
    RCL_ROS_TIME)) && (node->now() < result_timeout))
  {
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    odom_msg = this->get_last_odom_msg();
  }
  ASSERT_TRUE(static_cast<bool>(odom_msg));
  ASSERT_EQ(rclcpp::Time(odom_msg->header.stamp), rclcpp::Time(3, 0, RCL_ROS_TIME));

  // Both pose measurements are identical, so the optimized state should converge to them.
  EXPECT_NEAR(100.1, odom_msg->pose.pose.position.x, 0.10);
  EXPECT_NEAR(100.2, odom_msg->pose.pose.position.y, 0.10);
  EXPECT_NEAR(0.8660, odom_msg->pose.pose.orientation.z, 0.10);
  EXPECT_NEAR(0.5000, odom_msg->pose.pose.orientation.w, 0.10);
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
