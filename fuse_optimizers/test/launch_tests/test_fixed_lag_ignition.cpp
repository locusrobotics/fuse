/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Locus Robotics
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

#include <fuse_msgs/srv/set_pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <std_srvs/srv/empty.hpp>

class FixedLagIgnitionFixture : public ::testing::Test
{
public:
  FixedLagIgnitionFixture()
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
    received_odom_msg_ = msg;
  }

  std::thread spinner_;   //!< Internal thread for spinning the executor
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  nav_msgs::msg::Odometry::SharedPtr received_odom_msg_;
};

TEST_F(FixedLagIgnitionFixture, SetInitialState)
{
  auto node = rclcpp::Node::make_shared("fixed_lag_ignition_test");
  executor_->add_node(node);

  auto relative_pose_publisher =
    node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/relative_pose", 5);

  auto odom_subscriber =
    node->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 5, std::bind(&FixedLagIgnitionFixture::odom_callback, this, std::placeholders::_1));

  // Time should be valid after rclcpp::init() returns in main(). But it doesn't hurt to verify.
  ASSERT_TRUE(node->get_clock()->wait_until_started(rclcpp::Duration::from_seconds(1.0)));

  // Wait for the optimizer to be ready
  auto set_pose_client = node->create_client<fuse_msgs::srv::SetPose>("/fixed_lag_node/set_pose");
  auto reset_client = node->create_client<std_srvs::srv::Empty>("/fixed_lag_node/reset");
  ASSERT_TRUE(set_pose_client->wait_for_service(std::chrono::seconds(1)));
  ASSERT_TRUE(reset_client->wait_for_service(std::chrono::seconds(1)));

  // Set the initial pose to something far away from zero
  auto req = std::make_shared<fuse_msgs::srv::SetPose::Request>();
  req->pose.header.frame_id = "map";
  req->pose.header.stamp = rclcpp::Time(1, 0, RCL_ROS_TIME);
  req->pose.pose.pose.position.x = 100.1;
  req->pose.pose.pose.position.y = 100.2;
  req->pose.pose.pose.position.z = 0.0;
  req->pose.pose.pose.orientation.x = 0.0;
  req->pose.pose.pose.orientation.y = 0.0;
  req->pose.pose.pose.orientation.z = 0.8660;
  req->pose.pose.pose.orientation.w = 0.5000;
  req->pose.pose.covariance[0] = 1.0;
  req->pose.pose.covariance[7] = 1.0;
  req->pose.pose.covariance[35] = 1.0;
  auto result = set_pose_client->async_send_request(req);
  ASSERT_EQ(std::future_status::ready, result.wait_for(std::chrono::seconds(10)));
  EXPECT_TRUE(result.get()->success);

  // The 'set_pose' service call triggers all of the sensors to resubscribe to their topics.
  // I need to wait for those subscribers to be ready before sending them sensor data.
  rclcpp::Time subscriber_timeout = node->now() + rclcpp::Duration::from_seconds(10.0);
  while ((relative_pose_publisher->get_subscription_count() < 1u) &&
    (node->now() < subscriber_timeout))
  {
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_GE(relative_pose_publisher->get_subscription_count(), 1u);

  // Publish a relative pose
  auto pose_msg1 = geometry_msgs::msg::PoseWithCovarianceStamped();
  pose_msg1.header.stamp = rclcpp::Time(2, 0, RCL_ROS_TIME);
  pose_msg1.header.frame_id = "base_link";
  pose_msg1.pose.pose.position.x = 5.0;
  pose_msg1.pose.pose.position.y = 6.0;
  pose_msg1.pose.pose.position.z = 0.0;
  pose_msg1.pose.pose.orientation.x = 0.0;
  pose_msg1.pose.pose.orientation.y = 0.0;
  pose_msg1.pose.pose.orientation.z = 0.900;
  pose_msg1.pose.pose.orientation.w = 0.436;
  pose_msg1.pose.covariance[0] = 1.0;
  pose_msg1.pose.covariance[7] = 1.0;
  pose_msg1.pose.covariance[35] = 1.0;
  relative_pose_publisher->publish(pose_msg1);

  /// @todo(swilliams) Understand why the subscriber does not receive all the messages
  /// unless I force a delay between publishing.
  rclcpp::sleep_for(std::chrono::milliseconds(100));

  auto pose_msg2 = geometry_msgs::msg::PoseWithCovarianceStamped();
  pose_msg2.header.stamp = rclcpp::Time(3, 0, RCL_ROS_TIME);
  pose_msg2.header.frame_id = "base_link";
  pose_msg2.pose.pose.position.x = 10.0;
  pose_msg2.pose.pose.position.y = 20.0;
  pose_msg2.pose.pose.position.z = 0.0;
  pose_msg2.pose.pose.orientation.x = 0.0;
  pose_msg2.pose.pose.orientation.y = 0.0;
  pose_msg2.pose.pose.orientation.z = 0.5000;
  pose_msg2.pose.pose.orientation.w = 0.8660;
  pose_msg2.pose.covariance[0] = 1.0;
  pose_msg2.pose.covariance[7] = 1.0;
  pose_msg2.pose.covariance[35] = 1.0;
  relative_pose_publisher->publish(pose_msg2);

  // Wait for the optimizer to process all queued transactions and publish the last odometry msg
  rclcpp::Time result_timeout = node->now() + rclcpp::Duration::from_seconds(1.0);
  while ((!received_odom_msg_ || received_odom_msg_->header.stamp != rclcpp::Time(3, 0,
    RCL_ROS_TIME)) && (node->now() < result_timeout))
  {
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  ASSERT_EQ(rclcpp::Time(received_odom_msg_->header.stamp), rclcpp::Time(3, 0, RCL_ROS_TIME));

  // The optimizer is configured for 0 iterations, so it should return the initial variable values
  // If we did our job correctly, the initial variable values should be the same as the service call
  // state, give or take the motion model forward prediction.
  EXPECT_NEAR(100.1, received_odom_msg_->pose.pose.position.x, 0.10);
  EXPECT_NEAR(100.2, received_odom_msg_->pose.pose.position.y, 0.10);
  EXPECT_NEAR(0.8660, received_odom_msg_->pose.pose.orientation.z, 0.10);
  EXPECT_NEAR(0.5000, received_odom_msg_->pose.pose.orientation.w, 0.10);
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
