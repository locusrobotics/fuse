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
#include <fuse_constraints/absolute_pose_2d_stamped_constraint.hpp>
#include <fuse_core/eigen.hpp>
#include <fuse_core/transaction.hpp>
#include <fuse_core/uuid.hpp>
#include <fuse_graphs/hash_graph.hpp>
#include <fuse_publishers/path_2d_publisher.h>
#include <fuse_variables/orientation_2d_stamped.hpp>
#include <fuse_variables/position_2d_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
// Workaround ros2/geometry2#242
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // NOLINT(build/include_order)
#include <tf2/utils.h>

#include <gtest/gtest.h>

#include <vector>


/**
 * @brief Test fixture for the Path2DPublisher
 *
 * This test fixture provides a populated graph for testing the publish() function, and a subscriber callback
 * for the 'path' output topics.
 */
class Path2DPublisherTestFixture : public ::testing::Test
{
public:
  Path2DPublisherTestFixture() :
    graph_(fuse_graphs::HashGraph::make_shared()),
    transaction_(fuse_core::Transaction::make_shared()),
    received_path_msg_(false),
    received_pose_array_msg_(false)
  {
    // Add a few pose variables
    auto position1 =
      fuse_variables::Position2DStamped::make_shared(rclcpp::Time(1234, 10, RCL_ROS_TIME));
    position1->x() = 1.01;
    position1->y() = 2.01;
    auto orientation1 =
      fuse_variables::Orientation2DStamped::make_shared(rclcpp::Time(1234, 10, RCL_ROS_TIME));
    orientation1->yaw() = 3.01;
    auto position2 =
      fuse_variables::Position2DStamped::make_shared(rclcpp::Time(1235, 10, RCL_ROS_TIME));
    position2->x() = 1.02;
    position2->y() = 2.02;
    auto orientation2 =
      fuse_variables::Orientation2DStamped::make_shared(rclcpp::Time(1235, 10, RCL_ROS_TIME));
    orientation2->yaw() = 3.02;
    auto position3 =
      fuse_variables::Position2DStamped::make_shared(rclcpp::Time(1235, 9, RCL_ROS_TIME));
    position3->x() = 1.03;
    position3->y() = 2.03;
    auto orientation3 =
      fuse_variables::Orientation2DStamped::make_shared(rclcpp::Time(1235, 9, RCL_ROS_TIME));
    orientation3->yaw() = 3.03;
    auto position4 = fuse_variables::Position2DStamped::make_shared(
      rclcpp::Time(1235, 11, RCL_ROS_TIME), fuse_core::uuid::generate("kitt"));
    position4->x() = 1.04;
    position4->y() = 2.04;
    auto orientation4 = fuse_variables::Orientation2DStamped::make_shared(
        rclcpp::Time(1235, 11, RCL_ROS_TIME), fuse_core::uuid::generate("kitt"));
    orientation4->yaw() = 3.04;

    transaction_->addInvolvedStamp(position1->stamp());
    transaction_->addInvolvedStamp(orientation1->stamp());
    transaction_->addInvolvedStamp(position2->stamp());
    transaction_->addInvolvedStamp(orientation2->stamp());
    transaction_->addInvolvedStamp(position3->stamp());
    transaction_->addInvolvedStamp(orientation3->stamp());
    transaction_->addInvolvedStamp(position4->stamp());
    transaction_->addInvolvedStamp(orientation4->stamp());
    transaction_->addVariable(position1);
    transaction_->addVariable(orientation1);
    transaction_->addVariable(position2);
    transaction_->addVariable(orientation2);
    transaction_->addVariable(position3);
    transaction_->addVariable(orientation3);
    transaction_->addVariable(position4);
    transaction_->addVariable(orientation4);
    // Add some priors on the variables some we can optimize the graph
    fuse_core::Vector3d mean1;
    mean1 << 1.01, 2.01, 3.01;
    fuse_core::Matrix3d cov1;
    /* *INDENT-OFF* */
    cov1 << 1.01, 0.0, 0.0,  0.0, 2.01, 0.0,  0.0, 0.0, 3.01;
    /* *INDENT-ON* */
    auto constraint1 =
      fuse_constraints::AbsolutePose2DStampedConstraint::make_shared(
        "test", *position1, *orientation1, mean1, cov1);
    fuse_core::Vector3d mean2;
    mean2 << 1.02, 2.02, 3.02;
    fuse_core::Matrix3d cov2;
    /* *INDENT-OFF* */
    cov2 << 1.02, 0.0, 0.0,  0.0, 2.02, 0.0,  0.0, 0.0, 3.02;
    /* *INDENT-ON* */
    auto constraint2 =
      fuse_constraints::AbsolutePose2DStampedConstraint::make_shared(
        "test", *position2, *orientation2, mean2, cov2);
    fuse_core::Vector3d mean3;
    mean3 << 1.03, 2.03, 3.03;
    fuse_core::Matrix3d cov3;
    /* *INDENT-OFF* */
    cov3 << 1.03, 0.0, 0.0,  0.0, 2.03, 0.0,  0.0, 0.0, 3.03;
    /* *INDENT-ON* */
    auto constraint3 =
      fuse_constraints::AbsolutePose2DStampedConstraint::make_shared(
        "test", *position3, *orientation3, mean3, cov3);
    fuse_core::Vector3d mean4;
    mean4 << 1.04, 2.04, 3.04;
    fuse_core::Matrix3d cov4;
    /* *INDENT-OFF* */
    cov4 << 1.04, 0.0, 0.0,  0.0, 2.04, 0.0,  0.0, 0.0, 3.04;
    /* *INDENT-ON* */
    auto constraint4 =
      fuse_constraints::AbsolutePose2DStampedConstraint::make_shared(
        "test", *position4, *orientation4, mean4, cov4);
    transaction_->addConstraint(constraint1);
    transaction_->addConstraint(constraint2);
    transaction_->addConstraint(constraint3);
    transaction_->addConstraint(constraint4);
    // Add the transaction to the graph
    graph_->update(*transaction_);
    // Optimize the graph
    graph_->optimize();
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

  void pathCallback(const nav_msgs::msg::Path& msg)
  {
    path_msg_ = msg;
    received_path_msg_ = true;
  }

  void poseArrayCallback(const geometry_msgs::msg::PoseArray& msg)
  {
    pose_array_msg_ = msg;
    received_pose_array_msg_ = true;
  }

   std::thread spinner_;  //!< Internal thread for spinning the executor
   rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;

protected:
  fuse_graphs::HashGraph::SharedPtr graph_;
  fuse_core::Transaction::SharedPtr transaction_;
  bool received_path_msg_;
  nav_msgs::msg::Path path_msg_;
  bool received_pose_array_msg_;
  geometry_msgs::msg::PoseArray pose_array_msg_;
};

TEST_F(Path2DPublisherTestFixture, PublishPath)
{
  // Test that the expected PoseStamped message is published
  rclcpp::NodeOptions options;
  options.arguments({
    "--ros-args",
    "-p", "frame_id:=test_map"});
  auto node = rclcpp::Node::make_shared("test_path_2d_publisher_node", options);
  executor_->add_node(node);

  // Create a publisher and send it the graph
  fuse_publishers::Path2DPublisher publisher;

  // This is the name of the pub's inner node (to set params against)
  publisher.initialize(node, "test_publisher");
  publisher.start();

  // Subscribe to the "path" topic
  auto subscriber1 = node->create_subscription<nav_msgs::msg::Path>(
    "/test_publisher/path", 1,
    std::bind(&Path2DPublisherTestFixture::pathCallback, this, std::placeholders::_1));

  // Subscribe to the "pose_array" topic
  auto subscriber2 = node->create_subscription<geometry_msgs::msg::PoseArray>(
    "/test_publisher/pose_array", 1,
    std::bind(&Path2DPublisherTestFixture::poseArrayCallback, this, std::placeholders::_1));

  // Send the graph to the Publisher to trigger message publishing
  publisher.notify(transaction_, graph_);

  // Verify the subscriber received the expected pose
  rclcpp::Time timeout = node->now() + rclcpp::Duration::from_seconds(10.0);
  while ((!received_path_msg_) && (node->now() < timeout))
  {
    rclcpp::sleep_for(rclcpp::Duration::from_seconds(0.10).to_chrono<std::chrono::nanoseconds>());
  }

  ASSERT_TRUE(received_path_msg_);
  EXPECT_EQ(rclcpp::Time(1235, 10, RCL_ROS_TIME), path_msg_.header.stamp);
  EXPECT_EQ("test_map", path_msg_.header.frame_id);
  ASSERT_EQ(3ul, path_msg_.poses.size());

  EXPECT_EQ(rclcpp::Time(1234, 10, RCL_ROS_TIME), path_msg_.poses[0].header.stamp);
  EXPECT_EQ("test_map", path_msg_.poses[0].header.frame_id);
  EXPECT_NEAR(1.01, path_msg_.poses[0].pose.position.x, 1.0e-9);
  EXPECT_NEAR(2.01, path_msg_.poses[0].pose.position.y, 1.0e-9);
  EXPECT_NEAR(0.00, path_msg_.poses[0].pose.position.z, 1.0e-9);
  EXPECT_NEAR(3.01, tf2::getYaw(path_msg_.poses[0].pose.orientation), 1.0e-9);

  EXPECT_EQ(rclcpp::Time(1235, 9, RCL_ROS_TIME), path_msg_.poses[1].header.stamp);
  EXPECT_EQ("test_map", path_msg_.poses[1].header.frame_id);
  EXPECT_NEAR(1.03, path_msg_.poses[1].pose.position.x, 1.0e-9);
  EXPECT_NEAR(2.03, path_msg_.poses[1].pose.position.y, 1.0e-9);
  EXPECT_NEAR(0.00, path_msg_.poses[1].pose.position.z, 1.0e-9);
  EXPECT_NEAR(3.03, tf2::getYaw(path_msg_.poses[1].pose.orientation), 1.0e-9);

  EXPECT_EQ(rclcpp::Time(1235, 10, RCL_ROS_TIME), path_msg_.poses[2].header.stamp);
  EXPECT_EQ("test_map", path_msg_.poses[2].header.frame_id);
  EXPECT_NEAR(1.02, path_msg_.poses[2].pose.position.x, 1.0e-9);
  EXPECT_NEAR(2.02, path_msg_.poses[2].pose.position.y, 1.0e-9);
  EXPECT_NEAR(0.00, path_msg_.poses[2].pose.position.z, 1.0e-9);
  EXPECT_NEAR(3.02, tf2::getYaw(path_msg_.poses[2].pose.orientation), 1.0e-9);

  ASSERT_TRUE(received_pose_array_msg_);
  EXPECT_EQ(rclcpp::Time(1235, 10, RCL_ROS_TIME), pose_array_msg_.header.stamp);
  EXPECT_EQ("test_map", pose_array_msg_.header.frame_id);
  ASSERT_EQ(3ul, pose_array_msg_.poses.size());

  EXPECT_NEAR(1.01, pose_array_msg_.poses[0].position.x, 1.0e-9);
  EXPECT_NEAR(2.01, pose_array_msg_.poses[0].position.y, 1.0e-9);
  EXPECT_NEAR(0.00, pose_array_msg_.poses[0].position.z, 1.0e-9);
  EXPECT_NEAR(3.01, tf2::getYaw(pose_array_msg_.poses[0].orientation), 1.0e-9);

  EXPECT_NEAR(1.03, pose_array_msg_.poses[1].position.x, 1.0e-9);
  EXPECT_NEAR(2.03, pose_array_msg_.poses[1].position.y, 1.0e-9);
  EXPECT_NEAR(0.00, pose_array_msg_.poses[1].position.z, 1.0e-9);
  EXPECT_NEAR(3.03, tf2::getYaw(pose_array_msg_.poses[1].orientation), 1.0e-9);

  EXPECT_NEAR(1.02, pose_array_msg_.poses[2].position.x, 1.0e-9);
  EXPECT_NEAR(2.02, pose_array_msg_.poses[2].position.y, 1.0e-9);
  EXPECT_NEAR(0.00, pose_array_msg_.poses[2].position.z, 1.0e-9);
  EXPECT_NEAR(3.02, tf2::getYaw(pose_array_msg_.poses[2].orientation), 1.0e-9);
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
