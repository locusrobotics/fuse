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
#include <ros/ros.h>
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
    private_node_handle_("~"),
    graph_(fuse_graphs::HashGraph::make_shared()),
    transaction_(fuse_core::Transaction::make_shared()),
    received_path_msg_(false),
    received_pose_array_msg_(false)
  {
    // Add a few pose variables
    auto position1 = fuse_variables::Position2DStamped::make_shared(rclcpp::Time(1234, 10));
    position1->x() = 1.01;
    position1->y() = 2.01;
    auto orientation1 = fuse_variables::Orientation2DStamped::make_shared(rclcpp::Time(1234, 10));
    orientation1->yaw() = 3.01;
    auto position2 = fuse_variables::Position2DStamped::make_shared(rclcpp::Time(1235, 10));
    position2->x() = 1.02;
    position2->y() = 2.02;
    auto orientation2 = fuse_variables::Orientation2DStamped::make_shared(rclcpp::Time(1235, 10));
    orientation2->yaw() = 3.02;
    auto position3 = fuse_variables::Position2DStamped::make_shared(rclcpp::Time(1235, 9));
    position3->x() = 1.03;
    position3->y() = 2.03;
    auto orientation3 = fuse_variables::Orientation2DStamped::make_shared(rclcpp::Time(1235, 9));
    orientation3->yaw() = 3.03;
    auto position4 = fuse_variables::Position2DStamped::make_shared(rclcpp::Time(1235, 11),
                                                                    fuse_core::uuid::generate("kitt"));
    position4->x() = 1.04;
    position4->y() = 2.04;
    auto orientation4 = fuse_variables::Orientation2DStamped::make_shared(rclcpp::Time(1235, 11),
                                                                          fuse_core::uuid::generate("kitt"));
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
    cov1 << 1.01, 0.0, 0.0,  0.0, 2.01, 0.0,  0.0, 0.0, 3.01;
    auto constraint1 = fuse_constraints::AbsolutePose2DStampedConstraint::make_shared(
      "test", *position1, *orientation1, mean1, cov1);
    fuse_core::Vector3d mean2;
    mean2 << 1.02, 2.02, 3.02;
    fuse_core::Matrix3d cov2;
    cov2 << 1.02, 0.0, 0.0,  0.0, 2.02, 0.0,  0.0, 0.0, 3.02;
    auto constraint2 = fuse_constraints::AbsolutePose2DStampedConstraint::make_shared(
      "test", *position2, *orientation2, mean2, cov2);
    fuse_core::Vector3d mean3;
    mean3 << 1.03, 2.03, 3.03;
    fuse_core::Matrix3d cov3;
    cov3 << 1.03, 0.0, 0.0,  0.0, 2.03, 0.0,  0.0, 0.0, 3.03;
    auto constraint3 = fuse_constraints::AbsolutePose2DStampedConstraint::make_shared(
      "test", *position3, *orientation3, mean3, cov3);
    fuse_core::Vector3d mean4;
    mean4 << 1.04, 2.04, 3.04;
    fuse_core::Matrix3d cov4;
    cov4 << 1.04, 0.0, 0.0,  0.0, 2.04, 0.0,  0.0, 0.0, 3.04;
    auto constraint4 = fuse_constraints::AbsolutePose2DStampedConstraint::make_shared(
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

protected:
  // TODO(CH3): Replace with node_ (also, we don't need to support node interfaces here since it's a test...)
  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
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

  // Create a publisher and send it the graph
  private_node_handle_.setParam("test_publisher/frame_id", "test_map");
  fuse_publishers::Path2DPublisher publisher;
  publisher.initialize("test_publisher");
  publisher.start();

  // Subscribe to the "path" topic
  ros::Subscriber subscriber1 = private_node_handle_.subscribe(
    "test_publisher/path",
    1,
    &Path2DPublisherTestFixture::pathCallback,
    reinterpret_cast<Path2DPublisherTestFixture*>(this));

  // Subscribe to the "pose_array" topic
  ros::Subscriber subscriber2 = private_node_handle_.subscribe(
    "test_publisher/pose_array",
    1,
    &Path2DPublisherTestFixture::poseArrayCallback,
    reinterpret_cast<Path2DPublisherTestFixture*>(this));

  // Send the graph to the Publisher to trigger message publishing
  publisher.notify(transaction_, graph_);

  // Verify the subscriber received the expected pose
  rclcpp::Time timeout = node_->now() + rclcpp::Duration::from_seconds(10.0);
  while ((!received_path_msg_) && (node_->now() < timeout))
  {
    rclcpp::sleep_for(rclcpp::Duration::from_seconds(0.10));
  }

  ASSERT_TRUE(received_path_msg_);
  EXPECT_EQ(rclcpp::Time(1235, 10), path_msg_.header.stamp);
  EXPECT_EQ("test_map", path_msg_.header.frame_id);
  ASSERT_EQ(3ul, path_msg_.poses.size());

  EXPECT_EQ(rclcpp::Time(1234, 10), path_msg_.poses[0].header.stamp);
  EXPECT_EQ("test_map", path_msg_.poses[0].header.frame_id);
  EXPECT_NEAR(1.01, path_msg_.poses[0].pose.position.x, 1.0e-9);
  EXPECT_NEAR(2.01, path_msg_.poses[0].pose.position.y, 1.0e-9);
  EXPECT_NEAR(0.00, path_msg_.poses[0].pose.position.z, 1.0e-9);
  EXPECT_NEAR(3.01, tf2::getYaw(path_msg_.poses[0].pose.orientation), 1.0e-9);

  EXPECT_EQ(rclcpp::Time(1235, 9), path_msg_.poses[1].header.stamp);
  EXPECT_EQ("test_map", path_msg_.poses[1].header.frame_id);
  EXPECT_NEAR(1.03, path_msg_.poses[1].pose.position.x, 1.0e-9);
  EXPECT_NEAR(2.03, path_msg_.poses[1].pose.position.y, 1.0e-9);
  EXPECT_NEAR(0.00, path_msg_.poses[1].pose.position.z, 1.0e-9);
  EXPECT_NEAR(3.03, tf2::getYaw(path_msg_.poses[1].pose.orientation), 1.0e-9);

  EXPECT_EQ(rclcpp::Time(1235, 10), path_msg_.poses[2].header.stamp);
  EXPECT_EQ("test_map", path_msg_.poses[2].header.frame_id);
  EXPECT_NEAR(1.02, path_msg_.poses[2].pose.position.x, 1.0e-9);
  EXPECT_NEAR(2.02, path_msg_.poses[2].pose.position.y, 1.0e-9);
  EXPECT_NEAR(0.00, path_msg_.poses[2].pose.position.z, 1.0e-9);
  EXPECT_NEAR(3.02, tf2::getYaw(path_msg_.poses[2].pose.orientation), 1.0e-9);


  ASSERT_TRUE(received_pose_array_msg_);
  EXPECT_EQ(rclcpp::Time(1235, 10), pose_array_msg_.header.stamp);
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

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_path_2d_publisher");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
