/***************************************************************************
 * Copyright (C) 2024 Giacomo Franchini. All rights reserved.
 * Unauthorized copying of this file, via any medium, is strictly prohibited
 * Proprietary and confidential
 ***************************************************************************/
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <vector>

#include <fuse_models/common/sensor_proc.hpp>

namespace fm_common = fuse_models::common;
TEST(TestSuite, mergeFullPositionAndFullOrientationIndices)
{
  const std::vector<size_t> position_indices{0, 1, 2};
  const std::vector<size_t> orientation_indices{0, 1, 2};

  const size_t orientation_offset = 3;

  const auto merged_indices = fm_common::mergeIndices(
    position_indices, orientation_indices,
    orientation_offset);

  EXPECT_EQ(position_indices.size() + orientation_indices.size(), merged_indices.size());
  EXPECT_THAT(
    position_indices,
    testing::ElementsAreArray(
      merged_indices.begin(),
      merged_indices.begin() + position_indices.size()));
  EXPECT_EQ(orientation_indices.back() + orientation_offset, merged_indices.back());
}

TEST(TestSuite, mergeXYPositionAndRollYawOrientationIndices)
{
  const std::vector<size_t> position_indices{0, 1};
  const std::vector<size_t> orientation_indices{0, 2};

  const size_t orientation_offset = 3;

  const auto merged_indices = fm_common::mergeIndices(
    position_indices, orientation_indices,
    orientation_offset);

  EXPECT_EQ(position_indices.size() + orientation_indices.size(), merged_indices.size());
  EXPECT_THAT(
    position_indices,
    testing::ElementsAreArray(
      merged_indices.begin(),
      merged_indices.begin() + position_indices.size()));
  EXPECT_EQ(orientation_indices.back() + orientation_offset, merged_indices.back());
}

TEST(TestSuite, mergeFullPositionAndEmptyOrientationIndices)
{
  const std::vector<size_t> position_indices{0, 1, 2};
  const std::vector<size_t> orientation_indices;

  const size_t orientation_offset = 3;

  const auto merged_indices = fm_common::mergeIndices(
    position_indices, orientation_indices,
    orientation_offset);

  EXPECT_EQ(position_indices.size(), merged_indices.size());
  EXPECT_THAT(position_indices, testing::ElementsAreArray(merged_indices));
}

TEST(TestSuite, mergeEmptyPositionAndFullOrientationIndices)
{
  const std::vector<size_t> position_indices;
  const std::vector<size_t> orientation_indices{0, 1, 2};

  const size_t orientation_offset = 3;

  const auto merged_indices = fm_common::mergeIndices(
    position_indices, orientation_indices,
    orientation_offset);

  EXPECT_EQ(orientation_indices.size(), merged_indices.size());
  EXPECT_EQ(orientation_indices.back() + orientation_offset, merged_indices.back());
}

TEST(TestSuite, mergeEmptyPositionAndEmptyOrientationIndices)
{
  const std::vector<size_t> position_indices;
  const std::vector<size_t> orientation_indices;

  const size_t orientation_offset = 3;

  const auto merged_indices = fm_common::mergeIndices(
    position_indices, orientation_indices,
    orientation_offset);

  EXPECT_TRUE(merged_indices.empty());
}

TEST(TestSuite, populatePartialMeasurements)
{
  // Test both conversion from quaternion to RPY and partial measurement population
  // This one is just to generate a random unit quaternion and have the reference in RPY
  fuse_core::Vector3d rpy = fuse_core::Vector3d::Random();
  Eigen::Quaterniond q = 
    Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX());
  
  tf2::Transform tf2_pose;
  tf2_pose.setOrigin(tf2::Vector3(1.0, 2.0, 3.0));
  tf2_pose.setRotation(tf2::Quaternion(q.x(), q.y(), q.z(), q.w()));
  fuse_core::Vector6d pose_mean_partial;
  pose_mean_partial.head<3>() << tf2_pose.getOrigin().x(), tf2_pose.getOrigin().y(), tf2_pose.getOrigin().z();
  tf2::Matrix3x3(tf2_pose.getRotation()).getRPY(pose_mean_partial(3), pose_mean_partial(4), pose_mean_partial(5));
  
  fuse_core::Matrix6d pose_covariance = fuse_core::Matrix6d::Random();

  const std::vector<size_t> position_indices{0, 1};
  const std::vector<size_t> orientation_indices{2};

  const size_t orientation_offset = 3;

  const auto merged_indices = fm_common::mergeIndices(
    position_indices, orientation_indices,
    orientation_offset);
  
  std::replace_if(
    pose_mean_partial.data(), pose_mean_partial.data() + pose_mean_partial.size(),
    [&merged_indices, &pose_mean_partial](const double & value) {
      return std::find(
        merged_indices.begin(), 
        merged_indices.end(), 
        &value - pose_mean_partial.data()) == merged_indices.end();
    }, 0.0);

  fuse_core::MatrixXd pose_covariance_partial(3, 3);

  fm_common::populatePartialMeasurement(
    pose_covariance,
    merged_indices,
    pose_covariance_partial);

  fuse_core::Vector6d expected_pose;
  expected_pose << 1.0, 2.0, 0.0, 0.0, 0.0, rpy(2);
  fuse_core::Matrix3d expected_covariance;
  expected_covariance.col(0) << pose_covariance(0, 0), pose_covariance(1, 0), pose_covariance(5, 0);
  expected_covariance.col(1) << pose_covariance(0, 1), pose_covariance(1, 1), pose_covariance(5, 1);
  expected_covariance.col(2) << pose_covariance(0, 5), pose_covariance(1, 5), pose_covariance(5, 5);
  EXPECT_TRUE(expected_pose.isApprox(pose_mean_partial));
  EXPECT_TRUE(expected_covariance.isApprox(pose_covariance_partial));
}
