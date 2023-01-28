/***************************************************************************
 * Copyright (C) 2019 Clearpath Robotics. All rights reserved.
 * Unauthorized copying of this file, via any medium, is strictly prohibited
 * Proprietary and confidential
 ***************************************************************************/
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <vector>

#include <fuse_models/common/sensor_proc.hpp>

namespace fm_common = fuse_models::common;

TEST(TestSuite, mergeXYPositionAndYawOrientationIndices)
{
  const std::vector<size_t> position_indices{0, 1};
  const std::vector<size_t> orientation_indices{0};

  const size_t orientation_offset = 2;

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

TEST(TestSuite, mergeXPositionAndYawOrientationIndices)
{
  const std::vector<size_t> position_indices{0};
  const std::vector<size_t> orientation_indices{0};

  const size_t orientation_offset = 2;

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

TEST(TestSuite, mergeXYPositionAndEmptyOrientationIndices)
{
  const std::vector<size_t> position_indices{0, 1};
  const std::vector<size_t> orientation_indices;

  const size_t orientation_offset = 2;

  const auto merged_indices = fm_common::mergeIndices(
    position_indices, orientation_indices,
    orientation_offset);

  EXPECT_EQ(position_indices.size(), merged_indices.size());
  EXPECT_THAT(position_indices, testing::ElementsAreArray(merged_indices));
}

TEST(TestSuite, mergeEmptyPositionAndYawOrientationIndices)
{
  const std::vector<size_t> position_indices;
  const std::vector<size_t> orientation_indices{0};

  const size_t orientation_offset = 2;

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

  const size_t orientation_offset = 2;

  const auto merged_indices = fm_common::mergeIndices(
    position_indices, orientation_indices,
    orientation_offset);

  EXPECT_TRUE(merged_indices.empty());
}

TEST(TestSuite, populatePartialMeasurementXYPositionYawOrientation)
{
  fuse_core::VectorXd pose_mean(3);
  pose_mean << 1.0, 2.0, 3.0;

  fuse_core::MatrixXd pose_covariance(3, 3);
  pose_covariance << 0.1, 0.01, 0.001, 0.01, 0.2, 0.002, 0.001, 0.002, 0.3;

  const std::vector<size_t> position_indices{0, 1};
  const std::vector<size_t> orientation_indices{0};

  const size_t orientation_offset = 2;

  const auto merged_indices = fm_common::mergeIndices(
    position_indices, orientation_indices,
    orientation_offset);

  fuse_core::VectorXd pose_mean_partial(position_indices.size() + orientation_indices.size());
  fuse_core::MatrixXd pose_covariance_partial(pose_mean_partial.rows(), pose_mean_partial.rows());

  fm_common::populatePartialMeasurement(
    pose_mean,
    pose_covariance,
    merged_indices,
    pose_mean_partial,
    pose_covariance_partial);

  EXPECT_EQ(pose_mean, pose_mean_partial);
  EXPECT_EQ(pose_covariance, pose_covariance_partial);
}

TEST(TestSuite, populatePartialMeasurementXPositionYawOrientation)
{
  fuse_core::VectorXd pose_mean(3);
  pose_mean << 1.0, 2.0, 3.0;

  fuse_core::MatrixXd pose_covariance(3, 3);
  pose_covariance << 0.1, 0.01, 0.001, 0.01, 0.2, 0.002, 0.001, 0.002, 0.3;

  const std::vector<size_t> position_indices{0};
  const std::vector<size_t> orientation_indices{0};

  const size_t orientation_offset = 2;

  const auto merged_indices = fm_common::mergeIndices(
    position_indices, orientation_indices,
    orientation_offset);

  fuse_core::VectorXd pose_mean_partial(position_indices.size() + orientation_indices.size());
  fuse_core::MatrixXd pose_covariance_partial(pose_mean_partial.rows(), pose_mean_partial.rows());

  fm_common::populatePartialMeasurement(
    pose_mean,
    pose_covariance,
    merged_indices,
    pose_mean_partial,
    pose_covariance_partial);

  // Eigen indexing is only supported in the latest stable versions, so we avoid using that feature
  // for backwards compatibility
  const std::vector<int> expected_merged_indices{0, 2};

  EXPECT_THAT(expected_merged_indices, testing::ElementsAreArray(merged_indices));

  for (size_t row = 0; row < expected_merged_indices.size(); ++row) {
    EXPECT_EQ(pose_mean(expected_merged_indices[row]), pose_mean_partial(row));

    for (size_t column = 0; column < expected_merged_indices.size(); ++column) {
      EXPECT_EQ(
        pose_covariance(expected_merged_indices[row], expected_merged_indices[column]),
        pose_covariance_partial(row, column));
    }
  }
}

TEST(TestSuite, populatePartialMeasurementEmptyPositionYawOrientation)
{
  fuse_core::VectorXd pose_mean(3);
  pose_mean << 1.0, 2.0, 3.0;

  fuse_core::MatrixXd pose_covariance(3, 3);
  pose_covariance << 0.1, 0.01, 0.001, 0.01, 0.2, 0.002, 0.001, 0.002, 0.3;

  const std::vector<size_t> position_indices;
  const std::vector<size_t> orientation_indices{0};

  const size_t orientation_offset = 2;

  const auto merged_indices = fm_common::mergeIndices(
    position_indices, orientation_indices,
    orientation_offset);

  fuse_core::VectorXd pose_mean_partial(position_indices.size() + orientation_indices.size());
  fuse_core::MatrixXd pose_covariance_partial(pose_mean_partial.rows(), pose_mean_partial.rows());

  fm_common::populatePartialMeasurement(
    pose_mean,
    pose_covariance,
    merged_indices,
    pose_mean_partial,
    pose_covariance_partial);

  EXPECT_EQ(pose_mean.tail<1>(), pose_mean_partial);
  EXPECT_EQ(pose_covariance.bottomRightCorner(1, 1), pose_covariance_partial);
}

TEST(TestSuite, populatePartialMeasurementXYPositionEmptyOrientation)
{
  fuse_core::VectorXd pose_mean(3);
  pose_mean << 1.0, 2.0, 3.0;

  fuse_core::MatrixXd pose_covariance(3, 3);
  pose_covariance << 0.1, 0.01, 0.001, 0.01, 0.2, 0.002, 0.001, 0.002, 0.3;

  const std::vector<size_t> position_indices{0, 1};
  const std::vector<size_t> orientation_indices;

  const size_t orientation_offset = 2;

  const auto merged_indices = fm_common::mergeIndices(
    position_indices, orientation_indices,
    orientation_offset);

  fuse_core::VectorXd pose_mean_partial(position_indices.size() + orientation_indices.size());
  fuse_core::MatrixXd pose_covariance_partial(pose_mean_partial.rows(), pose_mean_partial.rows());

  fm_common::populatePartialMeasurement(
    pose_mean,
    pose_covariance,
    merged_indices,
    pose_mean_partial,
    pose_covariance_partial);

  EXPECT_EQ(pose_mean.head<2>(), pose_mean_partial);
  EXPECT_EQ(pose_covariance.topLeftCorner(2, 2), pose_covariance_partial);
}

TEST(TestSuite, populatePartialMeasurementEmptyPositionEmptyOrientation)
{
  fuse_core::VectorXd pose_mean(3);
  pose_mean << 1.0, 2.0, 3.0;

  fuse_core::MatrixXd pose_covariance(3, 3);
  pose_covariance << 0.1, 0.01, 0.001, 0.01, 0.2, 0.002, 0.001, 0.002, 0.3;

  const std::vector<size_t> position_indices;
  const std::vector<size_t> orientation_indices;

  const size_t orientation_offset = 2;

  const auto merged_indices = fm_common::mergeIndices(
    position_indices, orientation_indices,
    orientation_offset);

  fuse_core::VectorXd pose_mean_partial(position_indices.size() + orientation_indices.size());
  fuse_core::MatrixXd pose_covariance_partial(pose_mean_partial.rows(), pose_mean_partial.rows());

  fm_common::populatePartialMeasurement(
    pose_mean,
    pose_covariance,
    merged_indices,
    pose_mean_partial,
    pose_covariance_partial);

  EXPECT_EQ(0, pose_mean_partial.size());
  EXPECT_EQ(0, pose_covariance_partial.size());
}
