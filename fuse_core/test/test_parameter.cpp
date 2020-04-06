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
#include <fuse_core/parameter.h>
#include <ros/ros.h>

#include <gtest/gtest.h>

#include <numeric>
#include <string>

TEST(Parameter, GetPositiveParam)
{
  // Load parameters enforcing they are positive:
  const double default_value{ 1.0 };

  ros::NodeHandle node_handle;

  // Load a positive parameter:
  {
    double parameter{ default_value };
    fuse_core::getPositiveParam(node_handle, "positive_parameter", parameter);
    EXPECT_EQ(3.0, parameter);
  }

  // Load a negative parameter:
  {
    double parameter{ default_value };
    fuse_core::getPositiveParam(node_handle, "negative_parameter", parameter);
    EXPECT_EQ(default_value, parameter);
  }

  // Load a zero parameter:
  {
    double parameter{ default_value };
    fuse_core::getPositiveParam(node_handle, "zero_parameter", parameter);
    EXPECT_EQ(default_value, parameter);
  }

  // Load a zero parameter allowing zero (not strict):
  {
    double parameter{ default_value };
    fuse_core::getPositiveParam(node_handle, "zero_parameter", parameter, false);
    EXPECT_EQ(0.0, parameter);
  }
}

TEST(Parameter, GetCovarianceDiagonalParam)
{
  // Build expected covariance matrix:
  constexpr int Size = 3;
  constexpr double variance = 1.0e-3;
  constexpr double default_variance = 0.0;

  fuse_core::Matrix3d expected_covariance = fuse_core::Matrix3d::Identity();
  expected_covariance *= variance;

  fuse_core::Matrix3d default_covariance = fuse_core::Matrix3d::Identity();
  default_covariance *= default_variance;

  // Load covariance matrix diagonal from the parameter server:
  ros::NodeHandle node_handle;

  // A covariance diagonal with the expected size and valid should be the same as the expected one:
  {
    const std::string parameter_name{ "covariance_diagonal" };

    ASSERT_TRUE(node_handle.hasParam(parameter_name));

    try
    {
      const auto covariance =
          fuse_core::getCovarianceDiagonalParam<Size>(node_handle, parameter_name, default_variance);

      EXPECT_EQ(Size, covariance.rows());
      EXPECT_EQ(Size, covariance.cols());

      EXPECT_EQ(expected_covariance.rows() * expected_covariance.cols(),
                expected_covariance.cwiseEqual(covariance).count())
          << "Expected\n" << expected_covariance << "\nActual\n" << covariance;
    }
    catch (const std::exception& ex)
    {
      FAIL() << "Failed to get " << node_handle.resolveName(parameter_name) << ": " << ex.what();
    }
  }

  // If the parameter does not exist we should get the default covariance:
  {
    const std::string parameter_name{ "non_existent_parameter" };

    ASSERT_FALSE(node_handle.hasParam(parameter_name));

    try
    {
      const auto covariance =
          fuse_core::getCovarianceDiagonalParam<Size>(node_handle, parameter_name, default_variance);

      EXPECT_EQ(Size, covariance.rows());
      EXPECT_EQ(Size, covariance.cols());

      EXPECT_EQ(default_covariance.rows() * default_covariance.cols(),
                default_covariance.cwiseEqual(covariance).count())
          << "Expected\n" << default_covariance << "\nActual\n" << covariance;
    }
    catch (const std::exception& ex)
    {
      FAIL() << "Failed to get " << node_handle.resolveName(parameter_name) << ": " << ex.what();
    }
  }

  // A covariance diagonal with negative values should throw std::invalid_argument:
  {
    const std::string parameter_name{ "covariance_diagonal_with_negative_values" };

    ASSERT_TRUE(node_handle.hasParam(parameter_name));

    EXPECT_THROW(fuse_core::getCovarianceDiagonalParam<Size>(node_handle, parameter_name, default_variance),
                 std::invalid_argument);
  }

  // A covariance diagonal with size 2, smaller than expected, should throw std::invalid_argument:
  {
    const std::string parameter_name{ "covariance_diagonal_with_size_2" };

    ASSERT_TRUE(node_handle.hasParam(parameter_name));

    EXPECT_THROW(fuse_core::getCovarianceDiagonalParam<Size>(node_handle, parameter_name, default_variance),
                 std::invalid_argument);
  }

  // A covariance diagonal with size 4, larger than expected, should throw std::invalid_argument:
  {
    const std::string parameter_name{ "covariance_diagonal_with_size_4" };

    ASSERT_TRUE(node_handle.hasParam(parameter_name));

    EXPECT_THROW(fuse_core::getCovarianceDiagonalParam<Size>(node_handle, parameter_name, default_variance),
                 std::invalid_argument);
  }

  // A covariance diagonal with invalid element type does not throw, but nothing it is loaded, so we should get the
  // default covariance:
  {
    const std::string parameter_name{ "covariance_diagonal_with_strings" };

    ASSERT_TRUE(node_handle.hasParam(parameter_name));

    try
    {
      const auto covariance =
          fuse_core::getCovarianceDiagonalParam<Size>(node_handle, parameter_name, default_variance);

      EXPECT_EQ(Size, covariance.rows());
      EXPECT_EQ(Size, covariance.cols());

      EXPECT_EQ(default_covariance.rows() * default_covariance.cols(),
                default_covariance.cwiseEqual(covariance).count())
          << "Expected\n" << default_covariance << "\nActual\n" << covariance;
    }
    catch (const std::exception& ex)
    {
      FAIL() << "Failed to get " << node_handle.resolveName(parameter_name) << ": " << ex.what();
    }
  }

  // A covariance diagonal with invalid element type does not throw, but nothing it is loaded, so we should get the
  // default covariance:
  {
    const std::string parameter_name{ "covariance_diagonal_with_string" };

    ASSERT_TRUE(node_handle.hasParam(parameter_name));

    try
    {
      const auto covariance =
          fuse_core::getCovarianceDiagonalParam<Size>(node_handle, parameter_name, default_variance);

      EXPECT_EQ(Size, covariance.rows());
      EXPECT_EQ(Size, covariance.cols());

      EXPECT_EQ(default_covariance.rows() * default_covariance.cols(),
                default_covariance.cwiseEqual(covariance).count())
          << "Expected\n" << default_covariance << "\nActual\n" << covariance;
    }
    catch (const std::exception& ex)
    {
      FAIL() << "Failed to get " << node_handle.resolveName(parameter_name) << ": " << ex.what();
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "parameter_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
