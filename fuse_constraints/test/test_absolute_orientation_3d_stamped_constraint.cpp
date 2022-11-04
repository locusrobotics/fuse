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
#include <fuse_constraints/absolute_orientation_3d_stamped_constraint.h>
#include <fuse_core/eigen.h>
#include <fuse_core/eigen_gtest.h>
#include <fuse_core/serialization.h>
#include <fuse_core/uuid.h>
#include <fuse_variables/orientation_3d_stamped.h>
#include <geometry_msgs/Quaternion.h>

#include <ceres/covariance.h>
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <utility>
#include <vector>

using fuse_constraints::AbsoluteOrientation3DStampedConstraint;
using fuse_variables::Orientation3DStamped;


TEST(AbsoluteOrientation3DStampedConstraint, Constructor)
{
  // Construct a constraint just to make sure it compiles.
  Orientation3DStamped orientation_variable(rclcpp::Time(1234, 5678), fuse_core::uuid::generate("walle"));
  fuse_core::Vector4d mean;
  mean << 1.0, 0.0, 0.0, 0.0;
  fuse_core::Matrix3d cov;
  cov << 1.0, 0.1, 0.2, 0.1, 2.0, 0.3, 0.2, 0.3, 3.0;
  EXPECT_NO_THROW(AbsoluteOrientation3DStampedConstraint constraint("test", orientation_variable, mean, cov));

  Eigen::Quaterniond quat_eigen(1.0, 0.0, 0.0, 0.0);
  EXPECT_NO_THROW(AbsoluteOrientation3DStampedConstraint constraint("test", orientation_variable, quat_eigen, cov));

  geometry_msgs::Quaternion quat_geom;
  quat_geom.w = 1.0;
  std::array<double, 9> cov_arr = {1.0, 0.1, 0.2, 0.1, 2.0, 0.3, 0.2, 0.3, 3.0};
  EXPECT_NO_THROW(AbsoluteOrientation3DStampedConstraint constraint("test", orientation_variable, quat_geom, cov_arr));
}

TEST(AbsoluteOrientation3DStampedConstraint, Covariance)
{
  // Verify the covariance <--> sqrt information conversions are correct
  Orientation3DStamped orientation_variable(rclcpp::Time(1234, 5678), fuse_core::uuid::generate("mo"));
  fuse_core::Vector4d mean;
  mean << 1.0, 0.0, 0.0, 0.0;
  fuse_core::Matrix3d cov;
  cov << 1.0, 0.1, 0.2, 0.1, 2.0, 0.3, 0.2, 0.3, 3.0;
  AbsoluteOrientation3DStampedConstraint constraint("test", orientation_variable, mean, cov);

  // Define the expected matrices (used Octave to compute sqrt_info: 'chol(inv(A))')
  fuse_core::Matrix3d expected_sqrt_info;
  expected_sqrt_info <<  1.008395589795798, -0.040950074712520, -0.063131365181801,
                         0.000000000000000,  0.712470499879096, -0.071247049987910,
                         0.000000000000000,  0.000000000000000,  0.577350269189626;
  fuse_core::Matrix3d expected_cov = cov;

  // Compare
  EXPECT_MATRIX_NEAR(expected_cov, constraint.covariance(), 1.0e-9);
  EXPECT_MATRIX_NEAR(expected_sqrt_info, constraint.sqrtInformation(), 1.0e-9);
}

TEST(AbsoluteOrientation3DStampedConstraint, Optimization)
{
  // Optimize a single pose and single constraint, verify the expected value and covariance are generated.
  // Create the variables
  auto orientation_variable = Orientation3DStamped::make_shared(rclcpp::Time(1, 0), fuse_core::uuid::generate("spra"));
  orientation_variable->w() = 0.952;
  orientation_variable->x() = 0.038;
  orientation_variable->y() = -0.189;
  orientation_variable->z() = 0.239;

  // Create an absolute orientation constraint
  fuse_core::Vector4d mean;
  mean << 1.0, 0.0, 0.0, 0.0;

  fuse_core::Matrix3d cov;
  cov <<
    1.0, 0.1, 0.2,
    0.1, 2.0, 0.3,
    0.2, 0.3, 3.0;
  auto constraint = AbsoluteOrientation3DStampedConstraint::make_shared(
    "test",
    *orientation_variable,
    mean,
    cov);

  // Build the problem
  ceres::Problem::Options problem_options;
  problem_options.loss_function_ownership = fuse_core::Loss::Ownership;
  ceres::Problem problem(problem_options);
  problem.AddParameterBlock(
    orientation_variable->data(),
    orientation_variable->size(),
    orientation_variable->localParameterization());

  std::vector<double*> parameter_blocks;
  parameter_blocks.push_back(orientation_variable->data());
  problem.AddResidualBlock(
    constraint->costFunction(),
    constraint->lossFunction(),
    parameter_blocks);

  // Run the solver
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // Check
  EXPECT_NEAR(1.0, orientation_variable->w(), 1.0e-3);
  EXPECT_NEAR(0.0, orientation_variable->x(), 1.0e-3);
  EXPECT_NEAR(0.0, orientation_variable->y(), 1.0e-3);
  EXPECT_NEAR(0.0, orientation_variable->z(), 1.0e-3);

  // Compute the covariance
  std::vector<std::pair<const double*, const double*> > covariance_blocks;
  covariance_blocks.emplace_back(orientation_variable->data(), orientation_variable->data());
  ceres::Covariance::Options cov_options;
  ceres::Covariance covariance(cov_options);
  covariance.Compute(covariance_blocks, &problem);
  fuse_core::Matrix3d actual_covariance(orientation_variable->localSize(), orientation_variable->localSize());
  covariance.GetCovarianceBlockInTangentSpace(
    orientation_variable->data(), orientation_variable->data(), actual_covariance.data());

  // Define the expected covariance
  fuse_core::Matrix3d expected_covariance;
  expected_covariance <<
    1.0, 0.1, 0.2,
    0.1, 2.0, 0.3,
    0.2, 0.3, 3.0;
  EXPECT_MATRIX_NEAR(expected_covariance, actual_covariance, 1.0e-3);
}

TEST(AbsoluteOrientation3DStampedConstraint, Serialization)
{
  // Construct a constraint
  Orientation3DStamped orientation_variable(rclcpp::Time(1234, 5678), fuse_core::uuid::generate("walle"));
  fuse_core::Vector4d mean;
  mean << 1.0, 0.0, 0.0, 0.0;
  fuse_core::Matrix3d cov;
  cov << 1.0, 0.1, 0.2, 0.1, 2.0, 0.3, 0.2, 0.3, 3.0;
  AbsoluteOrientation3DStampedConstraint expected("test", orientation_variable, mean, cov);

  // Serialize the constraint into an archive
  std::stringstream stream;
  {
    fuse_core::TextOutputArchive archive(stream);
    expected.serialize(archive);
  }

  // Deserialize a new constraint from that same stream
  AbsoluteOrientation3DStampedConstraint actual;
  {
    fuse_core::TextInputArchive archive(stream);
    actual.deserialize(archive);
  }

  // Compare
  EXPECT_EQ(expected.uuid(), actual.uuid());
  EXPECT_EQ(expected.variables(), actual.variables());
  EXPECT_MATRIX_EQ(expected.mean(), actual.mean());
  EXPECT_MATRIX_EQ(expected.sqrtInformation(), actual.sqrtInformation());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
