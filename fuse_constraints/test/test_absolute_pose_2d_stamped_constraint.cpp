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
#include <fuse_constraints/absolute_pose_2d_stamped_constraint.h>
#include <fuse_core/eigen.h>
#include <fuse_core/eigen_gtest.h>
#include <fuse_core/serialization.h>
#include <fuse_core/uuid.h>
#include <fuse_variables/orientation_2d_stamped.h>
#include <fuse_variables/position_2d_stamped.h>

#include <ceres/covariance.h>
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <gtest/gtest.h>

#include <utility>
#include <vector>

using fuse_variables::Orientation2DStamped;
using fuse_variables::Position2DStamped;
using fuse_constraints::AbsolutePose2DStampedConstraint;


TEST(AbsolutePose2DStampedConstraint, Constructor)
{
  // Construct a constraint just to make sure it compiles.
  Orientation2DStamped orientation_variable(rclcpp::Time(1234, 5678), fuse_core::uuid::generate("walle"));
  Position2DStamped position_variable(rclcpp::Time(1234, 5678), fuse_core::uuid::generate("walle"));
  fuse_core::Vector3d mean;
  mean << 1.0, 2.0, 3.0;
  fuse_core::Matrix3d cov;
  cov << 1.0, 0.1, 0.2, 0.1, 2.0, 0.3, 0.2, 0.3, 3.0;
  EXPECT_NO_THROW(
    AbsolutePose2DStampedConstraint constraint("test", position_variable, orientation_variable, mean, cov));
}

TEST(AbsolutePose2DStampedConstraint, Covariance)
{
  // Verify the covariance <--> sqrt information conversions are correct
  Orientation2DStamped orientation_variable(rclcpp::Time(1234, 5678), fuse_core::uuid::generate("mo"));
  Position2DStamped position_variable(rclcpp::Time(1234, 5678), fuse_core::uuid::generate("mo"));
  fuse_core::Vector3d mean;
  mean << 1.0, 2.0, 3.0;
  fuse_core::Matrix3d cov;
  cov << 1.0, 0.1, 0.2, 0.1, 2.0, 0.3, 0.2, 0.3, 3.0;
  AbsolutePose2DStampedConstraint constraint("test", position_variable, orientation_variable, mean, cov);
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

TEST(AbsolutePose2DStampedConstraint, OptimizationFull)
{
  // Optimize a single pose and single constraint, verify the expected value and covariance are generated.
  // Create the variables
  auto orientation_variable = Orientation2DStamped::make_shared(rclcpp::Time(1, 0), fuse_core::uuid::generate("spra"));
  orientation_variable->yaw() = 0.8;
  auto position_variable = Position2DStamped::make_shared(rclcpp::Time(1, 0), fuse_core::uuid::generate("spra"));
  position_variable->x() = 1.5;
  position_variable->y() = -3.0;
  // Create an absolute pose constraint
  fuse_core::Vector3d mean;
  mean << 1.0, 2.0, 3.0;
  fuse_core::Matrix3d cov;
  cov << 1.0, 0.1, 0.2, 0.1, 2.0, 0.3, 0.2, 0.3, 3.0;
  auto constraint = AbsolutePose2DStampedConstraint::make_shared(
    "test",
    *position_variable,
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
  problem.AddParameterBlock(
    position_variable->data(),
    position_variable->size(),
    position_variable->localParameterization());
  std::vector<double*> parameter_blocks;
  parameter_blocks.push_back(position_variable->data());
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
  EXPECT_NEAR(1.0, position_variable->x(), 1.0e-5);
  EXPECT_NEAR(2.0, position_variable->y(), 1.0e-5);
  EXPECT_NEAR(3.0, orientation_variable->yaw(), 1.0e-5);
  // Compute the covariance
  std::vector<std::pair<const double*, const double*> > covariance_blocks;
  covariance_blocks.emplace_back(position_variable->data(), position_variable->data());
  covariance_blocks.emplace_back(position_variable->data(), orientation_variable->data());
  covariance_blocks.emplace_back(orientation_variable->data(), orientation_variable->data());
  ceres::Covariance::Options cov_options;
  ceres::Covariance covariance(cov_options);
  covariance.Compute(covariance_blocks, &problem);
  std::vector<double> covariance_vector1(position_variable->size() * position_variable->size());
  covariance.GetCovarianceBlock(position_variable->data(), position_variable->data(), covariance_vector1.data());
  std::vector<double> covariance_vector2(position_variable->size() * orientation_variable->size());
  covariance.GetCovarianceBlock(position_variable->data(), orientation_variable->data(), covariance_vector2.data());
  std::vector<double> covariance_vector3(orientation_variable->size() * orientation_variable->size());
  covariance.GetCovarianceBlock(orientation_variable->data(), orientation_variable->data(), covariance_vector3.data());
  // Assemble the full covariance from the covariance blocks
  fuse_core::Matrix3d actual_covariance;
  actual_covariance(0, 0) = covariance_vector1[0];
  actual_covariance(0, 1) = covariance_vector1[1];
  actual_covariance(1, 0) = covariance_vector1[2];
  actual_covariance(1, 1) = covariance_vector1[3];
  actual_covariance(0, 2) = covariance_vector2[0];
  actual_covariance(1, 2) = covariance_vector2[1];
  actual_covariance(2, 0) = covariance_vector2[0];
  actual_covariance(2, 1) = covariance_vector2[1];
  actual_covariance(2, 2) = covariance_vector3[0];
  fuse_core::Matrix3d expected_covariance = cov;
  EXPECT_MATRIX_NEAR(expected_covariance, actual_covariance, 1.0e-9);
}

TEST(AbsolutePose2DStampedConstraint, OptimizationPartial)
{
  // Optimize a single pose and single constraint, verify the expected value and covariance are generated.
  // Create the variables
  auto orientation_variable = Orientation2DStamped::make_shared(rclcpp::Time(1, 0), fuse_core::uuid::generate("spra"));
  orientation_variable->yaw() = 0.8;
  auto position_variable = Position2DStamped::make_shared(rclcpp::Time(1, 0), fuse_core::uuid::generate("spra"));
  position_variable->x() = 1.5;
  position_variable->y() = -3.0;

  // Create an absolute pose constraint
  fuse_core::Vector2d mean1;
  mean1 << 1.0, 3.0;
  fuse_core::Matrix2d cov1;
  cov1 << 1.0, 0.2, 0.2, 3.0;
  std::vector<size_t> axes_lin1 = {fuse_variables::Position2DStamped::X};
  std::vector<size_t> axes_ang1 = {fuse_variables::Orientation2DStamped::YAW};
  auto constraint1 = AbsolutePose2DStampedConstraint::make_shared(
    "test",
    *position_variable,
    *orientation_variable,
    mean1,
    cov1,
    axes_lin1,
    axes_ang1);

  // Create an absolute pose constraint
  fuse_core::Vector1d mean2;
  mean2 << 2.0;
  fuse_core::Matrix1d cov2;
  cov2 << 2.0;
  std::vector<size_t> axes_lin2 = {fuse_variables::Position2DStamped::Y};
  std::vector<size_t> axes_ang2;
  auto constraint2 = AbsolutePose2DStampedConstraint::make_shared(
    "test",
    *position_variable,
    *orientation_variable,
    mean2,
    cov2,
    axes_lin2,
    axes_ang2);

  // Build the problem
  ceres::Problem::Options problem_options;
  problem_options.loss_function_ownership = fuse_core::Loss::Ownership;
  ceres::Problem problem(problem_options);
  problem.AddParameterBlock(
    position_variable->data(),
    position_variable->size(),
    position_variable->localParameterization());
  problem.AddParameterBlock(
    orientation_variable->data(),
    orientation_variable->size(),
    orientation_variable->localParameterization());

  std::vector<double*> parameter_blocks;
  parameter_blocks.push_back(position_variable->data());
  parameter_blocks.push_back(orientation_variable->data());
  problem.AddResidualBlock(
    constraint1->costFunction(),
    constraint1->lossFunction(),
    parameter_blocks);
  problem.AddResidualBlock(
    constraint2->costFunction(),
    constraint2->lossFunction(),
    parameter_blocks);

  // Run the solver
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // Check
  EXPECT_NEAR(1.0, position_variable->x(), 1.0e-5);
  EXPECT_NEAR(2.0, position_variable->y(), 1.0e-5);
  EXPECT_NEAR(3.0, orientation_variable->yaw(), 1.0e-5);

  // Compute the covariance
  std::vector<std::pair<const double*, const double*> > covariance_blocks;
  covariance_blocks.emplace_back(position_variable->data(), position_variable->data());
  covariance_blocks.emplace_back(position_variable->data(), orientation_variable->data());
  covariance_blocks.emplace_back(orientation_variable->data(), orientation_variable->data());
  ceres::Covariance::Options cov_options;
  ceres::Covariance covariance(cov_options);
  covariance.Compute(covariance_blocks, &problem);
  std::vector<double> covariance_vector1(position_variable->size() * position_variable->size());
  covariance.GetCovarianceBlock(position_variable->data(), position_variable->data(), covariance_vector1.data());
  std::vector<double> covariance_vector2(position_variable->size() * orientation_variable->size());
  covariance.GetCovarianceBlock(position_variable->data(), orientation_variable->data(), covariance_vector2.data());
  std::vector<double> covariance_vector3(orientation_variable->size() * orientation_variable->size());
  covariance.GetCovarianceBlock(orientation_variable->data(), orientation_variable->data(), covariance_vector3.data());

  // Assemble the full covariance from the covariance blocks
  fuse_core::Matrix3d actual_covariance;
  actual_covariance(0, 0) = covariance_vector1[0];
  actual_covariance(0, 1) = covariance_vector1[1];
  actual_covariance(1, 0) = covariance_vector1[2];
  actual_covariance(1, 1) = covariance_vector1[3];
  actual_covariance(0, 2) = covariance_vector2[0];
  actual_covariance(1, 2) = covariance_vector2[1];
  actual_covariance(2, 0) = covariance_vector2[0];
  actual_covariance(2, 1) = covariance_vector2[1];
  actual_covariance(2, 2) = covariance_vector3[0];

  // Expected covariance should be the individual covariance matrices composed into a 3x3
  fuse_core::Matrix3d expected_covariance;
  expected_covariance.setZero();
  expected_covariance(0, 0) = cov1(0, 0);
  expected_covariance(0, 2) = cov1(0, 1);
  expected_covariance(2, 0) = cov1(1, 0);
  expected_covariance(2, 2) = cov1(1, 1);
  expected_covariance(1, 1) = cov2(0, 0);

  EXPECT_MATRIX_NEAR(expected_covariance, actual_covariance, 1.0e-9);
}

TEST(AbsolutePose2DStampedConstraint, Serialization)
{
  // Construct a constraint
  Orientation2DStamped orientation_variable(rclcpp::Time(1234, 5678), fuse_core::uuid::generate("walle"));
  Position2DStamped position_variable(rclcpp::Time(1234, 5678), fuse_core::uuid::generate("walle"));
  fuse_core::Vector3d mean;
  mean << 1.0, 2.0, 3.0;
  fuse_core::Matrix3d cov;
  cov << 1.0, 0.1, 0.2, 0.1, 2.0, 0.3, 0.2, 0.3, 3.0;
  AbsolutePose2DStampedConstraint expected("test", position_variable, orientation_variable, mean, cov);

  // Serialize the constraint into an archive
  std::stringstream stream;
  {
    fuse_core::TextOutputArchive archive(stream);
    expected.serialize(archive);
  }

  // Deserialize a new constraint from that same stream
  AbsolutePose2DStampedConstraint actual;
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
