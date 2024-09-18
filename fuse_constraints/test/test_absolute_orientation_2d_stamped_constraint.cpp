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
#include <ceres/covariance.h>
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <utility>
#include <vector>

#include <fuse_constraints/absolute_constraint.hpp>
#include <fuse_core/ceres_macros.hpp>
#include <fuse_core/eigen.hpp>
#include <fuse_core/eigen_gtest.hpp>
#include <fuse_core/serialization.hpp>
#include <fuse_core/uuid.hpp>
#include <fuse_variables/orientation_2d_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

using fuse_constraints::AbsoluteOrientation2DStampedConstraint;
using fuse_variables::Orientation2DStamped;


TEST(AbsoluteOrientation2DStampedConstraint, Constructor)
{
  // Construct a constraint just to make sure it compiles.
  Orientation2DStamped orientation_variable(rclcpp::Time(1234, 5678),
    fuse_core::uuid::generate("walle"));
  fuse_core::VectorXd mean(1);
  mean << 1.0;
  fuse_core::MatrixXd cov(1, 1);
  cov << 1.0;
  EXPECT_NO_THROW(
    AbsoluteOrientation2DStampedConstraint constraint(
      "test", orientation_variable,
      mean, cov));
}

TEST(AbsoluteOrientation2DStampedConstraint, Covariance)
{
  // Verify the covariance <--> sqrt information conversions are correct
  Orientation2DStamped orientation_variable(rclcpp::Time(1234, 5678), fuse_core::uuid::generate(
      "mo"));
  fuse_core::VectorXd mean(1);
  mean << 1.0;
  fuse_core::MatrixXd cov(1, 1);
  cov << 1.0;
  AbsoluteOrientation2DStampedConstraint constraint("test", orientation_variable, mean, cov);

  // Define the expected matrices (used Octave to compute sqrt_info: 'chol(inv(A))')
  fuse_core::Matrix1d expected_sqrt_info;
  expected_sqrt_info << 1.0;
  fuse_core::Matrix1d expected_cov = cov;

  // Compare
  EXPECT_MATRIX_NEAR(expected_cov, constraint.covariance(), 1.0e-9);
  EXPECT_MATRIX_NEAR(expected_sqrt_info, constraint.sqrtInformation(), 1.0e-9);
}

TEST(AbsoluteOrientation2DStampedConstraint, Optimization)
{
  // Optimize a single pose and single constraint, verify the expected value and covariance are
  // generated.
  // Create the variables
  auto orientation_variable = Orientation2DStamped::make_shared(
    rclcpp::Time(1, 0), fuse_core::uuid::generate("spra"));
  orientation_variable->yaw() = 1.0;

  // Create an absolute orientation constraint
  fuse_core::VectorXd mean(1);
  mean << 1.0;
  fuse_core::MatrixXd cov(1, 1);
  cov << 1.0;
  auto constraint = AbsoluteOrientation2DStampedConstraint::make_shared(
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
#if !CERES_SUPPORTS_MANIFOLDS
    orientation_variable->localParameterization());
#else
    orientation_variable->manifold());
#endif

  std::vector<double *> parameter_blocks;
  parameter_blocks.push_back(orientation_variable->data());
  problem.AddResidualBlock(
    constraint->costFunction(),
    constraint->lossFunction(),
    parameter_blocks);

  // Run the solver
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  EXPECT_TRUE(summary.IsSolutionUsable()) << summary.FullReport();

  // Check
  EXPECT_NEAR(1.0, orientation_variable->yaw(), 1.0e-3);

  // Compute the covariance
  std::vector<std::pair<const double *, const double *>> covariance_blocks;
  covariance_blocks.emplace_back(orientation_variable->data(), orientation_variable->data());
  ceres::Covariance::Options cov_options;
  ceres::Covariance covariance(cov_options);
  covariance.Compute(covariance_blocks, &problem);
  fuse_core::Matrix1d actual_covariance(orientation_variable->localSize(),
    orientation_variable->localSize());
  covariance.GetCovarianceBlockInTangentSpace(
    orientation_variable->data(), orientation_variable->data(), actual_covariance.data());

  // Define the expected covariance
  fuse_core::Matrix1d expected_covariance;
  expected_covariance << 1.0;
  EXPECT_MATRIX_NEAR(expected_covariance, actual_covariance, 1.0e-3);
}

TEST(AbsoluteOrientation2DStampedConstraint, OptimizationZero)
{
  // Optimize a single orientation at zero and single constraint, verify the expected value and
  // covariance are generated.

  // Create the variables
  auto orientation_variable = Orientation2DStamped::make_shared(
    rclcpp::Time(1, 0),
    fuse_core::uuid::generate("spra"));
  orientation_variable->yaw() = 0.0;

  // Create an absolute orientation constraint
  fuse_core::VectorXd mean(1);
  mean << 0.0;
  fuse_core::MatrixXd cov(1, 1);
  cov << 1.0;
  auto constraint = AbsoluteOrientation2DStampedConstraint::make_shared(
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
#if !CERES_SUPPORTS_MANIFOLDS
    orientation_variable->localParameterization());
#else
    orientation_variable->manifold());
#endif

  std::vector<double *> parameter_blocks;
  parameter_blocks.push_back(orientation_variable->data());
  problem.AddResidualBlock(
    constraint->costFunction(),
    constraint->lossFunction(),
    parameter_blocks);

  // Run the solver
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  EXPECT_TRUE(summary.IsSolutionUsable()) << summary.FullReport();

  // Check
  EXPECT_NEAR(0.0, orientation_variable->yaw(), 1.0e-3);

  // Compute the covariance
  std::vector<std::pair<const double *, const double *>> covariance_blocks;
  covariance_blocks.emplace_back(orientation_variable->data(), orientation_variable->data());
  ceres::Covariance::Options cov_options;
  ceres::Covariance covariance(cov_options);
  covariance.Compute(covariance_blocks, &problem);
  fuse_core::Matrix1d actual_covariance(orientation_variable->localSize(),
    orientation_variable->localSize());
  covariance.GetCovarianceBlockInTangentSpace(
    orientation_variable->data(), orientation_variable->data(), actual_covariance.data());

  // Define the expected covariance
  fuse_core::Matrix1d expected_covariance;
  expected_covariance << 1.0;
  EXPECT_MATRIX_NEAR(expected_covariance, actual_covariance, 1.0e-3);
}

// Temporarily disable this unit test. This should be fixed by PR #335.
// TEST(AbsoluteOrientation2DStampedConstraint, OptimizationPositivePi)
// {
//   // Optimize a single orientation at +PI and single constraint, verify the expected value and
//   // covariance are generated.
//
//   // Create the variables
//   auto orientation_variable = Orientation2DStamped::make_shared(
//     rclcpp::Time(1, 0),
//     fuse_core::uuid::generate("spra"));
//   orientation_variable->yaw() = M_PI;
//
//   // Create an absolute orientation constraint
//   fuse_core::Vector1d mean;
//   mean << M_PI;
//
//   fuse_core::Matrix1d cov;
//   cov << 1.0;
//   auto constraint = AbsoluteOrientation2DStampedConstraint::make_shared(
//     "test",
//     *orientation_variable,
//     mean,
//     cov);
//
//   // Build the problem
//   ceres::Problem::Options problem_options;
//   problem_options.loss_function_ownership = fuse_core::Loss::Ownership;
//   ceres::Problem problem(problem_options);
//   problem.AddParameterBlock(
//     orientation_variable->data(),
//     orientation_variable->size(),
// #if !CERES_SUPPORTS_MANIFOLDS
//     orientation_variable->localParameterization());
// #else
//     orientation_variable->manifold());
// #endif
//
//   std::vector<double *> parameter_blocks;
//   parameter_blocks.push_back(orientation_variable->data());
//   problem.AddResidualBlock(
//     constraint->costFunction(),
//     constraint->lossFunction(),
//     parameter_blocks);
//
//   // Run the solver
//   ceres::Solver::Options options;
//   ceres::Solver::Summary summary;
//   ceres::Solve(options, &problem, &summary);
//   EXPECT_TRUE(summary.IsSolutionUsable()) << summary.FullReport();
//
//   // Check
//   // We expect +PI to roll over to -PI because our range is [-PI, PI)
//   EXPECT_NEAR(-M_PI, orientation_variable->yaw(), 1.0e-3);
//
//   // Compute the covariance
//   std::vector<std::pair<const double *, const double *>> covariance_blocks;
//   covariance_blocks.emplace_back(orientation_variable->data(), orientation_variable->data());
//   ceres::Covariance::Options cov_options;
//   ceres::Covariance covariance(cov_options);
//   covariance.Compute(covariance_blocks, &problem);
//   fuse_core::Matrix1d actual_covariance(orientation_variable->localSize(),
//     orientation_variable->localSize());
//   covariance.GetCovarianceBlockInTangentSpace(
//     orientation_variable->data(), orientation_variable->data(), actual_covariance.data());
//
//   // Define the expected covariance
//   fuse_core::Matrix1d expected_covariance;
//   expected_covariance << 1.0;
//   EXPECT_MATRIX_NEAR(expected_covariance, actual_covariance, 1.0e-3);
// }

TEST(AbsoluteOrientation2DStampedConstraint, OptimizationNegativePi)
{
  // Optimize a single orientation at -PI and single constraint, verify the expected value and
  // covariance are generated.

  // Create the variables
  auto orientation_variable = Orientation2DStamped::make_shared(
    rclcpp::Time(1, 0),
    fuse_core::uuid::generate("spra"));
  orientation_variable->yaw() = -M_PI;

  // Create an absolute orientation constraint
  fuse_core::VectorXd mean(1);
  mean << -M_PI;
  fuse_core::MatrixXd cov(1, 1);
  cov << 1.0;
  auto constraint = AbsoluteOrientation2DStampedConstraint::make_shared(
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
#if !CERES_SUPPORTS_MANIFOLDS
    orientation_variable->localParameterization());
#else
    orientation_variable->manifold());
#endif

  std::vector<double *> parameter_blocks;
  parameter_blocks.push_back(orientation_variable->data());
  problem.AddResidualBlock(
    constraint->costFunction(),
    constraint->lossFunction(),
    parameter_blocks);

  // Run the solver
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  EXPECT_TRUE(summary.IsSolutionUsable()) << summary.FullReport();

  // Check
  EXPECT_NEAR(-M_PI, orientation_variable->yaw(), 1.0e-3);

  // Compute the covariance
  std::vector<std::pair<const double *, const double *>> covariance_blocks;
  covariance_blocks.emplace_back(orientation_variable->data(), orientation_variable->data());
  ceres::Covariance::Options cov_options;
  ceres::Covariance covariance(cov_options);
  covariance.Compute(covariance_blocks, &problem);
  fuse_core::Matrix1d actual_covariance(orientation_variable->localSize(),
    orientation_variable->localSize());
  covariance.GetCovarianceBlockInTangentSpace(
    orientation_variable->data(), orientation_variable->data(), actual_covariance.data());

  // Define the expected covariance
  fuse_core::Matrix1d expected_covariance;
  expected_covariance << 1.0;
  EXPECT_MATRIX_NEAR(expected_covariance, actual_covariance, 1.0e-3);
}

TEST(AbsoluteOrientation2DStampedConstraint, Serialization)
{
  // Construct a constraint
  Orientation2DStamped orientation_variable(rclcpp::Time(1234, 5678),
    fuse_core::uuid::generate("walle"));
  fuse_core::VectorXd mean(1);
  mean << 1.0;
  fuse_core::MatrixXd cov(1, 1);
  cov << 1.0;
  AbsoluteOrientation2DStampedConstraint expected("test", orientation_variable, mean, cov);

  // Serialize the constraint into an archive
  std::stringstream stream;
  {
    fuse_core::TextOutputArchive archive(stream);
    expected.serialize(archive);
  }

  // Deserialize a new constraint from that same stream
  AbsoluteOrientation2DStampedConstraint actual;
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
