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
#include <gtest/gtest.h>

#include <utility>
#include <vector>

#include <fuse_constraints/absolute_constraint.hpp>
#include <fuse_constraints/relative_constraint.hpp>
#include <fuse_core/ceres_macros.hpp>
#include <fuse_core/eigen.hpp>
#include <fuse_core/eigen_gtest.hpp>
#include <fuse_core/serialization.hpp>
#include <fuse_core/uuid.hpp>
#include <fuse_variables/acceleration_angular_2d_stamped.hpp>
#include <fuse_variables/acceleration_linear_2d_stamped.hpp>
#include <fuse_variables/orientation_2d_stamped.hpp>
#include <fuse_variables/position_2d_stamped.hpp>
#include <fuse_variables/position_3d_stamped.hpp>
#include <fuse_variables/velocity_angular_2d_stamped.hpp>
#include <fuse_variables/velocity_linear_2d_stamped.hpp>

TEST(RelativeConstraint, Constructor)
{
  // Construct a constraint for every type, just to make sure they compile.
  {
    fuse_variables::AccelerationAngular2DStamped x1(rclcpp::Time(1234, 5678),
      fuse_core::uuid::generate("robby"));
    fuse_variables::AccelerationAngular2DStamped x2(rclcpp::Time(1235, 5678),
      fuse_core::uuid::generate("robby"));
    fuse_core::VectorXd delta(1);
    delta << 3.0;
    fuse_core::MatrixXd cov(1, 1);
    cov << 1.0;
    EXPECT_NO_THROW(
      fuse_constraints::RelativeAccelerationAngular2DStampedConstraint constraint(
        "test", x1, x2,
        delta, cov));
  }
  {
    fuse_variables::AccelerationLinear2DStamped x1(rclcpp::Time(1234, 5678),
      fuse_core::uuid::generate("bender"));
    fuse_variables::AccelerationLinear2DStamped x2(rclcpp::Time(1235, 5678),
      fuse_core::uuid::generate("bender"));
    fuse_core::VectorXd delta(2);
    delta << 1.0, 2.0;
    fuse_core::MatrixXd cov(2, 2);
    cov << 1.0, 0.1, 0.1, 2.0;
    EXPECT_NO_THROW(
      fuse_constraints::RelativeAccelerationLinear2DStampedConstraint constraint(
        "test", x1, x2,
        delta, cov));
  }
  {
    fuse_variables::Orientation2DStamped x1(rclcpp::Time(1234, 5678),
      fuse_core::uuid::generate("johnny5"));
    fuse_variables::Orientation2DStamped x2(rclcpp::Time(1235, 5678),
      fuse_core::uuid::generate("johnny5"));
    fuse_core::VectorXd delta(1);
    delta << 3.0;
    fuse_core::MatrixXd cov(1, 1);
    cov << 1.0;
    EXPECT_NO_THROW(
      fuse_constraints::RelativeOrientation2DStampedConstraint constraint(
        "test", x1, x2, delta,
        cov));
  }
  {
    fuse_variables::Position2DStamped x1(rclcpp::Time(1234, 5678), fuse_core::uuid::generate(
        "rosie"));
    fuse_variables::Position2DStamped x2(rclcpp::Time(1235, 5678), fuse_core::uuid::generate(
        "rosie"));
    fuse_core::VectorXd delta(2);
    delta << 1.0, 2.0;
    fuse_core::MatrixXd cov(2, 2);
    cov << 1.0, 0.1, 0.1, 2.0;
    EXPECT_NO_THROW(
      fuse_constraints::RelativePosition2DStampedConstraint constraint("test", x1, x2, delta, cov));
  }
  {
    fuse_variables::Position3DStamped x1(rclcpp::Time(1234, 5678), fuse_core::uuid::generate(
        "clank"));
    fuse_variables::Position3DStamped x2(rclcpp::Time(1235, 5678), fuse_core::uuid::generate(
        "clank"));
    fuse_core::VectorXd delta(3);
    delta << 1.0, 2.0, 3.0;
    fuse_core::MatrixXd cov(3, 3);
    cov << 1.0, 0.1, 0.2, 0.3, 2.0, 0.3, 0.2, 0.3, 3.0;
    EXPECT_NO_THROW(
      fuse_constraints::RelativePosition3DStampedConstraint constraint("test", x1, x2, delta, cov));
  }
  {
    fuse_variables::VelocityAngular2DStamped x1(rclcpp::Time(1234, 5678),
      fuse_core::uuid::generate("gort"));
    fuse_variables::VelocityAngular2DStamped x2(rclcpp::Time(1235, 5678),
      fuse_core::uuid::generate("gort"));
    fuse_core::VectorXd delta(1);
    delta << 3.0;
    fuse_core::MatrixXd cov(1, 1);
    cov << 1.0;
    EXPECT_NO_THROW(
      fuse_constraints::RelativeVelocityAngular2DStampedConstraint constraint(
        "test", x1, x2, delta,
        cov));
  }
  {
    fuse_variables::VelocityLinear2DStamped x1(rclcpp::Time(1234, 5678),
      fuse_core::uuid::generate("bishop"));
    fuse_variables::VelocityLinear2DStamped x2(rclcpp::Time(1235, 5678),
      fuse_core::uuid::generate("bishop"));
    fuse_core::VectorXd delta(2);
    delta << 1.0, 2.0;
    fuse_core::MatrixXd cov(2, 2);
    cov << 1.0, 0.1, 0.1, 2.0;
    EXPECT_NO_THROW(
      fuse_constraints::RelativeVelocityLinear2DStampedConstraint constraint(
        "test", x1, x2, delta,
        cov));
  }
}

TEST(RelativeConstraint, PartialMeasurement)
{
  fuse_variables::Position3DStamped x1(rclcpp::Time(1234, 5678), fuse_core::uuid::generate("vici"));
  fuse_variables::Position3DStamped x2(rclcpp::Time(1235, 5678), fuse_core::uuid::generate("vici"));
  fuse_core::VectorXd delta(2);
  delta << 3.0, 1.0;
  fuse_core::MatrixXd cov(2, 2);
  cov << 3.0, 0.2, 0.2, 1.0;
  auto indices = std::vector<size_t>{2, 0};
  EXPECT_NO_THROW(
    fuse_constraints::RelativePosition3DStampedConstraint constraint(
      "test", x1, x2, delta, cov,
      indices));
}

TEST(RelativeConstraint, Covariance)
{
  // Test the covariance of a full measurement
  {
    // Verify the covariance <--> sqrt information conversions are correct
    fuse_variables::AccelerationLinear2DStamped x1(rclcpp::Time(1234, 5678),
      fuse_core::uuid::generate("chappie"));
    fuse_variables::AccelerationLinear2DStamped x2(rclcpp::Time(1235, 5678),
      fuse_core::uuid::generate("chappie"));
    fuse_core::VectorXd delta(2);
    delta << 1.0, 2.0;
    fuse_core::MatrixXd cov(2, 2);
    cov << 1.0, 0.1, 0.1, 2.0;
    fuse_constraints::RelativeAccelerationLinear2DStampedConstraint constraint("test", x1, x2,
      delta, cov);
    // Define the expected matrices (used Octave to compute sqrt_info: 'chol(inv(A))')
    fuse_core::Matrix2d expected_sqrt_info;
    expected_sqrt_info << 1.002509414234171, -0.050125470711709,
      0.000000000000000, 0.707106781186547;
    fuse_core::Matrix2d expected_cov = cov;
    // Compare
    EXPECT_TRUE(expected_cov.isApprox(constraint.covariance(), 1.0e-9));
    EXPECT_TRUE(expected_sqrt_info.isApprox(constraint.sqrtInformation(), 1.0e-9));
  }
  // Test the covariance of a partial measurement
  {
    fuse_variables::Position3DStamped x1(rclcpp::Time(1234, 5678),
      fuse_core::uuid::generate("astroboy"));
    fuse_variables::Position3DStamped x2(rclcpp::Time(1235, 5678),
      fuse_core::uuid::generate("astroboy"));
    fuse_core::VectorXd delta(2);
    delta << 3.0, 1.0;
    fuse_core::MatrixXd cov(2, 2);
    cov << 3.0, 0.2, 0.2, 1.0;
    auto indices = std::vector<size_t>{2, 0};
    fuse_constraints::RelativePosition3DStampedConstraint constraint("test", x1, x2, delta, cov,
      indices);
    // Define the expected matrices
    fuse_core::Vector3d expected_delta;
    expected_delta << 1.0, 0.0, 3.0;
    fuse_core::Matrix3d expected_cov;
    expected_cov << 1.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.2, 0.0, 3.0;
    fuse_core::MatrixXd expected_sqrt_info(2, 3);
    /* *INDENT-OFF* */
    expected_sqrt_info << -0.116247638743819,  0.000000000000000,  0.581238193719096,
                           1.000000000000000,  0.000000000000000,  0.000000000000000;
    /* *INDENT-ON* */
    // Compare
    EXPECT_TRUE(expected_delta.isApprox(constraint.delta(), 1.0e-9));
    EXPECT_TRUE(expected_cov.isApprox(constraint.covariance(), 1.0e-9));
    EXPECT_TRUE(expected_sqrt_info.isApprox(constraint.sqrtInformation(), 1.0e-9));
  }
}

TEST(RelativeConstraint, Optimization)
{
  // Test optimizing a full measurement
  {
    // Optimize a two-variable system with a prior on the first variable and a relative constraint
    // connecting the two. Verify the expected value and covariance are generated. Create the
    // variables
    auto x1 = fuse_variables::AccelerationLinear2DStamped::make_shared(
      rclcpp::Time(1234, 5678),
      fuse_core::uuid::generate("t800"));
    x1->x() = 10.7;
    x1->y() = -3.2;
    auto x2 = fuse_variables::AccelerationLinear2DStamped::make_shared(
      rclcpp::Time(1235, 5678),
      fuse_core::uuid::generate("t800"));
    x2->x() = -4.2;
    x2->y() = 1.9;
    // Create an absolute constraint
    fuse_core::VectorXd mean(2);
    mean << 1.0, 2.0;
    fuse_core::MatrixXd cov1(2, 2);
    cov1 << 1.0, 0.1, 0.1, 2.0;
    auto prior = fuse_constraints::AbsoluteAccelerationLinear2DStampedConstraint::make_shared(
      "test",
      *x1,
      mean,
      cov1);
    // Create an relative constraint
    fuse_core::VectorXd delta(2);
    delta << 0.1, 0.2;
    fuse_core::MatrixXd cov2(2, 2);
    cov2 << 1.0, 0.0, 0.0, 2.0;
    auto relative = fuse_constraints::RelativeAccelerationLinear2DStampedConstraint::make_shared(
      "test",
      *x1,
      *x2,
      delta,
      cov2);
    // Build the problem
    ceres::Problem::Options problem_options;
    problem_options.loss_function_ownership = fuse_core::Loss::Ownership;
    ceres::Problem problem(problem_options);
    problem.AddParameterBlock(
      x1->data(),
      x1->size(),
#if !CERES_SUPPORTS_MANIFOLDS
      x1->localParameterization());
#else
      x1->manifold());
#endif
    problem.AddParameterBlock(
      x2->data(),
      x2->size(),
#if !CERES_SUPPORTS_MANIFOLDS
      x2->localParameterization());
#else
      x2->manifold());
#endif
    std::vector<double *> prior_parameter_blocks;
    prior_parameter_blocks.push_back(x1->data());
    problem.AddResidualBlock(
      prior->costFunction(),
      prior->lossFunction(),
      prior_parameter_blocks);
    std::vector<double *> relative_parameter_blocks;
    relative_parameter_blocks.push_back(x1->data());
    relative_parameter_blocks.push_back(x2->data());
    problem.AddResidualBlock(
      relative->costFunction(),
      relative->lossFunction(),
      relative_parameter_blocks);
    // Run the solver
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    // Check
    EXPECT_NEAR(1.0, x1->x(), 1.0e-5);
    EXPECT_NEAR(2.0, x1->y(), 1.0e-5);
    EXPECT_NEAR(1.1, x2->x(), 1.0e-5);
    EXPECT_NEAR(2.2, x2->y(), 1.0e-5);
    // Compute the covariance
    std::vector<std::pair<const double *, const double *>> covariance_blocks;
    covariance_blocks.emplace_back(x1->data(), x1->data());
    covariance_blocks.emplace_back(x2->data(), x2->data());
    ceres::Covariance::Options cov_options;
    ceres::Covariance covariance(cov_options);
    covariance.Compute(covariance_blocks, &problem);
    // Check the x1 marginal covariance
    std::vector<double> x1_covariance_vector(x1->size() * x1->size());
    covariance.GetCovarianceBlock(x1->data(), x1->data(), x1_covariance_vector.data());
    fuse_core::Matrix2d x1_actual_covariance(x1_covariance_vector.data());
    fuse_core::Matrix2d x1_expected_covariance;
    x1_expected_covariance << 1.0, 0.1, 0.1, 2.0;
    EXPECT_TRUE(x1_expected_covariance.isApprox(x1_actual_covariance, 1.0e-9));
    // Check the x2 marginal covariance
    std::vector<double> x2_covariance_vector(x2->size() * x2->size());
    covariance.GetCovarianceBlock(x2->data(), x2->data(), x2_covariance_vector.data());
    fuse_core::Matrix2d x2_actual_covariance(x2_covariance_vector.data());
    fuse_core::Matrix2d x2_expected_covariance;
    x2_expected_covariance << 2.0, 0.1, 0.1, 4.0;
    EXPECT_TRUE(x2_expected_covariance.isApprox(x2_actual_covariance, 1.0e-9));
  }
  // Test optimizing a partial measurement. This is tricky, because a partial measurement is rank-
  // deficient by definition, which cannot be optimized alone. Instead, we will simply add a partial
  // measurement to our full measurement example.
  {
    // Optimize a two-variable system with a prior on the first variable and a relative constraint
    // connecting the two. Verify the expected value and covariance are generated. Create the
    // variables
    auto x1 = fuse_variables::Position3DStamped::make_shared(
      rclcpp::Time(1234, 5678),
      fuse_core::uuid::generate("t1000"));
    x1->x() = 10.7;
    x1->y() = -3.2;
    x1->z() = 0.4;
    auto x2 = fuse_variables::Position3DStamped::make_shared(
      rclcpp::Time(1235, 5678),
      fuse_core::uuid::generate("t1000"));
    x2->x() = -4.2;
    x2->y() = 1.9;
    x2->z() = 19.2;
    // Create an absolute constraint
    fuse_core::VectorXd mean1(3);
    mean1 << 1.0, 2.0, 3.0;
    fuse_core::MatrixXd cov1(3, 3);
    cov1 << 1.0, 0.1, 0.2, 0.1, 2.0, 0.3, 0.2, 0.3, 3.0;
    auto c1 = fuse_constraints::AbsolutePosition3DStampedConstraint::make_shared(
      "test",
      *x1,
      mean1,
      cov1);
    // Create an relative constraint
    fuse_core::VectorXd delta2(3);
    delta2 << 0.1, 0.2, 0.3;
    fuse_core::MatrixXd cov2(3, 3);
    cov2 << 1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 3.0;
    auto c2 = fuse_constraints::RelativePosition3DStampedConstraint::make_shared(
      "test",
      *x1,
      *x2,
      delta2,
      cov2);
    // Create an partial relative constraint
    fuse_core::VectorXd delta3(2);
    delta3 << 0.1, 0.2;
    fuse_core::MatrixXd cov3(2, 2);
    cov3 << 1.0, 0.0, 0.0, 3.0;
    auto indices3 = std::vector<size_t>{2, 0};
    auto c3 = fuse_constraints::RelativePosition3DStampedConstraint::make_shared(
      "test",
      *x1,
      *x2,
      delta3,
      cov3,
      indices3);
    // Build the problem
    ceres::Problem::Options problem_options;
    problem_options.loss_function_ownership = fuse_core::Loss::Ownership;
    ceres::Problem problem(problem_options);
    problem.AddParameterBlock(
      x1->data(),
      x1->size(),
#if !CERES_SUPPORTS_MANIFOLDS
      x1->localParameterization());
#else
      x1->manifold());
#endif
    problem.AddParameterBlock(
      x2->data(),
      x2->size(),
#if !CERES_SUPPORTS_MANIFOLDS
      x2->localParameterization());
#else
      x2->manifold());
#endif
    std::vector<double *> c1_parameter_blocks;
    c1_parameter_blocks.push_back(x1->data());
    problem.AddResidualBlock(
      c1->costFunction(),
      c1->lossFunction(),
      c1_parameter_blocks);
    std::vector<double *> c2_parameter_blocks;
    c2_parameter_blocks.push_back(x1->data());
    c2_parameter_blocks.push_back(x2->data());
    problem.AddResidualBlock(
      c2->costFunction(),
      c2->lossFunction(),
      c2_parameter_blocks);
    std::vector<double *> c3_parameter_blocks;
    c3_parameter_blocks.push_back(x1->data());
    c3_parameter_blocks.push_back(x2->data());
    problem.AddResidualBlock(
      c3->costFunction(),
      c3->lossFunction(),
      c3_parameter_blocks);
    // Run the solver
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    // Check
    EXPECT_NEAR(1.0, x1->x(), 1.0e-5);
    EXPECT_NEAR(2.0, x1->y(), 1.0e-5);
    EXPECT_NEAR(3.0, x1->z(), 1.0e-5);
    EXPECT_NEAR(1.125, x2->x(), 1.0e-5);
    EXPECT_NEAR(2.2, x2->y(), 1.0e-5);
    EXPECT_NEAR(3.15, x2->z(), 1.0e-5);
    // Compute the marginal covariances
    std::vector<std::pair<const double *, const double *>> covariance_blocks;
    covariance_blocks.emplace_back(x1->data(), x1->data());
    covariance_blocks.emplace_back(x2->data(), x2->data());
    ceres::Covariance::Options cov_options;
    ceres::Covariance covariance(cov_options);
    covariance.Compute(covariance_blocks, &problem);
    // Check the x1 marginal covariance
    std::vector<double> x1_covariance_vector(x1->size() * x1->size());
    covariance.GetCovarianceBlock(x1->data(), x1->data(), x1_covariance_vector.data());
    fuse_core::Matrix3d x1_actual_covariance(x1_covariance_vector.data());
    fuse_core::Matrix3d x1_expected_covariance;
    x1_expected_covariance << 1.0, 0.1, 0.2, 0.1, 2.0, 0.3, 0.2, 0.3, 3.0;
    EXPECT_TRUE(x1_expected_covariance.isApprox(x1_actual_covariance, 1.0e-9));
    // Check the x2 marginal covariance
    std::vector<double> x2_covariance_vector(x2->size() * x2->size());
    covariance.GetCovarianceBlock(x2->data(), x2->data(), x2_covariance_vector.data());
    fuse_core::Matrix3d x2_actual_covariance(x2_covariance_vector.data());
    fuse_core::Matrix3d x2_expected_covariance;
    x2_expected_covariance << 1.75, 0.1, 0.2, 0.1, 4.0, 0.3, 0.2, 0.3, 3.75;
    EXPECT_TRUE(x2_expected_covariance.isApprox(x2_actual_covariance, 1.0e-9));
  }
}

TEST(RelativeConstraint, RelativeOrientation2DOptimization)
{
  // Optimize a two-variable system with a prior on the first variable and a relative constraint
  // connecting the two. Verify the expected value and covariance are generated. Create the
  // variables
  auto x1 = fuse_variables::Orientation2DStamped::make_shared(
    rclcpp::Time(1234, 5678),
    fuse_core::uuid::generate("t800"));
  x1->yaw() = 0.7;
  auto x2 = fuse_variables::Orientation2DStamped::make_shared(
    rclcpp::Time(1235, 5678),
    fuse_core::uuid::generate("t800"));
  x2->yaw() = -2.2;
  // Create an absolute constraint
  fuse_core::VectorXd mean(1);
  mean << 1.0;
  fuse_core::MatrixXd cov1(1, 1);
  cov1 << 2.0;
  auto prior = fuse_constraints::AbsoluteOrientation2DStampedConstraint::make_shared(
    "test",
    *x1,
    mean,
    cov1);
  // Create an relative constraint
  fuse_core::VectorXd delta(1);
  delta << 0.1;
  fuse_core::MatrixXd cov2(1, 1);
  cov2 << 1.0;
  auto relative = fuse_constraints::RelativeOrientation2DStampedConstraint::make_shared(
    "test",
    *x1,
    *x2,
    delta,
    cov2);
  // Build the problem
  ceres::Problem::Options problem_options;
  problem_options.loss_function_ownership = fuse_core::Loss::Ownership;
  ceres::Problem problem(problem_options);
  problem.AddParameterBlock(
    x1->data(),
    x1->size(),
#if !CERES_SUPPORTS_MANIFOLDS
    x1->localParameterization());
#else
    x1->manifold());
#endif
  problem.AddParameterBlock(
    x2->data(),
    x2->size(),
#if !CERES_SUPPORTS_MANIFOLDS
    x2->localParameterization());
#else
    x2->manifold());
#endif
  std::vector<double *> prior_parameter_blocks;
  prior_parameter_blocks.push_back(x1->data());
  problem.AddResidualBlock(
    prior->costFunction(),
    prior->lossFunction(),
    prior_parameter_blocks);
  std::vector<double *> relative_parameter_blocks;
  relative_parameter_blocks.push_back(x1->data());
  relative_parameter_blocks.push_back(x2->data());
  problem.AddResidualBlock(
    relative->costFunction(),
    relative->lossFunction(),
    relative_parameter_blocks);
  // Run the solver
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  // Check
  EXPECT_NEAR(1.0, x1->yaw(), 1.0e-5);
  EXPECT_NEAR(1.1, x2->yaw(), 1.0e-5);
  // Compute the covariance
  std::vector<std::pair<const double *, const double *>> covariance_blocks;
  covariance_blocks.emplace_back(x1->data(), x1->data());
  covariance_blocks.emplace_back(x2->data(), x2->data());
  ceres::Covariance::Options cov_options;
  ceres::Covariance covariance(cov_options);
  covariance.Compute(covariance_blocks, &problem);
  // Check the x1 marginal covariance
  std::vector<double> x1_covariance_vector(x1->size() * x1->size());
  covariance.GetCovarianceBlock(x1->data(), x1->data(), x1_covariance_vector.data());
  fuse_core::Matrix1d x1_actual_covariance(x1_covariance_vector.data());
  fuse_core::Matrix1d x1_expected_covariance;
  x1_expected_covariance << 2.0;
  EXPECT_TRUE(x1_expected_covariance.isApprox(x1_actual_covariance, 1.0e-9));
  // Check the x2 marginal covariance
  std::vector<double> x2_covariance_vector(x2->size() * x2->size());
  covariance.GetCovarianceBlock(x2->data(), x2->data(), x2_covariance_vector.data());
  fuse_core::Matrix1d x2_actual_covariance(x2_covariance_vector.data());
  fuse_core::Matrix1d x2_expected_covariance;
  x2_expected_covariance << 3.0;
  EXPECT_TRUE(x2_expected_covariance.isApprox(x2_actual_covariance, 1.0e-9));
}

TEST(RelativeConstraint, Serialization)
{
  // Construct a constraint
  fuse_variables::AccelerationAngular2DStamped x1(rclcpp::Time(1234, 5678),
    fuse_core::uuid::generate("robby"));
  fuse_variables::AccelerationAngular2DStamped x2(rclcpp::Time(1235, 5678),
    fuse_core::uuid::generate("robby"));
  fuse_core::VectorXd delta(1);
  delta << 3.0;
  fuse_core::MatrixXd cov(1, 1);
  cov << 1.0;
  fuse_constraints::RelativeAccelerationAngular2DStampedConstraint expected("test", x1, x2, delta,
    cov);

  // Serialize the constraint into an archive
  std::stringstream stream;
  {
    fuse_core::TextOutputArchive archive(stream);
    expected.serialize(archive);
  }

  // Deserialize a new constraint from that same stream
  fuse_constraints::RelativeAccelerationAngular2DStampedConstraint actual;
  {
    fuse_core::TextInputArchive archive(stream);
    actual.deserialize(archive);
  }

  // Compare
  EXPECT_EQ(expected.uuid(), actual.uuid());
  EXPECT_EQ(expected.variables(), actual.variables());
  EXPECT_MATRIX_EQ(expected.delta(), actual.delta());
  EXPECT_MATRIX_EQ(expected.sqrtInformation(), actual.sqrtInformation());
}
