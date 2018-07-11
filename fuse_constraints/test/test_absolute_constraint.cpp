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
#include <fuse_constraints/absolute_constraint.h>
#include <fuse_core/uuid.h>
#include <fuse_variables/acceleration_angular_2d_stamped.h>
#include <fuse_variables/acceleration_linear_2d_stamped.h>
#include <fuse_variables/orientation_2d_stamped.h>
#include <fuse_variables/position_2d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>
#include <fuse_variables/velocity_angular_2d_stamped.h>
#include <fuse_variables/velocity_linear_2d_stamped.h>

#include <ceres/covariance.h>
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <Eigen/Core>
#include <gtest/gtest.h>

#include <utility>
#include <vector>


TEST(AbsoluteConstraint, Constructor)
{
  // Construct a constraint for every type, just to make sure they compile.
  {
    fuse_variables::AccelerationAngular2DStamped variable(ros::Time(1234, 5678), fuse_core::uuid::generate("robby"));
    Eigen::Matrix<double, 1, 1> mean;
    mean << 3.0;
    Eigen::Matrix<double, 1, 1> cov;
    cov << 1.0;
    EXPECT_NO_THROW(fuse_constraints::AbsoluteAccelerationAngular2DStampedConstraint constraint(variable, mean, cov));
  }
  {
    fuse_variables::AccelerationLinear2DStamped variable(ros::Time(1234, 5678), fuse_core::uuid::generate("bender"));
    Eigen::Matrix<double, 2, 1> mean;
    mean << 1.0, 2.0;
    Eigen::Matrix<double, 2, 2> cov;
    cov << 1.0, 0.1, 0.1, 2.0;
    EXPECT_NO_THROW(fuse_constraints::AbsoluteAccelerationLinear2DStampedConstraint constraint(variable, mean, cov));
  }
  {
    fuse_variables::Orientation2DStamped variable(ros::Time(1234, 5678), fuse_core::uuid::generate("johnny5"));
    Eigen::Matrix<double, 1, 1> mean;
    mean << 3.0;
    Eigen::Matrix<double, 1, 1> cov;
    cov << 1.0;
    EXPECT_NO_THROW(fuse_constraints::AbsoluteOrientation2DStampedConstraint constraint(variable, mean, cov));
  }
  {
    fuse_variables::Position2DStamped variable(ros::Time(1234, 5678), fuse_core::uuid::generate("rosie"));
    Eigen::Matrix<double, 2, 1> mean;
    mean << 1.0, 2.0;
    Eigen::Matrix<double, 2, 2> cov;
    cov << 1.0, 0.1, 0.1, 2.0;
    EXPECT_NO_THROW(fuse_constraints::AbsolutePosition2DStampedConstraint constraint(variable, mean, cov));
  }
  {
    fuse_variables::Position3DStamped variable(ros::Time(1234, 5678), fuse_core::uuid::generate("clank"));
    Eigen::Matrix<double, 3, 1> mean;
    mean << 1.0, 2.0, 3.0;
    Eigen::Matrix<double, 3, 3> cov;
    cov << 1.0, 0.1, 0.2, 0.1, 2.0, 0.3, 0.2, 0.3, 3.0;
    EXPECT_NO_THROW(fuse_constraints::AbsolutePosition3DStampedConstraint constraint(variable, mean, cov));
  }
  {
    fuse_variables::VelocityAngular2DStamped variable(ros::Time(1234, 5678), fuse_core::uuid::generate("gort"));
    Eigen::Matrix<double, 1, 1> mean;
    mean << 3.0;
    Eigen::Matrix<double, 1, 1> cov;
    cov << 1.0;
    EXPECT_NO_THROW(fuse_constraints::AbsoluteVelocityAngular2DStampedConstraint constraint(variable, mean, cov));
  }
  {
    fuse_variables::VelocityLinear2DStamped variable(ros::Time(1234, 5678), fuse_core::uuid::generate("bishop"));
    Eigen::Matrix<double, 2, 1> mean;
    mean << 1.0, 2.0;
    Eigen::Matrix<double, 2, 2> cov;
    cov << 1.0, 0.1, 0.1, 2.0;
    EXPECT_NO_THROW(fuse_constraints::AbsoluteVelocityLinear2DStampedConstraint constraint(variable, mean, cov));
  }
}

TEST(AbsoluteConstraint, PartialMeasurement)
{
  fuse_variables::Position3DStamped variable(ros::Time(1234, 5678), fuse_core::uuid::generate("vici"));
  Eigen::Matrix<double, 2, 1> mean;
  mean << 3.0, 1.0;
  Eigen::Matrix<double, 2, 2> cov;
  cov << 3.0, 0.2, 0.2, 1.0;
  auto indices = std::vector<size_t>{2, 0};
  EXPECT_NO_THROW(fuse_constraints::AbsolutePosition3DStampedConstraint constraint(variable, mean, cov, indices));
}

TEST(AbsoluteConstraint, Covariance)
{
  // Test the covariance of a full measurement
  {
    // Verify the covariance <--> sqrt information conversions are correct
    fuse_variables::AccelerationLinear2DStamped variable(ros::Time(1234, 5678), fuse_core::uuid::generate("chappie"));
    Eigen::Matrix<double, 2, 1> mean;
    mean << 1.0, 2.0;
    Eigen::Matrix<double, 2, 2> cov;
    cov << 1.0, 0.1, 0.1, 2.0;
    fuse_constraints::AbsoluteAccelerationLinear2DStampedConstraint constraint(variable, mean, cov);
    // Define the expected matrices (used Octave to compute sqrt_info: 'chol(inv(A))')
    Eigen::Matrix<double, 2, 2> expected_sqrt_info;
    expected_sqrt_info <<  1.002509414234171, -0.050125470711709,
                           0.000000000000000,  0.707106781186547;
    Eigen::Matrix<double, 2, 2> expected_cov = cov;
    // Compare
    EXPECT_TRUE(expected_cov.isApprox(constraint.covariance(), 1.0e-9));
    EXPECT_TRUE(expected_sqrt_info.isApprox(constraint.sqrtInformation(), 1.0e-9));
  }
  // Test the covariance of a partial measurement
  {
    fuse_variables::Position3DStamped variable(ros::Time(1234, 5678), fuse_core::uuid::generate("astroboy"));
    Eigen::Matrix<double, 2, 1> mean;
    mean << 3.0, 1.0;
    Eigen::Matrix<double, 2, 2> cov;
    cov << 3.0, 0.2, 0.2, 1.0;
    auto indices = std::vector<size_t>{2, 0};
    fuse_constraints::AbsolutePosition3DStampedConstraint constraint(variable, mean, cov, indices);
    // Define the expected matrices
    Eigen::Matrix<double, 3, 1> expected_mean;
    expected_mean << 1.0, 0.0, 3.0;
    Eigen::Matrix<double, 3, 3> expected_cov;
    expected_cov << 1.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.2, 0.0, 3.0;
    Eigen::Matrix<double, 2, 3> expected_sqrt_info;
    expected_sqrt_info << -0.116247638743819,  0.000000000000000,  0.581238193719096,
                           1.000000000000000,  0.000000000000000,  0.000000000000000;
    // Compare
    EXPECT_TRUE(expected_mean.isApprox(constraint.mean(), 1.0e-9));
    EXPECT_TRUE(expected_cov.isApprox(constraint.covariance(), 1.0e-9));
    EXPECT_TRUE(expected_sqrt_info.isApprox(constraint.sqrtInformation(), 1.0e-9));
  }
}

TEST(AbsoluteConstraint, Optimization)
{
  // Test optimizing a full measurement
  {
    // Optimize a single variable and single constraint, verify the expected value and covariance are generated.
    // Create a variable
    auto variable = fuse_variables::AccelerationLinear2DStamped::make_shared(ros::Time(1234, 5678),
                                                                             fuse_core::uuid::generate("t800"));
    variable->x() = 10.7;
    variable->y() = -3.2;
    // Create an absolute constraint
    Eigen::Matrix<double, 2, 1> mean;
    mean << 1.0, 2.0;
    Eigen::Matrix<double, 2, 2> cov;
    cov << 1.0, 0.1, 0.1, 2.0;
    auto constraint = fuse_constraints::AbsoluteAccelerationLinear2DStampedConstraint::make_shared(*variable,
                                                                                                   mean,
                                                                                                   cov);
    // Build the problem
    ceres::Problem problem;
    problem.AddParameterBlock(
      variable->data(),
      variable->size(),
      variable->localParameterization());
    std::vector<double*> parameter_blocks;
    parameter_blocks.push_back(variable->data());
    problem.AddResidualBlock(
      constraint->costFunction(),
      constraint->lossFunction(),
      parameter_blocks);
    // Run the solver
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    // Check
    EXPECT_NEAR(1.0, variable->x(), 1.0e-5);
    EXPECT_NEAR(2.0, variable->y(), 1.0e-5);
    // Compute the covariance
    std::vector<std::pair<const double*, const double*> > covariance_blocks;
    covariance_blocks.emplace_back(variable->data(), variable->data());
    ceres::Covariance::Options cov_options;
    ceres::Covariance covariance(cov_options);
    covariance.Compute(covariance_blocks, &problem);
    std::vector<double> covariance_vector(variable->size() * variable->size());
    covariance.GetCovarianceBlock(variable->data(), variable->data(), covariance_vector.data());
    Eigen::Matrix<double, 2, 2> covariance_matrix(covariance_vector.data());
    EXPECT_TRUE(cov.isApprox(covariance_matrix, 1.0e-9));
  }
  // Test optimizing a partial measurement. This is tricky, because a partial measurement is rank-deficient by
  // definition, which cannot be optimized alone. Instead, we will simply add a partial measurement to our full
  // measurement example.
  {
    // Optimize a single variable with a full measurement and a partial measurement
    // Verify the expected value and covariance are generated.
    // Create a variable
    auto var = fuse_variables::Position3DStamped::make_shared(ros::Time(1, 0), fuse_core::uuid::generate("t1000"));
    var->x() = 10.7;
    var->y() = -3.2;
    var->z() = 0.9;
    // Create a full measurement constraint
    Eigen::Matrix<double, 3, 1> mean1;
    mean1 << 1.0, 2.0, 3.0;
    Eigen::Matrix<double, 3, 3> cov1;
    cov1 << 1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0;
    auto constraint1 = fuse_constraints::AbsolutePosition3DStampedConstraint::make_shared(*var, mean1, cov1);
    Eigen::Matrix<double, 2, 1> mean2;
    mean2 << 4.0, 2.0;
    Eigen::Matrix<double, 2, 2> cov2;
    cov2 << 1.0, 0.0,
            0.0, 1.0;
    auto indices2 = std::vector<size_t>{2, 0};
    auto constraint2 = fuse_constraints::AbsolutePosition3DStampedConstraint::make_shared(*var, mean2, cov2, indices2);
    // Build the problem
    ceres::Problem problem;
    problem.AddParameterBlock(
      var->data(),
      var->size(),
      var->localParameterization());
    std::vector<double*> parameter_blocks;
    parameter_blocks.push_back(var->data());
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
    EXPECT_NEAR(1.5, var->x(), 1.0e-5);
    EXPECT_NEAR(2.0, var->y(), 1.0e-5);
    EXPECT_NEAR(3.5, var->z(), 1.0e-5);
    // Compute the covariance
    std::vector<std::pair<const double*, const double*> > covariance_blocks;
    covariance_blocks.emplace_back(var->data(), var->data());
    ceres::Covariance::Options cov_options;
    ceres::Covariance covariance(cov_options);
    covariance.Compute(covariance_blocks, &problem);
    std::vector<double> covariance_vector(var->size() * var->size());
    covariance.GetCovarianceBlock(var->data(), var->data(), covariance_vector.data());
    Eigen::Matrix<double, 3, 3> actual_cov(covariance_vector.data());
    Eigen::Matrix<double, 3, 3> expected_cov;
    expected_cov << 0.5, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 0.5;
    EXPECT_TRUE(expected_cov.isApprox(actual_cov, 1.0e-9));
  }
}

TEST(AbsoluteConstraint, AbsoluteOrientation2DOptimization)
{
  // Optimize a single variable and single constraint, verify the expected value and covariance are generated.
  // Create a variable
  auto variable = fuse_variables::Orientation2DStamped::make_shared(ros::Time(1234, 5678),
                                                                    fuse_core::uuid::generate("tiktok"));
  variable->yaw() = 0.7;
  // Create an absolute constraint
  Eigen::Matrix<double, 1, 1> mean;
  mean << 7.0;
  Eigen::Matrix<double, 1, 1> cov;
  cov << 0.10;
  auto constraint = fuse_constraints::AbsoluteOrientation2DStampedConstraint::make_shared(*variable, mean, cov);
  // Build the problem
  ceres::Problem problem;
  problem.AddParameterBlock(
    variable->data(),
    variable->size(),
    variable->localParameterization());
  std::vector<double*> parameter_blocks;
  parameter_blocks.push_back(variable->data());
  problem.AddResidualBlock(
    constraint->costFunction(),
    constraint->lossFunction(),
    parameter_blocks);
  // Run the solver
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  // Check
  EXPECT_NEAR(7.0 - 2 * M_PI, variable->yaw(), 1.0e-5);
  // Compute the covariance
  std::vector<std::pair<const double*, const double*> > covariance_blocks;
  covariance_blocks.emplace_back(variable->data(), variable->data());
  ceres::Covariance::Options cov_options;
  ceres::Covariance covariance(cov_options);
  covariance.Compute(covariance_blocks, &problem);
  std::vector<double> covariance_vector(variable->size() * variable->size());
  covariance.GetCovarianceBlock(variable->data(), variable->data(), covariance_vector.data());
  Eigen::Matrix<double, 1, 1> covariance_matrix(covariance_vector.data());
  EXPECT_TRUE(cov.isApprox(covariance_matrix, 1.0e-9));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
