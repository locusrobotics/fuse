/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Locus Robotics
 *  Copyright (c) 2023, Giacomo Franchini
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

#include <fuse_constraints/absolute_orientation_3d_stamped_euler_constraint.hpp>
#include <fuse_core/ceres_macros.hpp>
#include <fuse_core/eigen.hpp>
#include <fuse_core/eigen_gtest.hpp>
#include <fuse_core/uuid.hpp>
#include <fuse_variables/orientation_3d_stamped.hpp>

using fuse_constraints::AbsoluteOrientation3DStampedEulerConstraint;
using fuse_variables::Orientation3DStamped;


TEST(AbsoluteOrientation3DStampedEulerConstraint, Constructor)
{
  // Construct a constraint just to make sure it compiles.
  Orientation3DStamped orientation_variable(rclcpp::Time(1234, 5678),
    fuse_core::uuid::generate("walle"));
  fuse_core::Vector3d mean;
  mean << 1.0, 2.0, 3.0;
  fuse_core::Matrix3d cov;
  cov << 1.0, 0.1, 0.2, 0.1, 2.0, 0.3, 0.2, 0.3, 3.0;
  std::vector<Orientation3DStamped::Euler> axes =
  {Orientation3DStamped::Euler::YAW, Orientation3DStamped::Euler::ROLL,
    Orientation3DStamped::Euler::PITCH};
  EXPECT_NO_THROW(
    AbsoluteOrientation3DStampedEulerConstraint constraint(
      "test", orientation_variable, mean, cov,
      axes));
}

TEST(AbsoluteOrientation3DStampedEulerConstraint, Covariance)
{
  // Verify the covariance <--> sqrt information conversions are correct
  Orientation3DStamped orientation_variable(rclcpp::Time(1234, 5678), fuse_core::uuid::generate(
      "mo"));
  fuse_core::Vector3d mean;
  mean << 1.0, 2.0, 3.0;
  fuse_core::Matrix3d cov;
  cov << 1.0, 0.1, 0.2, 0.1, 2.0, 0.3, 0.2, 0.3, 3.0;
  std::vector<Orientation3DStamped::Euler> axes =
  {Orientation3DStamped::Euler::YAW, Orientation3DStamped::Euler::ROLL,
    Orientation3DStamped::Euler::PITCH};
  AbsoluteOrientation3DStampedEulerConstraint constraint("test", orientation_variable, mean, cov,
    axes);

  // Define the expected matrices (used Octave to compute sqrt_info: 'chol(inv(A))')
  fuse_core::Matrix3d expected_sqrt_info;
  /* *INDENT-OFF* */
  expected_sqrt_info <<  1.008395589795798, -0.040950074712520, -0.063131365181801,
                         0.000000000000000,  0.712470499879096, -0.071247049987910,
                         0.000000000000000,  0.000000000000000,  0.577350269189626;
  /* *INDENT-ON* */
  fuse_core::Matrix3d expected_cov = cov;

  // Compare
  EXPECT_TRUE(expected_cov.isApprox(constraint.covariance(), 1.0e-9));
  EXPECT_TRUE(expected_sqrt_info.isApprox(constraint.sqrtInformation(), 1.0e-9));
}

TEST(AbsoluteOrientation3DStampedEulerConstraint, OptimizationFull)
{
  // Optimize a single pose and single constraint, verify the expected value and covariance are
  // generated. Create the variables
  auto orientation_variable = Orientation3DStamped::make_shared(
    rclcpp::Time(1, 0), fuse_core::uuid::generate("spra"));
  orientation_variable->w() = 0.952;
  orientation_variable->x() = 0.038;
  orientation_variable->y() = -0.189;
  orientation_variable->z() = 0.239;

  // Create an absolute orientation constraint
  fuse_core::Vector3d mean;
  mean << 0.5, 1.0, 1.5;
  fuse_core::Matrix3d cov;
  cov << 1.0, 0.1, 0.2, 0.1, 2.0, 0.3, 0.2, 0.3, 3.0;
  std::vector<Orientation3DStamped::Euler> axes =
  {Orientation3DStamped::Euler::YAW, Orientation3DStamped::Euler::ROLL,
    Orientation3DStamped::Euler::PITCH};
  auto constraint = AbsoluteOrientation3DStampedEulerConstraint::make_shared(
    "test",
    *orientation_variable,
    mean,
    cov,
    axes);

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

  // Check
  Eigen::Quaterniond expected = 
    Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(1.5, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitX());
  EXPECT_NEAR(expected.w(), orientation_variable->w(), 5.0e-3);
  EXPECT_NEAR(expected.x(), orientation_variable->x(), 5.0e-3);
  EXPECT_NEAR(expected.y(), orientation_variable->y(), 5.0e-3);
  EXPECT_NEAR(expected.z(), orientation_variable->z(), 5.0e-3);

  // TODO(swilliams) Determine what I expect the covariance matrix to be and test it here
}

TEST(AbsoluteOrientation3DStampedEulerConstraint, OptimizationPartial)
{
  // Optimize a single pose and single constraint, verify the expected value and covariance are
  // generated. Create the variables
  auto orientation_variable = Orientation3DStamped::make_shared(
    rclcpp::Time(1, 0), fuse_core::uuid::generate("spra"));
  orientation_variable->w() = 0.952;
  orientation_variable->x() = 0.038;
  orientation_variable->y() = -0.189;
  orientation_variable->z() = 0.239;

  // Create an absolute orientation constraint
  fuse_core::Vector2d mean1;
  mean1 << 0.5, 1.5;
  fuse_core::Matrix2d cov1;
  cov1 << 1.0, 0.2, 0.2, 3.0;
  std::vector<Orientation3DStamped::Euler> axes1 =
  {Orientation3DStamped::Euler::YAW, Orientation3DStamped::Euler::PITCH};
  auto constraint1 = AbsoluteOrientation3DStampedEulerConstraint::make_shared(
    "test",
    *orientation_variable,
    mean1,
    cov1,
    axes1);

  fuse_core::Vector1d mean2;
  mean2 << 1.0;
  fuse_core::Matrix1d cov2;
  cov2 << 2.0;
  std::vector<Orientation3DStamped::Euler> axes2 =
  {Orientation3DStamped::Euler::ROLL};
  auto constraint2 = AbsoluteOrientation3DStampedEulerConstraint::make_shared(
    "test",
    *orientation_variable,
    mean2,
    cov2,
    axes2);

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
  Eigen::Quaterniond expected = 
    Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(1.5, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitX());
  EXPECT_NEAR(expected.w(), orientation_variable->w(), 5.0e-3);
  EXPECT_NEAR(expected.x(), orientation_variable->x(), 5.0e-3);
  EXPECT_NEAR(expected.y(), orientation_variable->y(), 5.0e-3);
  EXPECT_NEAR(expected.z(), orientation_variable->z(), 5.0e-3);

  // TODO(swilliams) Determine what I expect the covariance matrix to be and test it here
}

TEST(AbsoluteOrientation3DStampedEulerConstraint, Serialization)
{
  // Construct a constraint
  Orientation3DStamped orientation_variable(rclcpp::Time(1234, 5678),
    fuse_core::uuid::generate("walle"));
  fuse_core::Vector3d mean;
  mean << 1.0, 2.0, 3.0;
  fuse_core::Matrix3d cov;
  cov << 1.0, 0.1, 0.2, 0.1, 2.0, 0.3, 0.2, 0.3, 3.0;
  std::vector<Orientation3DStamped::Euler> axes =
  {Orientation3DStamped::Euler::YAW, Orientation3DStamped::Euler::ROLL,
    Orientation3DStamped::Euler::PITCH};
  AbsoluteOrientation3DStampedEulerConstraint expected("test", orientation_variable, mean, cov,
    axes);

  // Serialize the constraint into an archive
  std::stringstream stream;
  {
    fuse_core::TextOutputArchive archive(stream);
    expected.serialize(archive);
  }

  // Deserialize a new constraint from that same stream
  AbsoluteOrientation3DStampedEulerConstraint actual;
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
