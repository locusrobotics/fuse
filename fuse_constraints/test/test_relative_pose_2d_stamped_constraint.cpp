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
#include <fuse_constraints/relative_pose_2d_stamped_constraint.h>
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
using fuse_constraints::RelativePose2DStampedConstraint;


TEST(RelativePose2DStampedConstraint, Constructor)
{
  // Construct a constraint just to make sure it compiles.
  Orientation2DStamped orientation1(rclcpp::Time(1234, 5678), fuse_core::uuid::generate("r5d4"));
  Position2DStamped position1(rclcpp::Time(1234, 5678), fuse_core::uuid::generate("r5d4"));
  Orientation2DStamped orientation2(rclcpp::Time(1235, 5678), fuse_core::uuid::generate("r5d4"));
  Position2DStamped position2(rclcpp::Time(1235, 5678), fuse_core::uuid::generate("r5d4"));
  fuse_core::Vector3d delta;
  delta << 1.0, 2.0, 3.0;
  fuse_core::Matrix3d cov;
  cov << 1.0, 0.1, 0.2, 0.1, 2.0, 0.3, 0.2, 0.3, 3.0;
  EXPECT_NO_THROW(
    RelativePose2DStampedConstraint constraint("test", position1, orientation1, position2, orientation2, delta, cov));
}

TEST(RelativePose2DStampedConstraint, Covariance)
{
  // Verify the covariance <--> sqrt information conversions are correct
  Orientation2DStamped orientation1(rclcpp::Time(1234, 5678), fuse_core::uuid::generate("r5d4"));
  Position2DStamped position1(rclcpp::Time(1234, 5678), fuse_core::uuid::generate("r5d4"));
  Orientation2DStamped orientation2(rclcpp::Time(1235, 5678), fuse_core::uuid::generate("r5d4"));
  Position2DStamped position2(rclcpp::Time(1235, 5678), fuse_core::uuid::generate("r5d4"));
  fuse_core::Vector3d delta;
  delta << 1.0, 2.0, 3.0;
  fuse_core::Matrix3d cov;
  cov << 1.0, 0.1, 0.2, 0.1, 2.0, 0.3, 0.2, 0.3, 3.0;
  RelativePose2DStampedConstraint constraint(
    "test",
    position1,
    orientation1,
    position2,
    orientation2,
    delta,
    cov);
  // Define the expected matrices (used Octave to compute sqrt_info)
  fuse_core::Matrix3d expected_sqrt_info;
  expected_sqrt_info <<  1.008395589795798, -0.040950074712520, -0.063131365181801,
                         0.000000000000000,  0.712470499879096, -0.071247049987910,
                         0.000000000000000,  0.000000000000000,  0.577350269189626;
  fuse_core::Matrix3d expected_cov = cov;
  // Compare
  EXPECT_MATRIX_NEAR(expected_cov, constraint.covariance(), 1.0e-9);
  EXPECT_MATRIX_NEAR(expected_sqrt_info, constraint.sqrtInformation(), 1.0e-9);
}

TEST(RelativePose2DStampedConstraint, OptimizationFull)
{
  // Optimize a two-pose system with a pose prior and a relative pose constraint
  // Verify the expected poses and covariances are generated.
  // Create two poses
  auto orientation1 = Orientation2DStamped::make_shared(rclcpp::Time(1, 0), fuse_core::uuid::generate("3b6ra7"));
  orientation1->yaw() = 0.8;
  auto position1 = Position2DStamped::make_shared(rclcpp::Time(1, 0), fuse_core::uuid::generate("3b6ra7"));
  position1->x() = 1.5;
  position1->y() = -3.0;
  auto orientation2 = Orientation2DStamped::make_shared(rclcpp::Time(2, 0), fuse_core::uuid::generate("3b6ra7"));
  orientation2->yaw() = -2.7;
  auto position2 = Position2DStamped::make_shared(rclcpp::Time(2, 0), fuse_core::uuid::generate("3b6ra7"));
  position2->x() = 3.7;
  position2->y() = 1.2;
  // Create an absolute pose constraint at the origin
  fuse_core::Vector3d mean1;
  mean1 << 0.0, 0.0, 0.0;
  fuse_core::Matrix3d cov1;
  cov1 << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
  auto prior = AbsolutePose2DStampedConstraint::make_shared(
    "test",
    *position1,
    *orientation1,
    mean1,
    cov1);
  // Create a relative pose constraint for 1m in the x direction
  fuse_core::Vector3d delta2;
  delta2 << 1.0, 0.0, 0.0;
  fuse_core::Matrix3d cov2;
  cov2 << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
  auto relative = RelativePose2DStampedConstraint::make_shared(
    "test",
    *position1,
    *orientation1,
    *position2,
    *orientation2,
    delta2,
    cov2);
  // Build the problem
  ceres::Problem::Options problem_options;
  problem_options.loss_function_ownership = fuse_core::Loss::Ownership;
  ceres::Problem problem(problem_options);
  problem.AddParameterBlock(
    orientation1->data(),
    orientation1->size(),
    orientation1->localParameterization());
  problem.AddParameterBlock(
    position1->data(),
    position1->size(),
    position1->localParameterization());
  problem.AddParameterBlock(
    orientation2->data(),
    orientation2->size(),
    orientation2->localParameterization());
  problem.AddParameterBlock(
    position2->data(),
    position2->size(),
    position2->localParameterization());
  std::vector<double*> prior_parameter_blocks;
  prior_parameter_blocks.push_back(position1->data());
  prior_parameter_blocks.push_back(orientation1->data());
  problem.AddResidualBlock(
    prior->costFunction(),
    prior->lossFunction(),
    prior_parameter_blocks);
  std::vector<double*> relative_parameter_blocks;
  relative_parameter_blocks.push_back(position1->data());
  relative_parameter_blocks.push_back(orientation1->data());
  relative_parameter_blocks.push_back(position2->data());
  relative_parameter_blocks.push_back(orientation2->data());
  problem.AddResidualBlock(
    relative->costFunction(),
    relative->lossFunction(),
    relative_parameter_blocks);
  // Run the solver
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  // Check
  EXPECT_NEAR(0.0, position1->x(), 1.0e-5);
  EXPECT_NEAR(0.0, position1->y(), 1.0e-5);
  EXPECT_NEAR(0.0, orientation1->yaw(), 1.0e-5);
  EXPECT_NEAR(1.0, position2->x(), 1.0e-5);
  EXPECT_NEAR(0.0, position2->y(), 1.0e-5);
  EXPECT_NEAR(0.0, orientation2->yaw(), 1.0e-5);
  // Compute the marginal covariance for pose1
  {
    std::vector<std::pair<const double*, const double*> > covariance_blocks;
    covariance_blocks.emplace_back(position1->data(), position1->data());
    covariance_blocks.emplace_back(position1->data(), orientation1->data());
    covariance_blocks.emplace_back(orientation1->data(), orientation1->data());
    ceres::Covariance::Options cov_options;
    ceres::Covariance covariance(cov_options);
    covariance.Compute(covariance_blocks, &problem);
    std::vector<double> covariance_vector1(position1->size() * position1->size());
    covariance.GetCovarianceBlock(position1->data(), position1->data(), covariance_vector1.data());
    std::vector<double> covariance_vector2(position1->size() * orientation1->size());
    covariance.GetCovarianceBlock(position1->data(), orientation1->data(), covariance_vector2.data());
    std::vector<double> covariance_vector3(orientation1->size() * orientation1->size());
    covariance.GetCovarianceBlock(orientation1->data(), orientation1->data(), covariance_vector3.data());
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
    fuse_core::Matrix3d expected_covariance = cov1;
    EXPECT_MATRIX_NEAR(expected_covariance, actual_covariance, 1.0e-9);
  }
  // Compute the marginal covariance for pose2
  {
    std::vector<std::pair<const double*, const double*> > covariance_blocks;
    covariance_blocks.emplace_back(position2->data(), position2->data());
    covariance_blocks.emplace_back(position2->data(), orientation2->data());
    covariance_blocks.emplace_back(orientation2->data(), orientation2->data());
    ceres::Covariance::Options cov_options;
    ceres::Covariance covariance(cov_options);
    covariance.Compute(covariance_blocks, &problem);
    std::vector<double> covariance_vector1(position2->size() * position2->size());
    covariance.GetCovarianceBlock(position2->data(), position2->data(), covariance_vector1.data());
    std::vector<double> covariance_vector2(position2->size() * orientation2->size());
    covariance.GetCovarianceBlock(position2->data(), orientation2->data(), covariance_vector2.data());
    std::vector<double> covariance_vector3(orientation2->size() * orientation2->size());
    covariance.GetCovarianceBlock(orientation2->data(), orientation2->data(), covariance_vector3.data());
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
    fuse_core::Matrix3d expected_covariance;
    expected_covariance << 2.0, 0.0, 0.0, 0.0, 3.0, 1.0, 0.0, 1.0, 2.0;
    EXPECT_MATRIX_NEAR(expected_covariance, actual_covariance, 1.0e-9);
  }
}

TEST(RelativePose2DStampedConstraint, OptimizationPartial)
{
  // Optimize a two-pose system with a pose prior and a relative pose constraint
  // Verify the expected poses and covariances are generated.
  // Create two poses
  auto orientation1 = Orientation2DStamped::make_shared(rclcpp::Time(1, 0), fuse_core::uuid::generate("3b6ra7"));
  orientation1->yaw() = 0.8;
  auto position1 = Position2DStamped::make_shared(rclcpp::Time(1, 0), fuse_core::uuid::generate("3b6ra7"));
  position1->x() = 1.5;
  position1->y() = -3.0;

  auto orientation2 = Orientation2DStamped::make_shared(rclcpp::Time(2, 0), fuse_core::uuid::generate("3b6ra7"));
  orientation2->yaw() = -2.7;
  auto position2 = Position2DStamped::make_shared(rclcpp::Time(2, 0), fuse_core::uuid::generate("3b6ra7"));
  position2->x() = 3.7;
  position2->y() = 1.2;

  // Create an absolute pose constraint at the origin
  fuse_core::Vector3d mean1;
  mean1 << 0.0, 0.0, 0.0;
  fuse_core::Matrix3d cov1;
  cov1 << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
  auto prior = AbsolutePose2DStampedConstraint::make_shared(
    "test",
    *position1,
    *orientation1,
    mean1,
    cov1);

  // Create a relative pose constraint for 1m in the x direction
  fuse_core::Vector2d delta1;
  delta1 << 1.0, 0.0;
  fuse_core::Matrix2d cov_rel1;
  cov_rel1 << 1.0, 0.0, 0.0, 1.0;
  std::vector<size_t> axes_lin1 = {fuse_variables::Position2DStamped::X};
  std::vector<size_t> axes_ang1 = {fuse_variables::Orientation2DStamped::YAW};
  auto relative1 = RelativePose2DStampedConstraint::make_shared(
    "test",
    *position1,
    *orientation1,
    *position2,
    *orientation2,
    delta1,
    cov_rel1,
    axes_lin1,
    axes_ang1);

  // Create a relative pose constraint for 0m in the y direction
  fuse_core::Vector1d delta2;
  delta2 << 0.0;
  fuse_core::Matrix1d cov_rel2;
  cov_rel2 << 1.0;
  std::vector<size_t> axes_lin2 = {fuse_variables::Position2DStamped::Y};
  std::vector<size_t> axes_ang2 = {};
  auto relative2 = RelativePose2DStampedConstraint::make_shared(
    "test",
    *position1,
    *orientation1,
    *position2,
    *orientation2,
    delta2,
    cov_rel2,
    axes_lin2,
    axes_ang2);

  // Build the problem
  ceres::Problem::Options problem_options;
  problem_options.loss_function_ownership = fuse_core::Loss::Ownership;
  ceres::Problem problem(problem_options);
  problem.AddParameterBlock(
    orientation1->data(),
    orientation1->size(),
    orientation1->localParameterization());
  problem.AddParameterBlock(
    position1->data(),
    position1->size(),
    position1->localParameterization());
  problem.AddParameterBlock(
    orientation2->data(),
    orientation2->size(),
    orientation2->localParameterization());
  problem.AddParameterBlock(
    position2->data(),
    position2->size(),
    position2->localParameterization());

  std::vector<double*> prior_parameter_blocks;
  prior_parameter_blocks.push_back(position1->data());
  prior_parameter_blocks.push_back(orientation1->data());
  problem.AddResidualBlock(
    prior->costFunction(),
    prior->lossFunction(),
    prior_parameter_blocks);

  std::vector<double*> relative_parameter_blocks;
  relative_parameter_blocks.push_back(position1->data());
  relative_parameter_blocks.push_back(orientation1->data());
  relative_parameter_blocks.push_back(position2->data());
  relative_parameter_blocks.push_back(orientation2->data());
  problem.AddResidualBlock(
    relative1->costFunction(),
    relative1->lossFunction(),
    relative_parameter_blocks);
  problem.AddResidualBlock(
    relative2->costFunction(),
    relative2->lossFunction(),
    relative_parameter_blocks);

  // Run the solver
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // Check
  EXPECT_NEAR(0.0, position1->x(), 1.0e-5);
  EXPECT_NEAR(0.0, position1->y(), 1.0e-5);
  EXPECT_NEAR(0.0, orientation1->yaw(), 1.0e-5);
  EXPECT_NEAR(1.0, position2->x(), 1.0e-5);
  EXPECT_NEAR(0.0, position2->y(), 1.0e-5);
  EXPECT_NEAR(0.0, orientation2->yaw(), 1.0e-5);

  // Compute the marginal covariance for pose1
  {
    std::vector<std::pair<const double*, const double*> > covariance_blocks;
    covariance_blocks.emplace_back(position1->data(), position1->data());
    covariance_blocks.emplace_back(position1->data(), orientation1->data());
    covariance_blocks.emplace_back(orientation1->data(), orientation1->data());

    ceres::Covariance::Options cov_options;
    ceres::Covariance covariance(cov_options);
    covariance.Compute(covariance_blocks, &problem);

    std::vector<double> covariance_vector1(position1->size() * position1->size());
    covariance.GetCovarianceBlock(position1->data(), position1->data(), covariance_vector1.data());
    std::vector<double> covariance_vector2(position1->size() * orientation1->size());
    covariance.GetCovarianceBlock(position1->data(), orientation1->data(), covariance_vector2.data());
    std::vector<double> covariance_vector3(orientation1->size() * orientation1->size());
    covariance.GetCovarianceBlock(orientation1->data(), orientation1->data(), covariance_vector3.data());

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
    fuse_core::Matrix3d expected_covariance = cov1;
    EXPECT_MATRIX_NEAR(expected_covariance, actual_covariance, 1.0e-9);
  }
  // Compute the marginal covariance for pose2
  {
    std::vector<std::pair<const double*, const double*> > covariance_blocks;
    covariance_blocks.emplace_back(position2->data(), position2->data());
    covariance_blocks.emplace_back(position2->data(), orientation2->data());
    covariance_blocks.emplace_back(orientation2->data(), orientation2->data());

    ceres::Covariance::Options cov_options;
    ceres::Covariance covariance(cov_options);
    covariance.Compute(covariance_blocks, &problem);

    std::vector<double> covariance_vector1(position2->size() * position2->size());
    covariance.GetCovarianceBlock(position2->data(), position2->data(), covariance_vector1.data());
    std::vector<double> covariance_vector2(position2->size() * orientation2->size());
    covariance.GetCovarianceBlock(position2->data(), orientation2->data(), covariance_vector2.data());
    std::vector<double> covariance_vector3(orientation2->size() * orientation2->size());
    covariance.GetCovarianceBlock(orientation2->data(), orientation2->data(), covariance_vector3.data());

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
    fuse_core::Matrix3d expected_covariance;
    expected_covariance << 2.0, 0.0, 0.0, 0.0, 3.0, 1.0, 0.0, 1.0, 2.0;
    EXPECT_MATRIX_NEAR(expected_covariance, actual_covariance, 1.0e-9);
  }
}

TEST(RelativePose2DStampedConstraint, Serialization)
{
  // Construct a constraint
  Orientation2DStamped orientation1(rclcpp::Time(1234, 5678), fuse_core::uuid::generate("r5d4"));
  Position2DStamped position1(rclcpp::Time(1234, 5678), fuse_core::uuid::generate("r5d4"));
  Orientation2DStamped orientation2(rclcpp::Time(1235, 5678), fuse_core::uuid::generate("r5d4"));
  Position2DStamped position2(rclcpp::Time(1235, 5678), fuse_core::uuid::generate("r5d4"));
  fuse_core::Vector3d delta;
  delta << 1.0, 2.0, 3.0;
  fuse_core::Matrix3d cov;
  cov << 1.0, 0.1, 0.2, 0.1, 2.0, 0.3, 0.2, 0.3, 3.0;
  RelativePose2DStampedConstraint expected("test", position1, orientation1, position2, orientation2, delta, cov);

  // Serialize the constraint into an archive
  std::stringstream stream;
  {
    fuse_core::TextOutputArchive archive(stream);
    expected.serialize(archive);
  }

  // Deserialize a new constraint from that same stream
  RelativePose2DStampedConstraint actual;
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

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
