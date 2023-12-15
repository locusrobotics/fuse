/*
 * Software License Agreement (BSD License)
 *
 *  Author: Oscar Mendez
 *  Created on Fri Nov 17 2023
 *
 *  Copyright (c) 2023 Locus Robotics
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
#include <fuse_constraints/reprojection_error_constraint.h>
#include <fuse_core/eigen.h>
#include <fuse_core/eigen_gtest.h>
#include <fuse_core/serialization.h>
#include <fuse_core/uuid.h>
#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/point_3d_landmark.h>
#include <fuse_variables/point_3d_fixed_landmark.h>
#include <fuse_variables/position_3d_stamped.h>
#include <fuse_variables/pinhole_camera_fixed.h>

#include <ceres/covariance.h>
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <gtest/gtest.h>

#include <utility>
#include <vector>

using fuse_constraints::ReprojectionErrorConstraint;
using fuse_variables::Orientation3DStamped;
using fuse_variables::PinholeCameraFixed;
using fuse_variables::Point3DLandmark;
using fuse_variables::Point3DFixedLandmark;
using fuse_variables::Position3DStamped;

TEST(ReprojectionErrorConstraint, Constructor)
{
  // Construct a constraint just to make sure it compiles.
  Position3DStamped position_variable(ros::Time(1234, 5678), fuse_core::uuid::generate("walle"));
  Orientation3DStamped orientation_variable(ros::Time(1234, 5678), fuse_core::uuid::generate("walle"));
  Point3DLandmark point(0);
  PinholeCameraFixed calibration_variable(0);

  fuse_core::Vector2d mean;
  mean << 320.0, 240.0;  // Centre of a 640x480 camera

  // Assume Half a pixel Variance
  fuse_core::Matrix2d cov;
  cov << 0.25, 0.00,  // NOLINT
         0.00, 0.25;     // NOLINT

  EXPECT_NO_THROW(ReprojectionErrorConstraint constraint("test", position_variable, orientation_variable,
                                                         calibration_variable, mean, cov));
}

TEST(ReprojectionErrorConstraint, Covariance)
{
  // Verify the covariance <--> sqrt information conversions are correct
  Position3DStamped position_variable(ros::Time(1234, 5678), fuse_core::uuid::generate("mo"));
  Orientation3DStamped orientation_variable(ros::Time(1234, 5678), fuse_core::uuid::generate("mo"));
  Point3DLandmark point(0);
  PinholeCameraFixed calibration_variable(0);

  fuse_core::Vector2d mean;
  mean << 320.0, 240.0;  // Centre of a 640x480 camera

  // Assume Half a pixel Variance
  fuse_core::Matrix2d cov;
  cov << 0.25, 0.00,  // NOLINT
      0.00, 0.25;     // NOLINT

  ReprojectionErrorConstraint constraint("test", position_variable, orientation_variable, calibration_variable, mean,
                                         cov);

  // Define the expected matrices (used Octave to compute sqrt_info: 'chol(inv(A))')
  fuse_core::Matrix2d expected_sqrt_info;
  expected_sqrt_info << 2, 0,  // NOLINT
                      0, 2;    // NOLINT
  fuse_core::Matrix2d expected_cov = cov;

  // Compare
  EXPECT_MATRIX_NEAR(expected_cov, constraint.covariance(), 1.0e-9);
  EXPECT_MATRIX_NEAR(expected_sqrt_info, constraint.sqrtInformation(), 1.0e-9);
}

TEST(ReprojectionErrorConstraint, Optimization)
{
  // Optimize a single pose and single constraint, verify the expected value and covariance are generated.
  // Create the variables
  auto position_variable = Position3DStamped::make_shared(ros::Time(1, 0), fuse_core::uuid::generate("spra"));
  position_variable->x() = 1.5;
  position_variable->y() = -3.0;
  position_variable->z() = 10.0;

  auto orientation_variable = Orientation3DStamped::make_shared(ros::Time(1, 0), fuse_core::uuid::generate("spra"));
  orientation_variable->w() = 0.952;
  orientation_variable->x() = 0.038;
  orientation_variable->y() = -0.189;
  orientation_variable->z() = 0.239;

  auto calibration_variable = PinholeCameraFixed::make_shared(0);
  calibration_variable->fx() = 638.34478759765620;
  calibration_variable->fy() = 643.10717773437500;
  calibration_variable->cx() = 310.29060457226840;
  calibration_variable->cy() = 237.80861559081677;

  std::vector<Point3DFixedLandmark::SharedPtr> point_variables;
  point_variables.push_back(Point3DFixedLandmark::make_shared(0));
  point_variables.push_back(Point3DFixedLandmark::make_shared(1));
  point_variables.push_back(Point3DFixedLandmark::make_shared(2));
  point_variables.push_back(Point3DFixedLandmark::make_shared(3));
  point_variables[0]->array() = {-0.70710681, -1.0,  9.29289324};
  point_variables[1]->array() = {-0.70710681,  1.0,  9.29289324};
  point_variables[2]->array() = { 0.70710681, -1.0, 10.70710676};
  point_variables[3]->array() = { 0.70710681,  1.0, 10.70710676};

  // Create an observation
  std::vector<fuse_core::Vector2d> means(4);
  means[0] << 261.71822455, 168.60442225;
  means[1] << 261.71822455, 307.01280893;
  means[2] << 352.44745875, 177.74503448;
  means[3] << 352.44745875, 297.87219670;

  // Define Observation Covariance
  fuse_core::Matrix2d cov;
  cov <<  0.25, 0.00,  // NOLINT
          0.00, 0.25;  // NOLINT

  ceres::Problem::Options problem_options;
  problem_options.loss_function_ownership = fuse_core::Loss::Ownership;
  ceres::Problem problem(problem_options);

  // Build the problem
  problem.AddParameterBlock(position_variable->data(), position_variable->size(),
                            position_variable->localParameterization());
  problem.AddParameterBlock(orientation_variable->data(), orientation_variable->size(),
                            orientation_variable->localParameterization());
  problem.AddParameterBlock(calibration_variable->data(), calibration_variable->size(),
                            calibration_variable->localParameterization());

  if (calibration_variable->holdConstant())
  {
    problem.SetParameterBlockConstant(calibration_variable->data());
  }

  for (uint i = 0; i < point_variables.size(); i++)
  {
    auto constraint = ReprojectionErrorConstraint::make_shared("test",
                                                        *position_variable, *orientation_variable,
                                                        *calibration_variable, means[i], cov);

    problem.AddParameterBlock(point_variables[i]->data(), point_variables[i]->size(),
                        point_variables[i]->localParameterization());

    std::vector<double*> parameter_blocks;
    parameter_blocks.push_back(position_variable->data());
    parameter_blocks.push_back(orientation_variable->data());
    parameter_blocks.push_back(calibration_variable->data());
    parameter_blocks.push_back(point_variables[i]->data());

    problem.SetParameterBlockConstant(point_variables[i]->data());
    problem.AddResidualBlock(constraint->costFunction(), constraint->lossFunction(), parameter_blocks);
  }

  // Run the solver
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // Check
  EXPECT_NEAR(0.00, position_variable->x(), 1.0e-5);
  EXPECT_NEAR(0.00, position_variable->y(), 1.0e-5);
  EXPECT_NEAR(0.00, position_variable->z(), 1.0e-5);

  EXPECT_NEAR(1.0, orientation_variable->w(), 1.0e-3);
  EXPECT_NEAR(0.00, orientation_variable->x(), 1.0e-3);
  EXPECT_NEAR(0.00, orientation_variable->y(), 1.0e-3);
  EXPECT_NEAR(0.00, orientation_variable->z(), 1.0e-3);

  EXPECT_NEAR(638.34478759765620, calibration_variable->fx(), 1.0e-3);
  EXPECT_NEAR(643.10717773437500, calibration_variable->fy(), 1.0e-3);
  EXPECT_NEAR(310.29060457226840, calibration_variable->cx(), 1.0e-3);
  EXPECT_NEAR(237.80861559081677, calibration_variable->cy(), 1.0e-3);

  EXPECT_NEAR(-0.70710681, point_variables[0]->x(), 1.0e-3);
  EXPECT_NEAR(-1.0,        point_variables[0]->y(), 1.0e-3);
  EXPECT_NEAR(9.29289324,  point_variables[0]->z(), 1.0e-3);

  EXPECT_NEAR(-0.70710681, point_variables[1]->x(), 1.0e-3);
  EXPECT_NEAR(1.0,         point_variables[1]->y(), 1.0e-3);
  EXPECT_NEAR(9.29289324,  point_variables[1]->z(), 1.0e-3);

  EXPECT_NEAR(0.70710681,  point_variables[2]->x(), 1.0e-3);
  EXPECT_NEAR(-1.0,        point_variables[2]->y(), 1.0e-3);
  EXPECT_NEAR(10.70710676, point_variables[2]->z(), 1.0e-3);

  EXPECT_NEAR(0.70710681,  point_variables[3]->x(), 1.0e-3);
  EXPECT_NEAR(1.0,         point_variables[3]->y(), 1.0e-3);
  EXPECT_NEAR(10.70710676, point_variables[3]->z(), 1.0e-3);

  // // Compute the covariance
  // std::vector<std::pair<const double*, const double*> > covariance_blocks;
  // covariance_blocks.emplace_back(position_variable->data(), position_variable->data());
  // covariance_blocks.emplace_back(orientation_variable->data(), orientation_variable->data());
  // covariance_blocks.emplace_back(position_variable->data(), orientation_variable->data());

  // ceres::Covariance::Options cov_options;
  // ceres::Covariance covariance(cov_options);
  // covariance.Compute(covariance_blocks, &problem);
  // fuse_core::MatrixXd cov_pos_pos(position_variable->size(), position_variable->size());
  // covariance.GetCovarianceBlock(position_variable->data(), position_variable->data(), cov_pos_pos.data());

  // fuse_core::MatrixXd cov_or_or(orientation_variable->localSize(), orientation_variable->localSize());
  // covariance.GetCovarianceBlockInTangentSpace(
  //   orientation_variable->data(), orientation_variable->data(), cov_or_or.data());

  // fuse_core::MatrixXd cov_pos_or(position_variable->localSize(), orientation_variable->localSize());
  // covariance.GetCovarianceBlockInTangentSpace(
  //   position_variable->data(), orientation_variable->data(), cov_pos_or.data());

  // // Assemble the full covariance from the covariance blocks
  // fuse_core::Matrix6d actual_covariance;
  // actual_covariance << cov_pos_pos, cov_pos_or, cov_pos_or.transpose(), cov_or_or;

  // // Define the expected covariance
  // fuse_core::Matrix6d expected_covariance;
  // expected_covariance <<
  //   1.0, 0.1, 0.2, 0.3, 0.4, 0.5,
  //   0.1, 2.0, 0.6, 0.5, 0.4, 0.3,
  //   0.2, 0.6, 3.0, 0.2, 0.1, 0.2,
  //   0.3, 0.5, 0.2, 4.0, 0.3, 0.4,
  //   0.4, 0.4, 0.1, 0.3, 5.0, 0.5,
  //   0.5, 0.3, 0.2, 0.4, 0.5, 6.0;

  // EXPECT_MATRIX_NEAR(expected_covariance, actual_covariance, 1.0e-5);
}

TEST(ReprojectionErrorConstraint, Serialization)
{
  // Construct a constraint
  Position3DStamped position_variable(ros::Time(1234, 5678), fuse_core::uuid::generate("walle"));
  Orientation3DStamped orientation_variable(ros::Time(1234, 5678), fuse_core::uuid::generate("walle"));

  PinholeCameraFixed calibration_variable(0);
  calibration_variable.fx() = 638.34478759765620;
  calibration_variable.fy() = 643.10717773437500;
  calibration_variable.cx() = 310.29060457226840;
  calibration_variable.cy() = 237.80861559081677;

  fuse_core::Vector2d mean;
  mean << 261.71822455, 168.60442225;

  // Generated PD matrix using Octave: R = rand(6, 6); A = R * R' (use format long g to get the required precision)
  fuse_core::Matrix2d cov;
  cov << 0.25, 0.00,  //NOLINT
         0.00, 0.25;  //NOLINT


  ReprojectionErrorConstraint expected("test", position_variable,
                                        orientation_variable, calibration_variable,
                                        mean, cov);

  // Serialize the constraint into an archive
  std::stringstream stream;
  {
    fuse_core::TextOutputArchive archive(stream);
    expected.serialize(archive);
  }

  // Deserialize a new constraint from that same stream
  ReprojectionErrorConstraint actual;
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

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
