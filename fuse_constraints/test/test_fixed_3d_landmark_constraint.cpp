


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
#include <fuse_constraints/fixed_3d_landmark_constraint.h>
#include <fuse_core/eigen.h>
#include <fuse_core/eigen_gtest.h>
#include <fuse_core/serialization.h>
#include <fuse_core/uuid.h>
#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>
#include <fuse_variables/pinhole_camera_fixed.h>

#include <ceres/covariance.h>
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <gtest/gtest.h>

#include <utility>
#include <vector>

using fuse_variables::Orientation3DStamped;
using fuse_variables::Position3DStamped;
using fuse_variables::PinholeCameraFixed;
using fuse_constraints::Fixed3DLandmarkConstraint;


TEST(Fixed3DLandmarkConstraint, Constructor)
{
  // Construct a constraint just to make sure it compiles.
  Position3DStamped position_variable(ros::Time(1234, 5678), fuse_core::uuid::generate("walle"));
  Orientation3DStamped orientation_variable(ros::Time(1234, 5678), fuse_core::uuid::generate("walle"));
  PinholeCameraFixed calibration_variable(0);

  fuse_core::Vector1d marker_size;
  marker_size << 1.0;

  fuse_core::Vector7d mean;
  mean << 1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0;

  // Generated PD matrix using Octave: R = rand(6, 6); A = R * R' (use format long g to get the required precision)
  fuse_core::Matrix6d cov;
  cov << 2.0847236144069, 1.10752598122138, 1.02943174290333, 1.96120532313878, 1.96735470687891,  1.5153042667951,     // NOLINT
         1.10752598122138, 1.39176289439125, 0.643422499737987, 1.35471905449013, 1.18353784377297, 1.28979625492894,   // NOLINT
         1.02943174290333, 0.643422499737987, 1.26701658550187, 1.23641771365403, 1.55169301761377, 1.34706781598061,   // NOLINT
         1.96120532313878, 1.35471905449013, 1.23641771365403, 2.39750866789926, 2.06887486311147, 2.04350823837035,    // NOLINT
         1.96735470687891, 1.18353784377297, 1.55169301761377, 2.06887486311147,   2.503913946461, 1.73844731158092,    // NOLINT
         1.5153042667951, 1.28979625492894, 1.34706781598061, 2.04350823837035, 1.73844731158092, 2.15326088526198;     // NOLINT

  // 2D observations (arbitrary)
  Eigen::Matrix<double, 4, 2, Eigen::RowMajor> obs;
  obs <<  320, 240,        
          320, 240,        
          320, 240,        
          320, 240;  

  EXPECT_NO_THROW(
    Fixed3DLandmarkConstraint constraint("test", position_variable, orientation_variable, calibration_variable, marker_size, obs, mean, cov));
}

TEST(Fixed3DLandmarkConstraint, Covariance)
{
  // Verify the covariance <--> sqrt information conversions are correct
  Position3DStamped position_variable(ros::Time(1234, 5678), fuse_core::uuid::generate("mo"));
  Orientation3DStamped orientation_variable(ros::Time(1234, 5678), fuse_core::uuid::generate("mo"));
  PinholeCameraFixed calibration_variable(0);

  fuse_core::Vector1d marker_size;
  marker_size << 1.0;

  fuse_core::Vector7d mean;
  mean << 1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0;

  // Generated PD matrix using Octiave: R = rand(6, 6); A = R * R' (use format long g to get the required precision)
  fuse_core::Matrix6d cov;
  cov << 2.0847236144069, 1.10752598122138, 1.02943174290333, 1.96120532313878, 1.96735470687891,  1.5153042667951,     // NOLINT
         1.10752598122138, 1.39176289439125, 0.643422499737987, 1.35471905449013, 1.18353784377297, 1.28979625492894,   // NOLINT
         1.02943174290333, 0.643422499737987, 1.26701658550187, 1.23641771365403, 1.55169301761377, 1.34706781598061,   // NOLINT
         1.96120532313878, 1.35471905449013, 1.23641771365403, 2.39750866789926, 2.06887486311147, 2.04350823837035,    // NOLINT
         1.96735470687891, 1.18353784377297, 1.55169301761377, 2.06887486311147,   2.503913946461, 1.73844731158092,    // NOLINT
         1.5153042667951, 1.28979625492894, 1.34706781598061, 2.04350823837035, 1.73844731158092, 2.15326088526198;     // NOLINT

  // 2D observations (arbitrary)
  Eigen::Matrix<double, 4, 2, Eigen::RowMajor> obs;
  obs <<  320, 240,        
          320, 240,        
          320, 240,        
          320, 240;

  Fixed3DLandmarkConstraint constraint("test", position_variable, orientation_variable, calibration_variable, marker_size, obs, mean, cov);

  // Define the expected matrices (used Octave to compute sqrt_info: 'chol(inv(A))')
  fuse_core::Matrix6d expected_sqrt_info;
  expected_sqrt_info << 2.12658752275893, 1.20265444927878, 4.71225672571804, 1.43587520991272, -4.12764062992821, -3.19509486240291,    // NOLINT
                        0.0,              2.41958656956248, 5.93151964116945, 3.72535320852517, -4.23326858606213, -5.27776664777548,    // NOLINT
                        0.0,              0.0,              3.82674686590005, 2.80341171946161, -2.68168478581452, -2.8894384435255,     // NOLINT
                        0.0,              0.0,              0.0,              1.83006791372784, -0.696917410192509, -1.17412835464633,   // NOLINT
                        0.0,              0.0,              0.0,              0.0,               0.953302832761324, -0.769654414882847,  // NOLINT
                        0.0,              0.0,              0.0,              0.0,               0.0,                0.681477739760948;  // NOLINT
  fuse_core::Matrix6d expected_cov = cov;

  // Compare
  EXPECT_MATRIX_NEAR(expected_cov, constraint.covariance(), 1.0e-9);
  EXPECT_MATRIX_NEAR(expected_sqrt_info, constraint.sqrtInformation(), 1.0e-9);
}

TEST(Fixed3DLandmarkConstraint, Optimization)
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

  // Define Marker Size
  fuse_core::Vector1d marker_size;
  marker_size << 1.0;

  // Create an fixed position for the marker.
  fuse_core::Vector7d mean;
  mean << 0, 0, 10, 0.9238795, 0, -0.3826834, 0;

  // // Its 3D points:
  // Eigen::Matrix<T, 4, 4, Eigen::RowMajor> X;
  // X << T(-1.0), T(-1.0), T(0.0), T(1.0),  // NOLINT
  //       T(-1.0), T( 1.0), T(0.0), T(1.0),  // NOLINT
  //       T( 1.0), T(-1.0), T(0.0), T(1.0),  // NOLINT
  //       T( 1.0), T( 1.0), T(0.0), T(1.0);  // NOLINT
  
  // And observations...
  Eigen::Matrix<double, 4, 2, Eigen::RowMajor> obs;
  obs <<  261.71822455, 168.60442225,  // NOLINT
          261.71822455, 307.01280893,  // NOLINT
          352.44745875, 177.74503448,  // NOLINT
          352.44745875, 297.87219670;  // NOLINT

  // Define Pose Covariance
  fuse_core::Matrix6d cov;
  cov << 1.0, 0.1, 0.2, 0.3, 0.4, 0.5,  // NOLINT
         0.1, 2.0, 0.6, 0.5, 0.4, 0.3,  // NOLINT
         0.2, 0.6, 3.0, 0.2, 0.1, 0.2,  // NOLINT
         0.3, 0.5, 0.2, 4.0, 0.3, 0.4,  // NOLINT
         0.4, 0.4, 0.1, 0.3, 5.0, 0.5,  // NOLINT
         0.5, 0.3, 0.2, 0.4, 0.5, 6.0;  // NOLINT

  auto constraint = Fixed3DLandmarkConstraint::make_shared(
    "test",
    *position_variable,
    *orientation_variable,
    *calibration_variable,
    marker_size, 
    obs,
    mean,
    cov);

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
  problem.AddParameterBlock(
    calibration_variable->data(),
    calibration_variable->size(),
    calibration_variable->localParameterization());

  std::vector<double*> parameter_blocks;
  parameter_blocks.push_back(position_variable->data());
  parameter_blocks.push_back(orientation_variable->data());
  parameter_blocks.push_back(calibration_variable->data());

  if (calibration_variable->holdConstant())
  {
    problem.SetParameterBlockConstant(calibration_variable->data());
  }
  

  problem.AddResidualBlock(
    constraint->costFunction(),
    constraint->lossFunction(),
    parameter_blocks);

  // Run the solver
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // Check
  EXPECT_NEAR(0.0, position_variable->x(), 1.0e-5);
  EXPECT_NEAR(0.0, position_variable->y(), 1.0e-5);
  EXPECT_NEAR(0.0, position_variable->z(), 1.0e-5);
  
  EXPECT_NEAR(1.0, orientation_variable->w(), 1.0e-3);
  EXPECT_NEAR(0.0, orientation_variable->x(), 1.0e-3);
  EXPECT_NEAR(0.0, orientation_variable->y(), 1.0e-3);
  EXPECT_NEAR(0.0, orientation_variable->z(), 1.0e-3);

  EXPECT_NEAR(638.34478759765620, calibration_variable->fx(), 1.0e-3);
  EXPECT_NEAR(643.10717773437500, calibration_variable->fy(), 1.0e-3);
  EXPECT_NEAR(310.29060457226840, calibration_variable->cx(), 1.0e-3);
  EXPECT_NEAR(237.80861559081677, calibration_variable->cy(), 1.0e-3);

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


TEST(Fixed3DLandmarkConstraint, OptimizationScaledMarker)
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

  // Define Marker Size
  fuse_core::Vector1d marker_size;
  marker_size <<0.25;

  // Create an fixed position for the marker.
  fuse_core::Vector7d mean;
  mean << 0, 0, 10, 0.9238795, 0, -0.3826834, 0;
  
  // And observations...
  Eigen::Matrix<double, 4, 2, Eigen::RowMajor> obs;
  obs <<  298.8030834636202, 221.44160554498040,
          298.8030834636202, 254.17562563665314,
          321.3790354517349, 222.01021505859384,
          321.3790354517349, 253.60701612303970;

  // Define Pose Covariance
  fuse_core::Matrix6d cov;
  cov << 1.0, 0.1, 0.2, 0.3, 0.4, 0.5,  // NOLINT
         0.1, 2.0, 0.6, 0.5, 0.4, 0.3,  // NOLINT
         0.2, 0.6, 3.0, 0.2, 0.1, 0.2,  // NOLINT
         0.3, 0.5, 0.2, 4.0, 0.3, 0.4,  // NOLINT
         0.4, 0.4, 0.1, 0.3, 5.0, 0.5,  // NOLINT
         0.5, 0.3, 0.2, 0.4, 0.5, 6.0;  // NOLINT

  auto constraint = Fixed3DLandmarkConstraint::make_shared(
    "test",
    *position_variable,
    *orientation_variable,
    *calibration_variable,
    marker_size, 
    obs,
    mean,
    cov);

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
  problem.AddParameterBlock(
    calibration_variable->data(),
    calibration_variable->size(),
    calibration_variable->localParameterization());

  std::vector<double*> parameter_blocks;
  parameter_blocks.push_back(position_variable->data());
  parameter_blocks.push_back(orientation_variable->data());
  parameter_blocks.push_back(calibration_variable->data());

  if (calibration_variable->holdConstant())
  {
    problem.SetParameterBlockConstant(calibration_variable->data());
  }
  

  problem.AddResidualBlock(
    constraint->costFunction(),
    constraint->lossFunction(),
    parameter_blocks);

  // Run the solver
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // Check
  EXPECT_NEAR(0.0, position_variable->x(), 1.0e-5);
  EXPECT_NEAR(0.0, position_variable->y(), 1.0e-5);
  EXPECT_NEAR(0.0, position_variable->z(), 1.0e-5);
  
  EXPECT_NEAR(1.0, orientation_variable->w(), 1.0e-3);
  EXPECT_NEAR(0.0, orientation_variable->x(), 1.0e-3);
  EXPECT_NEAR(0.0, orientation_variable->y(), 1.0e-3);
  EXPECT_NEAR(0.0, orientation_variable->z(), 1.0e-3);

  EXPECT_NEAR(638.34478759765620, calibration_variable->fx(), 1.0e-3);
  EXPECT_NEAR(643.10717773437500, calibration_variable->fy(), 1.0e-3);
  EXPECT_NEAR(310.29060457226840, calibration_variable->cx(), 1.0e-3);
  EXPECT_NEAR(237.80861559081677, calibration_variable->cy(), 1.0e-3);

}

TEST(Fixed3DLandmarkConstraint, MultiViewOptimization)
{
  // Optimize a single pose and single constraint, verify the expected value and covariance are generated.
  // Create the variables

  auto calibration_variable = PinholeCameraFixed::make_shared(0);
  calibration_variable->fx() = 638.34478759765620;
  calibration_variable->fy() = 643.10717773437500;
  calibration_variable->cx() = 310.29060457226840;
  calibration_variable->cy() = 237.80861559081677;

  // Define Marker Size
  fuse_core::Vector1d marker_size;
  marker_size << 1.0;


  // Build the problem
  ceres::Problem::Options problem_options;
  problem_options.loss_function_ownership = fuse_core::Loss::Ownership;
  ceres::Problem problem(problem_options);
  
  std::vector<Eigen::Matrix<double, 4, 2, Eigen::RowMajor> > obs(5);
  // obs[0] << 193.02650778349710, 99.400228903584630,  // NOLINT
  //           193.02650778349710, 237.80861559081677,  // NOLINT
  //           292.82866541025334, 117.68145337097695,  // NOLINT
  //           292.82866541025334, 237.80861559081677;  // NOLINT  
  // obs[1] << 261.71822454589966, 168.60442224720070,  // NOLINT
  //           261.71822454589966, 307.01280893443280,  // NOLINT            
  //           352.44745875204165, 177.74503448089686,  // NOLINT
  //           352.44745875204165, 297.87219670073670;  // NOLINT  
  // obs[2] << 330.40994130830210, 237.80861559081677,  // NOLINT
  //           330.40994130830210, 376.21700227804890,  // NOLINT
  //           412.06625209383000, 237.80861559081677,  // NOLINT
  //           412.06625209383000, 357.93577781065660;  // NOLINT

obs[0]<<124.33479102109460, 30.196035559968568,  // NOLINT
        124.33479102109460, 168.60442224720070,  // NOLINT
        233.20987206846505, 57.617872261057050,  // NOLINT
        233.20987206846505, 177.74503448089686;  // NOLINT
      
obs[1]<<193.02650778349710, 99.400228903584630,  // NOLINT
        193.02650778349710, 237.80861559081677,  // NOLINT
        292.82866541025334, 117.68145337097695,  // NOLINT
        292.82866541025334, 237.80861559081677;  // NOLINT
 
obs[2]<<373.42853736463960, 168.46718090279768,  // NOLINT
        373.42853736463960, 307.15005027883580,  // NOLINT
        466.82773058187524, 176.09986812497370,  // NOLINT
        466.82773058187524, 299.51736305665980;  // NOLINT
 
obs[3]<<442.25647916056020, 237.80861559081674,  // NOLINT
        442.25647916056020, 376.49148496685490,  // NOLINT
        528.07950735845330, 237.80861559081677,  // NOLINT
        528.07950735845330, 361.22611052250290;  // NOLINT
 
obs[4]<<511.08442095648064, 307.15005027883580,  // NOLINT
        511.08442095648064, 445.83291965487400,  // NOLINT
        589.33128413503120, 299.51736305665980,  // NOLINT
        589.33128413503120, 422.93485798834600;  // NOLINT
 

  // Create N Constraints
  uint N = 5;

  std::vector<Position3DStamped::SharedPtr> position_vars(N);
  std::vector<Orientation3DStamped::SharedPtr> orientation_vars(N);

  for (uint i =0; i < N; i++){
    
    position_vars[i] = Position3DStamped::make_shared(ros::Time(1, 0), fuse_core::uuid::generate("spra"));
    position_vars[i]->x() = 0.0;
    position_vars[i]->y() = 0.0;
    position_vars[i]->z() = 0.0;

    orientation_vars[i] = Orientation3DStamped::make_shared(ros::Time(1, 0), fuse_core::uuid::generate("spra"));
    orientation_vars[i]->w() = 0.952;
    orientation_vars[i]->x() = 0.038;
    orientation_vars[i]->y() = -0.189;
    orientation_vars[i]->z() = 0.239;
    

    // position_vars[i] = position_variable;
    // orientation_vars[i] = orientation_variable;
    

    problem.AddParameterBlock(
    position_vars[i]->data(),
    position_vars[i]->size(),
    position_vars[i]->localParameterization());
    problem.AddParameterBlock(
    orientation_vars[i]->data(),
    orientation_vars[i]->size(),
    orientation_vars[i]->localParameterization());
    problem.AddParameterBlock(
    calibration_variable->data(),
    calibration_variable->size(),
    calibration_variable->localParameterization());
    

    std::vector<double*> parameter_blocks;
    parameter_blocks.push_back(position_vars[i]->data());
    parameter_blocks.push_back(orientation_vars[i]->data());
    parameter_blocks.push_back(calibration_variable->data());
    

    // Create an fixed position for the marker.
    fuse_core::Vector7d mean;
    mean << 0, 0, 10, 0.9238795, 0, -0.3826834, 0;

    fuse_core::Matrix6d cov;
    cov << 1.0, 0.1, 0.2, 0.3, 0.4, 0.5,  // NOLINT
          0.1, 2.0, 0.6, 0.5, 0.4, 0.3,  // NOLINT
          0.2, 0.6, 3.0, 0.2, 0.1, 0.2,  // NOLINT
          0.3, 0.5, 0.2, 4.0, 0.3, 0.4,  // NOLINT
          0.4, 0.4, 0.1, 0.3, 5.0, 0.5,  // NOLINT
          0.5, 0.3, 0.2, 0.4, 0.5, 6.0;  // NOLINT

    //   // And observations...
    // Eigen::Matrix<double, 4, 2, Eigen::RowMajor> obs;
    // obs <<  261.71822455, 168.60442225,        
    //         261.71822455, 307.01280893,        
    //         352.44745875, 177.74503448,        
    //         352.44745875, 297.87219670;  
    

    auto constraint = Fixed3DLandmarkConstraint::make_shared(
      "test",
      *position_vars[i],
      *orientation_vars[i],
      *calibration_variable,
      marker_size, 
      obs[i],
      mean,
      cov);
    
    
    problem.AddResidualBlock(
      constraint->costFunction(),
      constraint->lossFunction(),
      parameter_blocks);
    

}

  if (calibration_variable->holdConstant())
  {
    problem.SetParameterBlockConstant(calibration_variable->data());
  }

  // Run the solver
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

    // Check
  EXPECT_NEAR(-2.0, position_vars[0]->x(), 1.0e-5);
  EXPECT_NEAR(-2.0, position_vars[0]->y(), 1.0e-5);
  EXPECT_NEAR(0.0, position_vars[0]->z(), 1.0e-5);
  
  EXPECT_NEAR(1.0, orientation_vars[0]->w(), 1.0e-3);
  EXPECT_NEAR(0.0, orientation_vars[0]->x(), 1.0e-3);
  EXPECT_NEAR(0.0, orientation_vars[0]->y(), 1.0e-3);
  EXPECT_NEAR(0.0, orientation_vars[0]->z(), 1.0e-3);

  EXPECT_NEAR(-1.0, position_vars[1]->x(), 1.0e-5);
  EXPECT_NEAR(-1.0, position_vars[1]->y(), 1.0e-5);
  EXPECT_NEAR(0.0, position_vars[1]->z(), 1.0e-5);
  
  EXPECT_NEAR(1.0, orientation_vars[1]->w(), 1.0e-3);
  EXPECT_NEAR(0.0, orientation_vars[1]->x(), 1.0e-3);
  EXPECT_NEAR(0.0, orientation_vars[1]->y(), 1.0e-3);
  EXPECT_NEAR(0.0, orientation_vars[1]->z(), 1.0e-3);

  EXPECT_NEAR(0.0, position_vars[2]->x(), 1.0e-5);
  EXPECT_NEAR(0.0, position_vars[2]->y(), 1.0e-5);
  EXPECT_NEAR(0.0, position_vars[2]->z(), 1.0e-5);
  
  EXPECT_NEAR(0.9961947, orientation_vars[2]->w(), 1.0e-3);
  EXPECT_NEAR(0.0,       orientation_vars[2]->x(), 1.0e-3);
  EXPECT_NEAR(0.0871557, orientation_vars[2]->y(), 1.0e-3);
  EXPECT_NEAR(0.0,       orientation_vars[2]->z(), 1.0e-3);

  EXPECT_NEAR(1.0, position_vars[3]->x(), 1.0e-5);
  EXPECT_NEAR(1.0, position_vars[3]->y(), 1.0e-5);
  EXPECT_NEAR(0.0, position_vars[3]->z(), 1.0e-5);
  
  EXPECT_NEAR(0.9961947, orientation_vars[3]->w(), 1.0e-3);
  EXPECT_NEAR(0.0,       orientation_vars[3]->x(), 1.0e-3);
  EXPECT_NEAR(0.0871557, orientation_vars[3]->y(), 1.0e-3);
  EXPECT_NEAR(0.0,       orientation_vars[3]->z(), 1.0e-3);

  EXPECT_NEAR(2.0, position_vars[4]->x(), 1.0e-5);
  EXPECT_NEAR(2.0, position_vars[4]->y(), 1.0e-5);
  EXPECT_NEAR(0.0, position_vars[4]->z(), 1.0e-5);
  
  EXPECT_NEAR(0.9961947, orientation_vars[4]->w(), 1.0e-3);
  EXPECT_NEAR(0.0,       orientation_vars[4]->x(), 1.0e-3);
  EXPECT_NEAR(0.0871557, orientation_vars[4]->y(), 1.0e-3);
  EXPECT_NEAR(0.0,       orientation_vars[4]->z(), 1.0e-3);

  EXPECT_NEAR(638.34478759765620, calibration_variable->fx(), 1.0e-3);
  EXPECT_NEAR(643.10717773437500, calibration_variable->fy(), 1.0e-3);
  EXPECT_NEAR(310.29060457226840, calibration_variable->cx(), 1.0e-3);
  EXPECT_NEAR(237.80861559081677, calibration_variable->cy(), 1.0e-3);

}

TEST(Fixed3DLandmarkConstraint, Serialization)
{
  // Construct a constraint
  Position3DStamped position_variable(ros::Time(1234, 5678), fuse_core::uuid::generate("walle"));
  Orientation3DStamped orientation_variable(ros::Time(1234, 5678), fuse_core::uuid::generate("walle"));

  PinholeCameraFixed calibration_variable(0);
  calibration_variable.fx() = 638.34478759765620;
  calibration_variable.fy() = 643.10717773437500;
  calibration_variable.cx() = 310.29060457226840;
  calibration_variable.cy() = 237.80861559081677;

  fuse_core::Vector1d marker_size;
  marker_size << 1.0;

  fuse_core::Vector7d mean;
  mean << 1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0;

  // Generated PD matrix using Octave: R = rand(6, 6); A = R * R' (use format long g to get the required precision)
  fuse_core::Matrix6d cov;
  cov << 2.0847236144069, 1.10752598122138, 1.02943174290333, 1.96120532313878, 1.96735470687891,  1.5153042667951,
         1.10752598122138, 1.39176289439125, 0.643422499737987, 1.35471905449013, 1.18353784377297, 1.28979625492894,
         1.02943174290333, 0.643422499737987, 1.26701658550187, 1.23641771365403, 1.55169301761377, 1.34706781598061,
         1.96120532313878, 1.35471905449013, 1.23641771365403, 2.39750866789926, 2.06887486311147, 2.04350823837035,
         1.96735470687891, 1.18353784377297, 1.55169301761377, 2.06887486311147,   2.503913946461, 1.73844731158092,
         1.5153042667951, 1.28979625492894, 1.34706781598061, 2.04350823837035, 1.73844731158092, 2.15326088526198;

    // And observations...
  Eigen::Matrix<double, 4, 2, Eigen::RowMajor> obs;
  obs <<  261.71822455, 168.60442225,        
          261.71822455, 307.01280893,        
          352.44745875, 177.74503448,        
          352.44745875, 297.87219670;  

  Fixed3DLandmarkConstraint expected("test", position_variable, orientation_variable, calibration_variable, marker_size, obs, mean, cov);

  // Serialize the constraint into an archive
  std::stringstream stream;
  {
    fuse_core::TextOutputArchive archive(stream);
    expected.serialize(archive);
  }

  // Deserialize a new constraint from that same stream
  Fixed3DLandmarkConstraint actual;
  {
    fuse_core::TextInputArchive archive(stream);
    actual.deserialize(archive);
  }

  // Compare
  EXPECT_EQ(expected.uuid(), actual.uuid());
  EXPECT_EQ(expected.variables(), actual.variables());
  EXPECT_MATRIX_EQ(expected.mean(), actual.mean());
  EXPECT_MATRIX_EQ(expected.marker_size(), actual.marker_size());
  EXPECT_MATRIX_EQ(expected.sqrtInformation(), actual.sqrtInformation());
  EXPECT_MATRIX_EQ(expected.observations(), actual.observations());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
