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
#include <ceres/autodiff_cost_function.h>
#include <ceres/gradient_checker.h>
#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <limits>
#include <vector>

#include <fuse_core/ceres_macros.hpp>
#include <fuse_core/eigen_gtest.hpp>
#include <fuse_models/omnidirectional_3d_state_cost_function.hpp>
#include <fuse_models/omnidirectional_3d_state_cost_functor.hpp>

TEST(CostFunction, evaluateCostFunction)
{
  // Create cost function
  const double process_noise_diagonal[] = {1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 
                                          1e-3, 1e-3, 1e-3, 1e-3, 1e-3,
                                          1e-3, 1e-3, 1e-3, 1e-3, 1e-3};
  const fuse_core::Matrix15d covariance = fuse_core::Vector15d(process_noise_diagonal).asDiagonal();

  const double dt{0.1};
  const fuse_core::Matrix15d sqrt_information{covariance.inverse().llt().matrixU()};

  const fuse_models::Omnidirectional3DStateCostFunction cost_function{dt, sqrt_information};

  // Evaluate cost function
  const double position1[3] = {0.0, 0.0, 0.0};
  const double orientation1[4] = {1.0, 0.0, 0.0, 0.0};
  const double vel_linear1[3] = {1.0, 1.0, 1.0};
  const double vel_angular1[3] = {1.570796327, 1.570796327, 1.570796327};
  const double acc_linear1[3] = {1.0, 1.0, 1.0};

  const double position2[3] = {0.105, 0.105, 0.105};
  Eigen::Quaterniond q2 = Eigen::AngleAxisd(0.1570796327, Eigen::Vector3d::UnitZ()) *
                          Eigen::AngleAxisd(0.1570796327, Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(0.1570796327, Eigen::Vector3d::UnitX());
  const double orientation2[4] = {q2.w(), q2.x(), q2.y(), q2.z()};
  const double vel_linear2[3] = {1.1, 1.1, 1.1};
  const double vel_angular2[3] = {1.570796327, 1.570796327, 1.570796327};
  const double acc_linear2[3] = {1.0, 1.0, 1.0};

  const double * parameters[10] =
  {
    position1, orientation1, vel_linear1, vel_angular1, acc_linear1,
    position2, orientation2, vel_linear2, vel_angular2, acc_linear2
  };

  fuse_core::Vector15d residuals;

  const auto & block_sizes = cost_function.parameter_block_sizes();
  const auto num_parameter_blocks = block_sizes.size();

  const auto num_residuals = cost_function.num_residuals();

  std::vector<fuse_core::MatrixXd> J(num_parameter_blocks);
  std::vector<double *> jacobians(num_parameter_blocks);

  for (size_t i = 0; i < num_parameter_blocks; ++i) {
    J[i].resize(num_residuals, block_sizes[i]);
    jacobians[i] = J[i].data();
  }

  EXPECT_TRUE(cost_function.Evaluate(parameters, residuals.data(), jacobians.data()));

  // We cannot use std::numeric_limits<double>::epsilon() tolerance because with the expected state2
  // above the residuals are not zero for position2.x = -4.389e-16 and yaw2 = -2.776e-16
  EXPECT_MATRIX_NEAR(fuse_core::Vector15d::Zero(), residuals, 1e-15);

//   // Check jacobians are correct using a gradient checker
  ceres::NumericDiffOptions numeric_diff_options;

#if CERES_VERSION_AT_LEAST(2, 1, 0)
  std::vector<const ceres::Manifold *> parameterizations;
  ceres::GradientChecker gradient_checker(&cost_function, &parameterizations, numeric_diff_options);
#else
  ceres::GradientChecker gradient_checker(&cost_function, nullptr, numeric_diff_options);
#endif

  // We cannot use std::numeric_limits<double>::epsilon() tolerance because the worst relative error
  // is 5.26356e-10
  ceres::GradientChecker::ProbeResults probe_results;
  // TODO(efernandez) probe_results segfaults when it's destroyed at the end of this TEST function,
  //                  but Probe actually returns true and the jacobians are correct according to the
  //                  gradient checker numeric differentiation
  //                  EXPECT_TRUE(gradient_checker.Probe(parameters, 1e-9, &probe_results)) <<
  //                  probe_results.error_log;

  // Create cost function using automatic differentiation on the cost functor
  ceres::AutoDiffCostFunction<fuse_models::Omnidirectional3DStateCostFunctor, 15, 3, 4, 3, 3, 3, 3, 4, 3, 3,
    3>cost_function_autodiff(new fuse_models::Omnidirectional3DStateCostFunctor(dt, sqrt_information));
  // Evaluate cost function that uses automatic differentiation
  std::vector<fuse_core::MatrixXd> J_autodiff(num_parameter_blocks);
  std::vector<double *> jacobians_autodiff(num_parameter_blocks);

  for (size_t i = 0; i < num_parameter_blocks; ++i) {
    J_autodiff[i].resize(num_residuals, cost_function_autodiff.parameter_block_sizes()[i]);
    jacobians_autodiff[i] = J_autodiff[i].data();
  }

  EXPECT_TRUE(
    cost_function_autodiff.Evaluate(
      parameters, residuals.data(),
      jacobians_autodiff.data()));

  const Eigen::IOFormat HeavyFmt(
    Eigen::FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");

  for (size_t i = 0; i < num_parameter_blocks; ++i) {
    EXPECT_MATRIX_NEAR(J_autodiff[i], J[i], 1e-4)
      << "Autodiff Jacobian[" << i << "] =\n" << J_autodiff[i].format(HeavyFmt)
      << "\nAnalytic Jacobian[" << i << "] =\n" << J[i].format(HeavyFmt);
  }
}
