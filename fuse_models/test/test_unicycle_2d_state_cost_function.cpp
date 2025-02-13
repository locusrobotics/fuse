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
#include <fuse_models/unicycle_2d_state_cost_function.hpp>
#include <fuse_models/unicycle_2d_state_cost_functor.hpp>

TEST(CostFunction, evaluateCostFunction)
{
  // Create cost function
  const double process_noise_diagonal[] = {1e-3, 1e-3, 1e-2, 1e-6, 1e-6, 1e-4, 1e-9, 1e-9};
  const fuse_core::Matrix8d covariance = fuse_core::Vector8d(process_noise_diagonal).asDiagonal();

  const double dt{0.1};
  const fuse_core::Matrix8d sqrt_information{covariance.inverse().llt().matrixU()};

  const fuse_models::Unicycle2DStateCostFunction cost_function{dt, sqrt_information};

  // Evaluate cost function
  const double position1[] = {0.0, 0.0};
  const double yaw1[] = {0.0};
  const double vel_linear1[] = {1.0, 0.0};
  const double vel_yaw1[] = {1.570796327};
  const double acc_linear1[] = {1.0, 0.0};

  const double position2[] = {0.105, 0.0};
  const double yaw2[] = {0.1570796327};
  const double vel_linear2[] = {1.1, 0.0};
  const double vel_yaw2[] = {1.570796327};
  const double acc_linear2[] = {1.0, 0.0};

  const double * parameters[] =
  {
    position1, yaw1, vel_linear1, vel_yaw1, acc_linear1,
    position2, yaw2, vel_linear2, vel_yaw2, acc_linear2
  };

  fuse_core::Vector8d residuals;

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
  EXPECT_MATRIX_NEAR(fuse_core::Vector8d::Zero(), residuals, 1e-15);

  // Check jacobians are correct using a gradient checker
  ceres::NumericDiffOptions numeric_diff_options;
  ceres::GradientChecker gradient_checker(&cost_function, nullptr, numeric_diff_options);

  // We cannot use std::numeric_limits<double>::epsilon() tolerance because the worst relative error
  // is 5.26356e-10
  ceres::GradientChecker::ProbeResults probe_results;
  // TODO(efernandez) probe_results segfaults when it's destroyed at the end of this TEST function,
  //                  but Probe actually returns true and the jacobians are correct according to the
  //                  gradient checker numeric differentiation
  //                  EXPECT_TRUE(gradient_checker.Probe(parameters, 1e-9, &probe_results)) <<
  //                  probe_results.error_log;

  // Create cost function using automatic differentiation on the cost functor
  ceres::AutoDiffCostFunction<fuse_models::Unicycle2DStateCostFunctor, 8, 2, 1, 2, 1, 2, 2, 1, 2, 1,
    2>
  cost_function_autodiff(new fuse_models::Unicycle2DStateCostFunctor(dt, sqrt_information));

  // Evaluate cost function that uses automatic differentiation
  std::vector<fuse_core::MatrixXd> J_autodiff(num_parameter_blocks);
  std::vector<double *> jacobians_autodiff(num_parameter_blocks);

  for (size_t i = 0; i < num_parameter_blocks; ++i) {
    J_autodiff[i].resize(num_residuals, block_sizes[i]);
    jacobians_autodiff[i] = J_autodiff[i].data();
  }

  EXPECT_TRUE(
    cost_function_autodiff.Evaluate(
      parameters, residuals.data(),
      jacobians_autodiff.data()));

  const Eigen::IOFormat HeavyFmt(
    Eigen::FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");

  for (size_t i = 0; i < num_parameter_blocks; ++i) {
    EXPECT_MATRIX_NEAR(J_autodiff[i], J[i], std::numeric_limits<double>::epsilon())
      << "Autodiff Jacobian[" << i << "] =\n" << J_autodiff[i].format(HeavyFmt)
      << "\nAnalytic Jacobian[" << i << "] =\n" << J[i].format(HeavyFmt);
  }
}
