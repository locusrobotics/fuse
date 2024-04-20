/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Clearpath Robotics
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
#ifndef FUSE_CONSTRAINTS__TEST_COST_FUNTION_GTEST_HPP_  // NOLINT{build/header_guard}
#define FUSE_CONSTRAINTS__TEST_COST_FUNTION_GTEST_HPP_  // NOLINT{build/header_guard}

#include <gtest/gtest.h>

#include <memory>
#include <vector>
#include <Eigen/Core>

/**
 * @brief A helper function to compare a expected and actual cost function.
 *
 * This helper function is copied and slightly adapted from:
 *
 * https://github.com/ceres-solver/ceres-
 * solver/blob/27b71795/internal/ceres/cost_function_to_functor_test.cc#L46-L119
 *
 * @param[in] cost_function The expected cost function
 * @param[in] actual_cost_function The actual cost function
 */
static void ExpectCostFunctionsAreEqual(
  const ceres::CostFunction & cost_function,
  const ceres::CostFunction & actual_cost_function)
{
  EXPECT_EQ(cost_function.num_residuals(), actual_cost_function.num_residuals());
  const size_t num_residuals = cost_function.num_residuals();
  const std::vector<int32_t> & parameter_block_sizes = cost_function.parameter_block_sizes();
  const std::vector<int32_t> & actual_parameter_block_sizes =
    actual_cost_function.parameter_block_sizes();
  EXPECT_EQ(parameter_block_sizes.size(), actual_parameter_block_sizes.size());

  size_t num_parameters = 0;
  for (size_t i = 0; i < parameter_block_sizes.size(); ++i) {
    EXPECT_EQ(parameter_block_sizes[i], actual_parameter_block_sizes[i]);
    num_parameters += parameter_block_sizes[i];
  }

  std::unique_ptr<double[]> parameters(new double[num_parameters]);
  if ((num_parameters == 7) && (parameter_block_sizes[0] == 3) &&
    (parameter_block_sizes[1] == 4))
  {
    // Special case for parameters[1] as quaternion
    for (size_t i = 0; i < 3; i++) {
      parameters[i] = static_cast<double>(i) + 1.0;
    }
    Eigen::Quaterniond q = Eigen::Quaterniond::UnitRandom();
    parameters[3] = q.w();
    parameters[4] = q.x();
    parameters[5] = q.y();
    parameters[6] = q.z();
  } else {
    for (size_t i = 0; i < num_parameters; ++i) {
      parameters[i] = static_cast<double>(i) + 1.0;
    }
  }

  std::unique_ptr<double[]> residuals(new double[num_residuals]);
  std::unique_ptr<double[]> jacobians(new double[num_parameters * num_residuals]);

  std::unique_ptr<double[]> actual_residuals(new double[num_residuals]);
  std::unique_ptr<double[]> actual_jacobians(new double[num_parameters * num_residuals]);

  std::unique_ptr<double *[]> parameter_blocks(new double *[parameter_block_sizes.size()]);
  std::unique_ptr<double *[]> jacobian_blocks(new double *[parameter_block_sizes.size()]);
  std::unique_ptr<double *[]> actual_jacobian_blocks(new double *[parameter_block_sizes.size()]);

  num_parameters = 0;
  for (size_t i = 0; i < parameter_block_sizes.size(); ++i) {
    parameter_blocks[i] = parameters.get() + num_parameters;
    jacobian_blocks[i] = jacobians.get() + num_parameters * num_residuals;
    actual_jacobian_blocks[i] = actual_jacobians.get() + num_parameters * num_residuals;
    num_parameters += parameter_block_sizes[i];
  }

  EXPECT_TRUE(cost_function.Evaluate(parameter_blocks.get(), residuals.get(), nullptr));
  EXPECT_TRUE(
    actual_cost_function.Evaluate(
      parameter_blocks.get(), actual_residuals.get(),
      nullptr));
  for (size_t i = 0; i < num_residuals; ++i) {
    EXPECT_DOUBLE_EQ(residuals[i], actual_residuals[i]) << "residual id: " << i;
  }

  EXPECT_TRUE(
    cost_function.Evaluate(
      parameter_blocks.get(), residuals.get(),
      jacobian_blocks.get()));
  EXPECT_TRUE(
    actual_cost_function.Evaluate(
      parameter_blocks.get(), actual_residuals.get(),
      actual_jacobian_blocks.get()));
  for (size_t i = 0; i < num_residuals; ++i) {
    EXPECT_DOUBLE_EQ(residuals[i], actual_residuals[i]) << "residual : " << i;
  }

  for (size_t i = 0; i < num_residuals * num_parameters; ++i) {
    EXPECT_DOUBLE_EQ(jacobians[i], actual_jacobians[i])
      << "jacobian : " << i << " " << jacobians[i] << " " << actual_jacobians[i];
  }
}

#endif  // FUSE_CONSTRAINTS__TEST_COST_FUNTION_GTEST_HPP_  // NOLINT{build/header_guard}
