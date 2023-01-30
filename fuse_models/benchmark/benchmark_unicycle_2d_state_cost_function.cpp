/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Clearpath Robotics
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
#include <benchmark/benchmark.h>
#include <ceres/autodiff_cost_function.h>
#include <Eigen/Dense>

#include <vector>

#include <fuse_models/unicycle_2d_state_cost_function.hpp>
#include <fuse_models/unicycle_2d_state_cost_functor.hpp>

class Unicycle2DStateCostFunction : public benchmark::Fixture
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Unicycle2DStateCostFunction()
  : jacobians(num_parameter_blocks)
    , J(num_parameter_blocks)
  {
    for (size_t i = 0; i < num_parameter_blocks; ++i) {
      J[i].resize(num_residuals, block_sizes[i]);
      jacobians[i] = J[i].data();
    }
  }

  // Analytic cost function
  static constexpr double dt{0.1};
  static const fuse_core::Matrix8d sqrt_information;

  static const fuse_models::Unicycle2DStateCostFunction cost_function;

  // Parameters
  static const double * parameters[];

  // Residuals
  fuse_core::Vector8d residuals;

  static const std::vector<int32_t> & block_sizes;
  static const size_t num_parameter_blocks;

  static const size_t num_residuals;

  // Jacobians
  std::vector<double *> jacobians;

private:
  // Cost function process noise and covariance
  static const double process_noise_diagonal[];

  static const fuse_core::Matrix8d covariance;

  // Parameter blocks
  static const double position1[];
  static const double yaw1[];
  static const double vel_linear1[];
  static const double vel_yaw1[];
  static const double acc_linear1[];

  static const double position2[];
  static const double yaw2[];
  static const double vel_linear2[];
  static const double vel_yaw2[];
  static const double acc_linear2[];

  // Jacobian matrices
  std::vector<fuse_core::MatrixXd> J;
};

// Cost function process noise and covariance
const double Unicycle2DStateCostFunction::process_noise_diagonal[] = {
  1e-3, 1e-3, 1e-2, 1e-6, 1e-6, 1e-4, 1e-9, 1e-9
};

const fuse_core::Matrix8d Unicycle2DStateCostFunction::covariance =
  fuse_core::Vector8d(process_noise_diagonal).asDiagonal();

// Parameter blocks
const double Unicycle2DStateCostFunction::position1[] = {0.0, 0.0};
const double Unicycle2DStateCostFunction::yaw1[] = {0.0};
const double Unicycle2DStateCostFunction::vel_linear1[] = {1.0, 0.0};
const double Unicycle2DStateCostFunction::vel_yaw1[] = {1.570796327};
const double Unicycle2DStateCostFunction::acc_linear1[] = {1.0, 0.0};

const double Unicycle2DStateCostFunction::position2[] = {0.105, 0.0};
const double Unicycle2DStateCostFunction::yaw2[] = {0.1570796327};
const double Unicycle2DStateCostFunction::vel_linear2[] = {1.1, 0.0};
const double Unicycle2DStateCostFunction::vel_yaw2[] = {1.570796327};
const double Unicycle2DStateCostFunction::acc_linear2[] = {1.0, 0.0};

// Analytic cost function
const fuse_core::Matrix8d Unicycle2DStateCostFunction::sqrt_information(covariance.inverse().llt().
  matrixU());

const fuse_models::Unicycle2DStateCostFunction Unicycle2DStateCostFunction::cost_function{dt,
  sqrt_information};

// Parameters
const double * Unicycle2DStateCostFunction::parameters[] = {  // NOLINT(whitespace/braces)
  position1, yaw1, vel_linear1, vel_yaw1, acc_linear1, position2, yaw2, vel_linear2, vel_yaw2,
  acc_linear2
};

const std::vector<int32_t> & Unicycle2DStateCostFunction::block_sizes =
  cost_function.parameter_block_sizes();
const size_t Unicycle2DStateCostFunction::num_parameter_blocks = block_sizes.size();

const size_t Unicycle2DStateCostFunction::num_residuals = cost_function.num_residuals();

BENCHMARK_F(Unicycle2DStateCostFunction, AnalyticUnicycle2DCostFunction)(benchmark::State & state)
{
  for (auto _ : state) {
    cost_function.Evaluate(parameters, residuals.data(), jacobians.data());
  }
}

BENCHMARK_F(
  Unicycle2DStateCostFunction,
  AutoDiffUnicycle2DStateCostFunction)(benchmark::State & state)
{
  // Create cost function using automatic differentiation on the cost functor
  ceres::AutoDiffCostFunction<
    fuse_models::Unicycle2DStateCostFunctor, 8, 2, 1, 2, 1, 2, 2, 1, 2, 1, 2
  >
  cost_function_autodiff(new fuse_models::Unicycle2DStateCostFunctor(dt, sqrt_information));

  for (auto _ : state) {
    cost_function_autodiff.Evaluate(parameters, residuals.data(), jacobians.data());
  }
}

BENCHMARK_MAIN();
