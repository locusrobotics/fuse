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
#include <benchmark/benchmark.h>
#include <ceres/autodiff_cost_function.h>
#include <Eigen/Dense>

#include <vector>

#include <fuse_constraints/normal_prior_pose_2d.hpp>
#include <fuse_constraints/normal_prior_pose_2d_cost_functor.hpp>

class NormalPriorPose2DBenchmarkFixture : public benchmark::Fixture
{
public:
  NormalPriorPose2DBenchmarkFixture()
  : jacobians(num_parameter_blocks)
    , J(num_parameter_blocks)
  {
    for (size_t i = 0; i < num_parameter_blocks; ++i) {
      J[i].resize(num_residuals, block_sizes[i]);
      jacobians[i] = J[i].data();
    }
  }

  // Mean and sqrt information matrix
  static const fuse_core::Vector3d mean;
  static const fuse_core::Matrix3d sqrt_information;

  // Parameters
  static const double * parameters[];

  // Residuals
  fuse_core::Vector3d residuals;

  static const std::vector<int32_t> & block_sizes;
  static const size_t num_parameter_blocks;

  static const size_t num_residuals;

  // Jacobians
  std::vector<double *> jacobians;

private:
  // Cost function covariance
  static const double covariance_diagonal[];

  static const fuse_core::Matrix3d covariance;

  // Parameter blocks
  static const double position[];
  static const double orientation[];

  // Jacobian matrices
  std::vector<fuse_core::MatrixXd> J;
};

// Cost function covariance
const double NormalPriorPose2DBenchmarkFixture::covariance_diagonal[] = {2e-3, 1e-3, 1e-2};

const fuse_core::Matrix3d NormalPriorPose2DBenchmarkFixture::covariance =
  fuse_core::Vector3d(covariance_diagonal).asDiagonal();

// Parameter blocks
const double NormalPriorPose2DBenchmarkFixture::position[] = {0.0, 0.0};
const double NormalPriorPose2DBenchmarkFixture::orientation[] = {0.0};

// Mean and sqrt information matrix
const fuse_core::Vector3d NormalPriorPose2DBenchmarkFixture::mean{1.0, 2.0, 3.0};
const fuse_core::Matrix3d NormalPriorPose2DBenchmarkFixture::sqrt_information(covariance.inverse().
  llt().matrixU());

// Parameters
const double * NormalPriorPose2DBenchmarkFixture::parameters[] = {position, orientation};

const std::vector<int32_t> & NormalPriorPose2DBenchmarkFixture::block_sizes = {2, 1};
const size_t NormalPriorPose2DBenchmarkFixture::num_parameter_blocks = block_sizes.size();

const size_t NormalPriorPose2DBenchmarkFixture::num_residuals = 3;

BENCHMARK_DEFINE_F(
  NormalPriorPose2DBenchmarkFixture,
  AnalyticNormalPriorPose2D)(benchmark::State & state)
{
  // Create analytic cost function
  const fuse_constraints::NormalPriorPose2D cost_function{sqrt_information.topRows(state.range(0)),
    mean};

  for (auto _ : state) {
    cost_function.Evaluate(parameters, residuals.data(), jacobians.data());
  }
}

BENCHMARK_REGISTER_F(NormalPriorPose2DBenchmarkFixture, AnalyticNormalPriorPose2D)->DenseRange(
  1, 3);

BENCHMARK_DEFINE_F(
  NormalPriorPose2DBenchmarkFixture,
  AutoDiffNormalPriorPose2D)(benchmark::State & state)
{
  // Create cost function using automatic differentiation on the cost functor
  const auto partial_sqrt_information = sqrt_information.topRows(state.range(0));
  const ceres::AutoDiffCostFunction<
    fuse_constraints::NormalPriorPose2DCostFunctor,
    ceres::DYNAMIC,
    2, 1
  >
  cost_function_autodiff(new fuse_constraints::NormalPriorPose2DCostFunctor(
      partial_sqrt_information,
      mean),
    partial_sqrt_information.rows());

  for (auto _ : state) {
    cost_function_autodiff.Evaluate(parameters, residuals.data(), jacobians.data());
  }
}

BENCHMARK_REGISTER_F(NormalPriorPose2DBenchmarkFixture, AutoDiffNormalPriorPose2D)->DenseRange(
  1, 3);

BENCHMARK_MAIN();
