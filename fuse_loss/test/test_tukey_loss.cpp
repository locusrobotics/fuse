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
#include <ceres/autodiff_cost_function.h>
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <gtest/gtest.h>

#include <memory>

#include <fuse_core/serialization.hpp>
#include <fuse_loss/tukey_loss.hpp>

TEST(TukeyLoss, Constructor)
{
  // Create a default loss
  {
    fuse_loss::TukeyLoss loss;
    ASSERT_EQ(1.0, loss.a());
  }

  // Create a loss with a parameter
  {
    const double a{0.3};
    fuse_loss::TukeyLoss loss(a);
    ASSERT_EQ(a, loss.a());
  }
}

TEST(TukeyLoss, Evaluate)
{
  // Check that at s = 0: rho = [0, 1, -2 / a^2].
  fuse_loss::TukeyLoss loss(0.7);
  const std::unique_ptr<ceres::LossFunction> loss_function(loss.lossFunction());

  double rho[3];
  loss_function->Evaluate(0.0, rho);
  ASSERT_NEAR(rho[0], 0.0, 1e-6);
  ASSERT_NEAR(rho[1], 1.0, 1e-6);
  ASSERT_NEAR(rho[2], -2.0 / (0.7 * 0.7), 1e-6);
}

struct CostFunctor
{
  explicit CostFunctor(const double data)
  : data(data)
  {}

  template<typename T> bool operator()(const T * const x, T * residual) const
  {
    residual[0] = x[0] - T(data);
    return true;
  }

  double data{0.0};
};

TEST(TukeyLoss, Optimization)
{
  // Create a simple parameter
  double x{5.0};

  // Create a simple inlier constraint
  const double inlier{1.0};

  // Create a simple outlier constraint
  const double outlier{10.0};
  ceres::CostFunction * cost_function_outlier =
    new ceres::AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor(outlier));

  // Create loss with a = x, so the initial value of x is not handled in the outlier region, in
  // which case the optimization does not convergence and the initial solution does not change
  fuse_loss::TukeyLoss loss(x);

  // Build the problem.
  ceres::Problem::Options problem_options;
  problem_options.loss_function_ownership = fuse_core::Loss::Ownership;

  ceres::Problem problem(problem_options);

  const size_t num_inliers{1000};
  for (size_t i = 0; i < num_inliers; ++i) {
    problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor(inlier)),
      loss.lossFunction(),  // A nullptr here would produce a slightly better solution
      &x);
  }

  // Add outlier constraints
  const size_t num_outliers{9};
  for (size_t i = 0; i < num_outliers; ++i) {
    problem.AddResidualBlock(
      cost_function_outlier,
      loss.lossFunction(),
      &x);
  }

  // Run the solver
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // Check
  EXPECT_NEAR(inlier, x, 1.0e-3);

  // Evaluate problem cost
  double cost = 0.0;
  problem.Evaluate(ceres::Problem::EvaluateOptions(), &cost, nullptr, nullptr, nullptr);

  // Evaluate problem without applying the loss
  ceres::Problem::EvaluateOptions evaluate_options;
  evaluate_options.apply_loss_function = false;

  double raw_cost = 0.0;
  problem.Evaluate(evaluate_options, &raw_cost, nullptr, nullptr, nullptr);

  // Check the cost with loss is lower
  EXPECT_LT(cost, raw_cost);
}

TEST(TukeyLoss, Serialization)
{
  // Construct a loss
  const double a{0.3};
  fuse_loss::TukeyLoss expected(a);

  // Serialize the loss into an archive
  std::stringstream stream;
  {
    fuse_core::TextOutputArchive archive(stream);
    expected.serialize(archive);
  }

  // Deserialize a new loss from that same stream
  fuse_loss::TukeyLoss actual;
  {
    fuse_core::TextInputArchive archive(stream);
    actual.deserialize(archive);
  }

  // Compare
  EXPECT_EQ(expected.a(), actual.a());
  EXPECT_NE(nullptr, actual.lossFunction());

  // Test inlier (s <= a*a)
  const double s = 0.95 * a * a;
  double rho[3] = {0.0};
  actual.lossFunction()->Evaluate(s, rho);

  EXPECT_GT(a * a / 3.0, rho[0]);
  EXPECT_GT(1.0, rho[1]);
  EXPECT_GT(0.0, rho[2]);

  // Test outlier
  const double s_outlier = 1.05 * a * a;
  actual.lossFunction()->Evaluate(s_outlier, rho);

  // In the outlier region rho() satisfies:
  //
  //   rho(s) < s
  //   rho'(s) < 1
  //   rho''(s) < 0
  //
  // For the particular case of the Tukey loss we also have:
  //
  //   rho(s) = a^2 / 3
  //   rho'(s) = 0
  //   rho''(s) = 0
  //
  // where rho(s) is rho[0], rho'(s) is rho[1] and rho''(s) is rho[2]
  EXPECT_NEAR(a * a / 3.0, rho[0], 1e-6);
  EXPECT_NEAR(0.0, rho[1], 1e-6);
  EXPECT_NEAR(0.0, rho[2], 1e-6);
}
