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
#include <fuse_loss/composed_loss.hpp>
#include <fuse_loss/huber_loss.hpp>
#include <fuse_loss/scaled_loss.hpp>
#include <fuse_loss/tolerant_loss.hpp>
#include <fuse_loss/trivial_loss.hpp>

TEST(ComposedLoss, Constructor)
{
  // Create a default loss
  {
    fuse_loss::ComposedLoss composed_loss;
    EXPECT_EQ(nullptr, composed_loss.fLoss());
    EXPECT_EQ(nullptr, composed_loss.gLoss());

    // Check nullptr is handled as TrivialLoss internally
    std::unique_ptr<ceres::LossFunction> composed_loss_function = nullptr;
    ASSERT_NO_THROW(composed_loss_function.reset(composed_loss.lossFunction()));
    ASSERT_NE(nullptr, composed_loss_function);

    const double s = 1.5;
    double rho[3] = {0.0};
    composed_loss_function->Evaluate(s, rho);

    EXPECT_EQ(s, rho[0]);
    EXPECT_EQ(1.0, rho[1]);
    EXPECT_EQ(0.0, rho[2]);
  }

  // Create a loss with f_loss parameter only
  {
    std::shared_ptr<fuse_loss::HuberLoss> f_loss{new fuse_loss::HuberLoss};

    fuse_loss::ComposedLoss composed_loss(f_loss);
    EXPECT_NE(nullptr, composed_loss.fLoss().get());
    EXPECT_EQ(f_loss.get(), composed_loss.fLoss().get());
    EXPECT_EQ(nullptr, composed_loss.gLoss());

    // Check nullptr is handled as TrivialLoss internally
    std::unique_ptr<ceres::LossFunction> composed_loss_function = nullptr;
    ASSERT_NO_THROW(composed_loss_function.reset(composed_loss.lossFunction()));
    ASSERT_NE(nullptr, composed_loss_function);

    const auto f_loss_function = std::unique_ptr<ceres::LossFunction>(f_loss->lossFunction());
    ASSERT_NE(nullptr, f_loss_function);

    const double s = 1.5;
    double rho[3] = {0.0};
    composed_loss_function->Evaluate(s, rho);

    double f_rho[3] = {0.0};
    f_loss_function->Evaluate(s, f_rho);

    // Make sure 'f(s) != s', i.e. it is not an inlier, which would be a trivial case
    ASSERT_NE(s, f_rho[0]);

    // Check that 'f(g(s)) == f(s)' and the same for the first and second derivatives, since g is
    // the TrivialLoss
    for (size_t i = 0; i < 3; ++i) {
      EXPECT_EQ(f_rho[i], rho[i]);
    }
  }

  // Create a loss with g_loss parameter only
  {
    std::shared_ptr<fuse_loss::HuberLoss> g_loss{new fuse_loss::HuberLoss};

    fuse_loss::ComposedLoss composed_loss(nullptr, g_loss);
    EXPECT_EQ(nullptr, composed_loss.fLoss());
    EXPECT_NE(nullptr, composed_loss.gLoss().get());
    EXPECT_EQ(g_loss.get(), composed_loss.gLoss().get());

    // Check nullptr is handled as TrivialLoss internally
    std::unique_ptr<ceres::LossFunction> composed_loss_function = nullptr;
    ASSERT_NO_THROW(composed_loss_function.reset(composed_loss.lossFunction()));
    ASSERT_NE(nullptr, composed_loss_function);

    const auto g_loss_function = std::unique_ptr<ceres::LossFunction>(g_loss->lossFunction());
    ASSERT_NE(nullptr, g_loss_function);

    const double s = 1.5;
    double rho[3] = {0.0};
    composed_loss_function->Evaluate(s, rho);

    double g_rho[3] = {0.0};
    g_loss_function->Evaluate(s, g_rho);

    // Make sure 'g(s) != s', i.e. it is not an inlier, which would be a trivial case
    ASSERT_NE(s, g_rho[0]);

    // Check that 'f(g(s)) == g(s)' and the same for the first and second derivatives, since f is
    // the TrivialLoss
    for (size_t i = 0; i < 3; ++i) {
      EXPECT_EQ(g_rho[i], rho[i]);
    }
  }

  // Create a loss with f_loss and g_loss parameters
  {
    std::shared_ptr<fuse_loss::HuberLoss> f_loss{new fuse_loss::HuberLoss};
    std::shared_ptr<fuse_loss::TolerantLoss> g_loss{new fuse_loss::TolerantLoss};

    fuse_loss::ComposedLoss composed_loss(f_loss, g_loss);
    EXPECT_NE(nullptr, composed_loss.fLoss().get());
    EXPECT_EQ(f_loss.get(), composed_loss.fLoss().get());
    EXPECT_NE(nullptr, composed_loss.gLoss().get());
    EXPECT_EQ(g_loss.get(), composed_loss.gLoss().get());

    // Check the composed loss is computed as 'f(g(s))'
    std::unique_ptr<ceres::LossFunction> composed_loss_function = nullptr;
    ASSERT_NO_THROW(composed_loss_function.reset(composed_loss.lossFunction()));
    ASSERT_NE(nullptr, composed_loss_function);

    const auto f_loss_function = std::unique_ptr<ceres::LossFunction>(f_loss->lossFunction());
    ASSERT_NE(nullptr, f_loss_function);

    const auto g_loss_function = std::unique_ptr<ceres::LossFunction>(g_loss->lossFunction());
    ASSERT_NE(nullptr, g_loss_function);

    const double s = 1.5;
    double rho[3] = {0.0};
    composed_loss_function->Evaluate(s, rho);

    double g_rho[3] = {0.0};
    g_loss_function->Evaluate(s, g_rho);

    // Make sure 'g(s) != s', i.e. it is not an inlier, which would be a trivial case
    ASSERT_NE(s, g_rho[0]);

    double f_rho[3] = {0.0};
    f_loss_function->Evaluate(g_rho[0], f_rho);

    // Make sure 'f(s) != s', i.e. it is not an inlier, which would be a trivial case
    ASSERT_NE(s, f_rho[0]);

    EXPECT_EQ(f_rho[0], rho[0]);
    // f'(g(s)) * g'(s)
    EXPECT_EQ(f_rho[1] * g_rho[1], rho[1]);
    // f''(g(s)) * g'(s) * g'(s) + f'(g(s)) * g''(s)
    EXPECT_EQ(f_rho[2] * g_rho[1] * g_rho[1] + f_rho[1] * g_rho[2], rho[2]);
  }
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

TEST(ComposedLoss, Optimization)
{
  // Create a simple parameter
  double x{5.0};

  // Create a simple inlier constraint
  const double inlier{1.0};

  // Create a simple outlier constraint
  const double outlier{10.0};
  ceres::CostFunction * cost_function_outlier =
    new ceres::AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor(outlier));

  // Create an 'f' loss
  const double a{0.05};
  std::shared_ptr<fuse_loss::HuberLoss> f_loss{new fuse_loss::HuberLoss(a)};

  // Create an 'g' loss
  const double scaled_a{0.5};
  std::shared_ptr<fuse_loss::ScaledLoss> g_loss{new fuse_loss::ScaledLoss(scaled_a)};

  // Create a composed loss, which illustrates the case of scaling the residuals by a factor with a
  // fuse_loss::ScaledLoss in the 'g' loss and applies a fuse_loss::HuberLoss loss function robust
  // to outliers in the 'f' loss, which are used in the composition 'f(g(s))' for the squared
  // residuals 's'
  fuse_loss::ComposedLoss composed_loss(f_loss, g_loss);

  // Build the problem.
  ceres::Problem::Options problem_options;
  problem_options.loss_function_ownership = fuse_core::Loss::Ownership;

  ceres::Problem problem(problem_options);

  const size_t num_inliers{1000};
  for (size_t i = 0; i < num_inliers; ++i) {
    problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor(inlier)),
      composed_loss.lossFunction(),  // A nullptr here would produce a slightly better solution
      &x);
  }

  // Add outlier constraints
  const size_t num_outliers{9};
  for (size_t i = 0; i < num_outliers; ++i) {
    problem.AddResidualBlock(
      cost_function_outlier,
      composed_loss.lossFunction(),
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

TEST(ComposedLoss, Serialization)
{
  // Construct an 'f' loss
  const double f_loss_a{0.3};
  std::shared_ptr<fuse_loss::HuberLoss> f_loss{new fuse_loss::HuberLoss(f_loss_a)};

  // Construct a 'g' loss
  const double g_loss_a{0.3};
  const double g_loss_b{0.6};
  std::shared_ptr<fuse_loss::TolerantLoss> g_loss{new fuse_loss::TolerantLoss(g_loss_a, g_loss_b)};

  // Construct a composed loss
  fuse_loss::ComposedLoss expected(f_loss, g_loss);

  // Serialize the loss into an archive
  std::stringstream stream;
  {
    fuse_core::TextOutputArchive archive(stream);
    expected.serialize(archive);
  }

  // Deserialize a new loss from that same stream
  fuse_loss::ComposedLoss actual;
  {
    fuse_core::TextInputArchive archive(stream);
    actual.deserialize(archive);
  }

  // Compare
  const auto expected_loss_function = std::unique_ptr<ceres::LossFunction>(actual.lossFunction());
  const auto actual_loss_function = std::unique_ptr<ceres::LossFunction>(actual.lossFunction());

  ASSERT_NE(nullptr, actual_loss_function);
  EXPECT_NE(nullptr, actual.fLoss());
  EXPECT_NE(nullptr, actual.gLoss());

  // Test inlier (s <= g_loss_a*g_loss_a)
  const double s = 0.95 * g_loss_a * g_loss_a;
  double expected_rho[3] = {0.0};
  expected_loss_function->Evaluate(s, expected_rho);

  double actual_rho[3] = {0.0};
  actual_loss_function->Evaluate(s, actual_rho);

  for (size_t i = 0; i < 3; ++i) {
    EXPECT_EQ(expected_rho[i], actual_rho[i]);
  }

  // Test outlier (s > g_loss_b)
  const double s_outlier = 1.05 * g_loss_a * g_loss_a;

  expected_loss_function->Evaluate(s_outlier, expected_rho);
  actual_loss_function->Evaluate(s_outlier, actual_rho);

  for (size_t i = 0; i < 3; ++i) {
    EXPECT_EQ(expected_rho[i], actual_rho[i]);
  }
}
