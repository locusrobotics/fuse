/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Locus Robotics
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
#include <gtest/gtest.h>

#include <fuse_core/ceres_macros.hpp>
#include <fuse_core/eigen.hpp>
#include <fuse_core/eigen_gtest.hpp>

#if !CERES_SUPPORTS_MANIFOLDS
#include <fuse_core/autodiff_local_parameterization.hpp>

struct Plus
{
  template<typename T>
  bool operator()(const T * x, const T * delta, T * x_plus_delta) const
  {
    x_plus_delta[0] = x[0] + 2.0 * delta[0];
    x_plus_delta[1] = x[1] + 5.0 * delta[1];
    x_plus_delta[2] = x[2];
    return true;
  }
};

struct Minus
{
  template<typename T>
  bool operator()(const T * x, const T * y, T * y_minus_x) const
  {
    y_minus_x[0] = (y[0] - x[0]) / 2.0;
    y_minus_x[1] = (y[1] - x[1]) / 5.0;
    return true;
  }
};

using TestLocalParameterization = fuse_core::AutoDiffLocalParameterization<Plus, Minus, 3, 2>;


TEST(LocalParameterization, Plus)
{
  TestLocalParameterization parameterization;

  double x[3] = {1.0, 2.0, 3.0};
  double delta[2] = {0.5, 1.0};
  double actual[3] = {0.0, 0.0, 0.0};
  bool success = parameterization.Plus(x, delta, actual);

  EXPECT_TRUE(success);
  EXPECT_NEAR(2.0, actual[0], 1.0e-5);
  EXPECT_NEAR(7.0, actual[1], 1.0e-5);
  EXPECT_NEAR(3.0, actual[2], 1.0e-5);
}

TEST(LocalParameterization, PlusJacobian)
{
  TestLocalParameterization parameterization;

  double x[3] = {1.0, 2.0, 3.0};
  fuse_core::MatrixXd actual(3, 2);
  bool success = parameterization.ComputeJacobian(x, actual.data());

  fuse_core::MatrixXd expected(3, 2);

  /* *INDENT-OFF* */  // Bypass uncrustify
  expected << 2.0, 0.0,
              0.0, 5.0,
              0.0, 0.0;
  /* *INDENT-ON* */

  EXPECT_TRUE(success);
  EXPECT_MATRIX_NEAR(expected, actual, 1.0e-5);
}

TEST(LocalParameterization, Minus)
{
  TestLocalParameterization parameterization;

  double x1[3] = {1.0, 2.0, 3.0};
  double x2[3] = {2.0, 7.0, 3.0};
  double actual[2] = {0.0, 0.0};
  bool success = parameterization.Minus(x1, x2, actual);

  EXPECT_TRUE(success);
  EXPECT_NEAR(0.5, actual[0], 1.0e-5);
  EXPECT_NEAR(1.0, actual[1], 1.0e-5);
}

TEST(LocalParameterization, MinusJacobian)
{
  TestLocalParameterization parameterization;

  double x[3] = {1.0, 2.0, 3.0};
  fuse_core::MatrixXd actual(2, 3);
  bool success = parameterization.ComputeMinusJacobian(x, actual.data());

  fuse_core::MatrixXd expected(2, 3);

  /* *INDENT-OFF* */  // Bypass uncrustify
  expected << 0.5, 0.0, 0.0,
              0.0, 0.2, 0.0;
  /* *INDENT-ON* */

  EXPECT_TRUE(success);
  EXPECT_MATRIX_NEAR(expected, actual, 1.0e-5);
}

TEST(LocalParameterization, MinusSameVariablesIsZero)
{
  TestLocalParameterization parameterization;

  double x1[3] = {1.0, 2.0, 3.0};
  double actual[2] = {0.0, 0.0};
  bool success = parameterization.Minus(x1, x1, actual);

  EXPECT_TRUE(success);
  EXPECT_NEAR(0.0, actual[0], 1.0e-5);
  EXPECT_NEAR(0.0, actual[1], 1.0e-5);
}

TEST(LocalParameterization, PlusMinus)
{
  TestLocalParameterization parameterization;

  const double x1[3] = {1.0, 2.0, 3.0};
  const double delta[2] = {0.5, 1.0};
  double x2[3] = {0.0, 0.0, 0.0};
  bool success = parameterization.Plus(x1, delta, x2);

  ASSERT_TRUE(success);

  double actual[2] = {0.0, 0.0};
  success = parameterization.Minus(x1, x2, actual);

  EXPECT_TRUE(success);
  EXPECT_NEAR(delta[0], actual[0], 1.0e-5);
  EXPECT_NEAR(delta[1], actual[1], 1.0e-5);
}
#endif
