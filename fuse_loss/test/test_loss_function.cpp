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
#include <fuse_loss/loss_function.h>

#include <gtest/gtest.h>

// The following function has been copied and adapted from:
//
// https://github.com/ceres-solver/ceres-solver/blob/master/internal/ceres/loss_function_test.cc
//
// so we can test the loss functions implemented here.
namespace
{

// Helper function for testing a LossFunction callback.
//
// Compares the values of rho'(s) and rho''(s) computed by the
// callback with estimates obtained by symmetric finite differencing
// of rho(s).
void AssertLossFunctionIsValid(const ceres::LossFunction& loss, double s)
{
  ASSERT_GT(s, 0);

  // Evaluate rho(s), rho'(s) and rho''(s).
  double rho[3];
  loss.Evaluate(s, rho);

  // Use symmetric finite differencing to estimate rho'(s) and
  // rho''(s).
  const double kH = 1e-4;
  // Values at s + kH.
  double fwd[3];
  // Values at s - kH.
  double bwd[3];
  loss.Evaluate(s + kH, fwd);
  loss.Evaluate(s - kH, bwd);

  // First derivative.
  const double fd_1 = (fwd[0] - bwd[0]) / (2 * kH);
  ASSERT_NEAR(fd_1, rho[1], 1e-6);

  // Second derivative.
  const double fd_2 = (fwd[0] - 2*rho[0] + bwd[0]) / (kH * kH);
  ASSERT_NEAR(fd_2, rho[2], 1e-6);
}

}  // namespace

// Try two values of the scaling a = 0.7 and 1.3
// (where scaling makes sense) and of the squared norm
// s = 0.357 and 1.792
TEST(LossFunction, DCSLoss)
{
  AssertLossFunctionIsValid(ceres::DCSLoss(0.7), 0.357);
  AssertLossFunctionIsValid(ceres::DCSLoss(0.7), 1.792);
  AssertLossFunctionIsValid(ceres::DCSLoss(1.3), 0.357);
  AssertLossFunctionIsValid(ceres::DCSLoss(1.3), 1.792);
}

TEST(LossFunction, FairLoss)
{
  AssertLossFunctionIsValid(ceres::FairLoss(0.7), 0.357);
  AssertLossFunctionIsValid(ceres::FairLoss(0.7), 1.792);
  AssertLossFunctionIsValid(ceres::FairLoss(1.3), 0.357);
  AssertLossFunctionIsValid(ceres::FairLoss(1.3), 1.792);
}

TEST(LossFunction, GemanMcClureLoss)
{
  AssertLossFunctionIsValid(ceres::GemanMcClureLoss(0.7), 0.357);
  AssertLossFunctionIsValid(ceres::GemanMcClureLoss(0.7), 1.792);
  AssertLossFunctionIsValid(ceres::GemanMcClureLoss(1.3), 0.357);
  AssertLossFunctionIsValid(ceres::GemanMcClureLoss(1.3), 1.792);
}

TEST(LossFunction, WelschLoss)
{
  AssertLossFunctionIsValid(ceres::WelschLoss(0.7), 0.357);
  AssertLossFunctionIsValid(ceres::WelschLoss(0.7), 1.792);
  AssertLossFunctionIsValid(ceres::WelschLoss(1.3), 0.357);
  AssertLossFunctionIsValid(ceres::WelschLoss(1.3), 1.792);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
