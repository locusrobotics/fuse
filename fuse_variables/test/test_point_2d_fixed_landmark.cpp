/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Locus Robotics
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

#include <sstream>
#include <vector>

#include <fuse_core/serialization.hpp>
#include <fuse_variables/point_2d_fixed_landmark.hpp>
#include <fuse_variables/stamped.hpp>
#include <rclcpp/time.hpp>

using fuse_variables::Point2DFixedLandmark;


TEST(Point2DFixedLandmark, Type)
{
  Point2DFixedLandmark variable(0);
  EXPECT_EQ("fuse_variables::Point2DFixedLandmark", variable.type());
}

TEST(Point2DFixedLandmark, UUID)
{
  // Verify two positions with the same landmark ids produce the same uuids
  {
    Point2DFixedLandmark variable1(0);
    Point2DFixedLandmark variable2(0);
    EXPECT_EQ(variable1.uuid(), variable2.uuid());
  }

  // Verify two positions with the different landmark ids  produce different uuids
  {
    Point2DFixedLandmark variable1(0);
    Point2DFixedLandmark variable2(1);
    EXPECT_NE(variable1.uuid(), variable2.uuid());
  }
}

struct CostFunctor
{
  CostFunctor() {}

  template<typename T> bool operator()(const T * const x, T * residual) const
  {
    residual[0] = x[0] - T(3.0);
    residual[1] = x[1] + T(8.0);
    residual[2] = x[2] - T(3.1);
    return true;
  }
};

TEST(Point2DFixedLandmark, Optimization)
{
  // Create a Point2DFixedLandmark
  Point2DFixedLandmark position(0);
  position.x() = 1.5;
  position.y() = -3.0;

  // Create a simple a constraint
  ceres::CostFunction * cost_function = new ceres::AutoDiffCostFunction<CostFunctor, 2, 2>(
    new CostFunctor());

  // Build the problem.
  ceres::Problem problem;
  problem.AddParameterBlock(position.data(), position.size());
  std::vector<double *> parameter_blocks;
  parameter_blocks.push_back(position.data());
  problem.AddResidualBlock(cost_function, nullptr, parameter_blocks);
  if (position.holdConstant()) {
    problem.SetParameterBlockConstant(position.data());
  }

  // Run the solver
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // Check the variable value has not changed since the landmark is constant
  EXPECT_NEAR(1.5, position.x(), 1.0e-5);
  EXPECT_NEAR(-3.0, position.y(), 1.0e-5);
}

TEST(Point2DFixedLandmark, Serialization)
{
  // Create a Point2DFixedLandmark
  Point2DFixedLandmark expected(0);
  expected.x() = 1.5;
  expected.y() = -3.0;

  // Serialize the variable into an archive
  std::stringstream stream;
  {
    fuse_core::TextOutputArchive archive(stream);
    expected.serialize(archive);
  }

  // Deserialize a new variable from that same stream
  Point2DFixedLandmark actual;
  {
    fuse_core::TextInputArchive archive(stream);
    actual.deserialize(archive);
  }

  // Compare
  EXPECT_EQ(expected.id(), actual.id());
  EXPECT_EQ(expected.uuid(), actual.uuid());
  EXPECT_EQ(expected.x(), actual.x());
  EXPECT_EQ(expected.y(), actual.y());
}
