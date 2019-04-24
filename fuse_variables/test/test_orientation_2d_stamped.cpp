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
#include <fuse_variables/orientation_2d_stamped.h>
#include <fuse_variables/stamped.h>
#include <ros/time.h>

#include <ceres/autodiff_cost_function.h>
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <gtest/gtest.h>

#include <vector>

using fuse_variables::Orientation2DStamped;


TEST(Orientation2DStamped, Type)
{
  Orientation2DStamped variable(ros::Time(12345678, 910111213));
  EXPECT_EQ("fuse_variables::Orientation2DStamped", variable.type());
}

TEST(Orientation2DStamped, UUID)
{
  // Verify two velocities at the same timestamp produce the same UUID
  {
    Orientation2DStamped variable1(ros::Time(12345678, 910111213));
    Orientation2DStamped variable2(ros::Time(12345678, 910111213));
    EXPECT_EQ(variable1.uuid(), variable2.uuid());

    Orientation2DStamped variable3(ros::Time(12345678, 910111213), fuse_core::uuid::generate("c3po"));
    Orientation2DStamped variable4(ros::Time(12345678, 910111213), fuse_core::uuid::generate("c3po"));
    EXPECT_EQ(variable3.uuid(), variable4.uuid());
  }

  // Verify two velocities at different timestamps produce different UUIDs
  {
    Orientation2DStamped variable1(ros::Time(12345678, 910111213));
    Orientation2DStamped variable2(ros::Time(12345678, 910111214));
    Orientation2DStamped variable3(ros::Time(12345679, 910111213));
    EXPECT_NE(variable1.uuid(), variable2.uuid());
    EXPECT_NE(variable1.uuid(), variable3.uuid());
    EXPECT_NE(variable2.uuid(), variable3.uuid());
  }

  // Verify two velocities with different hardware IDs produce different UUIDs
  {
    Orientation2DStamped variable1(ros::Time(12345678, 910111213), fuse_core::uuid::generate("r2d2"));
    Orientation2DStamped variable2(ros::Time(12345678, 910111213), fuse_core::uuid::generate("bb8"));
    EXPECT_NE(variable1.uuid(), variable2.uuid());
  }
}

TEST(Orientation2DStamped, Stamped)
{
  fuse_core::Variable::SharedPtr base = Orientation2DStamped::make_shared(ros::Time(12345678, 910111213),
                                                                          fuse_core::uuid::generate("mo"));
  auto derived = std::dynamic_pointer_cast<Orientation2DStamped>(base);
  ASSERT_TRUE(static_cast<bool>(derived));
  EXPECT_EQ(ros::Time(12345678, 910111213), derived->stamp());
  EXPECT_EQ(fuse_core::uuid::generate("mo"), derived->deviceId());

  auto stamped = std::dynamic_pointer_cast<fuse_variables::Stamped>(base);
  ASSERT_TRUE(static_cast<bool>(stamped));
  EXPECT_EQ(ros::Time(12345678, 910111213), stamped->stamp());
  EXPECT_EQ(fuse_core::uuid::generate("mo"), stamped->deviceId());
}

struct CostFunctor
{
  CostFunctor() {}

  template <typename T> bool operator()(const T* const x, T* residual) const
  {
    residual[0] = x[0] - T(3.0);
    return true;
  }
};

// TEST(Orientation2DStamped, Optimization)
// {
//  // Create a Orientation2DStamped
//  Orientation2DStamped orientation(ros::Time(12345678, 910111213), fuse_core::uuid::generate("hal9000"));
//  orientation.yaw() = 1.5;
//
//  // Create a simple a constraint
//  ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor());
//
//  // Build the problem.
//  ceres::Problem problem;
//  problem.AddParameterBlock(
//    orientation.data(),
//    orientation.size(),
//    orientation.localParameterization());
//  std::vector<double*> parameter_blocks;
//  parameter_blocks.push_back(orientation.data());
//  problem.AddResidualBlock(
//    cost_function,
//    nullptr,
//    parameter_blocks);
//
//  // Run the solver
//  ceres::Solver::Options options;
//  ceres::Solver::Summary summary;
//  ceres::Solve(options, &problem, &summary);
//
//  // Check
//  EXPECT_NEAR(3.0, orientation.yaw(), 1.0e-5);
// }

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
