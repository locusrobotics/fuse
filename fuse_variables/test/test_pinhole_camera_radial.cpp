/*
 * Software License Agreement (BSD License)
 *
 *  Author:    Oscar Mendez
 *  Created:   11.13.2023
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
#include <fuse_core/serialization.h>
#include <fuse_variables/pinhole_camera_radial.h>
#include <fuse_variables/stamped.h>
#include <ros/time.h>

#include <ceres/autodiff_cost_function.h>
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <gtest/gtest.h>

#include <sstream>
#include <vector>

using fuse_variables::PinholeCameraRadial;

TEST(PinholeCameraRadialSimple, Type)
{
  PinholeCameraRadial variable(0);
  EXPECT_EQ("fuse_variables::PinholeCameraRadial", variable.type());
}

TEST(PinholeCameraRadial, UUID)
{
  // Verify two positions with the same landmark ids produce the same uuids
  {
    PinholeCameraRadial variable1(0);
    PinholeCameraRadial variable2(0);
    EXPECT_EQ(variable1.uuid(), variable2.uuid());
  }

  // Verify two positions with the different landmark ids  produce different uuids
  {
    PinholeCameraRadial variable1(0);
    PinholeCameraRadial variable2(1);
    EXPECT_NE(variable1.uuid(), variable2.uuid());
  }
}

struct CostFunctor
{
  CostFunctor()
  {
  }

  template <typename T>
  bool operator()(const T* const k, T* residual) const
  {
    residual[0] = k[0] - T(1.2);
    residual[1] = k[1] + T(0.8);
    residual[2] = k[2] - T(0.51);

    return true;
  }
};

TEST(PinholeCameraRadial, Optimization)
{
  // Create a Point3DLandmark
  PinholeCameraRadial K(0);
  K.f() = 4.1;
  K.r1() = 3.5;
  K.r2() = 5;

  // Create a simple a constraint
  ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<CostFunctor, 3, 3>(new CostFunctor());

  // Build the problem.
  ceres::Problem problem;
  problem.AddParameterBlock(K.data(), K.size());
  std::vector<double*> parameter_blocks;
  parameter_blocks.push_back(K.data());
  problem.AddResidualBlock(cost_function, nullptr, parameter_blocks);

  // Run the solver
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // Check
  EXPECT_NEAR(1.2, K.f(), 1.0e-5);
  EXPECT_NEAR(-0.8, K.r1(), 1.0e-5);
  EXPECT_NEAR(0.51, K.r2(), 1.0e-5);
}

TEST(PinholeCameraRadial, Serialization)
{
  // Create a Point3DLandmark
  PinholeCameraRadial expected(0);
  expected.f() = 640;
  expected.r1() = 0.5;
  expected.r2() = 0.5;

  // Serialize the variable into an archive
  std::stringstream stream;
  {
    fuse_core::TextOutputArchive archive(stream);
    expected.serialize(archive);
  }

  // Deserialize a new variable from that same stream
  PinholeCameraRadial actual;
  {
    fuse_core::TextInputArchive archive(stream);
    actual.deserialize(archive);
  }

  // Compare
  EXPECT_EQ(expected.id(), actual.id());
  EXPECT_EQ(expected.uuid(), actual.uuid());
  EXPECT_EQ(expected.f(), actual.f());
  EXPECT_EQ(expected.r1(), actual.r1());
  EXPECT_EQ(expected.r2(), actual.r2());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
