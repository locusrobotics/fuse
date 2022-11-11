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
#include <fuse_core/serialization.h>
#include <fuse_variables/position_3d_stamped.h>
#include <fuse_variables/stamped.h>
#include <fuse_core/time.h>

#include <ceres/autodiff_cost_function.h>
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <gtest/gtest.h>

#include <sstream>
#include <vector>

using fuse_variables::Position3DStamped;


TEST(Position3DStamped, Type)
{
  Position3DStamped variable(rclcpp::Time(12345678, 910111213));
  EXPECT_EQ("fuse_variables::Position3DStamped", variable.type());
}

TEST(Position3DStamped, UUID)
{
  // Verify two positions at the same timestamp produce the same UUID
  {
    Position3DStamped variable1(rclcpp::Time(12345678, 910111213));
    Position3DStamped variable2(rclcpp::Time(12345678, 910111213));
    EXPECT_EQ(variable1.uuid(), variable2.uuid());
  }

  auto uuid_1 = fuse_core::uuid::generate("test_hardware1");
  auto uuid_2 = fuse_core::uuid::generate("test_hardware2");

  // Verify two positions at the same timestamp and same hardware ID produce the same UUID
  {
    Position3DStamped variable1(rclcpp::Time(12345678, 910111213), uuid_1);
    Position3DStamped variable2(rclcpp::Time(12345678, 910111213), uuid_1);
    EXPECT_EQ(variable1.uuid(), variable2.uuid());
  }

  // Verify two positions with the same timestamp but different hardware IDs generate different UUIDs
  {
    Position3DStamped variable1(rclcpp::Time(12345678, 910111213), uuid_1);
    Position3DStamped variable2(rclcpp::Time(12345678, 910111213), uuid_2);
    EXPECT_NE(variable1.uuid(), variable2.uuid());
  }

  // Verify two positions with the same hardware ID and different timestamps produce different UUIDs
  {
    Position3DStamped variable1(rclcpp::Time(12345678, 910111213), uuid_1);
    Position3DStamped variable2(rclcpp::Time(12345678, 910111214), uuid_1);
    EXPECT_NE(variable1.uuid(), variable2.uuid());

    Position3DStamped variable3(rclcpp::Time(12345678, 910111213), uuid_1);
    Position3DStamped variable4(rclcpp::Time(12345679, 910111213), uuid_1);
    EXPECT_NE(variable3.uuid(), variable4.uuid());
  }

  // Verify two positions with different hardware IDs and different timestamps produce different UUIDs
  {
    Position3DStamped variable1(rclcpp::Time(12345678, 910111213), uuid_1);
    Position3DStamped variable2(rclcpp::Time(12345678, 910111214), uuid_2);
    EXPECT_NE(variable1.uuid(), variable2.uuid());

    Position3DStamped variable3(rclcpp::Time(12345678, 910111213), uuid_1);
    Position3DStamped variable4(rclcpp::Time(12345679, 910111213), uuid_2);
    EXPECT_NE(variable3.uuid(), variable4.uuid());
  }
}

TEST(Position3DStamped, Stamped)
{
  fuse_core::Variable::SharedPtr base = Position3DStamped::make_shared(rclcpp::Time(12345678, 910111213),
                                                                       fuse_core::uuid::generate("mo"));
  auto derived = std::dynamic_pointer_cast<Position3DStamped>(base);
  ASSERT_TRUE(static_cast<bool>(derived));
  EXPECT_EQ(rclcpp::Time(12345678, 910111213), derived->stamp());
  EXPECT_EQ(fuse_core::uuid::generate("mo"), derived->deviceId());

  auto stamped = std::dynamic_pointer_cast<fuse_variables::Stamped>(base);
  ASSERT_TRUE(static_cast<bool>(stamped));
  EXPECT_EQ(rclcpp::Time(12345678, 910111213), stamped->stamp());
  EXPECT_EQ(fuse_core::uuid::generate("mo"), stamped->deviceId());
}

struct CostFunctor
{
  CostFunctor() {}

  template <typename T> bool operator()(const T* const x, T* residual) const
  {
    residual[0] = x[0] - T(3.0);
    residual[1] = x[1] + T(8.0);
    residual[2] = x[2] - T(3.1);
    return true;
  }
};

TEST(Position3DStamped, Optimization)
{
  // Create a Position3DStamped
  Position3DStamped position(rclcpp::Time(12345678, 910111213));
  position.x() = 1.5;
  position.y() = -3.0;
  position.z() = 0.8;

  // Create a simple a constraint
  ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<CostFunctor, 3, 3>(new CostFunctor());

  // Build the problem.
  ceres::Problem problem;
  problem.AddParameterBlock(
    position.data(),
    position.size());
  std::vector<double*> parameter_blocks;
  parameter_blocks.push_back(position.data());
  problem.AddResidualBlock(
    cost_function,
    nullptr,
    parameter_blocks);

  // Run the solver
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // Check
  EXPECT_NEAR(3.0, position.x(), 1.0e-5);
  EXPECT_NEAR(-8.0, position.y(), 1.0e-5);
  EXPECT_NEAR(3.1, position.z(), 1.0e-5);
}

TEST(Position3DStamped, Serialization)
{
  // Create a Position3DStamped
  Position3DStamped expected(rclcpp::Time(12345678, 910111213), fuse_core::uuid::generate("hal9000"));
  expected.x() = 1.5;
  expected.y() = -3.0;
  expected.z() = 0.8;

  // Serialize the variable into an archive
  std::stringstream stream;
  {
    fuse_core::TextOutputArchive archive(stream);
    expected.serialize(archive);
  }

  // Deserialize a new variable from that same stream
  Position3DStamped actual;
  {
    fuse_core::TextInputArchive archive(stream);
    actual.deserialize(archive);
  }

  // Compare
  EXPECT_EQ(expected.deviceId(), actual.deviceId());
  EXPECT_EQ(expected.stamp(), actual.stamp());
  EXPECT_EQ(expected.x(), actual.x());
  EXPECT_EQ(expected.y(), actual.y());
  EXPECT_EQ(expected.z(), actual.z());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
