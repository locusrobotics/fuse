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
#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/stamped.h>
#include <ros/time.h>

#include <ceres/autodiff_cost_function.h>
#include <ceres/cost_function_to_functor.h>
#include <ceres/problem.h>
#include <ceres/rotation.h>
#include <ceres/solver.h>
#include <gtest/gtest.h>

#include <vector>


using fuse_variables::Orientation3DStamped;

TEST(Orientation3DStamped, Type)
{
  Orientation3DStamped variable(ros::Time(12345678, 910111213));
  EXPECT_EQ("fuse_variables::Orientation3DStamped", variable.type());
}

TEST(Orientation3DStamped, UUID)
{
  // Verify two orientations at the same timestamp produce the same UUID
  {
    Orientation3DStamped variable1(ros::Time(12345678, 910111213));
    Orientation3DStamped variable2(ros::Time(12345678, 910111213));
    EXPECT_EQ(variable1.uuid(), variable2.uuid());
  }

  auto uuid_1 = fuse_core::uuid::generate("test_hardware1");
  auto uuid_2 = fuse_core::uuid::generate("test_hardware2");

  // Verify two orientations at the same timestamp and same hardware ID produce the same UUID
  {
    Orientation3DStamped variable1(ros::Time(12345678, 910111213), uuid_1);
    Orientation3DStamped variable2(ros::Time(12345678, 910111213), uuid_1);
    EXPECT_EQ(variable1.uuid(), variable2.uuid());
  }

  // Verify two orientations with the same timestamp but different hardware IDs generate different UUIDs
  {
    Orientation3DStamped variable1(ros::Time(12345678, 910111213), uuid_1);
    Orientation3DStamped variable2(ros::Time(12345678, 910111213), uuid_2);
    EXPECT_NE(variable1.uuid(), variable2.uuid());
  }

  // Verify two orientations with the same hardware ID and different timestamps produce different UUIDs
  {
    Orientation3DStamped variable1(ros::Time(12345678, 910111213), uuid_1);
    Orientation3DStamped variable2(ros::Time(12345678, 910111214), uuid_1);
    EXPECT_NE(variable1.uuid(), variable2.uuid());

    Orientation3DStamped variable3(ros::Time(12345678, 910111213), uuid_1);
    Orientation3DStamped variable4(ros::Time(12345679, 910111213), uuid_1);
    EXPECT_NE(variable3.uuid(), variable4.uuid());
  }

  // Verify two orientations with different hardware IDs and different timestamps produce different UUIDs
  {
    Orientation3DStamped variable1(ros::Time(12345678, 910111213), uuid_1);
    Orientation3DStamped variable2(ros::Time(12345678, 910111214), uuid_2);
    EXPECT_NE(variable1.uuid(), variable2.uuid());

    Orientation3DStamped variable3(ros::Time(12345678, 910111213), uuid_1);
    Orientation3DStamped variable4(ros::Time(12345679, 910111213), uuid_2);
    EXPECT_NE(variable3.uuid(), variable4.uuid());
  }
}

TEST(Orientation3DStamped, Stamped)
{
  fuse_core::Variable::SharedPtr base = Orientation3DStamped::make_shared(ros::Time(12345678, 910111213),
                                                                          fuse_core::uuid::generate("mo"));
  auto derived = std::dynamic_pointer_cast<Orientation3DStamped>(base);
  ASSERT_TRUE(static_cast<bool>(derived));
  EXPECT_EQ(ros::Time(12345678, 910111213), derived->stamp());
  EXPECT_EQ(fuse_core::uuid::generate("mo"), derived->deviceId());

  auto stamped = std::dynamic_pointer_cast<fuse_variables::Stamped>(base);
  ASSERT_TRUE(static_cast<bool>(stamped));
  EXPECT_EQ(ros::Time(12345678, 910111213), stamped->stamp());
  EXPECT_EQ(fuse_core::uuid::generate("mo"), stamped->deviceId());
}

struct QuaternionCostFunction
{
  explicit QuaternionCostFunction(double *observation)
  {
    observation_[0] = observation[0];
    observation_[1] = observation[1];
    observation_[2] = observation[2];
    observation_[3] = observation[3];
  }

  template <typename T>
  bool operator()(const T* quaternion, T* residual) const
  {
    T inverse_quaternion[4] =
    {
      quaternion[0],
      -quaternion[1],
      -quaternion[2],
      -quaternion[3]
    };

    T observation[4] =
    {
      T(observation_[0]),
      T(observation_[1]),
      T(observation_[2]),
      T(observation_[3])
    };

    T output[4];

    ceres::QuaternionProduct(observation, inverse_quaternion, output);

    // Residual can just be the imaginary components
    residual[0] = output[1];
    residual[1] = output[2];
    residual[2] = output[3];

    return true;
  }

  double observation_[4];
};

TEST(Orientation3DStamped, Optimization)
{
  // Create an Orientation3DStamped with R, P, Y values of 10, -20, 30 degrees
  Orientation3DStamped orientation(ros::Time(12345678, 910111213));
  orientation.w() = 0.952;
  orientation.x() = 0.038;
  orientation.y() = -0.189;
  orientation.z() = 0.239;

  // Create a simple a constraint with an identity quaternion
  double target_quat[4] = {1.0, 0.0, 0.0, 0.0};
  ceres::CostFunction* cost_function =
    new ceres::AutoDiffCostFunction<QuaternionCostFunction, 3, 4>(new QuaternionCostFunction(target_quat));

  // Build the problem.
  ceres::Problem problem;
  problem.AddParameterBlock(
    orientation.data(),
    orientation.size(),
    orientation.localParameterization());
  std::vector<double*> parameter_blocks;
  parameter_blocks.push_back(orientation.data());
  problem.AddResidualBlock(
    cost_function,
    nullptr,
    parameter_blocks);

  // Run the solver
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // Check
  EXPECT_NEAR(target_quat[0], orientation.w(), 1.0e-3);
  EXPECT_NEAR(target_quat[1], orientation.x(), 1.0e-3);
  EXPECT_NEAR(target_quat[2], orientation.y(), 1.0e-3);
  EXPECT_NEAR(target_quat[3], orientation.z(), 1.0e-3);
}

TEST(Orientation3DStamped, Euler)
{
  const double RAD_TO_DEG = 180.0 / M_PI;

  // Create an Orientation3DStamped with R, P, Y values of 10, -20, 30 degrees
  Orientation3DStamped orientation_r(ros::Time(12345678, 910111213));
  orientation_r.w() = 0.9961947;
  orientation_r.x() = 0.0871557;
  orientation_r.y() = 0.0;
  orientation_r.z() = 0.0;

  EXPECT_NEAR(10.0, RAD_TO_DEG * orientation_r.roll(), 1e-5);

  Orientation3DStamped orientation_p(ros::Time(12345678, 910111213));
  orientation_p.w() = 0.9848078;
  orientation_p.x() = 0.0;
  orientation_p.y() = -0.1736482;
  orientation_p.z() = 0.0;

  EXPECT_NEAR(-20.0, RAD_TO_DEG * orientation_p.pitch(), 1e-5);

  Orientation3DStamped orientation_y(ros::Time(12345678, 910111213));
  orientation_y.w() = 0.9659258;
  orientation_y.x() = 0.0;
  orientation_y.y() = 0.0;
  orientation_y.z() = 0.258819;

  EXPECT_NEAR(30.0, RAD_TO_DEG * orientation_y.yaw(), 1e-5);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
