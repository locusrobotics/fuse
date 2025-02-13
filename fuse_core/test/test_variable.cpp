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
#include <ceres/autodiff_cost_function.h>
#include <ceres/cost_function_to_functor.h>
#include <ceres/problem.h>
#include <ceres/rotation.h>
#include <ceres/solver.h>
#include <gtest/gtest.h>

#include <fstream>
#include <vector>

#include "example_variable.hpp"

TEST(Variable, Type) {
  ExampleVariable variable;
  ASSERT_EQ("ExampleVariable", variable.type());
}

TEST(LegacyVariable, Serialization) {
  // Create an Orientation3DStamped
  LegacyVariable expected;
  expected.data()[0] = 0.952;
  expected.data()[1] = 0.038;
  expected.data()[2] = -0.189;
  expected.data()[3] = 0.239;

  // Serialize the variable into an archive
  std::stringstream stream;
  {
    fuse_core::TextOutputArchive archive(stream);
    expected.serialize(archive);
  }

  // Deserialize a new variable from that same stream
  LegacyVariable actual;
  {
    fuse_core::TextInputArchive archive(stream);
    actual.deserialize(archive);
  }

  // Compare
  EXPECT_EQ(expected.data()[0], actual.data()[0]);
  EXPECT_EQ(expected.data()[1], actual.data()[1]);
  EXPECT_EQ(expected.data()[2], actual.data()[2]);
  EXPECT_EQ(expected.data()[3], actual.data()[3]);
}

#if CERES_SUPPORTS_MANIFOLDS
struct QuaternionCostFunction
{
  explicit QuaternionCostFunction(double * observation)
  {
    observation_[0] = observation[0];
    observation_[1] = observation[1];
    observation_[2] = observation[2];
    observation_[3] = observation[3];
  }

  template<typename T>
  bool operator()(const T * quaternion, T * residual) const
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

TEST(LegacyVariable, ManifoldAdapter) {
  // Create an Orientation3DStamped with R, P, Y values of 10, -20, 30 degrees
  LegacyVariable orientation;
  orientation.data()[0] = 0.952;
  orientation.data()[1] = 0.038;
  orientation.data()[2] = -0.189;
  orientation.data()[3] = 0.239;

  // Create a simple a constraint with an identity quaternion
  double target_quat[4] = {1.0, 0.0, 0.0, 0.0};
  ceres::CostFunction * cost_function = new ceres::AutoDiffCostFunction<QuaternionCostFunction, 3,
      4>(new QuaternionCostFunction(target_quat));

  // Build the problem.
  ceres::Problem problem;
  problem.AddParameterBlock(orientation.data(), orientation.size(), orientation.manifold());
  std::vector<double *> parameter_blocks;
  parameter_blocks.push_back(orientation.data());
  problem.AddResidualBlock(
    cost_function,
    nullptr,
    parameter_blocks);

  // Run the solver
  ceres::Solver::Summary summary;
  ceres::Solver::Options options;
  ceres::Solve(options, &problem, &summary);

  // Check
  EXPECT_NEAR(target_quat[0], orientation.data()[0], 1.0e-3);
  EXPECT_NEAR(target_quat[1], orientation.data()[1], 1.0e-3);
  EXPECT_NEAR(target_quat[2], orientation.data()[2], 1.0e-3);
  EXPECT_NEAR(target_quat[3], orientation.data()[3], 1.0e-3);
}

TEST(LegacyVariable, Deserialization) {
  // Test loading a LegacyVariable that was serialized without manifold support.
  // Verify the deserialization works, and that a manifold pointer can be generated.

  LegacyVariable expected;
  expected.data()[0] = 0.952;
  expected.data()[1] = 0.038;
  expected.data()[2] = -0.189;
  expected.data()[3] = 0.239;

  // The LegacyVariable was serialized from an old version of fuse using the following code
  // {
  //   std::ofstream output_file("legacy_variable_deserialization.txt");
  //   fuse_core::TextOutputArchive archive(output_file);
  //   expected.serialize(archive);
  // }

  // Deserialize a new variable from the previously serialzied file
  LegacyVariable actual;
  {
    std::ifstream input_file("legacy_variable_deserialization.txt");
    fuse_core::TextInputArchive archive(input_file);
    actual.deserialize(archive);
  }

  // Compare
  EXPECT_EQ(expected.data()[0], actual.data()[0]);
  EXPECT_EQ(expected.data()[1], actual.data()[1]);
  EXPECT_EQ(expected.data()[2], actual.data()[2]);
  EXPECT_EQ(expected.data()[3], actual.data()[3]);

  // Test the manifold interface, and that the Legacy LocalParameterization is wrapped
  // in a ManifoldAdapter
  fuse_core::Manifold * actual_manifold = nullptr;
  ASSERT_NO_THROW(actual_manifold = actual.manifold());
  ASSERT_NE(actual_manifold, nullptr);
  auto actual_manifold_adapter = dynamic_cast<fuse_core::ManifoldAdapter *>(actual_manifold);
  ASSERT_NE(actual_manifold_adapter, nullptr);
}
#endif
