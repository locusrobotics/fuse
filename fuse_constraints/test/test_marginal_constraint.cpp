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
#include <fuse_constraints/marginal_constraint.h>
#include <fuse_core/eigen.h>
#include <fuse_core/eigen_gtest.h>
#include <fuse_core/serialization.h>
#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_2d_stamped.h>
#include <fuse_core/time.h>

#include <gtest/gtest.h>

#include <memory>
#include <vector>


TEST(MarginalConstraint, OneVariable)
{
  // Create a marginal constraint with one variable, no local parameterizations
  std::vector<fuse_variables::Position2DStamped> variables;
  fuse_variables::Position2DStamped x1(rclcpp::Time(1, 0));
  x1.x() = 1.0;
  x1.y() = 2.0;
  variables.push_back(x1);

  std::vector<fuse_core::MatrixXd> A;
  fuse_core::MatrixXd A1(1, 2);
  A1 << 5.0, 8.0;
  A.push_back(A1);

  fuse_core::Vector1d b;
  b << 3.0;

  auto constraint = fuse_constraints::MarginalConstraint(
    "test",
    variables.begin(),
    variables.end(),
    A.begin(),
    A.end(),
    b);

  auto cost_function = constraint.costFunction();

  // Update the variable value
  x1.x() = 4.0;
  x1.y() = 6.0;

  // Compute the actual residuals and jacobians
  std::vector<const double*> variable_values = {x1.data()};
  fuse_core::Vector1d actual_residuals;
  fuse_core::MatrixXd actual_jacobian1(1, 2);
  std::vector<double*> actual_jacobians = {actual_jacobian1.data()};
  cost_function->Evaluate(variable_values.data(), actual_residuals.data(), actual_jacobians.data());

  // Define the expected residuals and jacobians
  fuse_core::Vector1d expected_residuals;
  expected_residuals << 50.0;  // 5.0 * (4.0 - 1.0) + 8.0 * (6.0 - 2.0) + 3.0
  fuse_core::MatrixXd expected_jacobian1(1, 2);
  expected_jacobian1 << 5.0, 8.0;  // Just A1

  EXPECT_MATRIX_NEAR(expected_residuals, actual_residuals, 1.0e-5);
  EXPECT_MATRIX_NEAR(expected_jacobian1, actual_jacobian1, 1.0e-5);

  delete cost_function;
}

TEST(MarginalConstraint, TwoVariables)
{
  // Create a marginal constraint with one variable, no local parameterizations
  std::vector<fuse_variables::Position2DStamped> variables;
  fuse_variables::Position2DStamped x1(rclcpp::Time(1, 0));
  x1.x() = 1.0;
  x1.y() = 2.0;
  variables.push_back(x1);

  fuse_variables::Position2DStamped x2(rclcpp::Time(2, 0));
  x2.x() = 3.0;
  x2.y() = 4.0;
  variables.push_back(x2);

  std::vector<fuse_core::MatrixXd> A;
  fuse_core::MatrixXd A1(1, 2);
  A1 << 5.0, 6.0;
  A.push_back(A1);

  fuse_core::MatrixXd A2(1, 2);
  A2 << 7.0, 8.0;
  A.push_back(A2);

  fuse_core::Vector1d b;
  b << 9.0;

  auto constraint = fuse_constraints::MarginalConstraint(
    "test",
    variables.begin(),
    variables.end(),
    A.begin(),
    A.end(),
    b);

  auto cost_function = constraint.costFunction();

  // Update the variable value
  x1.x() = 10.0;
  x1.y() = 12.0;

  x2.x() = 15.0;
  x2.y() = 18.0;

  // Compute the actual residuals and jacobians
  std::vector<const double*> variable_values = {x1.data(), x2.data()};
  fuse_core::Vector1d actual_residuals;
  fuse_core::MatrixXd actual_jacobian1(1, 2);
  fuse_core::MatrixXd actual_jacobian2(1, 2);
  std::vector<double*> actual_jacobians = {actual_jacobian1.data(), actual_jacobian2.data()};
  cost_function->Evaluate(variable_values.data(), actual_residuals.data(), actual_jacobians.data());

  // Define the expected residuals and jacobians
  fuse_core::Vector1d expected_residuals;
  expected_residuals << 310.0;  // 5 * (10 - 1) + 6 * (12 - 2)   +   7 * (15 - 3) + 8 * (18 - 4)   +   9
  fuse_core::MatrixXd expected_jacobian1(1, 2);
  expected_jacobian1 << 5.0, 6.0;  // Just A1
  fuse_core::MatrixXd expected_jacobian2(1, 2);
  expected_jacobian2 << 7.0, 8.0;  // Just A2

  EXPECT_MATRIX_NEAR(expected_residuals, actual_residuals, 1.0e-5);
  EXPECT_MATRIX_NEAR(expected_jacobian1, actual_jacobian1, 1.0e-5);
  EXPECT_MATRIX_NEAR(expected_jacobian2, actual_jacobian2, 1.0e-5);

  delete cost_function;
}

TEST(MarginalConstraint, LocalParameterization)
{
  // Create a marginal constraint with one variable with a local parameterizations
  std::vector<fuse_variables::Orientation3DStamped> variables;
  fuse_variables::Orientation3DStamped x1(rclcpp::Time(1, 0));
  x1.w() = 0.842614977;
  x1.x() = 0.2;
  x1.y() = 0.3;
  x1.z() = 0.4;
  variables.push_back(x1);

  std::vector<fuse_core::MatrixXd> A;
  fuse_core::MatrixXd A1(1, 3);
  A1 << 5.0, 6.0, 7.0;
  A.push_back(A1);

  fuse_core::Vector1d b;
  b << 8.0;

  auto constraint = fuse_constraints::MarginalConstraint(
    "test",
    variables.begin(),
    variables.end(),
    A.begin(),
    A.end(),
    b);
  auto cost_function = constraint.costFunction();

  // Update the variable value
  // This is a delta of (0.15, -0.2, 0.433012702);
  x1.w() = 0.745561;
  x1.x() = 0.360184;
  x1.y() = 0.194124;
  x1.z() = 0.526043;

  // Compute the actual residuals and jacobians
  std::vector<const double*> variable_values = {x1.data()};
  fuse_core::Vector1d actual_residuals;
  fuse_core::MatrixXd actual_jacobian1(1, 4);
  std::vector<double*> actual_jacobians = {actual_jacobian1.data()};
  cost_function->Evaluate(variable_values.data(), actual_residuals.data(), actual_jacobians.data());

  // Define the expected residuals and jacobians
  fuse_core::Vector1d expected_residuals;
  expected_residuals << 10.581088914;  // 5.0 * 0.15  +  6.0 * -0.2  +  7.0 * 0.433012702   +   8.0
  fuse_core::MatrixXd expected_jacobian1(1, 4);
  expected_jacobian1 << -13.29593, 3.86083, 9.164586, 12.818822;
  // A1 * J_local
  // [5.0, 6.0, 7.0] * [-0.720368,  1.491122,  1.052086,  -0.388248]
  //                   [-0.388248, -1.052086,  1.491122,   0.720368]
  //                   [-1.052086,  0.388248, -0.720368,   1.491122]

  EXPECT_MATRIX_NEAR(expected_residuals, actual_residuals, 1.0e-5);
  EXPECT_MATRIX_NEAR(expected_jacobian1, actual_jacobian1, 1.0e-5);

  delete cost_function;
}

TEST(MarginalConstraint, Serialization)
{
  // Construct a constraint
  std::vector<fuse_variables::Orientation3DStamped> variables;
  fuse_variables::Orientation3DStamped x1(rclcpp::Time(1, 0));
  x1.w() = 0.842614977;
  x1.x() = 0.2;
  x1.y() = 0.3;
  x1.z() = 0.4;
  variables.push_back(x1);

  std::vector<fuse_core::MatrixXd> A;
  fuse_core::MatrixXd A1(1, 3);
  A1 << 5.0, 6.0, 7.0;
  A.push_back(A1);

  fuse_core::Vector1d b;
  b << 8.0;

  auto expected = fuse_constraints::MarginalConstraint(
    "test",
    variables.begin(),
    variables.end(),
    A.begin(),
    A.end(),
    b);

  // Serialize the constraint into an archive
  std::stringstream stream;
  {
    fuse_core::TextOutputArchive archive(stream);
    expected.serialize(archive);
  }

  // Deserialize a new constraint from that same stream
  fuse_constraints::MarginalConstraint actual;
  {
    fuse_core::TextInputArchive archive(stream);
    actual.deserialize(archive);
  }

  // Compare
  EXPECT_EQ(expected.uuid(), actual.uuid());
  EXPECT_EQ(expected.variables(), actual.variables());
  EXPECT_EQ(expected.A(), actual.A());
  EXPECT_EQ(expected.b(), actual.b());
  EXPECT_EQ(expected.x_bar(), actual.x_bar());
  // The shared ptrs will not be the same instances, but they should point to the same types
  using ExpectedLocalParam = fuse_variables::Orientation3DLocalParameterization;
  ASSERT_EQ(expected.localParameterizations().size(), actual.localParameterizations().size());
  for (auto i = 0u; i < actual.localParameterizations().size(); ++i)
  {
    auto actual_derived = std::dynamic_pointer_cast<ExpectedLocalParam>(actual.localParameterizations()[i]);
    EXPECT_TRUE(static_cast<bool>(actual_derived));
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
