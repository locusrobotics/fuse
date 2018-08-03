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
#include <fuse_core/constraint.h>
#include <fuse_core/uuid.h>
#include <fuse_core/variable.h>
#include <fuse_graphs/hash_graph.h>
#include <test/covariance_constraint.h>
#include <test/example_constraint.h>
#include <test/example_variable.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <utility>
#include <vector>


/**
 * @brief Static variable to hold the last unit test error description
 */
static std::string failure_description;

/**
 * @brief Compare all the properties of two Variable objects
 * @return True if all the properties match, false otherwise
 */
bool compareVariables(const fuse_core::Variable& expected, const fuse_core::Variable& actual)
{
  failure_description = "";
  bool variables_equal = true;
  if (expected.type() != actual.type())
  {
    variables_equal = false;
    failure_description += "The variables have different types.\n"
      "  expected type is '" + expected.type() + "'\n"
      "    actual type is '" + actual.type() + "'\n";
  }
  if (expected.size() != actual.size())
  {
    variables_equal = false;
    failure_description += "The variables have different sizes.\n"
      "  expected size is '" + std::to_string(expected.size()) + "'\n"
      "    actual size is '" + std::to_string(actual.size()) + "'\n";
  }
  if (expected.uuid() != actual.uuid())
  {
    variables_equal = false;
    failure_description += "The variables have different UUIDs.\n"
      "  expected UUID is '" + fuse_core::uuid::to_string(expected.uuid()) + "'\n"
      "    actual UUID is '" + fuse_core::uuid::to_string(actual.uuid()) + "'\n";
  }
  for (size_t i = 0; i < expected.size(); ++i)
  {
    if (expected.data()[i] != actual.data()[i])
    {
      variables_equal = false;
      failure_description += "The variables have different values.\n"
        "  expected data(" + std::to_string(i) + ") is '" + std::to_string(expected.data()[i]) + "'\n"
        "    actual data(" + std::to_string(i) + ") is '" + std::to_string(actual.data()[i]) + "'\n";
    }
  }
  return variables_equal;
}

/**
 * @brief Compare all the properties of two Constraint objects
 * @return True if all the properties match, false otherwise
 */
bool compareConstraints(const fuse_core::Constraint& expected, const fuse_core::Constraint& actual)
{
  failure_description = "";
  bool constraints_equal = true;
  if (expected.type() != actual.type())
  {
    constraints_equal = false;
    failure_description += "The constraints have different types.\n"
      "  expected type is '" + expected.type() + "'\n"
      "    actual type is '" + actual.type() + "'\n";
  }
  if (expected.uuid() != actual.uuid())
  {
    constraints_equal = false;
    failure_description += "The constraints have different UUIDs.\n"
      "  expected UUID is '" + fuse_core::uuid::to_string(expected.uuid()) + "'\n"
      "    actual UUID is '" + fuse_core::uuid::to_string(actual.uuid()) + "'\n";
  }
  if (expected.variables().size() != actual.variables().size())
  {
    constraints_equal = false;
    failure_description += "The constraints involve a different number of variables.\n"
      "  expected variable count is '" + std::to_string(expected.variables().size()) + "'\n"
      "    actual variable count is '" + std::to_string(actual.variables().size()) + "'\n";
  }
  for (size_t i = 0; i < expected.variables().size(); ++i)
  {
    if (expected.variables().at(i) != actual.variables().at(i))
    {
      constraints_equal = false;
      std::string i_str = std::to_string(i);
      failure_description += "The constraints involve different variable UUIDs.\n"
        "  expected variables(" + i_str + ") is '" + fuse_core::uuid::to_string(expected.variables()[i]) + "'\n"
        "    actual variables(" + i_str + ") is '" + fuse_core::uuid::to_string(actual.variables()[i]) + "'\n";
    }
  }
  return constraints_equal;
}

TEST(HashGraph, AddVariable)
{
  // Test adding variables to the graph
  // Also tests the variableExists() function

  // Create the graph
  fuse_graphs::HashGraph graph;

  // Create a few variables
  auto variable1 = ExampleVariable::make_shared();
  variable1->data()[0] = 1.0;

  auto variable2 = ExampleVariable::make_shared();
  variable2->data()[0] = 2.5;

  // Verify neither variable exists
  EXPECT_FALSE(graph.variableExists(variable1->uuid()));
  EXPECT_FALSE(graph.variableExists(variable2->uuid()));

  // Add variable1 to the graph
  EXPECT_TRUE(graph.addVariable(variable1));

  // Verify only variable1 exists
  EXPECT_TRUE(graph.variableExists(variable1->uuid()));
  EXPECT_FALSE(graph.variableExists(variable2->uuid()));

  // Add variable2 to the graph
  EXPECT_TRUE(graph.addVariable(variable2));

  // Verify both variables exist
  EXPECT_TRUE(graph.variableExists(variable1->uuid()));
  EXPECT_TRUE(graph.variableExists(variable2->uuid()));

  // Add variable1 again. This should return false -- the variable already exists
  EXPECT_FALSE(graph.addVariable(variable1));

  // Add a newly-created but identical variable. This should also return false. Different instances of a variable
  // with the same settings/uuid are considered the same variable.
  fuse_core::Variable::SharedPtr variable3 = variable1->clone();
  EXPECT_FALSE(graph.addVariable(variable3));
  // Variable3 should say it exists
  EXPECT_TRUE(graph.variableExists(variable3->uuid()));
}

TEST(HashGraph, RemoveVariable)
{
  // Test removing variables from the graph

  // Create the graph
  fuse_graphs::HashGraph graph;

  // Create a few variables
  auto variable1 = ExampleVariable::make_shared();
  variable1->data()[0] = 1.0;

  auto variable2 = ExampleVariable::make_shared();
  variable2->data()[0] = 2.5;

  // Add the variables to the graph
  EXPECT_TRUE(graph.addVariable(variable1));
  EXPECT_TRUE(graph.addVariable(variable2));

  // Verify both variable exists
  EXPECT_TRUE(graph.variableExists(variable1->uuid()));
  EXPECT_TRUE(graph.variableExists(variable2->uuid()));

  // Remove variable1 from the graph
  EXPECT_TRUE(graph.removeVariable(variable1->uuid()));

  // Verify only variable2 exists
  EXPECT_FALSE(graph.variableExists(variable1->uuid()));
  EXPECT_TRUE(graph.variableExists(variable2->uuid()));

  // Remove variable2 from the graph
  EXPECT_TRUE(graph.removeVariable(variable2->uuid()));

  // Verify neither variables exist
  EXPECT_FALSE(graph.variableExists(variable1->uuid()));
  EXPECT_FALSE(graph.variableExists(variable2->uuid()));

  // Attempt to remove variable1 again. It should return false.
  EXPECT_FALSE(graph.removeVariable(variable1->uuid()));

  // Add a constraint involving one of the variables
  auto constraint1 = ExampleConstraint::make_shared(variable1->uuid());
  graph.addVariable(variable1);
  graph.addVariable(variable2);
  graph.addConstraint(constraint1);

  // Attempt to remove variable1. This should throw a logic_error exception as the variable is still involved
  // in a constraint.
  EXPECT_THROW(graph.removeVariable(variable1->uuid()), std::logic_error);

  // Verify both variables still exist
  EXPECT_TRUE(graph.variableExists(variable1->uuid()));
  EXPECT_TRUE(graph.variableExists(variable2->uuid()));

  // Remove the constraint and retry. It should succeed now.
  graph.removeConstraint(constraint1->uuid());
  EXPECT_TRUE(graph.removeVariable(variable1->uuid()));

  // Verify that variables1 no longer exists
  EXPECT_FALSE(graph.variableExists(variable1->uuid()));
  EXPECT_TRUE(graph.variableExists(variable2->uuid()));
}

TEST(HashGraph, GetVariable)
{
  // Test accessing a single variables from the graph

  // Create the graph
  fuse_graphs::HashGraph graph;

  // Create a few variables
  auto variable1 = ExampleVariable::make_shared();
  variable1->data()[0] = 1.0;
  graph.addVariable(variable1);

  auto variable2 = ExampleVariable::make_shared();
  variable2->data()[0] = 2.5;
  graph.addVariable(variable2);

  // Verify all of the variables are available
  const fuse_core::Variable& actual1 = graph.getVariable(variable1->uuid());
  EXPECT_TRUE(compareVariables(*variable1, actual1)) << failure_description;

  const fuse_core::Variable& actual2 = graph.getVariable(variable2->uuid());
  EXPECT_TRUE(compareVariables(*variable2, actual2)) << failure_description;
}

TEST(HashGraph, GetVariables)
{
  // Test accessing the variables collection from the graph

  // Create the graph
  fuse_graphs::HashGraph graph;

  // Create a few variables
  auto variable1 = ExampleVariable::make_shared();
  variable1->data()[0] = 1.0;
  graph.addVariable(variable1);

  auto variable2 = ExampleVariable::make_shared();
  variable2->data()[0] = 2.5;
  graph.addVariable(variable2);

  auto variable3 = ExampleVariable::make_shared();
  variable3->data()[0] = -1.2;
  graph.addVariable(variable3);

  // Access all the variables using the getVariables() member function
  auto variables = graph.getVariables();
  ASSERT_EQ(3, std::distance(variables.begin(), variables.end()));

  // Verify we received the correct variables
  for (const auto& actual : variables)
  {
    if (actual.uuid() == variable1->uuid())
    {
      EXPECT_TRUE(compareVariables(*variable1, actual)) << failure_description;
      continue;
    }
    if (actual.uuid() == variable2->uuid())
    {
      EXPECT_TRUE(compareVariables(*variable2, actual)) << failure_description;
      continue;
    }
    if (actual.uuid() == variable3->uuid())
    {
      EXPECT_TRUE(compareVariables(*variable3, actual)) << failure_description;
      continue;
    }
    // The actual variable is not in the expected set. Fail the test.
    FAIL() << "The actual variable '" << actual.uuid() << "' was not in the expected collection.";
  }
}

TEST(HashGraph, AddConstraint)
{
  // Test adding constraints to the graph
  // Also tests the constraintExists() function

  // Create the graph
  fuse_graphs::HashGraph graph;

  // Add a few variables
  auto variable1 = ExampleVariable::make_shared();
  variable1->data()[0] = 1.0;
  graph.addVariable(variable1);

  auto variable2 = ExampleVariable::make_shared();
  variable2->data()[0] = 2.5;
  graph.addVariable(variable2);

  // Create two different constraints
  auto constraint1 = ExampleConstraint::make_shared(variable1->uuid());

  auto constraint2 = ExampleConstraint::make_shared(variable2->uuid());

  // Verify neither constraint exists
  EXPECT_FALSE(graph.constraintExists(constraint1->uuid()));
  EXPECT_FALSE(graph.constraintExists(constraint2->uuid()));

  // Add constraint1 to the graph
  EXPECT_TRUE(graph.addConstraint(constraint1));

  // Verify only constraint1 exists
  EXPECT_TRUE(graph.constraintExists(constraint1->uuid()));
  EXPECT_FALSE(graph.constraintExists(constraint2->uuid()));

  // Add constraint2 to the graph
  EXPECT_TRUE(graph.addConstraint(constraint2));

  // Verify both constraints exist
  EXPECT_TRUE(graph.constraintExists(constraint1->uuid()));
  EXPECT_TRUE(graph.constraintExists(constraint2->uuid()));

  // Add constraint1 again. This should return false -- the constraint already exists
  EXPECT_FALSE(graph.addConstraint(constraint1));

  // Add a newly-created but identical constraint. This should also return false. Different instances of a constraint
  // with the same settings/uuid are considered the same constraint.
  fuse_core::Constraint::SharedPtr constraint3 = constraint1->clone();
  EXPECT_FALSE(graph.addConstraint(constraint3));
  // constraint3 should say it exists
  EXPECT_TRUE(graph.constraintExists(constraint3->uuid()));

  // Create a constraint involving a new variable
  auto variable3 = ExampleVariable::make_shared();
  variable3->data()[0] = -1.2;
  auto constraint4 = ExampleConstraint::make_shared(variable3->uuid());

  // Attempt to add constraint4. This should throw a logic_error, as the variable has not been added yet.
  EXPECT_THROW(graph.addConstraint(constraint4), std::logic_error);
}

TEST(HashGraph, RemoveConstraint)
{
  // Test removing constraints from the graph

  // Create the graph
  fuse_graphs::HashGraph graph;

  // Add a few variables
  auto variable1 = ExampleVariable::make_shared();
  variable1->data()[0] = 1.0;
  graph.addVariable(variable1);

  auto variable2 = ExampleVariable::make_shared();
  variable2->data()[0] = 2.5;
  graph.addVariable(variable2);

  // Add a few constraints
  auto constraint1 = ExampleConstraint::make_shared(variable1->uuid());
  graph.addConstraint(constraint1);

  auto constraint2 = ExampleConstraint::make_shared(variable2->uuid());
  graph.addConstraint(constraint2);

  // Verify both constraints exists
  EXPECT_TRUE(graph.constraintExists(constraint1->uuid()));
  EXPECT_TRUE(graph.constraintExists(constraint2->uuid()));

  // Remove constraint1 from the graph
  EXPECT_TRUE(graph.removeConstraint(constraint1->uuid()));

  // Verify only constraint2 exists
  EXPECT_FALSE(graph.constraintExists(constraint1->uuid()));
  EXPECT_TRUE(graph.constraintExists(constraint2->uuid()));

  // Remove constraint2 from the graph
  EXPECT_TRUE(graph.removeConstraint(constraint2->uuid()));

  // Verify neither constraint exists
  EXPECT_FALSE(graph.constraintExists(constraint1->uuid()));
  EXPECT_FALSE(graph.constraintExists(constraint2->uuid()));

  // Attempt to remove constraint1 again. This should return false
  EXPECT_FALSE(graph.removeConstraint(constraint1->uuid()));
}

TEST(HashGraph, GetConstraint)
{
  // Test accessing the constraints in the graph

  // Create the graph
  fuse_graphs::HashGraph graph;

  // Add a few variables
  auto variable1 = ExampleVariable::make_shared();
  variable1->data()[0] = 1.0;
  graph.addVariable(variable1);

  auto variable2 = ExampleVariable::make_shared();
  variable2->data()[0] = 2.5;
  graph.addVariable(variable2);

  // Add a few constraints
  auto constraint1 = ExampleConstraint::make_shared(variable1->uuid());
  graph.addConstraint(constraint1);

  auto constraint2 = ExampleConstraint::make_shared(variable2->uuid());
  graph.addConstraint(constraint2);

  // Verify all of the constraints are available
  const fuse_core::Constraint& actual1 = graph.getConstraint(constraint1->uuid());
  EXPECT_TRUE(compareConstraints(*constraint1, actual1)) << failure_description;

  const fuse_core::Constraint& actual2 = graph.getConstraint(constraint2->uuid());
  EXPECT_TRUE(compareConstraints(*constraint2, actual2)) << failure_description;
}

TEST(HashGraph, GetConstraints)
{
  // Test accessing the constraints in the graph

  // Create the graph
  fuse_graphs::HashGraph graph;

  // Add a few variables
  auto variable1 = ExampleVariable::make_shared();
  graph.addVariable(variable1);

  auto variable2 = ExampleVariable::make_shared();
  graph.addVariable(variable2);

  // Add a few constraints
  auto constraint1 = ExampleConstraint::make_shared(variable1->uuid());
  graph.addConstraint(constraint1);

  auto constraint2 = ExampleConstraint::make_shared(variable2->uuid());
  graph.addConstraint(constraint2);

  auto constraint3 = ExampleConstraint::make_shared(variable1->uuid());
  graph.addConstraint(constraint3);

  // Access all constraints
  auto constraints = graph.getConstraints();
  ASSERT_EQ(3, std::distance(constraints.begin(), constraints.end()));

  // Verify we received the correct constraints
  for (const auto& actual : constraints)
  {
    if (actual.uuid() == constraint1->uuid())
    {
      EXPECT_TRUE(compareConstraints(*constraint1, actual)) << failure_description;
      continue;
    }
    if (actual.uuid() == constraint2->uuid())
    {
      EXPECT_TRUE(compareConstraints(*constraint2, actual)) << failure_description;
      continue;
    }
    if (actual.uuid() == constraint3->uuid())
    {
      EXPECT_TRUE(compareConstraints(*constraint3, actual)) << failure_description;
      continue;
    }
    // The constraint was not one of the expected constraints. Fail the test.
    FAIL() << "The actual constraint '" << actual.uuid() << "' was not in the expected collection.";
  }
}

TEST(HashGraph, Optimize)
{
  // Test optimizing a set of variables/constraints

  // Create the graph
  fuse_graphs::HashGraph graph;

  // Add a few variables
  auto variable1 = ExampleVariable::make_shared();
  variable1->data()[0] = 1.0;
  graph.addVariable(variable1);

  auto variable2 = ExampleVariable::make_shared();
  variable2->data()[0] = 2.5;
  graph.addVariable(variable2);

  // Add a few constraints
  auto constraint1 = ExampleConstraint::make_shared(variable1->uuid());
  constraint1->data = 5.0;
  graph.addConstraint(constraint1);

  auto constraint2 = ExampleConstraint::make_shared(variable2->uuid());
  constraint2->data = -3.0;
  graph.addConstraint(constraint2);

  // Optimize the constraints and variables.
  EXPECT_NO_THROW(graph.optimize());

  // Verify the correct solution was obtained.
  EXPECT_NEAR(5.0, variable1->data()[0], 1.0e-7);
  EXPECT_NEAR(-3.0, variable2->data()[0], 1.0e-7);
}

TEST(HashGraph, HoldVariable)
{
  // Test placing a variable on hold. The value of the variable should remain constant even after the optimization

  // Create the graph
  fuse_graphs::HashGraph graph;

  // Add a few variables
  auto variable1 = ExampleVariable::make_shared();
  variable1->data()[0] = 1.0;
  graph.addVariable(variable1);

  auto variable2 = ExampleVariable::make_shared();
  variable2->data()[0] = 2.5;
  graph.addVariable(variable2);

  // Add a few constraints
  auto constraint1 = ExampleConstraint::make_shared(variable1->uuid());
  constraint1->data = 5.0;
  graph.addConstraint(constraint1);

  auto constraint2 = ExampleConstraint::make_shared(variable2->uuid());
  constraint2->data = -3.0;
  graph.addConstraint(constraint2);

  // Place variable1 on hold
  EXPECT_NO_THROW(graph.holdVariable(variable1->uuid()));

  // Optimize the constraints and variables.
  EXPECT_NO_THROW(graph.optimize());

  // Verify the correct solution was obtained.
  // Variable1 should not have changed from its original value
  EXPECT_NEAR(1.0, variable1->data()[0], 1.0e-7);
  EXPECT_NEAR(-3.0, variable2->data()[0], 1.0e-7);
}

TEST(HashGraph, GetCovariance)
{
  // Create variables that match the Ceres unit test
  auto x = ExampleVariable::make_shared(2);
  x->data()[0] = 1;
  x->data()[1] = 1;
  auto y = ExampleVariable::make_shared(3);
  y->data()[0] = 2;
  y->data()[1] = 2;
  y->data()[2] = 2;
  auto z = ExampleVariable::make_shared(1);
  z->data()[0] = 3;
  // Create a constraint that matches the Ceres unit test
  auto constraint = CovarianceConstraint::make_shared(x->uuid(), y->uuid(), z->uuid());

  // Add the variables and constraints to the graph
  fuse_graphs::HashGraph graph;
  graph.addVariable(x);
  graph.addVariable(y);
  graph.addVariable(z);
  graph.addConstraint(constraint);

  // Solve the graph to generate the required variable values
  graph.optimize();

  // Test providing an empty covariance request
  {
    // Create an empty covariance request
    std::vector<std::pair<fuse_core::UUID, fuse_core::UUID> > covariance_requests;

    // The empty request should not throw an error
    std::vector<std::vector<double> > covariance_matrices;
    EXPECT_NO_THROW(graph.getCovariance(covariance_requests, covariance_matrices));

    // The output covariances should be empty
    EXPECT_EQ(0, covariance_matrices.size());
  }

  // Test extraction of the covariance from the graph
  // Adapted from the Ceres unit tests:
  // https://github.com/ceres-solver/ceres-solver/blob/master/internal/ceres/covariance_test.cc#L598
  {
    // Compute selected covariance blocks
    std::vector<std::pair<fuse_core::UUID, fuse_core::UUID> > covariance_requests;
    covariance_requests.emplace_back(x->uuid(), x->uuid());
    covariance_requests.emplace_back(x->uuid(), y->uuid());
    covariance_requests.emplace_back(y->uuid(), x->uuid());  // Adding both versions Cov(X,Y) and Cov(Y,X)
    covariance_requests.emplace_back(x->uuid(), y->uuid());  // Adding a duplicate to verify everything still works.
    covariance_requests.emplace_back(z->uuid(), y->uuid());
    std::vector<std::vector<double> > covariance_matrices;
    graph.getCovariance(covariance_requests, covariance_matrices);
    const std::vector<double>& actual0 = covariance_matrices.at(0);
    const std::vector<double>& actual1 = covariance_matrices.at(1);
    const std::vector<double>& actual2 = covariance_matrices.at(2);
    const std::vector<double>& actual3 = covariance_matrices.at(3);
    const std::vector<double>& actual4 = covariance_matrices.at(4);

    // Compare with the expected blocks
    //  full covariance = {
    //     7.0747e-02,  -8.4923e-03,   1.6821e-02,   3.3643e-02,   5.0464e-02,  -1.5809e-02,
    //    -8.4923e-03,   8.1352e-02,   2.4758e-02,   4.9517e-02,   7.4275e-02,   1.2978e-02,
    //     1.6821e-02,   2.4758e-02,   2.4904e-01,  -1.9271e-03,  -2.8906e-03,  -6.5325e-05,
    //     3.3643e-02,   4.9517e-02,  -1.9271e-03,   2.4615e-01,  -5.7813e-03,  -1.3065e-04,
    //     5.0464e-02,   7.4275e-02,  -2.8906e-03,  -5.7813e-03,   2.4133e-01,  -1.9598e-04,
    //    -1.5809e-02,   1.2978e-02,  -6.5325e-05,  -1.3065e-04,  -1.9598e-04,   3.9544e-02,
    //  };
    // Extracting the covariance blocks from the full covariance matrix
    // [ XX (2x2), XY(2x3), XZ (2x1)]
    // [ YX (3x2), YY(3x3), YZ (3x1)]
    // [ ZX (1x2), ZY(1x3), ZZ (1x1)]
    std::vector<double> expected0 = { 7.0747e-02, -8.4923e-03, -8.4923e-03, 8.1352e-02};  // XX
    std::vector<double> expected1 = { 1.6821e-02,  3.3643e-02,  5.0464e-02, 2.4758e-02, 4.9517e-02, 7.4275e-02};  // XY
    std::vector<double> expected2 = { 1.6821e-02,  2.4758e-02,  3.3643e-02, 4.9517e-02, 5.0464e-02, 7.4275e-02};  // YX
    std::vector<double> expected3 = { 1.6821e-02,  3.3643e-02,  5.0464e-02, 2.4758e-02, 4.9517e-02, 7.4275e-02};  // XY
    std::vector<double> expected4 = {-6.5325e-05, -1.3065e-04, -1.9598e-04};  // ZY

    ASSERT_EQ(expected0.size(), actual0.size());
    for (size_t i = 0; i < expected0.size(); ++i)
    {
      EXPECT_NEAR(expected0[i], actual0[i], 1.0e-5);
    }

    ASSERT_EQ(expected1.size(), actual1.size());
    for (size_t i = 0; i < expected1.size(); ++i)
    {
      EXPECT_NEAR(expected1[i], actual1[i], 1.0e-5);
    }

    ASSERT_EQ(expected2.size(), actual2.size());
    for (size_t i = 0; i < expected2.size(); ++i)
    {
      EXPECT_NEAR(expected2[i], actual2[i], 1.0e-5);
    }

    ASSERT_EQ(expected3.size(), actual3.size());
    for (size_t i = 0; i < expected3.size(); ++i)
    {
      EXPECT_NEAR(expected3[i], actual3[i], 1.0e-5);
    }

    ASSERT_EQ(expected4.size(), actual4.size());
    for (size_t i = 0; i < expected4.size(); ++i)
    {
      EXPECT_NEAR(expected4[i], actual4[i], 1.0e-5);
    }
  }
}

TEST(HashGraph, MarginalizeVariable)
{
  // TODO(swilliams): Write a marginalization unit test after the function has been implemented
}

TEST(HashGraph, MarginalizeVariables)
{
  // TODO(swilliams): Write a marginalization unit test after the function has been implemented
}

TEST(HashGraph, Copy)
{
    // Create the graph
  fuse_graphs::HashGraph graph;

  // Add a few variables
  auto variable1 = ExampleVariable::make_shared();
  variable1->data()[0] = 1.0;
  graph.addVariable(variable1);

  auto variable2 = ExampleVariable::make_shared();
  variable2->data()[0] = 2.5;
  graph.addVariable(variable2);

  // Add a few constraints
  auto constraint1 = ExampleConstraint::make_shared(variable1->uuid());
  constraint1->data = 5.0;
  graph.addConstraint(constraint1);

  auto constraint2 = ExampleConstraint::make_shared(variable2->uuid());
  constraint2->data = -3.0;
  graph.addConstraint(constraint2);

  // Test the copy constructor
  {
    fuse_graphs::HashGraph other(graph);
    // Verify the copy
    for (const auto& constraint : graph.getConstraints())
    {
      EXPECT_TRUE(other.constraintExists(constraint.uuid()));
    }
    for (const auto& variable : graph.getVariables())
    {
      EXPECT_TRUE(other.variableExists(variable.uuid()));
    }
    // Modify the variable values of the 'other' graph
    other.optimize();
    // The variables should have been cloned/copied, so modifying 'other' should not modify 'graph'.
    EXPECT_NEAR(1.0, graph.getVariable(variable1->uuid()).data()[0], 1.0e-7);
    EXPECT_NEAR(5.0, other.getVariable(variable1->uuid()).data()[0], 1.0e-7);
  }

  // Test the assignment operator
  {
    fuse_graphs::HashGraph other;
    other = graph;
    // Verify the copy
    for (const auto& constraint : graph.getConstraints())
    {
      EXPECT_TRUE(other.constraintExists(constraint.uuid()));
    }
    for (const auto& variable : graph.getVariables())
    {
      EXPECT_TRUE(other.variableExists(variable.uuid()));
    }
    // Modify the variable values of the 'other' graph
    other.optimize();
    // The variables should have been cloned/copied, so modifying 'other' should not modify 'graph'.
    EXPECT_NEAR(1.0, graph.getVariable(variable1->uuid()).data()[0], 1.0e-7);
    EXPECT_NEAR(5.0, other.getVariable(variable1->uuid()).data()[0], 1.0e-7);
  }

  // Test the clone method
  {
    auto other = graph.clone();
    // Verify the copy
    for (const auto& constraint : graph.getConstraints())
    {
      EXPECT_TRUE(other->constraintExists(constraint.uuid()));
    }
    for (const auto& variable : graph.getVariables())
    {
      EXPECT_TRUE(other->variableExists(variable.uuid()));
    }
    // Modify the variable values of the 'other' graph
    other->optimize();
    // The variables should have been cloned/copied, so modifying 'other' should not modify 'graph'.
    EXPECT_NEAR(1.0, graph.getVariable(variable1->uuid()).data()[0], 1.0e-7);
    EXPECT_NEAR(5.0, other->getVariable(variable1->uuid()).data()[0], 1.0e-7);
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
