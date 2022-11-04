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
#include <fuse_core/transaction.h>
#include <fuse_core/serialization.h>
#include <fuse_core/uuid.h>
#include <test/example_constraint.h>
#include <test/example_variable.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <initializer_list>
#include <vector>

using fuse_core::Transaction;
using fuse_core::UUID;


/**
 * @brief Test that a collection of stamps exist in a Transaction object's involved stamps container
 *
 * Order of the stamps is not important. Extra stamps in the Transaction will return False.
 *
 * @tparam TimeRange   A range or container with objects compatible with a "const rclcpp::Time&" signature
 * @param expected     The set of expected stamps
 * @param transaction  The transaction to test
 * @return             True if the expected stamps, and only the expected stamps, exist in the
 *                     transaction, False otherwise.
 */
template <typename TimeRange>
bool testInvolvedStamps(const TimeRange& expected, const Transaction& transaction)
{
  auto range = transaction.involvedStamps();
  if (std::distance(expected.begin(), expected.end()) != std::distance(range.begin(), range.end()))
  {
    return false;
  }

  for (auto iter = range.begin(); iter != range.end(); ++iter)
  {
    const auto& actual_stamp = *iter;

    bool found = false;
    for (const auto& expected_stamp : expected)
    {
      if (actual_stamp == expected_stamp)
      {
        found = true;
        break;
      }
    }

    if (!found)
    {
      return false;
    }
  }

  // All checks passed
  return true;
}

/**
 * @brief Test that a collection of constraints exist in a Transaction object's added container
 *
 * Order of the constraints is not important. Extra constraints in the Transaction will return False.
 *
 * @tparam ConstraintRange  A range or container with objects compatible with a "const ExampleConstraint&" signature
 * @param expected          The set of expected added constraints
 * @param transaction       The transaction to test
 * @return                  True if the expected constraints, and only the expected constraints, exist in the
 *                          transaction, False otherwise.
 */
template <typename ConstraintRange>
bool testAddedConstraints(const ConstraintRange& expected, const Transaction& transaction)
{
  auto range = transaction.addedConstraints();
  if (std::distance(expected.begin(), expected.end()) != std::distance(range.begin(), range.end()))
  {
    return false;
  }

  for (auto iter = range.begin(); iter != range.end(); ++iter)
  {
    const auto& actual_constraint = dynamic_cast<const ExampleConstraint&>(*iter);

    bool found = false;
    for (const auto& expected_constraint : expected)
    {
      if (actual_constraint.uuid() == expected_constraint.uuid())
      {
        found = true;
        bool is_equal = true;
        is_equal = is_equal && (expected_constraint.type() == actual_constraint.type());
        is_equal = is_equal && (expected_constraint.variables().size() == actual_constraint.variables().size());
        for (size_t i = 0; i < expected_constraint.variables().size(); ++i)
        {
          is_equal = is_equal && (expected_constraint.variables().at(i) == actual_constraint.variables().at(i));
        }
        const auto& expected_derived = dynamic_cast<const ExampleConstraint&>(expected_constraint);
        is_equal = is_equal && (expected_derived.data == actual_constraint.data);

        if (!is_equal)
        {
          return false;
        }
      }
    }

    if (!found)
    {
      return false;
    }
  }

  // All checks passed
  return true;
}

/**
 * @brief Test that a collection of constraint UUIDs exist in a Transaction object's removed container
 *
 * Order of the constraint UUIDs is not important. Extra constraint UUIDs in the Transaction will return False.
 *
 * @tparam UuidRange   A range or container with objects compatible with a "const fuse_core::UUID&" signature
 * @param expected     The set of expected removed constraint UUIDs
 * @param transaction  The transaction to test
 * @return             True if the expected constraints, and only the expected constraints, exist in the
 *                     transaction, False otherwise.
 */
template <typename UuidRange>
bool testRemovedConstraints(const UuidRange& expected, const Transaction& transaction)
{
  auto range = transaction.removedConstraints();
  if (std::distance(expected.begin(), expected.end()) != std::distance(range.begin(), range.end()))
  {
    return false;
  }

  for (auto iter = range.begin(); iter != range.end(); ++iter)
  {
    const auto& actual_constraint_uuid = *iter;

    bool found = false;
    for (const auto& expected_constraint_uuid : expected)
    {
      if (actual_constraint_uuid == expected_constraint_uuid)
      {
        found = true;
        break;
      }
    }

    if (!found)
    {
      return false;
    }
  }

  // All checks passed
  return true;
}

/**
 * @brief Test that a collection of variables exist in a Transaction object's added container
 *
 * Order of the variables is not important. Extra variables in the Transaction will return False.
 *
 * @tparam VariableRange  A range or container with objects compatible with a "const ExampleVariable&" signature
 * @param expected        The set of expected added variables
 * @param transaction     The transaction to test
 * @return                True if the expected variables, and only the expected variables, exist in the
 *                        transaction, False otherwise.
 */
template <typename VariableRange>
bool testAddedVariables(const VariableRange& expected, const Transaction& transaction)
{
  auto range = transaction.addedVariables();
  if (std::distance(expected.begin(), expected.end()) != std::distance(range.begin(), range.end()))
  {
    return false;
  }

  for (auto iter = range.begin(); iter != range.end(); ++iter)
  {
    const auto& actual_variable = dynamic_cast<const ExampleVariable&>(*iter);

    bool found = false;
    for (const auto& expected_variable : expected)
    {
      if (actual_variable.uuid() == expected_variable.uuid())
      {
        found = true;
        bool is_equal = true;
        is_equal = is_equal && (expected_variable.type() == actual_variable.type());
        is_equal = is_equal && (expected_variable.size() == actual_variable.size());
        is_equal = is_equal && (expected_variable.data()[0] == actual_variable.data()[0]);

        if (!is_equal)
        {
          return false;
        }
      }
    }

    if (!found)
    {
      return false;
    }
  }

  // All checks passed
  return true;
}

/**
 * @brief Test that a collection of variable UUIDs exist in a Transaction object's removed container
 *
 * Order of the variable UUIDs is not important. Extra variable UUIDs in the Transaction will return False.
 *
 * @tparam UuidRange   A range or container with objects compatible with a "const fuse_core::UUID&" signature
 * @param expected     The set of expected removed variable UUIDs
 * @param transaction  The transaction to test
 * @return             True if the expected variables, and only the expected variables, exist in the
 *                     transaction, False otherwise.
 */
template <typename UuidRange>
bool testRemovedVariables(const UuidRange& expected, const Transaction& transaction)
{
  auto range = transaction.removedVariables();
  if (std::distance(expected.begin(), expected.end()) != std::distance(range.begin(), range.end()))
  {
    return false;
  }

  for (auto iter = range.begin(); iter != range.end(); ++iter)
  {
    const auto& actual_variable_uuid = *iter;

    bool found = false;
    for (const auto& expected_variable_uuid : expected)
    {
      if (actual_variable_uuid == expected_variable_uuid)
      {
        found = true;
        break;
      }
    }

    if (!found)
    {
      return false;
    }
  }

  // All checks passed
  return true;
}

TEST(Transaction, Empty)
{
  // The default constructed transaction must be empty
  {
    Transaction transaction;

    EXPECT_TRUE(transaction.empty());
  }

  // A transaction with added constraints cannot be empty
  {
    const auto variable_uuid = fuse_core::uuid::generate();
    const auto constraint =
        ExampleConstraint::make_shared("test", std::initializer_list<UUID>{ variable_uuid });  // NOLINT

    Transaction transaction;
    transaction.addConstraint(constraint);

    EXPECT_FALSE(transaction.empty());
  }

  // A transaction with removed constraints cannot be empty
  {
    const auto constraint_uuid = fuse_core::uuid::generate();

    Transaction transaction;
    transaction.removeConstraint(constraint_uuid);

    EXPECT_FALSE(transaction.empty());
  }

  // A transaction with added variables cannot be empty
  {
    const auto variable = ExampleVariable::make_shared();

    Transaction transaction;
    transaction.addVariable(variable);

    EXPECT_FALSE(transaction.empty());
  }

  // A transaction with removed variables cannot be empty
  {
    const auto variable_uuid = fuse_core::uuid::generate();

    Transaction transaction;
    transaction.removeVariable(variable_uuid);

    EXPECT_FALSE(transaction.empty());
  }

  // A transaction with involved stamps cannot be empty
  {
    const rclcpp::Time involved_stamp(12345, 6789);

    Transaction transaction;
    transaction.addInvolvedStamp(involved_stamp);

    EXPECT_FALSE(transaction.empty());
  }
}

TEST(Transaction, AddConstraint)
{
  // Add a single constraint and verify it exists in the added constraints
  {
    UUID variable_uuid = fuse_core::uuid::generate();
    auto constraint = ExampleConstraint::make_shared("test", std::initializer_list<UUID>{variable_uuid});  // NOLINT

    Transaction transaction;
    transaction.addConstraint(constraint);

    std::vector<ExampleConstraint> expected_constraints;
    expected_constraints.push_back(*constraint);
    EXPECT_TRUE(testAddedConstraints(expected_constraints, transaction));
  }

  // Add the same constraint multiple times. Verify only one constraint exists in the Transaction.
  {
    UUID variable_uuid = fuse_core::uuid::generate();
    auto constraint = ExampleConstraint::make_shared("test", std::initializer_list<UUID>{variable_uuid});  // NOLINT

    Transaction transaction;
    transaction.addConstraint(constraint);
    transaction.addConstraint(constraint);
    transaction.addConstraint(constraint);

    std::vector<ExampleConstraint> expected_constraints;
    expected_constraints.push_back(*constraint);
    EXPECT_TRUE(testAddedConstraints(expected_constraints, transaction));
  }

  // Add multiple constraints. Verify they all exist in the Transaction.
  {
    UUID variable1_uuid = fuse_core::uuid::generate();
    auto constraint1 = ExampleConstraint::make_shared("test", std::initializer_list<UUID>{variable1_uuid});  // NOLINT
    UUID variable2_uuid = fuse_core::uuid::generate();
    auto constraint2 = ExampleConstraint::make_shared("test", std::initializer_list<UUID>{variable2_uuid});  // NOLINT
    UUID variable3_uuid = fuse_core::uuid::generate();
    auto constraint3 = ExampleConstraint::make_shared("test", std::initializer_list<UUID>{variable3_uuid});  // NOLINT

    Transaction transaction;
    transaction.addConstraint(constraint1);
    transaction.addConstraint(constraint2);
    transaction.addConstraint(constraint3);

    std::vector<ExampleConstraint> expected_constraints;
    expected_constraints.push_back(*constraint1);
    expected_constraints.push_back(*constraint2);
    expected_constraints.push_back(*constraint3);
    EXPECT_TRUE(testAddedConstraints(expected_constraints, transaction));
  }

  // Add a constraint that is marked for removal. The constraint should not be added, and it should also be deleted
  // from the 'removed' container.
  {
    UUID variable1_uuid = fuse_core::uuid::generate();
    auto constraint1 = ExampleConstraint::make_shared("test", std::initializer_list<UUID>{variable1_uuid});  // NOLINT

    UUID constraint2_uuid = fuse_core::uuid::generate();

    Transaction transaction;
    transaction.removeConstraint(constraint1->uuid());
    transaction.removeConstraint(constraint2_uuid);
    transaction.addConstraint(constraint1);

    std::vector<ExampleConstraint> added_constraints;  // empty
    std::vector<UUID> removed_constraints;
    removed_constraints.push_back(constraint2_uuid);
    EXPECT_TRUE(testAddedConstraints(added_constraints, transaction));
    EXPECT_TRUE(testRemovedConstraints(removed_constraints, transaction));
  }

  // Test the overwrite flag
  {
    // Create and add the constraint to the transaction
    UUID variable_uuid = fuse_core::uuid::generate();
    auto constraint1 = ExampleConstraint::make_shared("test", std::initializer_list<UUID>{variable_uuid});  // NOLINT
    constraint1->data = 1.0;

    Transaction transaction;
    transaction.addConstraint(constraint1);

    // Verify it exists
    {
      std::vector<ExampleConstraint> expected_constraints;
      expected_constraints.push_back(*constraint1);
      EXPECT_TRUE(testAddedConstraints(expected_constraints, transaction));
    }

    // Add a second constraint with the same UUID but with a different data value
    auto constraint2 = ExampleConstraint::make_shared(*constraint1);
    constraint2->data = 2.0;

    // Do not overwrite the constraint
    transaction.addConstraint(constraint2, false);

    // Verify the data value is unchanged
    {
      std::vector<ExampleConstraint> expected_constraints;
      expected_constraints.push_back(*constraint1);
      EXPECT_TRUE(testAddedConstraints(expected_constraints, transaction));
    }

    // Overwrite the constraint
    transaction.addConstraint(constraint2, true);

    // Verify the data value is updated
    {
      std::vector<ExampleConstraint> expected_constraints;
      expected_constraints.push_back(*constraint2);
      EXPECT_TRUE(testAddedConstraints(expected_constraints, transaction));
    }
  }
}

TEST(Transaction, RemoveConstraint)
{
  // Mark a single constraint for removal and verify it exists in the removed constraints
  {
    UUID constraint_uuid = fuse_core::uuid::generate();

    Transaction transaction;
    transaction.removeConstraint(constraint_uuid);

    std::vector<UUID> expected_constraints;
    expected_constraints.push_back(constraint_uuid);
    EXPECT_TRUE(testRemovedConstraints(expected_constraints, transaction));
  }

  // Mark the same constraint for removal multiple times. Verify only one constraint exists in removed constraints
  {
    UUID constraint_uuid = fuse_core::uuid::generate();

    Transaction transaction;
    transaction.removeConstraint(constraint_uuid);
    transaction.removeConstraint(constraint_uuid);
    transaction.removeConstraint(constraint_uuid);

    std::vector<UUID> expected_constraints;
    expected_constraints.push_back(constraint_uuid);
    EXPECT_TRUE(testRemovedConstraints(expected_constraints, transaction));
  }

  // Mark multiple constraints for removal. Verify they all exist in removed constraints
  {
    UUID constraint1_uuid = fuse_core::uuid::generate();
    UUID constraint2_uuid = fuse_core::uuid::generate();
    UUID constraint3_uuid = fuse_core::uuid::generate();

    Transaction transaction;
    transaction.removeConstraint(constraint1_uuid);
    transaction.removeConstraint(constraint2_uuid);
    transaction.removeConstraint(constraint3_uuid);

    std::vector<UUID> expected_constraints;
    expected_constraints.push_back(constraint1_uuid);
    expected_constraints.push_back(constraint2_uuid);
    expected_constraints.push_back(constraint3_uuid);
    EXPECT_TRUE(testRemovedConstraints(expected_constraints, transaction));
  }

  // Mark a constraint for removal that is in the added constraints. The constraint should not be marked for removal;
  // instead it should be deleted from the added constraints.
  {
    UUID variable1_uuid = fuse_core::uuid::generate();
    auto constraint1 = ExampleConstraint::make_shared("test", std::initializer_list<UUID>{variable1_uuid});  // NOLINT

    UUID variable2_uuid = fuse_core::uuid::generate();
    auto constraint2 = ExampleConstraint::make_shared("test", std::initializer_list<UUID>{variable2_uuid});  // NOLINT

    Transaction transaction;
    transaction.addConstraint(constraint1);
    transaction.addConstraint(constraint2);
    transaction.removeConstraint(constraint1->uuid());

    // Test
    std::vector<ExampleConstraint> expected_added_constraints;
    expected_added_constraints.push_back(*constraint2);
    EXPECT_TRUE(testAddedConstraints(expected_added_constraints, transaction));

    std::vector<UUID> expected_removed_constraints;
    EXPECT_TRUE(testRemovedConstraints(expected_removed_constraints, transaction));
  }
}

TEST(Transaction, AddVariable)
{
  // Add a single variable and verify it exists in the added variables
  {
    auto variable = ExampleVariable::make_shared();

    Transaction transaction;
    transaction.addVariable(variable);

    std::vector<ExampleVariable> expected_variables;
    expected_variables.push_back(*variable);
    EXPECT_TRUE(testAddedVariables(expected_variables, transaction));
  }

  // Add the same variable multiple times. Verify only one variable exists in the added variables.
  {
    auto variable = ExampleVariable::make_shared();

    Transaction transaction;
    transaction.addVariable(variable);
    transaction.addVariable(variable);
    transaction.addVariable(variable);

    std::vector<ExampleVariable> expected_variables;
    expected_variables.push_back(*variable);
    EXPECT_TRUE(testAddedVariables(expected_variables, transaction));
  }

  // Add multiple constraints. Verify they all exist in the Transaction.
  {
    auto variable1 = ExampleVariable::make_shared();
    auto variable2 = ExampleVariable::make_shared();
    auto variable3 = ExampleVariable::make_shared();

    Transaction transaction;
    transaction.addVariable(variable1);
    transaction.addVariable(variable2);
    transaction.addVariable(variable3);

    std::vector<ExampleVariable> expected_variables;
    expected_variables.push_back(*variable1);
    expected_variables.push_back(*variable2);
    expected_variables.push_back(*variable3);
    EXPECT_TRUE(testAddedVariables(expected_variables, transaction));
  }

  // Add a variable that is marked for removal. The variable should not be added, and it should also be deleted
  // from the 'removed' variables.
  {
    auto variable1 = ExampleVariable::make_shared();
    UUID variable2_uuid = fuse_core::uuid::generate();

    Transaction transaction;
    transaction.removeVariable(variable1->uuid());
    transaction.removeVariable(variable2_uuid);
    transaction.addVariable(variable1);

    // Test
    std::vector<ExampleVariable> expected_added_variables;
    EXPECT_TRUE(testAddedVariables(expected_added_variables, transaction));

    std::vector<UUID> expected_removed_variables;
    expected_removed_variables.push_back(variable2_uuid);
    EXPECT_TRUE(testRemovedVariables(expected_removed_variables, transaction));
  }

  // Test the overwrite flag
  {
    // Create a variable and add it to a transaction
    auto variable1 = ExampleVariable::make_shared();
    variable1->data()[0] = 1.0;

    Transaction transaction;
    transaction.addVariable(variable1);

    // Verify the variable exists
    {
      std::vector<ExampleVariable> expected_variables;
      expected_variables.push_back(*variable1);
      EXPECT_TRUE(testAddedVariables(expected_variables, transaction));
    }

    // Create a second variable with the same UUID but different data valeu
    auto variable2 = ExampleVariable::make_shared(*variable1);
    variable2->data()[0] = 2.0;

    // Do not overwrite the existing variable
    transaction.addVariable(variable2, false);

    // Verify the variable1 values still exist
    {
      std::vector<ExampleVariable> expected_variables;
      expected_variables.push_back(*variable1);
      EXPECT_TRUE(testAddedVariables(expected_variables, transaction));
    }

    // Overwrite the existing variable
    transaction.addVariable(variable2, true);

    // Verify the variable2 values now exist
    {
      std::vector<ExampleVariable> expected_variables;
      expected_variables.push_back(*variable2);
      EXPECT_TRUE(testAddedVariables(expected_variables, transaction));
    }
  }
}

TEST(Transaction, RemoveVariable)
{
  // Mark a single variable for removal and verify it exists in removed variables
  {
    UUID variable_uuid = fuse_core::uuid::generate();

    Transaction transaction;
    transaction.removeVariable(variable_uuid);

    std::vector<UUID> expected_variables;
    expected_variables.push_back(variable_uuid);
    EXPECT_TRUE(testRemovedVariables(expected_variables, transaction));
  }

  // Mark the same variable for removal multiple times. Verify only one variable exists in removed constraints
  {
    UUID variable_uuid = fuse_core::uuid::generate();

    Transaction transaction;
    transaction.removeVariable(variable_uuid);
    transaction.removeVariable(variable_uuid);
    transaction.removeVariable(variable_uuid);

    std::vector<UUID> expected_variables;
    expected_variables.push_back(variable_uuid);
    EXPECT_TRUE(testRemovedVariables(expected_variables, transaction));
  }

  // Mark multiple variables for removal. Verify they all exist in removed variables
  {
    UUID variable1_uuid = fuse_core::uuid::generate();
    UUID variable2_uuid = fuse_core::uuid::generate();
    UUID variable3_uuid = fuse_core::uuid::generate();

    Transaction transaction;
    transaction.removeVariable(variable1_uuid);
    transaction.removeVariable(variable2_uuid);
    transaction.removeVariable(variable3_uuid);

    std::vector<UUID> expected_variables;
    expected_variables.push_back(variable1_uuid);
    expected_variables.push_back(variable2_uuid);
    expected_variables.push_back(variable3_uuid);
    EXPECT_TRUE(testRemovedVariables(expected_variables, transaction));
  }

  // Mark a variable for removal that is in the added variables. The variable should not be marked for removal;
  // instead it should be deleted from the added variables.
  {
    auto variable1 = ExampleVariable::make_shared();
    auto variable2 = ExampleVariable::make_shared();

    Transaction transaction;
    transaction.addVariable(variable1);
    transaction.addVariable(variable2);
    transaction.removeVariable(variable1->uuid());

    // Test
    std::vector<ExampleVariable> expected_added_variables;
    expected_added_variables.push_back(*variable2);
    EXPECT_TRUE(testAddedVariables(expected_added_variables, transaction));

    std::vector<UUID> expected_removed_variables;
    EXPECT_TRUE(testRemovedVariables(expected_removed_variables, transaction));
  }
}

TEST(Transaction, Merge)
{
  // Create two transactions with different info
  rclcpp::Time involved_stamp1(12345, 6789);
  rclcpp::Time involved_stamp2(12346, 6789);
  rclcpp::Time involved_stamp3(12347, 6789);

  UUID variable1_uuid = fuse_core::uuid::generate();
  UUID variable2_uuid = fuse_core::uuid::generate();
  UUID variable3_uuid = fuse_core::uuid::generate();
  auto added_constraint1 = ExampleConstraint::make_shared("test", std::initializer_list<UUID>{variable1_uuid});  // NOLINT
  auto added_constraint2 = ExampleConstraint::make_shared("test", std::initializer_list<UUID>{variable2_uuid});  // NOLINT
  auto added_constraint3 = ExampleConstraint::make_shared("test", std::initializer_list<UUID>{variable3_uuid});  // NOLINT

  UUID removed_constraint1 = fuse_core::uuid::generate();
  UUID removed_constraint2 = fuse_core::uuid::generate();
  UUID removed_constraint3 = fuse_core::uuid::generate();

  auto added_variable1 = ExampleVariable::make_shared();
  auto added_variable2 = ExampleVariable::make_shared();
  auto added_variable3 = ExampleVariable::make_shared();

  UUID removed_variable1 = fuse_core::uuid::generate();
  UUID removed_variable2 = fuse_core::uuid::generate();
  UUID removed_variable3 = fuse_core::uuid::generate();

  Transaction transaction1;
  transaction1.stamp(involved_stamp2);
  transaction1.addInvolvedStamp(involved_stamp1);
  transaction1.addInvolvedStamp(involved_stamp2);
  transaction1.addConstraint(added_constraint1);
  transaction1.addConstraint(added_constraint2);
  transaction1.removeConstraint(removed_constraint1);
  transaction1.addVariable(added_variable2);
  transaction1.addVariable(added_variable3);
  transaction1.removeVariable(removed_variable2);
  transaction1.removeVariable(removed_variable3);

  Transaction transaction2;
  transaction2.stamp(involved_stamp3);
  transaction2.addInvolvedStamp(involved_stamp2);
  transaction2.addInvolvedStamp(involved_stamp3);
  transaction2.addConstraint(added_constraint2);
  transaction2.addConstraint(added_constraint3);
  transaction2.removeConstraint(removed_constraint2);
  transaction2.removeConstraint(removed_constraint3);
  transaction2.addVariable(added_variable1);
  transaction2.addVariable(added_variable3);
  transaction2.removeVariable(removed_variable1);
  transaction2.removeVariable(removed_variable2);

  // Merge the two transactions
  transaction1.merge(transaction2);

  // Verify the correct combination now exists in transaction1
  std::vector<rclcpp::Time> expected_involved_stamps;
  expected_involved_stamps.push_back(involved_stamp1);
  expected_involved_stamps.push_back(involved_stamp2);
  expected_involved_stamps.push_back(involved_stamp3);
  EXPECT_TRUE(testInvolvedStamps(expected_involved_stamps, transaction1));

  std::vector<ExampleConstraint> expected_added_constraints;
  expected_added_constraints.push_back(*added_constraint1);
  expected_added_constraints.push_back(*added_constraint2);
  expected_added_constraints.push_back(*added_constraint3);
  EXPECT_TRUE(testAddedConstraints(expected_added_constraints, transaction1));

  std::vector<UUID> expected_removed_constraints;
  expected_removed_constraints.push_back(removed_constraint1);
  expected_removed_constraints.push_back(removed_constraint2);
  expected_removed_constraints.push_back(removed_constraint3);
  EXPECT_TRUE(testRemovedConstraints(expected_removed_constraints, transaction1));

  std::vector<ExampleVariable> expected_added_variables;
  expected_added_variables.push_back(*added_variable1);
  expected_added_variables.push_back(*added_variable2);
  expected_added_variables.push_back(*added_variable3);
  EXPECT_TRUE(testAddedVariables(expected_added_variables, transaction1));

  std::vector<UUID> expected_removed_variables;
  expected_removed_variables.push_back(removed_variable1);
  expected_removed_variables.push_back(removed_variable2);
  expected_removed_variables.push_back(removed_variable3);
  EXPECT_TRUE(testRemovedVariables(expected_removed_variables, transaction1));

  EXPECT_EQ(std::max(involved_stamp2, involved_stamp3), transaction1.stamp());
}

TEST(Transaction, Clone)
{
  // Create two transactions with different info
  rclcpp::Time involved_stamp1(12345, 6789);
  rclcpp::Time involved_stamp2(12346, 6789);

  UUID variable1_uuid = fuse_core::uuid::generate();
  UUID variable2_uuid = fuse_core::uuid::generate();
  auto added_constraint1 = ExampleConstraint::make_shared("test", std::initializer_list<UUID>{variable1_uuid});  // NOLINT
  auto added_constraint2 = ExampleConstraint::make_shared("test", std::initializer_list<UUID>{variable2_uuid});  // NOLINT

  UUID removed_constraint1 = fuse_core::uuid::generate();
  UUID removed_constraint2 = fuse_core::uuid::generate();

  auto added_variable1 = ExampleVariable::make_shared();
  auto added_variable2 = ExampleVariable::make_shared();

  UUID removed_variable1 = fuse_core::uuid::generate();
  UUID removed_variable2 = fuse_core::uuid::generate();

  Transaction transaction1;
  transaction1.addInvolvedStamp(involved_stamp1);
  transaction1.addInvolvedStamp(involved_stamp2);
  transaction1.addConstraint(added_constraint1);
  transaction1.addConstraint(added_constraint2);
  transaction1.removeConstraint(removed_constraint1);
  transaction1.removeConstraint(removed_constraint2);
  transaction1.addVariable(added_variable1);
  transaction1.addVariable(added_variable2);
  transaction1.removeVariable(removed_variable1);
  transaction1.removeVariable(removed_variable2);

  // Clone the transaction
  auto transaction2 = transaction1.clone();

  // Verify the correct combination now exists in transaction2
  std::vector<rclcpp::Time> expected_involved_stamps;
  expected_involved_stamps.push_back(involved_stamp1);
  expected_involved_stamps.push_back(involved_stamp2);
  EXPECT_TRUE(testInvolvedStamps(expected_involved_stamps, *transaction2));

  std::vector<ExampleConstraint> expected_added_constraints;
  expected_added_constraints.push_back(*added_constraint1);
  expected_added_constraints.push_back(*added_constraint2);
  EXPECT_TRUE(testAddedConstraints(expected_added_constraints, *transaction2));

  std::vector<UUID> expected_removed_constraints;
  expected_removed_constraints.push_back(removed_constraint1);
  expected_removed_constraints.push_back(removed_constraint2);
  EXPECT_TRUE(testRemovedConstraints(expected_removed_constraints, *transaction2));

  std::vector<ExampleVariable> expected_added_variables;
  expected_added_variables.push_back(*added_variable1);
  expected_added_variables.push_back(*added_variable2);
  EXPECT_TRUE(testAddedVariables(expected_added_variables, *transaction2));

  std::vector<UUID> expected_removed_variables;
  expected_removed_variables.push_back(removed_variable1);
  expected_removed_variables.push_back(removed_variable2);
  EXPECT_TRUE(testRemovedVariables(expected_removed_variables, *transaction2));
}

TEST(Transaction, Serialize)
{
  // Create a transaction
  rclcpp::Time involved_stamp1(12345, 6789);
  rclcpp::Time involved_stamp2(12346, 6789);

  UUID variable1_uuid = fuse_core::uuid::generate();
  UUID variable2_uuid = fuse_core::uuid::generate();
  auto added_constraint1 = ExampleConstraint::make_shared("test", std::initializer_list<UUID>{variable1_uuid});  // NOLINT
  auto added_constraint2 = ExampleConstraint::make_shared("test", std::initializer_list<UUID>{variable2_uuid});  // NOLINT

  UUID removed_constraint1 = fuse_core::uuid::generate();
  UUID removed_constraint2 = fuse_core::uuid::generate();

  auto added_variable1 = ExampleVariable::make_shared();
  auto added_variable2 = ExampleVariable::make_shared();

  UUID removed_variable1 = fuse_core::uuid::generate();
  UUID removed_variable2 = fuse_core::uuid::generate();

  Transaction expected;
  expected.addInvolvedStamp(involved_stamp1);
  expected.addInvolvedStamp(involved_stamp2);
  expected.addConstraint(added_constraint1);
  expected.addConstraint(added_constraint2);
  expected.removeConstraint(removed_constraint1);
  expected.removeConstraint(removed_constraint2);
  expected.addVariable(added_variable1);
  expected.addVariable(added_variable2);
  expected.removeVariable(removed_variable1);
  expected.removeVariable(removed_variable2);

  // Serialize the object into an archive
  std::stringstream stream;
  {
    fuse_core::TextOutputArchive archive(stream);
    expected.serialize(archive);
  }

  // Deserialize a new object from that same stream
  Transaction actual;
  {
    fuse_core::TextInputArchive archive(stream);
    actual.deserialize(archive);
  }

  // Compare
  EXPECT_TRUE(testInvolvedStamps(expected.involvedStamps(), actual));
  EXPECT_TRUE(testAddedConstraints(expected.addedConstraints(), actual));
  EXPECT_TRUE(testRemovedConstraints(expected.removedConstraints(), actual));
  EXPECT_TRUE(testAddedVariables(expected.addedVariables(), actual));
  EXPECT_TRUE(testRemovedVariables(expected.removedVariables(), actual));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
