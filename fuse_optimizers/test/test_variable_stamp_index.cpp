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
#include <fuse_core/constraint.h>
#include <fuse_core/transaction.h>
#include <fuse_core/serialization.h>
#include <fuse_core/uuid.h>
#include <fuse_core/variable.h>
#include <fuse_optimizers/variable_stamp_index.h>
#include <fuse_variables/stamped.h>
#include <fuse_core/time.h>

#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <gtest/gtest.h>

#include <algorithm>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>


/**
 * @brief Create a simple stamped Variable for testing
 */
class StampedVariable : public fuse_core::Variable, public fuse_variables::Stamped
{
public:
  FUSE_VARIABLE_DEFINITIONS(StampedVariable)

  explicit StampedVariable(const rclcpp::Time& stamp = rclcpp::Time(0, 0)) :
    fuse_core::Variable(fuse_core::uuid::generate()),
    fuse_variables::Stamped(stamp),
    data_{}
  {
  }

  size_t size() const override
  {
    return 1;
  }

  const double* data() const override
  {
    return &data_;
  }

  double* data() override
  {
    return &data_;
  }

  void print(std::ostream& /*stream = std::cout*/) const override
  {
  }

private:
  double data_;

  // Allow Boost Serialization access to private methods
  friend class boost::serialization::access;

  /**
   * @brief The Boost Serialize method that serializes all of the data members in to/out of the archive
   *
   * @param[in/out] archive - The archive object that holds the serialized class members
   * @param[in] version - The version of the archive being read/written. Generally unused.
   */
  template<class Archive>
  void serialize(Archive& archive, const unsigned int /* version */)
  {
    archive & boost::serialization::base_object<fuse_core::Variable>(*this);
    archive & boost::serialization::base_object<fuse_variables::Stamped>(*this);
    archive & data_;
  }
};

BOOST_CLASS_EXPORT(StampedVariable);

/**
 * @brief Create a simple unstamped Variable for testing
 */
class UnstampedVariable : public fuse_core::Variable
{
public:
  FUSE_VARIABLE_DEFINITIONS(UnstampedVariable)

  UnstampedVariable() :
    fuse_core::Variable(fuse_core::uuid::generate()),
    data_{}
  {
  }

  size_t size() const override
  {
    return 1;
  }

  const double* data() const override
  {
    return &data_;
  }

  double* data() override
  {
    return &data_;
  }

  void print(std::ostream& /*stream = std::cout*/) const override
  {
  }

private:
  double data_;

  // Allow Boost Serialization access to private methods
  friend class boost::serialization::access;

  /**
   * @brief The Boost Serialize method that serializes all of the data members in to/out of the archive
   *
   * @param[in/out] archive - The archive object that holds the serialized class members
   * @param[in] version - The version of the archive being read/written. Generally unused.
   */
  template<class Archive>
  void serialize(Archive& archive, const unsigned int /* version */)
  {
    archive & boost::serialization::base_object<fuse_core::Variable>(*this);
    archive & data_;
  }
};

BOOST_CLASS_EXPORT(UnstampedVariable);

/**
 * @brief Create a simple Constraint for testing
 */
class GenericConstraint : public fuse_core::Constraint
{
public:
  FUSE_CONSTRAINT_DEFINITIONS(GenericConstraint)

  GenericConstraint() = default;

  GenericConstraint(const std::string& source, std::initializer_list<fuse_core::UUID> variable_uuids) :
    Constraint(source, variable_uuids)
  {
  }

  explicit GenericConstraint(const std::string& source, const fuse_core::UUID& variable1) :
    fuse_core::Constraint(source, {variable1})
  {
  }

  GenericConstraint(
    const std::string& source,
    const fuse_core::UUID& variable1,
    const fuse_core::UUID& variable2) :
      fuse_core::Constraint(source, {variable1, variable2})
  {
  }

  GenericConstraint(
    const std::string& source,
    const fuse_core::UUID& variable1,
    const fuse_core::UUID& variable2,
    const fuse_core::UUID& variable3) :
      fuse_core::Constraint(source, {variable1, variable2, variable3})
  {
  }

  void print(std::ostream& /*stream = std::cout*/) const override
  {
  }

  ceres::CostFunction* costFunction() const override
  {
    return nullptr;
  }

private:
  // Allow Boost Serialization access to private methods
  friend class boost::serialization::access;

  /**
   * @brief The Boost Serialize method that serializes all of the data members in to/out of the archive
   *
   * @param[in/out] archive - The archive object that holds the serialized class members
   * @param[in] version - The version of the archive being read/written. Generally unused.
   */
  template<class Archive>
  void serialize(Archive& archive, const unsigned int /* version */)
  {
    archive & boost::serialization::base_object<fuse_core::Constraint>(*this);
  }
};

BOOST_CLASS_EXPORT(GenericConstraint);


TEST(VariableStampIndex, Size)
{
  // Create an empty index
  auto index = fuse_optimizers::VariableStampIndex();
  EXPECT_TRUE(index.empty());
  EXPECT_EQ(0u, index.size());

  // Add some variables/constraints to the index
  auto x1 = UnstampedVariable::make_shared();
  auto transaction1 = fuse_core::Transaction();
  transaction1.addVariable(x1);
  index.addNewTransaction(transaction1);

  EXPECT_FALSE(index.empty());
  EXPECT_EQ(1u, index.size());

  // Remove the same variables
  auto transaction2 = fuse_core::Transaction();
  transaction2.removeVariable(x1->uuid());
  index.addNewTransaction(transaction2);

  EXPECT_TRUE(index.empty());
  EXPECT_EQ(0u, index.size());
}

TEST(VariableStampIndex, CurrentStamp)
{
  // Create an empty index
  auto index = fuse_optimizers::VariableStampIndex();

  // Verify the current stamp is 0
  EXPECT_EQ(rclcpp::Time(0, 0, RCL_CLOCK_UNINITIALIZED), index.currentStamp());

  // Add an unstamped variable
  auto x1 = UnstampedVariable::make_shared();
  auto transaction1 = fuse_core::Transaction();
  transaction1.addVariable(x1);
  index.addNewTransaction(transaction1);

  // Verify the current stamp is still 0
  EXPECT_EQ(rclcpp::Time(0, 0, RCL_CLOCK_UNINITIALIZED), index.currentStamp());

  // Add a stamped variable
  auto x2 = StampedVariable::make_shared(rclcpp::Time(1, 0));
  auto transaction2 = fuse_core::Transaction();
  transaction2.addVariable(x2);
  index.addNewTransaction(transaction2);

  // Verify the current stamp is now Time(1, 0)
  EXPECT_EQ(rclcpp::Time(1, 0), index.currentStamp());
}

TEST(VariableStampIndex, Query)
{
  // Create an empty index
  auto index = fuse_optimizers::VariableStampIndex();

  // Add some variables and constraints
  auto x1 = StampedVariable::make_shared(rclcpp::Time(1, 0));
  auto x2 = StampedVariable::make_shared(rclcpp::Time(2, 0));
  auto x3 = StampedVariable::make_shared(rclcpp::Time(3, 0));
  auto l1 = UnstampedVariable::make_shared();
  auto l2 = UnstampedVariable::make_shared();

  auto c1 = GenericConstraint::make_shared("test", x1->uuid(), x2->uuid());
  auto c2 = GenericConstraint::make_shared("test", x2->uuid(), x3->uuid());
  auto c3 = GenericConstraint::make_shared("test", x1->uuid(), l1->uuid());
  auto c4 = GenericConstraint::make_shared("test", x2->uuid(), l1->uuid());
  auto c5 = GenericConstraint::make_shared("test", x3->uuid(), l2->uuid());

  auto transaction = fuse_core::Transaction();
  transaction.addVariable(x1);
  transaction.addVariable(x2);
  transaction.addVariable(x3);
  transaction.addVariable(l1);
  transaction.addVariable(l2);
  transaction.addConstraint(c1);
  transaction.addConstraint(c2);
  transaction.addConstraint(c3);
  transaction.addConstraint(c4);
  transaction.addConstraint(c5);
  index.addNewTransaction(transaction);

  auto expected1 = std::vector<fuse_core::UUID>{};
  auto actual1 = std::vector<fuse_core::UUID>();
  index.query(rclcpp::Time(1, 500000), std::back_inserter(actual1));
  EXPECT_EQ(expected1, actual1);

  auto expected2 = std::vector<fuse_core::UUID>{x1->uuid(), l1->uuid()};
  std::sort(expected2.begin(), expected2.end());
  auto actual2 = std::vector<fuse_core::UUID>();
  index.query(rclcpp::Time(2, 500000), std::back_inserter(actual2));
  std::sort(actual2.begin(), actual2.end());
  EXPECT_EQ(expected2, actual2);
}

TEST(VariableStampIndex, MarginalTransaction)
{
  // Create an empty index
  auto index = fuse_optimizers::VariableStampIndex();

  // Add some variables and constraints
  auto x1 = StampedVariable::make_shared(rclcpp::Time(1, 0));
  auto x2 = StampedVariable::make_shared(rclcpp::Time(2, 0));
  auto x3 = StampedVariable::make_shared(rclcpp::Time(3, 0));
  auto l1 = UnstampedVariable::make_shared();
  auto l2 = UnstampedVariable::make_shared();

  auto c1 = GenericConstraint::make_shared("test", x1->uuid(), x2->uuid());
  auto c2 = GenericConstraint::make_shared("test", x2->uuid(), x3->uuid());
  auto c3 = GenericConstraint::make_shared("test", x1->uuid(), l1->uuid());
  auto c4 = GenericConstraint::make_shared("test", x2->uuid(), l1->uuid());
  auto c5 = GenericConstraint::make_shared("test", x3->uuid(), l2->uuid());

  auto transaction = fuse_core::Transaction();
  transaction.addVariable(x1);
  transaction.addVariable(x2);
  transaction.addVariable(x3);
  transaction.addVariable(l1);
  transaction.addVariable(l2);
  transaction.addConstraint(c1);
  transaction.addConstraint(c2);
  transaction.addConstraint(c3);
  transaction.addConstraint(c4);
  transaction.addConstraint(c5);
  index.addNewTransaction(transaction);

  // Now create a fake marginal transaction. The constraint connections should *not* change the unstamped variable
  // timestamps
  auto marginal = fuse_core::Transaction();
  marginal.removeVariable(x1->uuid());
  marginal.removeConstraint(c1->uuid());
  marginal.removeConstraint(c3->uuid());
  auto m1 = GenericConstraint::make_shared("test", x3->uuid(), l1->uuid());
  marginal.addConstraint(m1);
  index.addMarginalTransaction(marginal);

  // The x1 variable should be removed
  EXPECT_EQ(4u, index.size());

  // And the marginal constraint x3->l1 should not affect future queries
  auto expected = std::vector<fuse_core::UUID>{l1->uuid()};
  std::sort(expected.begin(), expected.end());
  auto actual = std::vector<fuse_core::UUID>();
  index.query(rclcpp::Time(2, 500000), std::back_inserter(actual));
  std::sort(actual.begin(), actual.end());
  EXPECT_EQ(expected, actual);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
