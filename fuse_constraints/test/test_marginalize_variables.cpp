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
#include <fuse_constraints/marginalize_variables.h>
#include <fuse_constraints/uuid_ordering.h>
#include <fuse_core/constraint.h>
#include <fuse_core/macros.h>
#include <fuse_core/uuid.h>
#include <fuse_core/variable.h>
#include <fuse_graphs/hash_graph.h>

#include <ceres/cost_function.h>
#include <gtest/gtest.h>

#include <vector>


/**
 * @brief Create a simple Variable implementation for testing
 */
class GenericVariable : public fuse_core::Variable
{
public:
  SMART_PTR_DEFINITIONS(GenericVariable);

  GenericVariable() :
    Variable(),
    data_{},
    uuid_{fuse_core::uuid::generate()}
  {}

  fuse_core::UUID uuid() const override { return uuid_; }

  size_t size() const override { return 1; }

  const double* data() const override { return &data_; }
  double* data() override { return &data_; }

  void print(std::ostream& stream = std::cout) const override {}

  fuse_core::Variable::UniquePtr clone() const override { return GenericVariable::make_unique(*this); }

protected:
  double data_;
  fuse_core::UUID uuid_;
};

/**
 * @brief Create a simple Constraint implementation for testing
 */
class GenericConstraint : public fuse_core::Constraint
{
public:
  SMART_PTR_DEFINITIONS(GenericConstraint);

  GenericConstraint(std::initializer_list<fuse_core::UUID> variable_uuids) : Constraint(variable_uuids) {}

  explicit GenericConstraint(const fuse_core::UUID& variable1) :
    fuse_core::Constraint{variable1} {}

  GenericConstraint(const fuse_core::UUID& variable1, const fuse_core::UUID& variable2) :
    fuse_core::Constraint{variable1, variable2} {}

  void print(std::ostream& stream = std::cout) const override {}

  ceres::CostFunction* costFunction() const override { return nullptr; }

  fuse_core::Constraint::UniquePtr clone() const override { return GenericConstraint::make_unique(*this); }
};


TEST(MarginalizeVariables, ComputeEliminationOrder)
{
  // Create a graph
  auto graph = fuse_graphs::HashGraph();
  auto x1 = GenericVariable::make_shared();
  auto x2 = GenericVariable::make_shared();
  auto x3 = GenericVariable::make_shared();
  auto l1 = GenericVariable::make_shared();
  auto l2 = GenericVariable::make_shared();
  auto c1 = GenericConstraint::make_shared(x1->uuid());
  auto c2 = GenericConstraint::make_shared(x1->uuid(), x2->uuid());
  auto c3 = GenericConstraint::make_shared(x2->uuid(), x3->uuid());
  auto c4 = GenericConstraint::make_shared(x1->uuid(), l1->uuid());
  auto c5 = GenericConstraint::make_shared(x2->uuid(), l1->uuid());
  auto c6 = GenericConstraint::make_shared(x3->uuid(), l2->uuid());
  graph.addVariable(x1);
  graph.addVariable(x2);
  graph.addVariable(x3);
  graph.addVariable(l1);
  graph.addVariable(l2);
  graph.addConstraint(c1);
  graph.addConstraint(c2);
  graph.addConstraint(c3);
  graph.addConstraint(c4);
  graph.addConstraint(c5);
  graph.addConstraint(c6);

  // Define the set of variables to be marginalized
  auto to_be_marginalized = std::vector<fuse_core::UUID>{x2->uuid(), x1->uuid()};

  // Compute the ordering
  auto actual = fuse_constraints::detail::computeEliminationOrder(to_be_marginalized, graph);

  // Define the expected order
  auto expected = fuse_constraints::UuidOrdering();
  expected.insert(x1->uuid());
  expected.insert(x2->uuid());
  expected.insert(x3->uuid());
  expected.insert(l1->uuid());

  // Check
  ASSERT_EQ(expected.size(), actual.size());
  for (size_t i = 0; i < expected.size(); ++i)
  {
    EXPECT_EQ(expected.at(i), actual.at(i));
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
