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
#include <fuse_constraints/relative_orientation_3d_stamped_constraint.h>
#include <fuse_constraints/uuid_ordering.h>
#include <fuse_core/constraint.h>
#include <fuse_core/macros.h>
#include <fuse_core/uuid.h>
#include <fuse_core/variable.h>
#include <fuse_graphs/hash_graph.h>
#include <fuse_variables/orientation_3d_stamped.h>

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
  auto graph = fuse_graphs::HashGraph();
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

TEST(MarginalizeVariables, Linearize)
{
  // Create a graph with one relative 3D orientation constraint
  auto x1 = fuse_variables::Orientation3DStamped::make_shared(ros::Time(1, 0));
  x1->w() = 0.927362;
  x1->x() = 0.1;
  x1->y() = 0.2;
  x1->z() = 0.3;

  auto x2 = fuse_variables::Orientation3DStamped::make_shared(ros::Time(2, 0));
  x2->w() = 0.848625;
  x2->x() = 0.13798;
  x2->y() = 0.175959;
  x2->z() = 0.479411;

  fuse_core::Vector4d delta;
  delta << 0.979795897, 0.0, 0.0, 0.2;
  fuse_core::Matrix3d cov;
  cov << 1.0, 0.0, 0.0,   0.0, 2.0, 0.0,   0.0, 0.0, 3.0;
  auto constraint = fuse_constraints::RelativeOrientation3DStampedConstraint::make_shared(*x1, *x2, delta, cov);

  auto graph = fuse_graphs::HashGraph();
  graph.addVariable(x1);
  graph.addVariable(x2);
  graph.addConstraint(constraint);

  // Create an elimination order
  auto elimination_order = fuse_constraints::UuidOrdering();
  elimination_order.insert(fuse_core::uuid::generate());  // Add a dummy variable to the elimination order
  elimination_order.insert(x2->uuid());
  elimination_order.insert(fuse_core::uuid::generate());  // Add a dummy variable to the elimination order
  elimination_order.insert(x1->uuid());

  // Compute the linear term
  auto actual = fuse_constraints::detail::linearize(*constraint, graph, elimination_order);

  // Define the expected values
  fuse_core::MatrixXd expected_A0(3, 3);
  expected_A0 << -0.91999992754510684367,    -0.39191852892782985673,    -2.8735440640859089001e-07,
                  0.27712824947756498073,    -0.65053818745824854020,    -2.5352112979770691226e-07,
                  7.1505019336171038447e-08,  2.5546009342625186633e-07, -0.57735026918958520792;

  fuse_core::MatrixXd expected_A1(3, 3);
  expected_A1 <<  0.99999999999996114219,    -1.8482708254510815671e-07, -2.873543621662033587e-07,
                  1.3069243487082160549e-07,  0.70710678118650927004,    -2.5352115484711390536e-07,
                  1.6590414383954588118e-07,  2.0699913566568639567e-07,  0.57735026918958520792;

  fuse_core::VectorXd expected_b(3);
  expected_b << 7.1706607563166841896e-07, -4.0638046747479327072e-07, 2.1341989211309879704e-07;

  // Check
  ASSERT_EQ(2u, actual.variables.size());
  EXPECT_EQ(3u, actual.variables[0]);
  EXPECT_EQ(1u, actual.variables[1]);

  Eigen::IOFormat clean(4, 0, ", ", "\n", "[", "]");
  EXPECT_TRUE(expected_A0.isApprox(actual.A[0], 1.0e-9)) <<
      "Expected is:\n" << expected_A0.format(clean) << "\n" <<
      "Actual is:\n" << actual.A[0].format(clean) << "\n" <<
      "Difference is:\n" << (expected_A0 - actual.A[0]).format(clean) << "\n";

  EXPECT_TRUE(expected_A1.isApprox(actual.A[1], 1.0e-9)) <<
      "Expected is:\n" << expected_A1.format(clean) << "\n" <<
      "Actual is:\n" << actual.A[1].format(clean) << "\n" <<
      "Difference is:\n" << (expected_A1 - actual.A[1]).format(clean) << "\n";

  EXPECT_TRUE(expected_b.isApprox(actual.b, 1.0e-9)) <<
      "Expected is:\n" << expected_b.format(clean) << "\n" <<
      "Actual is:\n" << actual.b.format(clean) << "\n" <<
      "Difference is:\n" << (expected_b - actual.b).format(clean) << "\n";
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
