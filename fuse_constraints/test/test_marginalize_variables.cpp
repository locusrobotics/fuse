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
#include <fuse_constraints/absolute_orientation_3d_stamped_constraint.h>
#include <fuse_constraints/marginalize_variables.h>
#include <fuse_constraints/relative_orientation_3d_stamped_constraint.h>
#include <fuse_constraints/uuid_ordering.h>
#include <fuse_core/constraint.h>
#include <fuse_core/eigen.h>
#include <fuse_core/eigen_gtest.h>
#include <fuse_core/macros.h>
#include <fuse_core/serialization.h>
#include <fuse_core/uuid.h>
#include <fuse_core/variable.h>
#include <fuse_graphs/hash_graph.h>
#include <fuse_variables/orientation_3d_stamped.h>

#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <ceres/cost_function.h>
#include <gtest/gtest.h>

#include <set>
#include <utility>
#include <vector>


/**
 * @brief Create a simple Variable implementation for testing
 */
class GenericVariable : public fuse_core::Variable
{
public:
  FUSE_VARIABLE_DEFINITIONS(GenericVariable)

  GenericVariable() :
    fuse_core::Variable(fuse_core::uuid::generate()),
    data_{}
  {}

  explicit GenericVariable(const fuse_core::UUID& uuid) :
    fuse_core::Variable(uuid),
    data_{}
  {}

  size_t size() const override { return 1; }

  const double* data() const override { return &data_; }
  double* data() override { return &data_; }

  void print(std::ostream& /*stream = std::cout*/) const override {}

protected:
  double data_;

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
    archive & boost::serialization::base_object<fuse_core::Variable>(*this);
    archive & data_;
  }
};

BOOST_CLASS_EXPORT(GenericVariable);

/**
 * @brief Create a simple Constraint implementation for testing
 */
class GenericConstraint : public fuse_core::Constraint
{
public:
  FUSE_CONSTRAINT_DEFINITIONS(GenericConstraint)

  GenericConstraint(std::initializer_list<fuse_core::UUID> variable_uuids) :
    Constraint("test", variable_uuids) {}

  explicit GenericConstraint(const fuse_core::UUID& variable1) :
    fuse_core::Constraint("test", {variable1}) {}

  GenericConstraint(const fuse_core::UUID& variable1, const fuse_core::UUID& variable2) :
    fuse_core::Constraint("test", {variable1, variable2}) {}

  void print(std::ostream& /*stream = std::cout*/) const override {}

  ceres::CostFunction* costFunction() const override { return nullptr; }

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

class FixedOrientation3DStamped : public fuse_variables::Orientation3DStamped
{
public:
  FUSE_VARIABLE_DEFINITIONS(FixedOrientation3DStamped)

  FixedOrientation3DStamped() = default;

  explicit FixedOrientation3DStamped(const rclcpp::Time& stamp, const fuse_core::UUID& device_id = fuse_core::uuid::NIL) :
    Orientation3DStamped(stamp, device_id)
  {
  }

  bool holdConstant() const override
  {
    return true;
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
    archive & boost::serialization::base_object<fuse_variables::Orientation3DStamped>(*this);
  }
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
  auto actual = fuse_constraints::computeEliminationOrder(to_be_marginalized, graph);

  // Define the expected order
  auto expected = fuse_constraints::UuidOrdering();
  expected.push_back(x1->uuid());
  expected.push_back(x2->uuid());
  expected.push_back(l1->uuid());
  expected.push_back(x3->uuid());

  // Check
  ASSERT_EQ(expected.size(), actual.size());
  for (size_t i = 0; i < expected.size(); ++i)
  {
    SCOPED_TRACE(i);
    EXPECT_EQ(fuse_core::uuid::to_string(expected.at(i)), fuse_core::uuid::to_string(actual.at(i)));
  }
}

TEST(MarginalizeVariables, ComputeEliminationOrderWithOrphanVariables)
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

  // Add orphan variables (with explicit UUID so it's easier to identify them if any of the checks fail)
  // With 1 or 2 orphan variables computeEliminationOrder throws an std::runtime_error exception because CCOLAMD fails
  // With 3 or more orphan variables computeEliminationOrder crashes with `free(): invalid next size (fast)`
  auto o1 = GenericVariable::make_shared(fuse_core::uuid::from_string("b726fbef-4015-4dc8-b4dd-cb57d4439c74"));
  graph.addVariable(o1);

  // Define the set of variables to be marginalized
  auto to_be_marginalized = std::vector<fuse_core::UUID>{ x2->uuid(), x1->uuid(), o1->uuid() };

  // Compute the ordering
  fuse_constraints::UuidOrdering actual;
  ASSERT_NO_THROW(actual = fuse_constraints::computeEliminationOrder(to_be_marginalized, graph));

  // Define the expected order
  auto expected = fuse_constraints::UuidOrdering();
  expected.push_back(x1->uuid());
  expected.push_back(x2->uuid());
  expected.push_back(o1->uuid());
  expected.push_back(l1->uuid());
  expected.push_back(x3->uuid());

  // Check
  ASSERT_EQ(expected.size(), actual.size());
  for (size_t i = 0; i < expected.size(); ++i)
  {
    SCOPED_TRACE(i);
    EXPECT_EQ(fuse_core::uuid::to_string(expected.at(i)), fuse_core::uuid::to_string(actual.at(i)));
  }

  // Check all marginalized variables are in the elimination order
  // This check is equivalent to the assert in marginalizeVariables
  for (const auto& variable_uuid : to_be_marginalized)
  {
    SCOPED_TRACE(fuse_core::uuid::to_string(variable_uuid));

    EXPECT_TRUE(actual.exists(variable_uuid));
    if (actual.exists(variable_uuid))
    {
      EXPECT_GT(to_be_marginalized.size(), actual.at(variable_uuid));
    }
  }
}

TEST(MarginalizeVariables, Linearize)
{
  // Create a graph with one relative 3D orientation constraint
  auto x1 = fuse_variables::Orientation3DStamped::make_shared(rclcpp::Time(1, 0));
  x1->w() = 0.927362;
  x1->x() = 0.1;
  x1->y() = 0.2;
  x1->z() = 0.3;

  auto x2 = fuse_variables::Orientation3DStamped::make_shared(rclcpp::Time(2, 0));
  x2->w() = 0.848625;
  x2->x() = 0.13798;
  x2->y() = 0.175959;
  x2->z() = 0.479411;

  fuse_core::Vector4d delta;
  delta << 0.979795897, 0.0, 0.0, 0.2;
  fuse_core::Matrix3d cov;
  cov << 1.0, 0.0, 0.0,   0.0, 2.0, 0.0,   0.0, 0.0, 3.0;
  auto constraint = fuse_constraints::RelativeOrientation3DStampedConstraint::make_shared("test", *x1, *x2, delta, cov);

  auto graph = fuse_graphs::HashGraph();
  graph.addVariable(x1);
  graph.addVariable(x2);
  graph.addConstraint(constraint);

  // Create an elimination order
  auto elimination_order = fuse_constraints::UuidOrdering();
  elimination_order.push_back(fuse_core::uuid::generate());  // Add a dummy variable to the elimination order
  elimination_order.push_back(x2->uuid());
  elimination_order.push_back(fuse_core::uuid::generate());  // Add a dummy variable to the elimination order
  elimination_order.push_back(x1->uuid());

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

  EXPECT_MATRIX_NEAR(expected_A0, actual.A[0], 1.0e-9);
  EXPECT_MATRIX_NEAR(expected_A1, actual.A[1], 1.0e-9);
  EXPECT_MATRIX_NEAR(expected_b, actual.b, 1.0e-9);
}

TEST(MarginalizeVariables, MarginalizeNext)
{
  // Construct a couple of linear terms
  auto term1 = fuse_constraints::detail::LinearTerm();
  term1.variables.push_back(1);
  auto A1 = fuse_core::MatrixXd(3, 3);
  A1 <<  0.99999999999999922284,     4.4999993911720714834e-08, -2.9999995598828377297e-08,
        -3.181980062078038074e-08,   0.70710678118654701763,     1.0606600528428877794e-08,
         1.7320505793505525105e-08, -8.6602525498080673572e-09,  0.57735026918962550901;
  term1.A.push_back(A1);
  auto b1 = fuse_core::VectorXd(3);
  b1 << -2.9999995786018886696e-08, -4.2426400911723613558e-08, -5.1961516896187549911e-08;
  term1.b = b1;

  auto term2 = fuse_constraints::detail::LinearTerm();
  term2.variables.push_back(3);
  term2.variables.push_back(1);
  auto A21 = fuse_core::MatrixXd(3, 3);
  A21 << 0.99999999999996114219,    -1.8482708254510815671e-07, -2.873543621662033587e-07,
         1.3069243487082160549e-07,  0.70710678118650927004,    -2.5352115484711390536e-07,
         1.6590414383954588118e-07,  2.0699913566568639567e-07,  0.57735026918958520792;
  auto A22 = fuse_core::MatrixXd(3, 3);
  A22 << -0.91999992754510684367,    -0.39191852892782985673,    -2.8735440640859089001e-07,
          0.27712824947756498073,    -0.6505381874582485402,     -2.5352112979770691226e-07,
          7.1505019336171038447e-08,  2.5546009342625186633e-07, -0.57735026918958520792;
  term2.A.push_back(A21);
  term2.A.push_back(A22);
  auto b2 = fuse_core::VectorXd(3);
  b2 << 7.1706607563166841896e-07, -4.0638046747479327072e-07, 2.1341989211309879704e-07;
  term2.b = b2;

  auto terms = std::vector<fuse_constraints::detail::LinearTerm>();
  terms.push_back(term1);
  terms.push_back(term2);

  // Marginalize out the lowest ordered variable
  auto actual = fuse_constraints::detail::marginalizeNext(terms);

  // Define the expected marginal
  auto expected = fuse_constraints::detail::LinearTerm();
  expected.variables.push_back(3);
  auto A_expected = fuse_core::MatrixXd(3, 3);
  A_expected << -0.686835139329528,   0.064384601986636,   0.000000153209328,
                 0.000000000000000,  -0.509885650799691,   0.000000079984512,
                 0.000000000000000,   0.000000000000000,   0.408248290463911;
  expected.A.push_back(A_expected);
  auto b_expected = fuse_core::VectorXd(3);
  b_expected << -0.000000497197868, 0.000000315186479, 0.000000114168337;
  expected.b = b_expected;

  // Test
  ASSERT_EQ(expected.variables.size(), actual.variables.size());
  EXPECT_EQ(expected.variables[0], actual.variables[0]);

  ASSERT_EQ(expected.A.size(), actual.A.size());
  EXPECT_MATRIX_NEAR(expected.A[0], actual.A[0], 1.0e-9);
  EXPECT_MATRIX_NEAR(expected.b, actual.b, 1.0e-9);
}

TEST(MarginalizeVariables, MarginalizeVariables)
{
  // Create variables
  auto x1 = fuse_variables::Orientation3DStamped::make_shared(rclcpp::Time(1.0));
  x1->w() = 0.927362;
  x1->x() = 0.1;
  x1->y() = 0.2;
  x1->z() = 0.3;
  auto x2 = fuse_variables::Orientation3DStamped::make_shared(rclcpp::Time(2.0));
  x2->w() = 0.848625;
  x2->x() = 0.13798;
  x2->y() = 0.175959;
  x2->z() = 0.479411;
  auto x3 = fuse_variables::Orientation3DStamped::make_shared(rclcpp::Time(3.0));
  x3->w() = 0.735597;
  x3->x() = 0.170384;
  x3->y() = 0.144808;
  x3->z() = 0.63945;
  auto l1 = fuse_variables::Orientation3DStamped::make_shared(rclcpp::Time(3.5));
  l1->w() = 0.803884;
  l1->x() = 0.304917;
  l1->y() = 0.268286;
  l1->z() = 0.434533;

  // Create some constraints
  fuse_core::Vector4d mean1;
  mean1 << 0.92736185, 0.1, 0.2, 0.3;
  fuse_core::Matrix3d cov1;
  cov1 << 1.0, 0.0, 0.0,  0.0, 2.0, 0.0,  0.0, 0.0, 3.0;
  auto prior_x1 = fuse_constraints::AbsoluteOrientation3DStampedConstraint::make_shared(
    "test", *x1, mean1, cov1);

  fuse_core::Vector4d delta2;
  delta2 << 0.979795897, 0.0, 0.0, 0.2;
  fuse_core::Matrix3d cov2;
  cov2 << 1.0, 0.0, 0.0,   0.0, 2.0, 0.0,   0.0, 0.0, 3.0;
  auto relative_x1_x2 = fuse_constraints::RelativeOrientation3DStampedConstraint::make_shared(
    "test", *x1, *x2, delta2, cov2);

  fuse_core::Vector4d delta3;
  delta3 << 0.979795897, 0.0, 0.0, 0.2;
  fuse_core::Matrix3d cov3;
  cov3 << 1.0, 0.0, 0.0,   0.0, 2.0, 0.0,   0.0, 0.0, 3.0;
  auto relative_x2_x3 = fuse_constraints::RelativeOrientation3DStampedConstraint::make_shared(
    "test", *x2, *x3, delta3, cov3);

  fuse_core::Vector4d delta4;
  delta4 << 0.979795897, 0.2, 0.0, 0.0;
  fuse_core::Matrix3d cov4;
  cov4 << 1.0, 0.0, 0.0,   0.0, 2.0, 0.0,   0.0, 0.0, 3.0;
  auto relative_x2_l1 = fuse_constraints::RelativeOrientation3DStampedConstraint::make_shared(
    "test", *x2, *l1, delta4, cov4);

  // Add to the graph
  auto graph = fuse_graphs::HashGraph();
  graph.addVariable(x1);
  graph.addVariable(x2);
  graph.addVariable(x3);
  graph.addVariable(l1);
  graph.addConstraint(prior_x1);
  graph.addConstraint(relative_x1_x2);
  graph.addConstraint(relative_x2_x3);
  graph.addConstraint(relative_x2_l1);

  // Run the solver
  graph.optimize();

  // Extract the optimal values and covariances
  auto expected_x2 = x2->array();
  auto expected_x3 = x3->array();
  auto expected_l1 = l1->array();

  auto requests = std::vector<std::pair<fuse_core::UUID, fuse_core::UUID>>
  {
    {x2->uuid(), x2->uuid()}, {x3->uuid(), x3->uuid()}, {l1->uuid(), l1->uuid()}
  };
  auto expected_covariances = std::vector<std::vector<double>>();
  graph.getCovariance(requests, expected_covariances);
  const auto& expected_x2_cov = expected_covariances[0];
  const auto& expected_x3_cov = expected_covariances[1];
  const auto& expected_l1_cov = expected_covariances[2];

  // Marginalize out X1
  auto transaction = fuse_constraints::marginalizeVariables("test", {x1->uuid()}, graph);  // NOLINT

  // Verify the computed transaction
  auto added_variables = transaction.addedVariables();
  EXPECT_EQ(0u, std::distance(added_variables.begin(), added_variables.end()));

  auto removed_variables_range = transaction.removedVariables();
  auto removed_variables = std::set<fuse_core::UUID>(removed_variables_range.begin(), removed_variables_range.end());
  EXPECT_EQ(1u, removed_variables.size());
  EXPECT_TRUE(removed_variables.count(x1->uuid()));

  auto added_constraints = transaction.addedConstraints();
  EXPECT_EQ(1u, std::distance(added_constraints.begin(), added_constraints.end()));

  auto removed_constraints_range = transaction.removedConstraints();
  auto removed_constraints = std::set<fuse_core::UUID>(removed_constraints_range.begin(),
                                                       removed_constraints_range.end());
  EXPECT_EQ(2u, removed_constraints.size());
  EXPECT_TRUE(removed_constraints.count(prior_x1->uuid()));
  EXPECT_TRUE(removed_constraints.count(relative_x1_x2->uuid()));

  // Apply the transaction to the graph
  graph.update(transaction);

  // Re-optimize the graph
  graph.optimize();

  // Get the post-marginal variable values and covariances
  auto actual_x2 = x2->array();
  auto actual_x3 = x3->array();
  auto actual_l1 = l1->array();

  auto actual_covariances = std::vector<std::vector<double>>();
  graph.getCovariance(requests, actual_covariances);
  const auto& actual_x2_cov = actual_covariances[0];
  const auto& actual_x3_cov = actual_covariances[1];
  const auto& actual_l1_cov = actual_covariances[2];

  // Compare. The post-marginal results should be identical to the pre-marginal results
  ASSERT_EQ(expected_x2.size(), actual_x2.size());
  for (size_t i = 0; i < expected_x2.size(); ++i)
  {
    EXPECT_NEAR(expected_x2[i], actual_x2[i], 1.0e-5);
  }
  ASSERT_EQ(expected_x2_cov.size(), actual_x2_cov.size());
  for (size_t i = 0; i < expected_x2_cov.size(); ++i)
  {
    EXPECT_NEAR(expected_x2_cov[i], actual_x2_cov[i], 1.0e-5);
  }

  ASSERT_EQ(expected_x3.size(), actual_x3.size());
  for (size_t i = 0; i < expected_x3.size(); ++i)
  {
    EXPECT_NEAR(expected_x3[i], actual_x3[i], 1.0e-5);
  }
  ASSERT_EQ(expected_x3_cov.size(), actual_x3_cov.size());
  for (size_t i = 0; i < expected_x3_cov.size(); ++i)
  {
    EXPECT_NEAR(expected_x3_cov[i], actual_x3_cov[i], 1.0e-5);
  }

  ASSERT_EQ(expected_l1.size(), actual_l1.size());
  for (size_t i = 0; i < expected_l1.size(); ++i)
  {
    EXPECT_NEAR(expected_l1[i], actual_l1[i], 1.0e-5);
  }
  ASSERT_EQ(expected_l1_cov.size(), actual_l1_cov.size());
  for (size_t i = 0; i < expected_l1_cov.size(); ++i)
  {
    EXPECT_NEAR(expected_l1_cov[i], actual_l1_cov[i], 1.0e-5);
  }
}

TEST(MarginalizeVariables, MarginalizeFixedVariables)
{
  // Create variables
  auto x1 = fuse_variables::Orientation3DStamped::make_shared(rclcpp::Time(1.0));
  x1->w() = 0.927362;
  x1->x() = 0.1;
  x1->y() = 0.2;
  x1->z() = 0.3;
  auto x2 = fuse_variables::Orientation3DStamped::make_shared(rclcpp::Time(2.0));
  x2->w() = 0.848625;
  x2->x() = 0.13798;
  x2->y() = 0.175959;
  x2->z() = 0.479411;
  auto x3 = fuse_variables::Orientation3DStamped::make_shared(rclcpp::Time(3.0));
  x3->w() = 0.735597;
  x3->x() = 0.170384;
  x3->y() = 0.144808;
  x3->z() = 0.63945;
  auto l1 = FixedOrientation3DStamped::make_shared(rclcpp::Time(3.5));
  l1->w() = 0.803884;
  l1->x() = 0.304917;
  l1->y() = 0.268286;
  l1->z() = 0.434533;

  // Create some constraints
  fuse_core::Vector4d mean1;
  mean1 << 0.92736185, 0.1, 0.2, 0.3;
  fuse_core::Matrix3d cov1;
  cov1 << 1.0, 0.0, 0.0,  0.0, 2.0, 0.0,  0.0, 0.0, 3.0;
  auto prior_x1 = fuse_constraints::AbsoluteOrientation3DStampedConstraint::make_shared(
    "test", *x1, mean1, cov1);

  // Note that this prior on the landmark is required. The covariance of the prior has no impact on the solution, as
  // the value of the landmark will be held constant. However, due to assumptions made in the marginalization code, the
  // landmark variable must be fully-constrained. Hopefully this requirement will be removed in a future version.
  fuse_core::Vector4d mean2;
  mean2 << 0.842614977, 0.2, 0.3, 0.4;
  fuse_core::Matrix3d cov2;
  cov2 << 3.0, 0.0, 0.0,  0.0, 3.1, 0.0,  0.0, 0.0, 3.2;
  auto prior_l1 = fuse_constraints::AbsoluteOrientation3DStampedConstraint::make_shared(
    "test", *l1, mean2, cov2);

  fuse_core::Vector4d delta3;
  delta3 << 0.979795897, 0.0, 0.0, 0.2;
  fuse_core::Matrix3d cov3;
  cov3 << 1.0, 0.0, 0.0,   0.0, 2.0, 0.0,   0.0, 0.0, 3.0;
  auto relative_x1_x2 = fuse_constraints::RelativeOrientation3DStampedConstraint::make_shared(
    "test", *x1, *x2, delta3, cov3);

  fuse_core::Vector4d delta4;
  delta4 << 0.979795897, 0.0, 0.0, 0.2;
  fuse_core::Matrix3d cov4;
  cov4 << 1.0, 0.0, 0.0,   0.0, 2.0, 0.0,   0.0, 0.0, 3.0;
  auto relative_x2_x3 = fuse_constraints::RelativeOrientation3DStampedConstraint::make_shared(
    "test", *x2, *x3, delta4, cov4);

  fuse_core::Vector4d delta5;
  delta5 << 0.979795897, 0.2, 0.0, 0.0;
  fuse_core::Matrix3d cov5;
  cov5 << 1.0, 0.0, 0.0,   0.0, 2.0, 0.0,   0.0, 0.0, 3.0;
  auto relative_x1_l1 = fuse_constraints::RelativeOrientation3DStampedConstraint::make_shared(
    "test", *x1, *l1, delta5, cov5);

  // Add to the graph
  auto graph = fuse_graphs::HashGraph();
  graph.addVariable(x1);
  graph.addVariable(x2);
  graph.addVariable(x3);
  graph.addVariable(l1);
  graph.addConstraint(prior_x1);
  graph.addConstraint(prior_l1);
  graph.addConstraint(relative_x1_x2);
  graph.addConstraint(relative_x2_x3);
  graph.addConstraint(relative_x1_l1);

  // Run the solver
  graph.optimize();

  // Extract the optimal values and covariances
  auto expected_x2 = x2->array();
  auto expected_x3 = x3->array();

  auto requests = std::vector<std::pair<fuse_core::UUID, fuse_core::UUID>>
  {
    {x2->uuid(), x2->uuid()}, {x3->uuid(), x3->uuid()}
  };
  auto expected_covariances = std::vector<std::vector<double>>();
  graph.getCovariance(requests, expected_covariances);
  const auto& expected_x2_cov = expected_covariances[0];
  const auto& expected_x3_cov = expected_covariances[1];

  // Marginalize out X1 and L1
  auto transaction = fuse_constraints::marginalizeVariables("test", {x1->uuid(), l1->uuid()}, graph);  // NOLINT

  // Verify the computed transaction
  auto added_variables = transaction.addedVariables();
  EXPECT_EQ(0u, std::distance(added_variables.begin(), added_variables.end()));

  auto removed_variables_range = transaction.removedVariables();
  auto removed_variables = std::set<fuse_core::UUID>(removed_variables_range.begin(), removed_variables_range.end());
  EXPECT_EQ(2u, removed_variables.size());
  EXPECT_TRUE(removed_variables.count(x1->uuid()));
  EXPECT_TRUE(removed_variables.count(l1->uuid()));

  auto added_constraints = transaction.addedConstraints();
  EXPECT_EQ(1u, std::distance(added_constraints.begin(), added_constraints.end()));

  auto removed_constraints_range = transaction.removedConstraints();
  auto removed_constraints = std::set<fuse_core::UUID>(removed_constraints_range.begin(),
                                                       removed_constraints_range.end());

  EXPECT_EQ(4u, removed_constraints.size());
  EXPECT_TRUE(removed_constraints.count(prior_x1->uuid()));
  EXPECT_TRUE(removed_constraints.count(prior_l1->uuid()));
  EXPECT_TRUE(removed_constraints.count(relative_x1_x2->uuid()));
  EXPECT_TRUE(removed_constraints.count(relative_x1_l1->uuid()));

  // Apply the transaction to the graph
  graph.update(transaction);

  // Re-optimize the graph
  graph.optimize();

  // Get the post-marginal variable values and covariances
  auto actual_x2 = x2->array();
  auto actual_x3 = x3->array();

  auto actual_covariances = std::vector<std::vector<double>>();
  graph.getCovariance(requests, actual_covariances);
  const auto& actual_x2_cov = actual_covariances[0];
  const auto& actual_x3_cov = actual_covariances[1];

  // Compare. The post-marginal results should be identical to the pre-marginal results
  ASSERT_EQ(expected_x2.size(), actual_x2.size());
  for (size_t i = 0; i < expected_x2.size(); ++i)
  {
    EXPECT_NEAR(expected_x2[i], actual_x2[i], 1.0e-3);
  }
  ASSERT_EQ(expected_x2_cov.size(), actual_x2_cov.size());
  for (size_t i = 0; i < expected_x2_cov.size(); ++i)
  {
    EXPECT_NEAR(expected_x2_cov[i], actual_x2_cov[i], 1.0e-3);
  }

  ASSERT_EQ(expected_x3.size(), actual_x3.size());
  for (size_t i = 0; i < expected_x3.size(); ++i)
  {
    EXPECT_NEAR(expected_x3[i], actual_x3[i], 1.0e-3);
  }
  ASSERT_EQ(expected_x3_cov.size(), actual_x3_cov.size());
  for (size_t i = 0; i < expected_x3_cov.size(); ++i)
  {
    EXPECT_NEAR(expected_x3_cov[i], actual_x3_cov[i], 1.0e-3);
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
