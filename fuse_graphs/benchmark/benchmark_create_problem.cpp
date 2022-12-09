/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Clearpath Robotics
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
#include <benchmark/benchmark.h>
#include <ceres/dynamic_autodiff_cost_function.h>

#include <algorithm>
#include <iterator>
#include <string>
#include <vector>

#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include "example_variable.hpp"
#include <fuse_core/constraint.hpp>
#include <fuse_core/serialization.hpp>
#include <fuse_core/uuid.hpp>
#include <fuse_graphs/hash_graph.hpp>

/**
 * @brief Testable fuse_graphs::HashGraph that exposes the protected createProblem method as public
 */
class TestableHashGraph : public fuse_graphs::HashGraph
{
public:
  using fuse_graphs::HashGraph::createProblem;
};

/**
 * @brief Example functor that support a dynamic number of variables and residuals
 */
class ExampleFunctor
{
public:
  explicit ExampleFunctor(const std::vector<double> & b)
  : b_(b)
  {
  }

  template<typename T>
  bool operator()(T const * const * variables, T * residuals) const
  {
    for (size_t i = 0; i < b_.size(); ++i) {
      residuals[i] = variables[i][0] - T(b_[i]);
    }
    return true;
  }

private:
  std::vector<double> b_;
};

/**
 * @brief Example constraint that support a dynamic number of variables
 */
class ExampleConstraint : public fuse_core::Constraint
{
public:
  FUSE_CONSTRAINT_DEFINITIONS(ExampleConstraint)

  ExampleConstraint() = default;

  template<typename VariableUuidIterator>
  explicit ExampleConstraint(
    const std::string & source, VariableUuidIterator first,
    VariableUuidIterator last)
  : fuse_core::Constraint(source, first, last),
    data(std::distance(first, last), 0.0)
  {
  }

  void print(std::ostream & /*stream = std::cout*/) const override {}
  ceres::CostFunction * costFunction() const override
  {
    auto cost_function =
      new ceres::DynamicAutoDiffCostFunction<ExampleFunctor>(new ExampleFunctor(data));

    for (size_t i = 0; i < data.size(); ++i) {
      cost_function->AddParameterBlock(1);
    }

    cost_function->SetNumResiduals(data.size());

    return cost_function;
  }

  std::vector<double> data;  // Public member variable just for testing

private:
  // Allow Boost Serialization access to private methods
  friend class boost::serialization::access;

  /**
   * @brief The Boost Serialize method that serializes all of the data members in to/out of the
   *        archive
   *
   * @param[in/out] archive - The archive object that holds the serialized class members
   * @param[in] version - The version of the archive being read/written. Generally unused.
   */
  template<class Archive>
  void serialize(Archive & archive, const unsigned int /* version */)
  {
    archive & boost::serialization::base_object<fuse_core::Constraint>(*this);
    archive & data;
  }
};

BOOST_CLASS_EXPORT(ExampleConstraint);

/**
 * @brief Helper function to make TestableHashGraph objects with a given number of constraints
 *
 * @param[in] num_constraints Number of constraints the graph should have
 * @param[in] num_variables_per_constraint Number of variables the constraints should have
 * @return The TestableHashGraph
 */
TestableHashGraph makeTestableHashGraph(
  const size_t num_constraints,
  const size_t num_variables_per_constraint)
{
  TestableHashGraph graph;

  for (size_t i = 0; i < num_constraints; ++i) {
    // Generate variables
    std::vector<fuse_core::Variable::SharedPtr> variables;
    variables.reserve(num_variables_per_constraint);
    std::generate_n(
      std::back_inserter(variables), num_variables_per_constraint,
      []() {return ExampleVariable::make_shared();});                  // NOLINT

    // Add variables to the graph
    for (const auto & variable : variables) {
      graph.addVariable(variable);
    }

    // Add constraint with the generated variables
    std::vector<fuse_core::UUID> variable_uuids;
    variable_uuids.reserve(variables.size());
    std::transform(
      variables.begin(), variables.end(), std::back_inserter(variable_uuids),
      [](const auto & variable) {return variable->uuid();});                // NOLINT

    graph.addConstraint(
      ExampleConstraint::make_shared(
        "test", variable_uuids.begin(),
        variable_uuids.end()));
  }

  return graph;
}

static void BM_createProblem(benchmark::State & state)
{
  const auto graph = makeTestableHashGraph(state.range(0), state.range(1));

  ceres::Problem problem;

  for (auto _ : state) {
    graph.createProblem(problem);
  }
}

BENCHMARK(BM_createProblem)->RangeMultiplier(2)->Ranges({{200, 4000}, {2, 12}});  // NOLINT

BENCHMARK_MAIN();
