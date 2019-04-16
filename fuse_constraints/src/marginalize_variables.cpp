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
#include <fuse_constraints/marginalize_variables.h>
#include <fuse_constraints/uuid_ordering.h>
#include <fuse_constraints/variable_constraints.h>
#include <fuse_core/uuid.h>

#include <boost/iterator/transform_iterator.hpp>
#include <Eigen/Core>
#include <suitesparse/ccolamd.h>

#include <algorithm>
#include <iterator>
#include <unordered_map>
#include <unordered_set>
#include <vector>


namespace fuse_constraints
{

namespace detail
{

UuidOrdering computeEliminationOrder(
  const std::vector<fuse_core::UUID>& to_be_marginalized,
  const fuse_core::Graph& graph)
{
  // COLAMD wants a somewhat weird structure
  // Variables are numbered sequentially in some arbitrary order. We call this the "variable index" order.
  // Similarly, the Constraints are numbered sequentially. We call this the "constraint index" order.
  // 'A' contains the constraint index for each connected constraint to a specific variable. The connected
  //     constraints are added to 'A' in variable index order.
  // 'p' contains the boundary indices for each variable in 'A'. So variable #1 spans entries
  //     from A[p[0]] to A[p[1]-1], and variable #2 is the range A[p[1]] to A[p[2] - 1], etc.
  // In order to compute A and p efficiently, we first construct a VariableConstraints object
  // We will construct sequential indices for the variables and constraints while we populate the VariableConstraints
  // object.
  auto variable_order = UuidOrdering();
  auto constraint_order = UuidOrdering();
  auto variable_constraints = VariableConstraints();
  for (const auto& variable_uuid : to_be_marginalized)
  {
    // Get all connected constraints to this variable
    auto constraints = graph.getConnectedConstraints(variable_uuid);

    // Add each constraint to the VariableConstraints object
    // New constraint and variable indices are automatically generated
    for (const auto& constraint : constraints)
    {
      unsigned int constraint_index = constraint_order[constraint.uuid()];
      for (const auto& variable_uuid : constraint.variables())
      {
        variable_constraints.insert(constraint_index, variable_order[variable_uuid]);
      }
    }
  }

  // Construct the CCOLAMD input structures
  auto recommended_size = ccolamd_recommended(
    variable_constraints.size(),
    constraint_order.size(),
    variable_order.size());
  auto A = std::vector<int>(recommended_size);
  auto p = std::vector<int>(variable_order.size() + 1);

  // Use the VariableConstraints table to construct the A and p structures
  auto A_iter = A.begin();
  auto p_iter = p.begin();
  *p_iter = 0;
  ++p_iter;
  for (unsigned int variable_index = 0; variable_index < variable_order.size(); ++variable_index)
  {
    variable_constraints.getConstraints(variable_index, A_iter);
    *p_iter = (A_iter - A.begin());
    ++p_iter;
  }

  // Define the variables groups used by CCOLAMD. All of the marginalized variables should be group0, all the
  // rest should be group1.
  std::vector<int> variable_groups(variable_order.size(), 1);  // Default all variables to group1
  for (const auto& variable_uuid : to_be_marginalized)
  {
    // Reassign the to_be_marginalized variables to group0
    variable_groups[variable_order[variable_uuid]] = 0;
  }

  // Create some additional CCOLAMD required structures
  double knobs[CCOLAMD_KNOBS];
  ccolamd_set_defaults(knobs);
  int stats[CCOLAMD_STATS];

  // Finally call CCOLAMD
  auto success = ccolamd(
    constraint_order.size(),
    variable_order.size(),
    recommended_size,
    A.data(),
    p.data(),
    knobs,
    stats,
    variable_groups.data());
  if (!success)
  {
    throw std::runtime_error("Failed to call CCOLAMD to generate the elimination order.");
  }

  // Extract the elimination order from CCOLAMD.
  // CCOLAMD returns the elimination order by updating the values stored in p with the variable index
  // Remember that p is larger than variable_order.size()
  auto elimination_order = UuidOrdering();
  for (size_t i = 0; i < variable_order.size(); ++i)
  {
    elimination_order.insert(variable_order[p[i]]);
  }

  return elimination_order;
}

LinearTerm linearize(
  const fuse_core::Constraint& constraint,
  const fuse_core::Graph& graph,
  const UuidOrdering& elimination_order)
{
  LinearTerm result;

  // Generate cost and loss functions from the input constraint
  auto cost_function = constraint.costFunction();
  size_t row_count = cost_function->num_residuals();

  // Loop over the constraint's variables and do several things:
  // * Generate a vector of variable value pointers. This is needed for the Ceres API.
  // * Allocate a matrix for each jacobian block
  // * Generate a vector of jacobian pointers. This is needed for the Ceres API.
  const auto& variable_uuids = constraint.variables();
  const size_t variable_count = variable_uuids.size();
  std::vector<const double*> variable_values;
  variable_values.reserve(variable_count);
  std::vector<double*> jacobians;
  jacobians.reserve(variable_count);
  result.variables.reserve(variable_count);
  result.A.reserve(variable_count);
  for (const auto& variable_uuid : variable_uuids)
  {
    const auto& variable = graph.getVariable(variable_uuid);
    variable_values.push_back(variable.data());
    result.variables.push_back(elimination_order.at(variable_uuid));
    result.A.push_back(fuse_core::MatrixXd(row_count, variable.size()));
    jacobians.push_back(result.A.back().data());
  }
  result.b = fuse_core::VectorXd(row_count);

  // Evaluate the cost function, populating the A matrices and b vector
  bool success = cost_function->Evaluate(variable_values.data(), result.b.data(), jacobians.data());
  delete cost_function;
  success = success && result.b.array().isFinite().all();
  for (const auto& A : result.A)
  {
    success = success && A.array().isFinite().all();
  }
  if (!success)
  {
    throw std::runtime_error("Error in evaluating the cost function. There are two possible reasons. "
                             "Either the CostFunction did not evaluate and fill all residual and jacobians "
                             "that were requested or there was a non-finite value (nan/infinite) generated "
                             "during the jacobian computation.");
  }

  // Update the jacobians with the local parameterizations.
  for (size_t index = 0; index < variable_count; ++index)
  {
    const auto& variable_uuid = variable_uuids[index];
    const auto& variable = graph.getVariable(variable_uuid);
    auto local_parameterization = variable.localParameterization();
    if (local_parameterization)
    {
      auto& jacobian = result.A[index];
      fuse_core::MatrixXd J(local_parameterization->GlobalSize(), local_parameterization->LocalSize());
      local_parameterization->ComputeJacobian(variable_values[index], J.data());
      delete local_parameterization;
      jacobian *= J;
    }
  }

  // Correct A and b for the effects of the loss function
  auto loss_function = constraint.lossFunction();
  if (loss_function)
  {
    double squared_norm = result.b.squaredNorm();
    double rho[3];
    loss_function->Evaluate(squared_norm, rho);
    delete loss_function;
    double sqrt_rho1 = std::sqrt(rho[1]);
    double alpha = 0.0;
    if ((squared_norm > 0.0) && (rho[2] > 0.0))
    {
      const double D = 1.0 + 2.0 * squared_norm * rho[2] / rho[1];
      alpha = 1.0 - std::sqrt(D);
    }

    // Correct the Jacobians
    for (auto& jacobian : result.A)
    {
      if (alpha == 0.0)
      {
        jacobian *= sqrt_rho1;
      }
      else
      {
        // TODO(swilliams) This may be inefficient, at least according to notes in the Ceres codebase.
        jacobian = sqrt_rho1 * (jacobian - (alpha / squared_norm) * result.b * (result.b.transpose() * jacobian));
      }
    }

    // Correct the residuals
    result.b *= sqrt_rho1 / (1 - alpha);
  }

  return result;
}

LinearTerm computeMarginal(
  const unsigned int variable,
  const std::vector<LinearTerm>& linear_terms)
{
  return LinearTerm();
}

MarginalConstraint::SharedPtr createMarginalConstraint(
  const LinearTerm& linear_term,
  const fuse_core::Graph& graph,
  const UuidOrdering& elimination_order)
{
  auto index_to_variable = [&graph, &elimination_order](const unsigned int index) -> const fuse_core::Variable&
  {
    return graph.getVariable(elimination_order.at(index));
  };

  return MarginalConstraint::make_shared(
    boost::make_transform_iterator(linear_term.variables.begin(), index_to_variable),
    boost::make_transform_iterator(linear_term.variables.end(), index_to_variable),
    linear_term.A.begin(),
    linear_term.A.end(),
    linear_term.b);
}

}  // namespace detail

fuse_core::Transaction marginalizeVariables(
  const std::vector<fuse_core::UUID>& to_be_marginalized,
  const fuse_core::Graph& graph)
{
  fuse_core::Transaction transaction;

  // Mark all of the to_be_marginalized variables for removal
  for (const auto& variable_uuid : to_be_marginalized)
  {
    transaction.removeVariable(variable_uuid);
  }

  // Compute an ordering
  auto elimination_order = detail::computeEliminationOrder(to_be_marginalized, graph);

  // Linearize all involved constraints, and store them with the variable where they will be used
  auto used_constraints = std::unordered_set<fuse_core::UUID, fuse_core::uuid::hash>();
  std::vector<std::vector<detail::LinearTerm>> linear_terms(elimination_order.size());
  for (size_t i = 0; i < to_be_marginalized.size(); ++i)
  {
    auto constraints = graph.getConnectedConstraints(elimination_order[i]);
    for (const auto& constraint : constraints)
    {
      if (used_constraints.find(constraint.uuid()) == used_constraints.end())
      {
        used_constraints.insert(constraint.uuid());
        linear_terms[i].push_back(detail::linearize(constraint, graph, elimination_order));
        transaction.removeConstraint(constraint.uuid());
      }
    }
  }

  // Use the linearized constraints to marginalize each variable in order
  // Place the resulting marginal in the linear constraint bucket associated with the earliest remaining variable
  for (size_t i = 0; i < to_be_marginalized.size(); ++i)
  {
    auto linear_marginal = detail::computeMarginal(i, linear_terms[i]);
    auto lowest_ordered_variable = linear_marginal.variables.front();
    linear_terms[lowest_ordered_variable].push_back(std::move(linear_marginal));
  }

  // Convert all remaining linear marginals into marginal constraints
  for (size_t i = to_be_marginalized.size(); i < linear_terms.size(); ++i)
  {
    for (const auto& linear_term : linear_terms[i])
    {
      auto marginal_constraint = detail::createMarginalConstraint(linear_term, graph, elimination_order);
      transaction.addConstraint(std::move(marginal_constraint));
    }
  }

  return transaction;
}

}  // namespace fuse_constraints
