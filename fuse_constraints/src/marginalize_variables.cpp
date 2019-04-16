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

}  // namespace detail

void marginalizeVariables(
  const std::vector<fuse_core::UUID>& to_be_marginalized,
  const fuse_core::Graph& graph,
  fuse_core::Transaction& transaction)
{
  // Compute an ordering
  auto elimination_order = detail::computeEliminationOrder(to_be_marginalized, graph);

  // Marginalize out each variable in sequence...
}

}  // namespace fuse_constraints
