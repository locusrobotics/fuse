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
#ifndef FUSE_CONSTRAINTS_MARGINALIZE_VARIABLES_H
#define FUSE_CONSTRAINTS_MARGINALIZE_VARIABLES_H

#include <fuse_constraints/marginal_constraint.h>
#include <fuse_constraints/uuid_ordering.h>
#include <fuse_core/constraint.h>
#include <fuse_core/eigen.h>
#include <fuse_core/graph.h>
#include <fuse_core/local_parameterization.h>
#include <fuse_core/macros.h>
#include <fuse_core/transaction.h>
#include <fuse_core/variable.h>

#include <boost/iterator/transform_iterator.hpp>
#include <ceres/cost_function.h>

#include <algorithm>
#include <cassert>
#include <iterator>
#include <ostream>
#include <vector>


namespace fuse_constraints
{

void marginalizeVariables(
  const std::vector<fuse_core::UUID>& to_be_marginalized,
  const fuse_core::Graph& graph,
  fuse_core::Transaction& transaction);

template <typename VariableUuidIterator>
void marginalizeVariables(
  VariableUuidIterator first,
  VariableUuidIterator last,
  const fuse_core::Graph& graph,
  fuse_core::Transaction& transaction)
{
  std::vector<fuse_core::UUID> to_be_marginalized;
  std::copy(first, last, std::back_inserter(to_be_marginalized));
  marginalizeVariables(to_be_marginalized, graph, transaction);
}

template <typename VariableUuidIterator>
fuse_core::Transaction marginalizeVariables(
  VariableUuidIterator first,
  VariableUuidIterator last,
  const fuse_core::Graph& graph)
{
  fuse_core::Transaction transaction;
  marginalizeVariables(first, last, graph, transaction);
  return transaction;
}

namespace detail
{

/**
 * @brief Compute an efficient elimination order that places the marginalized variables before the additional variables
 *
 * Each time a variable is eliminated from the system, the resulting reduced system is independent of the
 * eliminated variable. By eliminating the "to be marginalized variables" first, all of the
 * "additional connected variables" will remain in the system, but they will not depend on any of the
 * "to be marginalized variables"...which is what we want.
 *
 * This function uses CCOLAMD to find a good elimination order that eliminates all the "to be marginalized variables"
 * first.
 *
 * @param[in] to_be_marginalized The variable UUIDs to be marginalized out
 * @param[in] constraints        All constraints that involve at least one marginalized variable
 * @return The variable UUIDs in the computed elimination order
 */
UuidOrdering computeEliminationOrder(
  const std::vector<fuse_core::UUID>& to_be_marginalized,
  const fuse_core::Graph& graph);

}  // namespace detail

}  // namespace fuse_constraints

#endif  // FUSE_CONSTRAINTS_MARGINALIZE_VARIABLES_H
