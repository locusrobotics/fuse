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
#include <functional>

#include <boost/iterator/transform_iterator.hpp>
#include <fuse_core/graph.hpp>
#include <fuse_core/transaction.hpp>
#include <fuse_core/uuid.hpp>

namespace fuse_core
{

std::ostream & operator<<(std::ostream & stream, const Graph & graph)
{
  graph.print(stream);
  return stream;
}

Graph::const_variable_range Graph::getConnectedVariables(const UUID & constraint_uuid) const
{
  std::function<const fuse_core::Variable & (const UUID & variable_uuid)> uuid_to_variable_ref =
    [this](const UUID & variable_uuid) -> const Variable &
    {
      return this->getVariable(variable_uuid);
    };

  const auto & constraint = getConstraint(constraint_uuid);
  const auto & variable_uuids = constraint.variables();

  return const_variable_range(
    boost::make_transform_iterator(variable_uuids.cbegin(), uuid_to_variable_ref),
    boost::make_transform_iterator(variable_uuids.cend(), uuid_to_variable_ref));
}

void Graph::update(const Transaction & transaction)
{
  // Update the graph with a new transaction. In order to keep the graph consistent, variables are
  // added first, followed by the constraints which might use the newly added variables. Then
  // constraints are removed so that the variable usage is updated. Finally, variables are removed.

  // Insert the new variables into the graph
  for (const auto & variable : transaction.addedVariables()) {
    addVariable(variable.clone());
  }
  // Insert the new constraints into the graph
  for (const auto & constraint : transaction.addedConstraints()) {
    addConstraint(constraint.clone());
  }
  // Delete constraints from the graph
  for (const auto & constraint_uuid : transaction.removedConstraints()) {
    removeConstraint(constraint_uuid);
  }
  // Delete variables from the graph
  for (const auto & variable_uuid : transaction.removedVariables()) {
    removeVariable(variable_uuid);
  }
}

}  // namespace fuse_core
