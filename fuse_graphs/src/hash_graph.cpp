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
#include <fuse_graphs/hash_graph.h>
#include <fuse_core/uuid.h>

#include <boost/iterator/transform_iterator.hpp>

#include <algorithm>
#include <functional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>


namespace fuse_graphs
{

HashGraph::HashGraph(const ceres::Problem::Options& options) :
  problem_options_(options)
{
}

HashGraph::HashGraph(const HashGraph& other) :
  constraints_by_variable_uuid_(other.constraints_by_variable_uuid_),
  problem_options_(other.problem_options_),
  variables_on_hold_(other.variables_on_hold_)
{
  // Make a deep copy of the constraints
  std::transform(other.constraints_.begin(),
                 other.constraints_.end(),
                 std::inserter(constraints_, constraints_.end()),
                 [](const Constraints::value_type& uuid__constraint) -> Constraints::value_type
                 {
                   return {uuid__constraint.first, uuid__constraint.second->clone()};
                 });  // NOLINT(whitespace/braces)
  // Make a deep copy of the variables
  std::transform(other.variables_.begin(),
                 other.variables_.end(),
                 std::inserter(variables_, variables_.end()),
                 [](const Variables::value_type& uuid__variable) -> Variables::value_type
                 {
                   return {uuid__variable.first, uuid__variable.second->clone()};
                 });  // NOLINT(whitespace/braces)
}

HashGraph& HashGraph::operator=(const HashGraph& other)
{
  // Make a copy (might throw an exception)
  HashGraph tmp(other);
  // Then swap (won't throw an exception)
  std::swap(constraints_, tmp.constraints_);
  std::swap(constraints_by_variable_uuid_, tmp.constraints_by_variable_uuid_);
  std::swap(problem_options_, tmp.problem_options_);
  std::swap(variables_, tmp.variables_);
  std::swap(variables_on_hold_, tmp.variables_on_hold_);
  return *this;
}

fuse_core::Graph::UniquePtr HashGraph::clone() const
{
  return HashGraph::make_unique(*this);
}

bool HashGraph::constraintExists(const fuse_core::UUID& constraint_uuid) const noexcept
{
  // map.find() does not itself throw exceptions, but may as a result of the key comparison operator. Because the UUID
  // comparisons are marked as noexcept by Boost, I feel safe marking this function as noexcept as well.
  auto constraints_iter = constraints_.find(constraint_uuid);
  return (constraints_iter != constraints_.end());
}

bool HashGraph::addConstraint(fuse_core::Constraint::SharedPtr constraint)
{
  // Do nothing if the constraint is empty, or the constraint already exists
  if (!constraint || constraintExists(constraint->uuid()))
  {
    return false;
  }
  // Check that all of the referenced variables exist. Throw a logic_error if they do not.
  for (const auto& variable_uuid : constraint->variables())
  {
    if (!variableExists(variable_uuid))
    {
      throw std::logic_error("Attempting to add a constraint (" + fuse_core::uuid::to_string(constraint->uuid()) +
                             ") that uses an unknown variable (" + fuse_core::uuid::to_string(variable_uuid) + ")");
    }
  }
  // Add the constraint to the list of known constraints
  constraints_.emplace(constraint->uuid(), constraint);
  // Also add it to the variable-constraint cross reference
  for (const auto& variable_uuid : constraint->variables())
  {
    constraints_by_variable_uuid_[variable_uuid].push_back(constraint->uuid());
  }
  return true;
}

bool HashGraph::removeConstraint(const fuse_core::UUID& constraint_uuid)
{
  // Check if the constraint exists
  auto constraints_iter = constraints_.find(constraint_uuid);
  if (constraints_iter == constraints_.end())
  {
    return false;
  }
  // Remove the constraint from the cross-reference data structure
  for (const auto& variable_uuid : constraints_iter->second->variables())
  {
    auto& constraints = constraints_by_variable_uuid_.at(variable_uuid);
    constraints.erase(std::remove(constraints.begin(), constraints.end(), constraint_uuid), constraints.end());
  }
  // And remove the constraint
  constraints_.erase(constraints_iter);  // This does not throw
  return true;
}

const fuse_core::Constraint& HashGraph::getConstraint(const fuse_core::UUID& constraint_uuid) const
{
  auto constraints_iter = constraints_.find(constraint_uuid);
  if (constraints_iter == constraints_.end())
  {
    throw std::out_of_range("The constraint UUID " + fuse_core::uuid::to_string(constraint_uuid) + " does not exist.");
  }
  return *constraints_iter->second;
}

fuse_core::const_constraint_range HashGraph::getConstraints() const noexcept
{
  std::function<const fuse_core::Constraint&(const Constraints::value_type& uuid__constraint)> to_constraint_ref =
    [](const Constraints::value_type& uuid__constraint) -> const fuse_core::Constraint&
    {
      return *uuid__constraint.second;
    };

  return fuse_core::const_constraint_range(
    boost::make_transform_iterator(constraints_.cbegin(), to_constraint_ref),
    boost::make_transform_iterator(constraints_.cend(), to_constraint_ref));
}

bool HashGraph::variableExists(const fuse_core::UUID& variable_uuid) const noexcept
{
  auto variables_iter = variables_.find(variable_uuid);
  return (variables_iter != variables_.end());
}

bool HashGraph::addVariable(fuse_core::Variable::SharedPtr variable)
{
  // Do nothing if the variable is empty, or the variable already exists
  if (!variable || variableExists(variable->uuid()))
  {
    return false;
  }
  variables_.emplace(variable->uuid(), variable);
  return true;
}

bool HashGraph::removeVariable(const fuse_core::UUID& variable_uuid)
{
  // Check if the variable exists
  auto variables_iter = variables_.find(variable_uuid);
  if (variables_iter == variables_.end())
  {
    return false;
  }
  // Check that this variable is not used by any constraint. Throw a logic_error if the variable is currently used.
  auto cross_reference_iter = constraints_by_variable_uuid_.find(variable_uuid);
  if (cross_reference_iter != constraints_by_variable_uuid_.end() && !cross_reference_iter->second.empty())
  {
    throw std::logic_error("Attempting to remove a variable (" + fuse_core::uuid::to_string(variable_uuid)
      + ") that is used by existing constraints (" + fuse_core::uuid::to_string(cross_reference_iter->second.front())
      + " plus " + std::to_string(cross_reference_iter->second.size() - 1) + " others)");
  }
  // Remove the variable from all containers
  variables_.erase(variables_iter);  // Does not throw
  if (cross_reference_iter != constraints_by_variable_uuid_.end())
  {
    constraints_by_variable_uuid_.erase(cross_reference_iter);
  }
  return true;
}

const fuse_core::Variable& HashGraph::getVariable(const fuse_core::UUID& variable_uuid) const
{
  auto variables_iter = variables_.find(variable_uuid);
  if (variables_iter == variables_.end())
  {
    throw std::out_of_range("The variable UUID " + fuse_core::uuid::to_string(variable_uuid) + " does not exist.");
  }
  return *variables_iter->second;
}

fuse_core::const_variable_range HashGraph::getVariables() const noexcept
{
  std::function<const fuse_core::Variable&(const Variables::value_type& uuid__variable)> to_variable_ref =
    [](const Variables::value_type& uuid__variable) -> const fuse_core::Variable&
    {
      return *uuid__variable.second;
    };

  return fuse_core::const_variable_range(
    boost::make_transform_iterator(variables_.cbegin(), to_variable_ref),
    boost::make_transform_iterator(variables_.cend(), to_variable_ref));
}

void HashGraph::holdVariable(const fuse_core::UUID& variable_uuid, bool hold_constant)
{
  // Adjust the variable setting in the Ceres Problem object
  if (hold_constant)
  {
    variables_on_hold_.insert(variable_uuid);
  }
  else
  {
    variables_on_hold_.erase(variable_uuid);
  }
}

void HashGraph::marginalizeVariable(const fuse_core::UUID& variable_uuid)
{
  throw std::runtime_error("The function 'marginalizeVariable()' has not been implemented yet.");
}

void HashGraph::getCovariance(
  const std::vector<std::pair<fuse_core::UUID, fuse_core::UUID>>& covariance_requests,
  std::vector<std::vector<double>>& covariance_matrices,
  const ceres::Covariance::Options& options) const
{
  // Avoid doing a bunch of work if the request is empty
  if (covariance_requests.empty())
  {
    return;
  }
  // Construct the ceres::Problem object from scratch
  ceres::Problem problem(problem_options_);
  createProblem(problem);
  // The Ceres interface requires that the variable pairs not contain duplicates. Since the covariance matrix is
  // symmetric, requesting Cov(A,B) and Cov(B,A) counts as a duplicate. Create an expression to test a pair of data
  // pointers such that (A,B) == (A,B) OR (B,A)
  auto symmetric_equal = [](const std::pair<const double*, const double*>& x,
                            const std::pair<const double*, const double*>& y)
  {
    return ((x.first == y.first) && (x.second == y.second))
        || ((x.first == y.second) && (x.second == y.first));
  };
  // Convert the covariance requests into the input structure needed by Ceres. Namely, we must convert the variable
  // UUIDs into memory addresses. We create two containers of covariance blocks: one only contains the unique variable
  // pairs that we give to Ceres, and a second that contains all requested variable pairs used to keep the output
  // structure in sync with the request structure.
  std::vector<std::pair<const double*, const double*> > unique_covariance_blocks;
  std::vector<std::pair<const double*, const double*> > all_covariance_blocks;
  all_covariance_blocks.resize(covariance_requests.size());
  covariance_matrices.resize(covariance_requests.size());
  for (size_t i = 0; i < covariance_requests.size(); ++i)
  {
    const auto& request = covariance_requests.at(i);
    auto variable1_iter = variables_.find(request.first);
    if (variable1_iter == variables_.end())
    {
      throw std::out_of_range("The variable UUID " + fuse_core::uuid::to_string(request.first)
                            + " does not exist.");
    }
    auto variable2_iter = variables_.find(request.second);
    if (variable2_iter == variables_.end())
    {
      throw std::out_of_range("The variable UUID " + fuse_core::uuid::to_string(request.second)
                            + " does not exist.");
    }
    // Both variables exist. Continue processing.
    // Create the output covariance matrix
    covariance_matrices[i].resize(variable1_iter->second->size() * variable2_iter->second->size());
    // Add this covariance block to the container of all covariance blocks. This container is in sync with the
    // covariance_requests vector.
    auto& block = all_covariance_blocks.at(i);
    block.first = variable1_iter->second->data();
    block.second = variable2_iter->second->data();
    // Also maintain a container of unique covariance blocks. Since the covariance matrix is symmetric, requesting
    // Cov(X,Y) and Cov(Y,X) counts as a duplicate, so we use our special symmetric_equal function to test.
    if (std::none_of(unique_covariance_blocks.begin(),
                     unique_covariance_blocks.end(),
                     std::bind<bool>(symmetric_equal, block, std::placeholders::_1)))
    {
      unique_covariance_blocks.push_back(block);
    }
  }
  // Call the Ceres function to compute the unique set of requested covariance blocks
  ceres::Covariance covariance(options);
  if (!covariance.Compute(unique_covariance_blocks, &problem))
  {
    throw std::runtime_error("Could not compute requested covariance blocks.");
  }
  // Populate the computed covariance blocks into the output variable.
  // We use the temporary structure to avoid repeated map lookups.
  for (size_t i = 0; i < covariance_requests.size(); ++i)
  {
    if (!covariance.GetCovarianceBlock(all_covariance_blocks.at(i).first,
                                       all_covariance_blocks.at(i).second,
                                       covariance_matrices.at(i).data()))
    {
      throw std::runtime_error("Could not get covariance block for variable UUIDs " +
                               fuse_core::uuid::to_string(covariance_requests.at(i).first) + " and " +
                               fuse_core::uuid::to_string(covariance_requests.at(i).second) + ".");
    }
  }
}

ceres::Solver::Summary HashGraph::optimize(const ceres::Solver::Options& options)
{
  // Construct the ceres::Problem object from scratch
  ceres::Problem problem(problem_options_);
  createProblem(problem);
  // Run the solver. This will update the variables in place.
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  // Return the optimization summary
  return summary;
}

void HashGraph::createProblem(ceres::Problem& problem) const
{
  // Add all the variables to the problem
  for (auto& uuid__variable : variables_)
  {
    fuse_core::Variable& variable = *(uuid__variable.second);
    problem.AddParameterBlock(
      variable.data(),
      variable.size(),
      variable.localParameterization());
    // Handle variables that are held constant
    if (variables_on_hold_.find(variable.uuid()) != variables_on_hold_.end())
    {
      problem.SetParameterBlockConstant(variable.data());
    }
  }
  // Add the constraints
  for (auto& uuid__constraint : constraints_)
  {
    fuse_core::Constraint& constraint = *(uuid__constraint.second);
    // We need the memory address of each variable value referenced by this constraint
    std::vector<double*> parameter_blocks;
    parameter_blocks.reserve(constraint.variables().size());
    for (const auto& uuid : constraint.variables())
    {
      parameter_blocks.push_back(variables_.at(uuid)->data());
    }
    problem.AddResidualBlock(
      constraint.costFunction(),
      constraint.lossFunction(),
      parameter_blocks);
  }
}

}  // namespace fuse_graphs
