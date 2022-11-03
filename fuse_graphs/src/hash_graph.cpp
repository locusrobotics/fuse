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
#include <pluginlib/class_list_macros.hpp>

#include <boost/iterator/transform_iterator.hpp>
#include <boost/serialization/export.hpp>

#include <algorithm>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>


namespace fuse_graphs
{

HashGraph::HashGraph(const HashGraphParams& params) :
  problem_options_(params.problem_options)
{
  // Set Ceres loss function ownership according to the fuse_core::Loss specification
  problem_options_.loss_function_ownership = fuse_core::Loss::Ownership;
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

void HashGraph::clear()
{
  constraints_.clear();
  constraints_by_variable_uuid_.clear();
  variables_.clear();
  variables_on_hold_.clear();
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
                             ") that uses an unknown variable (" + fuse_core::uuid::to_string(variable_uuid) + ").");
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

fuse_core::Graph::const_constraint_range HashGraph::getConstraints() const noexcept
{
  std::function<const fuse_core::Constraint&(const Constraints::value_type& uuid__constraint)> to_constraint_ref =
    [](const Constraints::value_type& uuid__constraint) -> const fuse_core::Constraint&
    {
      return *uuid__constraint.second;
    };

  return fuse_core::Graph::const_constraint_range(
    boost::make_transform_iterator(constraints_.cbegin(), to_constraint_ref),
    boost::make_transform_iterator(constraints_.cend(), to_constraint_ref));
}

fuse_core::Graph::const_constraint_range HashGraph::getConnectedConstraints(const fuse_core::UUID& variable_uuid) const
{
  auto cross_reference_iter = constraints_by_variable_uuid_.find(variable_uuid);
  if (cross_reference_iter != constraints_by_variable_uuid_.end())
  {
    std::function<const fuse_core::Constraint&(const fuse_core::UUID& constraint_uuid)> uuid_to_constraint_ref =
      [this](const fuse_core::UUID& constraint_uuid) -> const fuse_core::Constraint&
      {
        return this->getConstraint(constraint_uuid);
      };

    const auto& constraints = cross_reference_iter->second;
    return fuse_core::Graph::const_constraint_range(
      boost::make_transform_iterator(constraints.cbegin(), uuid_to_constraint_ref),
      boost::make_transform_iterator(constraints.cend(), uuid_to_constraint_ref));
  }
  else if (variableExists(variable_uuid))
  {
    // User requested a valid variable, but there are no attached constraints. Return an empty range.
    return fuse_core::Graph::const_constraint_range();
  }
  else
  {
    // We only want to throw if the requested variable does not exist.
    throw std::logic_error("Attempting to access constraints connected to variable ("
        + fuse_core::uuid::to_string(variable_uuid) + "), but that variable does not exist in this graph.");
  }
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
  if (variable->holdConstant())
  {
    variables_on_hold_.insert(variable->uuid());
  }
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
      + " plus " + std::to_string(cross_reference_iter->second.size() - 1) + " others).");
  }
  // Remove the variable from all containers
  variables_.erase(variables_iter);  // Does not throw
  if (cross_reference_iter != constraints_by_variable_uuid_.end())
  {
    constraints_by_variable_uuid_.erase(cross_reference_iter);
  }
  variables_on_hold_.erase(variable_uuid);
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

fuse_core::Graph::const_variable_range HashGraph::getVariables() const noexcept
{
  std::function<const fuse_core::Variable&(const Variables::value_type& uuid__variable)> to_variable_ref =
    [](const Variables::value_type& uuid__variable) -> const fuse_core::Variable&
    {
      return *uuid__variable.second;
    };

  return fuse_core::Graph::const_variable_range(
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

bool HashGraph::isVariableOnHold(const fuse_core::UUID& variable_uuid) const
{
  return variables_on_hold_.find(variable_uuid) != variables_on_hold_.end();
}

void HashGraph::getCovariance(
  const std::vector<std::pair<fuse_core::UUID, fuse_core::UUID>>& covariance_requests,
  std::vector<std::vector<double>>& covariance_matrices,
  const ceres::Covariance::Options& options,
  const bool use_tangent_space) const
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
    if (use_tangent_space)
    {
      covariance_matrices[i].resize(variable1_iter->second->localSize() * variable2_iter->second->localSize());
    }
    else
    {
      covariance_matrices[i].resize(variable1_iter->second->size() * variable2_iter->second->size());
    }
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
  if (use_tangent_space)
  {
    for (size_t i = 0; i < covariance_requests.size(); ++i)
    {
      const auto& block = all_covariance_blocks.at(i);
      auto& output_matrix = covariance_matrices.at(i);
      if (!covariance.GetCovarianceBlockInTangentSpace(block.first,
                                                       block.second,
                                                       output_matrix.data()))
      {
        const auto& request = covariance_requests.at(i);
        throw std::runtime_error("Could not get covariance block for variable UUIDs " +
                                 fuse_core::uuid::to_string(request.first) + " and " +
                                 fuse_core::uuid::to_string(request.second) + ".");
      }
    }
  }
  else
  {
    for (size_t i = 0; i < covariance_requests.size(); ++i)
    {
      const auto& block = all_covariance_blocks.at(i);
      auto& output_matrix = covariance_matrices.at(i);
      if (!covariance.GetCovarianceBlock(block.first,
                                         block.second,
                                         output_matrix.data()))
      {
        const auto& request = covariance_requests.at(i);
        throw std::runtime_error("Could not get covariance block for variable UUIDs " +
                                 fuse_core::uuid::to_string(request.first) + " and " +
                                 fuse_core::uuid::to_string(request.second) + ".");
      }
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

ceres::Solver::Summary HashGraph::optimizeFor(
  const std::chrono::nanoseconds& max_optimization_time,
  const ceres::Solver::Options& options)
{
  auto start = std::chrono::system_clock::now();
  // Construct the ceres::Problem object from scratch
  ceres::Problem problem(problem_options_);
  createProblem(problem);
  auto created_problem = std::chrono::system_clock::now();
  // Modify the options to enforce the maximum time
  std::chrono::nanoseconds remaining = max_optimization_time - (created_problem - start);
  auto time_constrained_options = options;
  time_constrained_options.max_solver_time_in_seconds = std::max(0.0, remaining.seconds());
  // Run the solver. This will update the variables in place.
  ceres::Solver::Summary summary;
  ceres::Solve(time_constrained_options, &problem, &summary);
  // Return the optimization summary
  return summary;
}

bool HashGraph::evaluate(double* cost, std::vector<double>* residuals, std::vector<double>* gradient,
                         const ceres::Problem::EvaluateOptions& options) const
{
  ceres::Problem problem(problem_options_);
  createProblem(problem);

  return problem.Evaluate(options, cost, residuals, gradient, nullptr);
}

void HashGraph::print(std::ostream& stream) const
{
  stream << "HashGraph\n"
         << "  constraints:\n";
  for (const auto& constraint : constraints_)
  {
    stream << "   - " << *constraint.second << "\n";
  }
  stream << "  variables:\n";
  for (const auto& variable : variables_)
  {
    const auto is_on_hold = variables_on_hold_.find(variable.first) != variables_on_hold_.end();

    stream << "   - " << *variable.second << "\n"
           << "     on_hold: " << std::boolalpha << is_on_hold << "\n";
  }
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
    // Handle optimization bounds
    for (size_t index = 0; index < variable.size(); ++index)
    {
      auto lower_bound = variable.lowerBound(index);
      if (lower_bound > std::numeric_limits<double>::lowest())
      {
        problem.SetParameterLowerBound(variable.data(), index, lower_bound);
      }
      auto upper_bound = variable.upperBound(index);
      if (upper_bound < std::numeric_limits<double>::max())
      {
        problem.SetParameterUpperBound(variable.data(), index, upper_bound);
      }
    }
    // Handle variables that are held constant
    if (variables_on_hold_.find(variable.uuid()) != variables_on_hold_.end())
    {
      problem.SetParameterBlockConstant(variable.data());
    }
  }
  // Add the constraints
  std::vector<double*> parameter_blocks;
  for (auto& uuid__constraint : constraints_)
  {
    fuse_core::Constraint& constraint = *(uuid__constraint.second);
    // We need the memory address of each variable value referenced by this constraint
    parameter_blocks.clear();
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

BOOST_CLASS_EXPORT_IMPLEMENT(fuse_graphs::HashGraph)
PLUGINLIB_EXPORT_CLASS(fuse_graphs::HashGraph, fuse_core::Graph)
