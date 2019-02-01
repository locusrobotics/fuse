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
#ifndef FUSE_CORE_GRAPH_H
#define FUSE_CORE_GRAPH_H

#include <fuse_core/constraint.h>
#include <fuse_core/macros.h>
#include <fuse_core/transaction.h>
#include <fuse_core/uuid.h>
#include <fuse_core/variable.h>

#include <boost/range/any_range.hpp>
#include <ceres/covariance.h>
#include <ceres/solver.h>

#include <utility>
#include <vector>


namespace fuse_core
{

/**
 * @brief This is an interface definition describing the collection of constraints and variables that form the factor
 * graph, a graphical model of a nonlinear least-squares problem.
 *
 * Methods are provided to add, remove, and access the constraints and variables by several criteria, as well as to
 * optimize the variable values. Derived classes may store the constraints and variables using any mechanism, but the
 * same interface must be provided.
 */
class Graph
{
public:
  SMART_PTR_ALIASES_ONLY(Graph);

  /**
   * @brief A range of fuse_ros::Constraint objects
   *
   * An object representing a range defined by two iterators. It has begin() and end() methods (which means it can
   * be used in range-based for loops), an empty() method, and a front() method for directly accessing the first
   * member. When dereferenced, an iterator returns a const Constraint&.
   */
  using const_constraint_range = boost::any_range<const Constraint, boost::forward_traversal_tag>;

  /**
   * @brief A range of fuse_ros::Variable objects
   *
   * An object representing a range defined by two iterators. It has begin() and end() methods (which means it can
   * be used in range-based for loops), an empty() method, and a front() method for directly accessing the first
   * member. When dereferenced, an iterator returns a const Variable&.
   */
  using const_variable_range = boost::any_range<const Variable, boost::forward_traversal_tag>;

  /**
   * @brief Constructor
   *
   */
  Graph() = default;

  /**
   * @brief Destructor
   */
  virtual ~Graph() = default;

  /**
   * @brief Return a deep copy of the graph object.
   *
   * This should include deep copies of all variables and constraints; not pointer copies.
   */
  virtual Graph::UniquePtr clone() const = 0;

  /**
   * @brief Check if the constraint already exists in the graph
   *
   * @param[in] constraint_uuid The UUID of the constraint being searched for
   * @return                    True if this constraint already exists, False otherwise
   */
  virtual bool constraintExists(const UUID& constraint_uuid) const = 0;

  /**
   * @brief Add a new constraint to the graph
   *
   * Any referenced variables must exist in the graph before the constraint is added. The Graph will share ownership
   * of the constraint. If this constraint already exists in the graph, the function will return false.
   *
   * @param[in] constraint The new constraint to be added
   * @return               True if the constraint was added, false otherwise
   */
  virtual bool addConstraint(Constraint::SharedPtr constraint) = 0;

  /**
   * @brief Remove a constraint from the graph
   *
   * @param[in] constraint_uuid The UUID of the constraint to be removed
   * @return                    True if the constraint was removed, false otherwise
   */
  virtual bool removeConstraint(const UUID& constraint_uuid) = 0;

  /**
   * @brief Read-only access to a constraint from the graph by UUID
   *
   * If the requested constraint does not exist, an exception will be thrown.
   *
   * @param[in] constraint_uuid The UUID of the requested constraint
   * @return                    The constraint in the graph with the specified UUID
   */
  virtual const Constraint& getConstraint(const UUID& constraint_uuid) const = 0;

  /**
   * @brief Read-only access to all of the constraints in the graph
   *
   * @return A read-only iterator range containing all constraints
   */
  virtual const_constraint_range getConstraints() const = 0;

  /**
   * @brief Check if the variable already exists in the graph
   *
   * @param[in] variable_uuid The UUID of the variable being searched for
   * @return                  True if this variable already exists, False otherwise
   */
  virtual bool variableExists(const UUID& variable_uuid) const = 0;

  /**
   * @brief Add a new variable to the graph
   *
   * The Graph will share ownership of the Variable. If this variable already exists in the graph, the function will
   * return false.
   *
   * @param[in] variable The new variable to be added
   * @return             True if the variable was added, false otherwise
   */
  virtual bool addVariable(Variable::SharedPtr variable) = 0;

  /**
   * @brief Remove a variable from the graph
   *
   * @param[in] variable_uuid The UUID of the variable to be removed
   * @return                  True if the variable was removed, false otherwise
   */
  virtual bool removeVariable(const UUID& variable_uuid) = 0;

  /**
   * @brief Read-only access to a variable in the graph by UUID
   *
   * If the requested variable does not exist, an empty pointer will be returned.
   *
   * @param[in] variable_uuid The UUID of the requested variable
   * @return                  The variable in the graph with the specified UUID
   */
  virtual const Variable& getVariable(const UUID& variable_uuid) const = 0;

  /**
   * @brief Read-only access to all of the variables in the graph
   *
   * @return A read-only iterator range containing all variables
   */
  virtual const_variable_range getVariables() const = 0;

  /**
   * @brief Configure a variable to hold its current value constant during optimization
   *
   * Once set, the specified variable's value will no longer change during any subsequent optimization. To 'unhold'
   * a previously held variable, call Graph::holdVariable() with the \p hold_constant parameter set to false.
   *
   * @param[in] variable_uuid The variable to adjust
   * @param[in] hold_constant Flag indicating if the variable's value should be held constant during optimization,
   *                          or if the variable's value is allowed to change during optimization.
   */
  virtual void holdVariable(const UUID& variable_uuid, bool hold_constant = true) = 0;

  /**
   * @brief Marginalize out the provided variable from the graph
   *
   * Perform a marginalization operation, computing a new constraint that represents the remaining information from
   * all connected constraints on all the adjacent variables. The marginalized variable will be removed from the graph,
   * along with all constraints connected to the marginalized variable. The new computed constraint will be added to
   * the graph. Because this new constraint was computed from the linear system, all adjacent variables values will be
   * held to the current optimal value to ensure the constraint remains consistent.
   *
   * @param[in] variable_uuid The UUID of the variable to marginalize out of the problem
   */
  virtual void marginalizeVariable(const UUID& variable_uuid) = 0;

  /**
   * @brief Marginalize out all of the provided variables from the graph
   *
   * This performs the same marginalization operation as described in Graph::marginalizeVariable(), but to all of the
   * provided variables. This is not simply a loop around the wrapper around Graph::marginalizeVariable() function call,
   * but rather inspects the graph structure to combine several variables into a single marginalization operation when
   * possible. Thus, when marginalizing out many variables it is likely more efficient to perform a single call to
   * Graph::marginalizeVariables() than multiple calls to Graph::marginalizeVariable(). However, this call is guaranteed
   * to leave the graph in the same state as if successive calls were made to Graph::marginalizeVariable(). Namely,
   * all marginalized variables will be removed from the graph, all constraints connected to at least one marginalized
   * variable will be removed from the graph, new constraints will be added to the graph that constitute the remaining
   * information on the adjacent variables from the removed constraints, and all adjacent variables will be held
   * constant during future optimizations.
   *
   * @param[in] variable_uuids The UUIDs of the variables to marginalize out of the problem
   */
  virtual void marginalizeVariables(const std::vector<UUID>& variable_uuids);

  /**
   * @brief Compute the marginal covariance blocks for the requested set of variable pairs.
   *
   * To compute the marginal variance of a single variable, simply supply the same variable UUID for both members of
   * of the request pair. Computing the marginal covariance is an expensive operation; grouping multiple
   * variable pairs into a single call will be much faster than calling this function for each pair individually. The
   * marginal covariances can only be computed after calling Graph::computeUpdates() or Graph::optimize().
   *
   * @param[in]  covariance_requests A set of variable UUID pairs for which the marginal covariance is desired.
   * @param[out] covariance_matrices The dense covariance blocks of the requests.
   * @param[in]  options             A Ceres Covariance Options structure that controls the method and settings used
   *                                 to compute the covariance blocks.
   */
  virtual void getCovariance(
    const std::vector<std::pair<UUID, UUID> >& covariance_requests,
    std::vector<std::vector<double> >& covariance_matrices,
    const ceres::Covariance::Options& options = ceres::Covariance::Options()) const = 0;

  /**
   * @brief Update the graph with the contents of a transaction
   *
   * @param[in] transaction A set of variable and constraints additions and deletions
   */
  void update(const Transaction& transaction);

  /**
   * @brief Optimize the values of the current set of variables, given the current set of constraints.
   *
   * After the call, the values in the graph will be updated to the latest values.
   *
   * @param[in] options An optional Ceres Solver::Options object that controls various aspects of the optimizer.
   *                    See https://ceres-solver.googlesource.com/ceres-solver/+/master/include/ceres/solver.h#59
   * @return            A Ceres Solver Summary structure containing information about the optimization process
   */
  virtual ceres::Solver::Summary optimize(const ceres::Solver::Options& options = ceres::Solver::Options()) = 0;
};

}  // namespace fuse_core

#endif  // FUSE_CORE_GRAPH_H
