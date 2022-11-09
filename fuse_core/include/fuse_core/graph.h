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
#include <fuse_core/fuse_macros.h>
#include <fuse_core/serialization.h>
#include <fuse_core/transaction.h>
#include <fuse_core/uuid.h>
#include <fuse_core/variable.h>

#include <boost/core/demangle.hpp>
#include <boost/range/any_range.hpp>
#include <boost/serialization/access.hpp>
#include <boost/type_index/stl_type_index.hpp>
#include <ceres/covariance.h>
#include <ceres/problem.h>
#include <ceres/solver.h>

#include <memory>
#include <numeric>
#include <ostream>
#include <string>
#include <utility>
#include <vector>


/**
 * @brief Implementation of the serialize() and deserialize() member functions for derived classes
 *
 * Usage:
 * @code{.cpp}
 * class Derived : public Graph
 * {
 * public:
 *   FUSE_GRAPH_SERIALIZE_DEFINITION(Derived)
 *   // The rest of the derived graph implementation
 * }
 * @endcode
 */
#define FUSE_GRAPH_SERIALIZE_DEFINITION(...) \
  void serialize(fuse_core::BinaryOutputArchive& archive) const override \
  { \
    archive << *this; \
  }  /* NOLINT */ \
  void serialize(fuse_core::TextOutputArchive& archive) const override \
  { \
    archive << *this; \
  }  /* NOLINT */ \
  void deserialize(fuse_core::BinaryInputArchive& archive) override \
  { \
    archive >> *this; \
  }  /* NOLINT */ \
  void deserialize(fuse_core::TextInputArchive& archive) override \
  { \
    archive >> *this; \
  }

/**
 * @brief Implements the type() member function using the suggested implementation
 *
 * Also creates a static detail::type() function that may be used without an object instance
 *
 * Usage:
 * @code{.cpp}
 * class Derived : public Graph
 * {
 * public:
 *   FUSE_GRAPH_TYPE_DEFINITION(Derived)
 *   // The rest of the derived graph implementation
 * }
 * @endcode
 */
#define FUSE_GRAPH_TYPE_DEFINITION(...) \
  struct detail \
  { \
    static std::string type() \
    { \
      return boost::typeindex::stl_type_index::type_id<__VA_ARGS__>().pretty_name(); \
    }  /* NOLINT */ \
  };  /* NOLINT */ \
  std::string type() const override \
  { \
    return detail::type(); \
  }

/**
* @brief Convenience function that creates the required pointer aliases, and type() method
*
* Usage:
* @code{.cpp}
* class Derived : public Graph
* {
* public:
*   FUSE_GRAPH_DEFINITIONS(Derived)
*   // The rest of the derived graph implementation
* }
* @endcode
*/
#define FUSE_GRAPH_DEFINITIONS(...) \
  FUSE_SMART_PTR_DEFINITIONS(__VA_ARGS__) \
  FUSE_GRAPH_TYPE_DEFINITION(__VA_ARGS__) \
  FUSE_GRAPH_SERIALIZE_DEFINITION(__VA_ARGS__)


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
  FUSE_SMART_PTR_ALIASES_ONLY(Graph)

  /**
   * @brief A range of fuse_core::Constraint objects
   *
   * An object representing a range defined by two iterators. It has begin() and end() methods (which means it can
   * be used in range-based for loops), an empty() method, and a front() method for directly accessing the first
   * member. When dereferenced, an iterator returns a const Constraint&.
   */
  using const_constraint_range = boost::any_range<const Constraint, boost::forward_traversal_tag>;

  /**
   * @brief A range of fuse_core::Variable objects
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
   * @brief Returns a unique name for this graph type.
   *
   * The constraint type string must be unique for each class. As such, the fully-qualified class name is an excellent
   * choice for the type string.
   */
  virtual std::string type() const = 0;

  /**
   * @brief Clear all variables and constraints from the graph object.
   *
   * The object should be equivalent to a newly constructed object after clear() has been called.
   */
  virtual void clear() = 0;

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
   * @brief Read-only access to the subset of constraints that are connected to the specified variable
   *
   * @param[in] variable_uuid The UUID of the variable of interest
   * @return A read-only iterator range containing all constraints that involve the specified variable
   */
  virtual const_constraint_range getConnectedConstraints(const UUID& variable_uuid) const = 0;

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
   * @brief Read-only access to the subset of variables that are connected to the specified constraint
   *
   * @param[in] constraint_uuid The UUID of the constraint of interest
   * @return A read-only iterator range containing all variables that involve the specified constraint
   */
  virtual const_variable_range getConnectedVariables(const UUID& constraint_uuid) const;

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
   * @brief Check whether a variable is on hold or not
   *
   * @param[in] variable_uuid The variable to test
   * @return True if the variable is on hold, false otherwise
   */
  virtual bool isVariableOnHold(const UUID& variable_uuid) const = 0;

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
   * @param[in]  use_tangent_space   Flag indicating if the covariance should be computed in the variable's tangent
   *                                 space/local coordinates. Otherwise it is computed in the variable's parameter
   *                                 space.
   */
  virtual void getCovariance(
    const std::vector<std::pair<UUID, UUID>>& covariance_requests,
    std::vector<std::vector<double>>& covariance_matrices,
    const ceres::Covariance::Options& options = ceres::Covariance::Options(),
    const bool use_tangent_space = true) const = 0;

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

  /**
   * @brief Optimize the values of the current set of variables, given the current set of constraints for a maximum
   * amount of time.
   *
   * The \p max_optimization_time should be viewed as a "best effort" limit, and the actual optimization time may
   * exceed this limit by a small amount. After the call, the values in the graph will be updated to the latest values.
   *
   * @param[in] max_optimization_time The maximum allowed duration of the optimization call
   * @param[in] options An optional Ceres Solver::Options object that controls various aspects of the optimizer.
   *                    See https://ceres-solver.googlesource.com/ceres-solver/+/master/include/ceres/solver.h#59
   * @return            A Ceres Solver Summary structure containing information about the optimization process
   */
  virtual ceres::Solver::Summary optimizeFor(
    const std::chrono::nanoseconds& max_optimization_time,
    const ceres::Solver::Options& options = ceres::Solver::Options()) = 0;

  /**
   * @brief Evalute the values of the current set of variables, given the current set of constraints.
   *
   * The values in the graph do not change after the call.
   *
   * If any of the output arguments is nullptr, it will not be evaluated. This mimics the ceres::Problem::Evaluate
   * method API. Here all output arguments default to nullptr except for the cost.
   *
   * TODO(efernandez) support jacobian output argument
   * The jacobian output argument is not exposed at the moment because its type is a CRSMatrix, that probably needs to
   * be converted to another type.
   *
   * @param[out] cost      The cost of the entire problem represented by the graph.
   * @param[out] residuals The residuals of all constraints.
   * @param[out] gradient  The gradient for all constraints evaluated at the values of the current set of variables.
   * @param[in]  options   An optional Ceres Problem::EvaluateOptions object that controls various aspects of the
   *                       problem evaluation.
   *                       See https://ceres-solver.googlesource.com/ceres-solver/+/master/include/ceres/problem.h#401
   * @return True if the problem evaluation was successful; False, otherwise.
   */
  virtual bool evaluate(double* cost, std::vector<double>* residuals = nullptr, std::vector<double>* gradient = nullptr,
                        const ceres::Problem::EvaluateOptions& options = ceres::Problem::EvaluateOptions()) const = 0;

  /**
   * @brief Structure containing the cost and residual information for a single constraint.
   */
  struct ConstraintCost
  {
    double cost {};  //!< The pre-loss-function cost of the constraint, computed as the norm of the residuals
    double loss {};  //!< The final cost of the constraint after any loss functions have been applied
    std::vector<double> residuals;  //!< The individual residuals for the constraint
  };

  /**
   * @brief Compute the residual information for a collection of constraints
   *
   * If any of the requested constraints does not exist, an exception will be thrown.
   *
   * @param[in]  first   An iterator pointing to the first UUID of the desired constraints
   * @param[in]  last    An iterator pointing to one passed the last UUID of the desired constraints
   * @param[out] output  An output iterator capable of assignment to a ConstraintCost object
   */
  template <class UuidForwardIterator, class OutputIterator>
  void getConstraintCosts(
    UuidForwardIterator first,
    UuidForwardIterator last,
    OutputIterator output);

  /**
   * @brief Print a human-readable description of the graph to the provided stream.
   *
   * @param[out] stream The stream to write to. Defaults to stdout.
   */
  virtual void print(std::ostream& stream = std::cout) const = 0;

  /**
   * @brief Serialize this graph into the provided binary archive
   *
   * This can/should be implemented as follows in all derived classes:
   * @code{.cpp}
   * archive << *this;
   * @endcode
   *
   * @param[out] archive - The archive to serialize this graph into
   */
  virtual void serialize(fuse_core::BinaryOutputArchive& /* archive */) const = 0;

  /**
   * @brief Serialize this graph into the provided text archive
   *
   * This can/should be implemented as follows in all derived classes:
   * @code{.cpp}
   * archive << *this;
   * @endcode
   *
   * @param[out] archive - The archive to serialize this graph into
   */
  virtual void serialize(fuse_core::TextOutputArchive& /* archive */) const = 0;

  /**
   * @brief Deserialize data from the provided binary archive into this graph
   *
   * This can/should be implemented as follows in all derived classes:
   * @code{.cpp}
   * archive >> *this;
   * @endcode
   *
   * @param[in] archive - The archive holding serialized graph data
   */
  virtual void deserialize(fuse_core::BinaryInputArchive& /* archive */) = 0;

  /**
   * @brief Deserialize data from the provided text archive into this graph
   *
   * This can/should be implemented as follows in all derived classes:
   * @code{.cpp}
   * archive >> *this;
   * @endcode
   *
   * @param[in] archive - The archive holding serialized graph data
   */
  virtual void deserialize(fuse_core::TextInputArchive& /* archive */) = 0;

private:
  // Allow Boost Serialization access to private methods
  friend class boost::serialization::access;

  /**
   * @brief The Boost Serialize method that serializes all of the data members in to/out of the archive
   *
   * This method, or a combination of save() and load() methods, must be implemented by all derived classes. See
   * documentation on Boost Serialization for information on how to implement the serialize() method.
   * https://www.boost.org/doc/libs/1_70_0/libs/serialization/doc/
   *
   * @param[in/out] archive - The archive object that holds the serialized class members
   * @param[in] version - The version of the archive being read/written. Generally unused.
   */
  template<class Archive>
  void serialize(Archive& /* archive */, const unsigned int /* version */)
  {
  }
};

/**
 * Stream operator for printing Graph objects.
 */
std::ostream& operator <<(std::ostream& stream, const Graph& graph);


template <class UuidForwardIterator, class OutputIterator>
void Graph::getConstraintCosts(
  UuidForwardIterator first,
  UuidForwardIterator last,
  OutputIterator output)
{
  // @todo(swilliams) When I eventually refactor the Graph class to implement more of the requirements in the base
  //                  class, it should be possible to make better use of the Problem object and avoid creating and
  //                  deleting the cost and loss functions.
  while (first != last)
  {
    // Get the next requested constraint
    const auto& constraint = getConstraint(*first);
    // Collect all of the involved variables
    auto parameter_blocks = std::vector<const double*>();
    parameter_blocks.reserve(constraint.variables().size());
    for (auto variable_uuid : constraint.variables())
    {
      const auto& variable = getVariable(variable_uuid);
      parameter_blocks.push_back(variable.data());
    }
    // Compute the residuals for this constraint using the cost function
    auto cost_function = std::unique_ptr<ceres::CostFunction>(constraint.costFunction());
    auto cost = ConstraintCost();
    cost.residuals.resize(cost_function->num_residuals());
    cost_function->Evaluate(parameter_blocks.data(), cost.residuals.data(), nullptr);
    // Compute the combined cost
    cost.cost =
      std::sqrt(std::inner_product(cost.residuals.begin(), cost.residuals.end(), cost.residuals.begin(), 0.0));
    // Apply the loss function, if one is configured
    auto loss_function = std::unique_ptr<ceres::LossFunction>(constraint.lossFunction());
    if (loss_function)
    {
      double loss_result[3];  // The Loss function returns the loss-adjusted cost plus the first and second derivative
      loss_function->Evaluate(cost.cost, loss_result);
      cost.loss = loss_result[0];
    }
    else
    {
      cost.loss = cost.cost;
    }
    // Add the final cost to the output
    *output++ = std::move(cost);
    ++first;
  }
}

}  // namespace fuse_core

#endif  // FUSE_CORE_GRAPH_H
