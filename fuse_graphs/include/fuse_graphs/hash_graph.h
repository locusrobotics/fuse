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
#ifndef FUSE_GRAPHS_HASH_GRAPH_H
#define FUSE_GRAPHS_HASH_GRAPH_H

#include <fuse_core/constraint.h>
#include <fuse_core/graph.h>
#include <fuse_core/fuse_macros.h>
#include <fuse_core/serialization.h>
#include <fuse_core/uuid.h>
#include <fuse_core/variable.h>
#include <fuse_graphs/hash_graph_params.h>

#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/unordered_map.hpp>
#include <boost/serialization/unordered_set.hpp>
#include <ceres/covariance.h>
#include <ceres/problem.h>
#include <ceres/solver.h>

#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>


namespace fuse_graphs
{

/**
 * @brief This is a concrete implementation of the Graph interface using hashmaps to store the constraints and variables.
 *
 * This is reasonable graph implementation when a large number of variables and constraints are expected, such as with
 * full SLAM and mapping applications. The hashmap overhead may be too expensive for use in high-frequency systems with
 * a basically fixed graph size. Something base on a boost::flat_map or similar may perform better in those situations.
 * The final decision on the graph type should be based actual performance testing.
 *
 * This class is not thread-safe. If used in a multi-threaded application, standard thread synchronization techniques
 * should be used to guard access to the graph.
 */
class HashGraph : public fuse_core::Graph
{
public:
  FUSE_GRAPH_DEFINITIONS(HashGraph)

  /**
   * @brief Constructor
   *
   * @param[in] params HashGraph parameters.
   */
  explicit HashGraph(const HashGraphParams& params = HashGraphParams());

  /**
   * @brief Copy constructor
   *
   * Performs a deep copy of the graph
   */
  HashGraph(const HashGraph& other);

  /**
   * @brief Destructor
   */
  virtual ~HashGraph() = default;

  /**
   * @brief Assignment operator
   *
   * Performs a deep copy of the graph
   */
  HashGraph& operator=(const HashGraph& other);

  /**
   * @brief Clear all variables and constraints from the graph object.
   *
   * The object should be equivalent to a newly constructed object after clear() has been called.
   */
  void clear() override;

  /**
   * @brief Return a deep copy of the graph object.
   *
   * This should include deep copies of all variables and constraints; not pointer copies.
   */
  fuse_core::Graph::UniquePtr clone() const override;

  /**
   * @brief Check if the constraint already exists in the graph
   *
   * Exceptions: None
   * Complexity: O(1) (average)
   *
   * @param[in] constraint_uuid The UUID of the constraint being searched for
   * @return                    True if this constraint already exists, False otherwise
   */
  bool constraintExists(const fuse_core::UUID& constraint_uuid) const noexcept override;

  /**
   * @brief Add a new constraint to the graph
   *
   * Any referenced variables must exist in the graph before the constraint is added. The Graph will share ownership
   * of the constraint.
   *
   * Behavior: If this constraint already exists in the graph, the function will return false.
   * Exceptions: If the constraint's variables do not exist in the graph, a std::logic_error exception will be thrown.
   *             If any unexpected errors occur, an exception will be thrown.
   * Complexity: O(1) (average)
   *
   * @param[in] constraint The new constraint to be added
   * @return               True if the constraint was added, false otherwise
   */
  bool addConstraint(fuse_core::Constraint::SharedPtr constraint) override;

  /**
   * @brief Remove a constraint from the graph
   *
   * Behavior: If this constraint does not exist in the graph, the function will return false.
   * Exceptions: If the constraint UUID does not exist, a std::out_of_range exception will be thrown.
   *             If any unexpected errors occur, an exception will be thrown.
   * Complexity: O(1) (average)
   *
   * @param[in] constraint_uuid The UUID of the constraint to be removed
   * @return                    True if the constraint was removed, false otherwise
   */
  bool removeConstraint(const fuse_core::UUID& constraint_uuid) override;

  /**
   * @brief Read-only access to a constraint from the graph by UUID
   *
   * Exceptions: If the constraint UUID does not exist, a std::out_of_range exception will be thrown.
   * Complexity: O(1) (average)
   *
   * @param[in] constraint_uuid The UUID of the requested constraint
   * @return                    The constraint in the graph with the specified UUID
   */
  const fuse_core::Constraint& getConstraint(const fuse_core::UUID& constraint_uuid) const override;

  /**
   * @brief Read-only access to all of the constraints in the graph
   *
   * Behavior: This function returns iterators pointing to the beginning and end of the collection. No copies
   *           of the constraints are performed at this time.
   * Exceptions: None
   * Complexity: O(1) This function returns in constant time. However, iterating through the returned
   *                  range is naturally O(N).
   *
   * @return A read-only iterator range containing all constraints
   */
  fuse_core::Graph::const_constraint_range getConstraints() const noexcept override;

  /**
   * @brief Read-only access to the subset of constraints that are connected to the specified variable
   *
   * @param[in] variable_uuid The UUID of the variable of interest
   * @return A read-only iterator range containing all constraints that involve the specified variable
   */
  fuse_core::Graph::const_constraint_range getConnectedConstraints(const fuse_core::UUID& variable_uuid) const override;

  /**
   * @brief Check if the variable already exists in the graph
   *
   * Exceptions: None
   * Complexity: O(1) (average)
   *
   * @param[in] variable_uuid The UUID of the variable being searched for
   * @return                  True if this variable already exists, False otherwise
   */
  bool variableExists(const fuse_core::UUID& variable_uuid) const noexcept override;

  /**
   * @brief Add a new variable to the graph
   *
   * The Graph will share ownership of the Variable. If this variable already exists in the graph, the function will
   * return false.
   *
   * Behavior: If this variable already exists in the graph, the function will return false.
   * Exceptions: If any unexpected errors occur, an exception will be thrown.
   * Complexity: O(1) (average)
   *
   * @param[in] variable The new variable to be added
   * @return             True if the variable was added, false otherwise
   */
  bool addVariable(fuse_core::Variable::SharedPtr variable) override;

  /**
   * @brief Remove a variable from the graph
   *
   * Exceptions: If constraints still exist that refer to this variable, a std::logic_error exception will be thrown.
   *             If an unexpected error occurs during the removal, an exception will be thrown.
   * Complexity: O(1) (average)
   *
   * @param[in] variable_uuid The UUID of the variable to be removed
   * @return                  True if the variable was removed, false otherwise
   */
  bool removeVariable(const fuse_core::UUID& variable_uuid) override;

  /**
   * @brief Read-only access to a variable in the graph by UUID
   *
   * Exceptions: If the variable UUID does not exist, a std::out_of_range exception will be thrown.
   * Complexity: O(1) (average)
   *
   * @param[in] variable_uuid The UUID of the requested variable
   * @return                  The variable in the graph with the specified UUID
   */
  const fuse_core::Variable& getVariable(const fuse_core::UUID& variable_uuid) const override;

  /**
   * @brief Read-only access to all of the variables in the graph
   *
   * Behavior: This function returns iterators pointing to the beginning and end of the collection. No copies
   *           of the variables are performed at this time.
   * Exceptions: None
   * Complexity: O(1) This function returns in constant time. However, iterating through the returned
   *                  range is naturally O(N).
   *
   * @return A read-only iterator range containing all variables
   */
  fuse_core::Graph::const_variable_range getVariables() const noexcept override;

  /**
   * @brief Configure a variable to hold its current value during optimization
   *
   * Once set, the specified variable's value will no longer change during any subsequent optimization. To 'unhold'
   * a previously held variable, call Graph::holdVariable() with the \p hold_constant parameter set to false.
   *
   * Exceptions: If the variable does not exist, a std::out_of_range exception will be thrown.
   * Complexity: O(1) (average)
   *
   * @param[in] variable_uuid The variable to adjust
   * @param[in] hold_constant Flag indicating if the variable's value should be held constant during optimization,
   *                          or if the variable's value is allowed to change during optimization.
   */
  void holdVariable(const fuse_core::UUID& variable_uuid, bool hold_constant = true) override;

  /**
   * @brief Check whether a variable is on hold or not
   *
   * @param[in] variable_uuid The variable to test
   * @return True if the variable is on hold, false otherwise
   */
  bool isVariableOnHold(const fuse_core::UUID& variable_uuid) const override;

  /**
   * @brief Compute the marginal covariance blocks for the requested set of variable pairs.
   *
   * To compute the marginal variance of a single variable, simply supply the same variable UUID for both members of
   * of the request pair. Computing the marginal covariance is an expensive operation; grouping multiple
   * variable pairs into a single call will be much faster than calling this function for each pair individually.
   *
   * Exceptions: If the request contains unknown variables, a std::out_of_range exception will be thrown.
   *             If the covariance calculation fails, a std::runtime_error exception will be thrown.
   * Complexity: O(N) in the best case, O(N^3) in the worst case, where N is the total number of variables in
   *             the graph. In practice, it is significantly cheaper than the worst-case bound, but it is still
   *             an expensive operation.
   *
   * @param[in]  covariance_requests A set of variable UUID pairs for which the marginal covariance is desired.
   * @param[out] covariance_matrices The dense covariance blocks of the requests.
   * @param[in]  options             A Ceres Covariance Options structure that controls the method and settings used
   *                                 to compute the covariance blocks.
   * @param[in]  use_tangent_space   Flag indicating if the covariance should be computed in the variable's tangent
   *                                 space/local coordinates. Otherwise it is computed in the variable's parameter
   *                                 space.
   */
  void getCovariance(
    const std::vector<std::pair<fuse_core::UUID, fuse_core::UUID>>& covariance_requests,
    std::vector<std::vector<double>>& covariance_matrices,
    const ceres::Covariance::Options& options = ceres::Covariance::Options(),
    const bool use_tangent_space = true) const override;

  /**
   * @brief Optimize the values of the current set of variables, given the current set of constraints.
   *
   * After the call, the values in the graph will be updated to the latest values. This is where the
   * "work" of the optimization system is performed. As such, it is often a long-running process.
   *
   * Complexity: O(N) in the best case, O(M*N^3) in the worst case, where N is the total number of variables
   *             in the graph, and M is the maximum number of allowed iterations.
   *
   * @param[in] options An optional Ceres Solver::Options object that controls various aspects of the optimizer.
   *                    See https://ceres-solver.googlesource.com/ceres-solver/+/master/include/ceres/solver.h#59
   * @return            A Ceres Solver Summary structure containing information about the optimization process
   */
  ceres::Solver::Summary optimize(const ceres::Solver::Options& options = ceres::Solver::Options()) override;

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
  ceres::Solver::Summary optimizeFor(
    const std::chrono::nanoseconds& max_optimization_time,
    const ceres::Solver::Options& options = ceres::Solver::Options()) override;

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
  bool evaluate(double* cost, std::vector<double>* residuals = nullptr, std::vector<double>* gradient = nullptr,
                const ceres::Problem::EvaluateOptions& options = ceres::Problem::EvaluateOptions()) const override;

  /**
   * @brief Print a human-readable description of the graph to the provided stream.
   *
   * @param[out] stream The stream to write to. Defaults to stdout.
   */
  void print(std::ostream& stream = std::cout) const override;

protected:
  // Define some helpful typedefs
  using Constraints = std::unordered_map<fuse_core::UUID, fuse_core::Constraint::SharedPtr, fuse_core::uuid::hash>;
  using Variables = std::unordered_map<fuse_core::UUID, fuse_core::Variable::SharedPtr, fuse_core::uuid::hash>;
  using VariableSet = std::unordered_set<fuse_core::UUID, fuse_core::uuid::hash>;
  using CrossReference = std::unordered_map<fuse_core::UUID, std::vector<fuse_core::UUID>, fuse_core::uuid::hash>;

  Constraints constraints_;  //!< The set of all constraints
  CrossReference constraints_by_variable_uuid_;  //!< Index all of the constraints by variable uuids
  ceres::Problem::Options problem_options_;  //!< User-defined options to be applied to all constructed ceres::Problems
  Variables variables_;  //!< The set of all variables
  VariableSet variables_on_hold_;  //!< The set of variables that should be held constant

  /**
   * @brief Populate a ceres::Problem object using the current set of variables and constraints
   *
   * This function assumes the provided variables and constraints are consistent. No checks are performed for missing
   * variables or constraints.
   *
   * @param[out] problem The ceres::Problem object to modify
   */
  void createProblem(ceres::Problem& problem) const;

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
    archive & boost::serialization::base_object<fuse_core::Graph>(*this);
    archive & constraints_;
    archive & constraints_by_variable_uuid_;
    archive & problem_options_;
    archive & variables_;
    archive & variables_on_hold_;
  }
};

}  // namespace fuse_graphs

namespace boost
{
namespace serialization
{

/**
 * @brief Serialize a ceres::Problem::Options object using Boost Serialization
 */
template<class Archive>
void serialize(Archive& archive, ceres::Problem::Options& options, const unsigned int /* version */)
{
  archive & options.cost_function_ownership;
  archive & options.disable_all_safety_checks;
  archive & options.enable_fast_removal;
  archive & options.local_parameterization_ownership;
  archive & options.loss_function_ownership;
}

}  // namespace serialization
}  // namespace boost

BOOST_CLASS_EXPORT_KEY(fuse_graphs::HashGraph);

#endif  // FUSE_GRAPHS_HASH_GRAPH_H
