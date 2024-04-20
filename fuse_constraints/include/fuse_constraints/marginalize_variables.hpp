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
#ifndef FUSE_CONSTRAINTS__MARGINALIZE_VARIABLES_HPP_
#define FUSE_CONSTRAINTS__MARGINALIZE_VARIABLES_HPP_

#include <ceres/cost_function.h>

#include <algorithm>
#include <cassert>
#include <iterator>
#include <ostream>
#include <string>
#include <vector>

#include <fuse_constraints/marginal_constraint.hpp>
#include <fuse_constraints/uuid_ordering.hpp>
#include <fuse_core/constraint.hpp>
#include <fuse_core/eigen.hpp>
#include <fuse_core/fuse_macros.hpp>
#include <fuse_core/graph.hpp>
#include <fuse_core/transaction.hpp>
#include <fuse_core/variable.hpp>

#include <boost/iterator/transform_iterator.hpp>


namespace fuse_constraints
{

/**
 * @brief Compute an efficient elimination order for the marginalized variables
 *
 * The marginalized_variables are guaranteed to be placed before any additional connected variables.
 * Each time a variable is eliminated from the system, the resulting reduced system is independent
 * of the eliminated variable. By eliminating the "marginalized variables" first, all of the
 * "additional connected variables" will remain in the system, but they will not depend on any of
 * the "marginalized variables"...which is what we want.
 *
 * This function uses CCOLAMD to find a good elimination order that eliminates all the "marginalized
 * variables" first.
 *
 * @param[in] marginalized_variables The variable UUIDs to be marginalized out
 * @param[in] graph                  A graph containing, at least, all constraints that involve at
 *                                   least one marginalized variable
 * @return The mapping from variable UUID to the computed elimination order
 */
UuidOrdering computeEliminationOrder(
  const std::vector<fuse_core::UUID> & marginalized_variables,
  const fuse_core::Graph & graph);

/**
 * @brief Generate a transaction that, when applied to the graph, will marginalize out the requested
 *        variables
 *
 * This computes a linear approximation of the marginal information on the non-marginalized
 * variables. The current variable values in the graph are used as the linearization points for the
 * linear approximation. Thus, marginalizing out a variable will introduce linearization errors as
 * the optimal values move away from the fixed linearization points.
 *
 * This version computes an efficient elimination order using computeEliminationOrder().
 *
 * @param[in] source                 The name of the sensor or motion model that generated this
 *                                   constraint
 * @param[in] marginalized_variables The set of variable UUIDs to marginalize out
 * @param[in] graph                  A graph containing the variables and constraints that are
 *                                   connected to at least one marginalized variable. The graph may
 *                                   also contain additional variables and constraints.
 * @return A transaction object containing the computed marginal constraints to be added, as well as
 *         the set of variables and constraints to be removed.
 */
fuse_core::Transaction marginalizeVariables(
  const std::string & source,
  const std::vector<fuse_core::UUID> & marginalized_variables,
  const fuse_core::Graph & graph);

/**
 * @brief Generate a transaction that, when applied to the graph, will marginalize out the requested
 *        variables
 *
 * This computes a linear approximation of the marginal information on the non-marginalized
 * variables. The current variable values in the graph are used as the linearization points for the
 * linear approximation. Thus, marginalizing out a variable will introduce linearization errors as
 * the optimal values move away from the fixed linearization points.
 *
 * This version allows the user to provide their own elimination order. The marginalized_variables
 * *must* occur before any other variables in that elimination order.
 *
 * @param[in] source                 The name of the sensor or motion model that generated this
 *                                   constraint
 * @param[in] marginalized_variables The set of variable UUIDs to marginalize out
 * @param[in] graph                  A graph containing the variables and constraints that are
 *                                   connected to at least one marginalized variable. The graph may
 *                                   also contain additional variables and constraints.
 * @param[in] elimination_order      An sequential ordering of at least the marginalized variables
 * @return A transaction object containing the computed marginal constraints to be added, as well as
 *         the set of variables and constraints to be removed.
 */
fuse_core::Transaction marginalizeVariables(
  const std::string & source,
  const std::vector<fuse_core::UUID> & marginalized_variables,
  const fuse_core::Graph & graph,
  const fuse_constraints::UuidOrdering & elimination_order);

namespace detail
{

/**
 * @brief Structure holding linearized Jacobian blocks
 *
 * The LinearTerm uses sequential variable indices instead of UUIDs
 */
struct LinearTerm
{
  std::vector<unsigned int> variables;
  std::vector<fuse_core::MatrixXd> A;
  fuse_core::VectorXd b;
};

/**
 * @brief Linearize the nonlinear constraint
 *
 * Variable UUIDs are converted into indices using the \p elimination_order. The variable
 * linearization points are extracted from the current variable values in the \p graph.
 *
 * @param[in] constraint        The constraint to linearize
 * @param[in] graph             A graph containing, at least, the variables involved in the
 *                              constraint
 * @param[in] elimination_order A mapping from variable UUID to elimination order
 * @return A LinearTerm consisting of Jacobian blocks associated with each involved variable in
 *         elimination order
 */
LinearTerm linearize(
  const fuse_core::Constraint & constraint,
  const fuse_core::Graph & graph,
  const UuidOrdering & elimination_order);

/**
 * @brief Marginalize out the lowest-ordered variable from the provided set of linear terms
 *
 * A linear marginal term is returned. This represents the information on the remaining variables
 * after marginalizing out the lowest-ordered variable.
 *
 * @param[in] linear_terms The set of LinearTerms that are connected to the lowest-ordered variable
 *                         index
 * @return A LinearTerm object containing the information on the remaining variables
 */
LinearTerm marginalizeNext(const std::vector<LinearTerm> & linear_terms);

/**
 * @brief Convert the provided linear term into a MarginalConstraint
 *
 * @param[in] source            The name of the sensor or motion model that generated this
 *                              constraint
 * @param[in] linear_term       The LinearTerm object to convert
 * @param[in] graph             The graph object containing the current variable values
 * @param[in] elimination_order The mapping from variable UUID to LinearTerm variable index
 * @return An equivalent MarginalConstraint object
 */
MarginalConstraint::SharedPtr createMarginalConstraint(
  const std::string & source,
  const LinearTerm & linear_term,
  const fuse_core::Graph & graph,
  const UuidOrdering & elimination_order);
}  // namespace detail

}  // namespace fuse_constraints

#endif  // FUSE_CONSTRAINTS__MARGINALIZE_VARIABLES_HPP_
