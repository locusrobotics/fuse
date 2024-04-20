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
#include <suitesparse/ccolamd.h>

#include <algorithm>
#include <iterator>
#include <numeric>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

// The ROS 2 ament linter incorrectly recognizes Eigen includes as C instead of C++
#include <boost/iterator/transform_iterator.hpp>
#include <boost/range/empty.hpp>
#include <Eigen/Core>  // NOLINT[build/include_order]
#include <Eigen/Dense>  // NOLINT[build/include_order]

#include <fuse_constraints/marginal_constraint.hpp>
#include <fuse_constraints/marginalize_variables.hpp>
#include <fuse_constraints/uuid_ordering.hpp>
#include <fuse_constraints/variable_constraints.hpp>
#include <fuse_core/ceres_macros.hpp>
#include <fuse_core/local_parameterization.hpp>
#include <fuse_core/manifold.hpp>
#include <fuse_core/uuid.hpp>

namespace fuse_constraints
{
UuidOrdering computeEliminationOrder(
  const std::vector<fuse_core::UUID> & marginalized_variables,
  const fuse_core::Graph & graph)
{
  // COLAMD wants a somewhat weird structure
  // Variables are numbered sequentially in some arbitrary order. We call this the "variable index"
  // order.
  // Similarly, the Constraints are numbered sequentially. We call this the "constraint index"
  // order.
  //
  // 'A' contains the constraint index for each connected constraint to a specific variable.
  //     The connected constraints are added to 'A' in variable index order.
  // 'p' contains the boundary indices for each variable in 'A'. So variable #1 spans entries
  //     from A[p[0]] to A[p[1] - 1], and variable #2 is the range A[p[1]] to A[p[2] - 1], etc.
  //
  // In order to compute A and p efficiently, we first construct a VariableConstraints object
  // We will construct sequential indices for the variables and constraints while we populate the
  // VariableConstraints object.
  // For orphan variables, i.e. variables with no constraints, p[c] == p[c+1], which still satisfies
  // CCOLAMD specs:
  // https://github.com/DrTimothyAldenDavis/SuiteSparse/blob/master/CCOLAMD/Source/ccolamd.c#L174
  auto variable_order = UuidOrdering();
  auto constraint_order = UuidOrdering();
  auto variable_constraints = VariableConstraints();
  for (const auto & variable_uuid : marginalized_variables) {
    // Get all connected constraints to this variable
    const auto constraints = graph.getConnectedConstraints(variable_uuid);

    // If the variable is orphan (it has no constraints), add it to the VariableConstraints object
    // without constraints New variable index is automatically generated
    if (boost::empty(constraints)) {
      variable_constraints.insert(variable_order[variable_uuid]);
    } else {
      // Add each constraint to the VariableConstraints object
      // New constraint and variable indices are automatically generated
      for (const auto & constraint : constraints) {
        unsigned int constraint_index = constraint_order[constraint.uuid()];
        for (const auto & constraint_variable_uuid : constraint.variables()) {
          variable_constraints.insert(constraint_index, variable_order[constraint_variable_uuid]);
        }
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
  for (unsigned int variable_index = 0u; variable_index < variable_order.size(); ++variable_index) {
    A_iter = variable_constraints.getConstraints(variable_index, A_iter);
    *p_iter = std::distance(A.begin(), A_iter);
    ++p_iter;
  }

  // Define the variable groups used by CCOLAMD. All of the marginalized variables should be group0,
  // all the rest should be group1.
  std::vector<int> variable_groups(variable_order.size(), 1);  // Default all variables to group1
  for (const auto & variable_uuid : marginalized_variables) {
    // Reassign the marginalized variables to group0
    variable_groups[variable_order.at(variable_uuid)] = 0;
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
  if (!success) {
    throw std::runtime_error("Failed to call CCOLAMD to generate the elimination order.");
  }

  // Extract the elimination order from CCOLAMD.
  // CCOLAMD returns the elimination order by updating the values stored in p with the variable
  // index Remember that p is larger than variable_order.size()
  auto elimination_order = UuidOrdering();
  for (size_t i = 0ul; i < variable_order.size(); ++i) {
    elimination_order.push_back(variable_order[p[i]]);
  }

  return elimination_order;
}

fuse_core::Transaction marginalizeVariables(
  const std::string & source,
  const std::vector<fuse_core::UUID> & marginalized_variables,
  const fuse_core::Graph & graph)
{
  return marginalizeVariables(
    source,
    marginalized_variables,
    graph,
    computeEliminationOrder(marginalized_variables, graph));
}

fuse_core::Transaction marginalizeVariables(
  const std::string & source,
  const std::vector<fuse_core::UUID> & marginalized_variables,
  const fuse_core::Graph & graph,
  const fuse_constraints::UuidOrdering & elimination_order)
{
  // TODO(swilliams) The method used to marginalize variables assumes that all variables are fully
  //                 constrained. However, with the introduction of "variables held constant", it is
  //                 possible to have a well-behaved system that is not fully-constrained. Ceres
  //                 handles this issue by removing constant variables from the problem before the
  //                 linearization and solve steps. A similar approach should be implemented here,
  //                 but that will require a major refactor.

  assert(
    std::all_of(
      marginalized_variables.begin(),
      marginalized_variables.end(),
      [&elimination_order, &marginalized_variables](const fuse_core::UUID & variable_uuid)
      {
        return elimination_order.exists(variable_uuid) &&
               elimination_order.at(variable_uuid) < marginalized_variables.size();
      }));  // NOLINT

  fuse_core::Transaction transaction;

  // Mark all of the marginalized variables for removal
  for (const auto & variable_uuid : marginalized_variables) {
    transaction.removeVariable(variable_uuid);
  }

  // Copy the elimination order so we can add additional variables if needed
  auto variable_order = elimination_order;

  // Linearize all involved constraints, and store them with the variable where they will be used
  auto used_constraints = std::unordered_set<fuse_core::UUID, fuse_core::uuid::hash>();
  std::vector<std::vector<detail::LinearTerm>> linear_terms(variable_order.size());
  for (size_t i = 0ul; i < marginalized_variables.size(); ++i) {
    const auto constraints = graph.getConnectedConstraints(variable_order[i]);
    for (const auto & constraint : constraints) {
      if (used_constraints.find(constraint.uuid()) == used_constraints.end()) {
        used_constraints.insert(constraint.uuid());
        // Ensure all connected variables are added to the ordering
        for (const auto & variable_uuid : constraint.variables()) {
          variable_order.push_back(variable_uuid);
        }
        // Add the linearized constraint to the lowest-ordered connected variable
        linear_terms[i].push_back(detail::linearize(constraint, graph, variable_order));
        // And mark the constraint for removal from the graph
        transaction.removeConstraint(constraint.uuid());
      }
    }
  }

  // Expand the linear_terms to include all the connected variables as well
  // During the marginalize process, marginal variables may be associated with these higher-ordered
  // variables
  linear_terms.resize(variable_order.size());

  // Use the linearized constraints to marginalize each variable in order
  // Place the resulting marginal in the linear constraint bucket associated with the lowest-ordered
  // remaining variable
  for (size_t i = 0ul; i < marginalized_variables.size(); ++i) {
    auto linear_marginal = detail::marginalizeNext(linear_terms[i]);
    if (!linear_marginal.variables.empty()) {
      auto lowest_ordered_variable = linear_marginal.variables.front();
      linear_terms[lowest_ordered_variable].push_back(std::move(linear_marginal));
    }
  }

  // Convert all remaining linear marginals into marginal constraints
  for (size_t i = marginalized_variables.size(); i < linear_terms.size(); ++i) {
    for (const auto & linear_term : linear_terms[i]) {
      auto marginal_constraint = detail::createMarginalConstraint(
        source, linear_term, graph,
        variable_order);
      transaction.addConstraint(std::move(marginal_constraint));
    }
  }

  return transaction;
}

namespace detail
{
// TODO(swilliams) There are more graph lookups of each Variable than needed. Refactor so that each
//                 Variable is only accessed once. This will mean storing the current variable value
//                 and local parameterization in the LinearTerm.

/**
 * In order for the linearize function to work correctly, it must perform the same operations as
 * Google Ceres-Solver. Unfortunately those functions are not callable from the public API, so we
 * must replicate them here. The following function is not identical to the Ceres-Solver code, but
 * it was referenced heavily during the creation of this function. As such, I'd like to acknowledge
 * the original authors.
 *
 * Ceres Solver - A fast non-linear least squares minimizer
 * http://ceres-solver.org/
 * Author: keir@google.com (Keir Mierle)
 *         sameeragarwal@google.com (Sameer Agarwal)
 *
 *  - https://github.com/ceres-solver/ceres-solver/blob/master/internal/ceres/residual_block.cc
 *  - https://github.com/ceres-solver/ceres-solver/blob/master/internal/ceres/corrector.cc
 */
LinearTerm linearize(
  const fuse_core::Constraint & constraint,
  const fuse_core::Graph & graph,
  const UuidOrdering & elimination_order)
{
  LinearTerm result;

  // Generate the cost function from the input constraint
  auto cost_function = constraint.costFunction();
  size_t row_count = cost_function->num_residuals();

  // Loop over the constraint's variables and do several things:
  // * Generate a vector of variable value pointers. This is needed for the Ceres API.
  // * Allocate a matrix for each jacobian block. We will have Ceres populate the matrix.
  // * Generate a vector of jacobian pointers. This is needed for the Ceres API.
  const auto & variable_uuids = constraint.variables();
  const size_t variable_count = variable_uuids.size();
  std::vector<const double *> variable_values;
  variable_values.reserve(variable_count);
  std::vector<double *> jacobians;
  jacobians.reserve(variable_count);
  result.variables.reserve(variable_count);
  result.A.reserve(variable_count);
  for (const auto & variable_uuid : variable_uuids) {
    const auto & variable = graph.getVariable(variable_uuid);
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
  for (const auto & A : result.A) {
    success = success && A.array().isFinite().all();
  }
  if (!success) {
    throw std::runtime_error(
            "Error in evaluating the cost function. There are two possible reasons. "
            "Either the CostFunction did not evaluate and fill all residual and jacobians "
            "that were requested or there was a non-finite value (nan/infinite) generated "
            "during the jacobian computation.");
  }

  // Update the Jacobians with the local parameterizations. This potentially changes the size of the
  // Jacobian block. The classic example is a quaternion parameter, which has 4 components but only
  // 3 degrees of freedom. The Jacobian will be transformed from 4 columns to 3 columns after the
  // local parameterization is applied. We also check for variables that have been marked as
  // constants. Since these variables cannot change value, their derivatives/Jacobians should be
  // zero.
  for (size_t index = 0ul; index < variable_count; ++index) {
    const auto & variable_uuid = variable_uuids[index];
    const auto & variable = graph.getVariable(variable_uuid);
#if !CERES_SUPPORTS_MANIFOLDS
    auto local_parameterization = variable.localParameterization();
    auto & jacobian = result.A[index];
    if (variable.holdConstant()) {
      if (local_parameterization) {
        jacobian.resize(Eigen::NoChange, local_parameterization->LocalSize());
      }
      jacobian.setZero();
    } else if (local_parameterization) {
      fuse_core::MatrixXd J(local_parameterization->GlobalSize(),
        local_parameterization->LocalSize());
      local_parameterization->ComputeJacobian(variable_values[index], J.data());
      jacobian *= J;
    }
    if (local_parameterization) {
      delete local_parameterization;
    }
#else
    auto manifold = variable.manifold();
    auto & jacobian = result.A[index];
    if (variable.holdConstant()) {
      if (manifold) {
        jacobian.resize(Eigen::NoChange, manifold->TangentSize());
      }
      jacobian.setZero();
    } else if (manifold) {
      fuse_core::MatrixXd J(manifold->AmbientSize(), manifold->TangentSize());
      manifold->PlusJacobian(variable_values[index], J.data());
      jacobian *= J;
    }
    if (manifold) {
      delete manifold;
    }
#endif
  }

  // Correct A and b for the effects of the loss function
  auto loss_function = constraint.lossFunction();
  if (loss_function) {
    double squared_norm = result.b.squaredNorm();
    double rho[3];
    loss_function->Evaluate(squared_norm, rho);
    if (fuse_core::Loss::Ownership == ceres::Ownership::TAKE_OWNERSHIP) {
      delete loss_function;
    }
    double sqrt_rho1 = std::sqrt(rho[1]);
    double alpha = 0.0;
    if ((squared_norm > 0.0) && (rho[2] > 0.0)) {
      const double D = 1.0 + 2.0 * squared_norm * rho[2] / rho[1];
      alpha = 1.0 - std::sqrt(D);
    }

    // Correct the Jacobians
    for (auto & jacobian : result.A) {
      if (alpha == 0.0) {
        jacobian *= sqrt_rho1;
      } else {
        // TODO(swilliams) This may be inefficient, at least according to notes in the Ceres
        //                 codebase.
        jacobian = sqrt_rho1 *
          (jacobian - (alpha / squared_norm) * result.b * (result.b.transpose() * jacobian));
      }
    }

    // Correct the residuals
    result.b *= sqrt_rho1 / (1 - alpha);
  }

  return result;
}

LinearTerm marginalizeNext(const std::vector<LinearTerm> & linear_terms)
{
  if (linear_terms.empty()) {
    return {};
  }

  // We need to create a dense matrix from all of the provided linear terms, and that matrix must
  // order the variables in the proper elimination order. The LinearTerms have the elimination order
  // baked into the variable indices, but since not all variables are necessarily present, we need
  // to remove any gaps from the variable indices. We use vector operations instead of a std::set
  // because the number of variables is assumed to be small. You need 1000s of variables before the
  // std::set outperforms the std::vector.
  auto dense_to_index = std::vector<unsigned int>();
  for (const auto & linear_term : linear_terms) {
    std::copy(
      linear_term.variables.begin(), linear_term.variables.end(),
      std::back_inserter(dense_to_index));
  }
  std::sort(dense_to_index.begin(), dense_to_index.end());
  dense_to_index.erase(
    std::unique(dense_to_index.begin(), dense_to_index.end()),
    dense_to_index.end());

  // Construct the inverse mapping
  auto index_to_dense = std::vector<unsigned int>(dense_to_index.back() + 1, 0);
  for (size_t dense = 0ul; dense < dense_to_index.size(); ++dense) {
    index_to_dense[dense_to_index[dense]] = dense;
  }

  // Compute the row offsets
  auto row_offsets = std::vector<unsigned int>();
  row_offsets.reserve(linear_terms.size() + 1ul);
  row_offsets.push_back(0u);
  for (const auto & linear_term : linear_terms) {
    row_offsets.push_back(row_offsets.back() + linear_term.b.rows());
  }

  // Compute the column offsets
  auto index_to_cols = std::vector<unsigned int>(dense_to_index.back() + 1u, 0u);
  for (const auto & linear_term : linear_terms) {
    for (size_t i = 0ul; i < linear_term.variables.size(); ++i) {
      auto index = linear_term.variables[i];
      index_to_cols[index] = linear_term.A[i].cols();
    }
  }

  auto column_offsets = std::vector<unsigned int>();
  column_offsets.reserve(dense_to_index.size() + 1ul);
  column_offsets.push_back(0u);
  for (size_t dense = 0; dense < dense_to_index.size(); ++dense) {
    column_offsets.push_back(column_offsets.back() + index_to_cols[dense_to_index[dense]]);
  }

  // Construct the Ab matrix
  fuse_core::MatrixXd Ab =
    fuse_core::MatrixXd::Zero(row_offsets.back(), column_offsets.back() + 1u);
  for (size_t term_index = 0ul; term_index < linear_terms.size(); ++term_index) {
    const auto & linear_term = linear_terms[term_index];
    auto row_offset = row_offsets[term_index];
    for (size_t i = 0ul; i < linear_term.variables.size(); ++i) {
      const auto & A = linear_term.A[i];
      auto dense = index_to_dense[linear_term.variables[i]];
      auto column_offset = column_offsets[dense];
      for (int row = 0; row < A.rows(); ++row) {
        for (int col = 0; col < A.cols(); ++col) {
          Ab(row_offset + row, column_offset + col) = A(row, col);
        }
      }
    }
    const auto & b = linear_term.b;
    int column_offset = column_offsets.back();
    for (int row = 0; row < b.rows(); ++row) {
      Ab(row_offset + row, column_offset) = b(row);
    }
  }

  // Compute the QR decomposition
  // I really want to do this "in place" instead of making a copy into the Eigen QR object and a
  // second copy back out, but Eigen does not make it easy.
  // https://eigen.tuxfamily.org/dox/HouseholderQR_8h_source.html Line 379
  // HouseholderQR<MatrixType>::computeInPlace()
  {
    using MatrixType = fuse_core::MatrixXd;
    using HCoeffsType = Eigen::internal::plain_diag_type<MatrixType>::type;
    using RowVectorType = Eigen::internal::plain_row_type<MatrixType>::type;
    auto rows = Ab.rows();
    auto cols = Ab.cols();
    auto size = std::min(rows, cols);
    auto hCoeffs = HCoeffsType(size);
    auto temp = RowVectorType(cols);
    Eigen::internal::householder_qr_inplace_blocked<MatrixType, HCoeffsType>::run(
      Ab, hCoeffs, 48,
      temp.data());
    Ab.triangularView<Eigen::StrictlyLower>().setZero();  // Zero out the below-diagonal elements
  }

  // Extract the marginal term from R (now stored in Ab)
  // The first row block is the conditional term for the marginalized variable: P(x | y, z, ...)
  // The remaining rows are the marginal on the remaining variables: P(y, z, ...)
  auto min_row = column_offsets[1];
  // However, depending on the input, not all rows may be usable.
  auto max_row = std::min(Ab.rows(), Ab.cols() - 1);  // -1 for the included b vector
  auto marginal_rows = max_row - min_row;
  auto marginal_term = LinearTerm();
  if (marginal_rows > 0) {
    auto variable_count = dense_to_index.size() - 1;
    marginal_term.variables.reserve(variable_count);
    marginal_term.A.reserve(variable_count);
    // Skipping the marginalized variable
    for (size_t dense = 1ul; dense < dense_to_index.size(); ++dense) {
      auto index = dense_to_index[dense];
      marginal_term.variables.push_back(index);
      fuse_core::MatrixXd A = fuse_core::MatrixXd::Zero(marginal_rows, index_to_cols[index]);
      auto column_offset = column_offsets[dense];
      for (int row = 0; row < A.rows(); ++row) {
        for (int col = 0; col < A.cols(); ++col) {
          A(row, col) = Ab(min_row + row, column_offset + col);
        }
      }
      marginal_term.A.push_back(std::move(A));
    }
    marginal_term.b = fuse_core::VectorXd::Zero(marginal_rows);
    auto column_offset = column_offsets.back();
    for (int row = 0; row < marginal_term.b.rows(); ++row) {
      marginal_term.b(row) = Ab(min_row + row, column_offset);
    }
  }
  return marginal_term;
}

MarginalConstraint::SharedPtr createMarginalConstraint(
  const std::string & source,
  const LinearTerm & linear_term,
  const fuse_core::Graph & graph,
  const UuidOrdering & elimination_order)
{
  auto index_to_variable =
    [&graph, &elimination_order](const unsigned int index) -> const fuse_core::Variable &
    {
      return graph.getVariable(elimination_order.at(index));
    };

  return MarginalConstraint::make_shared(
    source,
    boost::make_transform_iterator(linear_term.variables.begin(), index_to_variable),
    boost::make_transform_iterator(linear_term.variables.end(), index_to_variable),
    linear_term.A.begin(),
    linear_term.A.end(),
    linear_term.b);
}

}  // namespace detail

}  // namespace fuse_constraints
