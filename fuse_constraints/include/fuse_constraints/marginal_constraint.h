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
#ifndef FUSE_CONSTRAINTS_MARGINAL_CONSTRAINT_H
#define FUSE_CONSTRAINTS_MARGINAL_CONSTRAINT_H

#include <fuse_core/constraint.h>
#include <fuse_core/eigen.h>
#include <fuse_core/local_parameterization.h>
#include <fuse_core/macros.h>
#include <fuse_core/variable.h>

#include <boost/iterator/transform_iterator.hpp>
#include <boost/iterator/zip_iterator.hpp>
#include <boost/tuple/tuple.hpp>
#include <ceres/cost_function.h>

#include <algorithm>
#include <cassert>
#include <ostream>
#include <vector>


namespace fuse_constraints
{

/**
 * @brief A constraint that represents remaining marginal information on a set of variables
 *
 * The marginal constraint cost function is of the form:
 *   cost = A1 * (x1 - x1_bar) + A2 * (x2 - x2_bar) + ... + b
 * where x_bar is the linearization point of the variable taken from the variable value at the time of construction,
 * and the minus operator in implemented in the variable's local parameterization.
 */
class MarginalConstraint : public fuse_core::Constraint
{
public:
  SMART_PTR_DEFINITIONS(MarginalConstraint);

  /**
   * @brief Create a linear/marginal constraint
   *
   * The variable iterators and matrix iterators must be the same size. Further, all A matrices and the b vector must
   * have the same number of rows, and the number of columns of each A matrix must match the \p localSize() of its
   * associated variable.
   *
   * @param[in] first_variable Iterator pointing to the first involved variable for this constraint
   * @param[in] last_variable  Iterator pointing to one past the last involved variable for this constraint
   * @param[in] first_A        Iterator pointing to the first A matrix, associated with the first variable
   * @param[in] last_A         Iterator pointing to one past the last A matrix
   * @param[in] b              The b vector of the marginal cost (of the form A*(x - x_bar) + b)
   */
  template<typename VariableIterator, typename MatrixIterator>
  MarginalConstraint(
    VariableIterator first_variable,
    VariableIterator last_variable,
    MatrixIterator first_A,
    MatrixIterator last_A,
    const fuse_core::VectorXd& b);

  /**
   * @brief Destructor
   */
  virtual ~MarginalConstraint() = default;

  /**
   * @brief Read-only access to the A matrices of the marginal constraint
   */
  const std::vector<fuse_core::MatrixXd>& A() const { return A_; }

  /**
   * @brief Read-only access to the b vector of the marginal constraint
   */
  const fuse_core::VectorXd& b() const { return b_; }

  /**
   * @brief Read-only access to the variable linearization points, x_bar
   */
  const std::vector<fuse_core::VectorXd>& x_bar() const { return x_bar_; }

  /**
   * @brief Read-only access to the variable local parameterizations
   */
  const std::vector<fuse_core::LocalParameterization::SharedPtr>& localParameterizations() const
  {
    return local_parameterizations_;
  }

  /**
   * @brief Print a human-readable description of the constraint to the provided stream.
   *
   * @param[out] stream The stream to write to. Defaults to stdout.
   */
  void print(std::ostream& stream = std::cout) const override;

  /**
   * @brief Perform a deep copy of the constraint and return a unique pointer to the copy
   *
   * Unique pointers can be implicitly upgraded to shared pointers if needed.
   *
   * @return A unique pointer to a new instance of the most-derived constraint
   */
  fuse_core::Constraint::UniquePtr clone() const override;

  /**
   * @brief Construct an instance of this constraint's cost function
   *
   * The function caller will own the new cost function instance. It is the responsibility of the caller to delete
   * the cost function object when it is no longer needed. If the pointer is provided to a Ceres::Problem object, the
   * Ceres::Problem object will take ownership of the pointer and delete it during destruction.
   *
   * @return A base pointer to an instance of a derived CostFunction.
   */
  ceres::CostFunction* costFunction() const override;

protected:
  std::vector<fuse_core::MatrixXd> A_;  //!< The A matrices of the marginal constraint
  fuse_core::VectorXd b_;  //!< The b vector of the marginal constraint
  std::vector<fuse_core::LocalParameterization::SharedPtr> local_parameterizations_;  //!< The local parameterizations
  std::vector<fuse_core::VectorXd> x_bar_;  //!< The linearization point of each involved variable
};

namespace detail
{

/**
 * @brief Return the UUID of the provided variable
 */
inline const fuse_core::UUID getUuid(const fuse_core::Variable& variable)
{
  return variable.uuid();
}

/**
 * @brief Return the current value of the provided variable
 */
inline const fuse_core::VectorXd getCurrentValue(const fuse_core::Variable& variable)
{
  return Eigen::Map<const fuse_core::VectorXd>(variable.data(), variable.size());
}

/**
 * @brief Return the local parameterization of the provided variable
 */
inline fuse_core::LocalParameterization::SharedPtr const getLocalParameterization(const fuse_core::Variable& variable)
{
  return fuse_core::LocalParameterization::SharedPtr(variable.localParameterization());
}

}  // namespace detail

template<typename VariableIterator, typename MatrixIterator>
MarginalConstraint::MarginalConstraint(
  VariableIterator first_variable,
  VariableIterator last_variable,
  MatrixIterator first_A,
  MatrixIterator last_A,
  const fuse_core::VectorXd& b) :
    Constraint(boost::make_transform_iterator(first_variable, &detail::getUuid),
               boost::make_transform_iterator(last_variable, &detail::getUuid)),
    A_(first_A, last_A),
    b_(b),
    local_parameterizations_(boost::make_transform_iterator(first_variable, &detail::getLocalParameterization),
                             boost::make_transform_iterator(last_variable, &detail::getLocalParameterization)),
    x_bar_(boost::make_transform_iterator(first_variable, &detail::getCurrentValue),
           boost::make_transform_iterator(last_variable, &detail::getCurrentValue))
{
  assert(!A_.empty());
  assert(A_.size() == x_bar_.size());
  assert(A_.size() == local_parameterizations_.size());
  assert(b_.rows() > 0);
  assert(std::all_of(A_.begin(), A_.end(), [this](const auto& A){ return A.rows() == this->b_.rows(); }));  // NOLINT
  assert(std::all_of(boost::make_zip_iterator(boost::make_tuple(A_.begin(), first_variable)),
                     boost::make_zip_iterator(boost::make_tuple(A_.end(), last_variable)),
                     [](const boost::tuple<const fuse_core::MatrixXd&, const fuse_core::Variable&>& tuple)  // NOLINT
                     {
                       return static_cast<size_t>(tuple.get<0>().cols()) == tuple.get<1>().localSize();
                     }));  // NOLINT
}

}  // namespace fuse_constraints

#endif  // FUSE_CONSTRAINTS_MARGINAL_CONSTRAINT_H
