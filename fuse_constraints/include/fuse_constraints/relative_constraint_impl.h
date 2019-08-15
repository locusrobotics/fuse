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
#ifndef FUSE_CONSTRAINTS_RELATIVE_CONSTRAINT_IMPL_H
#define FUSE_CONSTRAINTS_RELATIVE_CONSTRAINT_IMPL_H

#include <fuse_constraints/normal_delta.h>

#include <Eigen/Dense>

#include <vector>


namespace fuse_constraints
{

template<class Variable>
RelativeConstraint<Variable>::RelativeConstraint(
  const Variable& variable1,
  const Variable& variable2,
  const fuse_core::VectorXd& delta,
  const fuse_core::MatrixXd& covariance) :
    fuse_core::Constraint{variable1.uuid(), variable2.uuid()},
    delta_(delta),
    sqrt_information_(covariance.inverse().llt().matrixU())
{
  assert(variable1.size() == variable2.size());
  assert(delta.rows() == static_cast<int>(variable1.size()));
  assert(covariance.rows() == static_cast<int>(variable1.size()));
  assert(covariance.cols() == static_cast<int>(variable1.size()));
}

template<class Variable>
RelativeConstraint<Variable>::RelativeConstraint(
  const Variable& variable1,
  const Variable& variable2,
  const fuse_core::VectorXd& partial_delta,
  const fuse_core::MatrixXd& partial_covariance,
  const std::vector<size_t>& indices) :
    fuse_core::Constraint{variable1.uuid(), variable2.uuid()}
{
  assert(variable1.size() == variable2.size());
  assert(partial_delta.rows() == static_cast<int>(indices.size()));
  assert(partial_covariance.rows() == static_cast<int>(indices.size()));
  assert(partial_covariance.cols() == static_cast<int>(indices.size()));
  // Compute the sqrt information of the provided cov matrix
  fuse_core::MatrixXd partial_sqrt_information = partial_covariance.inverse().llt().matrixU();
  // Assemble a mean vector and sqrt information matrix from the provided values, but in proper Variable order
  // What are we doing here?
  // The constraint equation is defined as: cost(x) = ||A * (x1 - x0 - b)||^2
  // If we are measuring a subset of dimensions, we only want to produce costs for the measured dimensions.
  // But the variable vectors will be full sized. We can make this all work out by creating a non-square A
  // matrix, where each row computes a cost for one measured dimensions, and the columns are in the order
  // defined by the variable.
  delta_ = fuse_core::VectorXd::Zero(variable1.size());
  sqrt_information_ = fuse_core::MatrixXd::Zero(indices.size(), variable1.size());
  for (size_t i = 0; i < indices.size(); ++i)
  {
    delta_(indices[i]) = partial_delta(i);
    sqrt_information_.col(indices[i]) = partial_sqrt_information.col(i);
  }
}

template<class Variable>
fuse_core::MatrixXd RelativeConstraint<Variable>::covariance() const
{
  // We want to compute:
  // cov = (sqrt_info' * sqrt_info)^-1
  // With some linear algebra, we can swap the transpose and the inverse.
  // cov = (sqrt_info^-1) * (sqrt_info^-1)'
  // But sqrt_info _may_ not be square. So we need to compute the pseudoinverse instead.
  // Eigen doesn't have a pseudoinverse function (for probably very legitimate reasons).
  // So we set the right hand side to identity, then solve using one of Eigen's many decompositions.
  auto I = fuse_core::MatrixXd::Identity(sqrt_information_.rows(), sqrt_information_.cols());
  fuse_core::MatrixXd pinv = sqrt_information_.colPivHouseholderQr().solve(I);
  return pinv * pinv.transpose();
}

template<class Variable>
void RelativeConstraint<Variable>::print(std::ostream& stream) const
{
  stream << type() << "\n"
         << "  uuid: " << uuid() << "\n"
         << "  variable1: " << variables().at(0) << "\n"
         << "  variable2: " << variables().at(1) << "\n"
         << "  delta: " << delta().transpose() << "\n"
         << "  sqrt_info: " << sqrtInformation() << "\n";
}

template<class Variable>
ceres::CostFunction* RelativeConstraint<Variable>::costFunction() const
{
  // Create a Gaussian/Normal Delta constraint
  return new fuse_constraints::NormalDelta(sqrt_information_, delta_);
}

}  // namespace fuse_constraints

#endif  // FUSE_CONSTRAINTS_RELATIVE_CONSTRAINT_IMPL_H
