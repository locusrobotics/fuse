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
#ifndef FUSE_CONSTRAINTS_ABSOLUTE_CONSTRAINT_IMPL_H
#define FUSE_CONSTRAINTS_ABSOLUTE_CONSTRAINT_IMPL_H

#include <fuse_constraints/normal_prior_orientation_2d.h>

#include <ceres/normal_prior.h>
#include <Eigen/Dense>

#include <vector>


namespace fuse_constraints
{

template<class Variable>
AbsoluteConstraint<Variable>::AbsoluteConstraint(
  const Variable& variable,
  const Eigen::VectorXd& mean,
  const Eigen::MatrixXd& covariance) :
    fuse_core::Constraint{variable.uuid()},
    mean_(mean),
    sqrt_information_(covariance.inverse().llt().matrixU())
{
  assert(mean.rows() == static_cast<int>(variable.size()));
  assert(covariance.rows() == static_cast<int>(variable.size()));
  assert(covariance.cols() == static_cast<int>(variable.size()));
}

template<class Variable>
AbsoluteConstraint<Variable>::AbsoluteConstraint(
  const Variable& variable,
  const Eigen::VectorXd& partial_mean,
  const Eigen::MatrixXd& partial_covariance,
  const std::vector<size_t>& indices) :
    fuse_core::Constraint{variable.uuid()}
{
  assert(partial_mean.rows() == static_cast<int>(indices.size()));
  assert(partial_covariance.rows() == static_cast<int>(indices.size()));
  assert(partial_covariance.cols() == static_cast<int>(indices.size()));
  // Compute the sqrt information of the provided cov matrix
  Eigen::MatrixXd partial_sqrt_information = partial_covariance.inverse().llt().matrixU();
  // Assemble a mean vector and sqrt information matrix from the provided values, but in proper Variable order
  // What are we doing here?
  // The constraint equation is defined as: cost(x) = ||A * (x - b)||^2
  // If we are measuring a subset of dimensions, we only want to produce costs for the measured dimensions.
  // But the variable vectors will be full sized. We can make this all work out by creating a non-square A
  // matrix, where each row computes a cost for one measured dimensions, and the columns are in the order
  // defined by the variable.
  mean_ = Eigen::VectorXd::Zero(variable.size());
  sqrt_information_ = Eigen::MatrixXd::Zero(indices.size(), variable.size());
  for (size_t i = 0; i < indices.size(); ++i)
  {
    mean_(indices[i]) = partial_mean(i);
    sqrt_information_.col(indices[i]) = partial_sqrt_information.col(i);
  }
}

template<class Variable>
Eigen::MatrixXd AbsoluteConstraint<Variable>::covariance() const
{
  // We want to compute:
  // cov = (sqrt_info' * sqrt_info)^-1
  // With some linear algebra, we can swap the transpose and the inverse.
  // cov = (sqrt_info^-1) * (sqrt_info^-1)'
  // But sqrt_info _may_ not be square. So we need to compute the pseudoinverse instead.
  // Eigen doesn't have a pseudoinverse function (for probably very legitimate reasons).
  // So we set the right hand side to identity, then solve using one of Eigen's many decompositions.
  auto I = Eigen::MatrixXd::Identity(sqrt_information_.rows(), sqrt_information_.cols());
  Eigen::MatrixXd pinv = sqrt_information_.colPivHouseholderQr().solve(I);
  return pinv * pinv.transpose();
}

template<class Variable>
void AbsoluteConstraint<Variable>::print(std::ostream& stream) const
{
  stream << type() << "\n"
         << "  uuid: " << uuid() << "\n"
         << "  variable: " << variables_.at(0) << "\n"
         << "  mean: " << mean_.transpose() << "\n"
         << "  sqrt_info: " << sqrtInformation() << "\n";
}

template<class Variable>
fuse_core::Constraint::UniquePtr AbsoluteConstraint<Variable>::clone() const
{
  return AbsoluteConstraint<Variable>::make_unique(*this);
}

template<class Variable>
ceres::CostFunction* AbsoluteConstraint<Variable>::costFunction() const
{
  // Ceres ships with a "prior" cost function. Just use that here.
  return new ceres::NormalPrior(sqrt_information_, mean_);
}

// Specialization for Orientation2D
// We need to handle the 2*pi rollover for 2D orientations, so simple subtraction does not produce the correct cost
template<>
inline ceres::CostFunction* AbsoluteConstraint<fuse_variables::Orientation2DStamped>::costFunction() const
{
  // Ceres ships with a "prior" cost function. Just use that here.
  return new NormalPriorOrientation2D(sqrt_information_(0, 0), mean_(0));
}

}  // namespace fuse_constraints

#endif  // FUSE_CONSTRAINTS_ABSOLUTE_CONSTRAINT_IMPL_H
