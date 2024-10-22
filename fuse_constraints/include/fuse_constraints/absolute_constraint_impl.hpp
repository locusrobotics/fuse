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
#ifndef FUSE_CONSTRAINTS__ABSOLUTE_CONSTRAINT_IMPL_HPP_
#define FUSE_CONSTRAINTS__ABSOLUTE_CONSTRAINT_IMPL_HPP_

#include <ceres/normal_prior.h>
#include <Eigen/Dense>

#include <string>
#include <vector>

#include <fuse_constraints/normal_prior_orientation_2d.hpp>


namespace fuse_constraints
{

template<class Variable>
AbsoluteConstraint<Variable>::AbsoluteConstraint(
  const std::string & source,
  const Variable & variable,
  const fuse_core::VectorXd & mean,
  const fuse_core::MatrixXd & covariance)
: fuse_core::Constraint(source, {variable.uuid()}),    // NOLINT(whitespace/braces)
  mean_(mean),
  sqrt_information_(covariance.inverse().llt().matrixU())
{
  assert(mean.rows() == static_cast<int>(variable.size()));
  assert(covariance.rows() == static_cast<int>(variable.size()));
  assert(covariance.cols() == static_cast<int>(variable.size()));
}

template<class Variable>
AbsoluteConstraint<Variable>::AbsoluteConstraint(
  const std::string & source,
  const Variable & variable,
  const fuse_core::VectorXd & partial_mean,
  const fuse_core::MatrixXd & partial_covariance,
  const std::vector<size_t> & indices)
: fuse_core::Constraint(source, {variable.uuid()})    // NOLINT(whitespace/braces)
{
  assert(partial_mean.rows() == static_cast<int>(indices.size()));
  assert(partial_covariance.rows() == static_cast<int>(indices.size()));
  assert(partial_covariance.cols() == static_cast<int>(indices.size()));
  // Compute the sqrt information of the provided cov matrix
  fuse_core::MatrixXd partial_sqrt_information = partial_covariance.inverse().llt().matrixU();
  // Assemble a mean vector and sqrt information matrix from the provided values, but in proper
  // Variable order
  //
  // What are we doing here?
  // The constraint equation is defined as: cost(x) = ||A * (x - b)||^2
  // If we are measuring a subset of dimensions, we only want to produce costs for the measured
  // dimensions. But the variable vectors will be full sized. We can make this all work out by
  // creating a non-square A matrix, where each row computes a cost for one measured dimensions,
  // and the columns are in the order defined by the variable.
  mean_ = fuse_core::VectorXd::Zero(variable.size());
  sqrt_information_ = fuse_core::MatrixXd::Zero(indices.size(), variable.size());
  for (size_t i = 0; i < indices.size(); ++i) {
    mean_(indices[i]) = partial_mean(i);
    sqrt_information_.col(indices[i]) = partial_sqrt_information.col(i);
  }
}

template<class Variable>
fuse_core::MatrixXd AbsoluteConstraint<Variable>::covariance() const
{
  // We want to compute:
  //   cov = (sqrt_info' * sqrt_info)^-1
  // With some linear algebra, we can swap the transpose and the inverse.
  //   cov = (sqrt_info^-1) * (sqrt_info^-1)'
  // But sqrt_info _may_ not be square. So we need to compute the pseudoinverse instead.
  // Eigen doesn't have a pseudoinverse function (for probably very legitimate reasons).
  // So we set the right hand side to identity, then solve using one of Eigen's many decompositions.
  auto I = fuse_core::MatrixXd::Identity(sqrt_information_.rows(), sqrt_information_.cols());
  fuse_core::MatrixXd pinv = sqrt_information_.colPivHouseholderQr().solve(I);
  return pinv * pinv.transpose();
}

template<class Variable>
void AbsoluteConstraint<Variable>::print(std::ostream & stream) const
{
  stream << type() << "\n"
         << "  source: " << source() << "\n"
         << "  uuid: " << uuid() << "\n"
         << "  variable: " << variables().at(0) << "\n"
         << "  mean: " << mean().transpose() << "\n"
         << "  sqrt_info: " << sqrtInformation() << "\n";

  if (loss()) {
    stream << "  loss: ";
    loss()->print(stream);
  }
}

template<class Variable>
ceres::CostFunction * AbsoluteConstraint<Variable>::costFunction() const
{
  // Ceres ships with a "prior" cost function. Just use that here.
  return new ceres::NormalPrior(sqrt_information_, mean_);
}

// Specialization for Orientation2D
// We need to handle the 2*pi rollover for 2D orientations, so simple subtraction does not produce
// the correct cost
template<>
inline ceres::CostFunction * AbsoluteConstraint<
  fuse_variables::Orientation2DStamped
>::costFunction()
const
{
  return new NormalPriorOrientation2D(sqrt_information_(0, 0), mean_(0));
}

// Specialize the type() method to return the name that is registered with the plugins
template<>
inline std::string AbsoluteConstraint<fuse_variables::AccelerationAngular2DStamped>::type() const
{
  return "fuse_constraints::AbsoluteAccelerationAngular2DStampedConstraint";
}

template<>
inline std::string AbsoluteConstraint<fuse_variables::AccelerationLinear2DStamped>::type() const
{
  return "fuse_constraints::AbsoluteAccelerationLinear2DStampedConstraint";
}

template<>
inline std::string AbsoluteConstraint<fuse_variables::AccelerationAngular3DStamped>::type() const
{
  return "fuse_constraints::AbsoluteAccelerationAngular3DStampedConstraint";
}

template<>
inline std::string AbsoluteConstraint<fuse_variables::AccelerationLinear3DStamped>::type() const
{
  return "fuse_constraints::AbsoluteAccelerationLinear3DStampedConstraint";
}

template<>
inline std::string AbsoluteConstraint<fuse_variables::Orientation2DStamped>::type() const
{
  return "fuse_constraints::AbsoluteOrientation2DStampedConstraint";
}

template<>
inline std::string AbsoluteConstraint<fuse_variables::Position2DStamped>::type() const
{
  return "fuse_constraints::AbsolutePosition2DStampedConstraint";
}

template<>
inline std::string AbsoluteConstraint<fuse_variables::Position3DStamped>::type() const
{
  return "fuse_constraints::AbsolutePosition3DStampedConstraint";
}

template<>
inline std::string AbsoluteConstraint<fuse_variables::VelocityAngular2DStamped>::type() const
{
  return "fuse_constraints::AbsoluteVelocityAngular2DStampedConstraint";
}

template<>
inline std::string AbsoluteConstraint<fuse_variables::VelocityLinear2DStamped>::type() const
{
  return "fuse_constraints::AbsoluteVelocityLinear2DStampedConstraint";
}

template<>
inline std::string AbsoluteConstraint<fuse_variables::VelocityAngular3DStamped>::type() const
{
  return "fuse_constraints::AbsoluteVelocityAngular3DStampedConstraint";
}

template<>
inline std::string AbsoluteConstraint<fuse_variables::VelocityLinear3DStamped>::type() const
{
  return "fuse_constraints::AbsoluteVelocityLinear3DStampedConstraint";
}

}  // namespace fuse_constraints

#endif  // FUSE_CONSTRAINTS__ABSOLUTE_CONSTRAINT_IMPL_HPP_
