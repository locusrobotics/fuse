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
#include <ceres/autodiff_cost_function.h>
#include <Eigen/Dense>

#include <string>
#include <vector>

#include <boost/serialization/export.hpp>
#include <fuse_constraints/absolute_pose_2d_stamped_constraint.hpp>
#include <fuse_constraints/normal_prior_pose_2d.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace fuse_constraints
{

AbsolutePose2DStampedConstraint::AbsolutePose2DStampedConstraint(
  const std::string & source,
  const fuse_variables::Position2DStamped & position,
  const fuse_variables::Orientation2DStamped & orientation,
  const fuse_core::VectorXd & partial_mean,
  const fuse_core::MatrixXd & partial_covariance,
  const std::vector<size_t> & linear_indices,
  const std::vector<size_t> & angular_indices)
: fuse_core::Constraint(source, {position.uuid(), orientation.uuid()})  // NOLINT(whitespace/braces)
{
  size_t total_variable_size = position.size() + orientation.size();
  size_t total_indices = linear_indices.size() + angular_indices.size();

  assert(partial_mean.rows() == static_cast<int>(total_indices));
  assert(partial_covariance.rows() == static_cast<int>(total_indices));
  assert(partial_covariance.cols() == static_cast<int>(total_indices));

  // Compute the sqrt information of the provided cov matrix
  fuse_core::MatrixXd partial_sqrt_information = partial_covariance.inverse().llt().matrixU();

  // Assemble a mean vector and sqrt information matrix from the provided values, but in proper
  // Variable order
  //
  // What are we doing here? The constraint equation is defined as: cost(x) = ||A * (x - b)||^2
  // If we are measuring a subset of dimensions, we only want to produce costs for the measured
  // dimensions. But the variable vectors will be full sized. We can make this all work out by
  // creating a non-square A matrix, where each row computes a cost for one measured dimensions,
  // and the columns are in the order defined by the variable.
  mean_ = fuse_core::VectorXd::Zero(total_variable_size);
  sqrt_information_ = fuse_core::MatrixXd::Zero(total_indices, total_variable_size);
  for (size_t i = 0; i < linear_indices.size(); ++i) {
    mean_(linear_indices[i]) = partial_mean(i);
    sqrt_information_.col(linear_indices[i]) = partial_sqrt_information.col(i);
  }

  for (size_t i = linear_indices.size(); i < total_indices; ++i) {
    size_t final_index = position.size() + angular_indices[i - linear_indices.size()];
    mean_(final_index) = partial_mean(i);
    sqrt_information_.col(final_index) = partial_sqrt_information.col(i);
  }
}

fuse_core::Matrix3d AbsolutePose2DStampedConstraint::covariance() const
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

void AbsolutePose2DStampedConstraint::print(std::ostream & stream) const
{
  stream << type() << "\n"
         << "  source: " << source() << "\n"
         << "  uuid: " << uuid() << "\n"
         << "  position variable: " << variables().at(0) << "\n"
         << "  orientation variable: " << variables().at(1) << "\n"
         << "  mean: " << mean().transpose() << "\n"
         << "  sqrt_info: " << sqrtInformation() << "\n";

  if (loss()) {
    stream << "  loss: ";
    loss()->print(stream);
  }
}

ceres::CostFunction * AbsolutePose2DStampedConstraint::costFunction() const
{
  return new NormalPriorPose2D(sqrt_information_, mean_);
}

}  // namespace fuse_constraints

BOOST_CLASS_EXPORT_IMPLEMENT(fuse_constraints::AbsolutePose2DStampedConstraint);
PLUGINLIB_EXPORT_CLASS(fuse_constraints::AbsolutePose2DStampedConstraint, fuse_core::Constraint);
