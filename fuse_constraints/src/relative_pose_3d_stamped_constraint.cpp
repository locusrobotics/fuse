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

#include <string>

#include <boost/serialization/export.hpp>
#include <fuse_constraints/normal_delta_pose_3d_cost_functor.hpp>
#include <fuse_constraints/relative_pose_3d_stamped_constraint.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace fuse_constraints
{

RelativePose3DStampedConstraint::RelativePose3DStampedConstraint(
  const std::string & source,
  const fuse_variables::Position3DStamped & position1,
  const fuse_variables::Orientation3DStamped & orientation1,
  const fuse_variables::Position3DStamped & position2,
  const fuse_variables::Orientation3DStamped & orientation2,
  const fuse_core::Vector7d & delta,
  const fuse_core::Matrix6d & covariance)
: fuse_core::Constraint(
    source,
    {position1.uuid(), orientation1.uuid(), position2.uuid(), orientation2.uuid()}),  // NOLINT
  delta_(delta),
  sqrt_information_(covariance.inverse().llt().matrixU())
{
}

RelativePose3DStampedConstraint::RelativePose3DStampedConstraint(
  const std::string & source,
  const fuse_variables::Position3DStamped & position1,
  const fuse_variables::Orientation3DStamped & orientation1,
  const fuse_variables::Position3DStamped & position2,
  const fuse_variables::Orientation3DStamped & orientation2,
  const fuse_core::Vector7d & delta,
  const fuse_core::MatrixXd & partial_covariance,
  const std::vector<size_t> & variable_indices)
: fuse_core::Constraint(
    source,
    {position1.uuid(), orientation1.uuid(), position2.uuid(), orientation2.uuid()}),  // NOLINT
  delta_(delta)
{
  // Compute the partial sqrt information matrix of the provided cov matrix
  fuse_core::MatrixXd partial_sqrt_information = partial_covariance.inverse().llt().matrixU();
  // std::cout << "partial_sqrt_information: \n" << partial_sqrt_information << std::endl;

  // Assemble a sqrt information matrix from the provided values, but in proper Variable order
  //
  // What are we doing here? The constraint equation is defined as: cost(x) = ||A * (x - b)||^2
  // If we are measuring a subset of dimensions, we only want to produce costs for the measured
  // dimensions. But the variable vectors will be full sized. We can make this all work out by
  // creating a non-square A, where each row computes a cost for one measured dimensions,
  // and the columns are in the order defined by the variable.
  
  // Fill in the rows of the sqrt information matrix corresponding to the measured dimensions 
  sqrt_information_ = fuse_core::MatrixXd::Zero(variable_indices.size(), 6);
  for (size_t i = 0; i < variable_indices.size(); ++i)
  {
    sqrt_information_.col(variable_indices[i]) = partial_sqrt_information.col(i);
  }
  // std::cout << "sqrt_information_ = " << "\n" << sqrt_information_ << std::endl;
  // std::cout << "mean_ = " << mean_.transpose() << std::endl;
}

fuse_core::MatrixXd RelativePose3DStampedConstraint::covariance() const
{
  if (sqrt_information_.rows() == 6)
  {
    return (sqrt_information_.transpose() * sqrt_information_).inverse();
  }
  // Otherwise we need to compute the pseudoinverse
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

void RelativePose3DStampedConstraint::print(std::ostream & stream) const
{
  stream << type() << "\n"
         << "  source: " << source() << "\n"
         << "  uuid: " << uuid() << "\n"
         << "  position1 variable: " << variables().at(0) << "\n"
         << "  orientation1 variable: " << variables().at(1) << "\n"
         << "  position2 variable: " << variables().at(2) << "\n"
         << "  orientation2 variable: " << variables().at(3) << "\n"
         << "  delta: " << delta().transpose() << "\n"
         << "  sqrt_info: " << sqrtInformation() << "\n";
}

ceres::CostFunction * RelativePose3DStampedConstraint::costFunction() const
{
  return new ceres::AutoDiffCostFunction<NormalDeltaPose3DCostFunctor, ceres::DYNAMIC, 3, 4, 3, 4>(
    new NormalDeltaPose3DCostFunctor(sqrt_information_, delta_),
    sqrt_information_.rows());
}

}  // namespace fuse_constraints

BOOST_CLASS_EXPORT_IMPLEMENT(fuse_constraints::RelativePose3DStampedConstraint);
PLUGINLIB_EXPORT_CLASS(fuse_constraints::RelativePose3DStampedConstraint, fuse_core::Constraint);
