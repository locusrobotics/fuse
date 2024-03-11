/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, Giacomo Franchini
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
#include <Eigen/Dense>

#include <string>

#include <boost/serialization/export.hpp>

#include <fuse_constraints/absolute_pose_3d_stamped_euler_constraint.hpp>
#include <fuse_constraints/normal_prior_pose_3d_euler.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace fuse_constraints
{

AbsolutePose3DStampedEulerConstraint::AbsolutePose3DStampedEulerConstraint(
  const std::string & source,
  const fuse_variables::Position3DStamped & position,
  const fuse_variables::Orientation3DStamped & orientation,
  const fuse_core::Vector6d & mean,
  const fuse_core::Matrix6d & covariance)
: fuse_core::Constraint(source, {position.uuid(), orientation.uuid()}),  // NOLINT
  mean_(mean),
  sqrt_information_(covariance.inverse().llt().matrixU())
{ 
}

AbsolutePose3DStampedEulerConstraint::AbsolutePose3DStampedEulerConstraint(
  const std::string & source,
  const fuse_variables::Position3DStamped & position,
  const fuse_variables::Orientation3DStamped & orientation,
  const fuse_core::Vector6d & partial_mean,
  const fuse_core::MatrixXd & partial_covariance,
  const std::vector<size_t> & variable_indices)
: fuse_core::Constraint(source, {position.uuid(), orientation.uuid()}),  // NOLINT
  mean_(partial_mean)
{ 
  // Compute the partial sqrt information matrix of the provided cov matrix
  fuse_core::MatrixXd partial_sqrt_information = partial_covariance.inverse().llt().matrixU();

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
}

fuse_core::Matrix6d AbsolutePose3DStampedEulerConstraint::covariance() const
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

void AbsolutePose3DStampedEulerConstraint::print(std::ostream & stream) const
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

ceres::CostFunction * AbsolutePose3DStampedEulerConstraint::costFunction() const
{
  return new NormalPriorPose3DEuler(sqrt_information_, mean_);
  
  // Here we return a cost function that computes the analytic derivatives/jacobians, but we could
  // use automatic differentiation as follows:

  // return new ceres::AutoDiffCostFunction<NormalPriorPose3DEulerCostFunctor, ceres::DYNAMIC, 3, 4>(
    // new NormalPriorPose3DEulerCostFunctor(sqrt_information_, mean_), 
    // sqrt_information_.rows());
    
  // And including the followings:
  // #include <ceres/autodiff_cost_function.h>
  // #include <fuse_constraints/normal_prior_pose_3d_euler_cost_functor.hpp>
}

}  // namespace fuse_constraints

BOOST_CLASS_EXPORT_IMPLEMENT(fuse_constraints::AbsolutePose3DStampedEulerConstraint);
PLUGINLIB_EXPORT_CLASS(fuse_constraints::AbsolutePose3DStampedEulerConstraint, fuse_core::Constraint);
