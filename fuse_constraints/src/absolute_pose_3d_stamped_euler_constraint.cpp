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

#include <algorithm>
#include <string>

#include <boost/serialization/export.hpp>
#include <fuse_constraints/absolute_pose_3d_stamped_euler_constraint.hpp>
// #include <fuse_constraints/normal_prior_pose_3d_euler_cost_functor.hpp>
#include <fuse_constraints/normal_prior_pose_3d.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace fuse_constraints
{

AbsolutePose3DStampedEulerConstraint::AbsolutePose3DStampedEulerConstraint(
  const std::string & source,
  const fuse_variables::Position3DStamped & position,
  const fuse_variables::Orientation3DStamped & orientation,
  const fuse_core::Vector7d & mean,
  const fuse_core::Matrix6d & covariance,
  const std::vector<size_t> & linear_indices,
  const std::vector<size_t> & angular_indices)
: fuse_core::Constraint(source, {position.uuid(), orientation.uuid()})  // NOLINT
{
  axes_.resize(angular_indices.size());
  std::transform(
    angular_indices.begin(), angular_indices.end(), axes_.begin(),
    [](const size_t index) 
    {return static_cast<Euler>(index + 1UL);});
  
  // merge indices
  std::vector<size_t> total_indices;
  total_indices.reserve(linear_indices.size() + angular_indices.size());
  std::copy(linear_indices.begin(), linear_indices.end(), std::back_inserter(total_indices));
  std::copy(angular_indices.begin(), angular_indices.end(), std::back_inserter(total_indices));

  // Compute the sqrt information of the provided cov matrix
  fuse_core::Matrix6d sqrt_information = covariance.inverse().llt().matrixU();
  // std::cout << "sqrt_information = " << "\n" << sqrt_information << std::endl;

  // Assemble a mean vector and sqrt information matrix from the provided values, but in proper
  // Variable order
  //
  // What are we doing here? The constraint equation is defined as: cost(x) = ||A * (x - b)||^2
  // If we are measuring a subset of dimensions, we only want to produce costs for the measured
  // dimensions. But the variable vectors will be full sized. We can make this all work out by
  // creating a non-square A, where each row computes a cost for one measured dimensions,
  // and the columns are in the order defined by the variable.
  
  // build partial mean and information matrix
  mean_ = fuse_core::Vector7d::Zero();
  sqrt_information_ = fuse_core::Matrix6d::Zero();

  for (size_t i = 0; i < linear_indices.size(); ++i) {
    mean_(linear_indices[i]) = mean(linear_indices[i]);
  }

  mean_.tail(4) = mean.tail(4);

  for (auto& i : total_indices) {
    sqrt_information_.col(i) = sqrt_information.col(i);
  }  

  // std::cout << "sqrt_information_ = " << "\n" << sqrt_information_ << std::endl;
  // std::cout << "mean_ = " << mean_.transpose() << std::endl;
}

fuse_core::Matrix6d AbsolutePose3DStampedEulerConstraint::covariance() const
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
  // return new ceres::AutoDiffCostFunction<NormalPriorPose3DEulerCostFunctor, 6, 3, 4>(
    // new NormalPriorPose3DEulerCostFunctor(sqrt_information_, mean_, axes_));
  
  return new NormalPriorPose3D(sqrt_information_, mean_);
}

}  // namespace fuse_constraints

BOOST_CLASS_EXPORT_IMPLEMENT(fuse_constraints::AbsolutePose3DStampedEulerConstraint);
PLUGINLIB_EXPORT_CLASS(fuse_constraints::AbsolutePose3DStampedEulerConstraint, fuse_core::Constraint);
