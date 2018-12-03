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
#include <fuse_constraints/absolute_pose_2d_stamped_constraint.h>
#include <fuse_constraints/normal_prior_pose_2d_cost_functor.h>

#include <ceres/autodiff_cost_function.h>
#include <Eigen/Dense>

#include <vector>


namespace fuse_constraints
{

AbsolutePose2DStampedConstraint::AbsolutePose2DStampedConstraint(
  const fuse_variables::Position2DStamped& position,
  const fuse_variables::Orientation2DStamped& orientation,
  const fuse_core::VectorXd& partial_mean,
  const fuse_core::MatrixXd& partial_covariance,
  const std::vector<size_t>& linear_indices,
  const std::vector<size_t>& angular_indices) :
    fuse_core::Constraint{position.uuid(), orientation.uuid()}
{
  size_t total_variable_size = position.size() + orientation.size();
  total_indices_ = linear_indices.size() + angular_indices.size();

  assert(partial_mean.rows() == static_cast<int>(total_indices_));
  assert(partial_covariance.rows() == static_cast<int>(total_indices_));
  assert(partial_covariance.cols() == static_cast<int>(total_indices_));

  // Compute the sqrt information of the provided cov matrix
  fuse_core::MatrixXd partial_sqrt_information = partial_covariance.inverse().llt().matrixU();

  // Assemble a mean vector and sqrt information matrix from the provided values, but in proper Variable order
  // What are we doing here?
  // The constraint equation is defined as: cost(x) = ||A * (x - b)||^2
  // If we are measuring a subset of dimensions, we only want to produce costs for the measured dimensions.
  // But the variable vectors will be full sized. We can make this all work out by creating a non-square A
  // matrix, where each row computes a cost for one measured dimensions, and the columns are in the order
  // defined by the variable.
  mean_ = fuse_core::VectorXd::Zero(total_variable_size);
  sqrt_information_ = fuse_core::MatrixXd::Zero(total_indices_, total_variable_size);
  for (size_t i = 0; i < linear_indices.size(); ++i)
  {
    mean_(linear_indices[i]) = partial_mean(i);
    sqrt_information_.col(linear_indices[i]) = partial_sqrt_information.col(i);
  }

  for (size_t i = linear_indices.size(); i < total_indices_; ++i)
  {
    size_t final_index = position.size() + angular_indices[i - linear_indices.size()];
    mean_(final_index) = partial_mean(i);
    sqrt_information_.col(final_index) = partial_sqrt_information.col(i);
  }
}

void AbsolutePose2DStampedConstraint::print(std::ostream& stream) const
{
  stream << type() << "\n"
         << "  uuid: " << uuid() << "\n"
         << "  position variable: " << variables_.at(0) << "\n"
         << "  orientation variable: " << variables_.at(1) << "\n"
         << "  mean: " << mean_.transpose() << "\n"
         << "  sqrt_info: " << sqrtInformation() << "\n";
}

fuse_core::Constraint::UniquePtr AbsolutePose2DStampedConstraint::clone() const
{
  return AbsolutePose2DStampedConstraint::make_unique(*this);
}

ceres::CostFunction* AbsolutePose2DStampedConstraint::costFunction() const
{
  return new ceres::AutoDiffCostFunction<NormalPriorPose2DCostFunctor, ceres::DYNAMIC, 2, 1>(
    new NormalPriorPose2DCostFunctor(sqrt_information_, mean_), total_indices_);
}

}  // namespace fuse_constraints
