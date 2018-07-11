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


namespace fuse_constraints
{

AbsolutePose2DStampedConstraint::AbsolutePose2DStampedConstraint(
  const fuse_variables::Position2DStamped& position,
  const fuse_variables::Orientation2DStamped& orientation,
  const Eigen::Vector3d& mean,
  const Eigen::Matrix3d& covariance) :
    fuse_core::Constraint{position.uuid(), orientation.uuid()},
    mean_(mean),
    sqrt_information_(covariance.inverse().llt().matrixU())
{
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
  return new ceres::AutoDiffCostFunction<NormalPriorPose2DCostFunctor, 3, 2, 1>(
    new NormalPriorPose2DCostFunctor(sqrt_information_, mean_));
}

}  // namespace fuse_constraints
