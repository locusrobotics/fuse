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
  // TODO(giafranchini): implement cost function with analytical derivatives
  return new ceres::AutoDiffCostFunction<NormalDeltaPose3DCostFunctor, 6, 3, 4, 3, 4>(
    new NormalDeltaPose3DCostFunctor(sqrt_information_, delta_));
}

}  // namespace fuse_constraints

BOOST_CLASS_EXPORT_IMPLEMENT(fuse_constraints::RelativePose3DStampedConstraint);
PLUGINLIB_EXPORT_CLASS(fuse_constraints::RelativePose3DStampedConstraint, fuse_core::Constraint);
