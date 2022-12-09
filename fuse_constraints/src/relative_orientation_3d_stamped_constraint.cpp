/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Locus Robotics
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
#include <Eigen/Geometry>

#include <string>

#include <boost/serialization/export.hpp>
#include <fuse_constraints/normal_delta_orientation_3d_cost_functor.hpp>
#include <fuse_constraints/relative_orientation_3d_stamped_constraint.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace fuse_constraints
{

RelativeOrientation3DStampedConstraint::RelativeOrientation3DStampedConstraint(
  const std::string & source,
  const fuse_variables::Orientation3DStamped & orientation1,
  const fuse_variables::Orientation3DStamped & orientation2,
  const fuse_core::Vector4d & delta,
  const fuse_core::Matrix3d & covariance)
: fuse_core::Constraint(source, {orientation1.uuid(), orientation2.uuid()}),  // NOLINT
  delta_(delta),
  sqrt_information_(covariance.inverse().llt().matrixU())
{
}

RelativeOrientation3DStampedConstraint::RelativeOrientation3DStampedConstraint(
  const std::string & source,
  const fuse_variables::Orientation3DStamped & orientation1,
  const fuse_variables::Orientation3DStamped & orientation2,
  const Eigen::Quaterniond & delta,
  const fuse_core::Matrix3d & covariance)
: RelativeOrientation3DStampedConstraint(source, orientation1, orientation2, toEigen(
      delta), covariance)
{
}

RelativeOrientation3DStampedConstraint::RelativeOrientation3DStampedConstraint(
  const std::string & source,
  const fuse_variables::Orientation3DStamped & orientation1,
  const fuse_variables::Orientation3DStamped & orientation2,
  const geometry_msgs::msg::Quaternion & delta,
  const std::array<double, 9> & covariance)
: RelativeOrientation3DStampedConstraint(
    source, orientation1, orientation2, toEigen(delta), toEigen(covariance))
{
}

fuse_core::Matrix3d RelativeOrientation3DStampedConstraint::covariance() const
{
  return (sqrt_information_.transpose() * sqrt_information_).inverse();
}

void RelativeOrientation3DStampedConstraint::print(std::ostream & stream) const
{
  stream << type() << "\n"
         << "  source: " << source() << "\n"
         << "  uuid: " << uuid() << "\n"
         << "  orientation variable1: " << variables().at(0) << "\n"
         << "  orientation variable2: " << variables().at(1) << "\n"
         << "  delta: " << delta().transpose() << "\n"
         << "  sqrt_info: " << sqrtInformation() << "\n";

  if (loss()) {
    stream << "  loss: ";
    loss()->print(stream);
  }
}

ceres::CostFunction * RelativeOrientation3DStampedConstraint::costFunction() const
{
  return new ceres::AutoDiffCostFunction<NormalDeltaOrientation3DCostFunctor, 3, 4, 4>(
    new NormalDeltaOrientation3DCostFunctor(sqrt_information_, delta_));
}

fuse_core::Vector4d RelativeOrientation3DStampedConstraint::toEigen(
  const Eigen::Quaterniond & quaternion)
{
  fuse_core::Vector4d eigen_quaternion_vector;
  eigen_quaternion_vector << quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z();
  return eigen_quaternion_vector;
}

fuse_core::Vector4d RelativeOrientation3DStampedConstraint::toEigen(
  const geometry_msgs::msg::Quaternion & quaternion)
{
  fuse_core::Vector4d eigen_quaternion_vector;
  eigen_quaternion_vector << quaternion.w, quaternion.x, quaternion.y, quaternion.z;
  return eigen_quaternion_vector;
}

fuse_core::Matrix3d RelativeOrientation3DStampedConstraint::toEigen(
  const std::array<double,
  9> & covariance)
{
  return fuse_core::Matrix3d(covariance.data());
}

}  // namespace fuse_constraints

BOOST_CLASS_EXPORT_IMPLEMENT(fuse_constraints::RelativeOrientation3DStampedConstraint);
PLUGINLIB_EXPORT_CLASS(
  fuse_constraints::RelativeOrientation3DStampedConstraint,
  fuse_core::Constraint);
