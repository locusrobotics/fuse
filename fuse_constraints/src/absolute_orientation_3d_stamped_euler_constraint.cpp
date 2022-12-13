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
#include <fuse_constraints/absolute_orientation_3d_stamped_euler_constraint.hpp>
#include <fuse_constraints/normal_prior_orientation_3d_euler_cost_functor.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace fuse_constraints
{

AbsoluteOrientation3DStampedEulerConstraint::AbsoluteOrientation3DStampedEulerConstraint(
  const std::string & source,
  const fuse_variables::Orientation3DStamped & orientation,
  const fuse_core::VectorXd & mean,
  const fuse_core::MatrixXd & covariance,
  const std::vector<Euler> & axes)
: fuse_core::Constraint(source, {orientation.uuid()}),    // NOLINT(whitespace/braces)
  mean_(mean),
  sqrt_information_(covariance.inverse().llt().matrixU()),
  axes_(axes)
{
  assert(covariance.rows() == static_cast<int>(axes.size()));
  assert(covariance.cols() == static_cast<int>(axes.size()));
  assert(mean.rows() == static_cast<int>(axes.size()));
}

fuse_core::MatrixXd AbsoluteOrientation3DStampedEulerConstraint::covariance() const
{
  return (sqrt_information_.transpose() * sqrt_information_).inverse();
}

void AbsoluteOrientation3DStampedEulerConstraint::print(std::ostream & stream) const
{
  stream << type() << "\n"
         << "  source: " << source() << "\n"
         << "  uuid: " << uuid() << "\n"
         << "  orientation variable: " << variables().at(0) << "\n"
         << "  mean: " << mean().transpose() << "\n"
         << "  sqrt_info: " << sqrtInformation() << "\n";

  if (loss()) {
    stream << "  loss: ";
    loss()->print(stream);
  }
}

ceres::CostFunction * AbsoluteOrientation3DStampedEulerConstraint::costFunction() const
{
  return new ceres::AutoDiffCostFunction<NormalPriorOrientation3DEulerCostFunctor, ceres::DYNAMIC,
           4>(
    new NormalPriorOrientation3DEulerCostFunctor(sqrt_information_, mean_, axes_), axes_.size());
}

}  // namespace fuse_constraints

BOOST_CLASS_EXPORT_IMPLEMENT(fuse_constraints::AbsoluteOrientation3DStampedEulerConstraint);
PLUGINLIB_EXPORT_CLASS(
  fuse_constraints::AbsoluteOrientation3DStampedEulerConstraint,
  fuse_core::Constraint);
