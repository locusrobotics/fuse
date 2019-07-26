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
#include <fuse_constraints/dummy_constraint.h>
#include <fuse_variables/dummy_variable.h>

#include <pluginlib/class_list_macros.hpp>

#include <ceres/normal_prior.h>
#include <Eigen/Dense>

#include <vector>


namespace fuse_constraints
{

DummyConstraint::DummyConstraint(
  const fuse_variables::DummyVariable& variable,
  const fuse_core::Vector2d& mean,
  const fuse_core::Matrix2d& covariance) :
    fuse_core::Constraint{variable.uuid()},
    mean_(mean),
    sqrt_information_(covariance.inverse().llt().matrixU())
{
  assert(mean.rows() == static_cast<int>(variable.size()));
  assert(covariance.rows() == static_cast<int>(variable.size()));
  assert(covariance.cols() == static_cast<int>(variable.size()));
}

fuse_core::Matrix2d DummyConstraint::covariance() const
{
  // We want to compute:
  // cov = (sqrt_info' * sqrt_info)^-1
  // With some linear algebra, we can swap the transpose and the inverse.
  // cov = (sqrt_info^-1) * (sqrt_info^-1)'
  // But sqrt_info _may_ not be square. So we need to compute the pseudoinverse instead.
  // Eigen doesn't have a pseudoinverse function (for probably very legitimate reasons).
  // So we set the right hand side to identity, then solve using one of Eigen's many decompositions.
  auto I = fuse_core::Matrix2d::Identity();
  fuse_core::Matrix2d pinv = sqrt_information_.colPivHouseholderQr().solve(I);
  return pinv * pinv.transpose();
}

void DummyConstraint::print(std::ostream& stream) const
{
  stream << type() << "\n"
         << "  uuid: " << uuid() << "\n"
         << "  variable: " << variables()[0] << "\n"
         << "  mean: " << mean().transpose() << "\n"
         << "  sqrt_info: " << sqrtInformation() << "\n";
}

ceres::CostFunction* DummyConstraint::costFunction() const
{
  // Ceres ships with a "prior" cost function. Just use that here.
  return new ceres::NormalPrior(sqrt_information_, mean_);
}

}  // namespace fuse_constraints


// Register this variable with ROS as a plugin. This allows the pluginlib class loader to be used to deserialize
// variables in a generic manner in other classes.
PLUGINLIB_EXPORT_CLASS(fuse_constraints::DummyConstraint, fuse_core::Constraint);
