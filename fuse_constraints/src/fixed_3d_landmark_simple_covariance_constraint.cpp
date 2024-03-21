/*
 * Software License Agreement (BSD License)
 *
 *  Author: Oscar Mendez
 *  Created on Mon Nov 12 2023
 *
 *  Copyright (c) 2023, Locus Robotics
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
#include <fuse_constraints/fixed_3d_landmark_simple_covariance_constraint.h>
#include <fuse_constraints/fixed_3d_landmark_simple_covariance_cost_functor.h>
#include <pluginlib/class_list_macros.hpp>

#include <boost/serialization/export.hpp>
#include <ceres/autodiff_cost_function.h>
#include <Eigen/Dense>

#include <string>

namespace fuse_constraints
{

Fixed3DLandmarkSimpleCovarianceConstraint::Fixed3DLandmarkSimpleCovarianceConstraint(
    const std::string& source, const fuse_variables::Position3DStamped& position,
    const fuse_variables::Orientation3DStamped& orientation, const fuse_variables::PinholeCamera& calibration,
    const fuse_core::MatrixXd& pts3d, const fuse_core::MatrixXd& observations, const fuse_core::Vector7d& mean,
    const fuse_core::Matrix2d& covariance)
  : fuse_core::Constraint(source, { position.uuid(), orientation.uuid(), calibration.uuid() })
  , pts3d_(pts3d)
  , observations_(observations)
  , mean_(mean)
  , sqrt_information_(covariance.inverse().llt().matrixU())
{
  assert(pts3d_.cols() == 3);
  assert(observations_.cols() == 2);
  assert(pts3d_.rows() == observations_.rows());
}

Fixed3DLandmarkSimpleCovarianceConstraint::Fixed3DLandmarkSimpleCovarianceConstraint(
    const std::string& source, const fuse_variables::Position3DStamped& position,
    const fuse_variables::Orientation3DStamped& orientation, const fuse_variables::PinholeCamera& calibration,
    const double& marker_size, const fuse_core::MatrixXd& observations, const fuse_core::Vector7d& mean,
    const fuse_core::Matrix2d& covariance)
  : fuse_core::Constraint(source, { position.uuid(), orientation.uuid(), calibration.uuid() })
  , pts3d_(4, 3)
  , observations_(observations)
  , mean_(mean)
  , sqrt_information_(covariance.inverse().llt().matrixU())
{
  // Define 3D Homogeneous 3D Points at origin, assume z-up
  pts3d_ << -1.0, -1.0, 0.0,  // NOLINT
            -1.0,  1.0, 0.0,  // NOLINT
             1.0, -1.0, 0.0,  // NOLINT
             1.0,  1.0, 0.0;  // NOLINT
  pts3d_ *= marker_size;      // Scalar Multiplication

  assert(pts3d_.cols() == 3);
  assert(observations_.cols() == 2);
  assert(pts3d_.rows() == observations_.rows());
}

void Fixed3DLandmarkSimpleCovarianceConstraint::print(std::ostream& stream) const
{
  stream << type() << "\n"
         << "  source: " << source() << "\n"
         << "  uuid: " << uuid() << "\n"
         << "  position variable: " << variables().at(0) << "\n"
         << "  orientation variable: " << variables().at(1) << "\n"
         << "  mean: " << mean().transpose() << "\n"
         << "  sqrt_info: " << sqrtInformation() << "\n"
         << "  observations: " << observations() << "\n";

  if (loss())
  {
    stream << "  loss: ";
    loss()->print(stream);
  }
}

ceres::CostFunction* Fixed3DLandmarkSimpleCovarianceConstraint::costFunction() const
{
  return new ceres::AutoDiffCostFunction<Fixed3DLandmarkSimpleCovarianceCostFunctor, 8, 3, 4, 4>(
      new Fixed3DLandmarkSimpleCovarianceCostFunctor(sqrt_information_, mean_, observations_, pts3d_));
}

}  // namespace fuse_constraints

BOOST_CLASS_EXPORT_IMPLEMENT(fuse_constraints::Fixed3DLandmarkSimpleCovarianceConstraint);
PLUGINLIB_EXPORT_CLASS(fuse_constraints::Fixed3DLandmarkSimpleCovarianceConstraint, fuse_core::Constraint);
