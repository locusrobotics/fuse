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
#include <Eigen/Core>
#include <glog/logging.h>
#include <ceres/rotation.h>

#include <fuse_constraints/normal_prior_pose_3d_euler.hpp>
#include <fuse_core/eigen.hpp>
#include <fuse_core/util.hpp>

namespace fuse_constraints
{

NormalPriorPose3DEuler::NormalPriorPose3DEuler(const fuse_core::MatrixXd & A, const fuse_core::Vector6d & b)
: A_(A),
  b_(b)
{
  CHECK_GT(A_.rows(), 0);
  CHECK_EQ(A_.cols(), 6);
  set_num_residuals(A_.rows());
}

bool NormalPriorPose3DEuler::Evaluate(
  double const * const * parameters,
  double * residuals,
  double ** jacobians) const
{
  fuse_core::Vector6d full_residuals;
  double orientation_rpy[3];
  double j_quat2rpy[12];

  fuse_core::quaternion2rpy(parameters[1], orientation_rpy, j_quat2rpy);

  // Compute the position residual
  full_residuals(0) = parameters[0][0] - b_(0);
  full_residuals(1) = parameters[0][1] - b_(1);
  full_residuals(2) = parameters[0][2] - b_(2);

  // Compute the orientation residual
  full_residuals(3) = orientation_rpy[0] - b_(3);
  full_residuals(4) = orientation_rpy[1] - b_(4);
  full_residuals(5) = orientation_rpy[2] - b_(5);

  // Scale the residuals by the square root information matrix to account for
  // the measurement uncertainty.
    Eigen::Map<Eigen::Vector<double, Eigen::Dynamic>> residuals_map(residuals, A_.rows());
  residuals_map = A_ * full_residuals;

  if (jacobians != nullptr) {
    // Jacobian wrt position
    if (jacobians[0] != nullptr) {
      Eigen::Map<fuse_core::MatrixXd>(jacobians[0], num_residuals(), 3) = A_.leftCols<3>();
    }
    // Jacobian wrt orientation
    if (jacobians[1] != nullptr) {
      Eigen::Map<fuse_core::Matrix<double, 3, 4>> j_quat2angle_map(j_quat2rpy);
      Eigen::Map<fuse_core::MatrixXd>(jacobians[1], num_residuals(), 4) = 
        A_.rightCols<3>() * j_quat2angle_map;
    }
  }
  return true;
}

}  // namespace fuse_constraints
