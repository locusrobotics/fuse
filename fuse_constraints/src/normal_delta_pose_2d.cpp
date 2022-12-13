/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Clearpath Robotics
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

#include <fuse_constraints/normal_delta_pose_2d.hpp>
#include <fuse_core/util.hpp>

namespace fuse_constraints
{

NormalDeltaPose2D::NormalDeltaPose2D(const fuse_core::MatrixXd & A, const fuse_core::Vector3d & b)
: A_(A),
  b_(b)
{
  CHECK_GT(A_.rows(), 0);
  CHECK_EQ(A_.cols(), 3);
  set_num_residuals(A_.rows());
}

bool NormalDeltaPose2D::Evaluate(
  double const * const * parameters,
  double * residuals,
  double ** jacobians) const
{
  const fuse_core::Matrix2d R1_transpose =
    fuse_core::rotationMatrix2D(parameters[1][0]).transpose();  // orientation1
  const fuse_core::Vector2d position_delta =
    R1_transpose * fuse_core::Vector2d(
    parameters[2][0] - parameters[0][0],                        // position2.x - position1.x
    parameters[2][1] - parameters[0][1]);                       // position2.y - position1.y

  const fuse_core::Vector3d full_residuals_vector(
    position_delta[0] - b_[0], position_delta[1] - b_[1],
    // orientation2 - orientation1
    fuse_core::wrapAngle2D(parameters[3][0] - parameters[1][0] - b_[2]));

  // Scale the residuals by the square root information matrix to account for the measurement
  // uncertainty.
  Eigen::Map<fuse_core::VectorXd> residuals_vector(residuals, num_residuals());
  residuals_vector = A_ * full_residuals_vector;

  if (jacobians != nullptr) {
    // Jacobian wrt position1
    if (jacobians[0] != nullptr) {
      Eigen::Map<fuse_core::MatrixXd>(
        jacobians[0], num_residuals(),
        2) = -A_.leftCols<2>() * R1_transpose;
    }

    // Jacobian wrt orientation1
    if (jacobians[1] != nullptr) {
      Eigen::Map<fuse_core::VectorXd>(jacobians[1], num_residuals()) =
        A_ * fuse_core::Vector3d(position_delta[1], -position_delta[0], -1);
    }

    // Jacobian wrt position2
    if (jacobians[2] != nullptr) {
      Eigen::Map<fuse_core::MatrixXd>(
        jacobians[2], num_residuals(),
        2) = A_.leftCols<2>() * R1_transpose;
    }

    // Jacobian wrt orientation2
    if (jacobians[3] != nullptr) {
      Eigen::Map<fuse_core::VectorXd>(jacobians[3], num_residuals()) = A_.col(2);
    }
  }
  return true;
}

}  // namespace fuse_constraints
