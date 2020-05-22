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
#include <fuse_constraints/normal_prior_pose_2d.h>
#include <fuse_core/util.h>

#include <Eigen/Core>
#include <glog/logging.h>


namespace fuse_constraints
{

NormalPriorPose2D::NormalPriorPose2D(const fuse_core::MatrixXd& A, const fuse_core::Vector3d& b) :
  A_(A),
  b_(b)
{
  CHECK_GT(A_.rows(), 0);
  CHECK_EQ(A_.cols(), 3);
  set_num_residuals(A_.rows());
}

bool NormalPriorPose2D::Evaluate(
  double const* const* parameters,
  double* residuals,
  double** jacobians) const
{
  fuse_core::Vector3d full_residuals_vector;
  full_residuals_vector[0] = parameters[0][0] - b_[0];  // position x
  full_residuals_vector[1] = parameters[0][1] - b_[1];  // position y
  full_residuals_vector[2] = fuse_core::wrapAngle2D(parameters[1][0] - b_[2]);  // orientation

  // Scale the residuals by the square root information matrix to account for the measurement uncertainty.
  Eigen::Map<fuse_core::VectorXd> residuals_vector(residuals, num_residuals());
  residuals_vector = A_ * full_residuals_vector;

  if (jacobians != NULL)
  {
    // Jacobian wrt position
    if (jacobians[0] != NULL)
    {
      Eigen::Map<fuse_core::MatrixXd>(jacobians[0], num_residuals(), 2) = A_.leftCols<2>();
    }

    // Jacobian wrt orientation
    if (jacobians[1] != NULL)
    {
      Eigen::Map<fuse_core::VectorXd>(jacobians[1], num_residuals()) = A_.col(2);
    }
  }
  return true;
}

}  // namespace fuse_constraints
