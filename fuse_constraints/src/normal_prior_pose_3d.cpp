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
#include <ceres/rotation.h>

#include <fuse_constraints/normal_prior_pose_3d.hpp>
#include <fuse_core/util.hpp>

#include <covariance_geometry/covariance_representation.hpp>

namespace fuse_constraints
{

NormalPriorPose3D::NormalPriorPose3D(const fuse_core::MatrixXd & A, const fuse_core::Vector7d & b)
: A_(A),
  b_(b)
{
  CHECK_GT(A_.rows(), 0);
  CHECK_EQ(A_.cols(), 6);
  set_num_residuals(A_.rows());
}

bool NormalPriorPose3D::Evaluate(
  double const * const * parameters,
  double * residuals,
  double ** jacobians) const
{
  fuse_core::Vector6d full_residuals_vector;
  full_residuals_vector[0] = parameters[0][0] - b_[0];  // position x
  full_residuals_vector[1] = parameters[0][1] - b_[1];  // position y
  full_residuals_vector[2] = parameters[0][2] - b_[2];  // position z

  // double * qb_inverse[4], q_or[4], q_res[4];

  // qb_inverse = {
  //      b_(3),
  //     -b_(4),
  //     -b_(5),
  //     -b_(6)
  //   };

  // q_or = {
  //     parameters[1][0],
  //     parameters[1][1],
  //     parameters[1][2],
  //     parameters[1][3]
  //   };

  // ceres::QuaternionProduct(qb_inverse, q_or, q_res);

  Eigen::Map<const Eigen::Quaterniond> q_orientation_map(parameters[1]);
  auto qb_inv = Eigen::Quaterniond(b_(3), b_(4), b_(5), b_(6)).conjugate();
  auto q_res = qb_inv * q_orientation_map;
  
  full_residuals_vector[3] = fuse_core::getRoll(q_res.w(), q_res.x(), q_res.y(), q_res.z());  // orientation roll
  full_residuals_vector[4] = fuse_core::getPitch(q_res.w(), q_res.x(), q_res.y(), q_res.z()); // orientation pitch
  full_residuals_vector[5] = fuse_core::getYaw(q_res.w(), q_res.x(), q_res.y(), q_res.z());   // orientation yaw
  
  // Scale the residuals by the square root information matrix to account for the measurement
  // uncertainty.
  Eigen::Map<fuse_core::VectorXd> residuals_vector(residuals, num_residuals());
  residuals_vector = A_ * full_residuals_vector;

  if (jacobians != nullptr) {
    // Jacobian of the position residuals wrt position parameters block (max 3x3)
    if (jacobians[0] != nullptr) {
      Eigen::Map<fuse_core::MatrixXd>(jacobians[0], num_residuals(), 3) = A_.leftCols<3>();
    }

    // Jacobian of the orientation residuals wrt orientation parameters block (max 3x4)
    if (jacobians[1] != nullptr) {
      // We make use of the chain rule of derivatives:
      // First: compute the jacobian of the quaternion product wrt orientation parameters
      fuse_core::Matrix4d dqprod_dq1;
      dqprod_dq1(0, 0) =  qb_inv.w();
      dqprod_dq1(0, 1) = -qb_inv.x();
      dqprod_dq1(0, 2) = -qb_inv.y();
      dqprod_dq1(0, 3) = -qb_inv.z();

      dqprod_dq1(1, 0) =  qb_inv.x();
      dqprod_dq1(1, 1) =  qb_inv.w();
      dqprod_dq1(1, 2) = -qb_inv.z();
      dqprod_dq1(1, 3) =  qb_inv.y();

      dqprod_dq1(2, 0) =  qb_inv.y();
      dqprod_dq1(2, 1) =  qb_inv.z();
      dqprod_dq1(2, 2) =  qb_inv.w();
      dqprod_dq1(2, 3) = -qb_inv.x();

      dqprod_dq1(3, 0) =  qb_inv.z();
      dqprod_dq1(3, 1) = -qb_inv.y();
      dqprod_dq1(3, 2) =  qb_inv.x();
      dqprod_dq1(3, 3) =  qb_inv.w();

      // Second: compute the jacobian of the quat2eul function wrt the quaternion residual
      Eigen::Matrix3_4d dquat2eul_dq;
      covariance_geometry::jacobianQuaternionToRPY(q_res, dquat2eul_dq);

      // Third: apply the chain rule
      Eigen::Map<fuse_core::MatrixXd>(jacobians[1], num_residuals(), 3) = A_.rightCols<3>() * dquat2eul_dq * dqprod_dq1;
    }
  }
  return true;
}

}  // namespace fuse_constraints
