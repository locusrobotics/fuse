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
#include <Eigen/Core>
#include <glog/logging.h>

#include <fuse_constraints/normal_delta.hpp>

namespace fuse_constraints
{

NormalDelta::NormalDelta(const fuse_core::MatrixXd & A, const fuse_core::VectorXd & b)
: A_(A),
  b_(b)
{
  CHECK_GT(b_.rows(), 0);
  CHECK_GT(A_.rows(), 0);
  CHECK_EQ(b_.rows(), A.cols());
  set_num_residuals(A_.rows());
  mutable_parameter_block_sizes()->push_back(b_.rows());
  mutable_parameter_block_sizes()->push_back(b_.rows());
}

bool NormalDelta::Evaluate(
  double const * const * parameters,
  double * residuals,
  double ** jacobians) const
{
  Eigen::Map<const fuse_core::VectorXd> x0(parameters[0], parameter_block_sizes()[0]);
  Eigen::Map<const fuse_core::VectorXd> x1(parameters[1], parameter_block_sizes()[1]);
  Eigen::Map<fuse_core::VectorXd> r(residuals, num_residuals());
  r = A_ * (x1 - x0 - b_);
  if (jacobians != nullptr) {
    if (jacobians[0] != nullptr) {
      Eigen::Map<fuse_core::MatrixXd>(
        jacobians[0], num_residuals(),
        parameter_block_sizes()[0]) = -A_;
    }
    if (jacobians[1] != nullptr) {
      Eigen::Map<fuse_core::MatrixXd>(
        jacobians[1], num_residuals(),
        parameter_block_sizes()[1]) = A_;
    }
  }
  return true;
}

}  // namespace fuse_constraints
