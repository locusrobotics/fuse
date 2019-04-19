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
#include <fuse_constraints/marginal_cost_function.h>

#include <Eigen/Core>

#include <vector>
#include <iostream>


namespace fuse_constraints
{

MarginalCostFunction::MarginalCostFunction(
    const std::vector<fuse_core::MatrixXd>& A,
    const std::vector<fuse_core::VectorXd>& x_bar,
    const std::vector<fuse_core::LocalParameterization::SharedPtr>& local,
    const fuse_core::VectorXd& b) :
  A_(A),
  x_bar_(x_bar),
  local_(local),
  b_(b)
{
  set_num_residuals(b_.rows());
  for (const auto& x_bar : x_bar_)
  {
    mutable_parameter_block_sizes()->push_back(x_bar.size());
  }
}

bool MarginalCostFunction::Evaluate(
  double const* const* parameters,
  double* residuals,
  double** jacobians) const
{
  // Compute cost
  Eigen::Map<fuse_core::VectorXd> residuals_map(residuals, num_residuals());
  residuals_map = b_;
  for (size_t i = 0; i < A_.size(); ++i)
  {
    fuse_core::VectorXd delta(A_[i].cols());
    if (local_[i])
    {
      local_[i]->Minus(x_bar_[i].data(), parameters[i], delta.data());
    }
    else
    {
      for (int j = 0; j < x_bar_[i].rows(); ++j)
      {
        delta[j] = parameters[i][j] - x_bar_[i][j];
      }
    }
    residuals_map += A_[i] * delta;
  }

  // Compute requested Jacobians
  if (jacobians)
  {
    for (size_t i = 0; i < A_.size(); ++i)
    {
      if (jacobians[i])
      {
        if (local_[i])
        {
          const auto local = local_[i];
          fuse_core::MatrixXd J_local(local->LocalSize(), local->GlobalSize());
          local->ComputeMinusJacobian(parameters[i], J_local.data());
          Eigen::Map<fuse_core::MatrixXd>(jacobians[i], num_residuals(), parameter_block_sizes()[i]) = A_[i] * J_local;
        }
        else
        {
          Eigen::Map<fuse_core::MatrixXd>(jacobians[i], num_residuals(), parameter_block_sizes()[i]) = A_[i];
        }
      }
    }
  }

  return true;
}

}  // namespace fuse_constraints
