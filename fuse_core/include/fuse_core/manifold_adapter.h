/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, Clearpath Robotics
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

#ifndef FUSE_CORE_MANIFOLD_ADAPTER_H
#define FUSE_CORE_MANIFOLD_ADAPTER_H

#include <fuse_core/local_parameterization.h>
#include <fuse_core/manifold.h>

// This is only needed for exactly ceres == 2.1.x
#if CERES_VERSION_AT_LEAST(2, 1, 0)
#if !CERES_VERSION_AT_LEAST(2, 2, 0)

namespace fuse_core
{
class ManifoldAdapter : public fuse_core::Manifold
{
public:
  ManifoldAdapter(LocalParameterization* local_parameterization) : local_parameterization_(local_parameterization) {}

  int AmbientSize() const override { return local_parameterization_->GlobalSize(); }

  int TangentSize() const override { return local_parameterization_->LocalSize(); }

  bool Plus(const double* x, const double* delta, double* x_plus_delta) const override
  {
    return local_parameterization_->Plus(x, delta, x_plus_delta);
  }

  bool PlusJacobian(const double* x, double* jacobian) const override
  {
    return local_parameterization_->ComputeJacobian(x, jacobian);
  }

  bool Minus(const double* y, const double* x, double* y_minus_x) const override
  {
    return local_parameterization_->Minus(x, y, y_minus_x);
  }

  bool MinusJacobian(const double* x, double* jacobian) const override
  {
    return local_parameterization_->ComputeMinusJacobian(x, jacobian);
  }

private:
  fuse_core::LocalParameterization* local_parameterization_;
};

}  // namespace fuse_core

#endif
#endif

#endif  // FUSE_CORE_MANIFOLD_ADAPTER_H