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

#include <fuse_core/ceres_macros.h>

#include <memory>

#if CERES_SUPPORTS_MANIFOLDS
#include <fuse_core/local_parameterization.h>
#include <fuse_core/manifold.h>

namespace fuse_core
{
class ManifoldAdapter : public fuse_core::Manifold
{
public:
  /**
   * @brief Constructor to adapt a fuse::LocalParameterization into a fuse::Manifold
   *
   * @param[in] local_parameterization fuse::LocalParameterization
   */
  explicit ManifoldAdapter(fuse_core::LocalParameterization* local_parameterization)
  {
    *local_parameterization_ = *local_parameterization;
  }

  // Dimension of the ambient space in which the manifold is embedded.
  int AmbientSize() const override { return local_parameterization_->GlobalSize(); }

  // Dimension of the manifold/tangent space.
  int TangentSize() const override { return local_parameterization_->LocalSize(); }

  /**
   * @brief  x_plus_delta = Plus(x, delta),
   *
   * A generalization of vector addition in Euclidean space, Plus computes the
   * result of moving along delta in the tangent space at x, and then projecting
   * back onto the manifold that x belongs to.
   *
   * @param[in] x is a \p AmbientSize() vector.
   * @param[in] delta delta is a \p TangentSize() vector.
   * @param[out] x_plus_delta  is a \p AmbientSize() vector.
   * @return Return value indicates if the operation was successful or not.
   */
  bool Plus(const double* x, const double* delta, double* x_plus_delta) const override
  {
    return local_parameterization_->Plus(x, delta, x_plus_delta);
  }

  /**
   * @brief Compute the derivative of Plus(x, delta) w.r.t delta at delta = 0,
   * i.e.
   *
   * (D_2 Plus)(x, 0)
   *
   * @param[in] x is a \p AmbientSize() vector
   * @param[out] jacobian is a row-major \p AmbientSize() x \p TangentSize()
   * matrix.
   * @return
   */
  bool PlusJacobian(const double* x, double* jacobian) const override
  {
    return local_parameterization_->ComputeJacobian(x, jacobian);
  }

  /**
   * @brief y_minus_x = Minus(y, x)
   *
   * Given two points on the manifold, Minus computes the change to x in the
   * tangent space at x, that will take it to y.
   *
   * @param[in] y is a \p AmbientSize() vector.
   * @param[in] x is a \p AmbientSize() vector.
   * @param[out] y_minus_x is a \p TangentSize() vector.
   * @return Return value indicates if the operation was successful or not.
   */
  bool Minus(const double* y, const double* x, double* y_minus_x) const override
  {
    return local_parameterization_->Minus(x, y, y_minus_x);
  }

  /**
   * @brief Compute the derivative of Minus(y, x) w.r.t y at y = x, i.e
   *
   *      (D_1 Minus) (x, x)
   *
   * @param[in] x is a \p AmbientSize() vector.
   * @param[out] jacobian is a row-major \p TangentSize() x \p AmbientSize() matrix.
   * @return Return value indicates whether the operation was successful or not.
   */
  bool MinusJacobian(const double* x, double* jacobian) const override
  {
    const auto success = local_parameterization_->ComputeMinusJacobian(x, jacobian);
    const auto N = TangentSize() * AmbientSize();
    for (int i = 0; i < N; i++)
    {
      jacobian[i] = -jacobian[i];
    }
    return success;
  }

private:
  std::unique_ptr<fuse_core::LocalParameterization> local_parameterization_;
};

}  // namespace fuse_core

#endif

#endif  // FUSE_CORE_MANIFOLD_ADAPTER_H
