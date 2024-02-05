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
#ifndef FUSE_CORE_MANIFOLD_H
#define FUSE_CORE_MANIFOLD_H

#include <fuse_core/ceres_macros.h>
#include <fuse_core/fuse_macros.h>
#include <fuse_core/serialization.h>

#include <boost/serialization/access.hpp>

#if CERES_VERSION_AT_LEAST(2, 1, 0)
// Local parameterizations got marked as deprecated in favour of Manifold in
// version 2.1.0, see
// https://github.com/ceres-solver/ceres-solver/commit/0141ca090c315db2f3c38e1731f0fe9754a4e4cc
// and they got removed in 2.2.0, see
// https://github.com/ceres-solver/ceres-solver/commit/68c53bb39552cd4abfd6381df08638285f7386b3
#include <ceres/manifold.h>

namespace fuse_core 
{

/**
 * @brief The Manifold interface definition.
 *
 * This class extends the Ceres Manifold class but adds methods to match the
 * fuse::LocalParameterization API
 *
 * See the Ceres documentation for more details.
 * http://ceres-solver.org/nnls_modeling.html#manifold
 */
class Manifold : public ceres::Manifold 
{
public:
  FUSE_SMART_PTR_ALIASES_ONLY(Manifold);

  /**
   * @brief Generalization of the addition operation, with the condition that
   * Plus(x, 0) = x
   *
   * @param[in] x    The value of the first variable, of size \p GlobalSize()
   * @param[in] delta The value of the perturbation, of size \p LocalSize()
   * @param[out] x_plus_delta Perturbed x, of size \p GlobalSize()
   * @return True if successful, false otherwise
   */
  virtual bool Plus(const double *x, const double *delta,
                    double *x_plus_delta) const = 0;

  /**
   * @brief The jacobian of Plus(x, delta) w.r.t delta at delta = 0.
   *
   * @param[in] x    The value of the first variable, of size \p GlobalSize()
   * @param[out] jacobian Jacobian is a row-major GlobalSize() x LocalSize()
   * matrix.
   * @return True if successful, false otherwise
   */
  virtual bool ComputeJacobian(const double *x,
                               double *jacobian) const = 0;

  /**
   * @brief Generalization of the subtraction operation
   *
   * Minus(x2, x1) -> x2_minus_x1
   *
   * with the conditions that:
   *  - Minus(x, x) -> 0
   *  - if Plus(x1, delta) -> x2, then Minus(x2, x1) -> delta
   *
   * @param[in]  x2    The value of the first variable, of size \p GlobalSize()
   * @param[in]  x1    The value of the second variable, of size \p GlobalSize()
   * @param[out] x2_minus_x1 The difference between x2 and x1, of size \p
   * LocalSize()
   * @return True if successful, false otherwise
   */
  virtual bool Minus(const double *x2, const double *x1,
                     double *x2_minus_x1) const = 0;

  /**
   * @brief The jacobian of Minus(x1, x2) w.r.t x2 at x1 == x2 == x
   *
   * @param[in]  x        The value used to evaluate the Jacobian, of size \p
   * GlobalSize()
   * @param[out] jacobian The first-order derivative in row-major order, of size
   * \p LocalSize() x \p GlobalSize()
   * @return True if successful, false otherwise
   */
  virtual bool ComputeMinusJacobian(const double *x,
                                    double *jacobian) const = 0;

  // Size of x.
  virtual int GlobalSize() const = 0;
  // Size of delta.
  virtual int LocalSize() const = 0;

  /// Equivalent to \p GlobalSize()
  int AmbientSize() const override 
  { 
    return GlobalSize(); 
  }

  /// Equivalent to \p LocalSize()
  int TangentSize() const override 
  { 
    return LocalSize(); 
  }

  /// Equivalent to \p ComputeJacobian()
  bool PlusJacobian(const double *x, double *jacobian) const override 
  {
    return ComputeJacobian(x, jacobian);
  }

  /// Equivalent to \p ComputeMinusJacobian()
  bool MinusJacobian(const double *x, double *jacobian) const override 
  {
    return ComputeMinusJacobian(x, jacobian);
  }

private:
  // Allow Boost Serialization access to private methods
  friend class boost::serialization::access;

  /**
   * @brief The Boost Serialize method that serializes all of the data members
   * in to/out of the archive
   *
   * @param[in/out] archive - The archive object that holds the serialized class
   * members
   * @param[in] version - The version of the archive being read/written.
   * Generally unused.
   */
  template <class Archive>
  void serialize(Archive & /* archive */, const unsigned int /* version */) {}
};

}  // namespace fuse_core

#endif

#endif  // FUSE_CORE_MANIFOLD_H
