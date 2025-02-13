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
#ifndef FUSE_CORE__MANIFOLD_HPP_
#define FUSE_CORE__MANIFOLD_HPP_

#include <fuse_core/ceres_macros.hpp>

#if CERES_SUPPORTS_MANIFOLDS

#include <boost/serialization/access.hpp>
// Local parameterizations were marked as deprecated in favour of Manifold in
// version 2.1.0, see
// https://github.com/ceres-solver/ceres-solver/commit/0141ca090c315db2f3c38e1731f0fe9754a4e4cc
// and Local parameterizations were removed in 2.2.0, see
// https://github.com/ceres-solver/ceres-solver/commit/68c53bb39552cd4abfd6381df08638285f7386b3
#include <ceres/manifold.h>  // NOLINT[build/include_order]

#include <fuse_core/fuse_macros.hpp>
#include <fuse_core/serialization.hpp>

namespace fuse_core
{
/**
 * @brief The Manifold interface definition.
 *
 * In Ceres Solver version 2.1, the ceres::Manifold class was introduced, and in version 2.2 the
 * ceres::LocalParameterization was deprecated. In a similar way that the fuse::LocalParameterization
 * class wraps the ceres::LocalParameterization class, this class wraps the ceres::Manifold class
 * for use within the Fuse codebase.
 *
 * Conceptually, the LocalParameterization class and Manifold class represent the same concept --
 * switching between a nonlinear manifold and a linear approximation of the manifold tangent to
 * the manifold at a particular linearization point.
 *
 * The ceres::Manifold class renames a few members and adds a little more functionality.
 *  - ceres::LocalParameterization::GlobalSize becomes ceres::Manifold::AmbientSize
 *  - ceres::LocalParameterization::LocalSize becomes ceres::Manifold::TangentSize
 *  - ceres::LocalParameterization::ComputeJacobian() becomes ceres::Manifold::PlusJacobian()
 *  - ceres::Manifold adds Minus() and MinusJacobian() methods
 *
 * Note that the fuse::LocalParameterization has always had Minus() and ComputeMinusJacobian()
 * methods. However, the ceres::Manifold function signatures are slightly different from the
 * fuse::LocalParameterization function signatures.
 *
 * See the Ceres documentation for more details.
 * http://ceres-solver.org/nnls_modeling.html#manifold
 */
class Manifold : public ceres::Manifold
{
public:
  FUSE_SMART_PTR_ALIASES_ONLY(Manifold)

  /**
   * @brief Destroy the Manifold object
   */
  virtual ~Manifold() = default;

  /**
   * @brief Dimension of the ambient space in which the manifold is embedded.
   *
   * @return int Dimension of the ambient space in which the manifold is embedded.
   */
  virtual int AmbientSize() const = 0;

  /**
   * @brief Dimension of the manifold/tangent space.
   *
   * @return int Dimension of the manifold/tangent space.
   */
  virtual int TangentSize() const = 0;

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
  virtual bool Plus(const double * x, const double * delta, double * x_plus_delta) const = 0;

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
  virtual bool PlusJacobian(const double * x, double * jacobian) const = 0;

  /**
   * @brief Generalization of the subtraction operation
   *
   * Minus(y, x) -> y_minus_x
   *
   * with the conditions that:
   *  - Minus(x, x) -> 0
   *  - if Plus(x, delta) -> y, then Minus(y, x) -> delta
   *
   * @param[in] y is a \p AmbientSize() vector.
   * @param[in] x is a \p AmbientSize() vector.
   * @param[out] y_minus_x is a \p TangentSize() vector.
   * @return Return value indicates if the operation was successful or not.
   */
  virtual bool Minus(const double * y, const double * x, double * y_minus_x) const = 0;

  /**
   * @brief Compute the derivative of Minus(y, x) w.r.t y at y = x, i.e
   *
   *      (D_1 Minus) (y, y)
   *
   * @param[in] x is a \p AmbientSize() vector.
   * @param[out] jacobian is a row-major \p TangentSize() x \p AmbientSize() matrix.
   * @return Return value indicates whether the operation was successful or not.
   */
  virtual bool MinusJacobian(const double * x, double * jacobian) const = 0;

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
  template<class Archive>
  void serialize(Archive & /* archive */, const unsigned int /* version */) {}
};

}  // namespace fuse_core

#endif

#endif  // FUSE_CORE__MANIFOLD_HPP_
