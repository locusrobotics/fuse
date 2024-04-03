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

#ifndef FUSE_CORE__MANIFOLD_ADAPTER_HPP_
#define FUSE_CORE__MANIFOLD_ADAPTER_HPP_

#include <fuse_core/ceres_macros.hpp>

#if CERES_SUPPORTS_MANIFOLDS
#include <memory>
#include <utility>

#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/shared_ptr.hpp>

#include <fuse_core/fuse_macros.hpp>
#include <fuse_core/local_parameterization.hpp>
#include <fuse_core/manifold.hpp>

namespace fuse_core
{
class ManifoldAdapter : public fuse_core::Manifold
{
public:
  FUSE_SMART_PTR_DEFINITIONS(ManifoldAdapter)

  /**
   * @brief Constructor to adapt a fuse::LocalParameterization into a fuse::Manifold
   *
   * The ManifoldAdapter will take ownership of the provided LocalParameterization and
   * will delete it when the ManifoldAdapter goes out of scope.
   *
   * @param[in] local_parameterization Raw pointer to a derviced fuse::LocalParameterization object
   */
  explicit ManifoldAdapter(fuse_core::LocalParameterization * local_parameterization)
  {
    local_parameterization_.reset(local_parameterization);
  }

  /**
   * @brief Constructor to adapt a fuse::LocalParameterization into a fuse::Manifold
   *
   * The ManifoldAdapter will share ownership of the provided LocalParameterization object.
   *
   * @param[in] local_parameterization Shared pointer to a derived fuse::LocalParameterization object
   */
  explicit ManifoldAdapter(fuse_core::LocalParameterization::SharedPtr local_parameterization)
  {
    local_parameterization_ = std::move(local_parameterization);
  }

  /**
   * @brief Dimension of the ambient space in which the manifold is embedded.
   *
   * This is equivalent to the GlobalSize property of the LocalParameterization.
   *
   * @return int Dimension of the ambient space in which the manifold is embedded.
   */
  int AmbientSize() const override {return local_parameterization_->GlobalSize();}

  /**
   * @brief Dimension of the manifold/tangent space.
   *
   * This is equivalent to the LocalSize property of the LocalParameterization.
   *
   * @return int Dimension of the manifold/tangent space.
   */
  int TangentSize() const override {return local_parameterization_->LocalSize();}

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
  bool Plus(const double * x, const double * delta, double * x_plus_delta) const override
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
  bool PlusJacobian(const double * x, double * jacobian) const override
  {
    return local_parameterization_->ComputeJacobian(x, jacobian);
  }

  /**
   * @brief y_minus_x = Minus(y, x)
   *
   * Given two points on the manifold, Minus computes the change to x in the
   * tangent space at x, that will take it to y.
   *
   * Note that the parameter order for the Manifold class is different than
   * the parameter order for the LocalParameterization class.
   *
   * @param[in] y is a \p AmbientSize() vector.
   * @param[in] x is a \p AmbientSize() vector.
   * @param[out] y_minus_x is a \p TangentSize() vector.
   * @return Return value indicates if the operation was successful or not.
   */
  bool Minus(const double * y, const double * x, double * y_minus_x) const override
  {
    return local_parameterization_->Minus(x, y, y_minus_x);
  }

  /**
   * @brief Compute the derivative of Minus(y, x) w.r.t y at y = x, i.e
   *
   *      (D_1 Minus) (y, y)
   *
   * @param[in] x is a \p AmbientSize() vector.
   * @param[out] jacobian is a row-major \p TangentSize() x \p AmbientSize() matrix.
   * @return Return value indicates whether the operation was successful or not.
   */
  bool MinusJacobian(const double * x, double * jacobian) const override
  {
    return local_parameterization_->ComputeMinusJacobian(x, jacobian);
  }

private:
  // Allow Boost Serialization access to private methods
  friend class boost::serialization::access;

  ManifoldAdapter() = default;

  /**
   * @brief The Boost Serialize method that serializes all of the data members in to/out of the archive
   *
   * @param[in/out] archive - The archive object that holds the serialized class members
   * @param[in] version - The version of the archive being read/written. Generally unused.
   */
  template<class Archive>
  void serialize(Archive & archive, const unsigned int /* version */)
  {
    archive & boost::serialization::base_object<Manifold>(*this);
    archive & local_parameterization_;
  }

  /**
   * @brief A legacy LocalParametrization object that will be adapted to match the Manifold interface
   */
  fuse_core::LocalParameterization::SharedPtr local_parameterization_;
};

}  // namespace fuse_core

BOOST_CLASS_EXPORT_KEY(fuse_core::ManifoldAdapter);

#endif

#endif  // FUSE_CORE__MANIFOLD_ADAPTER_HPP_
