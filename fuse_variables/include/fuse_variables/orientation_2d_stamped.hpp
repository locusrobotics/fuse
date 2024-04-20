/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Locus Robotics
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
#ifndef FUSE_VARIABLES__ORIENTATION_2D_STAMPED_HPP_
#define FUSE_VARIABLES__ORIENTATION_2D_STAMPED_HPP_

#include <ostream>

#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>

#include <fuse_core/ceres_macros.hpp>
#include <fuse_core/fuse_macros.hpp>
#include <fuse_core/local_parameterization.hpp>
#include <fuse_core/manifold.hpp>
#include <fuse_core/serialization.hpp>
#include <fuse_core/util.hpp>
#include <fuse_core/uuid.hpp>
#include <fuse_core/variable.hpp>
#include <fuse_variables/fixed_size_variable.hpp>
#include <fuse_variables/stamped.hpp>
#include <rclcpp/time.hpp>

namespace fuse_variables
{

/**
 * @brief A LocalParameterization class for 2D Orientations.
 *
 * 2D orientations add and subtract in the "usual" way, except for the 2*pi rollover issue. This
 * local parameterization handles the rollover. Because the Jacobians for this parameterization are
 * always identity, we implement this parameterization with "analytic" derivatives, instead of using
 * the Ceres's autodiff system.
 */
class Orientation2DLocalParameterization : public fuse_core::LocalParameterization
{
public:
  FUSE_SMART_PTR_DEFINITIONS(Orientation2DLocalParameterization)

  int GlobalSize() const override
  {
    return 1;
  }

  int LocalSize() const override
  {
    return 1;
  }

  bool Plus(
    const double * x,
    const double * delta,
    double * x_plus_delta) const override
  {
    // Compute the angle increment as a linear update, and handle the 2*Pi rollover
    x_plus_delta[0] = fuse_core::wrapAngle2D(x[0] + delta[0]);
    return true;
  }

  bool ComputeJacobian(
    const double * /*x*/,
    double * jacobian) const override
  {
    jacobian[0] = 1.0;
    return true;
  }

  bool Minus(
    const double * x,
    const double * y,
    double * y_minus_x) const override
  {
    // Compute the difference from x to y, and handle the 2*Pi rollover
    y_minus_x[0] = fuse_core::wrapAngle2D(y[0] - x[0]);
    return true;
  }

  bool ComputeMinusJacobian(
    const double * /*x*/,
    double * jacobian) const override
  {
    jacobian[0] = 1.0;
    return true;
  }

private:
  // Allow Boost Serialization access to private methods
  friend class boost::serialization::access;

  /**
   * @brief The Boost Serialize method that serializes all of the data members in to/out of the
   *        archive
   *
   * @param[in/out] archive - The archive object that holds the serialized class members
   * @param[in] version - The version of the archive being read/written. Generally unused.
   */
  template<class Archive>
  void serialize(Archive & archive, const unsigned int /* version */)
  {
    archive & boost::serialization::base_object<fuse_core::LocalParameterization>(*this);
  }
};

#if CERES_SUPPORTS_MANIFOLDS
/**
 * @brief A Manifold class for 2D Orientations.
 *
 * 2D orientations add and subtract in the "usual" way, except for the 2*pi rollover issue. This local parameterization
 * handles the rollover. Because the Jacobians for this parameterization are always identity, we implement this
 * parameterization with "analytic" derivatives, instead of using the Ceres's autodiff system.
 */
class Orientation2DManifold : public fuse_core::Manifold
{
public:
  FUSE_SMART_PTR_DEFINITIONS(Orientation2DManifold)

  int AmbientSize() const override {return 1;}

  int TangentSize() const override {return 1;}

  bool Plus(const double * x, const double * delta, double * x_plus_delta) const override
  {
    // Compute the angle increment as a linear update, and handle the 2*Pi rollover
    x_plus_delta[0] = fuse_core::wrapAngle2D(x[0] + delta[0]);
    return true;
  }

  bool PlusJacobian(const double * /*x*/, double * jacobian) const override
  {
    jacobian[0] = 1.0;
    return true;
  }

  bool Minus(const double * y, const double * x, double * y_minus_x) const override
  {
    // Compute the difference from y to x, and handle the 2*Pi rollover
    y_minus_x[0] = fuse_core::wrapAngle2D(y[0] - x[0]);
    return true;
  }

  bool MinusJacobian(const double * /*x*/, double * jacobian) const override
  {
    jacobian[0] = 1.0;
    return true;
  }

private:
  // Allow Boost Serialization access to private methods
  friend class boost::serialization::access;

  /**
   * @brief The Boost Serialize method that serializes all of the data members in to/out of the archive
   *
   * @param[in/out] archive - The archive object that holds the serialized class members
   * @param[in] version - The version of the archive being read/written. Generally unused.
   */
  template<class Archive>
  void serialize(Archive & archive, const unsigned int /* version */)
  {
    archive & boost::serialization::base_object<fuse_core::Manifold>(*this);
  }
};
#endif

/**
 * @brief Variable representing a 2D orientation (theta) at a specific time, with a specific piece
 *        of hardware.
 *
 * This is commonly used to represent a robot's orientation within a map. The UUID of this class is
 * static after construction. As such, the timestamp and device id cannot be modified. The value of
 * the orientation can be modified.
 */
class Orientation2DStamped : public FixedSizeVariable<1>, public Stamped
{
public:
  FUSE_VARIABLE_DEFINITIONS(Orientation2DStamped)

  /**
   * @brief Can be used to directly index variables in the data array
   */
  enum : size_t
  {
    YAW = 0
  };

  /**
   * @brief Default constructor
   */
  Orientation2DStamped() = default;

  /**
   * @brief Construct a 2D orientation at a specific point in time.
   *
   * @param[in] stamp     The timestamp attached to this orientation.
   * @param[in] device_id An optional device id, for use when variables originate from multiple
   *                      robots or devices
   */
  explicit Orientation2DStamped(
    const rclcpp::Time & stamp,
    const fuse_core::UUID & device_id = fuse_core::uuid::NIL);

  /**
   * @brief Read-write access to the heading angle.
   */
  double & yaw() {return data_[YAW];}

  /**
   * @brief Read-only access to the heading angle.
   */
  const double & yaw() const {return data_[YAW];}

  /**
   * @brief Print a human-readable description of the variable to the provided stream.
   *
   * @param[out] stream The stream to write to. Defaults to stdout.
   */
  void print(std::ostream & stream = std::cout) const override;

  /**
   * @brief Returns the number of elements of the local parameterization space.
   *
   * Since we are overriding the \p localParameterization() method, it is good practice to override
   * the \p localSize() method as well.
   */
  size_t localSize() const override {return 1u;}

  /**
   * @brief Create a new Ceres local parameterization object to apply to updates of this variable
   *
   * A 2D rotation has a nonlinearity when the angle wraps around from -PI to PI. This is handled by
   * a custom local parameterization to ensure smooth derivatives.
   *
   * @return A base pointer to an instance of a derived LocalParameterization
   */
  fuse_core::LocalParameterization * localParameterization() const override;

#if CERES_SUPPORTS_MANIFOLDS
  /**
   * @brief Create a new Ceres manifold object to apply to updates of this variable
   *
   * A 2D rotation has a nonlinearity when the angle wraps around from -PI to PI. This is handled by a custom
   * manifold to ensure smooth derivatives.
   *
   * @return A base pointer to an instance of a derived manifold
   */
  fuse_core::Manifold * manifold() const override;
#endif

private:
  // Allow Boost Serialization access to private methods
  friend class boost::serialization::access;

  /**
   * @brief The Boost Serialize method that serializes all of the data members in to/out of the
   *        archive
   *
   * @param[in/out] archive - The archive object that holds the serialized class members
   * @param[in] version - The version of the archive being read/written. Generally unused.
   */
  template<class Archive>
  void serialize(Archive & archive, const unsigned int /* version */)
  {
    archive & boost::serialization::base_object<FixedSizeVariable<SIZE>>(*this);
    archive & boost::serialization::base_object<Stamped>(*this);
  }
};

}  // namespace fuse_variables

#if CERES_SUPPORTS_MANIFOLDS
BOOST_CLASS_EXPORT_KEY(fuse_variables::Orientation2DManifold);
#endif
BOOST_CLASS_EXPORT_KEY(fuse_variables::Orientation2DLocalParameterization);
BOOST_CLASS_EXPORT_KEY(fuse_variables::Orientation2DStamped);

#endif  // FUSE_VARIABLES__ORIENTATION_2D_STAMPED_HPP_
