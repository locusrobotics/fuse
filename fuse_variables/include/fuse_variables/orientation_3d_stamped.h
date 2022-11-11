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
#ifndef FUSE_VARIABLES_ORIENTATION_3D_STAMPED_H
#define FUSE_VARIABLES_ORIENTATION_3D_STAMPED_H

#include <fuse_core/local_parameterization.h>
#include <fuse_core/serialization.h>
#include <fuse_core/util.h>
#include <fuse_core/uuid.h>
#include <fuse_core/variable.h>
#include <fuse_variables/fixed_size_variable.h>
#include <fuse_variables/stamped.h>
#include <fuse_core/time.h>

#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <ceres/rotation.h>

#include <ostream>


namespace fuse_variables
{

/**
 * @brief A LocalParameterization class for 3D Orientations.
 *
 * 3D orientations add and subtract nonlinearly. Additionally, the typcial 3D orientation representation is a
 * quaternion, which has 4 degrees of freedom to parameterize a 3D space. This local parameterization uses the
 * Rodrigues/angle-axis formulas to combine 3D rotations, along with the appropriate "analytic" derivatives.
 */
class Orientation3DLocalParameterization : public fuse_core::LocalParameterization
{
public:
  /**
   * @brief Create the inverse quaternion
   *
   * ceres/rotation.h is missing this function for some reason.
   */
  template<typename T> inline
  static void QuaternionInverse(const T in[4], T out[4])
  {
    out[0] = in[0];
    out[1] = -in[1];
    out[2] = -in[2];
    out[3] = -in[3];
  }

  int GlobalSize() const override
  {
    return 4;
  }

  int LocalSize() const override
  {
    return 3;
  }

  bool Plus(
    const double* x,
    const double* delta,
    double* x_plus_delta) const override
  {
    double q_delta[4];
    ceres::AngleAxisToQuaternion(delta, q_delta);
    ceres::QuaternionProduct(x, q_delta, x_plus_delta);
    return true;
}

  bool ComputeJacobian(
    const double* x,
    double* jacobian) const override
  {
    double x0 = x[0] / 2;
    double x1 = x[1] / 2;
    double x2 = x[2] / 2;
    double x3 = x[3] / 2;
    jacobian[0] = -x1; jacobian[1]  = -x2; jacobian[2]  = -x3;  // NOLINT
    jacobian[3] =  x0; jacobian[4]  = -x3; jacobian[5]  =  x2;  // NOLINT
    jacobian[6] =  x3; jacobian[7]  =  x0; jacobian[8]  = -x1;  // NOLINT
    jacobian[9] = -x2; jacobian[10] =  x1; jacobian[11] =  x0;  // NOLINT
    return true;
  }

  bool Minus(
    const double* x1,
    const double* x2,
    double* delta) const override
  {
    double x1_inverse[4];
    QuaternionInverse(x1, x1_inverse);
    double q_delta[4];
    ceres::QuaternionProduct(x1_inverse, x2, q_delta);
    ceres::QuaternionToAngleAxis(q_delta, delta);
    return true;
  }

  bool ComputeMinusJacobian(
    const double* x,
    double* jacobian) const override
  {
    double x0 = x[0] * 2;
    double x1 = x[1] * 2;
    double x2 = x[2] * 2;
    double x3 = x[3] * 2;
    jacobian[0] = -x1; jacobian[1]  =  x0; jacobian[2]  =  x3;  jacobian[3]  = -x2;  // NOLINT
    jacobian[4] = -x2; jacobian[5]  = -x3; jacobian[6]  =  x0;  jacobian[7]  =  x1;  // NOLINT
    jacobian[8] = -x3; jacobian[9]  =  x2; jacobian[10] = -x1;  jacobian[11] =  x0;  // NOLINT
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
  void serialize(Archive& archive, const unsigned int /* version */)
  {
    archive & boost::serialization::base_object<fuse_core::LocalParameterization>(*this);
  }
};

/**
 * @brief Variable representing a 3D orientation as a quaternion at a specific time and for a specific piece of
 * hardware (e.g., robot)
 *
 * This is commonly used to represent a robot orientation in single or multi-robot systems. The UUID of this class is
 * static after construction. As such, the timestamp and device ID cannot be modified. The value of the orientation
 * can be modified.
 * 
 * The internal representation for this is different from the typical ROS representation, as w is the first component.
 * This is necessary to use the Ceres local parameterization for quaternions.
 */
class Orientation3DStamped : public FixedSizeVariable<4>, public Stamped
{
public:
  FUSE_VARIABLE_DEFINITIONS(Orientation3DStamped)

  /**
   * @brief Can be used to directly index variables in the quaternion
   */
  enum : size_t
  {
    W = 0,
    X = 1,
    Y = 2,
    Z = 3,
  };

  /**
   * @brief Can be used to reference Euler angles, but NOT as indices in the \p data_ member
   */
  enum class Euler : size_t
  {
    ROLL = 4,
    PITCH = 5,
    YAW = 6
  };

  /**
   * @brief Default constructor
   */
  Orientation3DStamped() = default;

  /**
   * @brief Construct a 3D orientation at a specific point in time.
   *
   * @param[in] stamp     The timestamp attached to this velocity.
   * @param[in] device_id An optional device id, for use when variables originate from multiple robots or devices
   */
  explicit Orientation3DStamped(const rclcpp::Time& stamp, const fuse_core::UUID& device_id = fuse_core::uuid::NIL);

  /**
   * @brief Read-write access to the quaternion w component
   */
  double& w() { return data_[W]; }

  /**
   * @brief Read-only access to the quaternion w component
   */
  const double& w() const { return data_[W]; }

  /**
   * @brief Read-write access to the quaternion x component
   */
  double& x() { return data_[X]; }

  /**
   * @brief Read-only access to the quaternion x component
   */
  const double& x() const { return data_[X]; }

  /**
   * @brief Read-write access to the quaternion y component
   */
  double& y() { return data_[Y]; }

  /**
   * @brief Read-only access to the quaternion y component
   */
  const double& y() const { return data_[Y]; }

  /**
   * @brief Read-write access to the quaternion z component
   */
  double& z() { return data_[Z]; }

  /**
   * @brief Read-only access to the quaternion z component
   */
  const double& z() const { return data_[Z]; }

  /**
   * @brief Read-only access to quaternion's Euler roll angle component
   */
  double roll() { return fuse_core::getRoll(w(), x(), y(), z()); }

  /**
   * @brief Read-only access to quaternion's Euler pitch angle component
   */
  double pitch() { return fuse_core::getPitch(w(), x(), y(), z()); }

  /**
   * @brief Read-only access to quaternion's Euler yaw angle component
   */
  double yaw() { return fuse_core::getYaw(w(), x(), y(), z()); }

  /**
   * @brief Print a human-readable description of the variable to the provided stream.
   *
   * @param  stream The stream to write to. Defaults to stdout.
   */
  void print(std::ostream& stream = std::cout) const override;

  /**
   * @brief Returns the number of elements of the local parameterization space.
   *
   * While a quaternion has 4 parameters, a 3D rotation only has 3 degrees of freedom. Hence, the local
   * parameterization space is only size 3.
   */
  size_t localSize() const override { return 3u; }

  /**
   * @brief Provides a Ceres local parameterization for the quaternion
   *
   * @return A pointer to a local parameterization object that indicates how to "add" increments to the quaternion
   */
  fuse_core::LocalParameterization* localParameterization() const override;

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
  void serialize(Archive& archive, const unsigned int /* version */)
  {
    archive & boost::serialization::base_object<FixedSizeVariable<SIZE>>(*this);
    archive & boost::serialization::base_object<Stamped>(*this);
  }
};

}  // namespace fuse_variables

BOOST_CLASS_EXPORT_KEY(fuse_variables::Orientation3DLocalParameterization);
BOOST_CLASS_EXPORT_KEY(fuse_variables::Orientation3DStamped);

#endif  // FUSE_VARIABLES_ORIENTATION_3D_STAMPED_H
