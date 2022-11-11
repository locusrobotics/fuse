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
#ifndef FUSE_VARIABLES_ACCELERATION_ANGULAR_3D_STAMPED_H
#define FUSE_VARIABLES_ACCELERATION_ANGULAR_3D_STAMPED_H

#include <fuse_core/uuid.h>
#include <fuse_core/serialization.h>
#include <fuse_core/variable.h>
#include <fuse_variables/fixed_size_variable.h>
#include <fuse_variables/stamped.h>
#include <fuse_core/time.h>

#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>

#include <ostream>


namespace fuse_variables
{

/**
 * @brief Variable representing a 3D angular acceleration (aroll, apitch, ayaw) at a specific time, with a specific
 * piece of hardware.
 *
 * This is commonly used to represent a robot's acceleration. The UUID of this class is static after construction.
 * As such, the timestamp and device id cannot be modified. The value of the acceleration can be modified.
 */
class AccelerationAngular3DStamped : public FixedSizeVariable<3>, public Stamped
{
public:
  FUSE_VARIABLE_DEFINITIONS(AccelerationAngular3DStamped)

  /**
   * @brief Can be used to directly index variables in the data array
   */
  enum : size_t
  {
    ROLL = 0,
    PITCH = 1,
    YAW = 2
  };

  /**
   * @brief Default constructor
   */
  AccelerationAngular3DStamped() = default;

  /**
   * @brief Construct a 3D angular acceleration at a specific point in time.
   *
   * @param[in] stamp     The timestamp attached to this angular acceleration.
   * @param[in] device_id An optional device id, for use when variables originate from multiple robots or devices
   */
  explicit AccelerationAngular3DStamped(
    const rclcpp::Time& stamp,
    const fuse_core::UUID& device_id = fuse_core::uuid::NIL);

  /**
   * @brief Read-write access to the roll (X-axis) angular acceleration.
   */
  double& roll() { return data_[ROLL]; }

  /**
   * @brief Read-only access to the roll (X-axis) angular acceleration.
   */
  const double& roll() const { return data_[ROLL]; }

  /**
   * @brief Read-write access to the pitch (Y-axis) angular acceleration.
   */
  double& pitch() { return data_[PITCH]; }

  /**
   * @brief Read-only access to the pitch (Y-axis) angular acceleration.
   */
  const double& pitch() const { return data_[PITCH]; }

  /**
   * @brief Read-write access to the yaw (Z-axis) angular acceleration.
   */
  double& yaw() { return data_[YAW]; }

  /**
   * @brief Read-only access to the yaw (Z-axis) angular acceleration.
   */
  const double& yaw() const { return data_[YAW]; }

  /**
   * @brief Print a human-readable description of the variable to the provided stream.
   *
   * @param[out] stream The stream to write to. Defaults to stdout.
   */
  void print(std::ostream& stream = std::cout) const override;

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

BOOST_CLASS_EXPORT_KEY(fuse_variables::AccelerationAngular3DStamped);

#endif  // FUSE_VARIABLES_ACCELERATION_ANGULAR_3D_STAMPED_H
