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
#ifndef FUSE_VARIABLES_ORIENTATION_3D_STAMPED_H
#define FUSE_VARIABLES_ORIENTATION_3D_STAMPED_H

#include <fuse_core/macros.h>
#include <fuse_core/uuid.h>
#include <fuse_variables/fixed_size_variable.h>
#include <ros/time.h>

#include <ostream>


namespace fuse_variables
{

/**
 * @brief Variable representing a 3D orientation as a quaternion at a specific time and for a specific piece of
 * hardware (e.g., robot)
 *
 * This is commonly used to represent a robot orientation in single or multi-robot systems. The UUID of this class is
 * static after construction. As such, the timestamp and hardware ID cannot be modified. The value of the orientation
 * can be modified.
 * 
 * The internal representation for this is different from the typical ROS representation, as w is the first component.
 * This is necessary to use the Ceres local parameterization for quaternions.
 */
class Orientation3DStamped : public FixedSizeVariable<4>
{
public:
  SMART_PTR_DEFINITIONS(Orientation3DStamped);

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
   * @brief Construct a 3D orientation at a specific point in time.
   *
   * @param[IN]  stamp  The timestamp attached to this orientation.
   */
  explicit Orientation3DStamped(const ros::Time& stamp, const fuse_core::UUID &hardware_id = fuse_core::uuid::NIL);

  /**
   * @brief Read-only access to quaternion's Euler pitch angle component
   */
  double pitch();

  /**
   * @brief Read-only access to quaternion's Euler roll angle component
   */
  double roll();

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
   * @brief Read-only access to quaternion's Euler yaw angle component
   */
  double yaw();

  /**
   * @brief Read-write access to the quaternion z component
   */
  double& z() { return data_[Z]; }

  /**
   * @brief Read-only access to the quaternion z component
   */
  const double& z() const { return data_[Z]; }

  /**
   * @brief Read-only access to the associated timestamp.
   */
  const ros::Time& stamp() const { return stamp_; }

  /**
   * @brief Read-only access to the unique ID of the hardware device (e.g., robot) for which this variable is measured
   */
  const fuse_core::UUID hardware_id() const { return hardware_id_; }

  /**
   * @brief Read-only access to the unique ID of this variable instance.
   *
   * All variables of this type with identical timestamps will return the same UUID.
   */
  fuse_core::UUID uuid() const override { return uuid_; }

  /**
   * @brief Print a human-readable description of the variable to the provided stream.
   *
   * @param  stream The stream to write to. Defaults to stdout.
   */
  void print(std::ostream& stream = std::cout) const override;

  /**
   * @brief Perform a deep copy of the Variable and return a unique pointer to the copy
   *
   * @return A unique pointer to a new instance of the most-derived Variable
   */
  fuse_core::Variable::UniquePtr clone() const override;

  /**
   * @brief Provides a Ceres local parameterization for the quaternion
   *
   * @return A pointer to a local parameterization object that indicates how to "add" increments to the quaternion
   */
  ceres::LocalParameterization* localParameterization() const override;

protected:
  fuse_core::UUID hardware_id_;  //!< The UUID corresponding to the hardware device for which this variable is measured
  ros::Time stamp_;  //!< The timestamp associated with this variable instance
  fuse_core::UUID uuid_;  //!< The UUID for this instance, computed during construction
};

}  // namespace fuse_variables

#endif  // FUSE_VARIABLES_ORIENTATION_3D_STAMPED_H
