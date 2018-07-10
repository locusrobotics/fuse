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
#ifndef FUSE_VARIABLES_ORIENTATION_2D_STAMPED_H
#define FUSE_VARIABLES_ORIENTATION_2D_STAMPED_H

#include <fuse_core/uuid.h>
#include <fuse_core/macros.h>
#include <fuse_variables/fixed_size_variable.h>
#include <ros/time.h>

#include <ceres/jet.h>
#include <ceres/local_parameterization.h>

#include <ostream>


namespace fuse_variables
{

/**
 * @brief Variable representing a 2D orientation (theta) at a specific time, with a specific piece of hardware.
 *
 * This is commonly used to represent a robot's orientation within a map. The UUID of this class is static after
 * construction. As such, the timestamp and hardware id cannot be modified (with the exception of the
 * deserializeMessage() function). The value of the orientation can be modified.
 */
class Orientation2DStamped final : public FixedSizeVariable<1>
{
public:
  SMART_PTR_DEFINITIONS(Orientation2DStamped);

  /**
   * @brief Can be used to directly index variables in the data array
   */
  enum : size_t
  {
    YAW = 0
  };

  /**
   * @brief Construct a 2D orientation at a specific point in time.
   *
   * @param[in] stamp       The timestamp attached to this orientation.
   * @param[in] hardware_id An optional hardware id, for use when variables originate from multiple robots or devices
   *
   */
  explicit Orientation2DStamped(const ros::Time& stamp, const fuse_core::UUID& hardware_id = fuse_core::uuid::NIL);

  /**
   * @brief Read-write access to the heading angle.
   */
  double& yaw() { return data_[YAW]; }

  /**
   * @brief Read-only access to the heading angle.
   */
  const double& yaw() const { return data_[YAW]; }

  /**
   * @brief Read-only access to the associated timestamp.
   */
  const ros::Time& stamp() const { return stamp_; }

  /**
   * @brief Read-only access to the associated hardware ID.
   */
  const fuse_core::UUID& hardwareId() const { return hardware_id_; }

  /**
   * @brief Read-only access to the unique ID of this variable instance.
   *
   * All variables of this type with identical timestamps will return the same UUID.
   */
  fuse_core::UUID uuid() const override { return uuid_; }

  /**
   * @brief Print a human-readable description of the variable to the provided stream.
   *
   * @param[out] stream The stream to write to. Defaults to stdout.
   */
  void print(std::ostream& stream = std::cout) const override;

  /**
   * @brief Perform a deep copy of the Variable and return a unique pointer to the copy
   *
   * @return A unique pointer to a new instance of the most-derived Variable
   */
  fuse_core::Variable::UniquePtr clone() const override;

  /**
   * @brief Create a new Ceres local parameterization object to apply to updates of this variable
   *
   * A 2D rotation has a nonlinearity when the angle wraps around from -PI to PI. This is handled by a custom
   * local parameterization to ensure smooth derivatives.
   *
   * @return A base pointer to an instance of a derived LocalParameterization
   */
  ceres::LocalParameterization* localParameterization() const override;

protected:
  fuse_core::UUID hardware_id_;  //!< The hardware UUID associated with this variable instance
  ros::Time stamp_;  //!< The timestamp associated with this variable instance
  fuse_core::UUID uuid_;  //!< The UUID for this instance, computed during construction

  /**
   * @brief Functor that computes an incremental update to a 2D orientation. This handles the 2*Pi rollover.
   *
   * This function is designed for use with Google's Ceres optimization engine. The Ceres variation of std::floor is
   * used, which has been specialized for Jet datatypes.
   */
  struct Orientation2DPlus
  {
    template<typename T>
    bool operator()(const T* x, const T* delta, T* x_plus_delta) const
    {
      // Define some necessary variations of PI with the correct type (double or Jet)
      static const T PI = T(M_PI);
      static const T TWO_PI = T(2 * M_PI);

      // Compute the angle increment as a linear update
      x_plus_delta[0] = x[0] + delta[0];
      // Then handle the 2*Pi roll-over
      // Use ceres::floor because it is specialized for double and Jet types.
      x_plus_delta[0] -= TWO_PI * ceres::floor((x_plus_delta[0] + PI) / TWO_PI);
      return true;
    }
  };
};

}  // namespace fuse_variables

#endif  // FUSE_VARIABLES_ORIENTATION_2D_STAMPED_H
