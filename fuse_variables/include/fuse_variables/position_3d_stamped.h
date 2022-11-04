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
#ifndef FUSE_VARIABLES_POSITION_3D_STAMPED_H
#define FUSE_VARIABLES_POSITION_3D_STAMPED_H

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
 * @brief Variable representing a 3D position (x, y, z) at a specific time and for a specific piece of hardware
 * (e.g., robot)
 *
 * This is commonly used to represent a robot position in single or multi-robot systems. The UUID of this class is
 * static after construction. As such, the timestamp and device ID cannot be modified. The value of the position
 * can be modified.
 */
class Position3DStamped : public FixedSizeVariable<3>, public Stamped
{
public:
  FUSE_VARIABLE_DEFINITIONS(Position3DStamped)

  /**
   * @brief Can be used to directly index variables in the data array
   */
  enum : size_t
  {
    X = 0,
    Y = 1,
    Z = 2
  };

  /**
   * @brief Default constructor
   */
  Position3DStamped() = default;

  /**
   * @brief Construct a 3D position at a specific point in time.
   *
   * @param[in] stamp     The timestamp attached to this position.
   * @param[in] device_id An optional device id, for use when variables originate from multiple robots or devices
   */
  explicit Position3DStamped(const rclcpp::Time& stamp, const fuse_core::UUID &device_id = fuse_core::uuid::NIL);

  /**
   * @brief Read-write access to the X-axis position.
   */
  double& x() { return data_[X]; }

  /**
   * @brief Read-only access to the X-axis position.
   */
  const double& x() const { return data_[X]; }

  /**
   * @brief Read-write access to the Y-axis position.
   */
  double& y() { return data_[Y]; }

  /**
   * @brief Read-only access to the Y-axis position.
   */
  const double& y() const { return data_[Y]; }

  /**
   * @brief Read-write access to the Z-axis position.
   */
  double& z() { return data_[Z]; }

  /**
   * @brief Read-only access to the Z-axis position.
   */
  const double& z() const { return data_[Z]; }

  /**
   * @brief Print a human-readable description of the variable to the provided stream.
   *
   * @param  stream The stream to write to. Defaults to stdout.
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

BOOST_CLASS_EXPORT_KEY(fuse_variables::Position3DStamped);

#endif  // FUSE_VARIABLES_POSITION_3D_STAMPED_H
