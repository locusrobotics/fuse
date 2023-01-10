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
#ifndef FUSE_VARIABLES__STAMPED_HPP_
#define FUSE_VARIABLES__STAMPED_HPP_

#include <fuse_core/fuse_macros.hpp>
#include <fuse_core/serialization.hpp>
#include <fuse_core/uuid.hpp>
#include <fuse_core/node_interfaces/node_interfaces.hpp>
#include <fuse_core/time.hpp>

#include <boost/serialization/access.hpp>


namespace fuse_variables
{

/**
 * @brief A class that provides a timestamp and device id
 *
 * This is intended to be used as secondary base class (multiple inheritance) for variables that are
 * time-varying. Some common examples include robot poses or velocities. This is in contrast to
 * variables that represent unknown but fixed quantities, such as the world position of landmarks,
 * or possibly calibration values that are assumed constant.
 */
class Stamped
{
public:
  FUSE_SMART_PTR_ALIASES_ONLY(Stamped)

  /**
   * @brief Default constructor
   */
  Stamped() = default;

  /**
   * @brief Constructor
   */
  explicit Stamped(
    const rclcpp::Time & stamp,
    const fuse_core::UUID & device_id = fuse_core::uuid::NIL)
  : device_id_(device_id),
    stamp_(stamp)
  {}

  /**
   * @brief Destructor
   */
  virtual ~Stamped() = default;

  /**
   * @brief Read-only access to the associated timestamp.
   */
  const rclcpp::Time & stamp() const {return stamp_;}

  /**
   * @brief Read-only access to the associated device ID.
   */
  const fuse_core::UUID & deviceId() const {return device_id_;}

private:
  fuse_core::UUID device_id_;  //!< The UUID associated with this specific device or hardware
  rclcpp::Time stamp_{ 0, 0, RCL_ROS_TIME };  //!< The timestamp associated with this variable instance

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
    archive & device_id_;
    archive & stamp_;
  }
};

/**
 * @brief Load a device id from the parameter server
 *
 * This function first checks if the 'device_id' parameter is available. If so, it attempts to load
 * the device_id using that parameter. There are a few supported formats:
 * - "01234567-89AB-CDEF-0123-456789ABCDEF"
 * - "01234567-89ab-cdef-0123-456789abcdef"
 * - "0123456789abcdef0123456789abcdef"
 * - "{01234567-89ab-cdef-0123-456789abcdef}"
 *
 * If the 'device_id' parameter doesn't exist, this function checks for the 'device_name' parameter.
 * If found, a device_id is generated by computing a hash on the provided string. See
 * fuse_core::uuid::generate(const std::string&) for details.
 *
 * If neither parameter exits, the device_id is populated by all zeros, the so-called NIL UUID.
 *
 * Will throw if the device_id parameter is not in an expected format.
 *
 * @param[in] interfaces  The node interfaces used to load parameters
 * @return                A device UUID
 */
fuse_core::UUID loadDeviceId(
  fuse_core::node_interfaces::NodeInterfaces<fuse_core::node_interfaces::Parameters> interfaces);

}  // namespace fuse_variables

#endif  // FUSE_VARIABLES__STAMPED_HPP_
