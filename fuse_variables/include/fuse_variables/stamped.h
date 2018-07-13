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
#ifndef FUSE_VARIABLES_STAMPED_H
#define FUSE_VARIABLES_STAMPED_H

#include <fuse_core/macros.h>
#include <fuse_core/uuid.h>
#include <ros/time.h>


namespace fuse_variables
{

/**
 * @brief A class that provides a timestamp and device id
 *
 * something something something...
 */
class Stamped
{
public:
  SMART_PTR_ALIASES_ONLY(Stamped);

  /**
   * @brief Constructor
   */
  Stamped(const ros::Time& stamp, const fuse_core::UUID& device_id = fuse_core::uuid::NIL) :
    device_id_(device_id),
    stamp_(stamp)
  {}

  /**
   * @brief Destructor
   */
  virtual ~Stamped() = default;

  /**
   * @brief Read-only access to the associated timestamp.
   */
  const ros::Time& stamp() const { return stamp_; }

  /**
   * @brief Read-only access to the associated device ID.
   */
  const fuse_core::UUID& deviceId() const { return device_id_; }

protected:
  fuse_core::UUID device_id_;  //!< The UUID associated with this specific device or hardware
  ros::Time stamp_;  //!< The timestamp associated with this variable instance
};

}  // namespace fuse_variables

#endif  // FUSE_VARIABLES_STAMPED_H
