/*
 * Software License Agreement (BSD License)
 *
 *  Author:    Oscar Mendez
 *  Created:   11.13.2023
 *
 *  Copyright (c) 2023, Locus Robotics
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
#ifndef FUSE_VARIABLES_PINHOLE_CAMERA_FIXED_H
#define FUSE_VARIABLES_PINHOLE_CAMERA_FIXED_H

#include <fuse_core/ceres_macros.h>
#include <fuse_core/fuse_macros.h>
#include <fuse_core/manifold.h>
#include <fuse_core/serialization.h>
#include <fuse_variables/pinhole_camera.h>

#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>

namespace fuse_variables
{
/**
 * @brief Variable representing a pinhole camera that exists across time.
 *
 */
class PinholeCameraFixed : public PinholeCamera
{
public:
  FUSE_VARIABLE_DEFINITIONS(PinholeCameraFixed);

  /**
   * @brief Default constructor
   */
  PinholeCameraFixed() = default;

  /**
   * @brief Construct a point 2D variable given a landmarks id
   *
   * @param[in] camera_id  The id associated to a camera
   */
  explicit PinholeCameraFixed(const uint64_t& camera_id);

  /**
   * @brief Construct a pinhole camera variable given a camera id and intrinsic parameters
   *
   * @param[in] camera_id  The id associated to a camera
   */
  explicit PinholeCameraFixed(const uint64_t& camera_id,
                          const double& fx, const double& fy,
                          const double& cx, const double& cy);
  /**
   * @brief Specifies if the value of the variable should not be changed during optimization
   */
  bool holdConstant() const override { return true; }

#if CERES_SUPPORTS_MANIFOLDS
  /**
   * @brief Create a null Ceres manifold
   *
   * Overriding the manifold() method prevents additional processing with the ManifoldAdapter
   */
  fuse_core::Manifold* manifold() const override { return nullptr; }
#endif

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
  template <class Archive>
  void serialize(Archive& archive, const unsigned int /* version */)
  {
    archive& boost::serialization::base_object<PinholeCamera>(*this);
  }
};

}  // namespace fuse_variables

BOOST_CLASS_EXPORT_KEY(fuse_variables::PinholeCameraFixed);

#endif  // FUSE_VARIABLES_PINHOLE_CAMERA_FIXED_H
