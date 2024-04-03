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
#ifndef FUSE_VARIABLES_PINHOLE_CAMERA_H
#define FUSE_VARIABLES_PINHOLE_CAMERA_H

#include <fuse_core/ceres_macros.h>
#include <fuse_core/fuse_macros.h>
#include <fuse_core/manifold.h>
#include <fuse_core/serialization.h>
#include <fuse_core/uuid.h>
#include <fuse_variables/fixed_size_variable.h>

#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>

#include <ostream>

namespace fuse_variables
{
/**
 * @brief Variable representing intrinsic parameters of a camera.
 *
 * The UUID of this class is constant after
 * construction and dependent on a user input database id. As such, the database id cannot be altered after
 * construction.
 */
class PinholeCamera : public FixedSizeVariable<4>
{
public:
  FUSE_VARIABLE_DEFINITIONS(PinholeCamera);

  /**
   * @brief Can be used to directly index variables in the data array
   */
  enum : size_t
  {
    FX = 0,
    FY = 1,
    CX = 2,
    CY = 3
  };

  /**
   * @brief Default constructor
   */
  PinholeCamera() = default;

  /**
   * @brief Construct a pinhole camera variable given a camera id
   *
   * @param[in] camera_id  The id associated to a camera
   */
  explicit PinholeCamera(const uint64_t& camera_id);

  /**
   * @brief Construct a pinhole camera variable given a camera id and intrinsic parameters
   *
   * @param[in] camera_id  The id associated to a camera
   */
  explicit PinholeCamera(const fuse_core::UUID& uuid, const uint64_t& camera_id,
                          const double& fx, const double& fy,
                          const double& cx, const double& cy);

  /**
   * @brief Read-write access to the cx parameter.
   */
  double& cx() { return data_[CX]; }

  /**
   * @brief Read-only access to the cx parameter.
   */
  const double& cx() const { return data_[CX]; }

  /**
   * @brief Read-write access to the cy parameter.
   */
  double& cy() { return data_[CY]; }

  /**
   * @brief Read-only access to the cy parameter.
   */
  const double& cy() const { return data_[CY]; }

  /**
   * @brief Read-write access to the fx parameter.
   */
  double& fx() { return data_[FX]; }

  /**
   * @brief Read-only access to the fx parameter.
   */
  const double& fx() const { return data_[FX]; }

  /**
   * @brief Read-write access to the fy parameter.
   */
  double& fy() { return data_[FY]; }

  /**
   * @brief Read-only access to the fy parameter.
   */
  const double& fy() const { return data_[FY]; }

  /**
   * @brief Read-only access to the id
   */
  const uint64_t& id() const { return id_; }

  /**
   * @brief Print a human-readable description of the variable to the provided
   * stream.
   *
   * @param[out] stream The stream to write to. Defaults to stdout.
   */
  void print(std::ostream& stream = std::cout) const override;

#if CERES_SUPPORTS_MANIFOLDS
  /**
   * @brief Create a null Ceres manifold
   *
   * Overriding the manifold() method prevents additional processing with the ManifoldAdapter
   */
  fuse_core::Manifold* manifold() const override { return nullptr; }
#endif

protected:
  /**
   * @brief Construct a point 3D variable given a landmarks id
   *
   * @param[in] landmark_id  The id associated to a landmark
   */
  PinholeCamera(const fuse_core::UUID& uuid, const uint64_t& landmark_id);

private:
  // Allow Boost Serialization access to private methods
  friend class boost::serialization::access;
  uint64_t id_ { 0 };

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
    archive& boost::serialization::base_object<FixedSizeVariable<SIZE>>(*this);
    archive& id_;
  }
};

}  // namespace fuse_variables

BOOST_CLASS_EXPORT_KEY(fuse_variables::PinholeCamera);

#endif  // FUSE_VARIABLES_PINHOLE_CAMERA_H
