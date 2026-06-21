/*
 * Software License Agreement (BSD License)
 *
 *  Author: Oscar Mendez
 *  Created on Mon Dec 11 2023
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
#ifndef FUSE_CONSTRAINTS_REPROJECTION_ERROR_CONSTRAINT_H
#define FUSE_CONSTRAINTS_REPROJECTION_ERROR_CONSTRAINT_H

#include <fuse_core/constraint.h>
#include <fuse_core/eigen.h>
#include <fuse_core/fuse_macros.h>
#include <fuse_core/serialization.h>
#include <fuse_core/uuid.h>
#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>
#include <fuse_variables/pinhole_camera.h>

#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <Eigen/Dense>

#include <ostream>
#include <string>
#include <vector>

namespace fuse_constraints
{

/**
 * @brief A constraint that represents an observation of a 3D point using a Pinhole camera model
 *        which contains 4 paramters (fx, fy, cx, cy)
 *
 * A landmark is represented as a 3D point . This class takes the location of the 3D landmark and 
 * applies a reprojection-error based constraint on the position, orientation and calibration of 
 * the camera that observed the landmark.
 *
 */
class ReprojectionErrorConstraint : public fuse_core::Constraint
{
public:
  FUSE_CONSTRAINT_DEFINITIONS_WITH_EIGEN(ReprojectionErrorConstraint);

  /**
   * @brief Default constructor
   */
  ReprojectionErrorConstraint() = default;

  /**
   * @brief Create a constraint
   *
   * @param[in] source        The name of the sensor or motion model that generated this constraint
   * @param[in] position      The variable representing the position components of the camera pose
   * @param[in] orientation   The variable representing the orientation components of the camera pose
   * @param[in] calibraton    The calibration parameters of the camera (4x1 vector: fx, fy, cx, cy).
   *                          NOTE: Best practice is to fix this variable unless we have several observations
   *                          with the same camera
   * @param[in] mean          The measured observation of the point as a vector (2x1 vector: u,v)
   * @param[in] covariance    The prior observation covariance (2x2 matrix: u, v)
   */
  ReprojectionErrorConstraint(const std::string& source,
                            const fuse_variables::Position3DStamped& position,
                            const fuse_variables::Orientation3DStamped& orientation,
                            const fuse_variables::PinholeCamera& calibraton,
                            const fuse_core::Vector2d& mean,
                            const fuse_core::Matrix2d& covariance);

  /**
   * @brief Destructor
   */
  virtual ~ReprojectionErrorConstraint() = default;

  /**
   * @brief Read-only access to the square root information matrix.
   *
   * Order is (u, v)
   */
  const fuse_core::Matrix2d& sqrtInformation() const
  {
    return sqrt_information_;
  }

  /**
   * @brief Read-only access to the measured/prior vector of mean values.
   *
   * Order is (u, v)
   */
  const fuse_core::Vector2d& mean() const
  {
    return mean_;
  }

  /**
   * @brief Compute the measurement covariance matrix.
   *
   * Order is (u, v)
   */
  fuse_core::Matrix2d covariance() const
  {
    return (sqrt_information_.transpose() * sqrt_information_).inverse();
  }

  /**
   * @brief Print a human-readable description of the constraint to the provided stream.
   *
   * @param[out] stream The stream to write to. Defaults to stdout.
   */
  void print(std::ostream& stream = std::cout) const override;

  /**
   * @brief Construct an instance of this constraint's cost function
   *
   * The function caller will own the new cost function instance. It is the responsibility of the caller to delete
   * the cost function object when it is no longer needed. If the pointer is provided to a Ceres::Problem object, the
   * Ceres::Problem object will takes ownership of the pointer and delete it during destruction.
   *
   * @return A base pointer to an instance of a derived CostFunction.
   */
  ceres::CostFunction* costFunction() const override;

protected:
  fuse_core::Vector2d mean_;       //!< The 2D observations (in pixel space)
  fuse_core::Matrix2d sqrt_information_;  //!< The square root information matrix

private:
  // Allow Boost Serialization access to private methods
  friend class boost::serialization::access;

  /**
   * @brief The Boost Serialize method that serializes all of the data members in to/out of the archive
   *
   * @param[in/out] archive - The archive object that holds the serialized class members
   * @param[in] version - The version of the archive being read/written. Generally unused.
   */
  template <class Archive>
  void serialize(Archive& archive, const unsigned int /* version */)
  {
    archive& boost::serialization::base_object<fuse_core::Constraint>(*this);
    archive& mean_;
    archive& sqrt_information_;
  }
};

}  // namespace fuse_constraints

BOOST_CLASS_EXPORT_KEY(fuse_constraints::ReprojectionErrorConstraint);

#endif  // FUSE_CONSTRAINTS_REPROJECTION_ERROR_CONSTRAINT_H
