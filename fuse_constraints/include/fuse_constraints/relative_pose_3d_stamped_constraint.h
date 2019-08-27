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
#ifndef FUSE_CONSTRAINTS_RELATIVE_POSE_3D_STAMPED_CONSTRAINT_H
#define FUSE_CONSTRAINTS_RELATIVE_POSE_3D_STAMPED_CONSTRAINT_H

#include <fuse_core/constraint.h>
#include <fuse_core/eigen.h>
#include <fuse_core/macros.h>
#include <fuse_core/serialization.h>
#include <fuse_core/uuid.h>
#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>

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
 * @brief A constraint that represents a measurement on the difference between two 3D poses.
 *
 * This type of constraint arises in many situations. Many types of incremental odometry measurements (e.g., visual
 * odometry) measure the change in the pose, not the pose directly. This constraint holds the measured 3D pose change
 * and the measurement uncertainty/covariance.
 */
class RelativePose3DStampedConstraint : public fuse_core::Constraint
{
public:
  FUSE_CONSTRAINT_DEFINITIONS_WITH_EIGEN(RelativePose3DStampedConstraint);

  /**
   * @brief Default constructor
   */
  RelativePose3DStampedConstraint() = default;

  /**
   * @brief Constructor
   *
   * @param[in] source       The name of the sensor or motion model that generated this constraint
   * @param[in] position1    The variable representing the position components of the first pose
   * @param[in] orientation1 The variable representing the orientation components of the first pose
   * @param[in] position2    The variable representing the position components of the second pose
   * @param[in] orientation2 The variable representing the orientation components of the second pose
   * @param[in] delta        The measured change in the pose (7x1 vector: dx, dy, dz, dqw, dqx, dqy, dqz)
   * @param[in] covariance   The measurement covariance (6x6 matrix: dx, dy, dz, dqx, dqy, dqz)
   */
  RelativePose3DStampedConstraint(
    const std::string& source,
    const fuse_variables::Position3DStamped& position1,
    const fuse_variables::Orientation3DStamped& orientation1,
    const fuse_variables::Position3DStamped& position2,
    const fuse_variables::Orientation3DStamped& orientation2,
    const fuse_core::Vector7d& delta,
    const fuse_core::Matrix6d& covariance);

  /**
   * @brief Destructor
   */
  virtual ~RelativePose3DStampedConstraint() = default;

  /**
   * @brief Read-only access to the measured pose change.
   */
  const fuse_core::Vector7d& delta() const { return delta_; }

  /**
   * @brief Read-only access to the square root information matrix.
   */
  const fuse_core::Matrix6d& sqrtInformation() const { return sqrt_information_; }

  /**
   * @brief Compute the measurement covariance matrix.
   */
  fuse_core::Matrix6d covariance() const { return (sqrt_information_.transpose() * sqrt_information_).inverse(); }

  /**
   * @brief Print a human-readable description of the constraint to the provided stream.
   *
   * @param[out] stream The stream to write to. Defaults to stdout.
   */
  void print(std::ostream& stream = std::cout) const override;

  /**
   * @brief Access the cost function for this constraint
   *
   * The function caller will own the new cost function instance. It is the responsibility of the caller to delete
   * the cost function object when it is no longer needed. If the pointer is provided to a Ceres::Problem object, the
   * Ceres::Problem object will takes ownership of the pointer and delete it during destruction.
   *
   * @return A base pointer to an instance of a derived CostFunction.
   */
  ceres::CostFunction* costFunction() const override;

protected:
  fuse_core::Vector7d delta_;  //!< The measured pose change (dx, dy, dz, dqw, dqx, dqy, dqz)
  fuse_core::Matrix6d sqrt_information_;  //!< The square root information matrix (derived from the covariance matrix)

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
    archive & boost::serialization::base_object<fuse_core::Constraint>(*this);
    archive & delta_;
    archive & sqrt_information_;
  }
};

}  // namespace fuse_constraints

BOOST_CLASS_EXPORT_KEY(fuse_constraints::RelativePose3DStampedConstraint);

#endif  // FUSE_CONSTRAINTS_RELATIVE_POSE_3D_STAMPED_CONSTRAINT_H
