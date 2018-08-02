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
#ifndef FUSE_CONSTRAINTS_RELATIVE_POSE_2D_STAMPED_CONSTRAINT_H
#define FUSE_CONSTRAINTS_RELATIVE_POSE_2D_STAMPED_CONSTRAINT_H

#include <fuse_core/constraint.h>
#include <fuse_core/eigen.h>
#include <fuse_core/macros.h>
#include <fuse_core/uuid.h>
#include <fuse_variables/orientation_2d_stamped.h>
#include <fuse_variables/position_2d_stamped.h>

#include <Eigen/Dense>

#include <ostream>
#include <vector>


namespace fuse_constraints
{

/**
 * @brief A constraint that represents a measurement on the difference between two poses.
 *
 * This type of constraint arises in many situations. Many types of incremental odometry measurements (wheel encoders,
 * inertial strap-down, visual odometry) measure the change in the pose, not the pose directly. This constraint
 * holds the measured 2D pose change and the measurement uncertainty/covariance.
 */
class RelativePose2DStampedConstraint : public fuse_core::Constraint
{
public:
  SMART_PTR_DEFINITIONS(RelativePose2DStampedConstraint);

  /**
   * @brief Constructor
   *
   * @param[in] position1    The variable representing the position components of the first pose
   * @param[in] orientation1 The variable representing the orientation components of the first pose
   * @param[in] position2    The variable representing the position components of the second pose
   * @param[in] orientation2 The variable representing the orientation components of the second pose
   * @param[in] delta        The measured change in the pose (3x1 vector: dx, dy, dyaw)
   * @param[in] covariance   The measurement covariance (3x3 matrix: dx, dy, dyaw)
   */
  RelativePose2DStampedConstraint(
    const fuse_variables::Position2DStamped& position1,
    const fuse_variables::Orientation2DStamped& orientation1,
    const fuse_variables::Position2DStamped& position2,
    const fuse_variables::Orientation2DStamped& orientation2,
    const fuse_core::Vector3d& delta,
    const fuse_core::Matrix3d& covariance);

  /**
   * @brief Destructor
   */
  virtual ~RelativePose2DStampedConstraint() = default;

  /**
   * @brief Read-only access to the measured pose change.
   */
  const fuse_core::Vector3d& delta() const { return delta_; }

  /**
   * @brief Read-only access to the square root information matrix.
   */
  const fuse_core::Matrix3d& sqrtInformation() const { return sqrt_information_; }

  /**
   * @brief Compute the measurement covariance matrix.
   */
  fuse_core::Matrix3d covariance() const { return (sqrt_information_.transpose() * sqrt_information_).inverse(); }

  /**
   * @brief Print a human-readable description of the constraint to the provided stream.
   *
   * @param[out] stream The stream to write to. Defaults to stdout.
   */
  void print(std::ostream& stream = std::cout) const override;

  /**
   * @brief Perform a deep copy of the constraint and return a unique pointer to the copy
   *
   * Unique pointers can be implicitly upgraded to shared pointers if needed.
   *
   * @return A unique pointer to a new instance of the most-derived constraint
   */
  fuse_core::Constraint::UniquePtr clone() const override;

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
  fuse_core::Vector3d delta_;  //!< The measured pose change (dx, dy, dyaw)
  fuse_core::Matrix3d sqrt_information_;  //!< The square root information matrix (derived from the covariance matrix)
};

}  // namespace fuse_constraints

#endif  // FUSE_CONSTRAINTS_RELATIVE_POSE_2D_STAMPED_CONSTRAINT_H
