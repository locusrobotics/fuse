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
#ifndef FUSE_CONSTRAINTS_ABSOLUTE_ORIENTATION_3D_STAMPED_CONSTRAINT_H
#define FUSE_CONSTRAINTS_ABSOLUTE_ORIENTATION_3D_STAMPED_CONSTRAINT_H

#include <fuse_core/constraint.h>
#include <fuse_core/eigen.h>
#include <fuse_core/macros.h>
#include <fuse_core/uuid.h>
#include <fuse_variables/orientation_3d_stamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Quaternion.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <array>
#include <ostream>


namespace fuse_constraints
{

/**
 * @brief A constraint that represents either prior information about a 3D orientation, or a direct measurement of the
 * 3D orientation.
 *
 * This constraint holds the measured 3D orientation and the measurement uncertainty/covariance.
 */
class AbsoluteOrientation3DStampedConstraint : public fuse_core::Constraint
{
public:
  SMART_PTR_DEFINITIONS(AbsoluteOrientation3DStampedConstraint);

  /**
   * @brief Create a constraint using a measurement/prior of a 3D orientation
   *
   * @param[in] orientation The variable representing the orientation components of the pose
   * @param[in] mean        The measured/prior orientation as a quaternion (4x1 vector: w, x, y, z)
   * @param[in] covariance  The measurement/prior covariance (3x3 matrix: qx, qy, qz)
   */
  AbsoluteOrientation3DStampedConstraint(
    const fuse_variables::Orientation3DStamped& orientation,
    const fuse_core::Vector4d& mean,
    const fuse_core::Matrix3d& covariance);

  /**
   * @brief Create a constraint using a measurement/prior of a 3D orientation
   *
   * @param[in] orientation The variable representing the orientation components of the pose
   * @param[in] mean        The measured/prior orientation as an Eigen quaternion
   * @param[in] covariance  The measurement/prior covariance (3x3 matrix: qx, qy, qz)
   */
  AbsoluteOrientation3DStampedConstraint(
    const fuse_variables::Orientation3DStamped& orientation,
    const Eigen::Quaterniond& mean,
    const fuse_core::Matrix3d& covariance);

  /**
   * @brief Create a constraint using a measurement/prior of a 3D orientation
   *
   * @param[in] orientation The variable representing the orientation components of the pose
   * @param[in] mean        The measured/prior orientation as a ROS quaternion message
   * @param[in] covariance  The measurement/prior covariance (3x3 matrix: qx, qy, qz)
   */
  AbsoluteOrientation3DStampedConstraint(
    const fuse_variables::Orientation3DStamped& orientation,
    const geometry_msgs::Quaternion& mean,
    const std::array<double, 9>& covariance);

  /**
   * @brief Destructor
   */
  virtual ~AbsoluteOrientation3DStampedConstraint() = default;

  /**
   * @brief Read-only access to the measured/prior vector of mean values.
   *
   * Order is (w, x, y, z)
   */
  const fuse_core::Vector4d& mean() const { return mean_; }

  /**
   * @brief Read-only access to the square root information matrix.
   *
   * Order is (qx, qy, qz)
   */
  const fuse_core::Matrix3d& sqrtInformation() const { return sqrt_information_; }

  /**
   * @brief Compute the measurement covariance matrix.
   *
   * Order is (qx, qy, qz)
   */
  fuse_core::Matrix3d covariance() const;

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
  /**
   * @brief Utility method to convert an Eigen quaternion to an Eigen Vector4d
   * @param[in] quaternion - The input Eigen quaternion
   * @return The \p quaternion, converted to an Eigen Vector4d
   */
  static fuse_core::Vector4d toEigen(const Eigen::Quaterniond& quaternion);

  /**
   * @brief Utility method to convert an ROS quaternion message to an Eigen Vector4d
   * @param[in] quaternion - The input ROS quaternion message
   * @return The \p quaternion, converted to an Eigen Vector4d
   */
  static fuse_core::Vector4d toEigen(const geometry_msgs::Quaternion& quaternion);

  /**
   * @brief Utility method to convert a flat 1D array to a 3x3 Eigen matrix
   * @param[in] covariance - The input covariance array
   * @return The \p covariance, converted to an Eigen Matrix3d
   */
  static fuse_core::Matrix3d toEigen(const std::array<double, 9>& covariance);

  fuse_core::Vector4d mean_;  //!< The measured/prior mean vector for this variable
  fuse_core::Matrix3d sqrt_information_;  //!< The square root information matrix
};

}  // namespace fuse_constraints

#endif  // FUSE_CONSTRAINTS_ABSOLUTE_ORIENTATION_3D_STAMPED_CONSTRAINT_H
