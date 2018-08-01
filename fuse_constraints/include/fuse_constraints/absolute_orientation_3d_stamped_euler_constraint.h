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
#ifndef FUSE_CONSTRAINTS_ABSOLUTE_ORIENTATION_3D_STAMPED_EULER_CONSTRAINT_H
#define FUSE_CONSTRAINTS_ABSOLUTE_ORIENTATION_3D_STAMPED_EULER_CONSTRAINT_H

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
#include <vector>


namespace fuse_constraints
{

/**
 * @brief A constraint that represents either prior information about a 3D orientation, or a direct measurement of the
 * 3D orientation.
 *
 * This constraint holds the measured 3D orientation and the measurement uncertainty/covariance. The orientation is
 * represented as Euler angles, and the covariance represents the error around each rotational axis. This constraint
 * also permits measurement of a subset of the Euler angles given in the variable.
 */
class AbsoluteOrientation3DStampedEulerConstraint : public fuse_core::Constraint
{
public:
  SMART_PTR_DEFINITIONS(AbsoluteOrientation3DStampedEulerConstraint);

  using Euler = fuse_variables::Orientation3DStamped::Euler;

  /**
   * @brief Create a constraint using a measurement/prior of a 3D orientation
   *
   * @param[in] orientation The variable representing the orientation components of the pose
   * @param[in] mean        The measured/prior Euler orientations in the order specified in /p axes
   * @param[in] covariance  The measurement/prior covariance
   * @param[in] axes        Used to specify which of the Euler axes they want to include in the constraint,
   *                        e.g. "{ Euler::ROLL, EULER::YAW }"
   */
  AbsoluteOrientation3DStampedEulerConstraint(
    const fuse_variables::Orientation3DStamped& orientation,
    const fuse_core::VectorXd& mean,
    const fuse_core::MatrixXd& covariance,
    const std::vector<Euler> &axes);

  /**
   * @brief Destructor
   */
  virtual ~AbsoluteOrientation3DStampedEulerConstraint() = default;

  /**
   * @brief Read-only access to the vector that dictates the order of the Euler axes in the \p mean, \p covariance, and
   *        \p sqrtInformation.
   */
  const std::vector<Euler> axes() const { return axes_; }

  /**
   * @brief Read-only access to the measured/prior vector of mean values.
   *
   * Order is defined by the provided \p axes parameter. This mean() function deviates from all other
   * currently implemented constraints in that the order does _not_ match the order defined in the variable.
   */
  const fuse_core::VectorXd& mean() const { return mean_; }

  /**
   * @brief Read-only access to the square root information matrix.
   *
   * Order is defined by the provided \p axes parameter. This sqrtInformation() function deviates from all other
   * currently implemented constraints in that the order does _not_ match the order defined in the variable.
   */
  const fuse_core::MatrixXd& sqrtInformation() const { return sqrt_information_; }

  /**
   * @brief Compute the measurement covariance matrix.
   *
   * Order is defined by the provided \p axes parameter. This covariance() function deviates from all other
   * currently implemented constraints in that the order does _not_ match the order defined in the variable.
   */
  fuse_core::MatrixXd covariance() const;

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
  fuse_core::VectorXd mean_;  //!< The measured/prior mean vector for this variable
  fuse_core::MatrixXd sqrt_information_;  //!< The square root information matrix
  std::vector<Euler> axes_;  //!< Which Euler angle axes we want to measure
};

}  // namespace fuse_constraints

#endif  // FUSE_CONSTRAINTS_ABSOLUTE_ORIENTATION_3D_STAMPED_EULER_CONSTRAINT_H
