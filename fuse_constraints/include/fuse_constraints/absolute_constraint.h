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
#ifndef FUSE_CONSTRAINTS_ABSOLUTE_CONSTRAINT_H
#define FUSE_CONSTRAINTS_ABSOLUTE_CONSTRAINT_H

#include <fuse_core/constraint.h>
#include <fuse_core/macros.h>
#include <fuse_core/uuid.h>
#include <fuse_variables/acceleration_angular_2d_stamped.h>
#include <fuse_variables/acceleration_linear_2d_stamped.h>
#include <fuse_variables/orientation_2d_stamped.h>
#include <fuse_variables/position_2d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>
#include <fuse_variables/velocity_angular_2d_stamped.h>
#include <fuse_variables/velocity_linear_2d_stamped.h>

#include <ceres/cost_function.h>
#include <Eigen/Core>

#include <ostream>
#include <vector>


namespace fuse_constraints
{

/**
 * @brief A constraint that represents prior information about a variable or a direct measurement of the variable.
 *
 * This type of constraint arises in many situations. In mapping it is common to define the very first pose as the
 * origin. Some sensors, such as an IMU, provide direct measurements of certain state variables (e.g. linear
 * acceleration). And localization systems often match laserscans to a prior map (scan-to-map measurements).
 * This constraint holds the measured variable value and the measurement uncertainty/covariance.
 */
template<class Variable>
class AbsoluteConstraint final : public fuse_core::Constraint
{
public:
  SMART_PTR_DEFINITIONS(AbsoluteConstraint<Variable>);

  /**
   * @brief Create a constraint using a measurement/prior of all dimensions of the target variable
   *
   * @param[in] variable   An object derived from fuse_core::Variable.
   * @param[in] mean       The measured/prior value of all variable dimensions
   * @param[in] covariance The measurement/prior uncertainty of all variable dimensions
   */
  AbsoluteConstraint(
    const Variable& variable,
    const Eigen::VectorXd& mean,
    const Eigen::MatrixXd& covariance);

  /**
   * @brief Create a constraint using a measurement/prior of only a partial set of dimensions of the target variable
   *
   * @param[in] variable           An object derived from fuse_core::Variable.
   * @param[in] partial_mean       The measured value of the subset of dimensions in the order defined by \p indices
   * @param[in] partial_covariance The uncertainty of the subset of dimensions in the order defined by \p indices.
   * @param[in] indices            The set of indices corresponding to the measured dimensions
   */
  AbsoluteConstraint(
    const Variable& variable,
    const Eigen::VectorXd& partial_mean,
    const Eigen::MatrixXd& partial_covariance,
    const std::vector<size_t>& indices);

  /**
   * @brief Destructor
   */
  virtual ~AbsoluteConstraint() = default;

  /**
   * @brief Read-only access to the measured/prior vector of mean values.
   *
   * All dimensions are present, even if only a partial set of dimensions were measured. Dimensions are in the order
   * defined by the variable, not the order defined by the \p indices parameter. All unmeasured variable dimensions
   * are set to zero.
   */
  const Eigen::VectorXd& mean() const { return mean_; }

  /**
   * @brief Read-only access to the square root information matrix.
   *
   * Dimensions are in the order defined by the variable, not the order defined by the \p indices parameter. The
   * square root information matrix will have size measured_dimensions X variable_dimensions. If only a partial set
   * of dimensions are measured, then this matrix will not be square.
   */
  const Eigen::MatrixXd& sqrtInformation() const { return sqrt_information_; }

  /**
   * @brief Compute the measurement covariance matrix.
   *
   * Dimensions are in the order defined by the variable, not the order defined by the \p indices parameter. The
   * covariance matrix will always be square, with size variable_dimensions X variable_dimensions. If only a
   * subset of dimensions are measured, then some rows/columns will be zero. This will result in a rank-deficient
   * covariance matrix. You have been warned.
   */
  Eigen::MatrixXd covariance() const;

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
  Eigen::VectorXd mean_;  //!< The measured/prior mean vector for this variable
  Eigen::MatrixXd sqrt_information_;  //!< The square root information matrix
};

// Define unique names for the different variations of the absolute constraint
using AbsoluteAccelerationAngular2DStampedConstraint = AbsoluteConstraint<fuse_variables::AccelerationAngular2DStamped>;
using AbsoluteAccelerationLinear2DStampedConstraint = AbsoluteConstraint<fuse_variables::AccelerationLinear2DStamped>;
using AbsoluteOrientation2DStampedConstraint = AbsoluteConstraint<fuse_variables::Orientation2DStamped>;
using AbsolutePosition2DStampedConstraint = AbsoluteConstraint<fuse_variables::Position2DStamped>;
using AbsolutePosition3DStampedConstraint = AbsoluteConstraint<fuse_variables::Position3DStamped>;
using AbsoluteVelocityAngular2DStampedConstraint = AbsoluteConstraint<fuse_variables::VelocityAngular2DStamped>;
using AbsoluteVelocityLinear2DStampedConstraint = AbsoluteConstraint<fuse_variables::VelocityLinear2DStamped>;
}  // namespace fuse_constraints

// Include the template implementation
#include <fuse_constraints/absolute_constraint_impl.h>

#endif  // FUSE_CONSTRAINTS_ABSOLUTE_CONSTRAINT_H
