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
#ifndef FUSE_CONSTRAINTS__ABSOLUTE_CONSTRAINT_HPP_
#define FUSE_CONSTRAINTS__ABSOLUTE_CONSTRAINT_HPP_

#include <ceres/cost_function.h>

#include <ostream>
#include <string>
#include <vector>

#include <fuse_core/constraint.hpp>
#include <fuse_core/eigen.hpp>
#include <fuse_core/fuse_macros.hpp>
#include <fuse_core/serialization.hpp>
#include <fuse_core/uuid.hpp>
#include <fuse_variables/acceleration_angular_2d_stamped.hpp>
#include <fuse_variables/acceleration_angular_3d_stamped.hpp>
#include <fuse_variables/acceleration_linear_2d_stamped.hpp>
#include <fuse_variables/acceleration_linear_3d_stamped.hpp>
#include <fuse_variables/orientation_2d_stamped.hpp>
#include <fuse_variables/position_2d_stamped.hpp>
#include <fuse_variables/position_3d_stamped.hpp>
#include <fuse_variables/velocity_angular_2d_stamped.hpp>
#include <fuse_variables/velocity_angular_3d_stamped.hpp>
#include <fuse_variables/velocity_linear_3d_stamped.hpp>
#include <fuse_variables/velocity_linear_2d_stamped.hpp>

#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>


namespace fuse_constraints
{

/**
 * @brief A constraint that represents prior information about a variable or a direct measurement of
 *        the variable.
 *
 * This type of constraint arises in many situations. In mapping it is common to define the very
 * first pose as the origin. Some sensors, such as an IMU, provide direct measurements of certain
 * state variables (e.g. linear acceleration). And localization systems often match laserscans to a
 * prior map (scan-to-map measurements). This constraint holds the measured variable value and the
 * measurement uncertainty/covariance.
 */
template<class Variable>
class AbsoluteConstraint : public fuse_core::Constraint
{
public:
  FUSE_CONSTRAINT_DEFINITIONS(AbsoluteConstraint<Variable>)

  /**
   * @brief Default constructor
   */
  AbsoluteConstraint() = default;

  /**
   * @brief Create a constraint using a measurement/prior of all dimensions of the target variable
   *
   * @param[in] source     The name of the sensor or motion model that generated this constraint
   * @param[in] variable   An object derived from fuse_core::Variable.
   * @param[in] mean       The measured/prior value of all variable dimensions
   * @param[in] covariance The measurement/prior uncertainty of all variable dimensions
   */
  AbsoluteConstraint(
    const std::string & source,
    const Variable & variable,
    const fuse_core::VectorXd & mean,
    const fuse_core::MatrixXd & covariance);

  /**
   * @brief Create a constraint using a measurement/prior of only a partial set of dimensions of the
   *        target variable
   *
   * @param[in] source             The name of the sensor or motion model that generated this
   *                               constraint
   * @param[in] variable           An object derived from fuse_core::Variable.
   * @param[in] partial_mean       The measured value of the subset of dimensions in the order
   *                               defined by \p indices
   * @param[in] partial_covariance The uncertainty of the subset of dimensions in the order defined
   *                               by \p indices.
   * @param[in] indices            The set of indices corresponding to the measured dimensions
   */
  AbsoluteConstraint(
    const std::string & source,
    const Variable & variable,
    const fuse_core::VectorXd & partial_mean,
    const fuse_core::MatrixXd & partial_covariance,
    const std::vector<size_t> & indices);

  /**
   * @brief Destructor
   */
  virtual ~AbsoluteConstraint() = default;

  /**
   * @brief Read-only access to the measured/prior vector of mean values.
   *
   * All dimensions are present, even if only a partial set of dimensions were measured. Dimensions
   * are in the order defined by the variable, not the order defined by the \p indices parameter.
   * All unmeasured variable dimensions are set to zero.
   */
  const fuse_core::VectorXd & mean() const {return mean_;}

  /**
   * @brief Read-only access to the square root information matrix.
   *
   * Dimensions are in the order defined by the variable, not the order defined by the \p indices
   * parameter. The square root information matrix will have size measured_dimensions X
   * variable_dimensions. If only a partial set of dimensions are measured, then this matrix will
   * not be square.
   */
  const fuse_core::MatrixXd & sqrtInformation() const {return sqrt_information_;}

  /**
   * @brief Compute the measurement covariance matrix.
   *
   * Dimensions are in the order defined by the variable, not the order defined by the \p indices
   * parameter. The covariance matrix will always be square, with size variable_dimensions X
   * variable_dimensions. If only a subset of dimensions are measured, then some rows/columns will
   * be zero. This will result in a rank-deficient covariance matrix. You have been warned.
   */
  fuse_core::MatrixXd covariance() const;

  /**
   * @brief Print a human-readable description of the constraint to the provided stream.
   *
   * @param[out] stream The stream to write to. Defaults to stdout.
   */
  void print(std::ostream & stream = std::cout) const override;

  /**
   * @brief Construct an instance of this constraint's cost function
   *
   * The function caller will own the new cost function instance. It is the responsibility of the
   * caller to delete the cost function object when it is no longer needed. If the pointer is
   * provided to a Ceres::Problem object, the Ceres::Problem object will takes ownership of the
   * pointer and delete it during destruction.
   *
   * @return A base pointer to an instance of a derived CostFunction.
   */
  ceres::CostFunction * costFunction() const override;

protected:
  fuse_core::VectorXd mean_;  //!< The measured/prior mean vector for this variable
  fuse_core::MatrixXd sqrt_information_;  //!< The square root information matrix

private:
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
    archive & boost::serialization::base_object<fuse_core::Constraint>(*this);
    archive & mean_;
    archive & sqrt_information_;
  }
};

// Define unique names for the different variations of the absolute constraint
using AbsoluteAccelerationAngular2DStampedConstraint =
  AbsoluteConstraint<fuse_variables::AccelerationAngular2DStamped>;
using AbsoluteAccelerationAngular3DStampedConstraint =
  AbsoluteConstraint<fuse_variables::AccelerationAngular3DStamped>;
using AbsoluteAccelerationLinear2DStampedConstraint =
  AbsoluteConstraint<fuse_variables::AccelerationLinear2DStamped>;
using AbsoluteAccelerationLinear3DStampedConstraint = 
  AbsoluteConstraint<fuse_variables::AccelerationLinear3DStamped>;
using AbsoluteOrientation2DStampedConstraint =
  AbsoluteConstraint<fuse_variables::Orientation2DStamped>;
using AbsolutePosition2DStampedConstraint = AbsoluteConstraint<fuse_variables::Position2DStamped>;
using AbsolutePosition3DStampedConstraint = AbsoluteConstraint<fuse_variables::Position3DStamped>;
using AbsoluteVelocityAngular2DStampedConstraint =
  AbsoluteConstraint<fuse_variables::VelocityAngular2DStamped>;
using AbsoluteVelocityAngular3DStampedConstraint =
  AbsoluteConstraint<fuse_variables::VelocityAngular3DStamped>;
using AbsoluteVelocityLinear2DStampedConstraint =
  AbsoluteConstraint<fuse_variables::VelocityLinear2DStamped>;
using AbsoluteVelocityLinear3DStampedConstraint =
  AbsoluteConstraint<fuse_variables::VelocityLinear3DStamped>;
}  // namespace fuse_constraints

// Include the template implementation
#include <fuse_constraints/absolute_constraint_impl.hpp>

BOOST_CLASS_EXPORT_KEY(fuse_constraints::AbsoluteAccelerationAngular2DStampedConstraint);
BOOST_CLASS_EXPORT_KEY(fuse_constraints::AbsoluteAccelerationAngular3DStampedConstraint);
BOOST_CLASS_EXPORT_KEY(fuse_constraints::AbsoluteAccelerationLinear2DStampedConstraint);
BOOST_CLASS_EXPORT_KEY(fuse_constraints::AbsoluteAccelerationLinear3DStampedConstraint);
BOOST_CLASS_EXPORT_KEY(fuse_constraints::AbsoluteOrientation2DStampedConstraint);
BOOST_CLASS_EXPORT_KEY(fuse_constraints::AbsolutePosition2DStampedConstraint);
BOOST_CLASS_EXPORT_KEY(fuse_constraints::AbsolutePosition3DStampedConstraint);
BOOST_CLASS_EXPORT_KEY(fuse_constraints::AbsoluteVelocityAngular2DStampedConstraint);
BOOST_CLASS_EXPORT_KEY(fuse_constraints::AbsoluteVelocityAngular3DStampedConstraint);
BOOST_CLASS_EXPORT_KEY(fuse_constraints::AbsoluteVelocityLinear2DStampedConstraint);
BOOST_CLASS_EXPORT_KEY(fuse_constraints::AbsoluteVelocityLinear3DStampedConstraint);

#endif  // FUSE_CONSTRAINTS__ABSOLUTE_CONSTRAINT_HPP_
