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
#ifndef FUSE_CONSTRAINTS__ABSOLUTE_ORIENTATION_3D_STAMPED_EULER_CONSTRAINT_HPP_
#define FUSE_CONSTRAINTS__ABSOLUTE_ORIENTATION_3D_STAMPED_EULER_CONSTRAINT_HPP_

#include <ostream>
#include <string>
#include <vector>

#include <fuse_core/constraint.hpp>
#include <fuse_core/eigen.hpp>
#include <fuse_core/fuse_macros.hpp>
#include <fuse_core/serialization.hpp>
#include <fuse_core/uuid.hpp>
#include <fuse_variables/orientation_3d_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/vector.hpp>


namespace fuse_constraints
{

/**
 * @brief A constraint that represents either prior information about a 3D orientation, or a direct
 *        measurement of the 3D orientation as roll-pitch-yaw Euler angles.
 *
 * This constraint holds the measured 3D orientation and the measurement uncertainty/covariance. The
 * orientation is represented as Euler angles, and the covariance represents the error around each
 * rotational axis. This constraint also permits measurement of a subset of the Euler angles given
 * in the variable.
 */
class AbsoluteOrientation3DStampedEulerConstraint : public fuse_core::Constraint
{
public:
  FUSE_CONSTRAINT_DEFINITIONS(AbsoluteOrientation3DStampedEulerConstraint)

  using Euler = fuse_variables::Orientation3DStamped::Euler;

  /**
   * @brief Default constructor
   */
  AbsoluteOrientation3DStampedEulerConstraint() = default;

  /**
   * @brief Create a constraint using a measurement/prior of a 3D orientation
   *
   * @param[in] source      The name of the sensor or motion model that generated this constraint
   * @param[in] orientation The variable representing the orientation components of the pose
   * @param[in] mean        The measured/prior Euler orientations in the order specified in /p axes
   * @param[in] covariance  The measurement/prior covariance
   * @param[in] axes        Used to specify which of the Euler axes they want to include in the
   *                        constraint, e.g. "{ Euler::ROLL, EULER::YAW }"
   */
  AbsoluteOrientation3DStampedEulerConstraint(
    const std::string & source,
    const fuse_variables::Orientation3DStamped & orientation,
    const fuse_core::VectorXd & mean,
    const fuse_core::MatrixXd & covariance,
    const std::vector<Euler> & axes);

  /**
   * @brief Destructor
   */
  virtual ~AbsoluteOrientation3DStampedEulerConstraint() = default;

  /**
   * @brief Read-only access to the vector that dictates the order of the Euler axes in the \p mean,
   *        \p covariance, and \p sqrtInformation.
   */
  const std::vector<Euler> axes() const {return axes_;}

  /**
   * @brief Read-only access to the measured/prior vector of mean values.
   *
   * Order is defined by the provided \p axes parameter. This mean() function deviates from all
   * other currently implemented constraints in that the order does _not_ match the order defined in
   * the variable.
   */
  const fuse_core::VectorXd & mean() const {return mean_;}

  /**
   * @brief Read-only access to the square root information matrix.
   *
   * Order is defined by the provided \p axes parameter. This sqrtInformation() function deviates
   * from all other currently implemented constraints in that the order does _not_ match the order
   * defined in the variable.
   */
  const fuse_core::MatrixXd & sqrtInformation() const {return sqrt_information_;}

  /**
   * @brief Compute the measurement covariance matrix.
   *
   * Order is defined by the provided \p axes parameter. This covariance() function deviates from
   * all other currently implemented constraints in that the order does _not_ match the order
   * defined in the variable.
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
  std::vector<Euler> axes_;  //!< Which Euler angle axes we want to measure

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
    archive & axes_;
  }
};

}  // namespace fuse_constraints

BOOST_CLASS_EXPORT_KEY(fuse_constraints::AbsoluteOrientation3DStampedEulerConstraint);

#endif  // FUSE_CONSTRAINTS__ABSOLUTE_ORIENTATION_3D_STAMPED_EULER_CONSTRAINT_HPP_
