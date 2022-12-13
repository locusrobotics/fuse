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
#ifndef FUSE_CONSTRAINTS__ABSOLUTE_POSE_2D_STAMPED_CONSTRAINT_HPP_
#define FUSE_CONSTRAINTS__ABSOLUTE_POSE_2D_STAMPED_CONSTRAINT_HPP_

#include <Eigen/Dense>

#include <ostream>
#include <string>
#include <vector>

#include <fuse_core/constraint.hpp>
#include <fuse_core/eigen.hpp>
#include <fuse_core/fuse_macros.hpp>
#include <fuse_core/serialization.hpp>
#include <fuse_core/uuid.hpp>
#include <fuse_variables/orientation_2d_stamped.hpp>
#include <fuse_variables/position_2d_stamped.hpp>

#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>


namespace fuse_constraints
{

/**
 * @brief A constraint that represents either prior information about a 2D pose, or a direct
 *        measurement of the 2D pose.
 *
 * A 2D pose is the combination of a 2D position and a 2D orientation variable. As a convenience,
 * this class applies an absolute constraint on both variables at once. This type of constraint
 * arises in many situations. In mapping it is common to define the very first pose as the origin.
 * Some sensors, such as GPS, provide direct measurements of the robot's pose in the global frame.
 * And localization systems often match laserscans to a prior map (scan-to-map measurements). This
 * constraint holds the measured 2D pose and the measurement uncertainty/covariance. It also permits
 * measurement of a subset of the pose provided in the position and orientation varables.
 */
class AbsolutePose2DStampedConstraint : public fuse_core::Constraint
{
public:
  FUSE_CONSTRAINT_DEFINITIONS(AbsolutePose2DStampedConstraint)

  /**
   * @brief Default constructor
   */
  AbsolutePose2DStampedConstraint() = default;

  /**
   * @brief Create a constraint using a measurement/prior of the 2D pose
   *
   * Note that, when measuring subset of dimensions, empty axis vectors are permitted. This
   * signifies, e.g., that you don't want to measure any of the quantities in that variable.
   *
   * The mean is given as a vector. The first components (if any) will be dictated, both in content
   * and in ordering, by the value of the \p linear_indices. The final component (if any) is
   * dictated by the \p angular_indices. The covariance matrix follows the same ordering.
   *
   * @param[in] source             The name of the sensor or motion model that generated this
   *                               constraint
   * @param[in] position           The variable representing the position components of the pose
   * @param[in] orientation        The variable representing the orientation components of the pose
   * @param[in] partial_mean       The measured/prior pose as a vector (max 3x1 vector, components
   *                               are dictated by \p linear_indices and \p angular_indices)
   * @param[in] partial_covariance The measurement/prior covariance (max 3x3 matrix, components are
   *                               dictated by \p linear_indices and \p angular_indices)
   * @param[in] linear_indices     The set of indices corresponding to the measured position
   *                               dimensions e.g. "{fuse_variables::Position2DStamped::X,
   *                               fuse_variables::Position2DStamped::Y}"
   * @param[in] angular_indices    The set of indices corresponding to the measured orientation
   *                               dimensions e.g. "{fuse_variables::Orientation2DStamped::Yaw}"
   */
  AbsolutePose2DStampedConstraint(
    const std::string & source,
    const fuse_variables::Position2DStamped & position,
    const fuse_variables::Orientation2DStamped & orientation,
    const fuse_core::VectorXd & partial_mean,
    const fuse_core::MatrixXd & partial_covariance,
    const std::vector<size_t> & linear_indices =
    {fuse_variables::Position2DStamped::X, fuse_variables::Position2DStamped::Y},               // NOLINT
    const std::vector<size_t> & angular_indices = {fuse_variables::Orientation2DStamped::YAW});  // NOLINT

  /**
   * @brief Destructor
   */
  virtual ~AbsolutePose2DStampedConstraint() = default;

  /**
   * @brief Read-only access to the measured/prior vector of mean values.
   *
   * Order is (x, y, yaw). Note that the returned vector will be full sized (3x1) and in the stated
   * order.
   */
  const fuse_core::Vector3d & mean() const {return mean_;}

  /**
   * @brief Read-only access to the square root information matrix.
   *
   * If only a partial covariance matrix was provided in the constructor, this covariance matrix
   * will not be square.
   */
  const fuse_core::MatrixXd & sqrtInformation() const {return sqrt_information_;}

  /**
   * @brief Compute the measurement covariance matrix.
   *
   * Order is (x, y, yaw). Note that the returned covariance matrix will be full sized (3x3) and in
   * the stated order. If only a partial covariance matrix was provided in the constructor, this
   * covariance matrix may be a different size and in a different order than the constructor input.
   */
  fuse_core::Matrix3d covariance() const;

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
  fuse_core::Vector3d mean_;  //!< The measured/prior mean vector for this variable
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

}  // namespace fuse_constraints

BOOST_CLASS_EXPORT_KEY(fuse_constraints::AbsolutePose2DStampedConstraint);

#endif  // FUSE_CONSTRAINTS__ABSOLUTE_POSE_2D_STAMPED_CONSTRAINT_HPP_
