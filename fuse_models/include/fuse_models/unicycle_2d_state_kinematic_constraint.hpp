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
#ifndef FUSE_MODELS__UNICYCLE_2D_STATE_KINEMATIC_CONSTRAINT_HPP_
#define FUSE_MODELS__UNICYCLE_2D_STATE_KINEMATIC_CONSTRAINT_HPP_

#include <ostream>
#include <string>
#include <vector>

#include <fuse_core/constraint.hpp>
#include <fuse_core/eigen.hpp>
#include <fuse_core/fuse_macros.hpp>
#include <fuse_core/serialization.hpp>
#include <fuse_core/uuid.hpp>
#include <fuse_variables/acceleration_linear_2d_stamped.hpp>
#include <fuse_variables/orientation_2d_stamped.hpp>
#include <fuse_variables/position_2d_stamped.hpp>
#include <fuse_variables/velocity_angular_2d_stamped.hpp>
#include <fuse_variables/velocity_linear_2d_stamped.hpp>

#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>


namespace fuse_models
{

/**
 * @brief A class that represents a kinematic constraint between 2D states at two different times
 *
 * The fuse_models 2D state is a combination of 2D position, 2D orientation, 2D linear velocity, 2D
 * angular velocity, and 2D linear acceleration.
 */
class Unicycle2DStateKinematicConstraint : public fuse_core::Constraint
{
public:
  FUSE_CONSTRAINT_DEFINITIONS_WITH_EIGEN(Unicycle2DStateKinematicConstraint)

  /**
   * @brief Default constructor
   */
  Unicycle2DStateKinematicConstraint() = default;

  /**
   * @brief Create a constraint using a time delta and a kinematic model cost functor
   *
   * The constraint is created between two states. The state is broken up into multiple fuse
   * variable types.
   *
   * @param[in] source The name of the sensor or motion model that generated this constraint
   * @param[in] position1 Position component variable of the fist state
   * @param[in] yaw1 Yaw component variable of the first state
   * @param[in] linear_velocity1 Linear velocity component variable of the first state
   * @param[in] yaw_velocity1 Yaw velocity component variable of the first state
   * @param[in] linear_acceleration1 Linear acceleration component variable of the first state
   * @param[in] position2 Position component variable of the second state
   * @param[in] yaw2 Yaw component variable of the second state
   * @param[in] linear_velocity2 Linear velocity component variable of the second state
   * @param[in] yaw_velocity2 Yaw velocity component variable of the second state
   * @param[in] linear_acceleration2 Linear acceleration component variable of the second state
   * @param[in] covariance - The covariance matrix used to weight the constraint. Order is (x, y,
   *                         yaw, x_vel, y_vel, yaw_vel, x_acc, y_acc)
   */
  Unicycle2DStateKinematicConstraint(
    const std::string & source,
    const fuse_variables::Position2DStamped & position1,
    const fuse_variables::Orientation2DStamped & yaw1,
    const fuse_variables::VelocityLinear2DStamped & linear_velocity1,
    const fuse_variables::VelocityAngular2DStamped & yaw_velocity1,
    const fuse_variables::AccelerationLinear2DStamped & linear_acceleration1,
    const fuse_variables::Position2DStamped & position2,
    const fuse_variables::Orientation2DStamped & yaw2,
    const fuse_variables::VelocityLinear2DStamped & linear_velocity2,
    const fuse_variables::VelocityAngular2DStamped & yaw_velocity2,
    const fuse_variables::AccelerationLinear2DStamped & linear_acceleration2,
    const fuse_core::Matrix8d & covariance);

  /**
   * @brief Destructor
   */
  virtual ~Unicycle2DStateKinematicConstraint() = default;

  /**
   * @brief Read-only access to the time delta between the first and second state (really, between
   *        the position1 and
   position2 variables in the constructor)
   */
  double dt() const {return dt_;}

  /**
   * @brief Read-only access to the square root information matrix.
   *
   * Order is (x, y, yaw, x_vel, y_vel, yaw_vel, x_acc, y_acc)
   */
  const fuse_core::Matrix8d & sqrtInformation() const {return sqrt_information_;}

  /**
   * @brief Compute the measurement covariance matrix.
   *
   * Order is (x, y, yaw, x_vel, y_vel, yaw_vel, x_acc, y_acc)
   */
  fuse_core::Matrix8d covariance() const
  {
    return (sqrt_information_.transpose() * sqrt_information_).inverse();
  }

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
  double dt_;  //!< The time delta for the constraint
  fuse_core::Matrix8d sqrt_information_;  //!< The square root information matrix

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
    archive & dt_;
    archive & sqrt_information_;
  }
};

}  // namespace fuse_models

BOOST_CLASS_EXPORT_KEY(fuse_models::Unicycle2DStateKinematicConstraint);

#endif  // FUSE_MODELS__UNICYCLE_2D_STATE_KINEMATIC_CONSTRAINT_HPP_
