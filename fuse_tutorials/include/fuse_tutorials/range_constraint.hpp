/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Locus Robotics
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
#ifndef FUSE_TUTORIALS__RANGE_CONSTRAINT_HPP_
#define FUSE_TUTORIALS__RANGE_CONSTRAINT_HPP_

#include <ceres/cost_function.h>

#include <ostream>
#include <string>

#include <fuse_core/constraint.hpp>
#include <fuse_core/fuse_macros.hpp>
#include <fuse_variables/point_2d_landmark.hpp>
#include <fuse_variables/position_2d_stamped.hpp>

#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>


namespace fuse_tutorials
{
/**
 * @brief Implements a range-only measurement constraint between the robot and a beacon.
 *
 * The main purpose for this constraint is to demonstrate how to write your own cost constraint
 * classes.
 *
 * For the purposes of this tutorial, let's imagine that you have developed some new robotic sensor
 * that is capable of measuring the distance to some sort of beacon, but does not provide any
 * information about the bearing/heading to that beacon. None of the fuse packages provide such a
 * sensor model, so you need to develop one yourself.
 *
 * The "sensor model" class provides an interface to ROS, allowing sensor messages to be received.
 * The sensor model class also acts as a "factory" (in a programming sense) that creates new sensor
 * constraints for each received sensor measurement. The constraint class does a number of things:
 *  - Represents a single measurement, and likely holds the actual measured value
 *  - Defines exactly which variable instances are involved in this measurement model
 *  - Defines any robust loss function that should be applied to the computed costs
 *  - Generates instances of a Ceres Solver cost function that implements the desired sensor
 *    measurement model
 *
 * This range-only constraint will use the mathematical model defined in the RangeCostFunctor, and
 * Ceres Solver's automatic differentiation system to create a Ceres Solver cost function.
 */
class RangeConstraint : public fuse_core::Constraint
{
public:
  // There are some boilerplate types and functions that must be implemented for each constraint,
  // such as shared pointer typedefs and clone() methods. These are formulaic, but the derived type
  // is needed for their proper implementation. A few different macro options are provided to make
  // implementing this boilerplate code easy.
  FUSE_CONSTRAINT_DEFINITIONS(RangeConstraint)

  /**
   * @brief Default constructor
   *
   * A default constructor is required to support the serialize/deserialize functionality.
   */
  RangeConstraint() = default;

  /**
   * @brief Create a range-only constraint between the provided robot position and the beacon
   *        position
   *
   * This is the constructor that will be used from within the RangeSensorModel. It accepts
   * references to the variables involved with this specific measurement -- the robot position at
   * the time the measurement was sampled, and the beacon that was measured.
   *
   * Note that, when measuring subset of dimensions, empty axis vectors are permitted. This
   * signifies, e.g., that you don't want to measure any of the quantities in that variable.
   *
   * The mean is given as a vector. The first components (if any) will be dictated, both in content
   * and in ordering, by the value of the \p linear_indices. The final component (if any) is
   * dictated by the \p angular_indices. The covariance matrix follows the same ordering.
   *
   * @param[in] source - The name of the sensor that generated this constraint. This is largely
   *                     information to aid in debugging or visualizing the system. If multiple
   *                     sensors of the same type exist, being able to disambiguate the constraints
   *                     from sensor1 versus sensor2 is useful.
   * @param[in] robot_position - The 2D position of the robot at the time the measurement was
   *                             sampled
   * @param[in] beacon_position - The 2D position of the sampled beacon
   * @param[in] z - The distance measured between the robot and beacon by our new sensor
   * @param[in] sigma - The uncertainty of measured distance
   */
  RangeConstraint(
    const std::string & source,
    const fuse_variables::Position2DStamped & robot_position,
    const fuse_variables::Point2DLandmark & beacon_position,
    const double z,
    const double sigma);

  /**
   * @brief Print a human-readable description of the constraint to the provided stream.
   *
   * This is required by the fuse_core::Constraint base class, but is largely just for debugging and
   * visualization.
   *
   * @param[out] stream - The stream to write to. Defaults to stdout.
   */
  void print(std::ostream & stream = std::cout) const override;

  /**
   * @brief Construct an instance of this constraint's cost function
   *
   * This is the most important operation of a Constraint -- to create a Ceres Solver CostFunction
   * object that is then optimized by the Ceres Solver least-squares solver. This implementation
   * uses the RangeCostFunctor and the Ceres Solver AutoDiffCostFunction class to automatically
   * compute the cost function Jacobians. This is also where the sizes of the input variables and
   * output cost array is defined.
   *
   * @return A base pointer to an instance of a derived Ceres Solver CostFunction. Ownership of the
   *         CostFunction object is transferred Ceres Solver; Ceres Solver will delete the
   *         CostFunction object when it is done. Also note that the fuse project guarantees the
   *         derived Constraint object will outlive the Ceres Solver CostFunction object.
   */
  ceres::CostFunction * costFunction() const override;

private:
  // Allow Boost Serialization access to private methods and members
  friend class boost::serialization::access;

  /**
   * @brief The Boost Serialize method that serializes all of the data members in to/out of the
   *        archive
   *
   * Implementing the serialize() function allows the derived Constraint object to be written to
   * disk, and retrieved again at a later date. In particular, the Graph classes make use of this
   * feature, allowing a full graph to be saved to disk and recalled later. This is extraordinarily
   * useful in debugging and replaying. It is highly recommended that the serialize() method be
   * implemented properly. And for most things, it is trivial to implement. See the Boost
   * Serialization documentation for more details:
   * https://www.boost.org/doc/libs/1_77_0/libs/serialization/doc/index.html
   *
   * @param[in/out] archive - The archive object that holds the serialized class members
   * @param[in] version - The version of the archive being read/written. Generally unused.
   */
  template<class Archive>
  void serialize(Archive & archive, const unsigned int /* version */)
  {
    archive & boost::serialization::base_object<fuse_core::Constraint>(*this);
    archive & sigma_;
    archive & z_;
  }

  double sigma_ {0.0};    //!< The standard deviation of the range measurement
  double z_ {0.0};    //!< The measured range to the beacon
};

}  // namespace fuse_tutorials

// This is part of the serialization requirement. Boost needs to be told this class is serializable.
BOOST_CLASS_EXPORT_KEY(fuse_tutorials::RangeConstraint);

#endif  // FUSE_TUTORIALS__RANGE_CONSTRAINT_HPP_
