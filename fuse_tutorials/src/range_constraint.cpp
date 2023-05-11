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
#include <fuse_tutorials/range_constraint.h>

#include <fuse_tutorials/range_cost_functor.h>
#include <pluginlib/class_list_macros.hpp>

#include <boost/serialization/export.hpp>
#include <ceres/autodiff_cost_function.h>

#include <ostream>
#include <string>

namespace fuse_tutorials
{
// The Constraint base class holds the list of involved Variable UUIDs. When constructing a new RangeConstraint
// object, the base class constructor must be provided with the list of variable UUIDs. Note that the order the
// variables are added to the list is important. Later, when Ceres Solver uses the CostFunction function to minimize
// the total error, it will provide access to the variables *in the same order we provide them to the base class
// constructor*. This means that the variable order defined in the RangeCostFunctor must match the variable order
// provided to the base class Constraint constructor. In this case, robot position, then the beacon position
//   fuse_core::Constraint(source, { robot_position.uuid(), beacon_position.uuid() })
RangeConstraint::RangeConstraint(
  const std::string& source,
  const fuse_variables::Position2DStamped& robot_position,
  const fuse_variables::Point2DLandmark& beacon_position,
  const double z,
  const double sigma) :
  fuse_core::Constraint(source, { robot_position.uuid(), beacon_position.uuid() }),  // NOLINT(whitespace/braces)
  sigma_(sigma),
  z_(z)
{
}

void RangeConstraint::print(std::ostream& stream) const
{
  stream << type() << "\n"
         << "  source: " << source() << "\n"
         << "  uuid: " << uuid() << "\n"
         << "  robot position variable: " << variables().at(0) << "\n"
         << "  beacon position variable: " << variables().at(1) << "\n"
         << "  range measurement: " << z_ << "\n"
         << "  range sigma: " << sigma_ << "\n";
}

ceres::CostFunction* RangeConstraint::costFunction() const
{
  // Here we use the Ceres Solver AutoDiffCostFunction class to generate a derived CostFunction object from our
  // RangeCostFunctor. The AutoDiffCostFunction requires the cost functor as the first template parameter. The following
  // template parameters provide size information:
  //   2nd: The size of the output residuals array of the cost functor. Our functor only computes a single distance
  //        error, so the size is 1.
  //   3rd: The size of the first involved variable. This the robot position (x, y), so the size is 2.
  //   4th: The size of the second involved variable. This the beacon position (x, y), so the size is also 2.
  // If there were additional involved variables, the size of each variable would appear here in order.
  return new ceres::AutoDiffCostFunction<RangeCostFunctor, 1, 2, 2>(new RangeCostFunctor(z_, sigma_));
}

}  // namespace fuse_tutorials

// This is part of the serialization requirement. Boost needs to be told this class is serializable.
BOOST_CLASS_EXPORT_IMPLEMENT(fuse_tutorials::RangeConstraint);
// Additionally we tell pluginlib about this class. This makes it loadable at runtime, if needed.
PLUGINLIB_EXPORT_CLASS(fuse_tutorials::RangeConstraint, fuse_core::Constraint);
