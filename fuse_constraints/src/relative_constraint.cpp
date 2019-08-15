/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Locus Robotics
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
#include <fuse_constraints/relative_constraint.h>

#include <fuse_constraints/normal_delta_orientation_2d.h>
#include <pluginlib/class_list_macros.h>

#include <boost/serialization/export.hpp>

#include <string>


namespace fuse_constraints
{

// Specialization for Orientation2D
template<>
ceres::CostFunction* RelativeConstraint<fuse_variables::Orientation2DStamped>::costFunction() const
{
  // Create a Gaussian/Normal Delta constraint
  return new NormalDeltaOrientation2D(sqrt_information_(0, 0), delta_(0));
}

// Specialize the type() method to return the name that is registered with the plugins
template<>
std::string RelativeConstraint<fuse_variables::AccelerationAngular2DStamped>::type() const
{
  return "RelativeAccelerationAngular2DStampedConstraint";
}

template<>
std::string RelativeConstraint<fuse_variables::AccelerationLinear2DStamped>::type() const
{
  return "RelativeAccelerationLinear2DStampedConstraint";
}

template<>
std::string RelativeConstraint<fuse_variables::Orientation2DStamped>::type() const
{
  return "RelativeOrientation2DStampedConstraint";
}

template<>
std::string RelativeConstraint<fuse_variables::Position2DStamped>::type() const
{
  return "RelativePosition2DStampedConstraint";
}

template<>
std::string RelativeConstraint<fuse_variables::Position3DStamped>::type() const
{
  return "RelativePosition3DStampedConstraint";
}

template<>
std::string RelativeConstraint<fuse_variables::VelocityAngular2DStamped>::type() const
{
  return "RelativeVelocityAngular2DStampedConstraint";
}

template<>
std::string RelativeConstraint<fuse_variables::VelocityLinear2DStamped>::type() const
{
  return "RelativeVelocityLinear2DStampedConstraint";
}

}  // namespace fuse_constraints

BOOST_CLASS_EXPORT_IMPLEMENT(fuse_constraints::RelativeAccelerationAngular2DStampedConstraint);
BOOST_CLASS_EXPORT_IMPLEMENT(fuse_constraints::RelativeAccelerationLinear2DStampedConstraint);
BOOST_CLASS_EXPORT_IMPLEMENT(fuse_constraints::RelativeOrientation2DStampedConstraint);
BOOST_CLASS_EXPORT_IMPLEMENT(fuse_constraints::RelativePosition2DStampedConstraint);
BOOST_CLASS_EXPORT_IMPLEMENT(fuse_constraints::RelativePosition3DStampedConstraint);
BOOST_CLASS_EXPORT_IMPLEMENT(fuse_constraints::RelativeVelocityAngular2DStampedConstraint);
BOOST_CLASS_EXPORT_IMPLEMENT(fuse_constraints::RelativeVelocityLinear2DStampedConstraint);

PLUGINLIB_EXPORT_CLASS(fuse_constraints::RelativeAccelerationAngular2DStampedConstraint, fuse_core::Constraint);
PLUGINLIB_EXPORT_CLASS(fuse_constraints::RelativeAccelerationLinear2DStampedConstraint, fuse_core::Constraint);
PLUGINLIB_EXPORT_CLASS(fuse_constraints::RelativeOrientation2DStampedConstraint, fuse_core::Constraint);
PLUGINLIB_EXPORT_CLASS(fuse_constraints::RelativePosition2DStampedConstraint, fuse_core::Constraint);
PLUGINLIB_EXPORT_CLASS(fuse_constraints::RelativePosition3DStampedConstraint, fuse_core::Constraint);
PLUGINLIB_EXPORT_CLASS(fuse_constraints::RelativeVelocityAngular2DStampedConstraint, fuse_core::Constraint);
PLUGINLIB_EXPORT_CLASS(fuse_constraints::RelativeVelocityLinear2DStampedConstraint, fuse_core::Constraint);
