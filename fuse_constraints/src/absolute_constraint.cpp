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
#include <fuse_constraints/absolute_constraint.h>

#include <fuse_constraints/normal_prior_orientation_2d.h>
#include <pluginlib/class_list_macros.h>

#include <boost/serialization/export.hpp>

#include <string>


namespace fuse_constraints
{

// Specialization for Orientation2D
// We need to handle the 2*pi rollover for 2D orientations, so simple subtraction does not produce the correct cost
template<>
ceres::CostFunction* AbsoluteConstraint<fuse_variables::Orientation2DStamped>::costFunction() const
{
  return new NormalPriorOrientation2D(sqrt_information_(0, 0), mean_(0));
}

// Specialize the type() method to return the name that is registered with the plugins
template<>
std::string AbsoluteConstraint<fuse_variables::AccelerationAngular2DStamped>::type() const
{
  return "AbsoluteAccelerationAngular2DStampedConstraint";
}

template<>
std::string AbsoluteConstraint<fuse_variables::AccelerationLinear2DStamped>::type() const
{
  return "AbsoluteAccelerationLinear2DStampedConstraint";
}

template<>
std::string AbsoluteConstraint<fuse_variables::Orientation2DStamped>::type() const
{
  return "AbsoluteOrientation2DStampedConstraint";
}

template<>
std::string AbsoluteConstraint<fuse_variables::Position2DStamped>::type() const
{
  return "AbsolutePosition2DStampedConstraint";
}

template<>
std::string AbsoluteConstraint<fuse_variables::Position3DStamped>::type() const
{
  return "AbsolutePosition3DStampedConstraint";
}

template<>
std::string AbsoluteConstraint<fuse_variables::VelocityAngular2DStamped>::type() const
{
  return "AbsoluteVelocityAngular2DStampedConstraint";
}

template<>
std::string AbsoluteConstraint<fuse_variables::VelocityLinear2DStamped>::type() const
{
  return "AbsoluteVelocityLinear2DStampedConstraint";
}

}  // namespace fuse_constraints

BOOST_CLASS_EXPORT_IMPLEMENT(fuse_constraints::AbsoluteAccelerationAngular2DStampedConstraint);
BOOST_CLASS_EXPORT_IMPLEMENT(fuse_constraints::AbsoluteAccelerationLinear2DStampedConstraint);
BOOST_CLASS_EXPORT_IMPLEMENT(fuse_constraints::AbsoluteOrientation2DStampedConstraint);
BOOST_CLASS_EXPORT_IMPLEMENT(fuse_constraints::AbsolutePosition2DStampedConstraint);
BOOST_CLASS_EXPORT_IMPLEMENT(fuse_constraints::AbsolutePosition3DStampedConstraint);
BOOST_CLASS_EXPORT_IMPLEMENT(fuse_constraints::AbsoluteVelocityAngular2DStampedConstraint);
BOOST_CLASS_EXPORT_IMPLEMENT(fuse_constraints::AbsoluteVelocityLinear2DStampedConstraint);

PLUGINLIB_EXPORT_CLASS(fuse_constraints::AbsoluteAccelerationAngular2DStampedConstraint, fuse_core::Constraint);
PLUGINLIB_EXPORT_CLASS(fuse_constraints::AbsoluteAccelerationLinear2DStampedConstraint, fuse_core::Constraint);
PLUGINLIB_EXPORT_CLASS(fuse_constraints::AbsoluteOrientation2DStampedConstraint, fuse_core::Constraint);
PLUGINLIB_EXPORT_CLASS(fuse_constraints::AbsolutePosition2DStampedConstraint, fuse_core::Constraint);
PLUGINLIB_EXPORT_CLASS(fuse_constraints::AbsolutePosition3DStampedConstraint, fuse_core::Constraint);
PLUGINLIB_EXPORT_CLASS(fuse_constraints::AbsoluteVelocityAngular2DStampedConstraint, fuse_core::Constraint);
PLUGINLIB_EXPORT_CLASS(fuse_constraints::AbsoluteVelocityLinear2DStampedConstraint, fuse_core::Constraint);
