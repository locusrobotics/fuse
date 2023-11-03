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
#include <boost/serialization/export.hpp>
#include <fuse_constraints/absolute_constraint.hpp>
#include <pluginlib/class_list_macros.hpp>

BOOST_CLASS_EXPORT_IMPLEMENT(fuse_constraints::AbsoluteAccelerationAngular2DStampedConstraint);
BOOST_CLASS_EXPORT_IMPLEMENT(fuse_constraints::AbsoluteAccelerationAngular3DStampedConstraint);
BOOST_CLASS_EXPORT_IMPLEMENT(fuse_constraints::AbsoluteAccelerationLinear2DStampedConstraint);
BOOST_CLASS_EXPORT_IMPLEMENT(fuse_constraints::AbsoluteAccelerationLinear3DStampedConstraint);
BOOST_CLASS_EXPORT_IMPLEMENT(fuse_constraints::AbsoluteOrientation2DStampedConstraint);
BOOST_CLASS_EXPORT_IMPLEMENT(fuse_constraints::AbsolutePosition2DStampedConstraint);
BOOST_CLASS_EXPORT_IMPLEMENT(fuse_constraints::AbsolutePosition3DStampedConstraint);
BOOST_CLASS_EXPORT_IMPLEMENT(fuse_constraints::AbsoluteVelocityAngular2DStampedConstraint);
BOOST_CLASS_EXPORT_IMPLEMENT(fuse_constraints::AbsoluteVelocityAngular3DStampedConstraint);
BOOST_CLASS_EXPORT_IMPLEMENT(fuse_constraints::AbsoluteVelocityLinear2DStampedConstraint);
BOOST_CLASS_EXPORT_IMPLEMENT(fuse_constraints::AbsoluteVelocityLinear3DStampedConstraint);

PLUGINLIB_EXPORT_CLASS(
  fuse_constraints::AbsoluteAccelerationAngular2DStampedConstraint,
  fuse_core::Constraint);
PLUGINLIB_EXPORT_CLASS(
  fuse_constraints::AbsoluteAccelerationAngular3DStampedConstraint,
  fuse_core::Constraint);
PLUGINLIB_EXPORT_CLASS(
  fuse_constraints::AbsoluteAccelerationLinear2DStampedConstraint,
  fuse_core::Constraint);
PLUGINLIB_EXPORT_CLASS(
  fuse_constraints::AbsoluteAccelerationLinear3DStampedConstraint,
  fuse_core::Constraint);
PLUGINLIB_EXPORT_CLASS(
  fuse_constraints::AbsoluteOrientation2DStampedConstraint,
  fuse_core::Constraint);
PLUGINLIB_EXPORT_CLASS(
  fuse_constraints::AbsolutePosition2DStampedConstraint,
  fuse_core::Constraint);
PLUGINLIB_EXPORT_CLASS(
  fuse_constraints::AbsolutePosition3DStampedConstraint,
  fuse_core::Constraint);
PLUGINLIB_EXPORT_CLASS(
  fuse_constraints::AbsoluteVelocityAngular2DStampedConstraint,
  fuse_core::Constraint);
PLUGINLIB_EXPORT_CLASS(
  fuse_constraints::AbsoluteVelocityAngular3DStampedConstraint,
  fuse_core::Constraint);
PLUGINLIB_EXPORT_CLASS(
  fuse_constraints::AbsoluteVelocityLinear2DStampedConstraint,
  fuse_core::Constraint);
PLUGINLIB_EXPORT_CLASS(
  fuse_constraints::AbsoluteVelocityLinear3DStampedConstraint,
  fuse_core::Constraint);