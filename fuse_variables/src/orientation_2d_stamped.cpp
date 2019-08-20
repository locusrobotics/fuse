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
#include <fuse_variables/orientation_2d_stamped.h>

#include <fuse_core/local_parameterization.h>
#include <fuse_core/util.h>
#include <fuse_core/uuid.h>
#include <fuse_variables/fixed_size_variable.h>
#include <fuse_variables/stamped.h>
#include <ros/time.h>

#include <boost/serialization/export.hpp>

#include <ostream>


namespace fuse_variables
{

/**
 * @brief A LocalParameterization class for 2D Orientations.
 *
 * 2D orientations add and subtract in the "usual" way, except for the 2*pi rollover issue. This local parameterization
 * handles the rollover. Because the Jacobians for this parameterization are always identity, we implement this
 * parameterization with "analytic" derivatives, instead of using the Ceres's autodiff system.
 */
class Orientation2DLocalParameterization : public fuse_core::LocalParameterization
{
public:
  int GlobalSize() const override
  {
    return 1;
  }

  int LocalSize() const override
  {
    return 1;
  }

  bool Plus(
    const double* x,
    const double* delta,
    double* x_plus_delta) const override
  {
    // Compute the angle increment as a linear update, and handle the 2*Pi rollover
    x_plus_delta[0] = fuse_core::wrapAngle2D(x[0] + delta[0]);
    return true;
  }

  bool ComputeJacobian(
    const double* /*x*/,
    double* jacobian) const override
  {
    jacobian[0] = 1.0;
    return true;
  }

  bool Minus(
    const double* x1,
    const double* x2,
    double* delta) const override
  {
    // Compute the difference from x2 to x1, and handle the 2*Pi rollover
    delta[0] = fuse_core::wrapAngle2D(x2[0] - x1[0]);
    return true;
  }

  bool ComputeMinusJacobian(
    const double* /*x*/,
    double* jacobian) const override
  {
    jacobian[0] = 1.0;
    return true;
  }
};

Orientation2DStamped::Orientation2DStamped(const ros::Time& stamp, const fuse_core::UUID& device_id) :
  FixedSizeVariable(fuse_core::uuid::generate(detail::type(), stamp, device_id)),
  Stamped(stamp, device_id)
{
}

void Orientation2DStamped::print(std::ostream& stream) const
{
  stream << type() << ":\n"
         << "  uuid: " << uuid() << "\n"
         << "  stamp: " << stamp() << "\n"
         << "  device_id: " << deviceId() << "\n"
         << "  size: " << size() << "\n"
         << "  data:\n"
         << "  - yaw: " << yaw() << "\n";
}

fuse_core::LocalParameterization* Orientation2DStamped::localParameterization() const
{
  return new Orientation2DLocalParameterization();
}

}  // namespace fuse_variables

BOOST_CLASS_EXPORT_IMPLEMENT(fuse_variables::Orientation2DStamped);
