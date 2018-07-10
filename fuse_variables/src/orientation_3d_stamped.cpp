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
#include <fuse_core/uuid.h>
#include <fuse_variables/orientation_3d_stamped.h>

#include <ceres/local_parameterization.h>

#include <cmath>

namespace fuse_variables
{

Orientation3DStamped::Orientation3DStamped(const ros::Time& stamp, const fuse_core::UUID &hardware_id) :
  hardware_id_(hardware_id),
  stamp_(stamp),
  uuid_(fuse_core::uuid::generate(type(), stamp_, hardware_id_))
{
}

double Orientation3DStamped::pitch()
{
  // Adapted from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  const double sin_pitch = 2.0 * (w() * y() - z() * x());

  if (std::abs(sin_pitch) >= 1.0)
  {
    return std::copysign(M_PI / 2.0, sin_pitch);
  }
  else
  {
    return ::asin(sin_pitch);
  }
}

double Orientation3DStamped::roll()
{
  // Adapted from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  const double sin_roll = 2.0 * (w() * x() + y() * z());
  const double cos_roll = 1.0 - (2.0 * (x() * x() + y() * y()));
  return ::atan2(sin_roll, cos_roll);
}

double Orientation3DStamped::yaw()
{
  // Adapted from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  const double sin_yaw = 2.0 * (w() * z() + x() * y());
  const double cos_yaw = 1.0 - (2.0 * (y() * y() + z() * z()));
  return ::atan2(sin_yaw, cos_yaw);
}

void Orientation3DStamped::print(std::ostream& stream) const
{
  stream << type() << ":\n"
         << "  uuid: " << uuid() << "\n"
         << "  hardware id: " << hardware_id() << "\n"
         << "  stamp: " << stamp() << "\n"
         << "  size: " << size() << "\n"
         << "  data:\n"
         << "  - w: " << w() << "\n"
         << "  - x: " << x() << "\n"
         << "  - y: " << y() << "\n"
         << "  - z: " << z() << "\n";
}

fuse_core::Variable::UniquePtr Orientation3DStamped::clone() const
{
  return Orientation3DStamped::make_unique(*this);
}

ceres::LocalParameterization* Orientation3DStamped::localParameterization() const
{
  return new ceres::QuaternionParameterization();
}

}  // namespace fuse_variables
