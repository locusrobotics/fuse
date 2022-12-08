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
#include <ostream>

#include <boost/serialization/export.hpp>
#include <fuse_core/local_parameterization.hpp>
#include <fuse_core/time.hpp>
#include <fuse_core/uuid.hpp>
#include <fuse_variables/fixed_size_variable.hpp>
#include <fuse_variables/orientation_3d_stamped.hpp>
#include <fuse_variables/stamped.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace fuse_variables
{

Orientation3DStamped::Orientation3DStamped(
  const rclcpp::Time & stamp,
  const fuse_core::UUID & device_id)
: FixedSizeVariable<4>(fuse_core::uuid::generate(detail::type(), stamp, device_id)),
  Stamped(stamp, device_id)
{
}

void Orientation3DStamped::print(std::ostream & stream) const
{
  stream << type() << ":\n"
         << "  uuid: " << uuid() << "\n"
         << "  device_id: " << deviceId() << "\n"
         << "  stamp: " << stamp().nanoseconds() << "\n"
         << "  size: " << size() << "\n"
         << "  data:\n"
         << "  - w: " << w() << "\n"
         << "  - x: " << x() << "\n"
         << "  - y: " << y() << "\n"
         << "  - z: " << z() << "\n";
}

fuse_core::LocalParameterization * Orientation3DStamped::localParameterization() const
{
  return new Orientation3DLocalParameterization();
}

}  // namespace fuse_variables

BOOST_CLASS_EXPORT_IMPLEMENT(fuse_variables::Orientation3DLocalParameterization);
BOOST_CLASS_EXPORT_IMPLEMENT(fuse_variables::Orientation3DStamped);
PLUGINLIB_EXPORT_CLASS(fuse_variables::Orientation3DStamped, fuse_core::Variable);
