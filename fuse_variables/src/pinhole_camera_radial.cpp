/*
 * Software License Agreement (BSD License)
 *
 *  Author:    Oscar Mendez
 *  Created:   11.13.2023
 *
 *  Copyright (c) 2023, Locus Robotics
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
#include <fuse_variables/pinhole_camera_radial.h>

#include <fuse_core/uuid.h>
#include <fuse_core/variable.h>
#include <fuse_variables/fixed_size_variable.h>
#include <pluginlib/class_list_macros.hpp>

#include <boost/serialization/export.hpp>

#include <ostream>

namespace fuse_variables
{
PinholeCameraRadial::PinholeCameraRadial(const fuse_core::UUID& uuid, const uint64_t& camera_id)
  : BaseCamera(uuid, camera_id)
{
}

PinholeCameraRadial::PinholeCameraRadial(const uint64_t& camera_id)
  : PinholeCameraRadial(fuse_core::uuid::generate(detail::type(), camera_id), camera_id)
{
}

PinholeCameraRadial::PinholeCameraRadial(const fuse_core::UUID& uuid, const uint64_t& camera_id,
                              const double& f, const double& r1, const double& r2)
  : PinholeCameraRadial(fuse_core::uuid::generate(detail::type(), camera_id), camera_id)
{
  data_[F] = f;
  data_[R1] = r1;
  data_[R2] = r2;
}

void PinholeCameraRadial::print(std::ostream& stream) const
{
  stream << type() << ":\n"
         << "  uuid: " << uuid() << "\n"
         << "  size: " << size() << "\n"
         << "  camera id: " << id() << "\n"
         << "  data:\n"
         << "  - f: " << f() << "\n"
         << "  - r1: " << r1() << "\n"
         << "  - r2: " << r2() << "\n";
}

}  // namespace fuse_variables

BOOST_CLASS_EXPORT_IMPLEMENT(fuse_variables::PinholeCameraRadial);
PLUGINLIB_EXPORT_CLASS(fuse_variables::PinholeCameraRadial, fuse_core::Variable);
