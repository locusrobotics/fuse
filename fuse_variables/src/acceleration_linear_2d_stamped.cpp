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
#include <fuse_variables/acceleration_linear_2d_stamped.h>


namespace fuse_variables
{

AccelerationLinear2DStamped::AccelerationLinear2DStamped() :
  hardware_id_(fuse_core::uuid::NIL),
  stamp_(0, 0),
  uuid_(fuse_core::uuid::NIL)
{
}

AccelerationLinear2DStamped::AccelerationLinear2DStamped(const ros::Time& stamp, const fuse_core::UUID& hardware_id) :
  hardware_id_(hardware_id),
  stamp_(stamp),
  uuid_(fuse_core::uuid::generate(type(), stamp, hardware_id))
{
}

void AccelerationLinear2DStamped::print(std::ostream& stream) const
{
  stream << type() << ":\n"
         << "  uuid: " << uuid() << "\n"
         << "  stamp: " << stamp() << "\n"
         << "  hardware_id: " << hardwareId() << "\n"
         << "  size: " << size() << "\n"
         << "  data:\n"
         << "  - ax: " << ax() << "\n"
         << "  - ay: " << ay() << "\n";
}

fuse_core::Variable::UniquePtr AccelerationLinear2DStamped::clone() const
{
  return AccelerationLinear2DStamped::make_unique(*this);
}

}  // namespace fuse_variables
