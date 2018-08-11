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
#include <fuse_variables/orientation_2d_stamped.h>

#include <boost/core/demangle.hpp>
#include <ceres/autodiff_local_parameterization.h>

#include <string>


namespace fuse_variables
{

const std::string Orientation2DStamped::TYPE = boost::core::demangle(typeid(Orientation2DStamped).name());

Orientation2DStamped::Orientation2DStamped(const ros::Time& stamp, const fuse_core::UUID& device_id) :
  Stamped(stamp, device_id),
  uuid_(fuse_core::uuid::generate(type(), stamp, device_id))
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

fuse_core::Variable::UniquePtr Orientation2DStamped::clone() const
{
  return Orientation2DStamped::make_unique(*this);
}

ceres::LocalParameterization* Orientation2DStamped::localParameterization() const
{
  return new ceres::AutoDiffLocalParameterization<Orientation2DPlus, 1, 1>();
}

}  // namespace fuse_variables
