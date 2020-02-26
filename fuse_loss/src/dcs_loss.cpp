/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Clearpath Robotics
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
#include <fuse_loss/dcs_loss.h>
#include <fuse_loss/loss_function.h>

#include <pluginlib/class_list_macros.h>
#include <ros/node_handle.h>

#include <boost/serialization/export.hpp>

#include <ostream>
#include <string>


namespace fuse_loss
{

DCSLoss::DCSLoss(const double a) : a_(a)
{
}

void DCSLoss::initialize(const std::string& name)
{
  ros::NodeHandle private_node_handle(name);

  private_node_handle.param("a", a_, a_);
}

void DCSLoss::print(std::ostream& stream) const
{
  stream << type() << "\n"
         << "  a: " << a_ << "\n";
}

ceres::LossFunction* DCSLoss::lossFunction() const
{
  return new ceres::DCSLoss(a_);
}

}  // namespace fuse_loss

BOOST_CLASS_EXPORT_IMPLEMENT(fuse_loss::DCSLoss);
PLUGINLIB_EXPORT_CLASS(fuse_loss::DCSLoss, fuse_core::Loss);
