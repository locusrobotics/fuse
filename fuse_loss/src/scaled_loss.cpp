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
#include <memory>
#include <ostream>
#include <string>

#include <boost/serialization/export.hpp>
#include <fuse_core/parameter.hpp>
#include <fuse_loss/scaled_loss.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace fuse_loss
{

ScaledLoss::ScaledLoss(const double a, const std::shared_ptr<fuse_core::Loss> & loss)
: a_(a), loss_(loss)
{
}

void ScaledLoss::initialize(
  fuse_core::node_interfaces::NodeInterfaces<
    fuse_core::node_interfaces::Base,
    fuse_core::node_interfaces::Logging,
    fuse_core::node_interfaces::Parameters
  > interfaces,
  const std::string & name)
{
  a_ = fuse_core::getParam(interfaces, name + ".a", a_);
  loss_ = fuse_core::loadLossConfig(interfaces, name + ".loss");
}

void ScaledLoss::print(std::ostream & stream) const
{
  stream << type() << "\n"
         << "  a: " << a_ << "\n";

  if (loss_) {
    stream << "  loss: " << loss_ << "\n";
  }
}

ceres::LossFunction * ScaledLoss::lossFunction() const
{
  return new ceres::ScaledLoss(loss_ ? loss_->lossFunction() : nullptr, a_, Ownership);
}

}  // namespace fuse_loss

BOOST_CLASS_EXPORT_IMPLEMENT(fuse_loss::ScaledLoss);
PLUGINLIB_EXPORT_CLASS(fuse_loss::ScaledLoss, fuse_core::Loss);
