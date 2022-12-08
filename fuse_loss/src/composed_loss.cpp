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
#include <fuse_loss/composed_loss.hpp>
#include <fuse_loss/trivial_loss.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace fuse_loss
{

ComposedLoss::ComposedLoss(
  const std::shared_ptr<fuse_core::Loss> & f_loss,
  const std::shared_ptr<fuse_core::Loss> & g_loss)
: f_loss_(f_loss), g_loss_(g_loss)
{
}

void ComposedLoss::initialize(
  fuse_core::node_interfaces::NodeInterfaces<
    fuse_core::node_interfaces::Base,
    fuse_core::node_interfaces::Logging,
    fuse_core::node_interfaces::Parameters
  > interfaces,
  const std::string & name)
{
  f_loss_ = fuse_core::loadLossConfig(interfaces, name + ".f_loss");
  g_loss_ = fuse_core::loadLossConfig(interfaces, name + ".g_loss");
}

void ComposedLoss::print(std::ostream & stream) const
{
  stream << type() << "\n";

  if (f_loss_) {
    stream << "  f_loss: " << f_loss_ << "\n";
  }

  if (g_loss_) {
    stream << "  g_loss: " << g_loss_ << "\n";
  }
}

ceres::LossFunction * ComposedLoss::lossFunction() const
{
  return new ceres::ComposedLoss(
    f_loss_ ? f_loss_->lossFunction() : TrivialLoss().lossFunction(), Ownership,
    g_loss_ ? g_loss_->lossFunction() : TrivialLoss().lossFunction(), Ownership);
}

}  // namespace fuse_loss

BOOST_CLASS_EXPORT_IMPLEMENT(fuse_loss::ComposedLoss);
PLUGINLIB_EXPORT_CLASS(fuse_loss::ComposedLoss, fuse_core::Loss);
