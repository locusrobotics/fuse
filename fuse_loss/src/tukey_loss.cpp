/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Clearpath Robotics
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
#include <string>

#include <boost/serialization/export.hpp>
#include <fuse_core/ceres_macros.hpp>
#include <fuse_core/parameter.hpp>
#include <fuse_loss/tukey_loss.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace fuse_loss
{

TukeyLoss::TukeyLoss(const double a)
: a_(a)
{
}

void TukeyLoss::initialize(
  fuse_core::node_interfaces::NodeInterfaces<
    fuse_core::node_interfaces::Base,
    fuse_core::node_interfaces::Logging,
    fuse_core::node_interfaces::Parameters
  > interfaces,
  const std::string & name)
{
  a_ = fuse_core::getParam(interfaces, name + ".a", a_);
}

void TukeyLoss::print(std::ostream & stream) const
{
  stream << type() << "\n"
         << "  a: " << a_ << "\n";
}

ceres::LossFunction * TukeyLoss::lossFunction() const
{
#if CERES_VERSION_AT_LEAST(2, 0, 0)
  return new ceres::TukeyLoss(a_);
#else
  // The Tukey loss function is incorrectly implemented in Ceres before the 2.* version because it
  // must be multiplied by 2, so instead of dividing by 6 it should have divided by 3 in:
  //
  //   https://github.com/ceres-solver/ceres-
  //   solver/blob/4362a2169966e0839425209/include/ceres/loss_function.h#L281-L282
  //
  // This was fix with the patch already sent and merged into Ceres in this PR:
  //
  //   https://ceres-solver-review.googlesource.com/c/ceres-solver/+/16700
  //
  // See:
  //
  //   https://github.com/ceres-solver/ceres-
  //   solver/commit/6da364713f5b78#diff-7f75c0abfe2c5756f744aa61097ca1c2L281-R283
  //
  // There is an easy workaround for this. We combine TukeyLoss with ScaledLoss, using a scaled
  // factor of 2.
  return new ceres::ScaledLoss(new ceres::TukeyLoss(a_), 2.0, Ownership);
#endif
}

}  // namespace fuse_loss

BOOST_CLASS_EXPORT_IMPLEMENT(fuse_loss::TukeyLoss);
PLUGINLIB_EXPORT_CLASS(fuse_loss::TukeyLoss, fuse_core::Loss);
