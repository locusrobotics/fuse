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
#include <cmath>
#include <limits>

#include <fuse_loss/loss_function.hpp>

namespace ceres
{

void DCSLoss::Evaluate(double s, double rho[3]) const
{
  if (s > a_) {
    // Outlier region
    const double inv = 1.0 / (a_ + s);
    const double scale = 2.0 * a_ * inv;

    rho[0] = a_ * (3.0 * s - a_) * inv;
    rho[1] = scale * scale;
    rho[2] = -2.0 * inv * rho[1];
  } else {
    // Inlier region
    rho[0] = s;
    rho[1] = 1.0;
    rho[2] = 0.0;
  }
}

void FairLoss::Evaluate(double s, double rho[3]) const
{
  const double r = std::sqrt(s);
  const double ra = r / a_;
  const double sum = 1.0 + ra;

  rho[0] = 2.0 * b_ * (ra - std::log(sum));
  rho[1] = 1.0 / sum;
  rho[2] = r == 0.0 ? std::numeric_limits<double>::lowest() : -0.5 / (a_ * r * sum * sum);
}

void GemanMcClureLoss::Evaluate(double s, double rho[3]) const
{
  const double sum = b_ + s;
  const double inv = 1.0 / sum;
  const double scale = b_ * inv;

  rho[0] = s * scale;
  rho[1] = scale * scale;
  rho[2] = -2.0 * inv * rho[1];
}

void WelschLoss::Evaluate(double s, double rho[3]) const
{
  const double exp = std::exp(s * c_);

  rho[0] = b_ * (1 - exp);
  rho[1] = exp;
  rho[2] = c_ * exp;
}

}  // namespace ceres
