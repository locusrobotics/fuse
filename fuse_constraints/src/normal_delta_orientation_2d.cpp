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
#include <fuse_constraints/normal_delta_orientation_2d.h>
#include <fuse_constraints/util.h>


namespace fuse_constraints
{

NormalDeltaOrientation2D::NormalDeltaOrientation2D(const double A, const double b) :
  A_(A),
  b_(b)
{
}

bool NormalDeltaOrientation2D::Evaluate(
  double const* const* parameters,
  double* residuals,
  double** jacobians) const
{
  // The following lines should read as
  // r = A_ * ((x1 - x0) - b_);
  // The wrap function handles the 2_pi wrap around issue with rotations
  double angle_diff = parameters[1][0] - parameters[0][0] - b_;
  wrapAngle2D(angle_diff);
  residuals[0] = A_ * angle_diff;
  if (jacobians != NULL)
  {
    if (jacobians[0] != NULL)
    {
      jacobians[0][0] = -A_;
    }
    if (jacobians[1] != NULL)
    {
      jacobians[1][0] = A_;
    }
  }
  return true;
}

}  // namespace fuse_constraints
