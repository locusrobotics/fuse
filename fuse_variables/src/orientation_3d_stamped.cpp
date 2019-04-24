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
#include <fuse_variables/orientation_3d_stamped.h>

#include <fuse_core/local_parameterization.h>
#include <fuse_core/uuid.h>
#include <fuse_variables/stamped.h>
#include <ros/time.h>

#include <ceres/rotation.h>

#include <ostream>


namespace
{
/**
 * @brief Create the inverse quaternion
 *
 * ceres/rotation.h is missing this function for some reason.
 */
template<typename T> inline
void QuaternionInverse(const T in[4], T out[4])
{
  out[0] = in[0];
  out[1] = -in[1];
  out[2] = -in[2];
  out[3] = -in[3];
}

}  // namespace

namespace fuse_variables
{

class Orientation3DLocalParameterization : public fuse_core::LocalParameterization
{
public:
  int GlobalSize() const override
  {
    return 4;
  }

  int LocalSize() const override
  {
    return 3;
  }

  bool Plus(
    const double* x,
    const double* delta,
    double* x_plus_delta) const override
  {
    double q_delta[4];
    ceres::AngleAxisToQuaternion(delta, q_delta);
    ceres::QuaternionProduct(x, q_delta, x_plus_delta);
    return true;
}

  bool ComputeJacobian(
    const double* x,
    double* jacobian) const override
  {
    double x0 = x[0] / 2;
    double x1 = x[1] / 2;
    double x2 = x[2] / 2;
    double x3 = x[3] / 2;
    jacobian[0] = -x1; jacobian[1]  = -x2; jacobian[2]  = -x3;  // NOLINT
    jacobian[3] =  x0; jacobian[4]  = -x3; jacobian[5]  =  x2;  // NOLINT
    jacobian[6] =  x3; jacobian[7]  =  x0; jacobian[8]  = -x1;  // NOLINT
    jacobian[9] = -x2; jacobian[10] =  x1; jacobian[11] =  x0;  // NOLINT
    return true;
  }

  bool Minus(
    const double* x1,
    const double* x2,
    double* delta) const override
  {
    double x1_inverse[4];
    QuaternionInverse(x1, x1_inverse);
    double q_delta[4];
    ceres::QuaternionProduct(x1_inverse, x2, q_delta);
    ceres::QuaternionToAngleAxis(q_delta, delta);
    return true;
  }

  bool ComputeMinusJacobian(
    const double* x,
    double* jacobian) const override
  {
    double x0 = x[0] * 2;
    double x1 = x[1] * 2;
    double x2 = x[2] * 2;
    double x3 = x[3] * 2;
    jacobian[0] = -x1; jacobian[1]  =  x0; jacobian[2]  =  x3;  jacobian[3]  = -x2;  // NOLINT
    jacobian[4] = -x2; jacobian[5]  = -x3; jacobian[6]  =  x0;  jacobian[7]  =  x1;  // NOLINT
    jacobian[8] = -x3; jacobian[9]  =  x2; jacobian[10] = -x1;  jacobian[11] =  x0;  // NOLINT
    return true;
  }
};

Orientation3DStamped::Orientation3DStamped(const ros::Time& stamp, const fuse_core::UUID& device_id) :
  Stamped(stamp, device_id),
  uuid_(fuse_core::uuid::generate(type(), stamp, device_id))
{
}

void Orientation3DStamped::print(std::ostream& stream) const
{
  stream << type() << ":\n"
         << "  uuid: " << uuid() << "\n"
         << "  device_id: " << deviceId() << "\n"
         << "  stamp: " << stamp() << "\n"
         << "  size: " << size() << "\n"
         << "  data:\n"
         << "  - w: " << w() << "\n"
         << "  - x: " << x() << "\n"
         << "  - y: " << y() << "\n"
         << "  - z: " << z() << "\n";
}

fuse_core::Variable::UniquePtr Orientation3DStamped::clone() const
{
  return Orientation3DStamped::make_unique(*this);
}

fuse_core::LocalParameterization* Orientation3DStamped::localParameterization() const
{
  return new Orientation3DLocalParameterization();
}

}  // namespace fuse_variables
