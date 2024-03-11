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
#include <gtest/gtest.h>

#include <numeric>
#include <string>

#include <ceres/autodiff_cost_function.h>
#include <fuse_core/util.hpp>
#include <fuse_core/eigen.hpp>

struct Quat2RPY {
  template <typename T>
  bool operator()(const T * const q, T * rpy) const {
    rpy[0] = fuse_core::getRoll(q[0], q[1], q[2], q[3]);
    rpy[1] = fuse_core::getPitch(q[0], q[1], q[2], q[3]);
    rpy[2] = fuse_core::getYaw(q[0], q[1], q[2], q[3]);
    return true;
  }

  static ceres::CostFunction* Create() {
    return (new ceres::AutoDiffCostFunction<Quat2RPY, 3, 4>(new Quat2RPY()));
  }
};

struct QuatProd {
  template <typename T>
  bool operator()(
    const T * const q1,
    const T * const q2,
    T * q_out) const {
      ceres::QuaternionProduct(q1, q2, q_out);
      return true;
    }

  static ceres::CostFunction* Create() {
    return (new ceres::AutoDiffCostFunction<QuatProd, 4, 4, 4>(new QuatProd()));
  }
};

struct Quat2AngleAxis {
  template <typename T>
  bool operator()(const T * const q, T * aa) const {
      ceres::QuaternionToAngleAxis(q, aa);
      return true;
    }

  static ceres::CostFunction* Create() {
    return (new ceres::AutoDiffCostFunction<Quat2AngleAxis, 3, 4>(new Quat2AngleAxis()));
  }
};

TEST(Util, wrapAngle2D)
{
  // Wrap angle already in [-Pi, +Pi) range
  {
    const double angle = 0.5;
    EXPECT_EQ(angle, fuse_core::wrapAngle2D(angle));
  }

  // Wrap angle equal to +Pi
  {
    const double angle = M_PI;
    EXPECT_EQ(-angle, fuse_core::wrapAngle2D(angle));
  }

  // Wrap angle equal to -Pi
  {
    const double angle = -M_PI;
    EXPECT_EQ(angle, fuse_core::wrapAngle2D(angle));
  }

  // Wrap angle greater than +Pi
  {
    const double angle = 0.5;
    EXPECT_EQ(angle, fuse_core::wrapAngle2D(angle + 3.0 * 2.0 * M_PI));
  }

  // Wrap angle smaller than -Pi
  {
    const double angle = 0.5;
    EXPECT_EQ(angle, fuse_core::wrapAngle2D(angle - 3.0 * 2.0 * M_PI));
  }

  // Join topic names
  {
    EXPECT_EQ("a/b", fuse_core::joinTopicName("a", "b"));
    EXPECT_EQ("/a/b", fuse_core::joinTopicName("/a", "b"));
    EXPECT_EQ("a/b", fuse_core::joinTopicName("a/", "b"));
    EXPECT_EQ("/b", fuse_core::joinTopicName("a", "/b"));
    EXPECT_EQ("/b", fuse_core::joinTopicName("a/", "/b"));
    EXPECT_EQ("~/b", fuse_core::joinTopicName("a/", "~/b"));
    EXPECT_EQ("~b", fuse_core::joinTopicName("a/", "~b"));
  }
}

TEST(Util, quaternion2rpy)
{
  // Test correct conversion from quaternion to roll-pitch-yaw
  double q[4] = {1.0, 0.0, 0.0, 0.0};
  double rpy[3];
  fuse_core::quaternion2rpy(q, rpy);
  EXPECT_EQ(0.0, rpy[0]);
  EXPECT_EQ(0.0, rpy[1]);
  EXPECT_EQ(0.0, rpy[2]);

  q[0] = 0.9818562;
  q[1] = 0.0640713;
  q[2] = 0.0911575;
  q[3] = -0.1534393;

  fuse_core::quaternion2rpy(q, rpy);
  EXPECT_NEAR(0.1, rpy[0], 1e-6);
  EXPECT_NEAR(0.2, rpy[1], 1e-6);
  EXPECT_NEAR(-0.3, rpy[2], 1e-6);

  // Test correct quaternion to roll-pitch-yaw function jacobian
  const Eigen::Quaterniond q_eigen = Eigen::Quaterniond::UnitRandom();
  double J_analytic[12], J_autodiff[12];
  q[0] = q_eigen.w();
  q[1] = q_eigen.x();
  q[2] = q_eigen.y();
  q[3] = q_eigen.z();
  
  fuse_core::quaternion2rpy(q, rpy, J_analytic);

  double * jacobians[1] = {J_autodiff};
  const double * parameters[1] = {q};

  ceres::CostFunction * quat2rpy_cf = Quat2RPY::Create();
  double rpy_autodiff[3];
  quat2rpy_cf->Evaluate(parameters, rpy_autodiff, jacobians);
  
  Eigen::Map<fuse_core::Matrix<double, 3, 4>> J_autodiff_map(jacobians[0]);
  Eigen::Map<fuse_core::Matrix<double, 3, 4>> J_analytic_map(J_analytic);
  
  EXPECT_TRUE(J_analytic_map.isApprox(J_autodiff_map));
}

TEST(Util, quaternionProduct)
{
  // Test correct quaternion product function jacobian
  const Eigen::Quaterniond q1_eigen = Eigen::Quaterniond::UnitRandom();
  const Eigen::Quaterniond q2_eigen = Eigen::Quaterniond::UnitRandom();
  double q_out[4];
  double q1[4] 
  {
    q1_eigen.w(),
    q1_eigen.x(),
    q1_eigen.y(),
    q1_eigen.z()
  };

  double q2[4] 
  {
    q2_eigen.w(),
    q2_eigen.x(),
    q2_eigen.y(),
    q2_eigen.z()
  };
  
  // Atm only the jacobian wrt the second quaternion is implemented. If the computation will be
  // extended in future, we just have to compare J_analytic_q1 with the other automatic J_autodiff_q1. 
  double J_analytic_q1[16], J_analytic_q2[16]; // Analytical Jacobians wrt first and second quaternion
  double J_autodiff_q1[16], J_autodiff_q2[16]; // Autodiff Jacobians wrt first and second quaternion
  
  fuse_core::quaternionProduct(q1, q2, q_out, J_analytic_q2);  

  double * jacobians[2];
  jacobians[0] = J_autodiff_q1;
  jacobians[1] = J_autodiff_q2;

  const double * parameters[2];
  parameters[0] = q1;
  parameters[1] = q2;

  ceres::CostFunction * quat_prod_cf = QuatProd::Create();
  double q_out_autodiff[4];
  quat_prod_cf->Evaluate(parameters, q_out_autodiff, jacobians);
  
  Eigen::Map<fuse_core::Matrix<double, 4, 4>> J_autodiff_q1_map(jacobians[0]);
  Eigen::Map<fuse_core::Matrix<double, 4, 4>> J_autodiff_q2_map(jacobians[1]);

  // Eigen::Map<fuse_core::Matrix<double, 4, 4>> J_analytic_q1_map(J_analytic_q1);
  Eigen::Map<fuse_core::Matrix<double, 4, 4>> J_analytic_q2_map(J_analytic_q2);
  
  EXPECT_TRUE(J_analytic_q2_map.isApprox(J_autodiff_q2_map));
}

TEST(Util, quaternionToAngleAxis)
{
  // Test correct quaternion to angle-axis function jacobian
  const Eigen::Quaterniond q_eigen = Eigen::Quaterniond::UnitRandom();
  double angle_axis[3];
  double q[4] 
  {
    q_eigen.w(),
    q_eigen.x(),
    q_eigen.y(),
    q_eigen.z()
  };
 
  double J_analytic[12]; 
  double J_autodiff[12];
  
  fuse_core::quaternionToAngleAxis(q, angle_axis, J_analytic);  

  double * jacobians[1] = {J_autodiff};
  const double * parameters[1] = {q};

  ceres::CostFunction * quat2angle_axis_cf = Quat2AngleAxis::Create();
  double angle_axis_autodiff[3];
  quat2angle_axis_cf->Evaluate(parameters, angle_axis_autodiff, jacobians);
  
  Eigen::Map<fuse_core::Matrix<double, 3, 4>> J_autodiff_map(jacobians[0]);
  Eigen::Map<fuse_core::Matrix<double, 3, 4>> J_analytic_map(J_analytic);
  
  EXPECT_TRUE(J_analytic_map.isApprox(J_autodiff_map));
}

