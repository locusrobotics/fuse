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
#include <gtest/gtest.h>

#include <array>
#include <limits>
#include <vector>

#include <fuse_core/eigen_gtest.hpp>
#include <fuse_models/unicycle_3d_predict.hpp>
// #include <tf2_2d/tf2_2d.hpp>

TEST(Predict, predictDirectVals)
{
  fuse_core::Vector3d position1 (0.0, 0.0, 0.0);
  fuse_core::Quaternion orientation1 (1.0, 0.0, 0.0, 0.0);
  fuse_core::Vector3d vel_linear1 (1.0, 0.0, 0.0);
  fuse_core::Vector3d vel_angular1 (0.0, 0.0, 1.570796327);
  fuse_core::Vector3d acc_linear1 (1.0, 0.0, 0.0);
  const double dt = 0.1;
  fuse_core::Vector3d position2;
  fuse_core::Quaternion orientation2;
  fuse_core::Vector3d vel_linear2;
  fuse_core::Vector3d vel_angular2;
  fuse_core::Vector3d acc_linear2;

  fuse_models::predict(
    position1,
    orientation1,
    vel_linear1,
    vel_angular1,
    acc_linear1,
    dt,
    position2,
    orientation2,
    vel_linear2,
    vel_angular2,
    acc_linear2);

  fuse_core::Quaternion q = Eigen::AngleAxisd(0.1570796327, Eigen::Vector3d::UnitZ()) *
                 Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                 Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());

  EXPECT_DOUBLE_EQ(0.105, position2.x());
  EXPECT_DOUBLE_EQ(0.0, position2.y());
  EXPECT_DOUBLE_EQ(0.0, position2.z());
  EXPECT_TRUE(q.isApprox(orientation2));
  EXPECT_DOUBLE_EQ(1.1, vel_linear2.x());
  EXPECT_DOUBLE_EQ(0.0, vel_linear2.y());
  EXPECT_DOUBLE_EQ(0.0, vel_linear2.z());
  EXPECT_DOUBLE_EQ(0.0, vel_angular2.x());
  EXPECT_DOUBLE_EQ(0.0, vel_angular2.y());
  EXPECT_DOUBLE_EQ(1.570796327, vel_angular2.z());
  EXPECT_DOUBLE_EQ(1.0, acc_linear2.x());
  EXPECT_DOUBLE_EQ(0.0, acc_linear2.y());
  EXPECT_DOUBLE_EQ(0.0, acc_linear2.z());

  // // Carry on with the output state from last time - show in-place update support
  fuse_models::predict(
    position2,
    orientation2,
    vel_linear2,
    vel_angular2,
    acc_linear2,
    dt,
    position2,
    orientation2,
    vel_linear2,
    vel_angular2,
    acc_linear2);

  q = Eigen::AngleAxisd(0.3141592654, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
  
  EXPECT_DOUBLE_EQ(0.21858415916807189, position2.x());
  EXPECT_DOUBLE_EQ(0.017989963481956205, position2.y());
  EXPECT_DOUBLE_EQ(0.0, position2.z());
  EXPECT_TRUE(q.isApprox(orientation2));
  EXPECT_DOUBLE_EQ(1.2, vel_linear2.x());
  EXPECT_DOUBLE_EQ(0.0, vel_linear2.y());
  EXPECT_DOUBLE_EQ(0.0, vel_linear2.z());
  EXPECT_DOUBLE_EQ(0.0, vel_angular2.x());
  EXPECT_DOUBLE_EQ(0.0, vel_angular2.y());
  EXPECT_DOUBLE_EQ(1.570796327, vel_angular2.z());
  EXPECT_DOUBLE_EQ(1.0, acc_linear2.x());
  EXPECT_DOUBLE_EQ(0.0, acc_linear2.y());
  EXPECT_DOUBLE_EQ(0.0, acc_linear2.z());

  // // Use non-zero Y values
  vel_linear1.y() = -1.0;
  vel_angular1.z() = -1.570796327;
  acc_linear1.y() = -1.0;

  fuse_models::predict(
    position1,
    orientation1,
    vel_linear1,
    vel_angular1,
    acc_linear1,
    dt,
    position2,
    orientation2,
    vel_linear2,
    vel_angular2,
    acc_linear2);
  
  q = Eigen::AngleAxisd(-0.1570796327, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());

  EXPECT_DOUBLE_EQ(0.105, position2.x());
  EXPECT_DOUBLE_EQ(-0.105, position2.y());
  EXPECT_DOUBLE_EQ(0.0, position2.z());
  EXPECT_TRUE(q.isApprox(orientation2));
  EXPECT_DOUBLE_EQ(1.1, vel_linear2.x());
  EXPECT_DOUBLE_EQ(-1.1, vel_linear2.y());
  EXPECT_DOUBLE_EQ(0.0, vel_linear2.z());
  EXPECT_DOUBLE_EQ(0.0, vel_angular2.x());
  EXPECT_DOUBLE_EQ(0.0, vel_angular2.y());
  EXPECT_DOUBLE_EQ(-1.570796327, vel_angular2.z());
  EXPECT_DOUBLE_EQ(1.0, acc_linear2.x());
  EXPECT_DOUBLE_EQ(-1.0, acc_linear2.y());

  // TODO: continue with out of plane motion
}

TEST(Predict, predictJacobians)
{
  // atm this test is doig nothing, we don't have a way to compute the jacobian analytically
  fuse_core::Vector3d position1 (0.0, 0.0, 0.0);
  fuse_core::Quaternion orientation1 (1.0, 0.0, 0.0, 0.0);
  fuse_core::Vector3d vel_linear1 (1.0, 0.0, 0.0);
  fuse_core::Vector3d vel_angular1 (0.0, 0.0, 1.570796327);
  fuse_core::Vector3d acc_linear1 (1.0, 0.0, 0.0);
  
  const double dt = 0.1;
  fuse_core::Vector3d position2;
  fuse_core::Quaternion orientation2;
  fuse_core::Vector3d vel_linear2;
  fuse_core::Vector3d vel_angular2;
  fuse_core::Vector3d acc_linear2;

  // Predict and compute Jacobian using autodiff
  using Jet = ceres::Jet<double, 3>;
  const Jet jet_dt (dt);
  const Jet jet_position1[3] {{position1.x(), 0}, {position1.y(), 1}, {position1.z(), 2}};
  const Jet jet_orientation1[3] {{orientation1.x(), 0}, {orientation1.y(), 1}, {orientation1.z(), 2}};
  const Jet jet_vel_linear1[3] {{vel_linear1.x(), 0}, {vel_linear1.y(), 1}, {vel_linear1.z(), 2}};
  const Jet jet_vel_angular1[3] {{vel_angular1.x(), 0}, {vel_angular1.y(), 1}, {vel_angular1.z(), 2}};
  const Jet jet_acc_linear1[3] {{acc_linear1.x(), 0}, {acc_linear1.y(), 1}, {acc_linear1.z(), 2}};

  Jet jet_position2[3], jet_orientation2[3], jet_vel_linear2[3], jet_vel_angular2[3], jet_acc_linear2[3];

  fuse_models::predict(
    jet_position1,
    jet_orientation1,
    jet_vel_linear1,
    jet_vel_angular1,
    jet_acc_linear1,
    jet_dt,
    jet_position2,
    jet_orientation2,
    jet_vel_linear2,
    jet_vel_angular2,
    jet_acc_linear2);

  fuse_core::Matrix15d J_autodiff;
  J_autodiff << jet_position2[0].v, jet_position2[1].v, jet_position2[2].v, 
                jet_orientation2[0].v, jet_orientation2[1].v, jet_orientation2[2].v, 
                jet_vel_linear2[0].v, jet_vel_linear2[1].v, jet_vel_linear2[2].v, 
                jet_vel_angular2[0].v, jet_vel_angular2[1].v, jet_vel_angular2[2].v, 
                jet_acc_linear2[0].v, jet_acc_linear2[1].v, jet_acc_linear2[2].v;
  J_autodiff.transposeInPlace();

  const Eigen::IOFormat HeavyFmt(
    Eigen::FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");
  
  std::cout << "Autodiff Jacobian =\n" << J_autodiff.format(HeavyFmt);
  // EXPECT_MATRIX_NEAR(J_autodiff, J_analytic, std::numeric_limits<double>::epsilon())
    // << "Autodiff Jacobian =\n" << J_autodiff.format(HeavyFmt);
    // << "\nAnalytic Jacobian =\n" << J_analytic.format(HeavyFmt);
}
