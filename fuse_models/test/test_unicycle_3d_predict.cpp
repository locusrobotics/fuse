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
#include <ceres/jet.h>

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
  EXPECT_DOUBLE_EQ(0.0, acc_linear2.z());

  // TODO: continue with out of plane motion
}

TEST(Predict, predictFromDoublePointers)
{
  const double position1[3] {0.0, 0.0, 0.0};
  const double orientation1[4] {1.0, 0.0, 0.0, 0.0};
  const double vel_linear1[3] {1.0, 0.0, 0.0};
  const double vel_angular1[3] {0.0, 0.0, 1.570796327};
  const double acc_linear1[3] {1.0, 0.0, 0.0};
  const double dt = 0.1;
  double position2[3] {1000.0, 0.0, 0.0};
  double orientation2[4];
  double vel_linear2[3];
  double vel_angular2[3];
  double acc_linear2[3];

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

  fuse_core::Quaternion q = 
    Eigen::AngleAxisd(0.1570796327, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());

  EXPECT_DOUBLE_EQ(0.105, position2[0]);
  EXPECT_DOUBLE_EQ(0.0,   position2[1]);
  EXPECT_DOUBLE_EQ(0.0,   position2[2]);
  EXPECT_DOUBLE_EQ(q.w(), orientation2[0]);
  EXPECT_DOUBLE_EQ(q.x(), orientation2[1]);
  EXPECT_DOUBLE_EQ(q.y(), orientation2[2]);
  EXPECT_DOUBLE_EQ(q.z(), orientation2[3]);
  EXPECT_DOUBLE_EQ(1.1,   vel_linear2[0]);
  EXPECT_DOUBLE_EQ(0.0,   vel_linear2[1]);
  EXPECT_DOUBLE_EQ(0.0,   vel_linear2[2]);
  EXPECT_DOUBLE_EQ(0.0,   vel_angular2[0]);
  EXPECT_DOUBLE_EQ(0.0,   vel_angular2[1]);
  EXPECT_DOUBLE_EQ(1.570796327, vel_angular2[2]);
  EXPECT_DOUBLE_EQ(1.0, acc_linear2[0]);
  EXPECT_DOUBLE_EQ(0.0, acc_linear2[1]);
  EXPECT_DOUBLE_EQ(0.0, acc_linear2[2]);

  // // Carry on with the output state from last time - show in-place update support
  // fuse_models::predict(
  //   position2,
  //   orientation2,
  //   vel_linear2,
  //   vel_angular2,
  //   acc_linear2,
  //   dt,
  //   position2,
  //   orientation2,
  //   vel_linear2,
  //   vel_angular2,
  //   acc_linear2);

  // q = Eigen::AngleAxisd(0.3141592654, Eigen::Vector3d::UnitZ()) *
  //     Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
  //     Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
  
  // EXPECT_DOUBLE_EQ(0.21858415916807189,  position2[0]);
  // EXPECT_DOUBLE_EQ(0.017989963481956205, position2[1]);
  // EXPECT_DOUBLE_EQ(0.0, position2[2]);
  // EXPECT_DOUBLE_EQ(q.w(), orientation2[0]);
  // EXPECT_DOUBLE_EQ(q.x(), orientation2[1]);
  // EXPECT_DOUBLE_EQ(q.y(), orientation2[2]);
  // EXPECT_DOUBLE_EQ(q.z(), orientation2[3]);
  // EXPECT_DOUBLE_EQ(1.2,   vel_linear2[0]);
  // EXPECT_DOUBLE_EQ(0.0,   vel_linear2[1]);
  // EXPECT_DOUBLE_EQ(0.0,   vel_linear2[2]);
  // EXPECT_DOUBLE_EQ(0.0,   vel_angular2[0]);
  // EXPECT_DOUBLE_EQ(0.0,   vel_angular2[1]);
  // EXPECT_DOUBLE_EQ(1.570796327, vel_angular2[2]);
  // EXPECT_DOUBLE_EQ(1.0, acc_linear2[0]);
  // EXPECT_DOUBLE_EQ(0.0, acc_linear2[1]);
  // EXPECT_DOUBLE_EQ(0.0, acc_linear2[2]);

  // // // Use non-zero Y values
  // vel_linear1[1] = -1.0;
  // vel_angular1[2] = -1.570796327;
  // acc_linear1[1] = -1.0;

  // fuse_models::predict(
  //   position1,
  //   orientation1,
  //   vel_linear1,
  //   vel_angular1,
  //   acc_linear1,
  //   dt,
  //   position2,
  //   orientation2,
  //   vel_linear2,
  //   vel_angular2,
  //   acc_linear2);
  
  // q = Eigen::AngleAxisd(-0.1570796327, Eigen::Vector3d::UnitZ()) *
  //     Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
  //     Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());

  // EXPECT_DOUBLE_EQ(0.105, position2[0]);
  // EXPECT_DOUBLE_EQ(-0.105,position2[1]);
  // EXPECT_DOUBLE_EQ(0.0,   position2[2]);
  // EXPECT_DOUBLE_EQ(q.w(), orientation2[0]);
  // EXPECT_DOUBLE_EQ(q.x(), orientation2[1]);
  // EXPECT_DOUBLE_EQ(q.y(), orientation2[2]);
  // EXPECT_DOUBLE_EQ(q.z(), orientation2[3]);
  // EXPECT_DOUBLE_EQ(1.1,   vel_linear2[0]);
  // EXPECT_DOUBLE_EQ(-1.1,  vel_linear2[1]);
  // EXPECT_DOUBLE_EQ(0.0,   vel_linear2[2]);
  // EXPECT_DOUBLE_EQ(0.0,   vel_angular2[0]);
  // EXPECT_DOUBLE_EQ(0.0,   vel_angular2[1]);
  // EXPECT_DOUBLE_EQ(-1.570796327, vel_angular2[2]);
  // EXPECT_DOUBLE_EQ(1.0,  acc_linear2[0]);
  // EXPECT_DOUBLE_EQ(-1.0, acc_linear2[1]);
  // EXPECT_DOUBLE_EQ(0.0, acc_linear2[2]);
  // // TODO: continue with out of plane motion
}

TEST(Predict, predictFromJetPointers)
{
  using Jet = ceres::Jet<double, 32>;

  Jet position1[3] = {Jet(0.0), Jet(0.0), Jet(0.0)};
  Jet orientation1[4] = {Jet(1.0), Jet(0.0), Jet(0.0), Jet(0.0)};
  Jet vel_linear1[3] = {Jet(1.0), Jet(0.0), Jet(0.0)};
  Jet vel_angular1[3] = {Jet(0.0), Jet(0.0), Jet(1.570796327)};
  Jet acc_linear1[3] = {Jet(1.0), Jet(0.0), Jet(0.0)};
  const Jet dt = Jet(0.1);
  Jet position2[3];
  Jet orientation2[4];
  Jet vel_linear2[3];
  Jet vel_angular2[3];
  Jet acc_linear2[3];

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

  Eigen::Quaternion<Jet> q = 
    Eigen::AngleAxis<Jet>(Jet(0.1570796327), Eigen::Vector3<Jet>::UnitZ()) *
    Eigen::AngleAxis<Jet>(Jet(0.0), Eigen::Vector3<Jet>::UnitY()) *
    Eigen::AngleAxis<Jet>(Jet(0.0), Eigen::Vector3<Jet>::UnitX());

  EXPECT_DOUBLE_EQ(Jet(0.105).a, position2[0].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a,   position2[1].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a,   position2[2].a);
  EXPECT_DOUBLE_EQ(q.w().a, orientation2[0].a);
  EXPECT_DOUBLE_EQ(q.x().a, orientation2[1].a);
  EXPECT_DOUBLE_EQ(q.y().a, orientation2[2].a);
  EXPECT_DOUBLE_EQ(q.z().a, orientation2[3].a);
  EXPECT_DOUBLE_EQ(Jet(1.1).a,   vel_linear2[0].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a,   vel_linear2[1].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a,   vel_linear2[2].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a,   vel_angular2[0].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a,   vel_angular2[1].a);
  EXPECT_DOUBLE_EQ(Jet(1.570796327).a, vel_angular2[2].a);
  EXPECT_DOUBLE_EQ(Jet(1.0).a, acc_linear2[0].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a, acc_linear2[1].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a, acc_linear2[2].a);

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

  q = Eigen::AngleAxis<Jet>(Jet(0.3141592654), Eigen::Vector3<Jet>::UnitZ()) *
      Eigen::AngleAxis<Jet>(Jet(0.0), Eigen::Vector3<Jet>::UnitY()) *
      Eigen::AngleAxis<Jet>(Jet(0.0), Eigen::Vector3<Jet>::UnitX());
  
  EXPECT_DOUBLE_EQ(Jet(0.21858415916807189).a,  position2[0].a);
  EXPECT_DOUBLE_EQ(Jet(0.017989963481956205).a, position2[1].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a, position2[2].a);
  EXPECT_DOUBLE_EQ(q.w().a, orientation2[0].a);
  EXPECT_DOUBLE_EQ(q.x().a, orientation2[1].a);
  EXPECT_DOUBLE_EQ(q.y().a, orientation2[2].a);
  EXPECT_DOUBLE_EQ(q.z().a, orientation2[3].a);
  EXPECT_DOUBLE_EQ(Jet(1.2).a,   vel_linear2[0].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a,   vel_linear2[1].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a,   vel_linear2[2].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a,   vel_angular2[0].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a,   vel_angular2[1].a);
  EXPECT_DOUBLE_EQ(Jet(1.570796327).a, vel_angular2[2].a);
  EXPECT_DOUBLE_EQ(Jet(1.0).a, acc_linear2[0].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a, acc_linear2[1].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a, acc_linear2[2].a);

  // // Use non-zero Y values
  vel_linear1[1]  = Jet(-1.0);
  vel_angular1[2] = Jet(-1.570796327);
  acc_linear1[1]  = Jet(-1.0);

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
  
  q = Eigen::AngleAxis<Jet>(Jet(-0.1570796327), Eigen::Vector3<Jet>::UnitZ()) *
      Eigen::AngleAxis<Jet>(Jet(0.0), Eigen::Vector3<Jet>::UnitY()) *
      Eigen::AngleAxis<Jet>(Jet(0.0), Eigen::Vector3<Jet>::UnitX());

  EXPECT_DOUBLE_EQ(Jet(0.105).a, position2[0].a);
  EXPECT_DOUBLE_EQ(Jet(-0.105).a,position2[1].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a,   position2[2].a);
  EXPECT_DOUBLE_EQ(q.w().a, orientation2[0].a);
  EXPECT_DOUBLE_EQ(q.x().a, orientation2[1].a);
  EXPECT_DOUBLE_EQ(q.y().a, orientation2[2].a);
  EXPECT_DOUBLE_EQ(q.z().a, orientation2[3].a);
  EXPECT_DOUBLE_EQ(Jet(1.1).a,   vel_linear2[0].a);
  EXPECT_DOUBLE_EQ(Jet(-1.1).a,  vel_linear2[1].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a,   vel_linear2[2].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a,   vel_angular2[0].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a,   vel_angular2[1].a);
  EXPECT_DOUBLE_EQ(Jet(-1.570796327).a, vel_angular2[2].a);
  EXPECT_DOUBLE_EQ(Jet(1.0).a,  acc_linear2[0].a);
  EXPECT_DOUBLE_EQ(Jet(-1.0).a, acc_linear2[1].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a,  acc_linear2[2].a);
  // TODO: continue with out of plane motion
}

TEST(Predict, predictFromEigenMatrixDouble)
{
  Eigen::Matrix<double, 3, 1> position1 = {0.0, 0.0, 0.0};
  Eigen::Quaterniond orientation1 = {1.0, 0.0, 0.0, 0.0};
  Eigen::Matrix<double, 3, 1> vel_linear1 = {1.0, 0.0, 0.0};
  Eigen::Matrix<double, 3, 1> vel_angular1 = {0.0, 0.0, 1.570796327};
  Eigen::Matrix<double, 3, 1> acc_linear1 = {1.0, 0.0, 0.0};
  const double dt = 0.1;
  Eigen::Matrix<double, 3, 1> position2;
  Eigen::Quaterniond orientation2;
  Eigen::Matrix<double, 3, 1> vel_linear2;
  Eigen::Matrix<double, 3, 1> vel_angular2;
  Eigen::Matrix<double, 3, 1> acc_linear2;

  fuse_models::predict_eigen(
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

  fuse_core::Quaternion q = 
    Eigen::AngleAxisd(0.1570796327, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());

  EXPECT_DOUBLE_EQ(0.105, position2.x());
  EXPECT_DOUBLE_EQ(0.0,   position2.y());
  EXPECT_DOUBLE_EQ(0.0,   position2.z());
  EXPECT_DOUBLE_EQ(q.w(), orientation2.w());
  EXPECT_DOUBLE_EQ(q.x(), orientation2.x());
  EXPECT_DOUBLE_EQ(q.y(), orientation2.y());
  EXPECT_DOUBLE_EQ(q.z(), orientation2.z());
  EXPECT_DOUBLE_EQ(1.1,   vel_linear2.x());
  EXPECT_DOUBLE_EQ(0.0,   vel_linear2.y());
  EXPECT_DOUBLE_EQ(0.0,   vel_linear2.z());
  EXPECT_DOUBLE_EQ(0.0,   vel_angular2.x());
  EXPECT_DOUBLE_EQ(0.0,   vel_angular2.y());
  EXPECT_DOUBLE_EQ(1.570796327, vel_angular2.z());
  EXPECT_DOUBLE_EQ(1.0, acc_linear2.x());
  EXPECT_DOUBLE_EQ(0.0, acc_linear2.y());
  EXPECT_DOUBLE_EQ(0.0, acc_linear2.z());

  // // Carry on with the output state from last time - show in-place update support
  fuse_models::predict_eigen(
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
  
  EXPECT_DOUBLE_EQ(0.21858415916807189,  position2.x());
  EXPECT_DOUBLE_EQ(0.017989963481956205, position2.y());
  EXPECT_DOUBLE_EQ(0.0, position2.z());
  EXPECT_DOUBLE_EQ(q.w(), orientation2.w());
  EXPECT_DOUBLE_EQ(q.x(), orientation2.x());
  EXPECT_DOUBLE_EQ(q.y(), orientation2.y());
  EXPECT_DOUBLE_EQ(q.z(), orientation2.z());
  EXPECT_DOUBLE_EQ(1.2,   vel_linear2.x());
  EXPECT_DOUBLE_EQ(0.0,   vel_linear2.y());
  EXPECT_DOUBLE_EQ(0.0,   vel_linear2.z());
  EXPECT_DOUBLE_EQ(0.0,   vel_angular2.x());
  EXPECT_DOUBLE_EQ(0.0,   vel_angular2.y());
  EXPECT_DOUBLE_EQ(1.570796327, vel_angular2.z());
  EXPECT_DOUBLE_EQ(1.0, acc_linear2.x());
  EXPECT_DOUBLE_EQ(0.0, acc_linear2.y());
  EXPECT_DOUBLE_EQ(0.0, acc_linear2.z());

  // // Use non-zero Y values
  vel_linear1[1] = -1.0;
  vel_angular1[2] = -1.570796327;
  acc_linear1[1] = -1.0;

  fuse_models::predict_eigen(
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
  EXPECT_DOUBLE_EQ(-0.105,position2.y());
  EXPECT_DOUBLE_EQ(0.0,   position2.z());
  EXPECT_DOUBLE_EQ(q.w(), orientation2.w());
  EXPECT_DOUBLE_EQ(q.x(), orientation2.x());
  EXPECT_DOUBLE_EQ(q.y(), orientation2.y());
  EXPECT_DOUBLE_EQ(q.z(), orientation2.z());
  EXPECT_DOUBLE_EQ(1.1,   vel_linear2.x());
  EXPECT_DOUBLE_EQ(-1.1,  vel_linear2.y());
  EXPECT_DOUBLE_EQ(0.0,   vel_linear2.z());
  EXPECT_DOUBLE_EQ(0.0,   vel_angular2.x());
  EXPECT_DOUBLE_EQ(0.0,   vel_angular2.y());
  EXPECT_DOUBLE_EQ(-1.570796327, vel_angular2.z());
  EXPECT_DOUBLE_EQ(1.0,  acc_linear2.x());
  EXPECT_DOUBLE_EQ(-1.0, acc_linear2.y());
  EXPECT_DOUBLE_EQ(0.0, acc_linear2.z());
  // TODO: continue with out of plane motion
}

TEST(Predict, predictFromEigenMatrixJet)
{
  using Jet = ceres::Jet<double, 32>;
  Eigen::Matrix<Jet, 3, 1> position1 = {Jet(0.0), Jet(0.0), Jet(0.0)};
  Eigen::Quaternion<Jet> orientation1 = {Jet(1.0), Jet(0.0), Jet(0.0), Jet(0.0)};
  Eigen::Matrix<Jet, 3, 1> vel_linear1 = {Jet(1.0), Jet(0.0), Jet(0.0)};
  Eigen::Matrix<Jet, 3, 1> vel_angular1 = {Jet(0.0), Jet(0.0), Jet(1.570796327)};
  Eigen::Matrix<Jet, 3, 1> acc_linear1 = {Jet(1.0), Jet(0.0), Jet(0.0)};
  const Jet dt = Jet(0.1);
  Eigen::Matrix<Jet, 3, 1> position2;
  Eigen::Quaternion<Jet> orientation2;
  Eigen::Matrix<Jet, 3, 1> vel_linear2;
  Eigen::Matrix<Jet, 3, 1> vel_angular2;
  Eigen::Matrix<Jet, 3, 1> acc_linear2;

  fuse_models::predict_eigen(
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

  Eigen::Quaternion<Jet> q = 
    Eigen::AngleAxis<Jet>(Jet(0.1570796327), Eigen::Vector3<Jet>::UnitZ()) *
    Eigen::AngleAxis<Jet>(Jet(0.0), Eigen::Vector3<Jet>::UnitY()) *
    Eigen::AngleAxis<Jet>(Jet(0.0), Eigen::Vector3<Jet>::UnitX());

  EXPECT_DOUBLE_EQ(Jet(0.105).a, position2[0].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a,   position2[1].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a,   position2[2].a);
  EXPECT_DOUBLE_EQ(q.w().a, orientation2.w().a);
  EXPECT_DOUBLE_EQ(q.x().a, orientation2.x().a);
  EXPECT_DOUBLE_EQ(q.y().a, orientation2.y().a);
  EXPECT_DOUBLE_EQ(q.z().a, orientation2.z().a);
  EXPECT_DOUBLE_EQ(Jet(1.1).a,   vel_linear2[0].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a,   vel_linear2[1].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a,   vel_linear2[2].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a,   vel_angular2[0].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a,   vel_angular2[1].a);
  EXPECT_DOUBLE_EQ(Jet(1.570796327).a, vel_angular2[2].a);
  EXPECT_DOUBLE_EQ(Jet(1.0).a, acc_linear2[0].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a, acc_linear2[1].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a, acc_linear2[2].a);

  // // Carry on with the output state from last time - show in-place update support
  fuse_models::predict_eigen(
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

  q = Eigen::AngleAxis<Jet>(Jet(0.3141592654), Eigen::Vector3<Jet>::UnitZ()) *
      Eigen::AngleAxis<Jet>(Jet(0.0), Eigen::Vector3<Jet>::UnitY()) *
      Eigen::AngleAxis<Jet>(Jet(0.0), Eigen::Vector3<Jet>::UnitX());
  
  EXPECT_DOUBLE_EQ(Jet(0.21858415916807189).a,  position2[0].a);
  EXPECT_DOUBLE_EQ(Jet(0.017989963481956205).a, position2[1].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a, position2[2].a);
  EXPECT_DOUBLE_EQ(q.w().a, orientation2.w().a);
  EXPECT_DOUBLE_EQ(q.x().a, orientation2.x().a);
  EXPECT_DOUBLE_EQ(q.y().a, orientation2.y().a);
  EXPECT_DOUBLE_EQ(q.z().a, orientation2.z().a);
  EXPECT_DOUBLE_EQ(Jet(1.2).a,   vel_linear2[0].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a,   vel_linear2[1].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a,   vel_linear2[2].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a,   vel_angular2[0].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a,   vel_angular2[1].a);
  EXPECT_DOUBLE_EQ(Jet(1.570796327).a, vel_angular2[2].a);
  EXPECT_DOUBLE_EQ(Jet(1.0).a, acc_linear2[0].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a, acc_linear2[1].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a, acc_linear2[2].a);

  // // Use non-zero Y values
  vel_linear1[1]  = Jet(-1.0);
  vel_angular1[2] = Jet(-1.570796327);
  acc_linear1[1]  = Jet(-1.0);

  fuse_models::predict_eigen(
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
  
  q = Eigen::AngleAxis<Jet>(Jet(-0.1570796327), Eigen::Vector3<Jet>::UnitZ()) *
      Eigen::AngleAxis<Jet>(Jet(0.0), Eigen::Vector3<Jet>::UnitY()) *
      Eigen::AngleAxis<Jet>(Jet(0.0), Eigen::Vector3<Jet>::UnitX());

  EXPECT_DOUBLE_EQ(Jet(0.105).a, position2[0].a);
  EXPECT_DOUBLE_EQ(Jet(-0.105).a,position2[1].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a,   position2[2].a);
  EXPECT_DOUBLE_EQ(q.w().a, orientation2.w().a);
  EXPECT_DOUBLE_EQ(q.x().a, orientation2.x().a);
  EXPECT_DOUBLE_EQ(q.y().a, orientation2.y().a);
  EXPECT_DOUBLE_EQ(q.z().a, orientation2.z().a);
  EXPECT_DOUBLE_EQ(Jet(1.1).a,   vel_linear2[0].a);
  EXPECT_DOUBLE_EQ(Jet(-1.1).a,  vel_linear2[1].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a,   vel_linear2[2].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a,   vel_angular2[0].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a,   vel_angular2[1].a);
  EXPECT_DOUBLE_EQ(Jet(-1.570796327).a, vel_angular2[2].a);
  EXPECT_DOUBLE_EQ(Jet(1.0).a,  acc_linear2[0].a);
  EXPECT_DOUBLE_EQ(Jet(-1.0).a, acc_linear2[1].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a,  acc_linear2[2].a);
  // TODO: continue with out of plane motion
}