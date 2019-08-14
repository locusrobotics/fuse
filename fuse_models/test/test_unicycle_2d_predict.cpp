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
#include <fuse_models/unicycle_2d_predict.h>

#include <gtest/gtest.h>
#include <tf2_2d/tf2_2d.h>

#include <vector>


TEST(Predict, predictDirectVals)
{
  double position1_x = 0.0;
  double position1_y = 0.0;
  double yaw1 = 0.0;
  double vel_linear1_x = 1.0;
  double vel_linear1_y = 0.0;
  double vel_yaw1 = 1.570796327;
  double acc_linear1_x = 1.0;
  double acc_linear1_y = 0.0;
  double dt = 0.1;
  double position2_x = 0.0;
  double position2_y = 0.0;
  double yaw2 = 0.0;
  double vel_linear2_x = 0.0;
  double vel_linear2_y = 0.0;
  double vel_yaw2 = 0.0;
  double acc_linear2_x = 0.0;
  double acc_linear2_y = 0.0;

  fuse_models::predict(
    position1_x,
    position1_y,
    yaw1,
    vel_linear1_x,
    vel_linear1_y,
    vel_yaw1,
    acc_linear1_x,
    acc_linear1_y,
    dt,
    position2_x,
    position2_y,
    yaw2,
    vel_linear2_x,
    vel_linear2_y,
    vel_yaw2,
    acc_linear2_x,
    acc_linear2_y);

  EXPECT_DOUBLE_EQ(0.105, position2_x);
  EXPECT_DOUBLE_EQ(0.0, position2_y);
  EXPECT_DOUBLE_EQ(0.1570796327, yaw2);
  EXPECT_DOUBLE_EQ(1.1, vel_linear2_x);
  EXPECT_DOUBLE_EQ(0.0, vel_linear2_y);
  EXPECT_DOUBLE_EQ(1.570796327, vel_yaw2);
  EXPECT_DOUBLE_EQ(1.0, acc_linear2_x);
  EXPECT_DOUBLE_EQ(0.0, acc_linear2_y);

  // Carry on with the output state from last time - show in-place update support
  fuse_models::predict(
    position2_x,
    position2_y,
    yaw2,
    vel_linear2_x,
    vel_linear2_y,
    vel_yaw2,
    acc_linear2_x,
    acc_linear2_y,
    dt,
    position2_x,
    position2_y,
    yaw2,
    vel_linear2_x,
    vel_linear2_y,
    vel_yaw2,
    acc_linear2_x,
    acc_linear2_y);

  EXPECT_DOUBLE_EQ(0.21858415916807189, position2_x);
  EXPECT_DOUBLE_EQ(0.017989963481956205, position2_y);
  EXPECT_DOUBLE_EQ(0.3141592654, yaw2);
  EXPECT_DOUBLE_EQ(1.2, vel_linear2_x);
  EXPECT_DOUBLE_EQ(0.0, vel_linear2_y);
  EXPECT_DOUBLE_EQ(1.570796327, vel_yaw2);
  EXPECT_DOUBLE_EQ(1.0, acc_linear2_x);
  EXPECT_DOUBLE_EQ(0.0, acc_linear2_y);

  // Use non-zero Y values
  vel_linear1_y = -1.0;
  vel_yaw1 = -1.570796327;
  acc_linear1_y = -1.0;

  fuse_models::predict(
    position1_x,
    position1_y,
    yaw1,
    vel_linear1_x,
    vel_linear1_y,
    vel_yaw1,
    acc_linear1_x,
    acc_linear1_y,
    dt,
    position2_x,
    position2_y,
    yaw2,
    vel_linear2_x,
    vel_linear2_y,
    vel_yaw2,
    acc_linear2_x,
    acc_linear2_y);

  EXPECT_DOUBLE_EQ(0.105, position2_x);
  EXPECT_DOUBLE_EQ(-0.105, position2_y);
  EXPECT_DOUBLE_EQ(-0.1570796327, yaw2);
  EXPECT_DOUBLE_EQ(1.1, vel_linear2_x);
  EXPECT_DOUBLE_EQ(-1.1, vel_linear2_y);
  EXPECT_DOUBLE_EQ(-1.570796327, vel_yaw2);
  EXPECT_DOUBLE_EQ(1.0, acc_linear2_x);
  EXPECT_DOUBLE_EQ(-1.0, acc_linear2_y);
}

TEST(Predict, predictPointers)
{
  std::vector<double> position1(2, 0.0);
  double yaw1 = 0.0;
  std::vector<double> vel_linear1(2, 0.0);
  vel_linear1[0] = 1.0;
  double vel_yaw1 = 1.570796327;
  std::vector<double> acc_linear1(2, 0.0);
  acc_linear1[0] = 1.0;
  double dt = 0.1;
  std::vector<double> position2(2, 0.0);
  double yaw2 = 0.0;
  std::vector<double> vel_linear2(2, 0.0);
  vel_linear2[0] = 1.0;
  double vel_yaw2 = 1.570796327;
  std::vector<double> acc_linear2(2, 0.0);

  fuse_models::predict(
    position1.data(),
    &yaw1,
    vel_linear1.data(),
    &vel_yaw1,
    acc_linear1.data(),
    dt,
    position2.data(),
    &yaw2,
    vel_linear2.data(),
    &vel_yaw2,
    acc_linear2.data());

  EXPECT_DOUBLE_EQ(0.105, position2[0]);
  EXPECT_DOUBLE_EQ(0.0, position2[1]);
  EXPECT_DOUBLE_EQ(0.1570796327, yaw2);
  EXPECT_DOUBLE_EQ(1.1, vel_linear2[0]);
  EXPECT_DOUBLE_EQ(0.0, vel_linear2[1]);
  EXPECT_DOUBLE_EQ(1.570796327, vel_yaw2);
  EXPECT_DOUBLE_EQ(1.0, acc_linear2[0]);
  EXPECT_DOUBLE_EQ(0.0, acc_linear2[1]);

  // Carry on with the output state from last time - show in-place update support
  fuse_models::predict(
    position2.data(),
    &yaw2,
    vel_linear2.data(),
    &vel_yaw2,
    acc_linear2.data(),
    dt,
    position2.data(),
    &yaw2,
    vel_linear2.data(),
    &vel_yaw2,
    acc_linear2.data());

  EXPECT_DOUBLE_EQ(0.21858415916807189, position2[0]);
  EXPECT_DOUBLE_EQ(0.017989963481956205, position2[1]);
  EXPECT_DOUBLE_EQ(0.3141592654, yaw2);
  EXPECT_DOUBLE_EQ(1.2, vel_linear2[0]);
  EXPECT_DOUBLE_EQ(0.0, vel_linear2[1]);
  EXPECT_DOUBLE_EQ(1.570796327, vel_yaw2);
  EXPECT_DOUBLE_EQ(1.0, acc_linear2[0]);
  EXPECT_DOUBLE_EQ(0.0, acc_linear2[1]);

  // Use non-zero Y values
  vel_linear1[1] = -1.0;
  vel_yaw1 = -1.570796327;
  acc_linear1[1] = -1.0;

  fuse_models::predict(
    position1.data(),
    &yaw1,
    vel_linear1.data(),
    &vel_yaw1,
    acc_linear1.data(),
    dt,
    position2.data(),
    &yaw2,
    vel_linear2.data(),
    &vel_yaw2,
    acc_linear2.data());

  EXPECT_DOUBLE_EQ(0.105, position2[0]);
  EXPECT_DOUBLE_EQ(-0.105, position2[1]);
  EXPECT_DOUBLE_EQ(-0.1570796327, yaw2);
  EXPECT_DOUBLE_EQ(1.1, vel_linear2[0]);
  EXPECT_DOUBLE_EQ(-1.1, vel_linear2[1]);
  EXPECT_DOUBLE_EQ(-1.570796327, vel_yaw2);
  EXPECT_DOUBLE_EQ(1.0, acc_linear2[0]);
  EXPECT_DOUBLE_EQ(-1.0, acc_linear2[1]);
}

TEST(Predict, predictObjects)
{
  tf2_2d::Transform pose1;
  tf2_2d::Vector2 vel_linear1;
  vel_linear1.setX(1.0);
  double vel_yaw1 = 1.570796327;
  tf2_2d::Vector2 acc_linear1;
  acc_linear1.setX(1.0);
  double dt = 0.1;
  tf2_2d::Transform pose2;
  tf2_2d::Vector2 vel_linear2;
  double vel_yaw2 = 0.0;
  tf2_2d::Vector2 acc_linear2;

  fuse_models::predict(
    pose1,
    vel_linear1,
    vel_yaw1,
    acc_linear1,
    dt,
    pose2,
    vel_linear2,
    vel_yaw2,
    acc_linear2);

  EXPECT_DOUBLE_EQ(0.105, pose2.x());
  EXPECT_DOUBLE_EQ(0.0, pose2.y());
  EXPECT_DOUBLE_EQ(0.1570796327, pose2.yaw());
  EXPECT_DOUBLE_EQ(1.1, vel_linear2.x());
  EXPECT_DOUBLE_EQ(0.0, vel_linear2.y());
  EXPECT_DOUBLE_EQ(1.570796327, vel_yaw2);
  EXPECT_DOUBLE_EQ(1.0, acc_linear2.x());
  EXPECT_DOUBLE_EQ(0.0, acc_linear2.y());

  // Carry on with the output state from last time - show in-place update support
  fuse_models::predict(
    pose2,
    vel_linear2,
    vel_yaw2,
    acc_linear2,
    dt,
    pose2,
    vel_linear2,
    vel_yaw2,
    acc_linear2);


  EXPECT_DOUBLE_EQ(0.21858415916807189, pose2.x());
  EXPECT_DOUBLE_EQ(0.017989963481956205, pose2.y());
  EXPECT_DOUBLE_EQ(0.3141592654, pose2.yaw());
  EXPECT_DOUBLE_EQ(1.2, vel_linear2.x());
  EXPECT_DOUBLE_EQ(0.0, vel_linear2.y());
  EXPECT_DOUBLE_EQ(1.570796327, vel_yaw2);
  EXPECT_DOUBLE_EQ(1.0, acc_linear2.x());
  EXPECT_DOUBLE_EQ(0.0, acc_linear2.y());

  // Use non-zero Y values
  vel_linear1.setY(-1.0);
  vel_yaw1 = -1.570796327;
  acc_linear1.setY(-1.0);

  fuse_models::predict(
    pose1,
    vel_linear1,
    vel_yaw1,
    acc_linear1,
    dt,
    pose2,
    vel_linear2,
    vel_yaw2,
    acc_linear2);

  EXPECT_DOUBLE_EQ(0.105, pose2.x());
  EXPECT_DOUBLE_EQ(-0.105, pose2.y());
  EXPECT_DOUBLE_EQ(-0.1570796327, pose2.yaw());
  EXPECT_DOUBLE_EQ(1.1, vel_linear2.x());
  EXPECT_DOUBLE_EQ(-1.1, vel_linear2.y());
  EXPECT_DOUBLE_EQ(-1.570796327, vel_yaw2);
  EXPECT_DOUBLE_EQ(1.0, acc_linear2.x());
  EXPECT_DOUBLE_EQ(-1.0, acc_linear2.y());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
