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

#include <fuse_core/util.hpp>

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
