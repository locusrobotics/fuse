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
#include <fuse_core/uuid.h>
#include <fuse_variables/stamped.h>
#include <ros/ros.h>

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>


TEST(Stamped, LoadDeviceId)
{
  // Test loading a device id from each test namespace
  {
    ros::NodeHandle node_handle("/id1");
    fuse_core::UUID actual = fuse_variables::loadDeviceId(node_handle);
    fuse_core::UUID expected =
    {
      0x01, 0x23, 0x45, 0x67,
      0x89, 0xAB,
      0xCD, 0xEF,
      0x01, 0x23,
      0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF
    };
    EXPECT_EQ(expected, actual);
  }
  {
    ros::NodeHandle node_handle("/id2");
    fuse_core::UUID actual = fuse_variables::loadDeviceId(node_handle);
    fuse_core::UUID expected =
    {
      0x01, 0x23, 0x45, 0x67,
      0x89, 0xAB,
      0xCD, 0xEF,
      0x01, 0x23,
      0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF
    };
    EXPECT_EQ(expected, actual);
  }
  {
    ros::NodeHandle node_handle("/id3");
    fuse_core::UUID actual = fuse_variables::loadDeviceId(node_handle);
    fuse_core::UUID expected =
    {
      0x01, 0x23, 0x45, 0x67,
      0x89, 0xAB,
      0xCD, 0xEF,
      0x01, 0x23,
      0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF
    };
    EXPECT_EQ(expected, actual);
  }
  {
    ros::NodeHandle node_handle("/id4");
    fuse_core::UUID actual = fuse_variables::loadDeviceId(node_handle);
    fuse_core::UUID expected =
    {
      0x01, 0x23, 0x45, 0x67,
      0x89, 0xAB,
      0xCD, 0xEF,
      0x01, 0x23,
      0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF
    };
    EXPECT_EQ(expected, actual);
  }
  {
    ros::NodeHandle node_handle("/id5");
    fuse_core::UUID actual = fuse_variables::loadDeviceId(node_handle);
    fuse_core::UUID expected =
    {
      0x01, 0x23, 0x45, 0x67,
      0x89, 0xAB,
      0xCD, 0xEF,
      0x01, 0x23,
      0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF
    };
    EXPECT_EQ(expected, actual);
  }
  {
    ros::NodeHandle node_handle("/id6");
    fuse_core::UUID actual = fuse_variables::loadDeviceId(node_handle);
    fuse_core::UUID expected =
    {
      0x01, 0x23, 0x45, 0x67,
      0x89, 0xAB,
      0xCD, 0xEF,
      0x01, 0x23,
      0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF
    };
    EXPECT_EQ(expected, actual);
  }
  {
    ros::NodeHandle node_handle("/id7");
    EXPECT_THROW(fuse_variables::loadDeviceId(node_handle), std::runtime_error);
  }
  {
    ros::NodeHandle node_handle("/name");
    fuse_core::UUID actual = fuse_variables::loadDeviceId(node_handle);
    fuse_core::UUID expected =
    {
      0x5B, 0x23, 0x43, 0x6D,
      0x8E, 0x7C,
      0x51, 0xCF,
      0x81, 0x62,
      0x5C, 0xD5, 0xFD, 0x37, 0x9E, 0xCF
    };
    EXPECT_EQ(expected, actual);
  }
  {
    ros::NodeHandle node_handle("/none");
    fuse_core::UUID actual = fuse_variables::loadDeviceId(node_handle);
    fuse_core::UUID expected = fuse_core::uuid::NIL;
    EXPECT_EQ(expected, actual);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_load_device_id");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
