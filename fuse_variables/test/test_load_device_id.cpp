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

#include <stdexcept>
#include <string>
#include <thread>

#include <fuse_core/uuid.hpp>
#include <fuse_variables/stamped.hpp>
#include <rclcpp/rclcpp.hpp>

class TestLoadDeviceId : public ::testing::Test
{
public:
  void SetUp() override
  {
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    spinner_ = std::thread(
      [&]() {
        executor_->spin();
      });
  }

  void TearDown() override
  {
    executor_->cancel();
    if (spinner_.joinable()) {
      spinner_.join();
    }
    executor_.reset();
  }

  std::thread spinner_;  //!< Internal thread for spinning the executor
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
};

TEST_F(TestLoadDeviceId, LoadDeviceId)
{
  // Test loading a device id from each test namespace
  {
    auto node = rclcpp::Node::make_shared("id1_node");
    node->declare_parameter("device_id", std::string("01234567-89AB-CDEF-0123-456789ABCDEF"));
    fuse_core::UUID actual = fuse_variables::loadDeviceId(*node);
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
    auto node = rclcpp::Node::make_shared("id2_node");
    node->declare_parameter("device_id", std::string("01234567-89ab-cdef-0123-456789abcdef"));
    fuse_core::UUID actual = fuse_variables::loadDeviceId(*node);
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
    auto node = rclcpp::Node::make_shared("id3_node");
    node->declare_parameter("device_id", std::string("0123456789ABCDEF0123456789ABCDEF"));
    fuse_core::UUID actual = fuse_variables::loadDeviceId(*node);
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
    auto node = rclcpp::Node::make_shared("id4_node");
    node->declare_parameter("device_id", std::string("0123456789abcdef0123456789abcdef"));
    fuse_core::UUID actual = fuse_variables::loadDeviceId(*node);
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
    auto node = rclcpp::Node::make_shared("id5_node");
    node->declare_parameter("device_id", std::string("{01234567-89ab-cdef-0123-456789abcdef}"));
    fuse_core::UUID actual = fuse_variables::loadDeviceId(*node);
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
    auto node = rclcpp::Node::make_shared("id6_node");
    node->declare_parameter("device_id", std::string("{01234567-89AB-CDEF-0123-456789ABCDEF}"));
    fuse_core::UUID actual = fuse_variables::loadDeviceId(*node);
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
    auto node = rclcpp::Node::make_shared("id7_node");
    node->declare_parameter("device_id", std::string("{THIS IS NOT VALID!!! 123456789ABCDEF}"));
    EXPECT_THROW(fuse_variables::loadDeviceId(*node), std::runtime_error);
  }
  {
    auto node = rclcpp::Node::make_shared("name_node");
    node->declare_parameter("device_name", std::string("Test"));
    fuse_core::UUID actual = fuse_variables::loadDeviceId(*node);
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
    auto node = rclcpp::Node::make_shared("none_node");
    node->declare_parameter("some_other_parameter", 1);
    fuse_core::UUID actual = fuse_variables::loadDeviceId(*node);
    fuse_core::UUID expected = fuse_core::uuid::NIL;
    EXPECT_EQ(expected, actual);
  }
}


// NOTE(CH3): This main is required because the test is manually run by a launch test
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
