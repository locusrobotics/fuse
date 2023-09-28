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
#include <fuse_core/error_handler.h>
#include <ros/ros.h>

#include <gtest/gtest.h>

#include <numeric>
#include <string>


// Simple toy class for use in testing callback registration
class ErrorTester
{
  public:
  ErrorTester()
  {
    errors_ = 0;
  }

  void testFunction(std::string unused)
  {
    ++errors_;
  }

  int errors_;
};

TEST(ErrorHandler, DefaultErrorHandler)
{
  // Tests are not guaranteed to be called in order. Ensure cleanup
  fuse_core::ErrorHandler::getHandler().reset();

  // Verify that default behavior for global throws exceptions
  // Many other unit tests use expected exceptions for their unit tests. This test is to see if they've been broken
  EXPECT_THROW(fuse_core::ErrorHandler::getHandler().invalidArgument("Expected invalid"), std::invalid_argument);
  EXPECT_THROW(fuse_core::ErrorHandler::getHandler().outOfRangeError("Expected out of range"), std::out_of_range);
  EXPECT_THROW(fuse_core::ErrorHandler::getHandler().logicError("Expected logic error"), std::logic_error);
  EXPECT_THROW(fuse_core::ErrorHandler::getHandler().runtimeError("Expected runtime error"), std::runtime_error);

  // Verify that callbacks can be registered as expected
  {
    ErrorTester test;

    fuse_core::ErrorHandler::getHandler().registerinvalidArgumentErrorCb(
      std::bind(&ErrorTester::testFunction, &test, std::placeholders::_1));
    fuse_core::ErrorHandler::getHandler().registerRuntimeErrorCb(
      std::bind(&ErrorTester::testFunction, &test, std::placeholders::_1));
    fuse_core::ErrorHandler::getHandler().registerOutOfRangeErrorCb(
      std::bind(&ErrorTester::testFunction, &test, std::placeholders::_1));
    fuse_core::ErrorHandler::getHandler().registerLogicErrorCb(
      std::bind(&ErrorTester::testFunction, &test, std::placeholders::_1));

    fuse_core::ErrorHandler::getHandler().invalidArgument("Expected invalid argument");
    fuse_core::ErrorHandler::getHandler().outOfRangeError("Expected out of range");
    fuse_core::ErrorHandler::getHandler().logicError("Expected logic error");
    fuse_core::ErrorHandler::getHandler().runtimeError("Expected runtime error");

    EXPECT_EQ(test.errors_, 4);
  }
}

TEST(ErrorHandler, ContextErrorHandler)
{
  // Tests are not guaranteed to be called in order. Ensure cleanup
  fuse_core::ErrorHandler::getHandler().reset();

  // By default, no context except for the generic one has a defined error callback
  EXPECT_THROW(fuse_core::ErrorHandler::getHandler().invalidArgument("Expected invalid",
    fuse_core::ErrorContext::CORE), std::out_of_range);
  EXPECT_THROW(fuse_core::ErrorHandler::getHandler().outOfRangeError("Expected out of range",
    fuse_core::ErrorContext::CONSTRAINT), std::out_of_range);
  EXPECT_THROW(fuse_core::ErrorHandler::getHandler().logicError("Expected logic error",
    fuse_core::ErrorContext::GRAPH), std::out_of_range);
  EXPECT_THROW(fuse_core::ErrorHandler::getHandler().runtimeError("Expected runtime error",
    fuse_core::ErrorContext::LOSS), std::out_of_range);

  // Verify that callbacks can be registered as expected for different contexts
  {
    ErrorTester test;

    fuse_core::ErrorHandler::getHandler().registerinvalidArgumentErrorCb(
      std::bind(&ErrorTester::testFunction, &test, std::placeholders::_1), fuse_core::ErrorContext::CORE);
    fuse_core::ErrorHandler::getHandler().registerRuntimeErrorCb(
      std::bind(&ErrorTester::testFunction, &test, std::placeholders::_1), fuse_core::ErrorContext::CONSTRAINT);
    fuse_core::ErrorHandler::getHandler().registerOutOfRangeErrorCb(
      std::bind(&ErrorTester::testFunction, &test, std::placeholders::_1), fuse_core::ErrorContext::GRAPH);
    fuse_core::ErrorHandler::getHandler().registerLogicErrorCb(
      std::bind(&ErrorTester::testFunction, &test, std::placeholders::_1), fuse_core::ErrorContext::LOSS);

    fuse_core::ErrorHandler::getHandler().invalidArgument("Expected invalid Argument", fuse_core::ErrorContext::CORE);
    fuse_core::ErrorHandler::getHandler().outOfRangeError("Expected out of Range", fuse_core::ErrorContext::GRAPH);
    fuse_core::ErrorHandler::getHandler().logicError("Expected logic error", fuse_core::ErrorContext::LOSS);
    fuse_core::ErrorHandler::getHandler().runtimeError("Expected runtime error", fuse_core::ErrorContext::CONSTRAINT);

    EXPECT_EQ(test.errors_, 4);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
