/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, Intrinsic.ai
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

#include <limits>
#include <map>
#include <string>

#include <fuse_core/parameter.hpp>

TEST(parameter, list_parameter_override_prefixes)
{
  const std::map<std::string, rclcpp::ParameterValue> overrides = {
    {"foo", {}},
    {"foo.baz", {}},
    {"foo.bar", {}},
    {"foo.bar.baz", {}},
    {"foo.bar.baz.bing", {}},
    {"foobar.baz", {}},
    {"baz", {}}
  };

  {
    auto matches = fuse_core::detail::list_parameter_override_prefixes(
      overrides, "foo");
    EXPECT_EQ(2u, matches.size());
    EXPECT_NE(matches.end(), matches.find("foo.baz"));
    EXPECT_NE(matches.end(), matches.find("foo.bar"));
  }
  {
    auto matches = fuse_core::detail::list_parameter_override_prefixes(
      overrides, "foo.bar");
    EXPECT_EQ(1u, matches.size());
    EXPECT_NE(matches.end(), matches.find("foo.bar.baz"));
  }
  {
    auto matches = fuse_core::detail::list_parameter_override_prefixes(
      overrides, "foo.bar.baz");
    EXPECT_EQ(1u, matches.size());
    EXPECT_NE(matches.end(), matches.find("foo.bar.baz.bing"));
  }
  {
    auto matches = fuse_core::detail::list_parameter_override_prefixes(
      overrides, "foo.baz");
    EXPECT_EQ(0u, matches.size());
  }
  {
    auto matches = fuse_core::detail::list_parameter_override_prefixes(
      overrides, "foobar");
    EXPECT_EQ(1u, matches.size());
    EXPECT_NE(matches.end(), matches.find("foobar.baz"));
  }
  {
    auto matches = fuse_core::detail::list_parameter_override_prefixes(
      overrides, "dne");
    EXPECT_EQ(0u, matches.size());
  }
  {
    auto matches = fuse_core::detail::list_parameter_override_prefixes(
      overrides, "");
    EXPECT_EQ(3u, matches.size());
    EXPECT_NE(matches.end(), matches.find("foo"));
    EXPECT_NE(matches.end(), matches.find("foobar"));
    EXPECT_NE(matches.end(), matches.find("baz"));
  }
}
