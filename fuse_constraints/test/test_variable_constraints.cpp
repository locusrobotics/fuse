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
#include <gtest/gtest.h>

#include <algorithm>
#include <iterator>
#include <vector>

#include <fuse_constraints/variable_constraints.hpp>

using fuse_constraints::VariableConstraints;

TEST(VariableConstraints, Size)
{
  auto vars = VariableConstraints();

  EXPECT_TRUE(vars.empty());
  EXPECT_EQ(0u, vars.size());

  vars.insert(0u, {0u, 1u});  // NOLINT
  vars.insert(1u, {0u, 1u});  // NOLINT
  vars.insert(2u, {0u, 1u});  // NOLINT

  EXPECT_FALSE(vars.empty());
  EXPECT_EQ(6u, vars.size());
}

TEST(VariableConstraints, NextVariableIndex)
{
  auto vars = VariableConstraints();
  EXPECT_EQ(0u, vars.nextVariableIndex());

  vars.insert(0u, {9u, 10u});  // NOLINT

  EXPECT_EQ(11u, vars.nextVariableIndex());
}

TEST(VariableConstraints, GetConstraints)
{
  auto vars = VariableConstraints();

  vars.insert(0u, {0u, 1u, 2u});  // NOLINT
  vars.insert(1u, {0u, 2u});      // NOLINT
  vars.insert(2u, {1u, 2u});      // NOLINT
  vars.insert(3u, {2u, 3u});      // NOLINT

  auto expected0 = std::vector<size_t>{0u, 1u};          // NOLINT
  auto expected1 = std::vector<size_t>{0u, 2u};          // NOLINT
  auto expected2 = std::vector<size_t>{0u, 1u, 2u, 3u};  // NOLINT
  auto expected3 = std::vector<size_t>{3u};              // NOLINT

  std::vector<size_t> actual0;
  vars.getConstraints(0u, std::back_inserter(actual0));
  std::sort(actual0.begin(), actual0.end());

  std::vector<size_t> actual1;
  vars.getConstraints(1u, std::back_inserter(actual1));
  std::sort(actual1.begin(), actual1.end());

  std::vector<size_t> actual2;
  vars.getConstraints(2u, std::back_inserter(actual2));
  std::sort(actual2.begin(), actual2.end());

  std::vector<size_t> actual3;
  vars.getConstraints(3u, std::back_inserter(actual3));
  std::sort(actual3.begin(), actual3.end());

  EXPECT_EQ(expected0, actual0);
  EXPECT_EQ(expected1, actual1);
  EXPECT_EQ(expected2, actual2);
  EXPECT_EQ(expected3, actual3);

  auto actual0_iter = actual0.begin();
  actual0_iter = vars.getConstraints(0u, actual0_iter);

  auto actual1_iter = actual1.begin();
  actual1_iter = vars.getConstraints(1u, actual1_iter);

  auto actual2_iter = actual2.begin();
  actual2_iter = vars.getConstraints(2u, actual2_iter);

  auto actual3_iter = actual3.begin();
  actual3_iter = vars.getConstraints(3u, actual3_iter);

  EXPECT_EQ(
    static_cast<std::ptrdiff_t>(expected0.size()),
    std::distance(actual0.begin(), actual0_iter));
  EXPECT_EQ(
    static_cast<std::ptrdiff_t>(expected1.size()),
    std::distance(actual1.begin(), actual1_iter));
  EXPECT_EQ(
    static_cast<std::ptrdiff_t>(expected2.size()),
    std::distance(actual2.begin(), actual2_iter));
  EXPECT_EQ(
    static_cast<std::ptrdiff_t>(expected3.size()),
    std::distance(actual3.begin(), actual3_iter));
}

TEST(VariableConstraints, InsertOrphanVariable)
{
  auto vars = VariableConstraints();

  vars.insert(0u);

  std::vector<size_t> actual;
  vars.getConstraints(0u, std::back_inserter(actual));

  EXPECT_TRUE(actual.empty());
}
