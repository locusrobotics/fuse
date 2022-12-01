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

#include <vector>

#include "example_constraint.hpp"
#include <fuse_core/uuid.hpp>

TEST(Constraint, Constructor)
{
  // Create a constraint with a single UUID
  {
    fuse_core::UUID variable_uuid1 = fuse_core::uuid::generate();
    ExampleConstraint constraint("test", {variable_uuid1});  // NOLINT
    ASSERT_EQ(1u, constraint.variables().size());
    ASSERT_EQ(variable_uuid1, constraint.variables().at(0));
  }
  // Create a constraint with multiple UUIDs
  {
    fuse_core::UUID variable_uuid1 = fuse_core::uuid::generate();
    fuse_core::UUID variable_uuid2 = fuse_core::uuid::generate();
    fuse_core::UUID variable_uuid3 = fuse_core::uuid::generate();
    ExampleConstraint constraint("test", {variable_uuid1, variable_uuid2, variable_uuid3});  // NOLINT
    ASSERT_EQ(3u, constraint.variables().size());
    ASSERT_EQ(variable_uuid1, constraint.variables().at(0));
    ASSERT_EQ(variable_uuid2, constraint.variables().at(1));
    ASSERT_EQ(variable_uuid3, constraint.variables().at(2));
  }
  // Create a constraint using the iterator constructor
  {
    std::vector<fuse_core::UUID> variable_uuids;
    variable_uuids.push_back(fuse_core::uuid::generate());
    variable_uuids.push_back(fuse_core::uuid::generate());
    variable_uuids.push_back(fuse_core::uuid::generate());
    variable_uuids.push_back(fuse_core::uuid::generate());
    ExampleConstraint constraint("test", variable_uuids.begin(), variable_uuids.end());
    ASSERT_EQ(variable_uuids.size(), constraint.variables().size());
    for (size_t i = 0; i < variable_uuids.size(); ++i) {
      ASSERT_EQ(variable_uuids.at(i), constraint.variables().at(i));
    }
  }
  // Copy constructor
  {
    fuse_core::UUID variable_uuid1 = fuse_core::uuid::generate();
    fuse_core::UUID variable_uuid2 = fuse_core::uuid::generate();
    fuse_core::UUID variable_uuid3 = fuse_core::uuid::generate();
    ExampleConstraint constraint1("test", {variable_uuid1, variable_uuid2, variable_uuid3});  // NOLINT
    ExampleConstraint constraint2(constraint1);

    ASSERT_EQ(constraint1.uuid(), constraint2.uuid());
    ASSERT_EQ(constraint1.variables().size(), constraint2.variables().size());
    for (size_t i = 0; i < constraint1.variables().size(); ++i) {
      ASSERT_EQ(constraint1.variables().at(i), constraint2.variables().at(i));
    }
  }
}

TEST(Constraint, Type)
{
  fuse_core::UUID variable_uuid1 = fuse_core::uuid::generate();
  ExampleConstraint constraint("test", {variable_uuid1});  // NOLINT
  ASSERT_EQ("ExampleConstraint", constraint.type());
}
