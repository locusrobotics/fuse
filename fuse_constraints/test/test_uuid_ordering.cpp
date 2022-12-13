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

#include <vector>

#include <fuse_constraints/uuid_ordering.hpp>
#include <fuse_core/uuid.hpp>

using fuse_constraints::UuidOrdering;

TEST(UuidOrdering, Constructor)
{
  // Default constructor
  EXPECT_NO_THROW(UuidOrdering());

  // Iterators
  std::vector<fuse_core::UUID> uuids{fuse_core::uuid::generate(), fuse_core::uuid::generate()};
  EXPECT_NO_THROW(UuidOrdering(uuids.begin(), uuids.end()));

  // Initializer List
  EXPECT_NO_THROW(UuidOrdering({fuse_core::uuid::generate(), fuse_core::uuid::generate()}));  // NOLINT
}

TEST(UuidOrdering, Access)
{
  auto uuid1 = fuse_core::uuid::generate();
  auto uuid2 = fuse_core::uuid::generate();
  auto uuid3 = fuse_core::uuid::generate();
  auto uuid4 = fuse_core::uuid::generate();
  auto order = UuidOrdering{uuid1, uuid2, uuid3};

  EXPECT_EQ(0u, order.at(uuid1));
  EXPECT_EQ(1u, order.at(uuid2));
  EXPECT_EQ(2u, order.at(uuid3));
  EXPECT_EQ(uuid1, order.at(0u));
  EXPECT_EQ(uuid2, order.at(1u));
  EXPECT_EQ(uuid3, order.at(2u));

  EXPECT_THROW(order.at(3u), std::out_of_range);
  EXPECT_THROW(order.at(uuid4), std::out_of_range);

  EXPECT_EQ(uuid1, order[0u]);
  EXPECT_EQ(uuid2, order[1u]);
  EXPECT_EQ(uuid3, order[2u]);
  EXPECT_EQ(0u, order[uuid1]);
  EXPECT_EQ(1u, order[uuid2]);
  EXPECT_EQ(2u, order[uuid3]);

  EXPECT_EQ(3u, order[uuid4]);
}

TEST(UuidOrdering, PushBack)
{
  auto uuid1 = fuse_core::uuid::generate();
  auto uuid2 = fuse_core::uuid::generate();
  auto uuid3 = fuse_core::uuid::generate();
  auto uuid4 = fuse_core::uuid::generate();
  auto order = UuidOrdering{uuid1, uuid2, uuid3};

  EXPECT_EQ(3u, order.size());
  EXPECT_FALSE(order.push_back(uuid3));
  EXPECT_TRUE(order.push_back(uuid4));
  EXPECT_EQ(4u, order.size());
  EXPECT_EQ(3u, order.at(uuid4));
  EXPECT_EQ(uuid4, order.at(3u));
  EXPECT_EQ(uuid4, order[3u]);
}

TEST(UuidOrdering, Size)
{
  auto order = UuidOrdering();

  EXPECT_TRUE(order.empty());
  EXPECT_EQ(0u, order.size());

  auto uuid1 = fuse_core::uuid::generate();
  order.push_back(uuid1);

  EXPECT_FALSE(order.empty());
  EXPECT_EQ(1u, order.size());
}

TEST(UuidOrdering, Exists)
{
  auto uuid1 = fuse_core::uuid::generate();
  auto uuid2 = fuse_core::uuid::generate();
  auto uuid3 = fuse_core::uuid::generate();
  auto uuid4 = fuse_core::uuid::generate();
  auto order = UuidOrdering{uuid1, uuid2, uuid3};

  EXPECT_TRUE(order.exists(0));
  EXPECT_TRUE(order.exists(1));
  EXPECT_TRUE(order.exists(2));
  EXPECT_FALSE(order.exists(3));

  EXPECT_TRUE(order.exists(uuid1));
  EXPECT_TRUE(order.exists(uuid2));
  EXPECT_TRUE(order.exists(uuid3));
  EXPECT_FALSE(order.exists(uuid4));
}
