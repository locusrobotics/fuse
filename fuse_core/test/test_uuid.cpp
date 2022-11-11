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
#include <fuse_core/time.h>

#include <gtest/gtest.h>

#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

using fuse_core::UUID;
using UUIDs = std::vector<fuse_core::UUID>;


TEST(UUID, Generate)
{
  // These tests are mostly just calling the different generate() signatures to verify they compile and work.
  // It's hard to validate that the "correct" random number has been generated. :)

  // Just get a random number
  {
    UUID id1 = fuse_core::uuid::generate();
    UUID id2 = fuse_core::uuid::generate();
    ASSERT_NE(id1, id2);
  }
  // Generate a UUID from a data buffer. The same buffer contents should always generate the same UUID.
  {
    std::string buffer1 = "Curse your sudden but inevitable betrayal!";
    std::string buffer2 = "Man walks down the street in a hat like that, you know he's not afraid of anything.";
    UUID id1 = fuse_core::uuid::generate(buffer1.data(), buffer1.size());
    UUID id2 = fuse_core::uuid::generate(buffer1.data(), buffer1.size());
    UUID id3 = fuse_core::uuid::generate(buffer2.data(), buffer2.size());
    ASSERT_EQ(id1, id2);
    ASSERT_NE(id1, id3);

    UUID id4 = fuse_core::uuid::generate(buffer1.c_str());
    UUID id5 = fuse_core::uuid::generate(buffer1.c_str());
    UUID id6 = fuse_core::uuid::generate(buffer2.c_str());
    ASSERT_EQ(id4, id1);
    ASSERT_EQ(id4, id5);
    ASSERT_NE(id4, id6);

    UUID id7 = fuse_core::uuid::generate(buffer1);
    UUID id8 = fuse_core::uuid::generate(buffer1);
    UUID id9 = fuse_core::uuid::generate(buffer2);
    ASSERT_EQ(id7, id1);
    ASSERT_EQ(id7, id8);
    ASSERT_NE(id7, id9);
  }
  // Generate a UUID from a namespace and a data buffer. The same name and buffer should always generate the same UUID.
  {
    std::string name1 = "Jayne";
    std::string name2 = "Hoban";
    std::string buffer1 = "Ten percent of nothing is, let me do the math here. Nothing into nothin'. Carry the nothin'";
    std::string buffer2 = "Some people juggle geese.";

    UUID id1 = fuse_core::uuid::generate(name1, buffer1.data(), buffer1.size());
    UUID id2 = fuse_core::uuid::generate(name1, buffer1.data(), buffer1.size());
    UUID id3 = fuse_core::uuid::generate(name1, buffer2.data(), buffer2.size());
    UUID id4 = fuse_core::uuid::generate(name2, buffer2.data(), buffer2.size());
    ASSERT_EQ(id1, id2);
    ASSERT_NE(id1, id3);
    ASSERT_NE(id1, id4);

    UUID id5 = fuse_core::uuid::generate(name1, buffer1.c_str());
    UUID id6 = fuse_core::uuid::generate(name1, buffer1.c_str());
    UUID id7 = fuse_core::uuid::generate(name1, buffer2.c_str());
    UUID id8 = fuse_core::uuid::generate(name2, buffer2.c_str());
    ASSERT_EQ(id5, id1);
    ASSERT_EQ(id5, id6);
    ASSERT_NE(id5, id7);
    ASSERT_NE(id5, id8);

    UUID id9 = fuse_core::uuid::generate(name1, buffer1);
    UUID id10 = fuse_core::uuid::generate(name1, buffer1);
    UUID id11 = fuse_core::uuid::generate(name1, buffer2);
    UUID id12 = fuse_core::uuid::generate(name2, buffer2);
    ASSERT_EQ(id9, id1);
    ASSERT_EQ(id9, id10);
    ASSERT_NE(id9, id11);
    ASSERT_NE(id9, id12);
  }
  // Generate a UUID from a namespace and a rclcpp::Time
  {
    std::string name1 = "Cobb";
    std::string name2 = "Frye";
    rclcpp::Time stamp1(1234, 5678);
    rclcpp::Time stamp2(1235, 5678);

    UUID id1 = fuse_core::uuid::generate(name1, stamp1);
    UUID id2 = fuse_core::uuid::generate(name1, stamp1);
    UUID id3 = fuse_core::uuid::generate(name1, stamp2);
    UUID id4 = fuse_core::uuid::generate(name2, stamp2);
    ASSERT_EQ(id1, id2);
    ASSERT_NE(id1, id3);
    ASSERT_NE(id1, id4);
  }
  // Generate a UUID from a namespace, a rclcpp::Time, and another UUID
  {
    std::string name1 = "Book";
    std::string name2 = "Tam";
    rclcpp::Time stamp1(1234, 5678);
    rclcpp::Time stamp2(1235, 5678);
    UUID uuid1 = fuse_core::uuid::generate();
    UUID uuid2 = fuse_core::uuid::generate();

    UUID id1 = fuse_core::uuid::generate(name1, stamp1, uuid1);
    UUID id2 = fuse_core::uuid::generate(name1, stamp1, uuid1);
    UUID id3 = fuse_core::uuid::generate(name2, stamp1, uuid1);
    UUID id4 = fuse_core::uuid::generate(name1, stamp2, uuid1);
    UUID id5 = fuse_core::uuid::generate(name1, stamp1, uuid2);
    ASSERT_EQ(id1, id2);
    ASSERT_NE(id1, id3);
    ASSERT_NE(id1, id4);
    ASSERT_NE(id1, id5);
  }
  // Generate a UUID from a namespace, and a uint64_t
  {
    std::string name1 = "McLaughlin";
    std::string name2 = "Aero";
    uint64_t user_id1 = 0;
    uint64_t user_id2 = 1;

    UUID id1 = fuse_core::uuid::generate(name1, user_id1);
    UUID id2 = fuse_core::uuid::generate(name1, user_id1);
    UUID id3 = fuse_core::uuid::generate(name2, user_id1);
    UUID id4 = fuse_core::uuid::generate(name1, user_id2);
    ASSERT_EQ(id1, id2);
    ASSERT_NE(id1, id3);
    ASSERT_NE(id1, id4);
  }
}

void generateUUIDs(UUIDs& uuids)
{
  constexpr size_t uuid_count = 100000;
  uuids.reserve(uuid_count);
  for (size_t i = 0; i < uuid_count; ++i)
  {
    auto uuid = fuse_core::uuid::generate();
    uuids.push_back(uuid);
  }
}

TEST(UUID, CollisionSingleThread)
{
  // Create many UUIDs
  UUIDs raw_uuids;
  generateUUIDs(raw_uuids);

  // Check for duplicates
  std::unordered_set<fuse_core::UUID> unique_uuids;
  for (const auto& uuid : raw_uuids)
  {
    ASSERT_TRUE(unique_uuids.find(uuid) == unique_uuids.end()) << "UUIDs before duplicate " << unique_uuids.size();
    unique_uuids.insert(uuid);
  }
}

TEST(UUID, CollisionManyThreads)
{
  // Create many UUIDs in several threads
  constexpr size_t thread_count = 12;
  std::vector<UUIDs> raw_uuids(thread_count);
  std::vector<std::thread> threads(thread_count);
  for (size_t i = 0; i < threads.size(); ++i)
  {
    threads[i] = std::thread(generateUUIDs, std::ref(raw_uuids[i]));
  }
  for (size_t i = 0; i < threads.size(); ++i)
  {
    threads[i].join();
  }

  // Check for duplicates
  std::unordered_set<fuse_core::UUID> unique_uuids;
  for (size_t i = 0; i < raw_uuids.size(); ++i)
  {
    for (const auto& uuid : raw_uuids[i])
    {
      ASSERT_TRUE(unique_uuids.find(uuid) == unique_uuids.end()) << "UUIDs before duplicate " << unique_uuids.size();
      unique_uuids.insert(uuid);
    }
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
