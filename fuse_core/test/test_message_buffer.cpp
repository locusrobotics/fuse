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
#include <fuse_core/message_buffer.h>
#include <rclcpp/duration.hpp>
#include <fuse_core/time.h>

#include <gtest/gtest.h>


/**
 * Test fixture that adds a known set of entries to the timestamp manager.
 * Used to test the interactions with existing entries.
 */
class MessageBufferTestFixture : public ::testing::Test
{
public:
  MessageBufferTestFixture() :
    buffer(rclcpp::Duration::max())
  {
  }

  void populate()
  {
    // Add a standard set of entries into the motion model
    buffer.insert(rclcpp::Time(10, 0), 1);
    buffer.insert(rclcpp::Time(20, 0), 2);
    buffer.insert(rclcpp::Time(30, 0), 3);
    buffer.insert(rclcpp::Time(40, 0), 4);
  }

  fuse_core::MessageBuffer<int> buffer;
};

TEST_F(MessageBufferTestFixture, Exceptions)
{
  // Call the query with the parameters in the wrong order. This should throw.
  EXPECT_THROW(buffer.query(rclcpp::Time(20, 0), rclcpp::Time(10, 0), false), std::invalid_argument);

  // Call the query when the buffer is empty. This should throw.
  EXPECT_THROW(buffer.query(rclcpp::Time(10, 0), rclcpp::Time(25, 0), false), std::out_of_range);

  populate();

  // Call the query with a beginning stamp that is too early
  EXPECT_THROW(buffer.query(rclcpp::Time(1, 0), rclcpp::Time(25, 0), false), std::out_of_range);

  // Call the query function with a timestamp within the range. This should not throw.
  EXPECT_NO_THROW(buffer.query(rclcpp::Time(20, 0), rclcpp::Time(30, 0)));
}

TEST_F(MessageBufferTestFixture, StandardRangeAligned)
{
  // Query the buffer with the standard range flag, where the query boundaries line up exactly with existing elements
  populate();
  auto msg_range = buffer.query(rclcpp::Time(10, 0), rclcpp::Time(30, 0), false);
  // Verify the returned message range contains the correct entries
  ASSERT_EQ(1, std::distance(msg_range.begin(), msg_range.end()));
  auto msg_range_iter = msg_range.begin();
  EXPECT_EQ(rclcpp::Time(20, 0), msg_range_iter->first);
  EXPECT_EQ(2, msg_range_iter->second);
}

TEST_F(MessageBufferTestFixture, StandardRangeUnaligned)
{
  // Query the buffer with the standard range flag, where the query boundaries do not line up with existing elements
  populate();
  auto msg_range = buffer.query(rclcpp::Time(15, 0), rclcpp::Time(25, 0), false);
  // Verify the returned message range contains the correct entries
  ASSERT_EQ(1, std::distance(msg_range.begin(), msg_range.end()));
  auto msg_range_iter = msg_range.begin();
  EXPECT_EQ(rclcpp::Time(20, 0), msg_range_iter->first);
  EXPECT_EQ(2, msg_range_iter->second);
}

TEST_F(MessageBufferTestFixture, ExtendedRangeAligned)
{
  // Query the buffer with the extended range flag, where the query boundaries line up exactly with existing elements
  populate();
  auto msg_range = buffer.query(rclcpp::Time(10, 0), rclcpp::Time(30, 0), true);
  // Verify the returned message range contains the correct entries
  ASSERT_EQ(3, std::distance(msg_range.begin(), msg_range.end()));
  auto msg_range_iter = msg_range.begin();
  EXPECT_EQ(rclcpp::Time(10, 0), msg_range_iter->first);
  EXPECT_EQ(1, msg_range_iter->second);
  ++msg_range_iter;
  EXPECT_EQ(rclcpp::Time(20, 0), msg_range_iter->first);
  EXPECT_EQ(2, msg_range_iter->second);
  ++msg_range_iter;
  EXPECT_EQ(rclcpp::Time(30, 0), msg_range_iter->first);
  EXPECT_EQ(3, msg_range_iter->second);
}

TEST_F(MessageBufferTestFixture, ExtendedRangeUnaligned)
{
  // Query the buffer with the extended range flag, where the query boundaries do not line up with existing elements
  populate();
  auto msg_range = buffer.query(rclcpp::Time(15, 0), rclcpp::Time(25, 0), true);
  // Verify the returned message range contains the correct entries
  ASSERT_EQ(3, std::distance(msg_range.begin(), msg_range.end()));
  auto msg_range_iter = msg_range.begin();
  EXPECT_EQ(rclcpp::Time(10, 0), msg_range_iter->first);
  EXPECT_EQ(1, msg_range_iter->second);
  ++msg_range_iter;
  EXPECT_EQ(rclcpp::Time(20, 0), msg_range_iter->first);
  EXPECT_EQ(2, msg_range_iter->second);
  ++msg_range_iter;
  EXPECT_EQ(rclcpp::Time(30, 0), msg_range_iter->first);
  EXPECT_EQ(3, msg_range_iter->second);
}

TEST_F(MessageBufferTestFixture, Purge)
{
  // Verify the finite buffer length purges old data correctly

  // Set a finite buffer length and populate it with some queries
  buffer.bufferLength(rclcpp::Duration::from_seconds(30.0));
  populate();

  // Verify the buffer contains the expected data.
  {
    auto stamps_range = buffer.stamps();
    ASSERT_EQ(4, std::distance(stamps_range.begin(), stamps_range.end()));
    auto stamps_range_iter = stamps_range.begin();
    EXPECT_EQ(rclcpp::Time(10, 0), *stamps_range_iter);
    ++stamps_range_iter;
    EXPECT_EQ(rclcpp::Time(20, 0), *stamps_range_iter);
    ++stamps_range_iter;
    EXPECT_EQ(rclcpp::Time(30, 0), *stamps_range_iter);
    ++stamps_range_iter;
    EXPECT_EQ(rclcpp::Time(40, 0), *stamps_range_iter);
  }

  // Add a new entry in the future. Verify the oldest entry is removed.
  {
    buffer.insert(rclcpp::Time(50, 0), 5);
    auto stamps_range = buffer.stamps();
    ASSERT_EQ(4, std::distance(stamps_range.begin(), stamps_range.end()));
    auto stamps_range_iter = stamps_range.begin();
    EXPECT_EQ(rclcpp::Time(20, 0), *stamps_range_iter);
    ++stamps_range_iter;
    EXPECT_EQ(rclcpp::Time(30, 0), *stamps_range_iter);
    ++stamps_range_iter;
    EXPECT_EQ(rclcpp::Time(40, 0), *stamps_range_iter);
    ++stamps_range_iter;
    EXPECT_EQ(rclcpp::Time(50, 0), *stamps_range_iter);
  }

  // Add a longer entry. This should cause multiple entries to get purged.
  {
    buffer.insert(rclcpp::Time(70, 0), 6);
    auto stamps_range = buffer.stamps();
    ASSERT_EQ(3, std::distance(stamps_range.begin(), stamps_range.end()));
    auto stamps_range_iter = stamps_range.begin();
    EXPECT_EQ(rclcpp::Time(40, 0), *stamps_range_iter);
    ++stamps_range_iter;
    EXPECT_EQ(rclcpp::Time(50, 0), *stamps_range_iter);
    ++stamps_range_iter;
    EXPECT_EQ(rclcpp::Time(70, 0), *stamps_range_iter);
  }

  // Add a very long entry. This should cause all but two entry to get purged.
  {
    buffer.insert(rclcpp::Time(1000, 0), 7);
    auto stamps_range = buffer.stamps();
    ASSERT_EQ(2, std::distance(stamps_range.begin(), stamps_range.end()));
    auto stamps_range_iter = stamps_range.begin();
    EXPECT_EQ(rclcpp::Time(70, 0), *stamps_range_iter);
    ++stamps_range_iter;
    EXPECT_EQ(rclcpp::Time(1000, 0), *stamps_range_iter);
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
