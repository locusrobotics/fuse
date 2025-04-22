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

#include <functional>
#include <set>
#include <utility>
#include <vector>

#include "example_variable.hpp"
#include <fuse_core/constraint.hpp>
#include <fuse_core/timestamp_manager.hpp>
#include <fuse_core/transaction.hpp>
#include <fuse_core/variable.hpp>
#include <rclcpp/duration.hpp>

/**
 * Test fixture that adds a known set of entries to the timestamp manager. Used to test the
 * interactions with existing entries.
 */
class TimestampManagerTestFixture : public ::testing::Test
{
public:
  TimestampManagerTestFixture()
  : manager(std::bind(&TimestampManagerTestFixture::generator,
      this,
      std::placeholders::_1,
      std::placeholders::_2,
      std::placeholders::_3,
      std::placeholders::_4),
      rclcpp::Duration::max())
  {
  }

  void populate()
  {
    // Add a standard set of entries into the motion model
    fuse_core::Transaction transaction;
    transaction.addInvolvedStamp(rclcpp::Time(10, 0));
    transaction.addInvolvedStamp(rclcpp::Time(20, 0));
    transaction.addInvolvedStamp(rclcpp::Time(30, 0));
    transaction.addInvolvedStamp(rclcpp::Time(40, 0));
    manager.query(transaction);
    generated_time_spans.clear();
  }

  void generator(
    const rclcpp::Time & beginning_stamp,
    const rclcpp::Time & ending_stamp,
    std::vector<fuse_core::Constraint::SharedPtr> & /*constraints*/,
    std::vector<fuse_core::Variable::SharedPtr> & variables)
  {
    generated_time_spans.emplace_back(beginning_stamp, ending_stamp);

    auto variable1 = ExampleVariable::make_shared(beginning_stamp);
    variable1->data()[0] = beginning_stamp.seconds();
    variables.push_back(variable1);

    auto variable2 = ExampleVariable::make_shared(ending_stamp);
    variable2->data()[0] = ending_stamp.seconds();
    variables.push_back(variable2);
  }

  fuse_core::TimestampManager manager;
  std::vector<std::pair<rclcpp::Time, rclcpp::Time>> generated_time_spans;
};


TEST_F(TimestampManagerTestFixture, Empty)
{
  // Test:
  // Existing: |-------------------------------------> t
  // Adding:   |------********-----------------------> t
  // Expected: |------********-----------------------> t

  // Perform a single query
  fuse_core::Transaction transaction;
  transaction.addInvolvedStamp(rclcpp::Time(10, 0));
  transaction.addInvolvedStamp(rclcpp::Time(20, 0));
  manager.query(transaction);

  // Verify the manager contains the timestamps
  auto stamp_range = manager.stamps();
  ASSERT_EQ(2, std::distance(stamp_range.begin(), stamp_range.end()));
  auto stamp_range_iter = stamp_range.begin();
  EXPECT_EQ(rclcpp::Time(10, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(20, 0), *stamp_range_iter);

  // Verify the expected queries were performed
  ASSERT_EQ(1ul, generated_time_spans.size());
  EXPECT_EQ(rclcpp::Time(10, 0), generated_time_spans[0].first);
  EXPECT_EQ(rclcpp::Time(20, 0), generated_time_spans[0].second);
}

TEST_F(TimestampManagerTestFixture, EmptySingleStamp)
{
  // Test:
  // Existing: |-------------------------------------> t
  // Adding:   |------*------------------------------> t
  // Expected: |------*------------------------------> t

  // Perform a single query
  fuse_core::Transaction transaction;
  transaction.addInvolvedStamp(rclcpp::Time(10, 0));
  manager.query(transaction);

  // Verify the manager contains the timestamps
  auto stamp_range = manager.stamps();
  ASSERT_EQ(1, std::distance(stamp_range.begin(), stamp_range.end()));
  auto stamp_range_iter = stamp_range.begin();
  EXPECT_EQ(rclcpp::Time(10, 0), *stamp_range_iter);

  // Verify the expected queries were performed
  ASSERT_EQ(1ul, generated_time_spans.size());
  EXPECT_EQ(rclcpp::Time(10, 0), generated_time_spans[0].first);
  EXPECT_EQ(generated_time_spans[0].first, generated_time_spans[0].second);
}

TEST_F(TimestampManagerTestFixture, Exceptions)
{
  // Set a finite buffer length and populate it with some queries
  manager.bufferLength(rclcpp::Duration::from_seconds(25.0));
  populate();
  // Call the query with a beginning stamp that is too early
  {
    fuse_core::Transaction transaction;
    transaction.addInvolvedStamp(rclcpp::Time(1, 0));
    EXPECT_THROW(manager.query(transaction), std::invalid_argument);
  }
  // Call the query function with a timestamp within the range. This should not throw.
  {
    fuse_core::Transaction transaction;
    transaction.addInvolvedStamp(rclcpp::Time(35, 0));
    EXPECT_NO_THROW(manager.query(transaction));
  }
  // Call the query with a timestamp outside of the buffer length, but within the current timespan
  // of the history This should not throw, as it is safe to perform this operation.
  {
    fuse_core::Transaction transaction;
    transaction.addInvolvedStamp(rclcpp::Time(11, 0));
    EXPECT_NO_THROW(manager.query(transaction));
  }
}

TEST_F(TimestampManagerTestFixture, Purge)
{
  // Set a finite buffer length and populate it with some queries
  manager.bufferLength(rclcpp::Duration(30, 0));
  populate();

  // The timespan is within the specified duration. All entries should still be present.
  // Verify the timestamp manager contains the expected data.
  {
    auto stamp_range = manager.stamps();
    ASSERT_EQ(4, std::distance(stamp_range.begin(), stamp_range.end()));
    auto stamp_range_iter = stamp_range.begin();
    EXPECT_EQ(rclcpp::Time(10, 0), *stamp_range_iter);
    ++stamp_range_iter;
    EXPECT_EQ(rclcpp::Time(20, 0), *stamp_range_iter);
    ++stamp_range_iter;
    EXPECT_EQ(rclcpp::Time(30, 0), *stamp_range_iter);
    ++stamp_range_iter;
    EXPECT_EQ(rclcpp::Time(40, 0), *stamp_range_iter);
  }

  // Add an entry right at the edge of the time range (calculations use the ending stamp).
  // This should still keep all entries.
  {
    fuse_core::Transaction transaction;
    transaction.addInvolvedStamp(rclcpp::Time(50, 0));
    manager.query(transaction);

    auto stamp_range = manager.stamps();
    ASSERT_EQ(5, std::distance(stamp_range.begin(), stamp_range.end()));
    auto stamp_range_iter = stamp_range.begin();
    EXPECT_EQ(rclcpp::Time(10, 0), *stamp_range_iter);
    ++stamp_range_iter;
    EXPECT_EQ(rclcpp::Time(20, 0), *stamp_range_iter);
    ++stamp_range_iter;
    EXPECT_EQ(rclcpp::Time(30, 0), *stamp_range_iter);
    ++stamp_range_iter;
    EXPECT_EQ(rclcpp::Time(40, 0), *stamp_range_iter);
    ++stamp_range_iter;
    EXPECT_EQ(rclcpp::Time(50, 0), *stamp_range_iter);
  }

  // Add another entry. This should cause the oldest entry to be purged.
  {
    fuse_core::Transaction transaction;
    transaction.addInvolvedStamp(rclcpp::Time(50, 1));
    manager.query(transaction);

    auto stamp_range = manager.stamps();
    ASSERT_EQ(5, std::distance(stamp_range.begin(), stamp_range.end()));
    auto stamp_range_iter = stamp_range.begin();
    EXPECT_EQ(rclcpp::Time(20, 0), *stamp_range_iter);
    ++stamp_range_iter;
    EXPECT_EQ(rclcpp::Time(30, 0), *stamp_range_iter);
    ++stamp_range_iter;
    EXPECT_EQ(rclcpp::Time(40, 0), *stamp_range_iter);
    ++stamp_range_iter;
    EXPECT_EQ(rclcpp::Time(50, 0), *stamp_range_iter);
    ++stamp_range_iter;
    EXPECT_EQ(rclcpp::Time(50, 1), *stamp_range_iter);
  }

  // Add a longer entry. This should cause multiple entries to get purged.
  {
    fuse_core::Transaction transaction;
    transaction.addInvolvedStamp(rclcpp::Time(70, 1));
    manager.query(transaction);

    auto stamp_range = manager.stamps();
    ASSERT_EQ(4, std::distance(stamp_range.begin(), stamp_range.end()));
    auto stamp_range_iter = stamp_range.begin();
    EXPECT_EQ(rclcpp::Time(40, 0), *stamp_range_iter);
    ++stamp_range_iter;
    EXPECT_EQ(rclcpp::Time(50, 0), *stamp_range_iter);
    ++stamp_range_iter;
    EXPECT_EQ(rclcpp::Time(50, 1), *stamp_range_iter);
    ++stamp_range_iter;
    EXPECT_EQ(rclcpp::Time(70, 1), *stamp_range_iter);
  }

  // Add a very long entry. This should cause all but two entry to get purged.
  {
    fuse_core::Transaction transaction;
    transaction.addInvolvedStamp(rclcpp::Time(1000, 0));
    manager.query(transaction);

    auto stamp_range = manager.stamps();
    ASSERT_EQ(2, std::distance(stamp_range.begin(), stamp_range.end()));
    auto stamp_range_iter = stamp_range.begin();
    EXPECT_EQ(rclcpp::Time(70, 1), *stamp_range_iter);
    ++stamp_range_iter;
    EXPECT_EQ(rclcpp::Time(1000, 0), *stamp_range_iter);
  }

  // Add an entry far in the future. This should leave only the gap segment and the new segment.
  {
    fuse_core::Transaction transaction;
    transaction.addInvolvedStamp(rclcpp::Time(1100, 0));
    transaction.addInvolvedStamp(rclcpp::Time(1101, 0));
    manager.query(transaction);

    auto stamp_range = manager.stamps();
    ASSERT_EQ(3, std::distance(stamp_range.begin(), stamp_range.end()));
    auto stamp_range_iter = stamp_range.begin();
    EXPECT_EQ(rclcpp::Time(1000, 0), *stamp_range_iter);
    ++stamp_range_iter;
    EXPECT_EQ(rclcpp::Time(1100, 0), *stamp_range_iter);
    ++stamp_range_iter;
    EXPECT_EQ(rclcpp::Time(1101, 0), *stamp_range_iter);
  }
}

TEST_F(TimestampManagerTestFixture, Existing)
{
  // Test:
  // Existing: |------111111112222222233333333-------> t
  // Adding:   |--------------********---------------> t
  // Expected: |------111111112222222233333333-------> t

  populate();
  fuse_core::Transaction transaction;
  transaction.addInvolvedStamp(rclcpp::Time(20, 0));
  transaction.addInvolvedStamp(rclcpp::Time(30, 0));
  manager.query(transaction);

  // Verify the manager contains the expected timestamps
  auto stamp_range = manager.stamps();
  ASSERT_EQ(4, std::distance(stamp_range.begin(), stamp_range.end()));
  auto stamp_range_iter = stamp_range.begin();
  EXPECT_EQ(rclcpp::Time(10, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(20, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(30, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(40, 0), *stamp_range_iter);

  // Verify no new motion model segments were generated
  ASSERT_EQ(0ul, generated_time_spans.size());
}

TEST_F(TimestampManagerTestFixture, BeforeBeginningAligned)
{
  // Test:
  // Existing: |------111111112222222233333333-------> t
  // Adding:   |--****-------------------------------> t
  // Expected: |--****111111112222222233333333-------> t

  populate();
  fuse_core::Transaction transaction;
  transaction.addInvolvedStamp(rclcpp::Time(5, 0));
  transaction.addInvolvedStamp(rclcpp::Time(10, 0));
  manager.query(transaction);

  // Verify the manager contains the expected timestamps
  auto stamp_range = manager.stamps();
  ASSERT_EQ(5, std::distance(stamp_range.begin(), stamp_range.end()));
  auto stamp_range_iter = stamp_range.begin();
  EXPECT_EQ(rclcpp::Time(5, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(10, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(20, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(30, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(40, 0), *stamp_range_iter);

  // Verify the expected new motion model segments were generated
  ASSERT_EQ(1ul, generated_time_spans.size());
  EXPECT_EQ(rclcpp::Time(5, 0), generated_time_spans[0].first);
  EXPECT_EQ(rclcpp::Time(10, 0), generated_time_spans[0].second);
}

TEST_F(TimestampManagerTestFixture, BeforeBeginningUnaligned)
{
  // Test:
  // Existing: |------111111112222222233333333-------> t
  // Adding:   |--***--------------------------------> t
  // Expected: |--***A111111112222222233333333-------> t

  populate();
  fuse_core::Transaction transaction;
  transaction.addInvolvedStamp(rclcpp::Time(5, 0));
  transaction.addInvolvedStamp(rclcpp::Time(8, 0));
  manager.query(transaction);

  // Verify the manager contains the expected timestamps
  auto stamp_range = manager.stamps();
  ASSERT_EQ(6, std::distance(stamp_range.begin(), stamp_range.end()));
  auto stamp_range_iter = stamp_range.begin();
  EXPECT_EQ(rclcpp::Time(5, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(8, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(10, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(20, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(30, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(40, 0), *stamp_range_iter);

  // Verify the expected new motion model segments were generated
  ASSERT_EQ(2ul, generated_time_spans.size());
  EXPECT_EQ(rclcpp::Time(5, 0), generated_time_spans[0].first);
  EXPECT_EQ(rclcpp::Time(8, 0), generated_time_spans[0].second);
  EXPECT_EQ(rclcpp::Time(8, 0), generated_time_spans[1].first);
  EXPECT_EQ(rclcpp::Time(10, 0), generated_time_spans[1].second);
}

TEST_F(TimestampManagerTestFixture, BeforeBeginningOverlap)
{
  // Test:
  // Existing: |------111111112222222233333333-------> t
  // Adding:   |--********---------------------------> t
  // Expected: |--AAAABBBBCCCC2222222233333333-------> t

  populate();
  fuse_core::Transaction transaction;
  transaction.addInvolvedStamp(rclcpp::Time(5, 0));
  transaction.addInvolvedStamp(rclcpp::Time(15, 0));
  manager.query(transaction);

  // Verify the manager contains the expected timestamps
  auto stamp_range = manager.stamps();
  ASSERT_EQ(6, std::distance(stamp_range.begin(), stamp_range.end()));
  auto stamp_range_iter = stamp_range.begin();
  EXPECT_EQ(rclcpp::Time(5, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(10, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(15, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(20, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(30, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(40, 0), *stamp_range_iter);

  // Verify the expected new motion model segments were generated
  ASSERT_EQ(3ul, generated_time_spans.size());
  EXPECT_EQ(rclcpp::Time(5, 0), generated_time_spans[0].first);
  EXPECT_EQ(rclcpp::Time(10, 0), generated_time_spans[0].second);
  EXPECT_EQ(rclcpp::Time(10, 0), generated_time_spans[1].first);
  EXPECT_EQ(rclcpp::Time(15, 0), generated_time_spans[1].second);
  EXPECT_EQ(rclcpp::Time(15, 0), generated_time_spans[2].first);
  EXPECT_EQ(rclcpp::Time(20, 0), generated_time_spans[2].second);
}

TEST_F(TimestampManagerTestFixture, AfterEndAligned)
{
  // Test:
  // Existing: |------111111112222222233333333-------> t
  // Adding:   |------------------------------****---> t
  // Expected: |------111111112222222233333333****---> t

  populate();
  fuse_core::Transaction transaction;
  transaction.addInvolvedStamp(rclcpp::Time(45, 0));
  manager.query(transaction);

  // Verify the manager contains the expected timestamps
  auto stamp_range = manager.stamps();
  ASSERT_EQ(5, std::distance(stamp_range.begin(), stamp_range.end()));
  auto stamp_range_iter = stamp_range.begin();
  EXPECT_EQ(rclcpp::Time(10, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(20, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(30, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(40, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(45, 0), *stamp_range_iter);

  // Verify the expected new motion model segments were generated
  ASSERT_EQ(1ul, generated_time_spans.size());
  EXPECT_EQ(rclcpp::Time(40, 0), generated_time_spans[0].first);
  EXPECT_EQ(rclcpp::Time(45, 0), generated_time_spans[0].second);
}

TEST_F(TimestampManagerTestFixture, AfterEndUnaligned)
{
  // Test:
  // Existing: |------111111112222222233333333-------> t
  // Adding:   |-------------------------------***---> t
  // Expected: |------111111112222222233333333A***---> t

  populate();
  fuse_core::Transaction transaction;
  transaction.addInvolvedStamp(rclcpp::Time(42, 0));
  transaction.addInvolvedStamp(rclcpp::Time(45, 0));
  manager.query(transaction);

  // Verify the manager contains the expected timestamps
  auto stamp_range = manager.stamps();
  ASSERT_EQ(6, std::distance(stamp_range.begin(), stamp_range.end()));
  auto stamp_range_iter = stamp_range.begin();
  EXPECT_EQ(rclcpp::Time(10, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(20, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(30, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(40, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(42, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(45, 0), *stamp_range_iter);

  // Verify the expected new motion model segments were generated
  ASSERT_EQ(2ul, generated_time_spans.size());
  EXPECT_EQ(rclcpp::Time(40, 0), generated_time_spans[0].first);
  EXPECT_EQ(rclcpp::Time(42, 0), generated_time_spans[0].second);
  EXPECT_EQ(rclcpp::Time(42, 0), generated_time_spans[1].first);
  EXPECT_EQ(rclcpp::Time(45, 0), generated_time_spans[1].second);
}

TEST_F(TimestampManagerTestFixture, AfterEndOverlap)
{
  // Test:
  // Existing: |------111111112222222233333333-------> t
  // Adding:   |--------------------------********---> t
  // Expected: |------1111111122222222AAAABBBBCCCC---> t

  populate();
  fuse_core::Transaction transaction;
  transaction.addInvolvedStamp(rclcpp::Time(35, 0));
  transaction.addInvolvedStamp(rclcpp::Time(45, 0));
  manager.query(transaction);

  // Verify the manager contains the expected timestamps
  auto stamp_range = manager.stamps();
  ASSERT_EQ(6, std::distance(stamp_range.begin(), stamp_range.end()));
  auto stamp_range_iter = stamp_range.begin();
  EXPECT_EQ(rclcpp::Time(10, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(20, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(30, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(35, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(40, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(45, 0), *stamp_range_iter);

  // Verify the expected new motion model segments were generated
  ASSERT_EQ(3ul, generated_time_spans.size());
  EXPECT_EQ(rclcpp::Time(30, 0), generated_time_spans[0].first);
  EXPECT_EQ(rclcpp::Time(35, 0), generated_time_spans[0].second);
  EXPECT_EQ(rclcpp::Time(35, 0), generated_time_spans[1].first);
  EXPECT_EQ(rclcpp::Time(40, 0), generated_time_spans[1].second);
  EXPECT_EQ(rclcpp::Time(40, 0), generated_time_spans[2].first);
  EXPECT_EQ(rclcpp::Time(45, 0), generated_time_spans[2].second);
}

TEST_F(TimestampManagerTestFixture, MultiSegment)
{
  // Test:
  // Existing: |------111111112222222233333333-------> t
  // Adding:   |------************************-------> t
  // Expected: |------111111112222222233333333-------> t

  populate();
  fuse_core::Transaction transaction;
  transaction.addInvolvedStamp(rclcpp::Time(10, 0));
  transaction.addInvolvedStamp(rclcpp::Time(40, 0));
  manager.query(transaction);

  // Verify the manager contains the expected timestamps
  auto stamp_range = manager.stamps();
  ASSERT_EQ(4, std::distance(stamp_range.begin(), stamp_range.end()));
  auto stamp_range_iter = stamp_range.begin();
  EXPECT_EQ(rclcpp::Time(10, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(20, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(30, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(40, 0), *stamp_range_iter);

  // Verify the expected new motion model segments were generated
  ASSERT_EQ(0ul, generated_time_spans.size());
}

TEST_F(TimestampManagerTestFixture, MultiSegmentBeforeBeginning)
{
  // Test:
  // Existing: |------111111112222222233333333-------> t
  // Adding:   |--****************************-------> t
  // Expected: |--AAAA111111112222222233333333-------> t

  populate();
  fuse_core::Transaction transaction;
  transaction.addInvolvedStamp(rclcpp::Time(5, 0));
  transaction.addInvolvedStamp(rclcpp::Time(40, 0));
  manager.query(transaction);

  // Verify the manager contains the expected timestamps
  auto stamp_range = manager.stamps();
  ASSERT_EQ(5, std::distance(stamp_range.begin(), stamp_range.end()));
  auto stamp_range_iter = stamp_range.begin();
  EXPECT_EQ(rclcpp::Time(5, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(10, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(20, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(30, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(40, 0), *stamp_range_iter);

  // Verify the expected new motion model segments were generated
  ASSERT_EQ(1ul, generated_time_spans.size());
  EXPECT_EQ(rclcpp::Time(5, 0), generated_time_spans[0].first);
  EXPECT_EQ(rclcpp::Time(10, 0), generated_time_spans[0].second);
}

TEST_F(TimestampManagerTestFixture, MultiSegmentPastEnd)
{
  // Test:
  // Existing: |------111111112222222233333333-------> t
  // Adding:   |------****************************---> t
  // Expected: |------111111112222222233333333AAAA---> t

  populate();
  fuse_core::Transaction transaction;
  transaction.addInvolvedStamp(rclcpp::Time(10, 0));
  transaction.addInvolvedStamp(rclcpp::Time(45, 0));
  manager.query(transaction);

  // Verify the manager contains the expected timestamps
  auto stamp_range = manager.stamps();
  ASSERT_EQ(5, std::distance(stamp_range.begin(), stamp_range.end()));
  auto stamp_range_iter = stamp_range.begin();
  EXPECT_EQ(rclcpp::Time(10, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(20, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(30, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(40, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(45, 0), *stamp_range_iter);

  // Verify the expected new motion model segments were generated
  ASSERT_EQ(1ul, generated_time_spans.size());
  EXPECT_EQ(rclcpp::Time(40, 0), generated_time_spans[0].first);
  EXPECT_EQ(rclcpp::Time(45, 0), generated_time_spans[0].second);
}

TEST_F(TimestampManagerTestFixture, MultiSegmentPastBothEnds)
{
  // Test:
  // Existing: |------111111112222222233333333-------> t
  // Adding:   |--********************************---> t
  // Expected: |--AAAA111111112222222233333333BBBB---> t

  populate();
  fuse_core::Transaction transaction;
  transaction.addInvolvedStamp(rclcpp::Time(5, 0));
  transaction.addInvolvedStamp(rclcpp::Time(45, 0));
  manager.query(transaction);

  // Verify the manager contains the expected timestamps
  auto stamp_range = manager.stamps();
  ASSERT_EQ(6, std::distance(stamp_range.begin(), stamp_range.end()));
  auto stamp_range_iter = stamp_range.begin();
  EXPECT_EQ(rclcpp::Time(5, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(10, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(20, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(30, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(40, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(45, 0), *stamp_range_iter);

  // Verify the expected new motion model segments were generated
  ASSERT_EQ(2ul, generated_time_spans.size());
  EXPECT_EQ(rclcpp::Time(5, 0), generated_time_spans[0].first);
  EXPECT_EQ(rclcpp::Time(10, 0), generated_time_spans[0].second);
  EXPECT_EQ(rclcpp::Time(40, 0), generated_time_spans[1].first);
  EXPECT_EQ(rclcpp::Time(45, 0), generated_time_spans[1].second);
}

TEST_F(TimestampManagerTestFixture, SplitBeginning)
{
  // Test:
  // Existing: |------111111112222222233333333-------> t
  // Adding:   |----------************---------------> t
  // Expected: |------AAAABBBB2222222233333333-------> t

  populate();
  fuse_core::Transaction transaction;
  transaction.addInvolvedStamp(rclcpp::Time(15, 0));
  transaction.addInvolvedStamp(rclcpp::Time(30, 0));
  manager.query(transaction);

  // Verify the manager contains the expected timestamps
  auto stamp_range = manager.stamps();
  ASSERT_EQ(5, std::distance(stamp_range.begin(), stamp_range.end()));
  auto stamp_range_iter = stamp_range.begin();
  EXPECT_EQ(rclcpp::Time(10, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(15, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(20, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(30, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(40, 0), *stamp_range_iter);

  // Verify the expected new motion model segments were generated
  ASSERT_EQ(2ul, generated_time_spans.size());
  EXPECT_EQ(rclcpp::Time(10, 0), generated_time_spans[0].first);
  EXPECT_EQ(rclcpp::Time(15, 0), generated_time_spans[0].second);
  EXPECT_EQ(rclcpp::Time(15, 0), generated_time_spans[1].first);
  EXPECT_EQ(rclcpp::Time(20, 0), generated_time_spans[1].second);
}

TEST_F(TimestampManagerTestFixture, SplitEnd)
{
  // Test:
  // Existing: |------111111112222222233333333-------> t
  // Adding:   |--------------************-----------> t
  // Expected: |------1111111122222222AAAABBBB-------> t

  populate();
  fuse_core::Transaction transaction;
  transaction.addInvolvedStamp(rclcpp::Time(20, 0));
  transaction.addInvolvedStamp(rclcpp::Time(35, 0));
  manager.query(transaction);

  // Verify the manager contains the expected timestamps
  auto stamp_range = manager.stamps();
  ASSERT_EQ(5, std::distance(stamp_range.begin(), stamp_range.end()));
  auto stamp_range_iter = stamp_range.begin();
  EXPECT_EQ(rclcpp::Time(10, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(20, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(30, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(35, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(40, 0), *stamp_range_iter);

  // Verify the expected new motion model segments were generated
  ASSERT_EQ(2ul, generated_time_spans.size());
  EXPECT_EQ(rclcpp::Time(30, 0), generated_time_spans[0].first);
  EXPECT_EQ(rclcpp::Time(35, 0), generated_time_spans[0].second);
  EXPECT_EQ(rclcpp::Time(35, 0), generated_time_spans[1].first);
  EXPECT_EQ(rclcpp::Time(40, 0), generated_time_spans[1].second);
}

TEST_F(TimestampManagerTestFixture, SplitBoth)
{
  // Test:
  // Existing: |------111111112222222233333333-------> t
  // Adding:   |----------****************-----------> t
  // Expected: |------AAAABBBB22222222CCCCDDDD-------> t

  populate();
  fuse_core::Transaction transaction;
  transaction.addInvolvedStamp(rclcpp::Time(15, 0));
  transaction.addInvolvedStamp(rclcpp::Time(35, 0));
  manager.query(transaction);

  // Verify the manager contains the expected timestamps
  auto stamp_range = manager.stamps();
  ASSERT_EQ(6, std::distance(stamp_range.begin(), stamp_range.end()));
  auto stamp_range_iter = stamp_range.begin();
  EXPECT_EQ(rclcpp::Time(10, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(15, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(20, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(30, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(35, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(40, 0), *stamp_range_iter);

  // Verify the expected new motion model segments were generated
  ASSERT_EQ(4ul, generated_time_spans.size());
  EXPECT_EQ(rclcpp::Time(10, 0), generated_time_spans[0].first);
  EXPECT_EQ(rclcpp::Time(15, 0), generated_time_spans[0].second);
  EXPECT_EQ(rclcpp::Time(15, 0), generated_time_spans[1].first);
  EXPECT_EQ(rclcpp::Time(20, 0), generated_time_spans[1].second);
  EXPECT_EQ(rclcpp::Time(30, 0), generated_time_spans[2].first);
  EXPECT_EQ(rclcpp::Time(35, 0), generated_time_spans[2].second);
  EXPECT_EQ(rclcpp::Time(35, 0), generated_time_spans[3].first);
  EXPECT_EQ(rclcpp::Time(40, 0), generated_time_spans[3].second);
}

TEST_F(TimestampManagerTestFixture, SplitSame)
{
  // Test:
  // Existing: |------111111112222222233333333-------> t
  // Adding:   |----------------****-----------------> t
  // Expected: |------11111111AABBBBCC33333333-------> t

  populate();
  fuse_core::Transaction transaction;
  transaction.addInvolvedStamp(rclcpp::Time(22, 0));
  transaction.addInvolvedStamp(rclcpp::Time(27, 0));
  manager.query(transaction);

  // Verify the manager contains the expected timestamps
  auto stamp_range = manager.stamps();
  ASSERT_EQ(6, std::distance(stamp_range.begin(), stamp_range.end()));
  auto stamp_range_iter = stamp_range.begin();
  EXPECT_EQ(rclcpp::Time(10, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(20, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(22, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(27, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(30, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(40, 0), *stamp_range_iter);

  // Verify the expected new motion model segments were generated
  ASSERT_EQ(3ul, generated_time_spans.size());
  EXPECT_EQ(rclcpp::Time(20, 0), generated_time_spans[0].first);
  EXPECT_EQ(rclcpp::Time(22, 0), generated_time_spans[0].second);
  EXPECT_EQ(rclcpp::Time(22, 0), generated_time_spans[1].first);
  EXPECT_EQ(rclcpp::Time(27, 0), generated_time_spans[1].second);
  EXPECT_EQ(rclcpp::Time(27, 0), generated_time_spans[2].first);
  EXPECT_EQ(rclcpp::Time(30, 0), generated_time_spans[2].second);
}

TEST_F(TimestampManagerTestFixture, SplitSameMultiple)
{
  // Test:
  // Existing: |------111111112222222233333333-------> t
  // Adding:   |----------------**%%#----------------> t
  // Expected: |------11111111AABBCCDE33333333-------> t

  populate();
  fuse_core::Transaction transaction;
  transaction.addInvolvedStamp(rclcpp::Time(22, 0));
  transaction.addInvolvedStamp(rclcpp::Time(25, 0));
  transaction.addInvolvedStamp(rclcpp::Time(27, 0));
  transaction.addInvolvedStamp(rclcpp::Time(29, 0));
  manager.query(transaction);

  // Verify the manager contains the expected timestamps
  auto stamp_range = manager.stamps();
  ASSERT_EQ(8, std::distance(stamp_range.begin(), stamp_range.end()));
  auto stamp_range_iter = stamp_range.begin();
  EXPECT_EQ(rclcpp::Time(10, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(20, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(22, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(25, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(27, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(29, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(30, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(40, 0), *stamp_range_iter);

  // Verify the expected new motion model segments were generated
  ASSERT_EQ(5ul, generated_time_spans.size());
  EXPECT_EQ(rclcpp::Time(20, 0), generated_time_spans[0].first);
  EXPECT_EQ(rclcpp::Time(22, 0), generated_time_spans[0].second);
  EXPECT_EQ(rclcpp::Time(22, 0), generated_time_spans[1].first);
  EXPECT_EQ(rclcpp::Time(25, 0), generated_time_spans[1].second);
  EXPECT_EQ(rclcpp::Time(25, 0), generated_time_spans[2].first);
  EXPECT_EQ(rclcpp::Time(27, 0), generated_time_spans[2].second);
  EXPECT_EQ(rclcpp::Time(27, 0), generated_time_spans[3].first);
  EXPECT_EQ(rclcpp::Time(29, 0), generated_time_spans[3].second);
  EXPECT_EQ(rclcpp::Time(29, 0), generated_time_spans[4].first);
  EXPECT_EQ(rclcpp::Time(30, 0), generated_time_spans[4].second);
}

TEST_F(TimestampManagerTestFixture, PriorOnLast)
{
  // Test:
  // Existing: |------111111112222222233333333-------> t
  // Adding:   |-----------------------------*-------> t
  // Expected: |------111111112222222233333333-------> t
  // In Issue 300, it was reported that adding a transaction that involves only the last
  // variable does not properly update the variable value in the input transaction.

  populate();
  fuse_core::Transaction transaction;
  transaction.addInvolvedStamp(rclcpp::Time(40, 0));
  // Add a variable for the last timestamp and set the value to something known. We expect
  // the query() to replace the variable value with the whatever was generated by the
  // motion model. It this test case, the variable value will be the timestamp converted
  // to a floating point value.
  auto variable1 = ExampleVariable::make_shared(rclcpp::Time(40, 0));
  variable1->data()[0] = -99.0;
  transaction.addVariable(variable1);
  // Request that the input variables be updated with the manager's variable values.
  manager.query(transaction, true);

  // Verify the manager contains the expected timestamps
  auto stamp_range = manager.stamps();
  ASSERT_EQ(4, std::distance(stamp_range.begin(), stamp_range.end()));
  auto stamp_range_iter = stamp_range.begin();
  EXPECT_EQ(rclcpp::Time(10, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(20, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(30, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(40, 0), *stamp_range_iter);

  // Verify no new motion model segments were generated
  ASSERT_EQ(0ul, generated_time_spans.size());

  // Verify the input variable value was updated to the timestamp manager's value
  // (40.0 in this case)
  for (auto && variable : transaction.addedVariables()) {
    if (variable.uuid() == variable1->uuid()) {
      ASSERT_DOUBLE_EQ(40.0, variable.data()[0]) << "Transaction contains:\n" << transaction;
    } else {
      FAIL() << "Unexpected added variable:\n" << variable;
    }
  }
}

TEST_F(TimestampManagerTestFixture, DeltaOnLast)
{
  // Test:
  // Existing: |------111111112222222233333333-------> t
  // Adding:   |----------------------********-------> t
  // Expected: |------111111112222222233333333-------> t
  // In Issue 300, it was reported that adding a transaction that involves only the last
  // variable does not properly update the variable value in the input transaction.
  // Verify that adding a transaction that involves two timestamps does update the
  // variable values as expected.

  populate();
  fuse_core::Transaction transaction;
  transaction.addInvolvedStamp(rclcpp::Time(30, 0));
  transaction.addInvolvedStamp(rclcpp::Time(40, 0));
  // Add a variable for the last timestamp and set the value to something known. We expect
  // the query() to replace the variable value with the whatever was generated by the
  // motion model. It this test case, the variable value will be the timestamp converted
  // to a floating point value.
  auto variable1 = ExampleVariable::make_shared(rclcpp::Time(30, 0));
  variable1->data()[0] = -98.0;
  transaction.addVariable(variable1);
  auto variable2 = ExampleVariable::make_shared(rclcpp::Time(40, 0));
  variable2->data()[0] = -99.0;
  transaction.addVariable(variable2);
  // Request that the input variables be updated with the manager's variable values.
  manager.query(transaction, true);

  // Verify the manager contains the expected timestamps
  auto stamp_range = manager.stamps();
  ASSERT_EQ(4, std::distance(stamp_range.begin(), stamp_range.end()));
  auto stamp_range_iter = stamp_range.begin();
  EXPECT_EQ(rclcpp::Time(10, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(20, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(30, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(rclcpp::Time(40, 0), *stamp_range_iter);

  // Verify no new motion model segments were generated
  ASSERT_EQ(0ul, generated_time_spans.size());

  // Verify the input variable values were updated to the timestamp manager's values
  // (30.0 and 40.0 in this case)
  for (auto && variable : transaction.addedVariables()) {
    if (variable.uuid() == variable1->uuid()) {
      ASSERT_DOUBLE_EQ(30.0, variable.data()[0]) << "Transaction contains:\n" << transaction;
    } else if (variable.uuid() == variable2->uuid()) {
      ASSERT_DOUBLE_EQ(40.0, variable.data()[0]) << "Transaction contains:\n" << transaction;
    } else {
      FAIL() << "Unexpected added variable: " << variable;
    }
  }
}
