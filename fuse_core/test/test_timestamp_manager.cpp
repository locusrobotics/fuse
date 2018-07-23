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
#include <fuse_core/constraint.h>
#include <fuse_core/timestamp_manager.h>
#include <fuse_core/transaction.h>
#include <fuse_core/variable.h>
#include <ros/duration.h>
#include <ros/time.h>

#include <gtest/gtest.h>

#include <functional>
#include <set>
#include <utility>
#include <vector>


/**
 * Test fixture that adds a known set of entries to the timestamp manager.
 * Used to test the interactions with existing entries.
 */
class TimestampManagerTestFixture : public ::testing::Test
{
public:
  TimestampManagerTestFixture() :
    manager(std::bind(&TimestampManagerTestFixture::generator,
                      this,
                      std::placeholders::_1,
                      std::placeholders::_2,
                      std::placeholders::_3,
                      std::placeholders::_4),
            ros::DURATION_MAX)
  {
  }

  void populate()
  {
    // Add a standard set of entries into the motion model
    std::set<ros::Time> stamps;
    stamps.insert(ros::Time(10, 0));
    stamps.insert(ros::Time(20, 0));
    stamps.insert(ros::Time(30, 0));
    stamps.insert(ros::Time(40, 0));
    fuse_core::Transaction transaction;
    manager.query(stamps, transaction);
    generated_time_spans.clear();
  }

  void generator(
    const ros::Time& beginning_stamp,
    const ros::Time& ending_stamp,
    std::vector<fuse_core::Constraint::SharedPtr>& constraints,
    std::vector<fuse_core::Variable::SharedPtr>& variables)
  {
    generated_time_spans.emplace_back(beginning_stamp, ending_stamp);
  }

  fuse_core::TimestampManager manager;
  std::vector<std::pair<ros::Time, ros::Time> > generated_time_spans;
};


TEST_F(TimestampManagerTestFixture, Empty)
{
  // Test:
  // Existing: |-------------------------------------> t
  // Adding:   |------********-----------------------> t
  // Expected: |------********-----------------------> t

  // Perform a single query
  std::set<ros::Time> stamps;
  stamps.insert(ros::Time(10, 0));
  stamps.insert(ros::Time(20, 0));
  fuse_core::Transaction transaction;
  manager.query(stamps, transaction);

  // Verify the manager contains the timestamps
  auto stamp_range = manager.stamps();
  ASSERT_EQ(2, std::distance(stamp_range.begin(), stamp_range.end()));
  auto stamp_range_iter = stamp_range.begin();
  EXPECT_EQ(ros::Time(10, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(20, 0), *stamp_range_iter);

  // Verify the expected queries were performed
  ASSERT_EQ(1, generated_time_spans.size());
  EXPECT_EQ(ros::Time(10, 0), generated_time_spans[0].first);
  EXPECT_EQ(ros::Time(20, 0), generated_time_spans[0].second);
}

TEST_F(TimestampManagerTestFixture, Exceptions)
{
  // Set a finite buffer length and populate it with some queries
  manager.bufferLength(ros::Duration(25.0));
  populate();
  // Call the query with a beginning stamp that is too early
  {
    std::set<ros::Time> stamps;
    stamps.insert(ros::Time(1, 0));
    fuse_core::Transaction transaction;
    EXPECT_THROW(manager.query(stamps, transaction), std::invalid_argument);
  }
  // Call the query function with a timestamp within the range. This should not throw.
  {
    std::set<ros::Time> stamps;
    stamps.insert(ros::Time(35, 0));
    fuse_core::Transaction transaction;
    EXPECT_NO_THROW(manager.query(stamps, transaction));
  }
  // Call the query with a timestamp outside of the buffer length, but within the current timespan of the history
  // This should not throw, as it is safe to perform this operation.
  {
    std::set<ros::Time> stamps;
    stamps.insert(ros::Time(11, 0));
    fuse_core::Transaction transaction;
    EXPECT_NO_THROW(manager.query(stamps, transaction));
  }
}

TEST_F(TimestampManagerTestFixture, Purge)
{
  // Set a finite buffer length and populate it with some queries
  manager.bufferLength(ros::Duration(30.0));
  populate();

  // The timespan is within the specified duration. All entries should still be present.
  // Verify the timestamp manager contains the expected data.
  {
    auto stamp_range = manager.stamps();
    ASSERT_EQ(4, std::distance(stamp_range.begin(), stamp_range.end()));
    auto stamp_range_iter = stamp_range.begin();
    EXPECT_EQ(ros::Time(10, 0), *stamp_range_iter);
    ++stamp_range_iter;
    EXPECT_EQ(ros::Time(20, 0), *stamp_range_iter);
    ++stamp_range_iter;
    EXPECT_EQ(ros::Time(30, 0), *stamp_range_iter);
    ++stamp_range_iter;
    EXPECT_EQ(ros::Time(40, 0), *stamp_range_iter);
  }

  // Add an entry right at the edge of the time range (calculations use the ending stamp).
  // This should still keep all entries.
  {
    std::set<ros::Time> stamps;
    stamps.insert(ros::Time(50, 0));
    fuse_core::Transaction transaction;
    manager.query(stamps, transaction);

    auto stamp_range = manager.stamps();
    ASSERT_EQ(5, std::distance(stamp_range.begin(), stamp_range.end()));
    auto stamp_range_iter = stamp_range.begin();
    EXPECT_EQ(ros::Time(10, 0), *stamp_range_iter);
    ++stamp_range_iter;
    EXPECT_EQ(ros::Time(20, 0), *stamp_range_iter);
    ++stamp_range_iter;
    EXPECT_EQ(ros::Time(30, 0), *stamp_range_iter);
    ++stamp_range_iter;
    EXPECT_EQ(ros::Time(40, 0), *stamp_range_iter);
    ++stamp_range_iter;
    EXPECT_EQ(ros::Time(50, 0), *stamp_range_iter);
  }

  // Add another entry. This should cause the oldest entry to be purged.
  {
    std::set<ros::Time> stamps;
    stamps.insert(ros::Time(50, 1));
    fuse_core::Transaction transaction;
    manager.query(stamps, transaction);

    auto stamp_range = manager.stamps();
    ASSERT_EQ(5, std::distance(stamp_range.begin(), stamp_range.end()));
    auto stamp_range_iter = stamp_range.begin();
    EXPECT_EQ(ros::Time(20, 0), *stamp_range_iter);
    ++stamp_range_iter;
    EXPECT_EQ(ros::Time(30, 0), *stamp_range_iter);
    ++stamp_range_iter;
    EXPECT_EQ(ros::Time(40, 0), *stamp_range_iter);
    ++stamp_range_iter;
    EXPECT_EQ(ros::Time(50, 0), *stamp_range_iter);
    ++stamp_range_iter;
    EXPECT_EQ(ros::Time(50, 1), *stamp_range_iter);
  }

  // Add a longer entry. This should cause multiple entries to get purged.
  {
    std::set<ros::Time> stamps;
    stamps.insert(ros::Time(70, 1));
    fuse_core::Transaction transaction;
    manager.query(stamps, transaction);

    auto stamp_range = manager.stamps();
    ASSERT_EQ(4, std::distance(stamp_range.begin(), stamp_range.end()));
    auto stamp_range_iter = stamp_range.begin();
    EXPECT_EQ(ros::Time(40, 0), *stamp_range_iter);
    ++stamp_range_iter;
    EXPECT_EQ(ros::Time(50, 0), *stamp_range_iter);
    ++stamp_range_iter;
    EXPECT_EQ(ros::Time(50, 1), *stamp_range_iter);
    ++stamp_range_iter;
    EXPECT_EQ(ros::Time(70, 1), *stamp_range_iter);
  }

  // Add a very long entry. This should cause all but two entry to get purged.
  {
    std::set<ros::Time> stamps;
    stamps.insert(ros::Time(1000, 0));
    fuse_core::Transaction transaction;
    manager.query(stamps, transaction);

    auto stamp_range = manager.stamps();
    ASSERT_EQ(2, std::distance(stamp_range.begin(), stamp_range.end()));
    auto stamp_range_iter = stamp_range.begin();
    EXPECT_EQ(ros::Time(70, 1), *stamp_range_iter);
    ++stamp_range_iter;
    EXPECT_EQ(ros::Time(1000, 0), *stamp_range_iter);
  }

  // Add an entry far in the future. This should leave only the gap segment and the new segment.
  {
    std::set<ros::Time> stamps;
    stamps.insert(ros::Time(1100, 0));
    stamps.insert(ros::Time(1101, 0));
    fuse_core::Transaction transaction;
    manager.query(stamps, transaction);

    auto stamp_range = manager.stamps();
    ASSERT_EQ(3, std::distance(stamp_range.begin(), stamp_range.end()));
    auto stamp_range_iter = stamp_range.begin();
    EXPECT_EQ(ros::Time(1000, 0), *stamp_range_iter);
    ++stamp_range_iter;
    EXPECT_EQ(ros::Time(1100, 0), *stamp_range_iter);
    ++stamp_range_iter;
    EXPECT_EQ(ros::Time(1101, 0), *stamp_range_iter);
  }
}

TEST_F(TimestampManagerTestFixture, Existing)
{
  // Test:
  // Existing: |------111111112222222233333333-------> t
  // Adding:   |--------------********---------------> t
  // Expected: |------111111112222222233333333-------> t

  populate();
  std::set<ros::Time> stamps;
  stamps.insert(ros::Time(20, 0));
  stamps.insert(ros::Time(30, 0));
  fuse_core::Transaction transaction;
  manager.query(stamps, transaction);

  // Verify the manager contains the expected timestamps
  auto stamp_range = manager.stamps();
  ASSERT_EQ(4, std::distance(stamp_range.begin(), stamp_range.end()));
  auto stamp_range_iter = stamp_range.begin();
  EXPECT_EQ(ros::Time(10, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(20, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(30, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(40, 0), *stamp_range_iter);

  // Verify no new motion model segments were generated
  ASSERT_EQ(0, generated_time_spans.size());
}

TEST_F(TimestampManagerTestFixture, BeforeBeginningAligned)
{
  // Test:
  // Existing: |------111111112222222233333333-------> t
  // Adding:   |--****-------------------------------> t
  // Expected: |--****111111112222222233333333-------> t

  populate();
  std::set<ros::Time> stamps;
  stamps.insert(ros::Time(5, 0));
  stamps.insert(ros::Time(10, 0));
  fuse_core::Transaction transaction;
  manager.query(stamps, transaction);

  // Verify the manager contains the expected timestamps
  auto stamp_range = manager.stamps();
  ASSERT_EQ(5, std::distance(stamp_range.begin(), stamp_range.end()));
  auto stamp_range_iter = stamp_range.begin();
  EXPECT_EQ(ros::Time(5, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(10, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(20, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(30, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(40, 0), *stamp_range_iter);

  // Verify the expected new motion model segments were generated
  ASSERT_EQ(1, generated_time_spans.size());
  EXPECT_EQ(ros::Time(5, 0), generated_time_spans[0].first);
  EXPECT_EQ(ros::Time(10, 0), generated_time_spans[0].second);
}

TEST_F(TimestampManagerTestFixture, BeforeBeginningUnaligned)
{
  // Test:
  // Existing: |------111111112222222233333333-------> t
  // Adding:   |--***--------------------------------> t
  // Expected: |--***A111111112222222233333333-------> t

  populate();
  std::set<ros::Time> stamps;
  stamps.insert(ros::Time(5, 0));
  stamps.insert(ros::Time(8, 0));
  fuse_core::Transaction transaction;
  manager.query(stamps, transaction);

  // Verify the manager contains the expected timestamps
  auto stamp_range = manager.stamps();
  ASSERT_EQ(6, std::distance(stamp_range.begin(), stamp_range.end()));
  auto stamp_range_iter = stamp_range.begin();
  EXPECT_EQ(ros::Time(5, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(8, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(10, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(20, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(30, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(40, 0), *stamp_range_iter);

  // Verify the expected new motion model segments were generated
  ASSERT_EQ(2, generated_time_spans.size());
  EXPECT_EQ(ros::Time(5, 0), generated_time_spans[0].first);
  EXPECT_EQ(ros::Time(8, 0), generated_time_spans[0].second);
  EXPECT_EQ(ros::Time(8, 0), generated_time_spans[1].first);
  EXPECT_EQ(ros::Time(10, 0), generated_time_spans[1].second);
}

TEST_F(TimestampManagerTestFixture, BeforeBeginningOverlap)
{
  // Test:
  // Existing: |------111111112222222233333333-------> t
  // Adding:   |--********---------------------------> t
  // Expected: |--AAAABBBBCCCC2222222233333333-------> t

  populate();
  std::set<ros::Time> stamps;
  stamps.insert(ros::Time(5, 0));
  stamps.insert(ros::Time(15, 0));
  fuse_core::Transaction transaction;
  manager.query(stamps, transaction);

  // Verify the manager contains the expected timestamps
  auto stamp_range = manager.stamps();
  ASSERT_EQ(6, std::distance(stamp_range.begin(), stamp_range.end()));
  auto stamp_range_iter = stamp_range.begin();
  EXPECT_EQ(ros::Time(5, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(10, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(15, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(20, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(30, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(40, 0), *stamp_range_iter);

  // Verify the expected new motion model segments were generated
  ASSERT_EQ(3, generated_time_spans.size());
  EXPECT_EQ(ros::Time(5, 0), generated_time_spans[0].first);
  EXPECT_EQ(ros::Time(10, 0), generated_time_spans[0].second);
  EXPECT_EQ(ros::Time(10, 0), generated_time_spans[1].first);
  EXPECT_EQ(ros::Time(15, 0), generated_time_spans[1].second);
  EXPECT_EQ(ros::Time(15, 0), generated_time_spans[2].first);
  EXPECT_EQ(ros::Time(20, 0), generated_time_spans[2].second);
}

TEST_F(TimestampManagerTestFixture, AfterEndAligned)
{
  // Test:
  // Existing: |------111111112222222233333333-------> t
  // Adding:   |------------------------------****---> t
  // Expected: |------111111112222222233333333****---> t

  populate();
  std::set<ros::Time> stamps;
  stamps.insert(ros::Time(45, 0));
  fuse_core::Transaction transaction;
  manager.query(stamps, transaction);

  // Verify the manager contains the expected timestamps
  auto stamp_range = manager.stamps();
  ASSERT_EQ(5, std::distance(stamp_range.begin(), stamp_range.end()));
  auto stamp_range_iter = stamp_range.begin();
  EXPECT_EQ(ros::Time(10, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(20, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(30, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(40, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(45, 0), *stamp_range_iter);

  // Verify the expected new motion model segments were generated
  ASSERT_EQ(1, generated_time_spans.size());
  EXPECT_EQ(ros::Time(40, 0), generated_time_spans[0].first);
  EXPECT_EQ(ros::Time(45, 0), generated_time_spans[0].second);
}

TEST_F(TimestampManagerTestFixture, AfterEndUnaligned)
{
  // Test:
  // Existing: |------111111112222222233333333-------> t
  // Adding:   |-------------------------------***---> t
  // Expected: |------111111112222222233333333A***---> t

  populate();
  std::set<ros::Time> stamps;
  stamps.insert(ros::Time(42, 0));
  stamps.insert(ros::Time(45, 0));
  fuse_core::Transaction transaction;
  manager.query(stamps, transaction);

  // Verify the manager contains the expected timestamps
  auto stamp_range = manager.stamps();
  ASSERT_EQ(6, std::distance(stamp_range.begin(), stamp_range.end()));
  auto stamp_range_iter = stamp_range.begin();
  EXPECT_EQ(ros::Time(10, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(20, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(30, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(40, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(42, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(45, 0), *stamp_range_iter);

  // Verify the expected new motion model segments were generated
  ASSERT_EQ(2, generated_time_spans.size());
  EXPECT_EQ(ros::Time(40, 0), generated_time_spans[0].first);
  EXPECT_EQ(ros::Time(42, 0), generated_time_spans[0].second);
  EXPECT_EQ(ros::Time(42, 0), generated_time_spans[1].first);
  EXPECT_EQ(ros::Time(45, 0), generated_time_spans[1].second);
}

TEST_F(TimestampManagerTestFixture, AfterEndOverlap)
{
  // Test:
  // Existing: |------111111112222222233333333-------> t
  // Adding:   |--------------------------********---> t
  // Expected: |------1111111122222222AAAABBBBCCCC---> t

  populate();
  std::set<ros::Time> stamps;
  stamps.insert(ros::Time(35, 0));
  stamps.insert(ros::Time(45, 0));
  fuse_core::Transaction transaction;
  manager.query(stamps, transaction);

  // Verify the manager contains the expected timestamps
  auto stamp_range = manager.stamps();
  ASSERT_EQ(6, std::distance(stamp_range.begin(), stamp_range.end()));
  auto stamp_range_iter = stamp_range.begin();
  EXPECT_EQ(ros::Time(10, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(20, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(30, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(35, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(40, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(45, 0), *stamp_range_iter);

  // Verify the expected new motion model segments were generated
  ASSERT_EQ(3, generated_time_spans.size());
  EXPECT_EQ(ros::Time(30, 0), generated_time_spans[0].first);
  EXPECT_EQ(ros::Time(35, 0), generated_time_spans[0].second);
  EXPECT_EQ(ros::Time(35, 0), generated_time_spans[1].first);
  EXPECT_EQ(ros::Time(40, 0), generated_time_spans[1].second);
  EXPECT_EQ(ros::Time(40, 0), generated_time_spans[2].first);
  EXPECT_EQ(ros::Time(45, 0), generated_time_spans[2].second);
}

TEST_F(TimestampManagerTestFixture, MultiSegment)
{
  // Test:
  // Existing: |------111111112222222233333333-------> t
  // Adding:   |------************************-------> t
  // Expected: |------111111112222222233333333-------> t

  populate();
  std::set<ros::Time> stamps;
  stamps.insert(ros::Time(10, 0));
  stamps.insert(ros::Time(40, 0));
  fuse_core::Transaction transaction;
  manager.query(stamps, transaction);

  // Verify the manager contains the expected timestamps
  auto stamp_range = manager.stamps();
  ASSERT_EQ(4, std::distance(stamp_range.begin(), stamp_range.end()));
  auto stamp_range_iter = stamp_range.begin();
  EXPECT_EQ(ros::Time(10, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(20, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(30, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(40, 0), *stamp_range_iter);

  // Verify the expected new motion model segments were generated
  ASSERT_EQ(0, generated_time_spans.size());
}

TEST_F(TimestampManagerTestFixture, MultiSegmentBeforBeginning)
{
  // Test:
  // Existing: |------111111112222222233333333-------> t
  // Adding:   |--****************************-------> t
  // Expected: |--AAAA111111112222222233333333-------> t

  populate();
  std::set<ros::Time> stamps;
  stamps.insert(ros::Time(5, 0));
  stamps.insert(ros::Time(40, 0));
  fuse_core::Transaction transaction;
  manager.query(stamps, transaction);

  // Verify the manager contains the expected timestamps
  auto stamp_range = manager.stamps();
  ASSERT_EQ(5, std::distance(stamp_range.begin(), stamp_range.end()));
  auto stamp_range_iter = stamp_range.begin();
  EXPECT_EQ(ros::Time(5, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(10, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(20, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(30, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(40, 0), *stamp_range_iter);

  // Verify the expected new motion model segments were generated
  ASSERT_EQ(1, generated_time_spans.size());
  EXPECT_EQ(ros::Time(5, 0), generated_time_spans[0].first);
  EXPECT_EQ(ros::Time(10, 0), generated_time_spans[0].second);
}

TEST_F(TimestampManagerTestFixture, MultiSegmentPastEnd)
{
  // Test:
  // Existing: |------111111112222222233333333-------> t
  // Adding:   |------****************************---> t
  // Expected: |------111111112222222233333333AAAA---> t

  populate();
  std::set<ros::Time> stamps;
  stamps.insert(ros::Time(10, 0));
  stamps.insert(ros::Time(45, 0));
  fuse_core::Transaction transaction;
  manager.query(stamps, transaction);

  // Verify the manager contains the expected timestamps
  auto stamp_range = manager.stamps();
  ASSERT_EQ(5, std::distance(stamp_range.begin(), stamp_range.end()));
  auto stamp_range_iter = stamp_range.begin();
  EXPECT_EQ(ros::Time(10, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(20, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(30, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(40, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(45, 0), *stamp_range_iter);

  // Verify the expected new motion model segments were generated
  ASSERT_EQ(1, generated_time_spans.size());
  EXPECT_EQ(ros::Time(40, 0), generated_time_spans[0].first);
  EXPECT_EQ(ros::Time(45, 0), generated_time_spans[0].second);
}

TEST_F(TimestampManagerTestFixture, MultiSegmentPastBothEnds)
{
  // Test:
  // Existing: |------111111112222222233333333-------> t
  // Adding:   |--********************************---> t
  // Expected: |--AAAA111111112222222233333333BBBB---> t

  populate();
  std::set<ros::Time> stamps;
  stamps.insert(ros::Time(5, 0));
  stamps.insert(ros::Time(45, 0));
  fuse_core::Transaction transaction;
  manager.query(stamps, transaction);

  // Verify the manager contains the expected timestamps
  auto stamp_range = manager.stamps();
  ASSERT_EQ(6, std::distance(stamp_range.begin(), stamp_range.end()));
  auto stamp_range_iter = stamp_range.begin();
  EXPECT_EQ(ros::Time(5, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(10, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(20, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(30, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(40, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(45, 0), *stamp_range_iter);

  // Verify the expected new motion model segments were generated
  ASSERT_EQ(2, generated_time_spans.size());
  EXPECT_EQ(ros::Time(5, 0), generated_time_spans[0].first);
  EXPECT_EQ(ros::Time(10, 0), generated_time_spans[0].second);
  EXPECT_EQ(ros::Time(40, 0), generated_time_spans[1].first);
  EXPECT_EQ(ros::Time(45, 0), generated_time_spans[1].second);
}

TEST_F(TimestampManagerTestFixture, SplitBeginning)
{
  // Test:
  // Existing: |------111111112222222233333333-------> t
  // Adding:   |----------************---------------> t
  // Expected: |------AAAABBBB2222222233333333-------> t

  populate();
  std::set<ros::Time> stamps;
  stamps.insert(ros::Time(15, 0));
  stamps.insert(ros::Time(30, 0));
  fuse_core::Transaction transaction;
  manager.query(stamps, transaction);

  // Verify the manager contains the expected timestamps
  auto stamp_range = manager.stamps();
  ASSERT_EQ(5, std::distance(stamp_range.begin(), stamp_range.end()));
  auto stamp_range_iter = stamp_range.begin();
  EXPECT_EQ(ros::Time(10, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(15, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(20, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(30, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(40, 0), *stamp_range_iter);

  // Verify the expected new motion model segments were generated
  ASSERT_EQ(2, generated_time_spans.size());
  EXPECT_EQ(ros::Time(10, 0), generated_time_spans[0].first);
  EXPECT_EQ(ros::Time(15, 0), generated_time_spans[0].second);
  EXPECT_EQ(ros::Time(15, 0), generated_time_spans[1].first);
  EXPECT_EQ(ros::Time(20, 0), generated_time_spans[1].second);
}

TEST_F(TimestampManagerTestFixture, SplitEnd)
{
  // Test:
  // Existing: |------111111112222222233333333-------> t
  // Adding:   |--------------************-----------> t
  // Expected: |------1111111122222222AAAABBBB-------> t

  populate();
  std::set<ros::Time> stamps;
  stamps.insert(ros::Time(20, 0));
  stamps.insert(ros::Time(35, 0));
  fuse_core::Transaction transaction;
  manager.query(stamps, transaction);

  // Verify the manager contains the expected timestamps
  auto stamp_range = manager.stamps();
  ASSERT_EQ(5, std::distance(stamp_range.begin(), stamp_range.end()));
  auto stamp_range_iter = stamp_range.begin();
  EXPECT_EQ(ros::Time(10, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(20, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(30, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(35, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(40, 0), *stamp_range_iter);

  // Verify the expected new motion model segments were generated
  ASSERT_EQ(2, generated_time_spans.size());
  EXPECT_EQ(ros::Time(30, 0), generated_time_spans[0].first);
  EXPECT_EQ(ros::Time(35, 0), generated_time_spans[0].second);
  EXPECT_EQ(ros::Time(35, 0), generated_time_spans[1].first);
  EXPECT_EQ(ros::Time(40, 0), generated_time_spans[1].second);
}

TEST_F(TimestampManagerTestFixture, SplitBoth)
{
  // Test:
  // Existing: |------111111112222222233333333-------> t
  // Adding:   |----------****************-----------> t
  // Expected: |------AAAABBBB22222222CCCCDDDD-------> t

  populate();
  std::set<ros::Time> stamps;
  stamps.insert(ros::Time(15, 0));
  stamps.insert(ros::Time(35, 0));
  fuse_core::Transaction transaction;
  manager.query(stamps, transaction);

  // Verify the manager contains the expected timestamps
  auto stamp_range = manager.stamps();
  ASSERT_EQ(6, std::distance(stamp_range.begin(), stamp_range.end()));
  auto stamp_range_iter = stamp_range.begin();
  EXPECT_EQ(ros::Time(10, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(15, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(20, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(30, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(35, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(40, 0), *stamp_range_iter);

  // Verify the expected new motion model segments were generated
  ASSERT_EQ(4, generated_time_spans.size());
  EXPECT_EQ(ros::Time(10, 0), generated_time_spans[0].first);
  EXPECT_EQ(ros::Time(15, 0), generated_time_spans[0].second);
  EXPECT_EQ(ros::Time(15, 0), generated_time_spans[1].first);
  EXPECT_EQ(ros::Time(20, 0), generated_time_spans[1].second);
  EXPECT_EQ(ros::Time(30, 0), generated_time_spans[2].first);
  EXPECT_EQ(ros::Time(35, 0), generated_time_spans[2].second);
  EXPECT_EQ(ros::Time(35, 0), generated_time_spans[3].first);
  EXPECT_EQ(ros::Time(40, 0), generated_time_spans[3].second);
}

TEST_F(TimestampManagerTestFixture, SplitSame)
{
  // Test:
  // Existing: |------111111112222222233333333-------> t
  // Adding:   |----------------****-----------------> t
  // Expected: |------11111111AABBBBCC33333333-------> t

  populate();
  std::set<ros::Time> stamps;
  stamps.insert(ros::Time(22, 0));
  stamps.insert(ros::Time(27, 0));
  fuse_core::Transaction transaction;
  manager.query(stamps, transaction);

  // Verify the manager contains the expected timestamps
  auto stamp_range = manager.stamps();
  ASSERT_EQ(6, std::distance(stamp_range.begin(), stamp_range.end()));
  auto stamp_range_iter = stamp_range.begin();
  EXPECT_EQ(ros::Time(10, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(20, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(22, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(27, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(30, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(40, 0), *stamp_range_iter);

  // Verify the expected new motion model segments were generated
  ASSERT_EQ(3, generated_time_spans.size());
  EXPECT_EQ(ros::Time(20, 0), generated_time_spans[0].first);
  EXPECT_EQ(ros::Time(22, 0), generated_time_spans[0].second);
  EXPECT_EQ(ros::Time(22, 0), generated_time_spans[1].first);
  EXPECT_EQ(ros::Time(27, 0), generated_time_spans[1].second);
  EXPECT_EQ(ros::Time(27, 0), generated_time_spans[2].first);
  EXPECT_EQ(ros::Time(30, 0), generated_time_spans[2].second);
}

TEST_F(TimestampManagerTestFixture, SplitSameMultiple)
{
  // Test:
  // Existing: |------111111112222222233333333-------> t
  // Adding:   |----------------**%%#----------------> t
  // Expected: |------11111111AABBCCDE33333333-------> t

  populate();
  std::set<ros::Time> stamps;
  stamps.insert(ros::Time(22, 0));
  stamps.insert(ros::Time(25, 0));
  stamps.insert(ros::Time(27, 0));
  stamps.insert(ros::Time(29, 0));
  fuse_core::Transaction transaction;
  manager.query(stamps, transaction);

  // Verify the manager contains the expected timestamps
  auto stamp_range = manager.stamps();
  ASSERT_EQ(8, std::distance(stamp_range.begin(), stamp_range.end()));
  auto stamp_range_iter = stamp_range.begin();
  EXPECT_EQ(ros::Time(10, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(20, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(22, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(25, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(27, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(29, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(30, 0), *stamp_range_iter);
  ++stamp_range_iter;
  EXPECT_EQ(ros::Time(40, 0), *stamp_range_iter);

  // Verify the expected new motion model segments were generated
  ASSERT_EQ(5, generated_time_spans.size());
  EXPECT_EQ(ros::Time(20, 0), generated_time_spans[0].first);
  EXPECT_EQ(ros::Time(22, 0), generated_time_spans[0].second);
  EXPECT_EQ(ros::Time(22, 0), generated_time_spans[1].first);
  EXPECT_EQ(ros::Time(25, 0), generated_time_spans[1].second);
  EXPECT_EQ(ros::Time(25, 0), generated_time_spans[2].first);
  EXPECT_EQ(ros::Time(27, 0), generated_time_spans[2].second);
  EXPECT_EQ(ros::Time(27, 0), generated_time_spans[3].first);
  EXPECT_EQ(ros::Time(29, 0), generated_time_spans[3].second);
  EXPECT_EQ(ros::Time(29, 0), generated_time_spans[4].first);
  EXPECT_EQ(ros::Time(30, 0), generated_time_spans[4].second);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
