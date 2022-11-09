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
#ifndef FUSE_CORE_MESSAGE_BUFFER_IMPL_H
#define FUSE_CORE_MESSAGE_BUFFER_IMPL_H

#include <rclcpp/duration.hpp>
#include <fuse_core/time.h>

#include <boost/iterator/transform_iterator.hpp>

#include <algorithm>
#include <sstream>
#include <stdexcept>
#include <utility>


namespace fuse_core
{

template<class Message>
MessageBuffer<Message>::MessageBuffer(const rclcpp::Duration& buffer_length) :
  buffer_length_(buffer_length)
{
}

template<class Message>
void MessageBuffer<Message>::insert(const rclcpp::Time& stamp, const Message& msg)
{
  buffer_.emplace_back(stamp, msg);
  purgeHistory();
}

template<class Message>
typename MessageBuffer<Message>::message_range MessageBuffer<Message>::query(
  const rclcpp::Time& beginning_stamp,
  const rclcpp::Time& ending_stamp,
  bool extended_range)
{
  // Verify the query is valid
  if (ending_stamp < beginning_stamp)
  {
    std::stringstream beginning_time_ss;
    beginning_time_ss << beginning_stamp;
    std::stringstream ending_time_ss;
    ending_time_ss << ending_stamp;
    throw std::invalid_argument("The beginning_stamp (" + beginning_time_ss.str() + ") must be less than or equal to "
                                "the ending_stamp (" + ending_time_ss.str() + ").");
  }
  // Verify the query is within the bounds of the buffer
  if (buffer_.empty() || (beginning_stamp < buffer_.front().first) || (ending_stamp > buffer_.back().first))
  {
    std::stringstream requested_time_range_ss;
    requested_time_range_ss << "(" << beginning_stamp << ", " << ending_stamp << ")";
    std::stringstream available_time_range_ss;
    if (buffer_.empty())
    {
      available_time_range_ss << "(EMPTY)";
    }
    else
    {
      available_time_range_ss << "(" << buffer_.front().first << ", " << buffer_.back().first << ")";
    }
    throw std::out_of_range("The requested time range " + requested_time_range_ss.str() + " is outside the available "
                            "time range " + available_time_range_ss.str() + ".");
  }
  // Find the entry that is strictly greater than the requested beginning stamp. If the extended range flag is true,
  // we will then back up one entry.
  auto upper_bound_comparison = [](const auto& stamp, const auto& element) -> bool
  {
    return (element.first > stamp);
  };
  auto beginning_iter = std::upper_bound(buffer_.begin(), buffer_.end(), beginning_stamp, upper_bound_comparison);
  if (extended_range)
  {
    --beginning_iter;
  }
  // Find the entry that is greater than or equal to the ending stamp. If the extended range flag is false, we will
  // back up one entry.
  auto lower_bound_comparison = [](const auto& element, const auto& stamp) -> bool
  {
    return (element.first < stamp);
  };
  auto ending_iter = std::lower_bound(buffer_.begin(), buffer_.end(), ending_stamp, lower_bound_comparison);
  if (extended_range && (ending_iter != buffer_.end()))
  {
    ++ending_iter;
  }
  // Return the beginning and ending iterators as an iterator range with the correct deference type
  return message_range(beginning_iter, ending_iter);
}

template<class Message>
typename MessageBuffer<Message>::stamp_range MessageBuffer<Message>::stamps() const
{
  return stamp_range(boost::make_transform_iterator(buffer_.begin(), extractStamp),
                     boost::make_transform_iterator(buffer_.end(), extractStamp));
}

template<class Message>
void MessageBuffer<Message>::purgeHistory()
{
  // Purge any messages that are more than buffer_length_ seconds older than the most recent entry
  // A setting of rclcpp::Duration::max() means "keep everything"
  // And we want to keep at least two entries in buffer at all times, regardless of the stamps.
  if ((buffer_length_ == rclcpp::Duration::max()) || (buffer_.size() <= 2))
  {
    return;
  }

  // Compute the expiration time carefully, as ROS can't handle negative times
  const auto& ending_stamp = buffer_.back().first;

  if (ending_stamp.seconds() > buffer_length_.seconds()) {
    auto expiration_time = ending_stamp - buffer_length_;
  } else {
    // NOTE(CH3): Uninitialized. But okay because it's just used for comparison.
    auto expiration_time = rclcpp::Time(0, 0, ending_stamp.get_clock_type);
  }

  // Remove buffer elements before the expiration time.
  // Be careful to ensure that:
  //  - at least two entries remains at all times
  //  - the buffer covers *at least* until the expiration time. Longer is acceptable.
  auto is_greater = [](const auto& stamp, const auto& element) -> bool
  {
    return (element.first > stamp);
  };
  auto expiration_iter = std::upper_bound(buffer_.begin(), buffer_.end(), expiration_time, is_greater);
  if (expiration_iter != buffer_.begin())
  {
    // expiration_iter points to the first element > expiration_time.
    // Back up one entry, to a point that is <= expiration_time
    buffer_.erase(buffer_.begin(), std::prev(expiration_iter));
  }
}

}  // namespace fuse_core

#endif  // FUSE_CORE_MESSAGE_BUFFER_IMPL_H
