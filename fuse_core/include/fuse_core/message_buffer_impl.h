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

#include <ros/duration.h>
#include <ros/time.h>

#include <boost/iterator/transform_iterator.hpp>

#include <algorithm>
#include <sstream>
#include <stdexcept>
#include <utility>


namespace fuse_core
{

template<class Message>
MessageBuffer<Message>::MessageBuffer(const ros::Duration& buffer_length) :
  buffer_length_(buffer_length)
{
}

template<class Message>
void MessageBuffer<Message>::insert(const ros::Time& stamp, const Message& msg)
{
  buffer_.emplace_back(stamp, msg);
  purgeHistory();
}

template<class Message>
typename MessageBuffer<Message>::message_range MessageBuffer<Message>::query(
  const ros::Time& beginning_stamp,
  const ros::Time& ending_stamp,
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
  auto upper_bound_comparison = [](const ros::Time& stamp, const typename Buffer::value_type& element) -> bool
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
  auto lower_bound_comparison = [](const typename Buffer::value_type& element, const ros::Time& stamp) -> bool
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
  // A setting of ros::DURATION_MAX means "keep everything"
  // And we want to keep at least two entries in buffer at all times, regardless of the stamps.
  if ((buffer_length_ == ros::DURATION_MAX) || (buffer_.size() <= 2))
  {
    return;
  }
  // Continue to remove the first entry from the buffer until we:
  // (a) are left with only two entries, OR
  // (b) the time delta between the beginning and end is within the buffer_length_
  const ros::Time& ending_stamp = buffer_.back().first;
  while ((buffer_.size() > 2)
      && ((ending_stamp - buffer_.front().first) > buffer_length_))
  {
    buffer_.pop_front();
  }
}

}  // namespace fuse_core

#endif  // FUSE_CORE_MESSAGE_BUFFER_IMPL_H
