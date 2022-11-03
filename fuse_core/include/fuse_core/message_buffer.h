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
#ifndef FUSE_CORE_MESSAGE_BUFFER_H
#define FUSE_CORE_MESSAGE_BUFFER_H

#include <fuse_core/fuse_macros.h>
#include <rclcpp/duration.hpp>
#include <fuse_core/time.h>

#include <boost/range/any_range.hpp>

#include <deque>
#include <utility>


namespace fuse_core
{

/**
 * @brief A utility class that maintains a history of received messages, and allows a range of messages to be easily
 * queried by timestamp.
 *
 * For motion models that rely on integrating multiple measurements together to form the requested constraint, this
 * message buffer allows the received messages within a given time range to be easily extracted. It is then a matter
 * of processing the messages to form the constraint.
 *
 * It is assumed that all messages are received sequentially.
 */
template <typename Message>
class MessageBuffer
{
public:
  FUSE_SMART_PTR_DEFINITIONS(MessageBuffer<Message>)

  /**
   * @brief A range of messages
   *
   * An object representing a range defined by two iterators. It has begin() and end() methods (which means it can
   * be used in range-based for loops), an empty() method, and a front() method for directly accessing the first
   * member. When dereferenced, an iterator returns a std::pair<rclcpp::Time, MESSAGE>&.
   */
  using message_range = boost::any_range<const std::pair<rclcpp::Time, Message>, boost::forward_traversal_tag>;

  /**
   * @brief A range of timestamps
   *
   * An object representing a range defined by two iterators. It has begin() and end() methods (which means it can
   * be used in range-based for loops), an empty() method, and a front() method for directly accessing the first
   * member. When dereferenced, an iterator returns a const rclcpp::Time&.
   */
  using stamp_range = boost::any_range<const rclcpp::Time, boost::forward_traversal_tag>;

  /**
   * Constructor
   *
   * @param[in] buffer_length The length of the message buffer history. If queries arrive involving timestamps
   *                          that are older than the buffer length, an exception will be thrown.
   */
  explicit MessageBuffer(const rclcpp::Duration& buffer_length = rclcpp::Duration::max());

  /**
   * @brief Destructor
   */
  virtual ~MessageBuffer() = default;

  /**
   * @brief Read-only access to the buffer length
   */
  const rclcpp::Duration& bufferLength() const
  {
    return buffer_length_;
  }

  /**
   * @brief Write access to the buffer length
   */
  void bufferLength(const rclcpp::Duration& buffer_length)
  {
    buffer_length_ = buffer_length;
  }

  /**
   * @brief Insert a message to the buffer, using the provided timestamp
   *
   * The provided timestamp is assigned to the message and used to sort the messages in the buffer.
   *
   * @param[in] stamp The stamp to assign to the message
   * @param[in] msg   A message
   */
  void insert(const rclcpp::Time& stamp, const Message& msg);

  /**
   * @brief Query the buffer for the set of messages between two timestamps
   *
   * The "edge behavior" is controlled by the \p extended_range flag.
   *
   * @param[in] beginning_stamp The beginning timestamp of the constraint. \p beginning_stamp must be less than or
   *                            equal to \p ending_stamp.
   * @param[in] ending_stamp    The ending timestamp of the constraint. \p ending_stamp must be greater than or
   *                            or equal to \p beginning_stamp.
   * @param[in] extended_range  A flag indicating if the message range should be extended to include one message
   *                            with a stamp less than or equal to the \p beginning_stamp and one message with a
   *                            stamp greater than or equal to the \p ending_stamp. If false, the returned range
   *                            only includes messages with stamps greater than \p beginning_stamp and less than
   *                            \p ending_stamp.
   * @return                    An iterator range containing all of the messages between the specified stamps.
   */
  message_range query(const rclcpp::Time& beginning_stamp, const rclcpp::Time& ending_stamp, bool extended_range = true);

  /**
   * @brief Read-only access to the current set of timestamps
   *
   * @return An iterator range containing all known timestamps in ascending order
   */
  stamp_range stamps() const;

protected:
  using Buffer = std::deque<std::pair<rclcpp::Time, Message>>;
  Buffer buffer_;  //!< The container of received messages, sorted by timestamp
  rclcpp::Duration buffer_length_;  //!< The length of the motion model history. Segments older than \p buffer_length_
                                 //!< will be removed from the motion model history

  /**
   * @brief Helper function used with boost::transform_iterators to convert the internal Buffer value type
   * into a const rclcpp::Time& iterator compatible with stamp_range
   */
  static const rclcpp::Time& extractStamp(const typename Buffer::value_type& element)
  {
    return element.first;
  }

  /**
   * @brief Remove any motion model segments that are older than \p buffer_length_
   *
   * The span of the buffer will be *at least* the requested buffer length, but it may be longer depending on the
   * specific stamps of received messages.
   */
  void purgeHistory();
};

}  // namespace fuse_core

#include <fuse_core/message_buffer_impl.h>

#endif  // FUSE_CORE_MESSAGE_BUFFER_H
