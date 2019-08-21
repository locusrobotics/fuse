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
#ifndef FUSE_CORE_SERIALIZATION_H
#define FUSE_CORE_SERIALIZATION_H

#include <fuse_core/uuid.h>

#include <ros/time.h>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/array.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/uuid/uuid_serialize.hpp>
#include <Eigen/Core>

#include <boost/iostreams/categories.hpp>

#include <vector>


namespace fuse_core
{
using BinaryInputArchive = boost::archive::binary_iarchive;
using BinaryOutputArchive = boost::archive::binary_oarchive;
using TextInputArchive = boost::archive::text_iarchive;
using TextOutputArchive = boost::archive::text_oarchive;

/**
 * @brief A Boost IOStreams source device designed to read bytes directly from a ROS message byte array ('uint8[]')
 */
class MessageBufferStreamSource
{
public:
  typedef char char_type;
  typedef boost::iostreams::source_tag category;

  /**
   * @brief Construct a stream source from a previously populated data vector
   *
   * The input vector type is designed to work with ROS message fields of type 'uint8[]'
   *
   * @param[in] data A byte vector from a ROS message
   */
  explicit MessageBufferStreamSource(const std::vector<unsigned char>& data);

  /**
   * @brief The stream source is non-copyable
   */
  MessageBufferStreamSource operator=(const MessageBufferStreamSource&) = delete;

  /**
   * @brief Read up to n characters from the data vector
   *
   * This fulfills the Boost IOStreams source concept.
   *
   * @param[in] s The destination location
   * @param[in] n The number of bytes to read from the stream
   * @return The number of bytes read, or -1 to indicate EOF
   */
  std::streamsize read(char_type* s, std::streamsize n);

private:
  const std::vector<unsigned char>& data_;  //!< Reference to the source container
  size_t index_;  //!< The next vector index to read
};

/**
 * @brief A Boost IOStreams sink device designed to write bytes directly from a ROS message byte array ('uint8[]')
 */
class MessageBufferStreamSink
{
public:
  typedef char char_type;
  typedef boost::iostreams::sink_tag category;

  /**
   * @brief Construct a stream sink from a data vector
   *
   * The input vector type is designed to work with ROS message fields of type 'uint8[]'
   *
   * @param[in] data A byte vector from a ROS message
   */
  explicit MessageBufferStreamSink(std::vector<unsigned char>& data);

  /**
   * @brief The stream sink is non-copyable
   */
  MessageBufferStreamSink operator=(const MessageBufferStreamSink&) = delete;

  /**
   * @brief Write n characters to the data vector
   *
   * This fulfills the Boost IOStreams sink concept.
   *
   * @param[in] s The source location
   * @param[in] n The number of bytes to write to the stream
   * @return The number of bytes written
   */
  std::streamsize write(const char_type* s, std::streamsize n);

private:
  std::vector<unsigned char>& data_;  //!< Reference to the destination container
};

}  // namespace fuse_core

namespace boost
{
namespace serialization
{

/**
 * @brief Serialize a ros::Time variable using Boost Serialization
 */
template<class Archive>
void serialize(Archive& archive, ros::Time& stamp, const unsigned int /* version */)
{
  archive & stamp.sec;
  archive & stamp.nsec;
}

/**
 * @brief Serialize an Eigen Matrix using Boost Serialization
 */
template<class Archive,
         class S,
         int Rows_,
         int Cols_,
         int Ops_,
         int MaxRows_,
         int MaxCols_>
inline void save(
  Archive& archive,
  const Eigen::Matrix<S, Rows_, Cols_, Ops_, MaxRows_, MaxCols_>& matrix,
  const unsigned int /* version */)
{
  int rows = matrix.rows();
  int cols = matrix.cols();

  archive & rows;
  archive & cols;
  archive & boost::serialization::make_array(matrix.data(), rows * cols);
}

/**
 * @brief Deserialize an Eigen Matrix using Boost Serialization
 */
template<class Archive,
         class S,
         int Rows_,
         int Cols_,
         int Ops_,
         int MaxRows_,
         int MaxCols_>
inline void load(
  Archive& archive,
  Eigen::Matrix<S, Rows_, Cols_, Ops_, MaxRows_, MaxCols_>& matrix,
  const unsigned int /* version */)
{
  int rows, cols;
  archive & rows;
  archive & cols;
  matrix.resize(rows, cols);
  archive & boost::serialization::make_array(matrix.data(), rows * cols);
}

/**
 * @brief Indicate the Eigen Matrix serialization uses separate save() and load() methods
 */
template<class Archive,
         class S,
         int Rows_,
         int Cols_,
         int Ops_,
         int MaxRows_,
         int MaxCols_>
inline void serialize(
  Archive& archive,
  Eigen::Matrix<S, Rows_, Cols_, Ops_, MaxRows_, MaxCols_>& matrix,
  const unsigned int version)
{
  split_free(archive, matrix, version);
}

}  // namespace serialization
}  // namespace boost

#endif  // FUSE_CORE_SERIALIZATION_H
