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
#ifndef FUSE_CORE_UUID_H
#define FUSE_CORE_UUID_H

#include <ros/time.h>

#include <boost/functional/hash.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <algorithm>
#include <array>
#include <string>


namespace fuse_core
{

using UUID = boost::uuids::uuid;

namespace uuid
{
  using boost::uuids::to_string;
  using hash = boost::hash<UUID>;
  constexpr UUID NIL = {{0}};

  /**
   * @brief Convert a string representation of the UUID into a UUID variable
   *
   * There are a few supported formats:
   * - "01234567-89AB-CDEF-0123-456789ABCDEF"
   * - "01234567-89ab-cdef-0123-456789abcdef"
   * - "0123456789abcdef0123456789abcdef"
   * - "{01234567-89ab-cdef-0123-456789abcdef}"
   */
  inline UUID from_string(const std::string& uuid_string)
  {
    return boost::uuids::string_generator()(uuid_string);
  }

  /**
   * @brief Generate a random UUID
   */
  inline UUID generate()
  {
    return boost::uuids::random_generator()();
  }

  /**
   * @brief Generate a UUID from a raw data buffer
   *
   * @param[in] data       A data buffer containing information that makes this item unique
   * @param[in] byte_count The number of bytes in the data buffer
   * @return               A repeatable UUID specific to the provided data
   */
  inline UUID generate(const void* data, size_t byte_count)
  {
    return boost::uuids::name_generator(NIL)(data, byte_count);
  }

  /**
   * @brief Generate a UUID from a C-style string
   *
   * @param[in] data A data buffer held in a C-style string
   * @return         A repeatable UUID specific to the provided data
   */
  inline UUID generate(const char* data)
  {
    return generate(data, std::strlen(data));
  }

  /**
   * @brief Generate a UUID from a C++ string
   *
   * @param[in] data A data buffer held in a C++-style string
   * @return         A repeatable UUID specific to the provided namespace and data
   */
  inline UUID generate(const std::string& data)
  {
    return generate(data.c_str(), data.length());
  }

  /**
   * @brief Generate a UUID from a namespace string and a raw data buffer
   *
   * @param[in] namespace_string A namespace or parent string used to generate non-overlapping UUIDs
   * @param[in] data             A data buffer containing information that makes this item unique
   * @param[in] byte_count       The number of bytes in the data buffer
   * @return                     A repeatable UUID specific to the provided namespace and data
   */
  inline UUID generate(const std::string& namespace_string, const void* data, size_t byte_count)
  {
    return boost::uuids::name_generator(generate(namespace_string))(data, byte_count);
  }

  /**
   * @brief Generate a UUID from a namespace string and C-style string
   *
   * @param[in] namespace_string A namespace or parent string used to generate non-overlapping UUIDs
   * @param[in] data             A data buffer held in a C-style string
   * @return                     A repeatable UUID specific to the provided namespace and data
   */
  inline UUID generate(const std::string& namespace_string, const char* data)
  {
    return generate(namespace_string, data, std::strlen(data));
  }

  /**
   * @brief Generate a UUID from a namespace string and a C++ string
   *
   * @param[in] namespace_string A namespace or parent string used to generate non-overlapping UUIDs
   * @param[in] data             A data buffer held in a C++-style string
   * @return                     A repeatable UUID specific to the provided namespace and data
   */
  inline UUID generate(const std::string& namespace_string, const std::string& data)
  {
    return generate(namespace_string, data.c_str(), data.length());
  }

  /**
   * @brief Generate a UUID from a namespace string and a ros timestamp
   *
   * Every unique timestamp will generate a unique UUID
   *
   * @param[in] namespace_string A namespace or parent string used to generate non-overlapping UUIDs
   * @param[in] stamp            A ROS::Time timestamp
   * @return                     A repeatable UUID specific to the provided namespace and timestamp
   */
  inline UUID generate(const std::string& namespace_string, const ros::Time& stamp)
  {
    constexpr size_t buffer_size = sizeof(stamp.sec) + sizeof(stamp.nsec);
    std::array<unsigned char, buffer_size> buffer;
    auto iter = buffer.begin();
    iter = std::copy(reinterpret_cast<const unsigned char*>(&stamp.sec),
                     reinterpret_cast<const unsigned char*>(&stamp.sec) + sizeof(stamp.sec),
                     iter);
    iter = std::copy(reinterpret_cast<const unsigned char*>(&stamp.nsec),
                     reinterpret_cast<const unsigned char*>(&stamp.nsec) + sizeof(stamp.nsec),
                     iter);
    return generate(namespace_string, buffer.data(), buffer.size());
  }

  /**
   * @brief Generate a UUID from a namespace string, a ros timestamp, and an additional id
   *
   * Every unique timestamp and id pair will generate a unique UUID
   *
   * @param[in] namespace_string A namespace or parent string used to generate non-overlapping UUIDs
   * @param[in] stamp            A ROS::Time timestamp
   * @param[in] id               A UUID
   * @return                     A repeatable UUID specific to the provided namespace and timestamp
   */
  inline UUID generate(const std::string& namespace_string, const ros::Time& stamp, const UUID& id)
  {
    constexpr size_t buffer_size = sizeof(stamp.sec) + sizeof(stamp.nsec) + UUID::static_size();
    std::array<unsigned char, buffer_size> buffer;
    auto iter = buffer.begin();
    iter = std::copy(reinterpret_cast<const unsigned char*>(&stamp.sec),
                     reinterpret_cast<const unsigned char*>(&stamp.sec) + sizeof(stamp.sec),
                     iter);
    iter = std::copy(reinterpret_cast<const unsigned char*>(&stamp.nsec),
                     reinterpret_cast<const unsigned char*>(&stamp.nsec) + sizeof(stamp.nsec),
                     iter);
    iter = std::copy(id.begin(),
                     id.end(),
                     iter);
    return generate(namespace_string, buffer.data(), buffer.size());
  }
}  // namespace uuid

}  // namespace fuse_core

#endif  // FUSE_CORE_UUID_H
