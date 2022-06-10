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

#include <algorithm>
#include <array>
#include <mutex>
#include <random>


namespace fuse_core
{

namespace uuid
{

UUID generate()
{
  static boost::uuids::random_generator generator;
  static std::mutex generator_mutex;

  UUID uuid;
  {
    std::lock_guard<std::mutex> lock(generator_mutex);
    uuid = generator();
  }

  return uuid;
}

UUID generate(const std::string& namespace_string, const TimeStamp& stamp)
{
  constexpr size_t buffer_size = sizeof(stamp);
  std::array<unsigned char, buffer_size> buffer;
  auto iter = buffer.begin();

  // XXX #warning "unsafe time packing"
  iter = std::copy(reinterpret_cast<const unsigned char*>(&stamp),
                   reinterpret_cast<const unsigned char*>(&stamp) + sizeof(stamp),
                   iter);
  return generate(namespace_string, buffer.data(), buffer.size());
}


UUID generate(const std::string& namespace_string, const rclcpp::Time& stamp)
{
  const auto nanoseconds = stamp.nanoseconds();
  constexpr size_t buffer_size = sizeof(nanoseconds);
  std::array<unsigned char, buffer_size> buffer;
  auto iter = buffer.begin();
  iter = std::copy(reinterpret_cast<const unsigned char*>(&nanoseconds),
                   reinterpret_cast<const unsigned char*>(&nanoseconds) + sizeof(nanoseconds),
                   iter);
  return generate(namespace_string, buffer.data(), buffer.size());
}

UUID generate(const std::string& namespace_string, const TimeStamp& stamp, const UUID& id)
{
  constexpr size_t buffer_size = sizeof(stamp) + UUID::static_size();
  std::array<unsigned char, buffer_size> buffer;
  auto iter = buffer.begin();

  // XXX #warning "unsafe time packing"
  iter = std::copy(reinterpret_cast<const unsigned char*>(&stamp),
                   reinterpret_cast<const unsigned char*>(&stamp) + sizeof(stamp),
                   iter);
  iter = std::copy(id.begin(),
                   id.end(),
                   iter);
  return generate(namespace_string, buffer.data(), buffer.size());
}


UUID generate(const std::string& namespace_string, const rclcpp::Time& stamp, const UUID& id)
{
  const auto nanoseconds = stamp.nanoseconds();
  constexpr size_t buffer_size = sizeof(nanoseconds) + UUID::static_size();
  std::array<unsigned char, buffer_size> buffer;
  auto iter = buffer.begin();
  iter = std::copy(reinterpret_cast<const unsigned char*>(&nanoseconds),
                   reinterpret_cast<const unsigned char*>(&nanoseconds) + sizeof(nanoseconds),
                   iter);
  iter = std::copy(id.begin(),
                   id.end(),
                   iter);
  return generate(namespace_string, buffer.data(), buffer.size());
}


UUID generate(const std::string& namespace_string, const uint64_t& user_id)
{
  return generate(namespace_string, reinterpret_cast<const unsigned char*>(&user_id), sizeof(user_id));
}

}  // namespace uuid

}  // namespace fuse_core
