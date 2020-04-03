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

#include <ros/time.h>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <algorithm>
#include <array>
#include <mutex>
#include <random>
#include <string>


namespace fuse_core
{

namespace uuid
{

// Use the stdlib random number generators. They seem to be significantly faster than the Boost equivalents.
// Implementation mimics original Boost UUID random generator implementation here:
// https://www.boost.org/doc/libs/1_65_0/boost/uuid/random_generator.hpp
//
// Boost random_generator.hpp header file  ----------------------------------------------//
// Copyright 2010 Andy Tompkins.
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//
// Boost Software License - Version 1.0 - August 17th, 2003
//
// Permission is hereby granted, free of charge, to any person or organization
// obtaining a copy of the software and accompanying documentation covered by
// this license (the "Software") to use, reproduce, display, distribute,
// execute, and transmit the Software, and to prepare derivative works of the
// Software, and to permit third-parties to whom the Software is furnished to
// do so, all subject to the following:
//
// The copyright notices in the Software and this entire statement, including
// the above license grant, this restriction and the following disclaimer,
// must be included in all copies of the Software, in whole or in part, and
// all derivative works of the Software, unless such copies or derivative
// works are solely in the form of machine-executable object code generated by
// a source language processor.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT
// SHALL THE COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE
// FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN CONTRACT, TORT OR OTHERWISE,
// ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.
UUID generate()
{
  static std::random_device rd;
  static std::mt19937 generator(rd());
  static std::uniform_int_distribution<uint64_t> distibution;
  static std::mutex distribution_mutex;

  uint64_t random1;
  uint64_t random2;
  {
    std::lock_guard<std::mutex> lock(distribution_mutex);
    random1 = generator();
    random2 = generator();
  }

  UUID u;
  std::memcpy(u.data, &random1, 8);
  std::memcpy(u.data + 8, &random2, 8);

  // set variant
  // must be 0b10xxxxxx
  *(u.begin() + 8) &= 0xBF;
  *(u.begin() + 8) |= 0x80;

  // set version
  // must be 0b0100xxxx
  *(u.begin() + 6) &= 0x4F;  // 0b01001111
  *(u.begin() + 6) |= 0x40;  // 0b01000000

  return u;
}

UUID generate(const std::string& namespace_string, const ros::Time& stamp)
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

UUID generate(const std::string& namespace_string, const ros::Time& stamp, const UUID& id)
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
