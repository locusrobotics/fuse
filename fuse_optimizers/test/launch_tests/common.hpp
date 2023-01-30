/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Clearpath Robotics
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

#ifndef FUSE_OPTIMIZERS__TEST_COMMON_HPP_  // NOLINT{build/header_guard}
#define FUSE_OPTIMIZERS__TEST_COMMON_HPP_  // NOLINT{build/header_guard}

#include <algorithm>
#include <iterator>
#include <ostream>
#include <string>
#include <unordered_map>
#include <vector>


/**
 * @brief Helper function to print the elements of std::vector<T> objects
 *
 * @param[in, out] os An output stream
 * @param[in] v A vector to print into the stream
 * @return The output stream with the vector printed into it
 */
template<typename T>
std::ostream & operator<<(std::ostream & os, const std::vector<T> & v)
{
  os << '[';

  if (!v.empty()) {
    std::copy(v.begin(), v.end() - 1, std::ostream_iterator<T>(os, ", "));
    os << v.back();
  }

  os << ']';

  return os;
}

/**
 * @brief Helper function to print the keys of std::unordered_map<K, V> objects
 *
 * @param[in, out] os An output stream
 * @param[in] m An unordered map with the keys to print into the stream
 * @return The output stream with the vector printed into it
 */
template<typename K, typename V>
std::ostream & operator<<(std::ostream & os, const std::unordered_map<K, V> & m)
{
  os << '[';

  for (const auto & entry : m) {
    os << entry.first << ", ";
  }

  os << ']';

  return os;
}

/**
 * @brief Helper function to compute the symmetric difference between a sorted
 *        std::vector<std::string> and the keys of an std::unordered_map<std::string, T>
 *
 * @param[in] lhs A sorted vector of strings
 * @param[in] rhs An unordered map of key strings
 * @return A vector with the symmetric difference strings
 */
template<typename T>
std::vector<std::string> set_symmetric_difference(
  const std::vector<std::string> & lhs,
  const std::unordered_map<std::string, T> & rhs)
{
  // Retrieve the keys:
  std::vector<std::string> rhs_keys;
  std::transform(
    rhs.begin(), rhs.end(), std::back_inserter(rhs_keys),
    [](const auto & pair) {return pair.first;});                // NOLINT(whitespace/braces)

  // Sort the keys so we can use std::set_symmetric_difference:
  std::sort(rhs_keys.begin(), rhs_keys.end());

  // Compute the symmetric difference:
  std::vector<std::string> diff;
  std::set_symmetric_difference(
    lhs.begin(), lhs.end(), rhs_keys.begin(),
    rhs_keys.end(), std::back_inserter(diff));

  return diff;
}


#endif  // FUSE_OPTIMIZERS__TEST_COMMON_HPP_  // NOLINT{build/header_guard}
