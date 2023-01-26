/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, Intrinsic.ai
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

#include <fuse_core/parameter.hpp>

#include <cassert>
#include <string>
#include <unordered_set>


namespace fuse_core
{
std::unordered_set<std::string>
list_parameter_override_prefixes(
  node_interfaces::NodeInterfaces<node_interfaces::Parameters> interfaces,
  std::string prefix)
{
  const std::map<std::string, rclcpp::ParameterValue> & overrides =
    interfaces.get_node_parameters_interface()->get_parameter_overrides();
  return detail::list_parameter_override_prefixes(overrides, prefix);
}

std::unordered_set<std::string>
detail::list_parameter_override_prefixes(
  const std::map<std::string, rclcpp::ParameterValue> & overrides,
  std::string prefix)
{
  // TODO(sloretz) ROS 2 must have this in a header somewhere, right?
  const char kParamSeparator = '.';

  // Find all overrides starting with "prefix.", unless the prefix is empty.
  // If the prefix is empty then look at all parameter overrides.
  if (!prefix.empty() && prefix.back() != kParamSeparator) {
    prefix += kParamSeparator;
  }

  std::unordered_set<std::string> output_names;
  for (const auto & kv : overrides) {
    const std::string & name = kv.first;
    if (name.size() <= prefix.size()) {
      // Too short, no point in checking
      continue;
    }
    assert(prefix.size() < name.size());
    // TODO(sloretz) use string::starts_with in c++20
    if (name.rfind(prefix, 0) == 0) {  // if name starts with prefix
      // Truncate names to the next separator
      size_t separator_pos = name.find(kParamSeparator, prefix.size());
      // Insert truncated name
      output_names.insert(name.substr(0, separator_pos));
    }
  }
  return output_names;
}
}  // namespace fuse_core
