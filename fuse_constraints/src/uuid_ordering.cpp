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
#include <fuse_constraints/uuid_ordering.hpp>
#include <fuse_core/uuid.hpp>

namespace fuse_constraints
{
UuidOrdering::UuidOrdering(std::initializer_list<fuse_core::UUID> uuid_list)
: UuidOrdering(uuid_list.begin(), uuid_list.end())
{
}

bool UuidOrdering::empty() const
{
  return order_.empty();
}

size_t UuidOrdering::size() const
{
  return order_.size();
}

bool UuidOrdering::exists(const unsigned int index) const
{
  return index < order_.size();
}

bool UuidOrdering::exists(const fuse_core::UUID & uuid) const
{
  return order_.right.find(uuid) != order_.right.end();
}

bool UuidOrdering::push_back(const fuse_core::UUID & uuid)
{
  auto result = order_.insert(order_.end(), UuidOrderMapping::value_type(order_.size(), uuid));
  return result.second;
}

const fuse_core::UUID & UuidOrdering::operator[](const unsigned int index) const
{
  return order_.left[index].second;
}

unsigned int UuidOrdering::operator[](const fuse_core::UUID & uuid)
{
  auto result = order_.insert(order_.end(), UuidOrderMapping::value_type(order_.size(), uuid));
  return (*result.first).get_left();
}

const fuse_core::UUID & UuidOrdering::at(const unsigned int index) const
{
  return order_.left.at(index).second;
}

unsigned int UuidOrdering::at(const fuse_core::UUID & uuid) const
{
  return order_.right.at(uuid);
}

}  // namespace fuse_constraints
