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
#include <fuse_constraints/variable_order.h>


namespace fuse_constraints
{
VariableOrder::VariableOrder(std::initializer_list<fuse_core::UUID> variable_uuid_list) :
  VariableOrder(variable_uuid_list.begin(), variable_uuid_list.end())
{
}

bool VariableOrder::empty() const
{
  return order_.empty();
}

size_t VariableOrder::size() const
{
  return order_.size();
}

bool VariableOrder::exists(const size_t index) const
{
  return (index < order_.size());
}

bool VariableOrder::exists(const fuse_core::UUID& variable_uuid) const
{
  return (order_.right.find(variable_uuid) != order_.right.end());
}

size_t VariableOrder::add(const fuse_core::UUID& variable_uuid)
{
  auto result = order_.insert(order_.end(), VariableOrderMapping::value_type(order_.size(), variable_uuid));
  return (*result.first).get_left();
}

const fuse_core::UUID& VariableOrder::operator[](const size_t index) const
{
  return order_.left[index].second;
}

const fuse_core::UUID& VariableOrder::at(const size_t index) const
{
  return order_.left.at(index).second;
}

const size_t VariableOrder::at(const fuse_core::UUID& variable_uuid) const
{
  return order_.right.at(variable_uuid);
}

}  // namespace fuse_constraints
