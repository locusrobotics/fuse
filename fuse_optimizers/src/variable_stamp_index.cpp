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
#include <fuse_optimizers/variable_stamp_index.h>

#include <fuse_core/transaction.h>
#include <fuse_core/uuid.h>
#include <fuse_variables/stamped.h>

#include <fuse_core/time.h>

#include <algorithm>
#include <stdexcept>
#include <vector>

namespace fuse_optimizers
{
rclcpp::Time VariableStampIndex::currentStamp() const
{
  auto compare_stamps = [](const StampedMap::value_type& lhs, const StampedMap::value_type& rhs)
  {
    return lhs.second < rhs.second;
  };
  auto iter = std::max_element(stamped_index_.begin(), stamped_index_.end(), compare_stamps);
  if (iter != stamped_index_.end())
  {
    return iter->second;
  }
  else
  {
    return rclcpp::Time(0, 0, RCL_CLOCK_UNINITIALIZED);  // NOTE(CH3): Signals uninitialized
  }
}

void VariableStampIndex::addNewTransaction(const fuse_core::Transaction& transaction)
{
  applyAddedVariables(transaction);
  applyAddedConstraints(transaction);
  applyRemovedConstraints(transaction);
  applyRemovedVariables(transaction);
}

void VariableStampIndex::addMarginalTransaction(const fuse_core::Transaction& transaction)
{
  // Only the removed variables and removed constraints should be applied to the VariableStampIndex
  // No variables will be added by a marginal transaction, and the added constraints add variable links
  // that we *do not* want to track. These links are merely an artifact of the marginalization process.
  applyRemovedConstraints(transaction);
  applyRemovedVariables(transaction);
}

void VariableStampIndex::applyAddedConstraints(const fuse_core::Transaction& transaction)
{
  for (const auto& constraint : transaction.addedConstraints())
  {
    constraints_[constraint.uuid()].insert(constraint.variables().begin(), constraint.variables().end());
    for (const auto& variable_uuid : constraint.variables())
    {
      variables_[variable_uuid].insert(constraint.uuid());
    }
  }
}

void VariableStampIndex::applyAddedVariables(const fuse_core::Transaction& transaction)
{
  for (const auto& variable : transaction.addedVariables())
  {
    auto stamped_variable = dynamic_cast<const fuse_variables::Stamped*>(&variable);
    if (stamped_variable)
    {
      stamped_index_[variable.uuid()] = stamped_variable->stamp();
    }
    variables_[variable.uuid()];  // Add an empty set of constraints
  }
}

void VariableStampIndex::applyRemovedConstraints(const fuse_core::Transaction& transaction)
{
  for (const auto& constraint_uuid : transaction.removedConstraints())
  {
    for (auto& variable_uuid : constraints_[constraint_uuid])
    {
      variables_[variable_uuid].erase(constraint_uuid);
    }
    constraints_.erase(constraint_uuid);
  }
}

void VariableStampIndex::applyRemovedVariables(const fuse_core::Transaction& transaction)
{
  for (const auto& variable_uuid : transaction.removedVariables())
  {
    stamped_index_.erase(variable_uuid);
    variables_.erase(variable_uuid);
  }
}

}  // namespace fuse_optimizers
