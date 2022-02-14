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

#include <ros/time.h>

#include <algorithm>
#include <stdexcept>
#include <vector>

namespace fuse_optimizers
{
ros::Time VariableStampIndex::operator[](int i)
{
  if (unique_stamps_.empty() || i < 0)
  {
    return ros::Time(0, 0);
  }
  else if ((size_t)i < unique_stamps_.size())
  {
    uint64_t stamp_id = (*std::next(unique_stamps_.begin(), i)).first;
    ros::Time stamp;
    stamp.fromNSec(stamp_id);
    return stamp;
  }
  else
  {
    throw std::runtime_error("Index exceeds size of Variable Stamp Index.");
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
      uint64_t stamp = stamped_variable->stamp().toNSec();
      stamped_index_[variable.uuid()] = stamped_variable->stamp();
      if (unique_stamps_.find(stamp) != unique_stamps_.end())
      {
        unique_stamps_[stamp].insert(variable.uuid());
      }
      else
      {
        std::unordered_set<fuse_core::UUID> set = { variable.uuid() };
        unique_stamps_.insert({ stamp, set });
      }
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
    // remove from unique stamps
    auto stamp = stamped_index_[variable_uuid].toNSec();
    if (unique_stamps_.find(stamp) != unique_stamps_.end())
    {
      if (unique_stamps_[stamp].find(variable_uuid) != unique_stamps_[stamp].end())
      {
        unique_stamps_[stamp].erase(variable_uuid);
      }
      if (unique_stamps_[stamp].size() == 0)
      {
        unique_stamps_.erase(stamp);
      }
    }

    stamped_index_.erase(variable_uuid);
    variables_.erase(variable_uuid);
  }
}

}  // namespace fuse_optimizers
