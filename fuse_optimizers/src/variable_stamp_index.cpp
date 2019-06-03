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

ros::Time VariableStampIndex::currentStamp() const
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
    return ros::Time(0, 0);
  }
}

ros::Time VariableStampIndex::at(const fuse_core::UUID& variable) const
{
  auto stamped_iter = stamped_index_.find(variable);
  if (stamped_iter != stamped_index_.end())
  {
    return stamped_iter->second;
  }
  auto unstamped_iter = unstamped_index_.find(variable);
  if (unstamped_iter != unstamped_index_.end())
  {
    return getMaxConstraintStamp(unstamped_iter->second);
  }
  throw std::out_of_range("The requested variable UUID '" + fuse_core::uuid::to_string(variable) + "' does not "
                          "exist in this VariableStampIndex object.");
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
    auto stamp = ros::Time(0, 0);
    auto unstamped_uuids = std::vector<fuse_core::UUID>();
    for (const auto& variable_uuid : constraint.variables())
    {
      auto stamped_iter = stamped_index_.find(variable_uuid);
      if (stamped_iter != stamped_index_.end())
      {
        if (stamped_iter->second > stamp)
        {
          stamp = stamped_iter->second;
        }
      }
      else
      {
        unstamped_uuids.push_back(variable_uuid);
      }
    }
    auto contraint_info = ConstraintInfo::value_type(constraint.uuid(), stamp);
    for (const auto& unstamped_uuid : unstamped_uuids)
    {
      unstamped_index_[unstamped_uuid].insert(contraint_info);
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
    else
    {
      unstamped_index_[variable.uuid()];  // Add an empty set
    }
  }
}

void VariableStampIndex::applyRemovedConstraints(const fuse_core::Transaction& transaction)
{
  for (const auto& constraint_uuid : transaction.removedConstraints())
  {
    for (auto& unstamped_variable : unstamped_index_)
    {
      unstamped_variable.second.erase(constraint_uuid);
    }
  }
}

void VariableStampIndex::applyRemovedVariables(const fuse_core::Transaction& transaction)
{
  for (const auto& variable : transaction.removedVariables())
  {
    auto stamped_iter = stamped_index_.find(variable);
    if (stamped_iter != stamped_index_.end())
    {
      stamped_index_.erase(stamped_iter);
      continue;
    }
    auto unstamped_iter = unstamped_index_.find(variable);
    if (unstamped_iter != unstamped_index_.end())
    {
      unstamped_index_.erase(unstamped_iter);
    }
  }
}

ros::Time VariableStampIndex::getMaxConstraintStamp(const ConstraintInfo& constraints) const
{
  auto compare_stamps = [](const ConstraintInfo::value_type& lhs, const ConstraintInfo::value_type& rhs)
  {
    return lhs.second < rhs.second;
  };
  auto iter = std::max_element(constraints.begin(), constraints.end(), compare_stamps);
  if (iter != constraints.end())
  {
    return iter->second;
  }
  else
  {
    return ros::Time(0, 0);
  }
}

}  // namespace fuse_optimizers
