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
#ifndef FUSE_OPTIMIZERS_VARIABLE_STAMP_INDEX_H
#define FUSE_OPTIMIZERS_VARIABLE_STAMP_INDEX_H

#include <fuse_core/macros.h>
#include <fuse_core/transaction.h>
#include <fuse_core/uuid.h>

#include <ros/time.h>

#include <unordered_map>
#include <unordered_set>
#include <vector>


namespace fuse_optimizers
{

/**
 * @brief Object designed to track the timestamps associated with each variable
 *
 * If the variable is derived from a fuse_variables::Stamped type, then the variable's stamp is used. If a variable
 * is not derived from a fuse_variables::Stamped type, then the timestamp of the newest fuse_variables::Stamped
 * variable *directly connected* to the unstamped variable is used. If an unstamped variable is not directly connected
 * to any variable, then it is assigned a zero timestamp.
 */
class VariableStampIndex
{
public:
  SMART_PTR_DEFINITIONS(VariableStampIndex);

  /**
   * @brief Constructor
   */
  VariableStampIndex() = default;

  /**
   * @brief Destructor
   */
  virtual ~VariableStampIndex() = default;

  /**
   * @brief Return true if no variables exist in the index
   */
  bool empty() const
  {
    return stamped_index_.empty() && unstamped_index_.empty();
  }

  /**
   * @brief Returns the number of variables in the index
   */
  size_t size() const
  {
    return stamped_index_.size() + unstamped_index_.size();
  }

  /**
   * @brief Clear all tracked state
   */
  void clear()
  {
    stamped_index_.clear();
    unstamped_index_.clear();
  }

  /**
   * @brief Returns the most recent timestamp associated with any variable
   */
  ros::Time currentStamp() const;

  /**
   * @brief Access the timestamp associated with a specific variable
   *
   * If the requested variable does not exist, an out_of_range exception will be thrown
   */
  ros::Time at(const fuse_core::UUID& variable) const;

  /**
   * @brief Update the index with the information from the added transactions
   *
   * @param[in] transaction The set of variables and constraints to add and remove
   */
  void addNewTransaction(const fuse_core::Transaction& transaction);

  /**
   * @brief Update the index with the information from a marginal transaction
   *
   * Only a subset of information is used from the marginal transaction. We do not want to track the variable
   * connections induced by marginal factors.
   *
   * @param[in] transaction The set of variables and constraints to remove
   */
  void addMarginalTransaction(const fuse_core::Transaction& transaction);

  /**
   * @brief Add all variables with a timestamp less than the provided stamp to the output container
   *
   * @param[in]  stamp  The reference timestamp. Only variables associated with timestamps less than this will be
   *                    added to the output container
   * @param[out] result An output iterator capable of receiving fuse_core::UUID objects
   */
  template <typename OutputUuidIterator>
  void query(const ros::Time& stamp, OutputUuidIterator result) const
  {
    for (const auto& variable_stamp_pair : stamped_index_)
    {
      if (variable_stamp_pair.second < stamp)
      {
        *result = variable_stamp_pair.first;
        ++result;
      }
    }
    for (const auto& variable_constraints_pair : unstamped_index_)
    {
      auto max_stamp = getMaxConstraintStamp(variable_constraints_pair.second);
      if (max_stamp < stamp)
      {
        *result = variable_constraints_pair.first;
        ++result;
      }
    }
  }

protected:
  using StampedMap = std::unordered_map<fuse_core::UUID, ros::Time, fuse_core::uuid::hash>;
  StampedMap stamped_index_;  //!< Container that holds the UUID->Stamp mapping for fuse_variables::Stamped variables

  using ConstraintInfo = std::unordered_map<fuse_core::UUID, ros::Time, fuse_core::uuid::hash>;
  using UnstampedMap = std::unordered_map<fuse_core::UUID, ConstraintInfo, fuse_core::uuid::hash>;
  UnstampedMap unstamped_index_;  //!< Container holding the UnstampedUUID->{Constraints} mapping

  /**
   * @brief Update this VariableStampIndex with the added constraints from the provided transaction
   */
  void applyAddedConstraints(const fuse_core::Transaction& transaction);

  /**
   * @brief Update this VariableStampIndex with the added variables from the provided transaction
   */
  void applyAddedVariables(const fuse_core::Transaction& transaction);

  /**
   * @brief Update this VariableStampIndex with the removed constraints from the provided transaction
   */
  void applyRemovedConstraints(const fuse_core::Transaction& transaction);

  /**
   * @brief Update this VariableStampIndex with the removed variables from the provided transaction
   */
  void applyRemovedVariables(const fuse_core::Transaction& transaction);

  /**
   * @brief Find the max stamp inside a ConstraintInfo object
   */
  ros::Time getMaxConstraintStamp(const ConstraintInfo& constraints) const;
};

}  // namespace fuse_optimizers

#endif  // FUSE_OPTIMIZERS_VARIABLE_STAMP_INDEX_H
