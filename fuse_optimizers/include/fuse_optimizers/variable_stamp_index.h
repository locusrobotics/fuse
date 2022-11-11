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

#include <fuse_core/fuse_macros.h>
#include <fuse_core/transaction.h>
#include <fuse_core/uuid.h>

#include <fuse_core/time.h>

#include <unordered_map>
#include <unordered_set>

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
  FUSE_SMART_PTR_DEFINITIONS(VariableStampIndex)

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
  bool empty() const { return variables_.empty() && constraints_.empty(); }

  /**
   * @brief Returns the number of variables in the index
   */
  size_t size() const { return variables_.size(); }

  /**
   * @brief Clear all tracked state
   */
  void clear()
  {
    stamped_index_.clear();
    variables_.clear();
    constraints_.clear();
  }

  /**
   * @brief Returns the most recent timestamp associated with any variable
   *
   * If there is no timestamp in the index, returns rclcpp::Time(0, 0, RCL_CLOCK_UNINITIALIZED)
   */
  rclcpp::Time currentStamp() const;

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
   * @brief Add all variables that are not directly connected to a stamped variable with a timestamp greater than or
   *        equal to the provided stamp
   *
   * @param[in]  stamp  The reference timestamp. Only variables not associated with timestamps greater than or equal to
   *                    this will be added to the output container
   * @param[out] result An output iterator capable of receiving fuse_core::UUID objects
   */
  template <typename OutputUuidIterator>
  void query(const rclcpp::Time& stamp, OutputUuidIterator result) const
  {
    // First get all of the stamped variables greater than or equal to the input stamp
    std::unordered_set<fuse_core::UUID> recent_variable_uuids;
    for (const auto& variable_stamp_pair : stamped_index_)
    {
      if (variable_stamp_pair.second >= stamp)
      {
        recent_variable_uuids.insert(variable_stamp_pair.first);
      }
    }

    // Now find all of the variables connected to the recent variables
    std::unordered_set<fuse_core::UUID> connected_variable_uuids;
    for (const auto& recent_variable_uuid : recent_variable_uuids)
    {
      // Add the recent variable to ensure connected_variable_uuids is a superset of recent_variable_uuids
      connected_variable_uuids.insert(recent_variable_uuid);

      const auto variables_iter = variables_.find(recent_variable_uuid);
      if (variables_iter != variables_.end())
      {
        for (const auto& connected_constraint_uuid : variables_iter->second)
        {
          const auto constraints_iter = constraints_.find(connected_constraint_uuid);
          if (constraints_iter != constraints_.end())
          {
            for (const auto& connected_variable_uuid : constraints_iter->second)
            {
              connected_variable_uuids.insert(connected_variable_uuid);
            }
          }
        }
      }
    }

    // Return the set of variables that are not connected
    for (const auto& variable : variables_)
    {
      if (connected_variable_uuids.find(variable.first) == connected_variable_uuids.end())
      {
        *result = variable.first;
        ++result;
      }
    }
  }

protected:
  using StampedMap = std::unordered_map<fuse_core::UUID, rclcpp::Time>;
  StampedMap stamped_index_;  //!< Container that holds the UUID->Stamp mapping for fuse_variables::Stamped variables

  using VariableToConstraintsMap = std::unordered_map<fuse_core::UUID, std::unordered_set<fuse_core::UUID>>;
  VariableToConstraintsMap variables_;

  using ConstraintToVariablesMap = std::unordered_map<fuse_core::UUID, std::unordered_set<fuse_core::UUID>>;
  ConstraintToVariablesMap constraints_;

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
};

}  // namespace fuse_optimizers

#endif  // FUSE_OPTIMIZERS_VARIABLE_STAMP_INDEX_H
