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
#include <algorithm>
#include <iterator>
#include <set>
#include <stdexcept>
#include <utility>

#include <boost/iterator/transform_iterator.hpp>
#include <fuse_core/timestamp_manager.hpp>

namespace fuse_core
{

TimestampManager::TimestampManager(
  MotionModelFunction generator,
  const rclcpp::Duration & buffer_length)
: generator_(generator),
  buffer_length_(buffer_length)
{
}

void TimestampManager::query(
  Transaction & transaction,
  bool update_variables)
{
  // Handle the trivial cases first
  const auto & stamps = transaction.involvedStamps();
  if (stamps.empty()) {
    return;
  }
  // Verify the query is within the buffer length
  if ( (!motion_model_history_.empty()) &&
    (buffer_length_ != rclcpp::Duration::max()) &&
    (stamps.front() < motion_model_history_.begin()->first) &&
    (stamps.front() < (motion_model_history_.rbegin()->first - buffer_length_)))
  {
    throw std::invalid_argument(
            "All timestamps must be within the defined buffer length of the motion model");
  }
  // Create a list of all the required timestamps involved in motion model segments that must be
  // created Add all of the existing timestamps between the first and last input stamp
  Transaction motion_model_transaction;
  std::set<rclcpp::Time> augmented_stamps(stamps.begin(), stamps.end());
  auto first_stamp = *augmented_stamps.begin();
  auto last_stamp = *augmented_stamps.rbegin();
  {
    auto begin = motion_model_history_.upper_bound(first_stamp);
    if (begin != motion_model_history_.begin()) {
      --begin;
    }
    auto end = motion_model_history_.upper_bound(last_stamp);
    for (auto iter = begin; iter != end; ++iter) {
      augmented_stamps.insert(iter->first);
    }
    if (end != motion_model_history_.end()) {
      augmented_stamps.insert(end->first);
    }
    // If requested, add the motion model version of the variables involved in the input
    // transaction. This ensures that the variables in the final transaction will be
    // overwritten with the motion model version.
    // Issue #300: Moved the variable update step outside the pair handling loop to
    // ensure that transactions involving only a single stamp will still have their
    // initial conditions updated.
    if (update_variables) {
      // The motion model segments are indexed by the first timestamp of the segment.
      // The very last entry in the motion model history is always an empty record
      // assigned to the final timestamp of the final pair. If the first input timestamp
      // references the very last entry, then \p begin will point at this last, empty
      // entry in the motion model history. Check for that case, and back up one position
      // if needed.
      if (begin != motion_model_history_.begin()) {
        --begin;
      }
      auto transaction_variables = transaction.addedVariables();
      for (auto iter = begin; iter != end; ++iter) {
        for (const auto & variable : iter->second.variables) {
          if (std::any_of(
              transaction_variables.begin(),
              transaction_variables.end(),
              [variable_uuid = variable->uuid()](const auto & input_variable)
              {
                return input_variable.uuid() == variable_uuid;
              }))
          {
            motion_model_transaction.addVariable(variable, update_variables);
          }
        }
      }
    }
  }
  // Convert the sequence of stamps into stamp pairs that must be generated
  std::vector<std::pair<rclcpp::Time, rclcpp::Time>> stamp_pairs;
  {
    for (auto previous_iter = augmented_stamps.begin(),
      current_iter = std::next(augmented_stamps.begin());
      current_iter != augmented_stamps.end();
      ++previous_iter, ++current_iter)
    {
      const rclcpp::Time & previous_stamp = *previous_iter;
      const rclcpp::Time & current_stamp = *current_iter;
      // Check if the timestamp pair is exactly an existing pair. If so, don't add it.
      auto history_iter = motion_model_history_.lower_bound(previous_stamp);
      if ((history_iter != motion_model_history_.end()) &&
        (history_iter->second.beginning_stamp == previous_stamp) &&
        (history_iter->second.ending_stamp == current_stamp))
      {
        continue;
      }
      // Check if this stamp is in the middle of an existing entry. If so, delete it.
      if ((history_iter != motion_model_history_.end()) &&
        (history_iter->second.beginning_stamp < current_stamp) &&
        (history_iter->second.ending_stamp >= current_stamp))
      {
        removeSegment(history_iter, motion_model_transaction);
      }
      // Add this pair
      stamp_pairs.emplace_back(previous_stamp, current_stamp);
    }
  }
  // Create the required segments
  for (const auto & stamp_pair : stamp_pairs) {
    addSegment(stamp_pair.first, stamp_pair.second, motion_model_transaction);
  }
  // Add a dummy entry for the last stamp if one does not already exist
  if (motion_model_history_.empty() || (motion_model_history_.rbegin()->first < last_stamp)) {
    if (motion_model_history_.empty()) {
      // Call the motion model generator so it inserts the last timestamp into its state history.
      std::vector<Constraint::SharedPtr> constraints;
      std::vector<Variable::SharedPtr> variables;
      generator_(last_stamp, last_stamp, constraints, variables);
    }

    // Insert the last timestamp into the motion model history, but with no constraints. The last
    // entry in the motion model history will always contain no constraints.
    motion_model_history_.emplace(last_stamp, MotionModelSegment(last_stamp.get_clock_type()));
  }
  // Purge any old entries from the motion model history
  purgeHistory();
  // Finally, update the input transaction with the created constraints
  transaction.merge(motion_model_transaction, update_variables);
}

TimestampManager::const_stamp_range TimestampManager::stamps() const
{
  std::function<const rclcpp::Time & (const MotionModelHistory::value_type &)> extract_stamp =
    [](const MotionModelHistory::value_type & element) -> const rclcpp::Time &
    {
      return element.first;
    };

  return const_stamp_range(
    boost::make_transform_iterator(motion_model_history_.begin(), extract_stamp),
    boost::make_transform_iterator(motion_model_history_.end(), extract_stamp));
}

void TimestampManager::addSegment(
  const rclcpp::Time & beginning_stamp,
  const rclcpp::Time & ending_stamp,
  Transaction & transaction)
{
  // Generate the set of constraints and variables to add
  std::vector<Constraint::SharedPtr> constraints;
  std::vector<Variable::SharedPtr> variables;
  generator_(beginning_stamp, ending_stamp, constraints, variables);
  // Update the transaction with the generated constraints/variables
  transaction.addInvolvedStamp(beginning_stamp);
  transaction.addInvolvedStamp(ending_stamp);
  for (const auto & constraint : constraints) {
    transaction.addConstraint(constraint);
  }
  for (const auto & variable : variables) {
    transaction.addVariable(variable);
  }
  motion_model_history_.insert_or_assign(
    beginning_stamp, MotionModelSegment(
      beginning_stamp,
      ending_stamp,
      constraints,
      variables));
}

void TimestampManager::removeSegment(
  MotionModelHistory::iterator & iter,
  Transaction & transaction)
{
  // Mark the previously generated constraints for removal
  transaction.addInvolvedStamp(iter->second.beginning_stamp);
  transaction.addInvolvedStamp(iter->second.ending_stamp);
  for (const auto & constraint : iter->second.constraints) {
    transaction.removeConstraint(constraint->uuid());
  }
  // We do not remove variables here. It is assumed the variables are still in use by other
  // constraints.

  // Erase the motion model segment from the history
  motion_model_history_.erase(iter);
}

void TimestampManager::splitSegment(
  MotionModelHistory::iterator & iter,
  const rclcpp::Time & stamp,
  Transaction & transaction)
{
  rclcpp::Time removed_beginning_stamp = iter->second.beginning_stamp;
  rclcpp::Time removed_ending_stamp = iter->second.ending_stamp;
  // We need to remove the existing constraint.
  removeSegment(iter, transaction);
  // And add a new constraint from the beginning of the removed constraint to the provided stamp
  addSegment(removed_beginning_stamp, stamp, transaction);
  // And add a new constraint from the provided stamp to the end of the removed constraint
  addSegment(stamp, removed_ending_stamp, transaction);
}

void TimestampManager::purgeHistory()
{
  // Purge any motion model segments that are more than buffer_length_ seconds older than the most
  // recent entry A setting of rclcpp::Duration::max() means "keep everything" And we want to keep
  // at least one entry in motion model history, regardless of the stamps.
  if ((buffer_length_ == rclcpp::Duration::max()) || (motion_model_history_.size() <= 1)) {
    return;
  }
  // Continue to remove the first entry from the history until we:
  // (a) are left with only one entry, OR
  // (b) the time delta between the beginning and end is within the buffer_length_ We compare with
  //     the ending timestamp of each segment to be conservative
  rclcpp::Time ending_stamp = motion_model_history_.rbegin()->first;
  while ( (motion_model_history_.size() > 1) &&
    ((ending_stamp - motion_model_history_.begin()->second.ending_stamp) > buffer_length_))
  {
    motion_model_history_.erase(motion_model_history_.begin());
  }
}

}  // namespace fuse_core
