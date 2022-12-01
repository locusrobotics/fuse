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
#include <utility>

#include <boost/iterator/transform_iterator.hpp>
#include <boost/range/empty.hpp>
#include <fuse_core/transaction.hpp>

namespace fuse_core
{

const rclcpp::Time & Transaction::minStamp() const
{
  if (involved_stamps_.empty()) {
    return stamp_;
  } else {
    return std::min(*involved_stamps_.begin(), stamp_);
  }
}

const rclcpp::Time & Transaction::maxStamp() const
{
  if (involved_stamps_.empty()) {
    return stamp_;
  } else {
    return std::max(*involved_stamps_.rbegin(), stamp_);
  }
}

void Transaction::addInvolvedStamp(const rclcpp::Time & stamp)
{
  involved_stamps_.insert(stamp);
}

Transaction::const_constraint_range Transaction::addedConstraints() const
{
  std::function<const Constraint & (const Constraint::SharedPtr &)> to_constraint_ref =
    [](const Constraint::SharedPtr & constraint)->const Constraint &
    {
      return *constraint;
    };

  return const_constraint_range(
    boost::make_transform_iterator(added_constraints_.cbegin(), to_constraint_ref),
    boost::make_transform_iterator(added_constraints_.cend(), to_constraint_ref));
}

void Transaction::addConstraint(Constraint::SharedPtr constraint, bool overwrite)
{
  // If the constraint being added is in the 'removed' container, then delete it from the 'removed'
  // container instead of adding it to the 'added' container.
  UUID constraint_uuid = constraint->uuid();
  auto removed_constraints_iter = std::find(
    removed_constraints_.begin(),
    removed_constraints_.end(), constraint_uuid);
  if (removed_constraints_iter != removed_constraints_.end()) {
    removed_constraints_.erase(removed_constraints_iter);
    return;
  }

  // Also don't add the same constraint twice
  auto is_constraint_added = [&constraint_uuid](const Constraint::SharedPtr & added_constraint)
    {
      return constraint_uuid == added_constraint->uuid();
    };
  auto added_constraints_iter = std::find_if(
    added_constraints_.begin(),
    added_constraints_.end(), is_constraint_added);
  if (added_constraints_iter == added_constraints_.end()) {
    added_constraints_.push_back(std::move(constraint));
  } else if (overwrite) {
    *added_constraints_iter = std::move(constraint);
  }
}

void Transaction::removeConstraint(const UUID & constraint_uuid)
{
  // If the constraint being removed is in the 'added' container, then delete it from the 'added'
  // container instead of adding it to the 'removed' container.
  auto is_constraint_added = [&constraint_uuid](const Constraint::SharedPtr & added_constraint)
    {
      return constraint_uuid == added_constraint->uuid();
    };
  auto added_constraints_iter = std::find_if(
    added_constraints_.begin(),
    added_constraints_.end(), is_constraint_added);
  if (added_constraints_iter != added_constraints_.end()) {
    added_constraints_.erase(added_constraints_iter);
    return;
  }
  // Also don't remove the same constraint twice
  auto removed_constraints_iter = std::find(
    removed_constraints_.begin(),
    removed_constraints_.end(), constraint_uuid);
  if (removed_constraints_iter == removed_constraints_.end()) {
    removed_constraints_.push_back(constraint_uuid);
    return;
  }
}

Transaction::const_variable_range Transaction::addedVariables() const
{
  std::function<const Variable & (const Variable::SharedPtr &)> to_variable_ref =
    [](const Variable::SharedPtr & variable)->const Variable &
    {
      return *variable;
    };

  return const_variable_range(
    boost::make_transform_iterator(added_variables_.cbegin(), to_variable_ref),
    boost::make_transform_iterator(added_variables_.cend(), to_variable_ref));
}

bool Transaction::empty() const
{
  return boost::empty(added_variables_) && boost::empty(removed_variables_) &&
         boost::empty(added_constraints_) && boost::empty(removed_constraints_) &&
         involved_stamps_.empty();
}

void Transaction::addVariable(Variable::SharedPtr variable, bool overwrite)
{
  // If the variable being added is in the 'removed' container, then delete it from the 'removed'
  // container instead of adding it to the 'added' container.

  UUID variable_uuid = variable->uuid();
  auto removed_variables_iter = std::find(
    removed_variables_.begin(),
    removed_variables_.end(), variable_uuid);
  if (removed_variables_iter != removed_variables_.end()) {
    removed_variables_.erase(removed_variables_iter);
    return;
  }

  // Also don't add the same variable twice
  auto is_variable_added = [&variable_uuid](const Variable::SharedPtr & added_variable)
    {
      return variable_uuid == added_variable->uuid();
    };
  auto added_variables_iter = std::find_if(
    added_variables_.begin(),
    added_variables_.end(), is_variable_added);
  if (added_variables_iter == added_variables_.end()) {
    added_variables_.push_back(std::move(variable));
  } else if (overwrite) {
    *added_variables_iter = std::move(variable);
  }
}

void Transaction::removeVariable(const UUID & variable_uuid)
{
  // If the variable being removed is in the 'added' container, then delete it from the 'added'
  // container instead of adding it to the 'removed' container.
  auto is_variable_added = [&variable_uuid](const Variable::SharedPtr & added_variable)
    {
      return variable_uuid == added_variable->uuid();
    };
  auto added_variables_iter = std::find_if(
    added_variables_.begin(),
    added_variables_.end(), is_variable_added);
  if (added_variables_iter != added_variables_.end()) {
    added_variables_.erase(added_variables_iter);
    return;
  }

  // Also don't remove the same variable twice
  auto removed_variables_iter = std::find(
    removed_variables_.begin(),
    removed_variables_.end(), variable_uuid);
  if (removed_variables_iter == removed_variables_.end()) {
    removed_variables_.push_back(variable_uuid);
    return;
  }
}

void Transaction::merge(const Transaction & other, bool overwrite)
{
  stamp_ = std::max(stamp_, other.stamp_);
  involved_stamps_.insert(other.involved_stamps_.begin(), other.involved_stamps_.end());
  for (const auto & added_constraint : other.added_constraints_) {
    addConstraint(added_constraint, overwrite);
  }
  for (const auto & removed_constraint : other.removed_constraints_) {
    removeConstraint(removed_constraint);
  }
  for (const auto & added_variable : other.added_variables_) {
    addVariable(added_variable, overwrite);
  }
  for (const auto & removed_variable : other.removed_variables_) {
    removeVariable(removed_variable);
  }
}

void Transaction::print(std::ostream & stream) const
{
  stream << "Stamp: " << stamp_.nanoseconds() << "\n";
  stream << "Involved Timestamps:\n";
  for (const auto & involved_stamp : involved_stamps_) {
    stream << " - " << involved_stamp.nanoseconds() << "\n";
  }
  stream << "Added Variables:\n";
  for (const auto & added_variable : added_variables_) {
    stream << " - " << *added_variable << "\n";
  }
  stream << "Added Constraints:\n";
  for (const auto & added_constraint : added_constraints_) {
    stream << " - " << *added_constraint << "\n";
  }
  stream << "Removed Variables:\n";
  for (const auto & removed_variable : removed_variables_) {
    stream << " - " << removed_variable << "\n";
  }
  stream << "Removed Constraints:\n";
  for (const auto & removed_constraint : removed_constraints_) {
    stream << " - " << removed_constraint << "\n";
  }
}

Transaction::UniquePtr Transaction::clone() const
{
  return Transaction::make_unique(*this);
}

void Transaction::serialize(fuse_core::BinaryOutputArchive & archive) const
{
  archive << *this;
}

void Transaction::serialize(fuse_core::TextOutputArchive & archive) const
{
  archive << *this;
}

void Transaction::deserialize(fuse_core::BinaryInputArchive & archive)
{
  archive >> *this;
}

void Transaction::deserialize(fuse_core::TextInputArchive & archive)
{
  archive >> *this;
}

std::ostream & operator<<(std::ostream & stream, const Transaction & transaction)
{
  transaction.print(stream);
  return stream;
}

}  // namespace fuse_core
