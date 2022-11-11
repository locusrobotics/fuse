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
#ifndef FUSE_CORE_TRANSACTION_H
#define FUSE_CORE_TRANSACTION_H

#include <fuse_core/constraint.h>
#include <fuse_core/fuse_macros.h>
#include <fuse_core/serialization.h>
#include <fuse_core/uuid.h>
#include <fuse_core/variable.h>
#include <fuse_core/time.h>

#include <boost/range/any_range.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/vector.hpp>

#include <ostream>
#include <iostream>
#include <set>
#include <vector>


namespace fuse_core
{

/**
 * @brief A transaction is a group of variable and constraint additions and subtractions that should all be
 *        processed at the same time.
 *
 * This arises most often with graph edits, when you want to remove an existing constraint and replace it with one
 * or more new constraints. You don't want the removal to happen independently of the additions. All graph operations
 * are contained within a Transaction object so that all operations are treated equally.
 */
class Transaction
{
public:
  FUSE_SMART_PTR_DEFINITIONS(Transaction)

  /**
   * @brief A range of Constraint::SharedPtr objects
   *
   * An object representing a range defined by two iterators. It has begin() and end() methods (which means it can
   * be used in range-based for loops), an empty() method, and a front() method for directly accessing the first
   * member. When dereferenced, an iterator returns a const Constraint&.
   */
  using const_constraint_range = boost::any_range<const Constraint, boost::forward_traversal_tag>;

  /**
   * @brief A range of rclcpp::Time objects
   *
   * An object representing a range defined by two iterators. It has begin() and end() methods (which means it can
   * be used in range-based for loops), an empty() method, and a front() method for directly accessing the first
   * member. When dereferenced, an iterator returns a const rclcpp::Time&.
   */
  using const_stamp_range = boost::any_range<const rclcpp::Time, boost::forward_traversal_tag>;

  /**
   * @brief A range of UUID objects
   *
   * An object representing a range defined by two iterators. It has begin() and end() methods (which means it can
   * be used in range-based for loops), an empty() method, and a front() method for directly accessing the first
   * member. When dereferenced, an iterator returns a const UUID&.
   */
  using const_uuid_range = boost::any_range<const UUID, boost::forward_traversal_tag>;

  /**
   * @brief A range of Variable::SharedPtr objects
   *
   * An object representing a range defined by two iterators. It has begin() and end() methods (which means it can
   * be used in range-based for loops), an empty() method, and a front() method for directly accessing the first
   * member. When dereferenced, an iterator returns a const Variable&.
   */
  using const_variable_range = boost::any_range<const Variable, boost::forward_traversal_tag>;

  /**
   * @brief Read-only access to this transaction's timestamp
   */
  const rclcpp::Time& stamp() const { return stamp_; }

  /**
   * @brief Write access to this transaction's timestamp
   */
  void stamp(const rclcpp::Time& stamp) { stamp_ = stamp; }

  /**
   * @brief Read-only access to the set of timestamps involved in this transaction
   *
   * @return An iterator range containing all involved timestamps, ordered oldest to newest
   */
  const_stamp_range involvedStamps() const { return involved_stamps_; }

  /**
   * @brief Read-only access to the minimum (oldest), timestamp among the transaction's stamp and all involved
   * timestamps, if any
   *
   * @return The minimum (oldest) timestamp.
   */
  const rclcpp::Time& minStamp() const;

  /**
   * @brief Read-only access to the maximum (newest) timestamp among the transaction's stamp and all involved
   * timestamps, if any
   *
   * @return The maximum (newest) timestamp.
   */
  const rclcpp::Time& maxStamp() const;

  /**
   * @brief Read-only access to the added constraints
   *
   * @return  An iterator range containing all added constraints
   */
  const_constraint_range addedConstraints() const;

  /**
   * @brief Read-only access to the removed constraints
   *
   * @return  An iterator range containing all removed constraint UUIDs
   */
  const_uuid_range removedConstraints() const { return removed_constraints_; }

  /**
   * @brief Read-only access to the added variables
   *
   * @return  An iterator range containing all added variables
   */
  const_variable_range addedVariables() const;

  /**
   * @brief Read-only access to the removed variables
   *
   * @return  An iterator range containing all removed variable UUIDs
   */
  const_uuid_range removedVariables() const { return removed_variables_; }

  /**
   * @brief Check if the transaction is empty, i.e. it has no added or removed constraints or variables, and no involved
   * stamps
   *
   * @return  True if the transaction is empty, false otherwise
   */
  bool empty() const;

  /**
   * @brief Add a timestamp to the "involved stamps" collection
   *
   * Duplicate timestamps will be ignored, so adding a stamp multiple times will have no effect.
   *
   * @param[in] stamp The timestamp to be added
   */
  void addInvolvedStamp(const rclcpp::Time& stamp);

  /**
   * @brief Add a constraint to this transaction
   *
   * The transaction will shared ownership of the provided constraint. This function also performs several checks
   * to ensure the same constraint is not added twice, or added and removed.
   *
   * @param[in] constraint The constraint to be added
   * @param[in] overwrite  Flag indicating the provided constraint should overwrite an existing constraint with
   *                       the same UUID
   */
  void addConstraint(Constraint::SharedPtr constraint, bool overwrite = false);

  /**
   * @brief Remove a constraint from this transaction if it was previously added, or mark the constraint for removal
   *        from the graph.
   *
   * The constraint UUID is marked to be removed by the receiver of this Transaction. This function also performs
   * several checks to ensure the same constraint is not removed twice, or added and removed.
   *
   * @param[in] constraint_uuid The UUID of the constraint to remove
   */
  void removeConstraint(const UUID& constraint_uuid);

  /**
   * @brief Add a variable to this transaction
   *
   * The transaction will shared ownership of the provided variable. This function also performs several checks
   * to ensure the same variable is not added twice, or added and removed.
   *
   * @param[in] variable  The variable to be added
   * @param[in] overwrite Flag indicating the provided variable should overwrite an existing variable with the
   *                      same UUID
   */
  void addVariable(Variable::SharedPtr variable, bool overwrite = false);

  /**
   * @brief Remove the variable from this transaction if it was previously added, or mark the variable for removal
   *        from the graph.
   *
   * The variable UUID is marked to be removed by the receiver of this Transaction. This function also performs
   * several checks to ensure the same variable is not removed twice, or added and removed.
   *
   * @param[in] variable_uuid The UUID of the variable to remove
   */
  void removeVariable(const UUID& variable_uuid);

  /**
   * @brief Merge the contents of another transaction into this one.
   *
   * @param[in] other     The transaction to merge in
   * @param[in] overwrite Flag indicating that variables and constraints in \p other should overwrite existing
   *                      variables and constraints with the UUIDs.
   */
  void merge(const Transaction& other, bool overwrite = false);

  /**
   * @brief Print a human-readable description of the transaction to the provided stream.
   *
   * @param[out] stream The stream to write to. Defaults to stdout.
   */
  void print(std::ostream& stream = std::cout) const;

  /**
   * @brief Perform a deep copy of the Transaction and return a unique pointer to the copy
   *
   * Unique pointers can be implicitly upgraded to shared pointers if needed.
   *
   * @return A unique pointer to a new instance of the most-derived Variable
   */
  Transaction::UniquePtr clone() const;

  /**
   * @brief Serialize this Constraint into the provided binary archive
   *
   * @param[out] archive - The archive to serialize this constraint into
   */
  void serialize(fuse_core::BinaryOutputArchive& /* archive */) const;

  /**
   * @brief Serialize this Constraint into the provided text archive
   *
   * @param[out] archive - The archive to serialize this constraint into
   */
  void serialize(fuse_core::TextOutputArchive& /* archive */) const;

  /**
   * @brief Deserialize data from the provided binary archive into this Constraint
   *
   * @param[in] archive - The archive holding serialized Constraint data
   */
  void deserialize(fuse_core::BinaryInputArchive& /* archive */);

  /**
   * @brief Deserialize data from the provided text archive into this Constraint
   *
   * @param[in] archive - The archive holding serialized Constraint data
   */
  void deserialize(fuse_core::TextInputArchive& /* archive */);

private:
  rclcpp::Time stamp_;  //!< The transaction message timestamp
  std::vector<Constraint::SharedPtr> added_constraints_;  //!< The constraints to be added
  std::vector<Variable::SharedPtr> added_variables_;  //!< The variables to be added
  std::set<rclcpp::Time> involved_stamps_;  //!< The set of timestamps involved in this transaction
  std::vector<UUID> removed_constraints_;  //!< The constraint UUIDs to be removed
  std::vector<UUID> removed_variables_;  //!< The variable UUIDs to be removed

  // Allow Boost Serialization access to private methods
  friend class boost::serialization::access;

  /**
   * @brief The Boost Serialize method that serializes all of the data members in to/out of the archive
   *
   * @param[in/out] archive - The archive object that holds the serialized class members
   * @param[in] version - The version of the archive being read/written. Generally unused.
   */
  template<class Archive>
  void serialize(Archive& archive, const unsigned int /* version */)
  {
    archive & stamp_;
    archive & added_constraints_;
    archive & added_variables_;
    archive & involved_stamps_;
    archive & removed_constraints_;
    archive & removed_variables_;
  }
};

/**
 * Stream operator for printing Transaction objects.
 */
std::ostream& operator <<(std::ostream& stream, const Transaction& transaction);

}  // namespace fuse_core

#endif  // FUSE_CORE_TRANSACTION_H
