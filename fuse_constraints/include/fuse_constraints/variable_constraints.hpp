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
#ifndef FUSE_CONSTRAINTS__VARIABLE_CONSTRAINTS_HPP_
#define FUSE_CONSTRAINTS__VARIABLE_CONSTRAINTS_HPP_

#include <algorithm>
#include <initializer_list>
#include <iterator>
#include <iostream>
#include <unordered_set>
#include <vector>


namespace fuse_constraints
{

/**
 * @brief Holds the per-variable constraint list
 *
 * Each variable is represented by a unique index. The indices are expected to be "small and
 * compact", i.e. sequentially numbered starting from zero. Failure to meet this expectation will
 * result in excess memory allocation.
 */
class VariableConstraints
{
public:
  /**
   * @brief Pre-allocate enough memory to hold a specified number of variables
   */
  void reserve(const size_t variable_count);

  /**
   * @brief Return true if no variables or constraints have been added
   */
  bool empty() const;

  /**
   * @brief The total number of unique (variable id, constraint id) pairs
   */
  size_t size() const;

  /**
   * @brief The next available variable index
   *
   * This is one larger than the current maximum variable index
   */
  unsigned int nextVariableIndex() const;

  /**
   * @brief Add this constraint to a single variable
   */
  void insert(const unsigned int constraint, const unsigned int variable);

  /**
   * @brief Add this constraint to all variables in the provided list
   */
  void insert(const unsigned int constraint, std::initializer_list<unsigned int> variable_list);

  /**
   * @brief Add this constraint to all variables in the provided range
   */
  template<typename VariableIndexIterator>
  void insert(
    const unsigned int constraint, VariableIndexIterator first,
    VariableIndexIterator last);

  /**
   * @brief Add a single orphan variable, i.e. a variable without constraints
   */
  void insert(const unsigned int variable);

  /**
   * @brief Insert all of the constraints connected to the requested variable into the provided
   *        container
   *
   * Accessing a variable id that is not part of this container results in undefined behavior
   */
  template<typename OutputIterator>
  OutputIterator getConstraints(const unsigned int variable_id, OutputIterator result) const;

  /**
   * @brief Print a human-readable description of the variable constraints to the provided stream.
   *
   * @param[out] stream The stream to write to. Defaults to stdout.
   */
  void print(std::ostream & stream = std::cout) const;

private:
  using ConstraintCollection = std::unordered_set<unsigned int>;
  using ConstraintsByVariable = std::vector<ConstraintCollection>;

  ConstraintsByVariable variable_constraints_;  //!< The collection of constraints for each variable
};

template<typename VariableIndexIterator>
void VariableConstraints::insert(
  const unsigned int constraint, VariableIndexIterator first,
  VariableIndexIterator last)
{
  for (; first != last; ++first) {
    insert(constraint, *first);
  }
}

template<class OutputIterator>
OutputIterator VariableConstraints::getConstraints(
  const unsigned int variable_id,
  OutputIterator result) const
{
  const auto & constraints = variable_constraints_[variable_id];
  return std::copy(std::begin(constraints), std::end(constraints), result);
}

/**
 * Stream operator for printing VariableConstraints objects.
 */
std::ostream & operator<<(std::ostream & stream, const VariableConstraints & variable_constraints);

}  // namespace fuse_constraints

#endif  // FUSE_CONSTRAINTS__VARIABLE_CONSTRAINTS_HPP_
