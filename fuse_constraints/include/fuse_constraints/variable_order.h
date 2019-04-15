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
#ifndef FUSE_CONSTRAINTS_VARIABLE_ORDER_H
#define FUSE_CONSTRAINTS_VARIABLE_ORDER_H

#include <fuse_core/uuid.h>

#include <boost/bimap/bimap.hpp>
#include <boost/bimap/unordered_set_of.hpp>
#include <boost/bimap/vector_of.hpp>


// #include <algorithm>
// #include <deque>
// #include <set>
// #include <vector>


namespace fuse_constraints
{

/**
 * @brief A class that represents a sequential ordering of variables
 *
 * This is designed for use when marginalizing out variables, but it may have order uses.
 *
 * Specifically, this class maps a \p Variable UUID to a sequential index. Bidirectional access is possible.
 * If you a Variable UUID, the variable index can be retrieved in constant-time. And if you know the index, the
 * Variable UUID may be retrieved in constant (and fast) time. Also, iterating through the Variable UUIDs is an
 * efficient operation.
 * 
 * The VariableIndex is not designed to be highly dynamic. Variables can be added, but not removed. Variables are
 * assigned an index based on the order of insertion and cannot be modified.
 */
class VariableOrder
{
public:
  // TODO(swilliams) Add iterator access

  /**
   * @brief Default constructor
   */
  VariableOrder() = default;

  /**
   * @brief Construct a VariableOrder with the provided UUIDs
   *
   * Accepts an arbitrary number of variable UUIDs directly. It can be called like:
   * @code{.cpp}
   * VariableOrder{uuid1, uuid2, uuid3};
   * @endcode
   *
   * @param[in] variable_uuid_list The list of involved variable UUIDs
   */
  VariableOrder(std::initializer_list<fuse_core::UUID> variable_uuid_list);

  /**
   * @brief Construct a VariableOrder with the UUIDs from the provided collection
   * 
   * The \p VariableUuidConstIterator class must meet the ForwardIterator requirements, and when dereferenced must
   * be compatible with a \p const fuse_core::UUID&.
   *
   * @param[in] first Iterator pointing to the first Variable UUID to add to the ordering
   * @param[in] last  Iterator pointing to one passe the last Variable UUID to add to the ordering
   */
  template <typename VariableUuidConstIterator>
  VariableOrder(VariableUuidConstIterator first, VariableUuidConstIterator last);

  /**
   * @brief Returns true if there are no Variable UUIDs in this ordering
   */
  bool empty() const;

  /**
   * @brief Returns the number of Variable UUIDs in this ordering
   * 
   * This is always equal to "last index + 1"
   */
  size_t size() const;

  /**
   * @brief Return true if the index exists in the ordering
   */
  bool exists(const size_t index) const;

  /**
   * @brief Return true if the index exists in the ordering
   */
  bool exists(const fuse_core::UUID& variable_uuid) const;

  /**
   * @brief Add a new Variable UUID to the ordering
   *
   * If the variable already exists, no change to the ordering will occur.
   *
   * @param[in] variable_uuid The variable UUID to add
   * @return The Variable index associated with the input UUID
   */
  size_t add(const fuse_core::UUID& variable_uuid);

  /**
   * @brief Access the Variable UUID stored at the provided index
   *
   * Accessing an index that does not exist results in undefined behavior
   */
  const fuse_core::UUID& operator[](const size_t index) const;

  /**
   * @brief Access the Variable UUID stored at the provided index
   * 
   * If the requested index does not exist, an out_of_range exception will be thrown.
   */
  const fuse_core::UUID& at(const size_t index) const;

  /**
   * @brief Access the index associated with the provided Variable UUID
   * 
   * If the requested Variable UUID does not exist, an out_of_range exception will be thrown.
   */
  const size_t at(const fuse_core::UUID& variable_uuid) const;

private:
  using VariableOrderMapping = boost::bimaps::bimap<boost::bimaps::vector_of<size_t>,
                                                    boost::bimaps::unordered_set_of<fuse_core::UUID>>;
  VariableOrderMapping order_;  //!< Collection that contains the Index<-->UUID mapping
};

template <typename VariableUuidConstIterator>
VariableOrder::VariableOrder(VariableUuidConstIterator first, VariableUuidConstIterator last)
{
  for (; first != last; ++first)
  {
    order_.left.push_back(VariableOrderMapping::left_value_type(order_.size(), *first));
  }
}

}  // namespace fuse_constraints

#endif  // FUSE_CONSTRAINTS_VARIABLE_ORDER_H
