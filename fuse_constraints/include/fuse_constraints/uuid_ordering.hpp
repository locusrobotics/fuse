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
#ifndef FUSE_CONSTRAINTS__UUID_ORDERING_HPP_
#define FUSE_CONSTRAINTS__UUID_ORDERING_HPP_

#include <fuse_core/uuid.hpp>

#include <boost/bimap/bimap.hpp>
#include <boost/bimap/unordered_set_of.hpp>
#include <boost/bimap/vector_of.hpp>


namespace fuse_constraints
{

/**
 * @brief A class that represents a sequential ordering of UUIDs
 *
 * This is designed for use when marginalizing out variables, but it may have other uses.
 *
 * Specifically, this class maps a UUID to a sequential index. Bidirectional access is possible.
 * If you have a UUID, the index can be retrieved in constant time. And if you have the index, the
 * UUID may be retrieved in constant (and fast) time. Also, iterating through the UUIDs in sequence
 * is an efficient operation.
 *
 * The UuidOrdering is not designed to be highly dynamic. UUIDs can be added, but not removed. UUIDs
 * are assigned an index based on the order of insertion and cannot be modified.
 */
class UuidOrdering
{
public:
  /**
   * @brief Default constructor
   */
  UuidOrdering() = default;

  /**
   * @brief Construct a UuidOrdering with the provided UUIDs
   *
   * Accepts an arbitrary number of UUIDs directly. It can be called like:
   * @code{.cpp}
   * UuidOrdering{uuid1, uuid2, uuid3};
   * @endcode
   *
   * @param[in] uuid_list The list of involved UUIDs
   */
  UuidOrdering(std::initializer_list<fuse_core::UUID> uuid_list);

  /**
   * @brief Construct a UuidOrdering with the UUIDs from the provided collection
   *
   * The \p UuidConstIterator class must meet the ForwardIterator requirements, and when
   * dereferenced must be compatible with a \p const fuse_core::UUID&.
   *
   * @param[in] first Iterator pointing to the first UUID to add to the ordering
   * @param[in] last  Iterator pointing to one past the last UUID to add to the ordering
   */
  template<typename UuidConstIterator>
  UuidOrdering(UuidConstIterator first, UuidConstIterator last);

  /**
   * @brief Returns true if there are no UUIDs in this ordering
   */
  bool empty() const;

  /**
   * @brief Returns the number of UUIDs in this ordering
   *
   * This is always equal to "last index + 1"
   */
  size_t size() const;

  /**
   * @brief Return true if the index exists in the ordering
   */
  bool exists(const unsigned int index) const;

  /**
   * @brief Return true if the UUID exists in the ordering
   */
  bool exists(const fuse_core::UUID & uuid) const;

  /**
   * @brief Add a new UUID to the back of the ordering
   *
   * If the UUID already exists, no change to the ordering will occur.
   *
   * @param[in] uuid The UUID to insert
   * @return True if the UUID was inserted, false if the UUID already existed
   */
  bool push_back(const fuse_core::UUID & uuid);

  /**
   * @brief Access the UUID stored at the provided index
   *
   * Accessing an index that does not exist results in undefined behavior
   */
  const fuse_core::UUID & operator[](const unsigned int index) const;

  /**
   * @brief Access the index associated with the provided UUID
   *
   * Accessing a UUID that does not exist results in the provided UUID being added to the ordering
   */
  unsigned int operator[](const fuse_core::UUID & uuid);

  /**
   * @brief Access the UUID stored at the provided index
   *
   * If the requested index does not exist, an out_of_range exception will be thrown.
   */
  const fuse_core::UUID & at(const unsigned int index) const;

  /**
   * @brief Access the index associated with the provided UUID
   *
   * If the requested UUID does not exist, an out_of_range exception will be thrown.
   */
  unsigned int at(const fuse_core::UUID & uuid) const;

private:
  using UuidOrderMapping = boost::bimaps::bimap<boost::bimaps::vector_of<unsigned int>,
      boost::bimaps::unordered_set_of<fuse_core::UUID>>;
  UuidOrderMapping order_;  //!< Collection that contains the Index<-->UUID mapping
};

template<typename UuidConstIterator>
UuidOrdering::UuidOrdering(UuidConstIterator first, UuidConstIterator last)
{
  for (; first != last; ++first) {
    order_.insert(order_.end(), UuidOrderMapping::value_type(order_.size(), *first));
  }
}

}  // namespace fuse_constraints

#endif  // FUSE_CONSTRAINTS__UUID_ORDERING_HPP_
