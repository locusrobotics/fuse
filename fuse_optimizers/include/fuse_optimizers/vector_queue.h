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
#ifndef FUSE_OPTIMIZERS_VECTOR_QUEUE_H
#define FUSE_OPTIMIZERS_VECTOR_QUEUE_H

#include <fuse_core/macros.h>

#include <algorithm>
#include <functional>
#include <utility>
#include <vector>


namespace fuse_optimizers
{

/**
 * @brief A container that maintains all the elements in a sorted order based on a key, but stores them
 *        in a std::vector
 */
template <typename KEY, typename T, typename Compare = std::less<KEY>>
class VectorQueue
{
public:
  SMART_PTR_DEFINITIONS(VectorQueue<KEY, T, Compare>);
  using key_type = KEY;
  using value_type = T;

  /**
   * @brief Constructor
   */
  VectorQueue() = default;

  /**
   * @brief Constructor
   */
  explicit VectorQueue(const Compare& comp) :
    comparitor_(comp)
  {
  }

  /**
   * @brief Destructor
   */
  virtual ~VectorQueue() = default;

  /**
   * @brief Return true if the vector contains no elements
   */
  bool empty() const
  {
    return container_.empty();
  }

  /**
   * @brief Return the number of elements in the vector
   */
  size_t size() const
  {
    return container_.size();
  }

  /**
   * @brief Access the first element in the vector
   */
  const key_type& front_key() const
  {
    return container_.front().first;
  }

  /**
   * @brief Access the first element in the vector
   */
  const value_type& front_value() const
  {
    return container_.front().second;
  }

  /**
   * @brief Access the first element in the vector
   */
  value_type& front_value()
  {
    return container_.front().second;
  }

  /**
   * @brief Access the last element in the vector
   */
  const key_type& back_key() const
  {
    return container_.back().first;
  }

  /**
   * @brief Access the last element in the vector
   */
  const value_type& back_value() const
  {
    return container_.back().second;
  }

  /**
   * @brief Access the last element in the vector
   */
  value_type& back_value()
  {
    return container_.back().second;
  }

  /**
   * @brief Insert a new element into the vector
   */
  void insert(key_type key, value_type value)
  {
    // Find the insertion point
    auto insertion_point = std::upper_bound(container_.begin(), container_.end(), key, comparitor_);
    container_.insert(insertion_point, std::make_pair(std::move(key), std::move(value)));
  }

  /**
   * @brief Remove the first element of the vector
   */
  void pop_front()
  {
    container_.erase(container_.begin());
  }

  /**
   * @brief Remove the last element of the vector
   */
  void pop_back()
  {
    container_.pop_back();
  }

protected:
  using Container = std::vector<std::pair<key_type, value_type>>;

  struct Comparitor : std::binary_function<value_type, value_type, bool>
  {
    explicit Comparitor(const Compare& key_comparitor = Compare()) :
      key_comparitor_(key_comparitor)
    {
    }

    bool operator()(const key_type& lhs, const key_type& rhs) const
    {
      return key_comparitor_(lhs, rhs);
    }

    bool operator()(const typename Container::value_type& lhs, const key_type& rhs) const
    {
      return key_comparitor_(lhs.first, rhs);
    }

    bool operator()(const key_type& lhs, const typename Container::value_type& rhs) const
    {
      return key_comparitor_(lhs, rhs.first);
    }

  private:
    Compare key_comparitor_;
  };

  Container container_;  //!< The container used to hold the elements
  const Comparitor comparitor_;  //!< Compare functor used to compare value_type objects based on their key
};

}  // namespace fuse_optimizers

#endif  // FUSE_OPTIMIZERS_VECTOR_QUEUE_H
