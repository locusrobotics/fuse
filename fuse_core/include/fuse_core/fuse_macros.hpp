// Copyright 2014 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/***************************************************************************
 * This file has been modified from its original published source.
 * https://raw.githubusercontent.com/ros2/rclcpp/master/rclcpp/include/rclcpp/macros.hpp
 * Modifications are provided under the BSD license.
 ***************************************************************************/

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

#ifndef FUSE_CORE__FUSE_MACROS_HPP_
#define FUSE_CORE__FUSE_MACROS_HPP_

// Required by __MAKE_SHARED_ALIGNED_DEFINITION, that uses Eigen::aligned_allocator<T>().
#include <Eigen/Core>

#include <memory>
#include <string>
#include <utility>

/**
 * Creates a custom new() implementation that ensures memory is allocated with proper byte
 * alignment. This should be added to the public section of classes or structs that contain
 * fixed-sized vectorable Eigen objects.
 *
 * For examples of vectorable types, see: https://eigen.tuxfamily.org/dox-
 * devel/group__TopicFixedSizeVectorizable.html
 *
 * This function is called internally by the SMART_PTR_DEFINITIONS_WITH_EIGEN below. You only
 * need to call this function manually if the class or struct is not adding the smart pointer
 * definitions.
 */
#if __cpp_aligned_new
  #define FUSE_MAKE_ALIGNED_OPERATOR_NEW()
#else
  #define FUSE_MAKE_ALIGNED_OPERATOR_NEW() EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif

/**
 * Defines smart pointer aliases and static functions for a typical class.
 *
 * Use in the public section of the class.
 */
#define FUSE_SMART_PTR_DEFINITIONS(...) \
  __FUSE_SHARED_PTR_ALIAS(__VA_ARGS__) \
  __FUSE_MAKE_SHARED_DEFINITION(__VA_ARGS__) \
  __FUSE_WEAK_PTR_ALIAS(__VA_ARGS__) \
  __FUSE_UNIQUE_PTR_ALIAS(__VA_ARGS__) \
  __FUSE_MAKE_UNIQUE_DEFINITION(__VA_ARGS__)

/**
 * Defines smart pointer aliases and static functions for a class that contains fixed-sized
 * vectorable Eigen member variables.
 *
 * Same as SMART_PTR_DEFINITIONS except it ensures that shared ptr memory is allocated with
 * proper byte alignment. For examples of vectorable types, see: https://eigen.tuxfamily.org/dox-
 * devel/group__TopicFixedSizeVectorizable.html
 *
 * Use in the public section of the class.
 */
#if __cpp_aligned_new
  #define FUSE_SMART_PTR_DEFINITIONS_WITH_EIGEN(...) \
    FUSE_SMART_PTR_DEFINITIONS(__VA_ARGS__)
#else
  #define FUSE_SMART_PTR_DEFINITIONS_WITH_EIGEN(...) \
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW \
    __FUSE_SHARED_PTR_ALIAS(__VA_ARGS__) \
    __FUSE_MAKE_SHARED_ALIGNED_DEFINITION(__VA_ARGS__) \
    __FUSE_WEAK_PTR_ALIAS(__VA_ARGS__) \
    __FUSE_UNIQUE_PTR_ALIAS(__VA_ARGS__) \
    __FUSE_MAKE_UNIQUE_DEFINITION(__VA_ARGS__)
#endif

/**
 * Defines smart pointers aliases only for abstract classes.
 *
 * Same as SMART_PTR_DEFINITIONS except it excludes the static method definitions which do not
 * work on pure virtual classes and classes which are not CopyConstructable.
 *
 * Use in the public section of the class.
 */
#define FUSE_SMART_PTR_ALIASES_ONLY(...) \
  __FUSE_SHARED_PTR_ALIAS(__VA_ARGS__) \
  __FUSE_WEAK_PTR_ALIAS(__VA_ARGS__) \
  __FUSE_UNIQUE_PTR_ALIAS(__VA_ARGS__)

#define __FUSE_SHARED_PTR_ALIAS(...) \
  using SharedPtr = std::shared_ptr<__VA_ARGS__>; \
  using ConstSharedPtr = std::shared_ptr<const __VA_ARGS__>;

#define __FUSE_MAKE_SHARED_DEFINITION(...) \
  template<typename ... Args> \
  static std::shared_ptr<__VA_ARGS__> \
  make_shared(Args && ... args) \
  { \
    return std::make_shared<__VA_ARGS__>(std::forward<Args>(args) ...); \
  }

#define __FUSE_MAKE_SHARED_ALIGNED_DEFINITION(...) \
  template<typename ... Args> \
  static std::shared_ptr<__VA_ARGS__> \
  make_shared(Args && ... args) \
  { \
    return std::allocate_shared<__VA_ARGS__>( \
      Eigen::aligned_allocator<__VA_ARGS__>(), \
      std::forward<Args>(args) ...); \
  }

#define __FUSE_WEAK_PTR_ALIAS(...) \
  using WeakPtr = std::weak_ptr<__VA_ARGS__>; \
  using ConstWeakPtr = std::weak_ptr<const __VA_ARGS__>;

#define __FUSE_UNIQUE_PTR_ALIAS(...) \
  using UniquePtr = std::unique_ptr<__VA_ARGS__>;

#if __cplusplus >= 201402L
  #define __FUSE_MAKE_UNIQUE_DEFINITION(...) \
    template<typename ... Args> \
    static std::unique_ptr<__VA_ARGS__> \
    make_unique(Args && ... args) \
    { \
      return std::make_unique<__VA_ARGS__>(std::forward<Args>(args) ...); \
    }
#else
  #define __FUSE_MAKE_UNIQUE_DEFINITION(...) \
    template<typename ... Args> \
    static std::unique_ptr<__VA_ARGS__> \
    make_unique(Args && ... args) \
    { \
      return std::unique_ptr<__VA_ARGS__>(new __VA_ARGS__(std::forward<Args>(args) ...)); \
    }
#endif

#endif  // FUSE_CORE__FUSE_MACROS_HPP_
