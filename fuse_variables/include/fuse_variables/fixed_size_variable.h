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
#ifndef FUSE_VARIABLES_FIXED_SIZE_VARIABLE_H
#define FUSE_VARIABLES_FIXED_SIZE_VARIABLE_H

#include <fuse_core/macros.h>
#include <fuse_core/variable.h>

#include <array>


namespace fuse_variables
{

/**
 * @brief A Variable base class for fixed-sized variables
 *
 * The FixedSizeVariable class implements a statically sized array to hold the scalar values. The size of the variable
 * is provided as the template argument \p N when creating a derived class. The FixedSizeVariable class implements the
 * Variable::data() accessor methods, and provides access to the scalar values as a std::array. This allows easier
 * manipulation in C++ (iterators, range-based for loops, etc.). The FixedSizeVariable class is designed for variables
 * where the size of the state vector is known at compile time...which should be almost all variable types. The
 * dimension of typical variable types (points, poses, calibration parameters) are all known at design/compile time.
 */
template<size_t N>
class FixedSizeVariable : public fuse_core::Variable
{
public:
  SMART_PTR_ALIASES_ONLY(FixedSizeVariable<N>);

  /**
   * @brief A static version of the variable size
   */
  constexpr static size_t SIZE = N;

  /**
   * @brief Constructor
   */
  FixedSizeVariable() :
    Variable(),
    data_{}  // zero-initialize the data array
  {}

  /**
   * @brief Destructor
   */
  virtual ~FixedSizeVariable() = default;

  /**
   * @brief Returns the number of elements of this variable.
   *
   * The number of scalar values contained by this variable type is defined by the class template parameter \p N.
   */
  size_t size() const override { return N; }

  /**
   * @brief Read-only access to the variable data
   */
  const double* data() const override { return data_.data(); }

  /**
   * @brief Read-write access to the variable data
   */
  double* data() override { return data_.data(); }

  /**
   * @brief Read-only access to the variable data as a std::array
   */
  const std::array<double, N>& array() const { return data_; }

  /**
   * @brief Read-write access to the variable data as a std::array
   */
  std::array<double, N>& array() { return data_; }

protected:
  std::array<double, N> data_;  //!< Fixed-sized, contiguous memory for holding the variable data members
};

// Define the constant that was decalred above
template<size_t N>
constexpr size_t FixedSizeVariable<N>::SIZE;
}  // namespace fuse_variables

#endif  // FUSE_VARIABLES_FIXED_SIZE_VARIABLE_H
