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
#ifndef FUSE_CORE_TEST_EXAMPLE_CONSTRAINT_H  // NOLINT{build/header_guard}
#define FUSE_CORE_TEST_EXAMPLE_CONSTRAINT_H  // NOLINT{build/header_guard}

#include <fuse_core/constraint.h>
#include <fuse_core/macros.h>
#include <fuse_core/uuid.h>

#include <initializer_list>


/**
 * @brief Dummy constraint implementation for testing
 */
class ExampleConstraint : public fuse_core::Constraint
{
public:
  SMART_PTR_DEFINITIONS(ExampleConstraint);

  ExampleConstraint(std::initializer_list<fuse_core::UUID> variable_uuid_list) :
    fuse_core::Constraint(variable_uuid_list)
  {
  }

  template<typename VariableUuidIterator>
  ExampleConstraint(VariableUuidIterator first, VariableUuidIterator last) :
    fuse_core::Constraint(first, last)
  {
  }

  void print(std::ostream& stream = std::cout) const override {}
  ceres::CostFunction* costFunction() const override { return nullptr; }
  fuse_core::Constraint::UniquePtr clone() const override { return ExampleConstraint::make_unique(*this); }
};

#endif  // FUSE_CORE_TEST_EXAMPLE_CONSTRAINT_H  // NOLINT{build/header_guard}
