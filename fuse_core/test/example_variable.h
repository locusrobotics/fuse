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
#ifndef FUSE_CORE_TEST_EXAMPLE_VARIABLE_H
#define FUSE_CORE_TEST_EXAMPLE_VARIABLE_H

#include <fuse_core/macros.h>
#include <fuse_core/uuid.h>
#include <fuse_core/variable.h>


/**
 * @brief Dummy variable implementation for testing
 */
class ExampleVariable : public fuse_core::Variable
{
public:
  SMART_PTR_DEFINITIONS(ExampleVariable);

  ExampleVariable() :
    data_(0.0),
    uuid_(fuse_core::uuid::generate())
  {
  }

  fuse_core::UUID uuid() const override { return uuid_; }
  size_t size() const override { return 1; }
  const double* data() const override { return &data_; };
  double* data() override { return &data_; };
  void print(std::ostream& stream = std::cout) const override {}
  fuse_core::Variable::UniquePtr clone() const override { return ExampleVariable::make_unique(*this); }

private:
  double data_;
  fuse_core::UUID uuid_;
};

#endif  // FUSE_CORE_TEST_EXAMPLE_VARIABLE_H
