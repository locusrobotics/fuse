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
#include <fuse_core/macros.h>
#include <fuse_variables/fixed_size_variable.h>

#include <gtest/gtest.h>


class TestVariable : public fuse_variables::FixedSizeVariable<2>
{
public:
  SMART_PTR_DEFINITIONS(TestVariable);

  TestVariable() :
    uuid_(fuse_core::uuid::generate())
  {}
  virtual ~TestVariable() = default;

  fuse_core::UUID uuid() const override { return uuid_; }
  void print(std::ostream& stream = std::cout) const override {}
  fuse_core::Variable::UniquePtr clone() const override { return TestVariable::make_unique(*this); }
private:
  fuse_core::UUID uuid_;
};


TEST(FixedSizeVariable, Size)
{
  // Verify the expected size is returned
  TestVariable variable;
  EXPECT_EQ(2, variable.size());  // base class interface
  EXPECT_EQ(2, TestVariable::SIZE);  // static member variable
}

TEST(FixedSizeVariable, Data)
{
  // Verify access to the data methods
  TestVariable variable;
  EXPECT_NO_THROW(variable.data()[0] = 1.0);
  EXPECT_NO_THROW(variable.data()[1] = 2.0);
  const TestVariable& const_variable = variable;
  bool success = true;
  EXPECT_NO_THROW(success = success && const_variable.data()[0] == 1.0);
  EXPECT_NO_THROW(success = success && const_variable.data()[1] == 2.0);
  EXPECT_TRUE(success);
}

TEST(FixedSizeVariable, Array)
{
  // Verify access to the array
  TestVariable variable;
  EXPECT_NO_THROW(variable.array()[0] = 1.0);
  EXPECT_NO_THROW(variable.array().at(1) = 2.0);
  EXPECT_NO_THROW(variable.array().front() = 3.0);
  EXPECT_NO_THROW(variable.array().back() = 4.0);
  const TestVariable& const_variable = variable;
  bool success = true;
  EXPECT_NO_THROW(success = success && const_variable.array()[0] == 3.0);
  EXPECT_NO_THROW(success = success && const_variable.array().at(1) == 4.0);
  EXPECT_NO_THROW(success = success && const_variable.array().front() == 3.0);
  EXPECT_NO_THROW(success = success && const_variable.array().back() == 4.0);
  EXPECT_TRUE(success);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
