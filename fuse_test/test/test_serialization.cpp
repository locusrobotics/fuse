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
#include <fuse_core/serialization.h>
#include <fuse_variables/dummy_variable.h>

#include <gtest/gtest.h>

// NOLINTNEXTLINE(build/namespaces): using operator""s issues a warning.
using namespace std::string_literals;


TEST(Serialization, VariableInstance)
{
  // Create a variable instance
  auto stamp = ros::Time(12345678, 910111213);
  auto quest = "Rescue Queen Guenever"s;
  auto variable = fuse_variables::DummyVariable(stamp, quest);
  variable.a() = 5.1;
  variable.b() = -0.03;

  // Serialize the variable
  // There are several different serialization formats. Using JSON here so the results are somewhat readable.
  std::stringstream stream;
  cereal::JSONOutputArchive archive(stream);
  archive(CEREAL_NVP(variable));
  std::cout << stream.str() << std::endl;
}

TEST(Serialization, DerivedPointer)
{
  // Create a variable instance
  auto stamp = ros::Time(12345678, 910111213);
  auto quest = "Seek the Holy Grail"s;
  auto variable = fuse_variables::DummyVariable::make_shared(stamp, quest);
  variable->a() = 5.1;
  variable->b() = -0.03;

  // Serialize the variable
  std::stringstream stream;
  {
    cereal::JSONOutputArchive archive(stream);
    archive(CEREAL_NVP(variable));
  }
  std::cout << stream.str() << std::endl;
}

TEST(Serialization, BaseClassPointer)
{
  // Create a variable instance
  auto stamp = ros::Time(12345678, 910111213);
  auto quest = "Draw the sword from the stone"s;
  fuse_core::Variable::SharedPtr variable = fuse_variables::DummyVariable::make_shared(stamp, quest);
  variable->data()[0] = 5.1;
  variable->data()[1] = -0.03;

  // Serialize the variable
  std::stringstream stream;
  {
    cereal::JSONOutputArchive archive(stream);
    archive(CEREAL_NVP(variable));
  }
  std::cout << stream.str() << std::endl;
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
