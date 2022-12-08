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
#ifndef FUSE_GRAPHS__TEST_EXAMPLE_VARIABLE_HPP_  // NOLINT{build/header_guard}
#define FUSE_GRAPHS__TEST_EXAMPLE_VARIABLE_HPP_  // NOLINT{build/header_guard}

#include <vector>

#include <fuse_core/serialization.hpp>
#include <fuse_core/uuid.hpp>
#include <fuse_core/variable.hpp>

#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/vector.hpp>


/**
 * @brief Dummy variable implementation for testing
 */
class ExampleVariable : public fuse_core::Variable
{
public:
  FUSE_VARIABLE_DEFINITIONS(ExampleVariable)

  explicit ExampleVariable(size_t N = 1)
  : fuse_core::Variable(fuse_core::uuid::generate()),
    data_(N, 0.0)
  {
  }

  size_t size() const override {return data_.size();}
  const double * data() const override {return data_.data();}
  double * data() override {return data_.data();}
  void print(std::ostream & /*stream = std::cout*/) const override {}

private:
  std::vector<double> data_;

  // Allow Boost Serialization access to private methods
  friend class boost::serialization::access;

  /**
   * @brief The Boost Serialize method that serializes all of the data members in to/out of the
   *        archive
   *
   * @param[in/out] archive - The archive object that holds the serialized class members
   * @param[in] version - The version of the archive being read/written. Generally unused.
   */
  template<class Archive>
  void serialize(Archive & archive, const unsigned int /* version */)
  {
    archive & boost::serialization::base_object<fuse_core::Variable>(*this);
    archive & data_;
  }
};

BOOST_CLASS_EXPORT(ExampleVariable);

#endif  // FUSE_GRAPHS__TEST_EXAMPLE_VARIABLE_HPP_  // NOLINT{build/header_guard}
