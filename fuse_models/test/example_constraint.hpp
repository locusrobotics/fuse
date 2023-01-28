/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Clearpath Robotics
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

#ifndef FUSE_MODELS__TEST_EXAMPLE_CONSTRAINT_HPP_  // NOLINT{build/header_guard}
#define FUSE_MODELS__TEST_EXAMPLE_CONSTRAINT_HPP_  // NOLINT{build/header_guard}

#include <algorithm>
#include <initializer_list>
#include <iterator>
#include <string>

#include <fuse_core/constraint.hpp>
#include <fuse_core/fuse_macros.hpp>
#include <fuse_core/serialization.hpp>
#include <fuse_core/uuid.hpp>

#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>


/**
 * @brief Dummy constraint implementation for testing
 */
class ExampleConstraint : public fuse_core::Constraint
{
public:
  FUSE_CONSTRAINT_DEFINITIONS(ExampleConstraint)

  ExampleConstraint() = default;

  ExampleConstraint(
    const std::string & source,
    std::initializer_list<fuse_core::UUID> variable_uuid_list)
  : fuse_core::Constraint(source, variable_uuid_list), data(0.0)
  {
  }

  template<typename VariableUuidIterator>
  ExampleConstraint(
    const std::string & source, VariableUuidIterator first,
    VariableUuidIterator last)
  : fuse_core::Constraint(source, first, last), data(0.0)
  {
  }

  void print(std::ostream & stream = std::cout) const override
  {
    stream << type() << ":\n"
           << "  source: " << source() << '\n'
           << "  uuid: " << uuid() << '\n'
           << "  variables: [";

    const auto & variable_uuids = variables();
    if (!variable_uuids.empty()) {
      std::copy(
        variable_uuids.begin(),
        variable_uuids.end() - 1, std::ostream_iterator<fuse_core::UUID>(stream, ", "));
      stream << variable_uuids.back();
    }

    stream << "]\n"
           << "  data: " << data << '\n';
  }

  ceres::CostFunction * costFunction() const override
  {
    return nullptr;
  }

  double data;  // Public member variable just for testing

private:
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
    archive & boost::serialization::base_object<fuse_core::Constraint>(*this);
    archive & data;
  }
};

BOOST_CLASS_EXPORT(ExampleConstraint);

#endif  // FUSE_MODELS__TEST_EXAMPLE_CONSTRAINT_HPP_  // NOLINT{build/header_guard}
