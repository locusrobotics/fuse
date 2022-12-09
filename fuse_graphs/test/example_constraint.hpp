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
#ifndef FUSE_GRAPHS__TEST_EXAMPLE_CONSTRAINT_HPP_  // NOLINT{build/header_guard}
#define FUSE_GRAPHS__TEST_EXAMPLE_CONSTRAINT_HPP_  // NOLINT{build/header_guard}

#include <ceres/autodiff_cost_function.h>

#include <string>

#include <fuse_core/constraint.hpp>
#include <fuse_core/fuse_macros.hpp>
#include <fuse_core/serialization.hpp>
#include <fuse_core/uuid.hpp>

#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>


/**
 * @brief Dummy cost function used for testing
 */
class ExampleFunctor
{
public:
  explicit ExampleFunctor(const double & b)
  : b_(b)
  {
  }

  template<typename T>
  bool operator()(const T * const variable, T * residual) const
  {
    residual[0] = variable[0] - T(b_);
    return true;
  }

private:
  double b_;
};

/**
 * @brief Dummy constraint implementation for testing
 */
class ExampleConstraint : public fuse_core::Constraint
{
public:
  FUSE_CONSTRAINT_DEFINITIONS(ExampleConstraint)

  ExampleConstraint() = default;

  explicit ExampleConstraint(const std::string & source, const fuse_core::UUID & variable_uuid)
  : fuse_core::Constraint(source, {variable_uuid}),  // NOLINT
    data(0.0)
  {
  }

  void print(std::ostream & /*stream = std::cout*/) const override {}
  ceres::CostFunction * costFunction() const override
  {
    return new ceres::AutoDiffCostFunction<ExampleFunctor, 1, 1>(new ExampleFunctor(data));
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

#endif  // FUSE_GRAPHS__TEST_EXAMPLE_CONSTRAINT_HPP_  // NOLINT{build/header_guard}
