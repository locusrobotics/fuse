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
#ifndef EXAMPLE_VARIABLE_HPP_
#define EXAMPLE_VARIABLE_HPP_

#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <ceres/rotation.h>  // NOLINT[build/include_order]

#include <fuse_core/ceres_macros.hpp>
#include <fuse_core/fuse_macros.hpp>
#include <fuse_core/serialization.hpp>
#include <fuse_core/uuid.hpp>
#include <fuse_core/variable.hpp>

/**
 * @brief Dummy variable implementation for testing
 */
class ExampleVariable : public fuse_core::Variable
{
public:
  FUSE_VARIABLE_DEFINITIONS(ExampleVariable)

  ExampleVariable()
  : fuse_core::Variable(fuse_core::uuid::generate()),
    data_(0.0)
  {
  }

  explicit ExampleVariable(const rclcpp::Time & stamp)
  : fuse_core::Variable(fuse_core::uuid::generate("ExampleVariable", stamp)),
    data_(0.0)
  {
  }

  size_t size() const override {return 1;}
  const double * data() const override {return &data_;}
  double * data() override {return &data_;}
  void print(std::ostream & stream = std::cout) const override
  {
    stream << type() << ":\n"
           << "  uuid: " << uuid() << "\n"
           << "  data:\n"
           << "  - [0]: " << data()[0] << "\n";
  }

#if CERES_SUPPORTS_MANIFOLDS
  /**
   * @brief Create a null Ceres manifold
   *
   * Overriding the manifold() method prevents additional processing with the ManifoldAdapter
   */
  fuse_core::Manifold * manifold() const override {return nullptr;}
#endif

private:
  double data_;

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

/**
 * @brief This is a test parameterization based on a quaternion representing a 3D rotation
 */
class LegacyParameterization : public fuse_core::LocalParameterization
{
public:
  FUSE_SMART_PTR_DEFINITIONS(LegacyParameterization)

  int GlobalSize() const override {return 4;}

  int LocalSize() const override {return 3;}

  bool Plus(const double * x, const double * delta, double * x_plus_delta) const override
  {
    double q_delta[4];
    ceres::AngleAxisToQuaternion(delta, q_delta);
    ceres::QuaternionProduct(x, q_delta, x_plus_delta);
    return true;
  }

  bool ComputeJacobian(const double * x, double * jacobian) const override
  {
    double x0 = x[0] / 2;
    double x1 = x[1] / 2;
    double x2 = x[2] / 2;
    double x3 = x[3] / 2;
    /* *INDENT-OFF* */
    jacobian[0] = -x1; jacobian[1]  = -x2; jacobian[2]  = -x3;  // NOLINT
    jacobian[3] =  x0; jacobian[4]  = -x3; jacobian[5]  =  x2;  // NOLINT
    jacobian[6] =  x3; jacobian[7]  =  x0; jacobian[8]  = -x1;  // NOLINT
    jacobian[9] = -x2; jacobian[10] =  x1; jacobian[11] =  x0;  // NOLINT
    /* *INDENT-ON* */
    return true;
  }

  bool Minus(const double * x, const double * y, double * y_minus_x) const override
  {
    double x_inverse[4];
    x_inverse[0] = x[0];
    x_inverse[1] = -x[1];
    x_inverse[2] = -x[2];
    x_inverse[3] = -x[3];

    double q_delta[4];
    ceres::QuaternionProduct(x_inverse, y, q_delta);
    ceres::QuaternionToAngleAxis(q_delta, y_minus_x);
    return true;
  }

  bool ComputeMinusJacobian(const double * x, double * jacobian) const override
  {
    double x0 = x[0] * 2;
    double x1 = x[1] * 2;
    double x2 = x[2] * 2;
    double x3 = x[3] * 2;
    /* *INDENT-OFF* */
    jacobian[0] = -x1; jacobian[1]  =  x0; jacobian[2]  =  x3;  jacobian[3]  = -x2;  // NOLINT
    jacobian[4] = -x2; jacobian[5]  = -x3; jacobian[6]  =  x0;  jacobian[7]  =  x1;  // NOLINT
    jacobian[8] = -x3; jacobian[9]  =  x2; jacobian[10] = -x1;  jacobian[11] =  x0;  // NOLINT
    /* *INDENT-ON* */
    return true;
  }

private:
  // Allow Boost Serialization access to private methods
  friend class boost::serialization::access;

  /**
   * @brief The Boost Serialize method that serializes all of the data members in to/out of
   *        the archive
   *
   * @param[in/out] archive - The archive object that holds the serialized class members
   * @param[in] version - The version of the archive being read/written. Generally unused.
   */
  template<class Archive>
  void serialize(Archive & archive, const unsigned int /* version */)
  {
    archive & boost::serialization::base_object<fuse_core::LocalParameterization>(*this);
  }
};

class LegacyVariable : public fuse_core::Variable
{
public:
  FUSE_VARIABLE_DEFINITIONS(LegacyVariable);

  LegacyVariable()
  : fuse_core::Variable(fuse_core::uuid::generate()),
    data_{1.0, 0.0, 0.0, 0.0}
  {
  }

  size_t size() const override {return 4;}
  const double * data() const override {return data_;}
  double * data() override {return data_;}
  void print(std::ostream & /*stream = std::cout*/) const override {}

  /**
   * @brief Returns the number of elements of the local parameterization space.
   *
   * While a quaternion has 4 parameters, a 3D rotation only has 3 degrees of freedom. Hence, the local
   * parameterization space is only size 3.
   */
  size_t localSize() const override {return 3u;}

  /**
   * @brief Provides a Ceres local parameterization for the quaternion
   *
   * @return A pointer to a local parameterization object that indicates how to "add" increments to the quaternion
   */
  fuse_core::LocalParameterization * localParameterization() const override
  {
    return new LegacyParameterization();
  }

private:
  double data_[4];

  // Allow Boost Serialization access to private methods
  friend class boost::serialization::access;

  /**
   * @brief The Boost Serialize method that serializes all of the data members in to/out of the archive
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
BOOST_CLASS_EXPORT(LegacyParameterization);
BOOST_CLASS_EXPORT(LegacyVariable);

#endif  // EXAMPLE_VARIABLE_HPP_
