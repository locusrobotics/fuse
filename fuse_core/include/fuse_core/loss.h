/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Clearpath Robotics
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
#ifndef FUSE_CORE_LOSS_H
#define FUSE_CORE_LOSS_H

#include <fuse_core/fuse_macros.h>
#include <fuse_core/serialization.h>

#include <boost/serialization/access.hpp>
#include <boost/type_index/stl_type_index.hpp>
#include <ceres/loss_function.h>

#include <iostream>
#include <string>

/**
 * @brief Implementation of the clone() member function for derived classes
 *
 * Usage:
 * @code{.cpp}
 * class Derived : public Loss
 * {
 * public:
 *   FUSE_LOSS_CLONE_DEFINITION(Derived)
 *   // The rest of the derived loss function implementation
 * }
 * @endcode
 */
#define FUSE_LOSS_CLONE_DEFINITION(...) \
  fuse_core::Loss::UniquePtr clone() const override \
  { \
    return __VA_ARGS__::make_unique(*this); \
  }

/**
 * @brief Implementation of the serialize() and deserialize() member functions for derived classes
 *
 * Usage:
 * @code{.cpp}
 * class Derived : public Loss
 * {
 * public:
 *   FUSE_LOSS_SERIALIZE_DEFINITION(Derived)
 *   // The rest of the derived loss function implementation
 * }
 * @endcode
 */
#define FUSE_LOSS_SERIALIZE_DEFINITION(...) \
  void serialize(fuse_core::BinaryOutputArchive& archive) const override \
  { \
    archive << *this; \
  }  /* NOLINT */ \
  void serialize(fuse_core::TextOutputArchive& archive) const override \
  { \
    archive << *this; \
  }  /* NOLINT */ \
  void deserialize(fuse_core::BinaryInputArchive& archive) override \
  { \
    archive >> *this; \
  }  /* NOLINT */ \
  void deserialize(fuse_core::TextInputArchive& archive) override \
  { \
    archive >> *this; \
  }

/**
 * @brief Implements the type() member function using the suggested implementation
 *
 * Also creates a static detail::type() function that may be used without an object instance
 *
 * Usage:
 * @code{.cpp}
 * class Derived : public Loss
 * {
 * public:
 *   FUSE_LOSS_TYPE_DEFINITION(Derived)
 *   // The rest of the derived loss function implementation
 * }
 * @endcode
 */
#define FUSE_LOSS_TYPE_DEFINITION(...) \
  struct detail \
  { \
    static std::string type() \
    { \
      return boost::typeindex::stl_type_index::type_id<__VA_ARGS__>().pretty_name(); \
    }  /* NOLINT */ \
  };  /* NOLINT */ \
  std::string type() const override \
  { \
    return detail::type(); \
  }

/**
 * @brief Convenience function that creates the required pointer aliases, clone() method, and type() method
 *
 * Usage:
 * @code{.cpp}
 * class Derived : public Loss
 * {
 * public:
 *   FUSE_LOSS_DEFINITIONS(Derived)
 *   // The rest of the derived loss function implementation
 * }
 * @endcode
 */
#define FUSE_LOSS_DEFINITIONS(...) \
  FUSE_SMART_PTR_DEFINITIONS(__VA_ARGS__) \
  FUSE_LOSS_TYPE_DEFINITION(__VA_ARGS__) \
  FUSE_LOSS_CLONE_DEFINITION(__VA_ARGS__) \
  FUSE_LOSS_SERIALIZE_DEFINITION(__VA_ARGS__)


namespace fuse_core
{

/**
 * @brief The Loss function interface definition.
 *
 * This class encapsulates the ceres::LossFunction class, adding the ability to serialize it.
 *
 * The attributes from all derived ceres::LossFunction classes are private, so we cannot
 * serialize them directly. For that reason we need this fuse_core::Loss to provide additional
 * functionality on top of them.
 *
 * See http://ceres-solver.org/nnls_modeling.html#lossfunction for a detailed description about loss functions.
 * Basically, a loss function defines the penalty associated with a specific amount error. By default, or if a NULL
 * pointer is provided as the loss function, the penalty will be quadratic. This is the loss function associated
 * with standard least-squares optimization. By providing a different loss function, the penalty for the constraint's
 * error can be altered.
 *
 * This is generally done to reduce the effect of outlier measurements that made it into the optimization problem.
 * It is always better to remove outliers before they make it into the optimization problem, but no method is perfect.
 * Using robust loss functions can significantly improve the results and stability of the solution in the presence
 * of outlier measurements.
 */
class Loss
{
public:
  FUSE_SMART_PTR_ALIASES_ONLY(Loss)

  static constexpr ceres::Ownership Ownership =
      ceres::Ownership::TAKE_OWNERSHIP;  //!< The ownership of the ceres::LossFunction* returned by lossFunction()

  /**
   * @brief Default constructor
   */
  Loss() = default;

  /**
   * @brief Destructor
   */
  virtual ~Loss() = default;

  /**
   * @brief Perform any required post-construction initialization, such as reading from the parameter server.
   *
   * This will be called on each plugin after construction.
   *
   * @param[in] name A unique name to initialize this plugin instance, such as from the parameter server.
   */
  virtual void initialize(const std::string& name) = 0;

  /**
   * @brief Returns a unique name for this loss function type.
   *
   * The loss function type string must be unique for each class. As such, the fully-qualified class name is an
   * excellent choice for the type string.
   */
  virtual std::string type() const = 0;

  /**
   * @brief Print a human-readable description of the loss function to the provided stream.
   *
   * @param  stream The stream to write to. Defaults to stdout.
   */
  virtual void print(std::ostream& stream = std::cout) const = 0;

  /**
   * @brief Return a raw pointer to a ceres::LossFunction that implements the loss function
   *
   * The Ceres interface requires a raw pointer. Ceres will take ownership of the pointer and promises to properly
   * delete the loss function when it is done. Additionally, Fuse promises that the Loss object will outlive any
   * generated loss functions (i.e. the Ceres objects will be destroyed before the Loss Function objects). This
   * guarantee may allow optimizations for the creation of the loss function objects.
   *
   * @return A base pointer to an instance of a derived ceres::LossFunction.
   */
  virtual ceres::LossFunction* lossFunction() const = 0;

  /**
   * @brief Perform a deep copy of the Loss and return a unique pointer to the copy
   *
   * This can/should be implemented as follows in all derived classes:
   * @code{.cpp}
   * return Derived::make_unique(*this);
   * @endcode
   *
   * @return A unique pointer to a new instance of the most-derived Loss
   */
  virtual Loss::UniquePtr clone() const = 0;

  /**
   * @brief Serialize this Loss into the provided binary archive
   *
   * This can/should be implemented as follows in all derived classes:
   * @code{.cpp}
   * archive << *this;
   * @endcode
   *
   * @param[out] archive - The archive to serialize this loss function into
   */
  virtual void serialize(fuse_core::BinaryOutputArchive& /* archive */) const = 0;

  /**
   * @brief Serialize this Loss into the provided text archive
   *
   * This can/should be implemented as follows in all derived classes:
   * @code{.cpp}
   * archive << *this;
   * @endcode
   *
   * @param[out] archive - The archive to serialize this loss function into
   */
  virtual void serialize(fuse_core::TextOutputArchive& /* archive */) const = 0;

  /**
   * @brief Deserialize data from the provided binary archive into this Loss
   *
   * This can/should be implemented as follows in all derived classes:
   * @code{.cpp}
   * archive >> *this;
   * @endcode
   *
   * @param[in] archive - The archive holding serialized Loss data
   */
  virtual void deserialize(fuse_core::BinaryInputArchive& /* archive */) = 0;

  /**
   * @brief Deserialize data from the provided text archive into this Loss
   *
   * This can/should be implemented as follows in all derived classes:
   * @code{.cpp}
   * archive >> *this;
   * @endcode
   *
   * @param[in] archive - The archive holding serialized Loss data
   */
  virtual void deserialize(fuse_core::TextInputArchive& /* archive */) = 0;

private:
  // Allow Boost Serialization access to private methods
  friend class boost::serialization::access;

  /**
   * @brief The Boost Serialize method that serializes all of the data members in to/out of the archive
   *
   * @param[in/out] archive - The archive object that holds the serialized class members
   * @param[in] version - The version of the archive being read/written. Generally unused.
   */
  template<class Archive>
  void serialize(Archive& /* archive */, const unsigned int /* version */)
  {
  }
};

/**
 * Stream operator implementation used for all derived Loss classes.
 */
std::ostream& operator <<(std::ostream& stream, const Loss& loss);

}  // namespace fuse_core

#endif  // FUSE_CORE_LOSS_H
