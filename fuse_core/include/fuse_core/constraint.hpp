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
#ifndef FUSE_CORE__CONSTRAINT_HPP_
#define FUSE_CORE__CONSTRAINT_HPP_

#include <ceres/cost_function.h>
#include <ceres/loss_function.h>

#include <initializer_list>
#include <iostream>
#include <memory>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include <boost/serialization/access.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/type_index/stl_type_index.hpp>
#include <fuse_core/fuse_macros.hpp>
#include <fuse_core/loss.hpp>
#include <fuse_core/serialization.hpp>
#include <fuse_core/uuid.hpp>

/**
 * @brief Implementation of the clone() member function for derived classes
 *
 * Usage:
 * @code{.cpp}
 * class Derived : public Constraint
 * {
 * public:
 *   FUSE_CONSTRAINT_CLONE_DEFINITION(Derived)
 *   // The rest of the derived constraint implementation
 * }
 * @endcode
 */
#define FUSE_CONSTRAINT_CLONE_DEFINITION(...) \
  fuse_core::Constraint::UniquePtr clone() const override \
  { \
    return __VA_ARGS__::make_unique(*this); \
  }

/**
 * @brief Implementation of the serialize() and deserialize() member functions for derived classes
 *
 * Usage:
 * @code{.cpp}
 * class Derived : public Constraint
 * {
 * public:
 *   FUSE_CONSTRAINT_SERIALIZE_DEFINITION(Derived)
 *   // The rest of the derived constraint implementation
 * }
 * @endcode
 */
#define FUSE_CONSTRAINT_SERIALIZE_DEFINITION(...) \
  void serialize(fuse_core::BinaryOutputArchive & archive) const override \
  { \
    archive << *this; \
  }  /* NOLINT */ \
  void serialize(fuse_core::TextOutputArchive & archive) const override \
  { \
    archive << *this; \
  }  /* NOLINT */ \
  void deserialize(fuse_core::BinaryInputArchive & archive) override \
  { \
    archive >> *this; \
  }  /* NOLINT */ \
  void deserialize(fuse_core::TextInputArchive & archive) override \
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
 * class Derived : public Constraint
 * {
 * public:
 *   FUSE_CONSTRAINT_TYPE_DEFINITION(Derived)
 *   // The rest of the derived constraint implementation
 * }
 * @endcode
 */
#define FUSE_CONSTRAINT_TYPE_DEFINITION(...) \
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
 * @brief Convenience function that creates the required pointer aliases, clone() method, and type()
 *        method
 *
 * Usage:
 * @code{.cpp}
 * class Derived : public Constraint
 * {
 * public:
 *   FUSE_CONSTRAINT_DEFINITIONS(Derived)
 *   // The rest of the derived constraint implementation
 * }
 * @endcode
 */
#define FUSE_CONSTRAINT_DEFINITIONS(...) \
  FUSE_SMART_PTR_DEFINITIONS(__VA_ARGS__) \
  FUSE_CONSTRAINT_TYPE_DEFINITION(__VA_ARGS__) \
  FUSE_CONSTRAINT_CLONE_DEFINITION(__VA_ARGS__) \
  FUSE_CONSTRAINT_SERIALIZE_DEFINITION(__VA_ARGS__)

/**
 * @brief Convenience function that creates the required pointer aliases, clone() method, and type()
 *        method for derived Constraint classes that have fixed-sized Eigen member objects.
 *
 * Usage:
 * @code{.cpp}
 * class Derived : public Constraint
 * {
 * public:
 *   FUSE_CONSTRAINT_DEFINITIONS_WTIH_EIGEN(Derived)
 *   // The rest of the derived constraint implementation
 * }
 * @endcode
 */
#define FUSE_CONSTRAINT_DEFINITIONS_WITH_EIGEN(...) \
  FUSE_SMART_PTR_DEFINITIONS_WITH_EIGEN(__VA_ARGS__) \
  FUSE_CONSTRAINT_TYPE_DEFINITION(__VA_ARGS__) \
  FUSE_CONSTRAINT_CLONE_DEFINITION(__VA_ARGS__) \
  FUSE_CONSTRAINT_SERIALIZE_DEFINITION(__VA_ARGS__)


namespace fuse_core
{

/**
 * @brief The Constraint interface definition.
 *
 * A Constraint defines a cost function that is connected to one or more variables. This base
 * class defines the required interface of all Constraint objects, and holds the ordered list of
 * involved variable UUIDs. All other functionality is left to the derived classes to implement.
 *
 * Most importantly, the implementation of the cost function is left to the derived classes,
 * allowing arbitrarily complex sensor models to be implemented outside of the core fuse
 * packages. The cost function must be a valid ceres::CostFunction object. Ceres provides many
 * nice features to make implementing the cost function easier, including an automatic
 * differentiation system. Please see the Ceres documentation for details on creating valid
 * ceres::CostFunction objects (http://ceres-solver.org/nnls_modeling.html). In addition to the
 * cost function itself, an optional loss function may be provided. Loss functions provide a
 * mechanism for reducing the impact of outlier measurements on the final optimization results.
 * Again, see the Ceres documentation for details (http://ceres-
 * solver.org/nnls_modeling.html#lossfunction).
 */
class Constraint
{
public:
  FUSE_SMART_PTR_ALIASES_ONLY(Constraint)

  /**
   * @brief Default constructor
   */
  Constraint() = default;

  /**
   * @brief Constructor
   *
   * Accepts an arbitrary number of variable UUIDs directly. It can be called like:
   * @code{.cpp}
   * Constraint("source", {uuid1, uuid2, uuid3});
   * @endcode
   *
   * @param[in] variable_uuid_list The list of involved variable UUIDs
   */
  Constraint(const std::string & source, std::initializer_list<UUID> variable_uuid_list);

  /**
   * @brief Constructor
   *
   * Accepts an arbitrary number of variable UUIDs stored in a container using iterators.
   */
  template<typename VariableUuidIterator>
  Constraint(const std::string & source, VariableUuidIterator first, VariableUuidIterator last);

  /**
   * @brief Destructor
   */
  virtual ~Constraint() = default;

  /**
   * @brief Returns a unique name for this constraint type.
   *
   * The constraint type string must be unique for each class. As such, the fully-qualified
   * class name is an excellent choice for the type string.
   */
  virtual std::string type() const = 0;

  /**
   * @brief Returns the UUID for this constraint.
   *
   * Each constraint will generate a unique, random UUID during construction.
   */
  const UUID & uuid() const {return uuid_;}

  /**
   * @brief Returns the name of the sensor or motion model that generated this constraint
   */
  const std::string & source() const {return source_;}

  /**
   * @brief Print a human-readable description of the constraint to the provided stream.
   *
   * @param  stream The stream to write to. Defaults to stdout.
   */
  virtual void print(std::ostream & stream = std::cout) const = 0;

  /**
   * @brief Create a new Ceres cost function and return a raw pointer to it.
   *
   * The Ceres interface requires a raw pointer. Ceres will take ownership of the pointer and
   * promises to properly delete the cost function when it is done. Additionally, fuse promises
   * that the Constraint object will outlive any generated cost functions (i.e. the Ceres
   * objects will be destroyed before the Constraint objects). This guarantee may allow
   * optimizations for the creation of the cost function objects.
   *
   * @return A base pointer to an instance of a derived ceres::CostFunction.
   */
  virtual ceres::CostFunction * costFunction() const = 0;

  /**
   * @brief Read-only access to the loss.
   *
   * The loss interfaces wraps a ceres::LossFunction that can be accessed directly with
   * lossFunction().
   *
   * @return A base shared pointer to an instance of a derived Loss.
   */
  Loss::SharedPtr loss() const
  {
    return loss_;
  }

  /**
   * @brief Set the constraint loss function
   *
   * @param[in] loss - The loss function
   */
  void loss(Loss::SharedPtr loss)
  {
    loss_ = std::move(loss);
  }

  /**
   * @brief Read-only access to the Ceres loss function.
   *
   * @return A base pointer to an instance of a derived ceres::LossFunction.
   */
  ceres::LossFunction * lossFunction() const
  {
    return loss_ ? loss_->lossFunction() : nullptr;
  }

  /**
   * @brief Perform a deep copy of the Constraint and return a unique pointer to the copy
   *
   * This can/should be implemented as follows in all derived classes:
   * @code{.cpp}
   * return Derived::make_unique(*this);
   * @endcode
   *
   * @return A unique pointer to a new instance of the most-derived Constraint
   */
  virtual Constraint::UniquePtr clone() const = 0;

  /**
   * @brief Read-only access to the ordered list of variable UUIDs involved in this constraint
   */
  const std::vector<UUID> & variables() const {return variables_;}

  /**
   * @brief Serialize this Constraint into the provided binary archive
   *
   * This can/should be implemented as follows in all derived classes:
   * @code{.cpp}
   * archive << *this;
   * @endcode
   *
   * @param[out] archive - The archive to serialize this constraint into
   */
  virtual void serialize(fuse_core::BinaryOutputArchive & /* archive */) const = 0;

  /**
   * @brief Serialize this Constraint into the provided text archive
   *
   * This can/should be implemented as follows in all derived classes:
   * @code{.cpp}
   * archive << *this;
   * @endcode
   *
   * @param[out] archive - The archive to serialize this constraint into
   */
  virtual void serialize(fuse_core::TextOutputArchive & /* archive */) const = 0;

  /**
   * @brief Deserialize data from the provided binary archive into this Constraint
   *
   * This can/should be implemented as follows in all derived classes:
   * @code{.cpp}
   * archive >> *this;
   * @endcode
   *
   * @param[in] archive - The archive holding serialized Constraint data
   */
  virtual void deserialize(fuse_core::BinaryInputArchive & /* archive */) = 0;

  /**
   * @brief Deserialize data from the provided text archive into this Constraint
   *
   * This can/should be implemented as follows in all derived classes:
   * @code{.cpp}
   * archive >> *this;
   * @endcode
   *
   * @param[in] archive - The archive holding serialized Constraint data
   */
  virtual void deserialize(fuse_core::TextInputArchive & /* archive */) = 0;

private:
  std::string source_;  //!< The name of the sensor or motion model that generated this constraint
  UUID uuid_;  //!< The unique ID associated with this constraint
  std::vector<UUID> variables_;  //!< The ordered set of variables involved with this constraint
  std::shared_ptr<Loss> loss_{nullptr};    //!< The loss function

  // Allow Boost Serialization access to private methods
  friend class boost::serialization::access;

  /**
   * @brief The Boost Serialize method that serializes all of the data members in to/out of the
   *        archive
   *
   * This method, or a combination of save() and load() methods, must be implemented by all
   * derived classes. See documentation on Boost Serialization for information on how to
   * implement the serialize() method.
   * https://www.boost.org/doc/libs/1_70_0/libs/serialization/doc/
   *
   * @param[in/out] archive - The archive object that holds the serialized class members
   * @param[in] version - The version of the archive being read/written. Generally unused.
   */
  template<class Archive>
  void serialize(Archive & archive, const unsigned int /* version */)
  {
    archive & source_;
    archive & uuid_;
    archive & variables_;
    archive & loss_;
  }
};

/**
 * Stream operator implementation used for all derived Constraint classes.
 */
std::ostream & operator<<(std::ostream & stream, const Constraint & constraint);


template<typename VariableUuidIterator>
Constraint::Constraint(
  const std::string & source, VariableUuidIterator first,
  VariableUuidIterator last)
: source_(source),
  uuid_(uuid::generate()),
  variables_(first, last)
{
}

}  // namespace fuse_core

#endif  // FUSE_CORE__CONSTRAINT_HPP_
