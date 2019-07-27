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
#ifndef FUSE_VARIABLES_DUMMY_VARIABLE_H
#define FUSE_VARIABLES_DUMMY_VARIABLE_H

#include <fuse_core/eigen.h>
#include <fuse_core/serialization.h>
#include <fuse_core/uuid.h>
#include <fuse_core/variable.h>
#include <ros/time.h>

#include <cereal/types/array.hpp>
#include <cereal/types/base_class.hpp>
#include <cereal/types/polymorphic.hpp>

#include <array>
#include <ostream>
#include <string>


namespace fuse_variables
{

/**
 * @brief Dummy variable to play with serialization
 */
class DummyVariable : public fuse_core::Variable
{
public:
  FUSE_VARIABLE_DEFINITIONS(DummyVariable);

  /**
   * @brief All plugins must have a default constructor
   */
  DummyVariable() = default;

  /**
   * @brief Construct a dummy variable
   *
   * @param[in] stamp The timestamp attached to this variable.
   * @param[in] quest An unlikely variable property that no one could have predicted
   */
  explicit DummyVariable(const ros::Time& stamp, const std::string& quest = "To seek the Holy Grail");

  /**
   * @brief Read-write access to the A value.
   */
  double& a() { return data_[0]; }

  /**
   * @brief Read-only access to the A value.
   */
  const double& a() const { return data_[0]; }

  /**
   * @brief Read-write access to the B value.
   */
  double& b() { return data_[1]; }

  /**
   * @brief Read-only access to the B value.
   */
  const double& b() const { return data_[1]; }

  /**
   * @brief Read-only access to the stamp value.
   */
  const std::string& quest() const { return quest_; }

  /**
   * @brief Read-only access to the stamp value.
   */
  const ros::Time& stamp() const { return stamp_; }

  /** ----- Variable Interface ----**/

  /**
   * @brief Read-only access to the variable data
   */
  const double* data() const override { return data_.data(); }

  /**
   * @brief Read-write access to the variable data
   */
  double* data() override { return data_.data(); }

  /**
   * @brief Print a human-readable description of the variable to the provided stream.
   *
   * @param[out] stream The stream to write to. Defaults to stdout.
   */
  void print(std::ostream& stream = std::cout) const override;

  /**
   * @brief Returns the number of elements of this variable.
   *
   * The number of scalar values contained by this variable type is defined by the class template parameter \p N.
   */
  size_t size() const override { return 2; }

  /**
   * @brief Serialize the Dummy Variable members
   */
  template<class Archive>
  void serialize(Archive& archive)
  {
    archive(cereal::make_nvp("base", cereal::base_class<fuse_core::Variable>(this)),
            CEREAL_NVP(data_),
            CEREAL_NVP(quest_),
            CEREAL_NVP(stamp_));
  }

  void serializeVariable(cereal::JSONOutputArchive& archive) const override
  {
    archive(cereal::make_nvp("variable", *this));
  }

  void deserializeVariable(cereal::JSONInputArchive& archive) override
  {
    archive(cereal::make_nvp("variable", *this));
  }

private:
  std::array<double, 2> data_;
  std::string quest_;
  ros::Time stamp_;
};

}  // namespace fuse_variables

// Register the derived fuse Variable with Cereal.
CEREAL_REGISTER_TYPE(fuse_variables::DummyVariable);

#endif  // FUSE_VARIABLES_DUMMY_VARIABLE_H
