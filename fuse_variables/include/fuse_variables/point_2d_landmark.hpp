/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Locus Robotics
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
#ifndef FUSE_VARIABLES__POINT_2D_LANDMARK_HPP_
#define FUSE_VARIABLES__POINT_2D_LANDMARK_HPP_

#include <ostream>

#include <fuse_core/fuse_macros.hpp>
#include <fuse_core/serialization.hpp>
#include <fuse_variables/fixed_size_variable.hpp>

#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>


namespace fuse_variables
{
/**
 * @brief Variable representing a 2D point landmark that exists across time.
 *
 * This is commonly used to represent locations of visual features. The UUID of this class is
 * constant after construction and dependent on a user input database id. As such, the database id
 * cannot be altered after construction.
 */
class Point2DLandmark : public FixedSizeVariable<2>
{
public:
  FUSE_VARIABLE_DEFINITIONS(Point2DLandmark)

  /**
   * @brief Can be used to directly index variables in the data array
   */
  enum : size_t
  {
    X = 0,
    Y = 1
  };

  /**
   * @brief Default constructor
   */
  Point2DLandmark() = default;

  /**
   * @brief Construct a point 2D variable given a landmarks id
   *
   * @param[in] landmark_id  The id associated to a landmark
   */
  explicit Point2DLandmark(const uint64_t & landmark_id);

  /**
   * @brief Read-write access to the X-axis position.
   */
  double & x() {return data_[X];}

  /**
   * @brief Read-only access to the X-axis position.
   */
  const double & x() const {return data_[X];}

  /**
   * @brief Read-write access to the Y-axis position.
   */
  double & y() {return data_[Y];}

  /**
   * @brief Read-only access to the Y-axis position.
   */
  const double & y() const {return data_[Y];}

  /**
   * @brief Read-only access to the id
   */
  const uint64_t & id() const {return id_;}

  /**
   * @brief Print a human-readable description of the variable to the provided stream.
   *
   * @param[out] stream The stream to write to. Defaults to stdout.
   */
  void print(std::ostream & stream = std::cout) const override;

private:
  // Allow Boost Serialization access to private methods
  friend class boost::serialization::access;
  uint64_t id_ {0};

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
    archive & boost::serialization::base_object<FixedSizeVariable<SIZE>>(*this);
    archive & id_;
  }
};

}  // namespace fuse_variables

BOOST_CLASS_EXPORT_KEY(fuse_variables::Point2DLandmark);

#endif  // FUSE_VARIABLES__POINT_2D_LANDMARK_HPP_
