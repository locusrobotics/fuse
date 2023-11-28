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
#ifndef FUSE_MODELS__COMMON__VARIABLE_TRAITS_HPP_
#define FUSE_MODELS__COMMON__VARIABLE_TRAITS_HPP_

#include <fuse_variables/acceleration_linear_2d_stamped.hpp>
#include <fuse_variables/acceleration_linear_3d_stamped.hpp>
#include <fuse_variables/orientation_2d_stamped.hpp>
#include <fuse_variables/orientation_3d_stamped.hpp>
#include <fuse_variables/position_2d_stamped.hpp>
#include <fuse_variables/position_3d_stamped.hpp>
#include <fuse_variables/velocity_angular_2d_stamped.hpp>
#include <fuse_variables/velocity_angular_3d_stamped.hpp>
#include <fuse_variables/velocity_linear_2d_stamped.hpp>
#include <fuse_variables/velocity_linear_3d_stamped.hpp>


namespace fuse_models
{

namespace common
{

template<typename T>
struct is_linear_2d
{
  static const bool value = false;
};

template<>
struct is_linear_2d<fuse_variables::AccelerationLinear2DStamped>
{
  static const bool value = true;
};

template<>
struct is_linear_2d<fuse_variables::VelocityLinear2DStamped>
{
  static const bool value = true;
};

template<>
struct is_linear_2d<fuse_variables::Position2DStamped>
{
  static const bool value = true;
};

template<typename T>
struct is_linear_3d
{
  static const bool value = false;
};

template<>
struct is_linear_3d<fuse_variables::AccelerationLinear3DStamped>
{
  static const bool value = true;
};

template<>
struct is_linear_3d<fuse_variables::VelocityLinear3DStamped>
{
  static const bool value = true;
};

template<>
struct is_linear_3d<fuse_variables::Position3DStamped>
{
  static const bool value = true;
};

template<typename T>
struct is_angular_2d
{
  static const bool value = false;
};

template<>
struct is_angular_2d<fuse_variables::Orientation2DStamped>
{
  static const bool value = true;
};

template<>
struct is_angular_2d<fuse_variables::VelocityAngular2DStamped>
{
  static const bool value = true;
};

template<typename T>
struct is_angular_3d
{
  static const bool value = false;
};
template<>
struct is_angular_3d<fuse_variables::Orientation3DStamped>
{
  static const bool value = true;
};
template<>
struct is_angular_3d<fuse_variables::VelocityAngular3DStamped>
{
  static const bool value = true;
};

template<typename T>
struct is_orientation
{
  static const bool value = false;
};
template<>
struct is_orientation<fuse_variables::Orientation2DStamped>
{
  static const bool value = true;
};
template<>
struct is_orientation<fuse_variables::Orientation3DStamped>
{
  static const bool value = true;
};
}  // namespace common

}  // namespace fuse_models

#endif  // FUSE_MODELS__COMMON__VARIABLE_TRAITS_HPP_
