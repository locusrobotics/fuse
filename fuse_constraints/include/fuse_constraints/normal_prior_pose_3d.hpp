/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Locus Robotics
 *  Copyright (c) 2023, Giacomo Franchini
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
#ifndef FUSE_CONSTRAINTS__NORMAL_PRIOR_POSE_3D_HPP_
#define FUSE_CONSTRAINTS__NORMAL_PRIOR_POSE_3D_HPP_

#include <ceres/sized_cost_function.h>

#include <fuse_core/eigen.hpp>


namespace fuse_constraints
{

/**
 * @brief Create a prior cost function on both the position and orientation variables at once.
 *
 * The Ceres::NormalPrior cost function only supports a single variable. This is a convenience cost
 * function that applies a prior constraint on both the position and orientation variables at once.
 *
 * The cost function is of the form:
 *
 *             ||                [  x - b(0)]             ||^2
 *   cost(x) = ||  A *           [  y - b(1)]             ||
 *             ||                [  z - b(2)]             ||
 *             ||     [  quat2angleaxis(b(3-6)^-1 * q)]   ||
 * 
 * In case the user is interested in implementing a cost function of the form:
 *
 *   cost(X) = (X - mu)^T S^{-1} (X - mu)
 *
 * where, mu is a vector and S is a covariance matrix, then, A = S^{-1/2}, i.e the matrix A is the
 * square root information matrix (the inverse of the covariance).
 */
class NormalPriorPose3D : public ceres::SizedCostFunction<6, 3, 4>
{
public:
  /**
   * @brief Construct a cost function instance
   *
   * @param[in] A The residual weighting matrix, most likely the square root information matrix in
   *              order (x, y, z, roll, pitch, yaw)
   * @param[in] b The pose measurement or prior in order (x, y, z, qw, qx, qy, qz)
   */
  NormalPriorPose3D(const fuse_core::Matrix6d & A, const fuse_core::Vector7d & b);

  /**
   * @brief Destructor
   */
  virtual ~NormalPriorPose3D() = default;

  /**
   * @brief Compute the cost values/residuals, and optionally the Jacobians, using the provided
   *        variable/parameter values
   */
  virtual bool Evaluate(
    double const * const * parameters,
    double * residuals,
    double ** jacobians) const;

private:
  fuse_core::Matrix6d A_;  //!< The residual weighting matrix, most likely the square root
                           //!< information matrix
  fuse_core::Vector7d b_;  //!< The measured 3D pose value
};

}  // namespace fuse_constraints

#endif  // FUSE_CONSTRAINTS__NORMAL_PRIOR_POSE_3D_HPP_
