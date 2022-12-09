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
#include <ceres/autodiff_cost_function.h>
#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <array>
#include <string>

#include "cost_function_gtest.hpp"
#include <fuse_constraints/normal_delta_pose_2d.hpp>
#include <fuse_constraints/normal_delta_pose_2d_cost_functor.hpp>
#include <fuse_core/eigen_gtest.hpp>

/**
 * @brief Test fixture that initializes a full pose 2d delta and sqrt information matrix.
 */
class NormalDeltaPose2DTestFixture : public ::testing::Test
{
public:
  //!< The automatic differentiation cost function type for the pose 2d cost functor
  using AutoDiffNormalDeltaPose2D =
    ceres::AutoDiffCostFunction<fuse_constraints::NormalDeltaPose2DCostFunctor, ceres::DYNAMIC, 2,
      1, 2, 1>;

  /**
   * @brief Constructor
   */
  NormalDeltaPose2DTestFixture()
  {
    full_sqrt_information = covariance.inverse().llt().matrixU();
  }

  const fuse_core::Matrix3d covariance =
    fuse_core::Vector3d(2e-3, 1e-3, 1e-2).asDiagonal();  //!< The full pose 2d covariance for the x,
                                                         //!< y and yaw components
  Eigen::Matrix3d full_sqrt_information;  //!< The full pose 2d sqrt information matrix for the x, y
                                          //!< and yaw components
  const Eigen::Vector3d full_delta{1.0, 2.0, 3.0};  //!< The full pose 2d delta components: x, y and
                                                    //!< yaw
};

TEST_F(NormalDeltaPose2DTestFixture, AnalyticAndAutoDiffCostFunctionsAreEqualForFullResiduals)
{
  // Create cost function
  const fuse_constraints::NormalDeltaPose2D cost_function{full_sqrt_information, full_delta};

  // Create automatic differentiation cost function
  const auto num_residuals = full_sqrt_information.rows();

  AutoDiffNormalDeltaPose2D autodiff_cost_function(
    new fuse_constraints::NormalDeltaPose2DCostFunctor(full_sqrt_information, full_delta),
    num_residuals);

  // Compare the expected, automatic differentiation, cost function and the actual one
  ExpectCostFunctionsAreEqual(autodiff_cost_function, cost_function);
}

TEST_F(NormalDeltaPose2DTestFixture, AnalyticAndAutoDiffCostFunctionsAreEqualForTwoResiduals)
{
  // Create cost function for each possible pair of two residuals, the ones in each possible pair of
  // rows
  using IndicesPair = std::array<int, 2>;
  std::array<IndicesPair,
    3> indices_pairs = {IndicesPair{0, 1}, IndicesPair{0, 2}, IndicesPair{1, 2}};
  for (const auto & indices_pair : indices_pairs) {
    // It is a shame we need Eigen 3.4+ in order to use the slicing and indexing API documented in:
    //
    //   https://eigen.tuxfamily.org/dox-devel/group__TutorialSlicingIndexing.html
    //
    // That would allow us to simply do:
    //
    //   const fuse_core::Matrix<double, 2, 3> partial_sqrt_information =
    //       full_sqrt_information(indices_pair, Eigen::all);
    fuse_core::Matrix<double, 2, 3> partial_sqrt_information;
    for (size_t i = 0; i < indices_pair.size(); ++i) {
      partial_sqrt_information.row(i) = full_sqrt_information.row(indices_pair[i]);
    }

    const fuse_constraints::NormalDeltaPose2D cost_function{partial_sqrt_information, full_delta};

    // Create automatic differentiation cost function
    const auto num_residuals = partial_sqrt_information.rows();

    AutoDiffNormalDeltaPose2D autodiff_cost_function(
      new fuse_constraints::NormalDeltaPose2DCostFunctor(partial_sqrt_information, full_delta),
      num_residuals);

    ExpectCostFunctionsAreEqual(autodiff_cost_function, cost_function);
  }
}

TEST_F(NormalDeltaPose2DTestFixture, AnalyticAndAutoDiffCostFunctionsAreEqualForOneResidual)
{
  // Create cost function for one residual, the one in each row
  for (size_t i = 0; i < 3; ++i) {
    SCOPED_TRACE("Residual " + std::to_string(i));

    const fuse_core::Matrix<double, 1, 3> partial_sqrt_information = full_sqrt_information.row(i);

    const fuse_constraints::NormalDeltaPose2D cost_function{partial_sqrt_information, full_delta};

    // Create automatic differentiation cost function
    const auto num_residuals = partial_sqrt_information.rows();

    AutoDiffNormalDeltaPose2D autodiff_cost_function(
      new fuse_constraints::NormalDeltaPose2DCostFunctor(partial_sqrt_information, full_delta),
      num_residuals);

    ExpectCostFunctionsAreEqual(autodiff_cost_function, cost_function);
  }
}
