/*
 * Software License Agreement (BSD License)
 *
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
#include <ceres/autodiff_cost_function.h>
#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <array>
#include <string>

#include "cost_function_gtest.hpp"
#include <fuse_constraints/normal_prior_pose_3d_euler.hpp>
#include <fuse_constraints/normal_prior_pose_3d_euler_cost_functor.hpp>
#include <fuse_core/eigen_gtest.hpp>

/**
 * @brief Test fixture that initializes a full pose 3d mean and sqrt information matrix.
 */
class NormalPriorPose3DEulerTestFixture : public ::testing::Test
{
public:
  //!< The automatic differentiation cost function type for the pose 3d cost functor
  using AutoDiffNormalPriorPose3DEuler =
    ceres::AutoDiffCostFunction<fuse_constraints::NormalPriorPose3DEulerCostFunctor, ceres::DYNAMIC, 3,
      4>;

  /**
   * @brief Constructor
   */
  NormalPriorPose3DEulerTestFixture()
  {
    full_sqrt_information = covariance.inverse().llt().matrixU();
  }

  const fuse_core::Matrix6d covariance =
    fuse_core::Vector6d(1e-3, 1e-3, 1e-3, 
                        1e-3, 1e-3, 1e-3).asDiagonal(); //!< The full pose 3d covariance for the x,
                                                         //!< y, z, roll, pitch and yaw components
  Eigen::Matrix<double, 6, 6> full_sqrt_information;  //!< The full pose 3d sqrt information matrix for the x, y
                                          //!< z, roll, pitch, and yaw components
  Eigen::Vector<double, 6> full_mean{1.0, 2.0, 1.0, 0.0, 0.0, 0.0}; //!< The full pose 3d mean 
                                                                      //!< components: x, y z, 
                                                                      //!< roll, pitch, and yaw
};

TEST_F(NormalPriorPose3DEulerTestFixture, AnalyticAndAutoDiffCostFunctionsAreEqualForFullResiduals)
{
  // Create cost function
  auto rpy = Eigen::Vector3d::Random();
  full_mean << 1.0, 2.0, 1.0, rpy.x(), rpy.y(), rpy.z();  
  const fuse_constraints::NormalPriorPose3DEuler cost_function{full_sqrt_information, full_mean};
  const auto num_residuals = full_sqrt_information.rows();

  AutoDiffNormalPriorPose3DEuler autodiff_cost_function(
    new fuse_constraints::NormalPriorPose3DEulerCostFunctor(full_sqrt_information, full_mean),
    num_residuals);
  
  // Compare the expected, automatic differentiation, cost function and the actual one
  // N.B. in ExpectCostFunctionsAreEqual constructor, the first argument is the expected cost function
  // and the second argument is the actual cost function 
  ExpectCostFunctionsAreEqual(cost_function, autodiff_cost_function, 1e-12);
}

TEST_F(NormalPriorPose3DEulerTestFixture, AnalyticAndAutoDiffCostFunctionsAreEqualForPartialPositionResiduals)
{
  // Create cost function for a subset of residuals
  // Version with y position = 0
  std::vector<int> indices = {0, 2, 3, 4, 5};
  auto rpy = Eigen::Vector3d::Random();
  full_mean << 1.0, 0.0, 1.0, rpy.x(), rpy.y(), rpy.z();
  fuse_core::Matrix<double, 5, 6> partial_sqrt_information;
  for (size_t i = 0; i < indices.size(); ++i) {
    partial_sqrt_information.row(i) = full_sqrt_information.row(indices[i]);
  }

  std::cout << "full_mean: " << full_mean << std::endl;
  std::cout << "partial_sqrt_information: " << partial_sqrt_information << std::endl;

  const fuse_constraints::NormalPriorPose3DEuler cost_function{partial_sqrt_information, full_mean};

  // Create automatic differentiation cost function
  const auto num_residuals = partial_sqrt_information.rows();

  AutoDiffNormalPriorPose3DEuler autodiff_cost_function(
    new fuse_constraints::NormalPriorPose3DEulerCostFunctor(partial_sqrt_information, full_mean),
    num_residuals);

  ExpectCostFunctionsAreEqual(cost_function, autodiff_cost_function, 1e-12);
}

TEST_F(NormalPriorPose3DEulerTestFixture, AnalyticAndAutoDiffCostFunctionsAreEqualForPartialOrientationResiduals)
{
  // Create cost function for a subset of residuals
  // Version with roll, pitch = 0
  std::vector<int> indices = {0, 1, 2, 5};
  Eigen::Vector3d rpy = Eigen::Vector3d::Random();
  rpy(0) = 0.0;
  rpy(1) = 0.0;
  full_mean << 1.0, 0.0, 1.0, rpy.x(), rpy.y(), rpy.z();
  fuse_core::Matrix<double, 4, 6> partial_sqrt_information;

  for (size_t i = 0; i < indices.size(); ++i) {
    partial_sqrt_information.row(i) = full_sqrt_information.row(indices[i]);
  }

  const fuse_constraints::NormalPriorPose3DEuler cost_function{partial_sqrt_information, full_mean};

  // Create automatic differentiation cost function
  const auto num_residuals = partial_sqrt_information.rows();

  AutoDiffNormalPriorPose3DEuler autodiff_cost_function(
    new fuse_constraints::NormalPriorPose3DEulerCostFunctor(partial_sqrt_information, full_mean),
    num_residuals);

  ExpectCostFunctionsAreEqual(cost_function, autodiff_cost_function, 1e-12);
}

TEST_F(NormalPriorPose3DEulerTestFixture, AnalyticAndAutoDiffCostFunctionsAreEqualForPartialResidualsPositionOnly)
{
  // Create cost function for a subset of residuals
  // Version with z = 0, orientation = 0
  std::vector<int> indices = {0, 1};
  Eigen::Vector3d rpy {0.0, 0.0, 0.0};
  full_mean << 0.1, 0.5, 0.0, rpy.x(), rpy.y(), rpy.z();
  fuse_core::Matrix<double, 2, 6> partial_sqrt_information;

  for (size_t i = 0; i < indices.size(); ++i) {
    partial_sqrt_information.row(i) = full_sqrt_information.row(indices[i]);
  }

  const fuse_constraints::NormalPriorPose3DEuler cost_function{partial_sqrt_information, full_mean};

  // Create automatic differentiation cost function
  const auto num_residuals = partial_sqrt_information.rows();

  AutoDiffNormalPriorPose3DEuler autodiff_cost_function(
    new fuse_constraints::NormalPriorPose3DEulerCostFunctor(partial_sqrt_information, full_mean),
    num_residuals);

  ExpectCostFunctionsAreEqual(cost_function, autodiff_cost_function, 1e-12);
}

TEST_F(NormalPriorPose3DEulerTestFixture, AnalyticAndAutoDiffCostFunctionsAreEqualForPartialResidualsOrientationOnly)
{
  // Create cost function for a subset of residuals
  // Version with position = 0, roll = 0
  std::vector<int> indices = {4, 5};
  Eigen::Vector3d rpy = Eigen::Vector3d::Random();
  rpy(0) = 0.0;
  full_mean << 0.0, 0.0, 0.0, rpy.x(), rpy.y(), rpy.z();
  fuse_core::Matrix<double, 2, 6> partial_sqrt_information;

  for (size_t i = 0; i < indices.size(); ++i) {
    partial_sqrt_information.row(i) = full_sqrt_information.row(indices[i]);
  }

  const fuse_constraints::NormalPriorPose3DEuler cost_function{partial_sqrt_information, full_mean};

  // Create automatic differentiation cost function
  const auto num_residuals = partial_sqrt_information.rows();

  AutoDiffNormalPriorPose3DEuler autodiff_cost_function(
    new fuse_constraints::NormalPriorPose3DEulerCostFunctor(partial_sqrt_information, full_mean),
    num_residuals);

  ExpectCostFunctionsAreEqual(cost_function, autodiff_cost_function, 1e-12);
}
