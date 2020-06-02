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
#include <fuse_constraints/normal_prior_pose_2d.h>
#include <fuse_constraints/normal_prior_pose_2d_cost_functor.h>

#include <gtest/gtest.h>
#include <fuse_core/eigen_gtest.h>

#include <ceres/autodiff_cost_function.h>
#include <ceres/cost_function_to_functor.h>
#include <Eigen/Dense>

#include <memory>
#include <vector>

/**
 * @brief A helper function to compare a expected and actual cost function.
 *
 * This helper function is copied and slightly adapted from:
 *
 *   https://github.com/ceres-solver/ceres-solver/blob/27b71795/internal/ceres/cost_function_to_functor_test.cc#L46-L119
 *
 * @param[in] cost_function The expected cost function
 * @param[in] actual_cost_function The actual cost function
 * @param[in] tolerance The tolerance to use when comparing the cost functions are equal. Defaults to 1e-18
 */
static void ExpectCostFunctionsAreEqual(const ceres::CostFunction& cost_function,
                                        const ceres::CostFunction& actual_cost_function, double tolerance = 1e-18)
{
  EXPECT_EQ(cost_function.num_residuals(), actual_cost_function.num_residuals());
  const size_t num_residuals = cost_function.num_residuals();
  const std::vector<int32_t>& parameter_block_sizes = cost_function.parameter_block_sizes();
  const std::vector<int32_t>& actual_parameter_block_sizes = actual_cost_function.parameter_block_sizes();
  EXPECT_EQ(parameter_block_sizes.size(), actual_parameter_block_sizes.size());

  size_t num_parameters = 0;
  for (size_t i = 0; i < parameter_block_sizes.size(); ++i)
  {
    EXPECT_EQ(parameter_block_sizes[i], actual_parameter_block_sizes[i]);
    num_parameters += parameter_block_sizes[i];
  }

  std::unique_ptr<double[]> parameters(new double[num_parameters]);
  for (size_t i = 0; i < num_parameters; ++i)
  {
    parameters[i] = static_cast<double>(i) + 1.0;
  }

  std::unique_ptr<double[]> residuals(new double[num_residuals]);
  std::unique_ptr<double[]> jacobians(new double[num_parameters * num_residuals]);

  std::unique_ptr<double[]> actual_residuals(new double[num_residuals]);
  std::unique_ptr<double[]> actual_jacobians(new double[num_parameters * num_residuals]);

  std::unique_ptr<double* []> parameter_blocks(new double*[parameter_block_sizes.size()]);
  std::unique_ptr<double* []> jacobian_blocks(new double*[parameter_block_sizes.size()]);
  std::unique_ptr<double* []> actual_jacobian_blocks(new double*[parameter_block_sizes.size()]);

  num_parameters = 0;
  for (size_t i = 0; i < parameter_block_sizes.size(); ++i)
  {
    parameter_blocks[i] = parameters.get() + num_parameters;
    jacobian_blocks[i] = jacobians.get() + num_parameters * num_residuals;
    actual_jacobian_blocks[i] = actual_jacobians.get() + num_parameters * num_residuals;
    num_parameters += parameter_block_sizes[i];
  }

  EXPECT_TRUE(cost_function.Evaluate(parameter_blocks.get(), residuals.get(), NULL));
  EXPECT_TRUE(actual_cost_function.Evaluate(parameter_blocks.get(), actual_residuals.get(), NULL));
  for (size_t i = 0; i < num_residuals; ++i)
  {
    EXPECT_NEAR(residuals[i], actual_residuals[i], tolerance) << "residual id: " << i;
  }

  EXPECT_TRUE(cost_function.Evaluate(parameter_blocks.get(), residuals.get(), jacobian_blocks.get()));
  EXPECT_TRUE(
      actual_cost_function.Evaluate(parameter_blocks.get(), actual_residuals.get(), actual_jacobian_blocks.get()));
  for (size_t i = 0; i < num_residuals; ++i)
  {
    EXPECT_NEAR(residuals[i], actual_residuals[i], tolerance) << "residual : " << i;
  }

  for (size_t i = 0; i < num_residuals * num_parameters; ++i)
  {
    EXPECT_NEAR(jacobians[i], actual_jacobians[i], tolerance)
        << "jacobian : " << i << " " << jacobians[i] << " " << actual_jacobians[i];
  }
}

/**
 * @brief Test fixture that initializes a full pose 2d mean and sqrt information matrix.
 */
class NormalPriorPose2DTestFixture : public ::testing::Test
{
public:
  using AutoDiffNormalPriorPose2D =
      ceres::AutoDiffCostFunction<fuse_constraints::NormalPriorPose2DCostFunctor, ceres::DYNAMIC, 2,
                                  1>;  //!< The automatic differentiation cost function type for the pose 2d cost
                                       //!< functor

  /**
   * @brief Constructor
   */
  NormalPriorPose2DTestFixture()
  {
    full_sqrt_information = covariance.inverse().llt().matrixU();
  }

  const fuse_core::Matrix3d covariance =
      fuse_core::Vector3d(2e-3, 1e-3, 1e-2).asDiagonal();  //!< The full pose 2d covariance for the x, y and yaw
                                                           //!< components
  Eigen::Matrix3d full_sqrt_information;  //!< The full pose 2d sqrt information matrix for the x, y and yaw components
  const Eigen::Vector3d full_mean{ 1.0, 2.0, 3.0 };  //!< The full pose 2d mean components: x, y and yaw
};

TEST_F(NormalPriorPose2DTestFixture, AnalyticAndAutoDiffCostFunctionsAreEqualForFullResiduals)
{
  // Create cost function
  const fuse_constraints::NormalPriorPose2D cost_function{ full_sqrt_information, full_mean };

  // Create automatic differentiation cost function
  const auto num_residuals = full_sqrt_information.rows();

  AutoDiffNormalPriorPose2D autodiff_cost_function(
      new fuse_constraints::NormalPriorPose2DCostFunctor(full_sqrt_information, full_mean), num_residuals);

  // Compare the expected, automatic differentiation, cost function and the actual one
  ExpectCostFunctionsAreEqual(autodiff_cost_function, cost_function);
}

TEST_F(NormalPriorPose2DTestFixture, AnalyticAndAutoDiffCostFunctionsAreEqualForTwoResiduals)
{
  // Create cost function for each possible pair of two residuals, the ones in each possible pair of rows
  using IndicesPair = std::array<int, 2>;
  std::array<IndicesPair, 3> indices_pairs = { IndicesPair{ 0, 1 }, IndicesPair{ 0, 2 }, IndicesPair{ 1, 2 } };
  for (const auto& indices_pair : indices_pairs)
  {
    // It is a shame we need Eigen 3.4+ in order to use the slicing and indexing API documented in:
    //
    //   https://eigen.tuxfamily.org/dox-devel/group__TutorialSlicingIndexing.html
    //
    // That would allow us to simply do:
    //
    //   const fuse_core::Matrix<double, 2, 3> partial_sqrt_information =
    //       full_sqrt_information(indices_pair, Eigen::all);
    fuse_core::Matrix<double, 2, 3> partial_sqrt_information;
    for (size_t i = 0; i < indices_pair.size(); ++i)
    {
      partial_sqrt_information.row(i) = full_sqrt_information.row(indices_pair[i]);
    }

    const fuse_constraints::NormalPriorPose2D cost_function{ partial_sqrt_information, full_mean };

    // Create automatic differentiation cost function
    const auto num_residuals = partial_sqrt_information.rows();

    AutoDiffNormalPriorPose2D autodiff_cost_function(
        new fuse_constraints::NormalPriorPose2DCostFunctor(partial_sqrt_information, full_mean), num_residuals);

    ExpectCostFunctionsAreEqual(autodiff_cost_function, cost_function);
  }
}

TEST_F(NormalPriorPose2DTestFixture, AnalyticAndAutoDiffCostFunctionsAreEqualForOneResidual)
{
  // Create cost function for one residual, the one in each row
  for (size_t i = 0; i < 3; ++i)
  {
    SCOPED_TRACE("Residual " + std::to_string(i));

    const fuse_core::Matrix<double, 1, 3> partial_sqrt_information = full_sqrt_information.row(i);

    const fuse_constraints::NormalPriorPose2D cost_function{ partial_sqrt_information, full_mean };

    // Create automatic differentiation cost function
    const auto num_residuals = partial_sqrt_information.rows();

    AutoDiffNormalPriorPose2D autodiff_cost_function(
        new fuse_constraints::NormalPriorPose2DCostFunctor(partial_sqrt_information, full_mean), num_residuals);

    ExpectCostFunctionsAreEqual(autodiff_cost_function, cost_function);
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
