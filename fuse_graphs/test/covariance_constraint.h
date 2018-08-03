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
#ifndef FUSE_GRAPHS_TEST_COVARIANCE_CONSTRAINT_H  // NOLINT{build/header_guard}
#define FUSE_GRAPHS_TEST_COVARIANCE_CONSTRAINT_H  // NOLINT{build/header_guard}

#include <fuse_core/constraint.h>
#include <fuse_core/macros.h>
#include <fuse_core/uuid.h>

#include <ceres/cost_function.h>

#include <algorithm>


/**
 * @brief Create a cost fuction that implements one of the Ceres unit tests
 * 
 * UnaryCostFunctions are added to all three variables
 * - https://github.com/ceres-solver/ceres-solver/blob/master/internal/ceres/covariance_test.cc#L423
 * - https://github.com/ceres-solver/ceres-solver/blob/master/internal/ceres/covariance_test.cc#L428
 * - https://github.com/ceres-solver/ceres-solver/blob/master/internal/ceres/covariance_test.cc#L433
 * BinaryCostFunctions are added for (Y,X) and (Z,X)
 * - https://github.com/ceres-solver/ceres-solver/blob/master/internal/ceres/covariance_test.cc#L441
 * - https://github.com/ceres-solver/ceres-solver/blob/master/internal/ceres/covariance_test.cc#L451
 */
class CovarianceCostFunction : public ceres::CostFunction
{
public:
  CovarianceCostFunction()
  {
    set_num_residuals(8);
    mutable_parameter_block_sizes()->push_back(2);
    mutable_parameter_block_sizes()->push_back(3);
    mutable_parameter_block_sizes()->push_back(1);
  }

  bool Evaluate(double const* const* parameters,
                double* residuals,
                double** jacobians) const override
  {
    residuals[0] = 1;
    residuals[1] = 1;
    residuals[2] = 1;
    residuals[3] = 1;
    residuals[4] = 1;
    residuals[5] = 1;
    residuals[6] = 2;
    residuals[7] = 2;

    if (jacobians != NULL)
    {
      if (jacobians[0] != NULL)
      {
        static const double jacobian0[] =
        {
            1.0,  0.0,
            0.0,  1.0,
            0.0,  0.0,
            0.0,  0.0,
            0.0,  0.0,
            0.0,  0.0,
          -5.0, -6.0,
            3.0, -2.0
        };
        std::copy(jacobian0, jacobian0 + 16, jacobians[0]);
      }
      if (jacobians[1] != NULL)
      {
        static const double jacobian1[] =
        {
            0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,
            2.0,  0.0,  0.0,
            0.0,  2.0,  0.0,
            0.0,  0.0,  2.0,
            0.0,  0.0,  0.0,
            1.0,  2.0,  3.0,
            0.0,  0.0,  0.0
        };
        std::copy(jacobian1, jacobian1 + 24, jacobians[1]);
      }
      if (jacobians[2] != NULL)
      {
        static const double jacobian2[] =
        {
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            5.0,
            0.0,
            2.0
        };
        std::copy(jacobian2, jacobian2 + 8, jacobians[2]);
      }
    }

    return true;
  }
};

/**
 * @brief Constraint implementing the covariance cost function
 */
class CovarianceConstraint : public fuse_core::Constraint
{
public:
  SMART_PTR_DEFINITIONS(CovarianceConstraint);

  CovarianceConstraint(
    const fuse_core::UUID& variable1_uuid,
    const fuse_core::UUID& variable2_uuid,
    const fuse_core::UUID& variable3_uuid) :
    fuse_core::Constraint{variable1_uuid, variable2_uuid, variable3_uuid}
  {
  }

  void print(std::ostream& stream = std::cout) const override {}
  fuse_core::Constraint::UniquePtr clone() const override { return CovarianceConstraint::make_unique(*this); }
  ceres::CostFunction* costFunction() const override { return new CovarianceCostFunction(); }
};

#endif  // FUSE_GRAPHS_TEST_COVARIANCE_CONSTRAINT_H  // NOLINT{build/header_guard}
