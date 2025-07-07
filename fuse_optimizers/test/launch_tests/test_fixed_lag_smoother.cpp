/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2025, Locus Robotics
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
#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "common.hpp"
#include <fuse_optimizers/fixed_lag_smoother.hpp>
#include <fuse_graphs/hash_graph.hpp>

#include <fuse_graphs/hash_graph_params.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>  // NOLINT


class FixedLagSmootherForTest : public fuse_optimizers::FixedLagSmoother
{
public:
  FixedLagSmootherForTest(
    fuse_core::node_interfaces::NodeInterfaces<ALL_FUSE_CORE_NODE_INTERFACES> interfaces,
    fuse_core::Graph::UniquePtr graph = nullptr)
  : fuse_optimizers::FixedLagSmoother(interfaces, std::move(graph)) {}

  using fuse_optimizers::FixedLagSmoother::terminationTypeToDiagnosticStatus;
};

class TestFixedLagSmoother : public ::testing::Test
{
protected:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<FixedLagSmootherForTest> smoother_;

  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("fixed_lag_smoother_test");
    smoother_ = std::make_shared<FixedLagSmootherForTest>(*node_);
  }

  void TearDown() override
  {
    smoother_.reset();
    node_.reset();
  }
};

TEST_F(TestFixedLagSmoother, terminationTypeToDiagnosticStatus)
{
  ceres::TerminationType termination_type;
  std::vector<std::string> diagnostic_warning_status {"NO_CONVERGENCE"};
  std::vector<std::string> diagnostic_error_status {"FAILURE", "USER_FAILURE"};
  diagnostic_msgs::msg::DiagnosticStatus diag_msg;

  // NO_CONVERGENCE as WARNING
  termination_type = ceres::TerminationType::NO_CONVERGENCE;
  diag_msg = smoother_->terminationTypeToDiagnosticStatus(
    termination_type, diagnostic_warning_status, diagnostic_error_status);
  EXPECT_EQ(diag_msg.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);

  // NO_CONVERGENCE as OK
  diagnostic_warning_status = {};
  diag_msg = smoother_->terminationTypeToDiagnosticStatus(
    termination_type, diagnostic_warning_status, diagnostic_error_status);
  EXPECT_EQ(diag_msg.level, diagnostic_msgs::msg::DiagnosticStatus::OK);

  // CONVERGENCE
  termination_type = ceres::TerminationType::CONVERGENCE;
  diag_msg = smoother_->terminationTypeToDiagnosticStatus(
    termination_type, diagnostic_warning_status, diagnostic_error_status);
  EXPECT_EQ(diag_msg.level, diagnostic_msgs::msg::DiagnosticStatus::OK);

  // USER_SUCCESS
  termination_type = ceres::TerminationType::USER_SUCCESS;
  diag_msg = smoother_->terminationTypeToDiagnosticStatus(
    termination_type, diagnostic_warning_status, diagnostic_error_status);
  EXPECT_EQ(diag_msg.level, diagnostic_msgs::msg::DiagnosticStatus::OK);

  // FAILURE
  termination_type = ceres::TerminationType::FAILURE;
  diag_msg = smoother_->terminationTypeToDiagnosticStatus(
    termination_type, diagnostic_warning_status, diagnostic_error_status);
  EXPECT_EQ(diag_msg.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);

  // USER_FAILURE
  termination_type = ceres::TerminationType::USER_FAILURE;
  diag_msg = smoother_->terminationTypeToDiagnosticStatus(
    termination_type, diagnostic_warning_status, diagnostic_error_status);
  EXPECT_EQ(diag_msg.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
