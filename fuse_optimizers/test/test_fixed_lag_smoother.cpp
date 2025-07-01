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

#include <fuse_graphs/hash_graph.h>
#include <fuse_graphs/hash_graph_params.h>
#include <fuse_optimizers/fixed_lag_smoother.h>

#include <gtest/gtest.h>
#include <ros/ros.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>


class FixedLagSmootherForTest : public fuse_optimizers::FixedLagSmoother
{
public:
  FixedLagSmootherForTest(fuse_core::Graph::UniquePtr graph,
                          const ros::NodeHandle& nh,
                          const ros::NodeHandle& pnh)
    : fuse_optimizers::FixedLagSmoother(std::move(graph), nh, pnh) {}

  using fuse_optimizers::FixedLagSmoother::terminationTypeToDiagnosticStatus;
};

class TestFixedLagSmoother : public ::testing::Test
{
protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::unique_ptr<FixedLagSmootherForTest> smoother_;

  void SetUp() override
  {
    fuse_graphs::HashGraphParams graph_params;
    graph_params.loadFromROS(pnh_);
    smoother_ = std::make_unique<FixedLagSmootherForTest>(
        fuse_graphs::HashGraph::make_unique(graph_params),
        nh_, pnh_);
  }

  void TearDown() override
  {
    smoother_.reset();
  }
};

TEST_F(TestFixedLagSmoother, terminationTypeToDiagnosticStatus)
{
  ceres::TerminationType termination_type;
  std::vector<std::string> diagnostic_warning_status { "NO_CONVERGENCE" };
  std::vector<std::string> diagnostic_error_status {"FAILURE", "USER_FAILURE"};
  diagnostic_msgs::DiagnosticStatus diag_msg;

  // NO_CONVERGENCE as WARNING
  termination_type = ceres::TerminationType::NO_CONVERGENCE;
  diag_msg = smoother_->terminationTypeToDiagnosticStatus(termination_type, diagnostic_warning_status,
                                                          diagnostic_error_status);
  EXPECT_EQ(diag_msg.level, diagnostic_msgs::DiagnosticStatus::WARN);

  // NO_CONVERGENCE as OK
  diagnostic_warning_status = {};
  diag_msg = smoother_->terminationTypeToDiagnosticStatus(termination_type, diagnostic_warning_status,
                                                          diagnostic_error_status);
  EXPECT_EQ(diag_msg.level, diagnostic_msgs::DiagnosticStatus::OK);

  // CONVERGENCE
  termination_type = ceres::TerminationType::CONVERGENCE;
  diag_msg = smoother_->terminationTypeToDiagnosticStatus(termination_type, diagnostic_warning_status,
                                                          diagnostic_error_status);
  EXPECT_EQ(diag_msg.level, diagnostic_msgs::DiagnosticStatus::OK);

  // USER_SUCCESS
  termination_type = ceres::TerminationType::USER_SUCCESS;
  diag_msg = smoother_->terminationTypeToDiagnosticStatus(termination_type, diagnostic_warning_status,
                                                          diagnostic_error_status);
  EXPECT_EQ(diag_msg.level, diagnostic_msgs::DiagnosticStatus::OK);

  // FAILURE
  termination_type = ceres::TerminationType::FAILURE;
  diag_msg = smoother_->terminationTypeToDiagnosticStatus(termination_type, diagnostic_warning_status,
                                                          diagnostic_error_status);
  EXPECT_EQ(diag_msg.level, diagnostic_msgs::DiagnosticStatus::ERROR);

  // USER_FAILURE
  termination_type = ceres::TerminationType::USER_FAILURE;
  diag_msg = smoother_->terminationTypeToDiagnosticStatus(termination_type, diagnostic_warning_status,
                                                          diagnostic_error_status);
  EXPECT_EQ(diag_msg.level, diagnostic_msgs::DiagnosticStatus::ERROR);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "fixed_lag_smoother_node");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
