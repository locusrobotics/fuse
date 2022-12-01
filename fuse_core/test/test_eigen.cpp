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
#include <gtest/gtest.h>

#include <fuse_core/eigen.hpp>

TEST(Eigen, isSymmetric)
{
  const auto random_matrix = fuse_core::Matrix3d::Random().eval();

  // A symmetric matrix:
  const auto symmetric_matrix = (0.5 * (random_matrix + random_matrix.transpose())).eval();

  EXPECT_TRUE(fuse_core::isSymmetric(symmetric_matrix)) << "Matrix\n"
                                                        << symmetric_matrix <<
    "\n expected to be symmetric.";

  // A non-symmetric matrix:
  const double asymmetry_error = 1.0e-6;

  auto non_symmetric_matrix = symmetric_matrix;
  non_symmetric_matrix(0, 1) += asymmetry_error;

  EXPECT_FALSE(fuse_core::isSymmetric(non_symmetric_matrix))
    << "Matrix\n"
    << non_symmetric_matrix << "\n expected to not be symmetric.";

  // Checking symmetry with precision larger than asymmetry error in non-symmetric matrix:
  const double precision = 1.0e2 * asymmetry_error;

  EXPECT_TRUE(fuse_core::isSymmetric(non_symmetric_matrix, precision))
    << "Matrix\n"
    << non_symmetric_matrix << "\n expected to be symmetric with precision " << precision << ".";

  // fuse_core::isSymmetric is not defined for non-square matrices. The following will simply fail
  // to compile because it is not allowed, as intended:
  //
  // const auto non_square_matrix = fuse_core::Matrix<double, 2, 3>::Random().eval();
  //
  // EXPECT_FALSE(fuse_core::isSymmetric(non_square_matrix));
}

TEST(Eigen, isPositiveDefinite)
{
  const auto random_matrix = fuse_core::Matrix3d::Random().eval();

  // A Positive Definite matrix:
  const auto symmetric_matrix = (0.5 * (random_matrix + random_matrix.transpose())).eval();
  const auto psd_matrix = (symmetric_matrix + 3 * fuse_core::Matrix3d::Identity()).eval();

  EXPECT_TRUE(fuse_core::isPositiveDefinite(psd_matrix)) << "Matrix\n"
                                                         << psd_matrix <<
    "\n expected to be Positive Definite.";

  // A non Positive Definite matrix:
  auto non_psd_matrix = psd_matrix;
  non_psd_matrix(0, 0) *= -1.0;

  EXPECT_FALSE(fuse_core::isPositiveDefinite(non_psd_matrix))
    << "Matrix\n"
    << non_psd_matrix << "\n expected to not be Positive Definite.";

  // fuse_core::isPositiveDefinite is not defined for non-square matrices. The following will simply
  // fail to compile because it is allowed, as intended:
  //
  // const auto non_square_matrix = fuse_core::Matrix<double, 2, 3>::Random().eval();
  //
  // EXPECT_FALSE(fuse_core::isPositiveDefinite(non_square_matrix));
}
