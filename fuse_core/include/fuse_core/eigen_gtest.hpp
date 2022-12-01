/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Locus Robotics
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
#ifndef FUSE_CORE__EIGEN_GTEST_HPP_
#define FUSE_CORE__EIGEN_GTEST_HPP_

#include <Eigen/Core>
#include <gtest/gtest.h>

/**
 * @file eigen_gtest.h
 *
 * @brief Provides ASSERT_MATRIX_EQ()/EXPECT_MATRIX_EQ() and
 *        ASSERT_MATRIX_NEAR()/EXPECT_MATRIX_NEAR() gtest macros
 */

namespace testing
{

/**
 * @brief Internal helper function for implementing {EXPECT|ASSERT}_MATRIX_EQ.
 *
 * Don't use this in your code.
 *
 * @param[in] e1  Expected matrix name
 * @param[in] e2  Actual matrix name
 * @param[in] v1  Expected matrix
 * @param[in] v2  Actual matrix
 * @return AssertionSuccess or AssertionFailure
 */
template<typename Derived1, typename Derived2>
AssertionResult AssertMatrixEqualHelper(
  const char * e1,
  const char * e2,
  const Eigen::MatrixBase<Derived1> & v1,
  const Eigen::MatrixBase<Derived2> & v2)
{
  if (v1 == v2) {
    return AssertionSuccess();
  }

  Eigen::IOFormat clean(4, 0, ", ", "\n", "[", "]");
  return AssertionFailure() << e1 << " is:\n" << v1.format(clean) << "\n"
                            << e2 << " is:\n" << v2.format(clean) << "\n"
                            << "Difference is:\n" << (v1 - v2).format(clean) << "\n";
}

/**
 * @brief Internal helper function for implementing {EXPECT|ASSERT}_MATRIX_NEAR.
 *
 * Don't use this in your code.
 *
 * @param[in] e1  Expected matrix name
 * @param[in] e2  Actual matrix name
 * @param[in] v1  Expected matrix
 * @param[in] v2  Actual matrix
 * @param[in] tol Tolerance
 * @return AssertionSuccess or AssertionFailure
 */
template<typename Derived1, typename Derived2>
AssertionResult AssertMatrixNearHelper(
  const char * e1,
  const char * e2,
  const Eigen::MatrixBase<Derived1> & v1,
  const Eigen::MatrixBase<Derived2> & v2,
  double tol)
{
  if ((v1 - v2).cwiseAbs().maxCoeff() < tol) {
    return AssertionSuccess();
  }

  Eigen::IOFormat clean(4, 0, ", ", "\n", "[", "]");
  return AssertionFailure() << e1 << " is:\n" << v1.format(clean) << "\n"
                            << e2 << " is:\n" << v2.format(clean) << "\n"
                            << "Difference is:\n" << (v1 - v2).format(clean) << "\n";
}

// Internal macro for implementing {EXPECT|ASSERT}_MATRIX_EQ.
// Don't use this in your code.
#define GTEST_MATRIX_EQUAL_(v1, v2, on_failure) \
  GTEST_ASSERT_( \
    ::testing::AssertMatrixEqualHelper( \
      #v1, \
      #v2, \
      v1, \
      v2), on_failure)

// Internal macro for implementing {EXPECT|ASSERT}_MATRIX_NEAR.
// Don't use this in your code.
#define GTEST_MATRIX_NEAR_(v1, v2, tol, on_failure) \
  GTEST_ASSERT_( \
    ::testing::AssertMatrixNearHelper( \
      #v1, \
      #v2, \
      v1, \
      v2, \
      tol), on_failure)

// Define gtest macros for use with Eigen

/**
 * @brief Non-fatal check for exact equality of two Eigen matrix-like objects.
 *
 * This should probably be used only for integer-based matrix types
 *
 * @param[in] v1 The expected matrix
 * @param[in] v2 The actual matrix
 */
#define EXPECT_MATRIX_EQ(v1, v2) \
  GTEST_MATRIX_EQUAL_(v1, v2, GTEST_NONFATAL_FAILURE_)

/**
 * @brief Fatal check for exact equality of two Eigen matrix-like objects.
 *
 * This should probably be used only for integer-based matrix types
 *
 * @param[in] v1 The expected matrix
 * @param[in] v2 The actual matrix
 */
#define ASSERT_MATRIX_EQ(v1, v2) \
  GTEST_MATRIX_EQUAL_(v1, v2, GTEST_FATAL_FAILURE_)

/**
 * @brief Non-fatal check for approximate equality of two Eigen matrix-like objects.
 *
 * This version return success if abs(v1[i] - v2[i]) < tol for every element i in the matrix.
 *
 * @param[in] v1  The expected matrix
 * @param[in] v2  The actual matrix
 * @param[in] tol The allowed tolerance between any entries in v1 and v2
 */
#define EXPECT_MATRIX_NEAR(v1, v2, tol) \
  GTEST_MATRIX_NEAR_(v1, v2, tol, GTEST_NONFATAL_FAILURE_)

/**
 * @brief Fatal check for approximate equality of two Eigen matrix-like objects.
 *
 * This version return success if abs(v1[i] - v2[i]) < tol for every element i in the matrix.
 *
 * @param[in] v1  The expected matrix
 * @param[in] v2  The actual matrix
 * @param[in] tol The allowed tolerance between any entries in v1 and v2
 */
#define ASSERT_MATRIX_NEAR(v1, v2, tol) \
  GTEST_MATRIX_NEAR_(v1, v2, tol, GTEST_FATAL_FAILURE_)

}  // namespace testing

#endif  // FUSE_CORE__EIGEN_GTEST_HPP_
