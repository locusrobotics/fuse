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
#ifndef FUSE_CORE__EIGEN_HPP_
#define FUSE_CORE__EIGEN_HPP_

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <sstream>
#include <string>

namespace fuse_core
{

// Define some Eigen Typedefs that use Row-Major order
using VectorXd = Eigen::Matrix<double, Eigen::Dynamic, 1>;
using Vector1d = Eigen::Matrix<double, 1, 1>;
using Vector2d = Eigen::Matrix<double, 2, 1>;
using Vector3d = Eigen::Matrix<double, 3, 1>;
using Vector4d = Eigen::Matrix<double, 4, 1>;
using Vector5d = Eigen::Matrix<double, 5, 1>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Vector8d = Eigen::Matrix<double, 8, 1>;
using Vector9d = Eigen::Matrix<double, 9, 1>;
using Vector15d = Eigen::Matrix<double, 15, 1>;

using MatrixXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using Matrix1d = Eigen::Matrix<double, 1, 1, Eigen::RowMajor>;
using Matrix2d = Eigen::Matrix<double, 2, 2, Eigen::RowMajor>;
using Matrix3d = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;
using Matrix4d = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;
using Matrix5d = Eigen::Matrix<double, 5, 5, Eigen::RowMajor>;
using Matrix6d = Eigen::Matrix<double, 6, 6, Eigen::RowMajor>;
using Matrix7d = Eigen::Matrix<double, 7, 7, Eigen::RowMajor>;
using Matrix8d = Eigen::Matrix<double, 8, 8, Eigen::RowMajor>;
using Matrix9d = Eigen::Matrix<double, 9, 9, Eigen::RowMajor>;
using Matrix15d = Eigen::Matrix<double, 15, 15, Eigen::RowMajor>;

template<typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime>
using Matrix = Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime, Eigen::RowMajor>;

/**
 * @brief Serialize a matrix into an std::string using this format:
 *
 * [1, 2, 3]
 * [4, 5, 6]
 * [7, 8, 9]
 *
 * @param[in] m - The matrix to serialize into an std::string.
 * @param[in] precision - The precision to print the matrix elements with.
 * @return An std::string with the matrix serialized into it.
 */
template<typename Derived>
std::string to_string(const Eigen::DenseBase<Derived> & m, const int precision = 4)
{
  static const Eigen::IOFormat pretty(precision, 0, ", ", "\n", "[", "]");

  std::ostringstream oss;
  oss << m.format(pretty) << '\n';
  return oss.str();
}

/**
 * @brief Check if a matrix is symmetric.
 *
 * @param[in] m - Square matrix to check symmetry on
 * @param[in] precision - Precision used to compared the matrix m with its transpose, which is the
 *                        property used to check for symmetry.
 * @return True if the matrix m is symmetric; False, otherwise.
 */
template<typename Derived>
bool isSymmetric(
  const Eigen::DenseBase<Derived> & m,
  const typename Eigen::DenseBase<Derived>::RealScalar precision =
  Eigen::NumTraits<typename Eigen::DenseBase<Derived>::Scalar>::dummy_precision())
{
  // We do not use `isApprox`:
  //
  // return m.isApprox(m.transpose(), precision);
  //
  // because it does not play well when `m` is close to zero.
  //
  // See:
  // https://eigen.tuxfamily.org/dox/classEigen_1_1DenseBase.html#ae8443357b808cd393be1b51974213f9c
  const auto & derived = m.derived();
  return (derived - derived.transpose()).cwiseAbs().maxCoeff() < precision;
}

/**
 * @brief Check if a matrix is Positive Definite (PD), i.e. all eigenvalues are `> 0.0`.
 *
 * @param[in] m - Square matrix to check PD-ness on.
 * @return True if the matrix m is PD; False, otherwise.
 */
template<typename Derived>
bool isPositiveDefinite(const Eigen::DenseBase<Derived> & m)
{
  Eigen::SelfAdjointEigenSolver<Derived> solver(m);
  return solver.eigenvalues().minCoeff() > 0.0;
}

}  // namespace fuse_core

#endif  // FUSE_CORE__EIGEN_HPP_
