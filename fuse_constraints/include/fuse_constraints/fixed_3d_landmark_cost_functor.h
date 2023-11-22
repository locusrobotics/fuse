/*
 * Software License Agreement (BSD License)
 *
 *  Author: Oscar Mendez
 *  Created on Mon Nov 12 2023
 * 
 *  Copyright (c) 2023, Locus Robotics
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
#ifndef FUSE_CONSTRAINTS_NORMAL_PRIOR_POSE_3D_COST_FUNCTOR_H
#define FUSE_CONSTRAINTS_NORMAL_PRIOR_POSE_3D_COST_FUNCTOR_H

#include <fuse_constraints/normal_prior_orientation_3d_cost_functor.h>
#include <fuse_core/eigen.h>
#include <fuse_core/fuse_macros.h>
#include <fuse_core/util.h>

#include <Eigen/Core>

namespace fuse_constraints
{

/**
 * @brief Create a prior cost function on both the 3D position and orientation variables at once.
 *
 * The Ceres::NormalPrior cost function only supports a single variable. This is a convenience cost function that
 * applies a prior constraint on both the 3D position and orientation variables at once.
 *
 * The cost function is of the form:
 *
 *   cost(x) = || A * [  p - b(0:2)               ] ||^2
 *             ||     [  AngleAxis(b(3:6)^-1 * q) ] ||
 *
 * where, the matrix A and the vector b are fixed, p is the position variable, and q is the orientation variable.
 * Note that the covariance submatrix for the quaternion is 3x3, representing errors in the orientation local
 * parameterization tangent space. In case the user is interested in implementing a cost function of the form
 *
 *   cost(X) = (X - mu)^T S^{-1} (X - mu)
 *
 * where, mu is a vector and S is a covariance matrix, then, A = S^{-1/2}, i.e the matrix A is the square root
 * information matrix (the inverse of the covariance).
 */
class Fixed3DLandmarkCostFunctor
{
public:
  FUSE_MAKE_ALIGNED_OPERATOR_NEW();

  /**
   * @brief Construct a cost function instance
   *
   * @param[in] A The residual weighting matrix, most likely the square root information matrix in order
   *              (x, y, z, qx, qy, qz)
   * @param[in] b The 3D pose measurement or prior in order (x, y, z, qw, qx, qy, qz)
   * 
   * @param[in] obs The 2D projection of marker points.
   * 
   * @param[in] marker_size The marker_size of the makrker (in meters).
   **/
  Fixed3DLandmarkCostFunctor(const fuse_core::MatrixXd& A, const fuse_core::Vector7d& b, const fuse_core::MatrixXd& obs, const fuse_core::Vector1d& marker_size);

  /**
   * @brief Evaluate the cost function. Used by the Ceres optimization engine.
   */
  template <typename T>
  bool operator()(const T* const position, const T* const orientation, const T* const calibration, T* residual) const;

private:
  fuse_core::MatrixXd A_;
  fuse_core::Vector7d b_;
  fuse_core::MatrixXd obs_;
  fuse_core::Vector1d marker_size_;
  fuse_core::Matrix4d pts3d_;

  NormalPriorOrientation3DCostFunctor orientation_functor_;
};

Fixed3DLandmarkCostFunctor::Fixed3DLandmarkCostFunctor(const fuse_core::MatrixXd& A, const fuse_core::Vector7d& b, const fuse_core::MatrixXd& obs, const fuse_core::Vector1d& marker_size)
  : A_(A), b_(b), obs_(obs), marker_size_(marker_size), orientation_functor_(fuse_core::Matrix3d::Identity(), b_.tail<4>())  // Delta will not be scaled
{
  // Define 3D Homogeneous 3D Points at origin, assume z-up
  pts3d_ << -1.0, -1.0, 0.0, 1.0,  // NOLINT
            -1.0,  1.0, 0.0, 1.0,  // NOLINT
             1.0, -1.0, 0.0, 1.0,  // NOLINT
             1.0,  1.0, 0.0, 1.0;  // NOLINT
  
  // Scale
  pts3d_.block(0,0,4,2)*=marker_size_.value(); // Scalar Multiplication
  pts3d_.transposeInPlace();

    // Create Marker Transform Matrix (Tm)
  double qm[4] =
  {
    b_(3),
    b_(4),
    b_(5),
    b_(6)
  };
  double rm[9];
  ceres::QuaternionToRotation(qm, rm);

  Eigen::Matrix<double, 4, 4, Eigen::RowMajor> Tm;
  Tm <<
    rm[0], rm[1], rm[2], b_(0),  // NOLINT
    rm[3], rm[4], rm[5], b_(1),  // NOLINT
    rm[6], rm[7], rm[8], b_(2),  // NOLINT
      0.0,   0.0,   0.0,   1.0;  // NOLINT
    
  // TODO: (omendez) Can we do this directly on quaternions? 
  // ceres::QuaternionProduct(q, position, difference);
  // ceres::QuaternionRotatePoint()

  // Transform points to marker pose and store for later use.
  pts3d_ = Tm * pts3d_;
  
}

template <typename T>
bool Fixed3DLandmarkCostFunctor::operator()(const T* const position, const T* const orientation,
                                            const T* const calibration, T* residual) const
{
  
  // Create Calibration Matrix K
  Eigen::Matrix<T, 4, 4, Eigen::RowMajor> K(4, 4);
  K <<  calibration[0],   T(0.0),           calibration[2],   T(0.0),  // NOLINT
        T(0.0),           calibration[1],   calibration[3],   T(0.0),  // NOLINT
        T(0.0),           T(0.0),           T(1.0),           T(0.0),  // NOLINT
        T(0.0),           T(0.0),           T(0.0),           T(1.0);

  // Create Marker Transform Matrix (Tm)
  T qm[4] =
  {
    T(b_(3)),
    T(b_(4)),
    T(b_(5)),
    T(b_(6))
  };
  T rm[9];
  ceres::QuaternionToRotation(qm, rm);

  Eigen::Matrix<T, 4, 4, Eigen::RowMajor> Tm;
  Tm <<
    rm[0], rm[1], rm[2], T(b_(0)),
    rm[3], rm[4], rm[5], T(b_(1)),
    rm[6], rm[7], rm[8], T(b_(2)),
    T(0.0), T(0.0), T(0.0), T(1.0);

  // Create Camera Translation Matrix from Params.
  T q[4] =
  {
    orientation[0],
    orientation[1],
    orientation[2],
    orientation[3]
  };
  T r[9];  
  ceres::QuaternionToRotation(q, r);

  Eigen::Matrix<T, 4, 4, Eigen::RowMajor> Ta;
  Ta <<
    r[0], r[1], r[2], position[0],
    r[3], r[4], r[5], position[1],
    r[6], r[7], r[8], position[2],
    T(0.0), T(0.0), T(0.0), T(1.0);
    
  // TODO: (omendez) Can we do this directly on quaternions? 
  // ceres::QuaternionProduct(q, position, difference);
  // ceres::QuaternionRotatePoint()

  // Transform 3D Points to Marker Location
  Eigen::Matrix<T, 4, 4, Eigen::RowMajor> xp;

  // Transform points to marker pose. 
  // Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> X;
  // X = Tm * pts3d_.cast<T>();
  
  // Project to camera at Ta
  xp = (K * Ta * pts3d_.cast<T>());
  xp.transposeInPlace();
  xp = xp.array().colwise() / xp.col(2).array();

  auto d = (obs_.cast<T>()-xp.block(0,0,4,2));
  
  for (uint i = 0; i < 4; i++)
  {
    residual[i * 2] = d.row(i)[0];
    residual[i * 2 + 1] = d.row(i)[1];
  }


  // std::cout<< " K: " << K.rows()<<" "<< K.cols()<<" \n"<< K<<std::endl;
  // std::cout<< "Tm: " <<Tm.rows()<<" "<<Tm.cols()<<" \n"<<Tm<<std::endl;
  // std::cout<< "Ta: " <<Ta.rows()<<" "<<Ta.cols()<<" \n"<<Ta<<std::endl;
  // std::cout<< " X: " <<X.rows()<<" "<<X.cols()<<" \n"<<X<<std::endl;
  // std::cout<< "obs: " <<obs_.rows()<<" "<<obs_.cols()<<" \n"<<obs_<<std::endl;
  // std::cout<< " xp: " <<xp.rows()<<" "<<xp.cols()<<" \n"<<xp<<std::endl; 
  // std::cout<< " xp: " <<xp.block(0,0,4,2).rows()<<" "<<xp.block(0,0,4,2).cols()<<" \n"<<xp.block(0,0,4,2)<<std::endl;      
  // std::cout<< "  d: " <<xp.rows()<<" "<<xp.cols()<<" \n"<<(obs_.cast<T>() - xp.block(0,0,4,2))<<std::endl;   
  // std::cout<< "  d: " <<d.rows()<<" "<<d.cols()<<" \n"<<d<<std::endl;   


  // Create Camera Transform Matrix T = [ R | t ]
  // ceres::QuaternionToRotation R(orientation);
  // Eigen::Matrix<T, 4, 4, Eigen::RowMajor> T_ca({
  //   R[0,0], T(0.0), T(0.0), position[0],
  //   T(0.0), T(1.0), T(0.0), position[1],
  //   T(0.0), T(0.0), T(1.0), position[2],
  //   T(0.0), T(0.0), T(0.0), T(1.0)});

  // // Project to Camera
  // Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> xp(4, 4);
  // xp = (K * X.cast<T>().transpose());

  // // TODO(omendez): There is probably a better way to do this using hnormalized on operation above.
  // xp.transposeInPlace();
  // xp = xp.array().colwise() / X.cast<T>().col(2).array();

  // Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> d = x.cast<T>() - xp;

  // for (uint i = 0; i < 8; i++)
  // {
  //   residual[i * 2] = T(d.row(i)[0]);
  //   residual[i * 2 + 1] = T(d.row(i)[1]);

  
  

  // // Compute the position error
  // residual[0] = position[0] - T(b_(0));
  // residual[1] = position[1] - T(b_(1));
  // residual[2] = position[2] - T(b_(2));

  // // Use the 3D orientation cost functor to compute the orientation delta
  // orientation_functor_(orientation, &residual[3]);

  // // Scale the residuals by the square root information matrix to account for
  // // the measurement uncertainty.
  // Eigen::Map<Eigen::Matrix<T, 6, 1>> residual_map(residual);
  // residual_map.applyOnTheLeft(A_.template cast<T>());

  return true;
}

}  // namespace fuse_constraints

#endif  // FUSE_CONSTRAINTS_NORMAL_PRIOR_POSE_3D_COST_FUNCTOR_H
