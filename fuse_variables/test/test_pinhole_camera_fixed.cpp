/*
 * Software License Agreement (BSD License)
 *
 *  Author:    Oscar Mendez
 *  Created:   11.13.2023
 *
 *  Copyright (c) 2021, Locus Robotics
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
#include <fuse_core/serialization.h>
#include <fuse_variables/pinhole_camera_fixed.h>
#include <fuse_variables/stamped.h>
#include <ros/time.h>

#include <ceres/autodiff_cost_function.h>
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <gtest/gtest.h>

#include <sstream>
#include <vector>

using fuse_variables::PinholeCameraFixed;

TEST(PinholeCameraFixed, Type)
{
  PinholeCameraFixed variable(0);
  EXPECT_EQ("fuse_variables::PinholeCameraFixed", variable.type());
}

TEST(PinholeCameraFixed, UUID)
{
  // Verify two positions with the same landmark ids produce the same uuids
  {
    PinholeCameraFixed variable1(0);
    PinholeCameraFixed variable2(0);
    EXPECT_EQ(variable1.uuid(), variable2.uuid());
  }

  // Verify two positions with the different landmark ids  produce different uuids
  {
    PinholeCameraFixed variable1(0);
    PinholeCameraFixed variable2(1);
    EXPECT_NE(variable1.uuid(), variable2.uuid());
  }
}

struct CostFunctor
{
  CostFunctor()
  {
  }

  template <typename T>
  bool operator()(const T* const k, T* residual) const
  {
    residual[0] = k[0] - T(1.2);
    residual[1] = k[1] + T(0.8);
    residual[2] = k[2] - T(0.51);
    residual[3] = k[3] - T(0.49);
    return true;
  }
};

TEST(PinholeCameraFixed, Optimization)
{
  // Create a Point3DLandmark
  PinholeCameraFixed K(0);
  K.fx() = 4.10;
  K.fy() = 3.50;
  K.cx() = 5.00;
  K.cy() = 2.49;

  // Create a simple a constraint
  ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<CostFunctor, 4, 4>(new CostFunctor());

  // Build the problem.
  ceres::Problem problem;
  problem.AddParameterBlock(K.data(), K.size());
  std::vector<double*> parameter_blocks;
  parameter_blocks.push_back(K.data());

  if (K.holdConstant())
  {
    problem.SetParameterBlockConstant(K.data());
  }
  problem.AddResidualBlock(cost_function, nullptr, parameter_blocks);

  // Run the solver
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // Check
  EXPECT_NEAR(4.10, K.fx(), 1.0e-5);
  EXPECT_NEAR(3.50, K.fy(), 1.0e-5);
  EXPECT_NEAR(5.00, K.cx(), 1.0e-5);
  EXPECT_NEAR(2.49, K.cy(), 1.0e-5);
}

struct FuseProjectionCostFunctor
{
  Eigen::Matrix<double, 8, 3> X;
  Eigen::Matrix<double, 8, 3> x;

  FuseProjectionCostFunctor()
  {
    // Define 3D Points
    X << (-1.0), (-1.0), (9.0),  // NOLINT
        (-1.0), (-1.0), (11.0),  // NOLINT
        (-1.0), (1.0), (9.0),    // NOLINT
        (-1.0), (1.0), (11.0),   // NOLINT
        (1.0), (-1.0), (9.0),    // NOLINT
        (1.0), (-1.0), (11.0),   // NOLINT
        (1.0), (1.0), (9.0),     // NOLINT
        (1.0), (1.0), (11.0);    // NOLINT

    // Define 2D Points
    x << (239.36340595030663), (166.35226250921954), (1.0),  // NOLINT
        (252.25926024520876), (179.34432670587358), (1.0),   // NOLINT
        (239.36340595030663), (309.26496867241400), (1.0),   // NOLINT
        (252.25926024520876), (296.27290447575996), (1.0),   // NOLINT
        (381.21780319423020), (166.35226250921954), (1.0),   // NOLINT
        (368.32194889932810), (179.34432670587358), (1.0),   // NOLINT
        (381.21780319423020), (309.26496867241400), (1.0),   // NOLINT
        (368.32194889932810), (296.27290447575996), (1.0);   // NOLINT
  }

  /* FuseProjectionCostFunctor
   *  Projects a set of 3D poitns X to 2D points x using the equation:
   *  x = KX
   *  and optiimzes K using a reprojection error as residual
   *
   */

  template <typename T>
  bool operator()(const T* const k, T* residual) const
  {
    // Create Matrix
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> K(3, 3);
    K <<  k[0],   T(0.0), k[2],     // NOLINT
          T(0.0), k[1],   k[3],     // NOLINT
          T(0.0), T(0.0), T(1.0);   // NOLINT

    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> xp(8, 3);
    xp = (K * X.cast<T>().transpose());

    // TODO(omendez): There is probably a better way to do this using hnormalized on operation above.
    xp.transposeInPlace();
    xp = xp.array().colwise() / X.cast<T>().col(2).array();

    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> d = x.cast<T>() - xp;

    for (uint i = 0; i < 8; i++)
    {
      residual[i * 2] = T(d.row(i)[0]);
      residual[i * 2 + 1] = T(d.row(i)[1]);
    }

    return true;
  }
};

TEST(PinholeCameraFixed, FuseProjectionOptimization)
{
  // Create a Camera to Optimize
  PinholeCameraFixed K(0);
  K.fx() = 640.0;  // fx == w
  K.fy() = 640.0;  // fy == w
  K.cx() = 320.0;  // cx == w/2
  K.cy() = 240.0;  // cy == h/2

  // Expected Intrinsics
  PinholeCameraFixed expected(0);
  expected.fx() = 640.0;
  expected.fy() = 640.0;
  expected.cx() = 320.0;
  expected.cy() = 240.0;

  // Create a simple a constraint
  // (NOTE: Separating X and Y residuals is significantly better than joint distance.
  // Therefore, 16 residuals for 8 points.)
  ceres::CostFunction* cost_function =
      new ceres::AutoDiffCostFunction<FuseProjectionCostFunctor, 16, 4>(new FuseProjectionCostFunctor());

  // Build the problem.
  ceres::Problem problem;
  problem.AddParameterBlock(K.data(), K.size());
  std::vector<double*> parameter_blocks;
  parameter_blocks.push_back(K.data());
  problem.AddResidualBlock(cost_function, nullptr, parameter_blocks);

  if (K.holdConstant())
  {
    problem.SetParameterBlockConstant(K.data());
  }

  // Run the solver
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // Check
  EXPECT_NEAR(expected.fx(), K.fx(), 0.1);  // 0.1 Pixel Error
  EXPECT_NEAR(expected.fy(), K.fy(), 0.1);
  EXPECT_NEAR(expected.cx(), K.cx(), 0.1);
  EXPECT_NEAR(expected.cy(), K.cy(), 0.1);
}

struct ProjectionCostFunctor
{
  double X[8][3];  // 3D Points (2x2 Cube at 0,0,10)
  double x[8][2];  // 2D Points (Projection of Cube w/GT intrinsics)
  ProjectionCostFunctor()
    :  // Define 3D Points (2x2 Cube at 0,0,10)
    X  // NOLINT
    {
      -1.0, -1.0, 9.0,   // NOLINT
      -1.0, -1.0, 11.0,  // NOLINT
      -1.0, 1.0,  9.0,   // NOLINT
      -1.0, 1.0,  11.0,  // NOLINT
      1.0,  -1.0, 9.0,   // NOLINT
      1.0,  -1.0, 11.0,  // NOLINT
      1.0,  1.0,  9.0,   // NOLINT
      1.0,  1.0,  11.0   // NOLINT
    }
    ,
    // Define 2D Points
    x  // NOLINT
    {
      239.36340595030663, 166.35226250921954,  // NOLINT
      252.25926024520876, 179.34432670587358,  // NOLINT
      239.36340595030663, 309.26496867241400,  // NOLINT
      252.25926024520876, 296.27290447575996,  // NOLINT
      381.21780319423020, 166.35226250921954,  // NOLINT
      368.32194889932810, 179.34432670587358,  // NOLINT
      381.21780319423020, 309.26496867241400,  // NOLINT
      368.32194889932810, 296.27290447575996   // NOLINT
    }
  {
  }

  /* ProjectionCostFunctor
   *  Projects a set of 3D points X to 2D points x using the equation:
   *  x = (x*f_x + z * c_x)/z
   *  y = (y*f_y + z * c_y)/z
   *  and optiimzes f_x, f_y, c_x, c_y using a reprojection error as residual.
   * (NOTE: Separating X and Y residuals is significantly better than joint distance.
   *
   */

  template <typename T>
  bool operator()(const T* const k, T* residual) const
  {
    // Do Projection Manually
    T xp[8][2];
    for (uint i = 0; i < 8; i++)
    {
      xp[i][0] = (T(X[i][0]) * k[0] + T(X[i][2]) * k[2]) / T(X[i][2]);  // x = (x*f_x + z * c_x)/z
      xp[i][1] = (T(X[i][1]) * k[1] + T(X[i][2]) * k[3]) / T(X[i][2]);  // y = (y*f_y + z * c_y)/z

      T xerr = xp[i][0] - T(x[i][0]);
      T yerr = xp[i][1] - T(x[i][1]);
      residual[i * 2] = xerr;
      residual[i * 2 + 1] = yerr;
    }
    return true;
  }
};

TEST(PinholeCameraFixed, ProjectionOptimization)
{
  // Create a Camera to Optimize
  PinholeCameraFixed K(0, 640, 640, 320, 240);

  PinholeCameraFixed expected(0);
  expected.fx() = 640.0;
  expected.fy() = 640.0;
  expected.cx() = 320.0;
  expected.cy() = 240.0;

  // Create a simple a constraint
  ceres::CostFunction* cost_function =
      new ceres::AutoDiffCostFunction<ProjectionCostFunctor, 16, 4>(new ProjectionCostFunctor());

  // Build the problem.
  ceres::Problem problem;
  problem.AddParameterBlock(K.data(), K.size());
  std::vector<double*> parameter_blocks;
  parameter_blocks.push_back(K.data());
  problem.AddResidualBlock(cost_function, nullptr, parameter_blocks);

  if (K.holdConstant())
  {
    problem.SetParameterBlockConstant(K.data());
  }

  // Run the solver
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // Check
  EXPECT_NEAR(expected.fx(), K.fx(), 1e-5);  // 0.1 Pixel Error
  EXPECT_NEAR(expected.fy(), K.fy(), 1e-5);
  EXPECT_NEAR(expected.cx(), K.cx(), 1e-5);
  EXPECT_NEAR(expected.cy(), K.cy(), 1e-5);
}

struct PerPointProjectionCostFunctor
{
  double pt3d_x;
  double pt3d_y;
  double pt3d_z;
  double observed_x;
  double observed_y;

  PerPointProjectionCostFunctor(double pt3d_x, double pt3d_y, double pt3d_z, double observed_x, double observed_y)
    : pt3d_x(pt3d_x), pt3d_y(pt3d_y), pt3d_z(pt3d_z), observed_x(observed_x), observed_y(observed_y)
  {
  }

  /* PerPointProjectionCostFunctor
   *
   *  Projects a 3D point XYZ to 2D xy points using the equation:
   *  x = (X*f_x + Z * c_x)/Z
   *  y = (Y*f_y + Z * c_y)/Z
   *  and optiimzes f_x, f_y, c_x, c_y using a reprojection error as residual.
   * (NOTE: Separating X and Y residuals is significantly better than joint distance.
   *
   */

  template <typename T>
  bool operator()(const T* const k, T* residual) const
  {
    // Do Projection Manually
    T xp = (T(pt3d_x) * k[0] + T(pt3d_z) * k[2]) / T(pt3d_z);  // x = (x*f_x + z * c_x)/z
    T yp = (T(pt3d_y) * k[1] + T(pt3d_z) * k[3]) / T(pt3d_z);  // y = (y*f_y + z * c_y)/z

    residual[0] = T(xp - T(observed_x));
    residual[1] = T(yp - T(observed_y));

    return true;
  }
};

TEST(PinholeCameraFixed, PerPointProjectionCostFunctor)
{
  // Create a Camera to Optimize
  PinholeCameraFixed K(0);
  K.fx() = 645.0;  // fx == w
  K.fy() = 635.0;  // fy == w
  K.cx() = 325.0;  // cx == w/2
  K.cy() = 245.0;  // cy == h/2

  PinholeCameraFixed expected(0);
  expected.fx() = 645.0;
  expected.fy() = 635.0;
  expected.cx() = 325.0;
  expected.cy() = 245.0;

  double X[8][3] = { -1.0, -1.0, 9.0,     // NOLINT
                     -1.0, -1.0, 11.0,    // NOLINT
                     -1.0, 1.0,  9.0,     // NOLINT
                     -1.0, 1.0,  11.0,    // NOLINT
                     1.0,  -1.0, 9.0,     // NOLINT
                     1.0,  -1.0, 11.0,    // NOLINT
                     1.0,  1.0,  9.0,     // NOLINT
                     1.0,  1.0,  11.0 };  // NOLINT

  // Define 2D Points
  double pts2d[8][2] = { 239.36340595030663, 166.35226250921954,    // NOLINT
                         252.25926024520876, 179.34432670587358,    // NOLINT
                         239.36340595030663, 309.26496867241400,    // NOLINT
                         252.25926024520876, 296.27290447575996,    // NOLINT
                         381.21780319423020, 166.35226250921954,    // NOLINT
                         368.32194889932810, 179.34432670587358,    // NOLINT
                         381.21780319423020, 309.26496867241400,    // NOLINT
                         368.32194889932810, 296.27290447575996 };  // NOLINT

  // Build the problem.
  ceres::Problem problem;
  problem.AddParameterBlock(K.data(), K.size());

  std::vector<double*> camera_parameter_blocks;
  camera_parameter_blocks.push_back(K.data());

  if (K.holdConstant())
  {
    problem.SetParameterBlockConstant(K.data());
  }

  for (uint i = 0; i < 8; i++)
  {
    // Create a simple a constraint
    ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<PerPointProjectionCostFunctor, 2, 4>(
        new PerPointProjectionCostFunctor(X[i][0], X[i][1], X[i][2], pts2d[i][0], pts2d[i][1]));
    problem.AddResidualBlock(cost_function, nullptr, K.data());
  }

  // Run the solver
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // Check
  EXPECT_NEAR(expected.fx(), K.fx(), 0.1);  // 0.1 Pixel Error
  EXPECT_NEAR(expected.fy(), K.fy(), 0.1);
  EXPECT_NEAR(expected.cx(), K.cx(), 0.1);
  EXPECT_NEAR(expected.cy(), K.cy(), 0.1);
}

TEST(PinholeCameraFixed, Serialization)
{
  // Create a Point3DLandmark
  PinholeCameraFixed expected(0);
  expected.fx() = 640;
  expected.fy() = 480;
  expected.cx() = 320;
  expected.cy() = 240;

  // Serialize the variable into an archive
  std::stringstream stream;
  {
    fuse_core::TextOutputArchive archive(stream);
    expected.serialize(archive);
  }

  // Deserialize a new variable from that same stream
  PinholeCameraFixed actual;
  {
    fuse_core::TextInputArchive archive(stream);
    actual.deserialize(archive);
  }

  // Compare
  EXPECT_EQ(expected.id(), actual.id());
  EXPECT_EQ(expected.uuid(), actual.uuid());
  EXPECT_EQ(expected.fx(), actual.fx());
  EXPECT_EQ(expected.fy(), actual.fy());
  EXPECT_EQ(expected.cx(), actual.cx());
  EXPECT_EQ(expected.cy(), actual.cy());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
