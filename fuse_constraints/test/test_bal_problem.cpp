/*
 * Software License Agreement (BSD License)
 *
 *  Author: Oscar Mendez
 *  Created on Dec 12 2023
 *
 *  Copyright (c) 2023 Locus Robotics
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
#include <fuse_constraints/reprojection_error_snavelly_constraint.h>
#include <fuse_core/eigen.h>
#include <fuse_core/eigen_gtest.h>
#include <fuse_core/serialization.h>
#include <fuse_core/uuid.h>
#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/point_3d_landmark.h>
#include <fuse_variables/point_3d_fixed_landmark.h>
#include <fuse_variables/position_3d_stamped.h>
#include <fuse_variables/pinhole_camera_radial.h>

#include <ceres/covariance.h>
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <gtest/gtest.h>

#include <string>
#include <utility>
#include <vector>

using fuse_constraints::ReprojectionErrorSnavellyConstraint;
using fuse_variables::Orientation3DStamped;
using fuse_variables::PinholeCameraRadial;
using fuse_variables::Point3DFixedLandmark;
using fuse_variables::Point3DLandmark;
using fuse_variables::Position3DStamped;

#ifndef BAL_PROBLEM_H
#define FUSE_CONSTRAINTS_REPROJECTION_ERROR_CONSTRAINT_H

// BALProblem adapted from:
// https://ceres-solver.googlesource.com/ceres-solver/+/master/examples/simple_bundle_adjuster.cc
class BALProblem
{
public:
  BALProblem()
  {
  }
  ~BALProblem()
  {
    delete[] point_index_;
    delete[] camera_index_;
    delete[] observations_;
    delete[] parameters_;
  }
  int num_observations() const
  {
    return num_observations_;
  }
  const double* observations() const
  {
    return observations_;
  }
  double* mutable_cameras()
  {
    return parameters_;
  }
  double* mutable_points()
  {
    return parameters_ + 10 * num_cameras_;
  }
  double* mutable_camera_for_observation(int i)
  {
    return mutable_cameras() + camera_index_[i] * 10;
  }
  double* mutable_point_for_observation(int i)
  {
    return mutable_points() + point_index_[i] * 3;
  }
  int camera_for_observation(int i)
  {
    return camera_index_[i];
  }
  int point_for_observation(int i)
  {
    return point_index_[i];
  }
  double* camera(int i)
  {
    return mutable_cameras() + i * 10;
  }
  double* points(int i)
  {
    return mutable_points() + i * 3;
  }

  bool LoadFile(const char* filename)
  {
    FILE* fptr = fopen(filename, "r");
    if (fptr == nullptr)
    {
      return false;
    };
    FscanfOrDie(fptr, "%d", &num_cameras_);
    FscanfOrDie(fptr, "%d", &num_points_);
    FscanfOrDie(fptr, "%d", &num_observations_);
    point_index_ = new int[num_observations_];
    camera_index_ = new int[num_observations_];
    observations_ = new double[2 * num_observations_];
    num_parameters_ = 9 * num_cameras_ + 3 * num_points_;
    parameters_ = new double[num_parameters_];
    for (int i = 0; i < num_observations_; ++i)
    {
      FscanfOrDie(fptr, "%d", camera_index_ + i);
      FscanfOrDie(fptr, "%d", point_index_ + i);
      for (int j = 0; j < 2; ++j)
      {
        FscanfOrDie(fptr, "%lf", observations_ + 2 * i + j);
      }
    }

    for (int i = 0; i < num_parameters_; ++i)
    {
      FscanfOrDie(fptr, "%lf", parameters_ + i);
    }

    {
      // Switch the angle-axis rotations to quaternions.
      num_parameters_ = 10 * num_cameras_ + 3 * num_points_;
      auto* quaternion_parameters = new double[num_parameters_];
      double* original_cursor = parameters_;
      double* quaternion_cursor = quaternion_parameters;
      for (int i = 0; i < num_cameras_; ++i)
      {
        ceres::AngleAxisToQuaternion(original_cursor, quaternion_cursor);
        quaternion_cursor += 4;
        original_cursor += 3;
        for (int j = 4; j < 10; ++j)
        {
          *quaternion_cursor++ = *original_cursor++;
        }
      }
      // Copy the rest of the points.
      for (int i = 0; i < 3 * num_points_; ++i)
      {
        *quaternion_cursor++ = *original_cursor++;
      }
      // Swap in the quaternion parameters.
      delete[] parameters_;
      parameters_ = quaternion_parameters;
    }

    return true;
  }

  int num_cameras()
  {
    return num_cameras_;
  }

  int num_points()
  {
    return num_points_;
  }

private:
  template <typename T>
  void FscanfOrDie(FILE* fptr, const char* format, T* value)
  {
    int num_scanned = fscanf(fptr, format, value);
    if (num_scanned != 1)
    {
      LOG(FATAL) << "Invalid UW data file.";
    }
  }
  int num_cameras_;
  int num_points_;
  int num_observations_;
  int num_parameters_;
  int* point_index_;
  int* camera_index_;
  double* observations_;
  double* parameters_;
  bool use_quaternions_;
};
#endif

// CeresSnavellyReprojectionErrorWithQuaternion adapted from:
// https://ceres-solver.googlesource.com/ceres-solver/+/master/examples/snavely_reprojection_error.h
// Templated pinhole camera model for used with Ceres.  The camera is
// parameterized using 9 parameters: 3 for rotation, 3 for translation, 1 for
// focal length and 2 for radial distortion. The principal point is not modeled
// (i.e. it is assumed be located at the image center).
struct SnavelyReprojectionErrorWithQuaternions
{
  // (u, v): the position of the observation with respect to the image
  // center point.
  SnavelyReprojectionErrorWithQuaternions(double observed_x, double observed_y)
    : observed_x(observed_x), observed_y(observed_y)
  {
  }
  template <typename T>
  bool operator()(const T* const camera, const T* const point, T* residuals) const
  {
    // camera[0,1,2,3] is the rotation of the camera as a quaternion.
    //
    // We use QuaternionRotatePoint as it does not assume that the
    // quaternion is normalized, since one of the ways to run the
    // bundle adjuster is to let Ceres optimize all 4 quaternion
    // parameters without using a Quaternion manifold.
    T p[3];
    ceres::QuaternionRotatePoint(camera, point, p);
    p[0] += camera[4];
    p[1] += camera[5];
    p[2] += camera[6];
    // Compute the center of distortion. The sign change comes from
    // the camera model that Noah Snavely's Bundler assumes, whereby
    // the camera coordinate system has a negative z axis.
    const T xp = -p[0] / p[2];
    const T yp = -p[1] / p[2];
    // Apply second and fourth order radial distortion.
    const T& l1 = camera[8];
    const T& l2 = camera[9];
    const T r2 = xp * xp + yp * yp;
    const T distortion = 1.0 + r2 * (l1 + l2 * r2);
    // Compute final projected point position.
    const T& focal = camera[7];
    const T predicted_x = focal * distortion * xp;
    const T predicted_y = focal * distortion * yp;

    residuals[0] = predicted_x - observed_x;
    residuals[1] = predicted_y - observed_y;
    return true;
  }

  static ceres::CostFunction* Create(const double observed_x, const double observed_y)
  {
    return (new ceres::AutoDiffCostFunction<SnavelyReprojectionErrorWithQuaternions, 2, 10, 3>(
        new SnavelyReprojectionErrorWithQuaternions(observed_x, observed_y)));
  }
  double observed_x;
  double observed_y;
};

TEST(ReprojectionErrorSnavellyConstraint, Constructor)
{
  // Construct a constraint just to make sure it compiles.
  Position3DStamped position_variable(ros::Time(1234, 5678), fuse_core::uuid::generate("walle"));
  Orientation3DStamped orientation_variable(ros::Time(1234, 5678), fuse_core::uuid::generate("walle"));
  Point3DLandmark point(0);
  PinholeCameraRadial calibration_variable(0);

  fuse_core::Vector2d mean;
  mean << 320.0, 240.0;  // Centre of a 640x480 camera

  // Assume Half a pixel Variance
  fuse_core::Matrix2d cov;
  cov << 0.5, 0.0,  // NOLINT
      0.0, 0.5;     // NOLINT

  EXPECT_NO_THROW(ReprojectionErrorSnavellyConstraint constraint("test", position_variable, orientation_variable,
                                                                 calibration_variable, mean, cov));
}

TEST(ReprojectionErrorSnavellyConstraint, Covariance)
{
  // Verify the covariance <--> sqrt information conversions are correct
  Position3DStamped position_variable(ros::Time(1234, 5678), fuse_core::uuid::generate("mo"));
  Orientation3DStamped orientation_variable(ros::Time(1234, 5678), fuse_core::uuid::generate("mo"));
  Point3DLandmark point(0);
  PinholeCameraRadial calibration_variable(0);

  fuse_core::Vector2d mean;
  mean << 320.0, 240.0;  // Centre of a 640x480 camera

  // Assume Half a pixel Variance
  fuse_core::Matrix2d cov;
  cov << 0.5, 0.0,  // NOLINT
      0.0, 0.5;     // NOLINT

  ReprojectionErrorSnavellyConstraint constraint("test", position_variable, orientation_variable, calibration_variable,
                                                 mean, cov);

  // Define the expected matrices (used Octave to compute sqrt_info: 'chol(inv(A))')
  fuse_core::Matrix2d expected_sqrt_info;
  expected_sqrt_info << 1.414213562373095, 0,  // NOLINT
      0, 1.414213562373095;                    // NOLINT
  fuse_core::Matrix2d expected_cov = cov;

  // Compare
  EXPECT_MATRIX_NEAR(expected_cov, constraint.covariance(), 1.0e-9);
  EXPECT_MATRIX_NEAR(expected_sqrt_info, constraint.sqrtInformation(), 1.0e-9);
}

TEST(ReprojectionErrorSnavellyConstraint, BAL)
{
  std::string filename = "problem-21-11315-pre.txt";
  BALProblem bal_problem_ceres;
  if (!bal_problem_ceres.LoadFile(filename.c_str()))
  {
    std::cerr << "ERROR: unable to open file " << filename << "\n";
    throw;
  }

  // Use Ceres Standard
  const double* observations_ceres = bal_problem_ceres.observations();
  ceres::Problem problem_ceres;
  for (int i = 0; i < bal_problem_ceres.num_observations(); ++i)
  {
    ceres::CostFunction* cost_function =
        SnavelyReprojectionErrorWithQuaternions::Create(observations_ceres[2 * i + 0], observations_ceres[2 * i + 1]);
    problem_ceres.AddResidualBlock(cost_function, nullptr /* squared loss */,
                                   bal_problem_ceres.mutable_camera_for_observation(i),
                                   bal_problem_ceres.mutable_point_for_observation(i));
  }
  ceres::Solver::Options options_ceres;
  options_ceres.linear_solver_type = ceres::DENSE_SCHUR;
  ceres::Solver::Summary summary_ceres;
  ceres::Solve(options_ceres, &problem_ceres, &summary_ceres);

  BALProblem bal_problem;
  if (!bal_problem.LoadFile(filename.c_str()))
  {
    std::cerr << "ERROR: unable to open file " << filename << "\n";
    throw;
  }

  std::vector<Position3DStamped> cams_p(bal_problem.num_cameras());
  std::vector<Orientation3DStamped> cams_q(bal_problem.num_cameras());
  std::vector<PinholeCameraRadial> cams_k(bal_problem.num_cameras());
  for (int i = 0; i < bal_problem.num_cameras(); i++)
  {
    auto cam = bal_problem.camera(i);

    cams_q[i].w() = cam[0];
    cams_q[i].x() = cam[1];
    cams_q[i].y() = cam[2];
    cams_q[i].z() = cam[3];

    cams_p[i].x() = cam[4];
    cams_p[i].y() = cam[5];
    cams_p[i].z() = cam[6];

    cams_k[i].f() = cam[7];
    cams_k[i].r1() = cam[8];
    cams_k[i].r2() = cam[9];
  }

  std::vector<Point3DLandmark> pts(bal_problem.num_points());
  for (int i = 0; i < bal_problem.num_points(); i++)
  {
    auto pt = bal_problem.points(i);
    pts[i].x() = pt[0];
    pts[i].y() = pt[1];
    pts[i].z() = pt[2];
  }

  ceres::Problem problem;
  const double* observations = bal_problem.observations();
  for (int i = 0; i < bal_problem.num_observations(); ++i)
  {
    int c = bal_problem.camera_for_observation(i);
    cams_q[c];
    cams_p[c];
    cams_k[c];

    int p = bal_problem.point_for_observation(i);
    pts[p];

    // Create an observation
    fuse_core::Vector2d mean;
    mean << observations[2 * i + 0], observations[2 * i + 1];

    // Define Observation Covariance
    fuse_core::Matrix2d cov;
    cov << 1e-5, 0.0,  // NOLINT
        0.0, 1e-5;     // NOLINT

    auto constraint =
        ReprojectionErrorSnavellyConstraint::make_shared("test", cams_p[c], cams_q[c], cams_k[c], mean, cov);

    problem.AddParameterBlock(pts[p].data(), pts[p].size(), pts[p].localParameterization());

    std::vector<double*> parameter_blocks;
    parameter_blocks.push_back(cams_p[c].data());
    parameter_blocks.push_back(cams_q[c].data());
    parameter_blocks.push_back(cams_k[c].data());
    parameter_blocks.push_back(pts[p].data());

    problem.AddResidualBlock(constraint->costFunction(), constraint->lossFunction(), parameter_blocks);
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  for (int i = 0; i < bal_problem.num_cameras(); i++)
  {
    auto cam = bal_problem_ceres.camera(i);

    EXPECT_NEAR(cams_q[i].w(), cam[0], 1e-2);
    EXPECT_NEAR(cams_q[i].x(), cam[1], 1e-2);
    EXPECT_NEAR(cams_q[i].y(), cam[2], 1e-2);
    EXPECT_NEAR(cams_q[i].z(), cam[3], 1e-2);

    EXPECT_NEAR(cams_p[i].x(), cam[4], 1e-2);
    EXPECT_NEAR(cams_p[i].y(), cam[5], 1e-2);
    EXPECT_NEAR(cams_p[i].z(), cam[6], 1e-2);

    EXPECT_NEAR(cams_k[i].f(), cam[7], 1e-2);
    EXPECT_NEAR(cams_k[i].r1(), cam[8], 1e-2);
    EXPECT_NEAR(cams_k[i].r2(), cam[9], 1e-2);
  }

  for (int i = 0; i < bal_problem.num_points(); i++)
  {
    auto pt = bal_problem_ceres.points(i);
    EXPECT_NEAR(pts[i].x(), pt[0], 1e-2);
    EXPECT_NEAR(pts[i].y(), pt[1], 1e-2);
    EXPECT_NEAR(pts[i].z(), pt[2], 1e-2);
  }
}

TEST(ReprojectionErrorSnavellyConstraint, Serialization)
{
  // Construct a constraint
  Position3DStamped position_variable(ros::Time(1234, 5678), fuse_core::uuid::generate("walle"));
  Orientation3DStamped orientation_variable(ros::Time(1234, 5678), fuse_core::uuid::generate("walle"));

  PinholeCameraRadial calibration_variable(0);
  calibration_variable.f() = 640;
  calibration_variable.r1() = 0.1;
  calibration_variable.r2() = 0.1;

  fuse_core::Vector2d mean;
  mean << 261.71822455, 168.60442225;

  // Generated PD matrix using Octave: R = rand(6, 6); A = R * R' (use format long g to get the required precision)
  fuse_core::Matrix2d cov;
  cov << 0.5, 0.0,  // NOLINT
      0.5, 0.5;     // NOLINT

  ReprojectionErrorSnavellyConstraint expected("test", position_variable, orientation_variable, calibration_variable,
                                               mean, cov);

  // Serialize the constraint into an archive
  std::stringstream stream;
  {
    fuse_core::TextOutputArchive archive(stream);
    expected.serialize(archive);
  }

  // Deserialize a new constraint from that same stream
  ReprojectionErrorSnavellyConstraint actual;
  {
    fuse_core::TextInputArchive archive(stream);
    actual.deserialize(archive);
  }

  // Compare
  EXPECT_EQ(expected.uuid(), actual.uuid());
  EXPECT_EQ(expected.variables(), actual.variables());
  EXPECT_MATRIX_EQ(expected.mean(), actual.mean());
  EXPECT_MATRIX_EQ(expected.sqrtInformation(), actual.sqrtInformation());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
