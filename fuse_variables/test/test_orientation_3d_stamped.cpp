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
#include <ceres/autodiff_cost_function.h>
#include <ceres/cost_function_to_functor.h>
#include <ceres/problem.h>
#include <ceres/rotation.h>
#include <ceres/solver.h>
#include <Eigen/Core>
#include <gtest/gtest.h>

#include <sstream>
#include <vector>

#include <fuse_core/autodiff_local_parameterization.hpp>
#include <fuse_core/ceres_macros.hpp>
#include <fuse_core/eigen.hpp>
#include <fuse_core/serialization.hpp>
#include <fuse_variables/orientation_3d_stamped.hpp>
#include <fuse_variables/stamped.hpp>
#include <rclcpp/time.hpp>

using fuse_variables::Orientation3DStamped;

TEST(Orientation3DStamped, Type)
{
  Orientation3DStamped variable(rclcpp::Time(12345678, 910111213));
  EXPECT_EQ("fuse_variables::Orientation3DStamped", variable.type());
}

TEST(Orientation3DStamped, UUID)
{
  // Verify two orientations at the same timestamp produce the same UUID
  {
    Orientation3DStamped variable1(rclcpp::Time(12345678, 910111213));
    Orientation3DStamped variable2(rclcpp::Time(12345678, 910111213));
    EXPECT_EQ(variable1.uuid(), variable2.uuid());
  }

  auto uuid_1 = fuse_core::uuid::generate("test_hardware1");
  auto uuid_2 = fuse_core::uuid::generate("test_hardware2");

  // Verify two orientations at the same timestamp and same hardware ID produce the same UUID
  {
    Orientation3DStamped variable1(rclcpp::Time(12345678, 910111213), uuid_1);
    Orientation3DStamped variable2(rclcpp::Time(12345678, 910111213), uuid_1);
    EXPECT_EQ(variable1.uuid(), variable2.uuid());
  }

  // Verify two orientations with the same timestamp but different hardware IDs generate different
  // UUIDs
  {
    Orientation3DStamped variable1(rclcpp::Time(12345678, 910111213), uuid_1);
    Orientation3DStamped variable2(rclcpp::Time(12345678, 910111213), uuid_2);
    EXPECT_NE(variable1.uuid(), variable2.uuid());
  }

  // Verify two orientations with the same hardware ID and different timestamps produce different
  // UUIDs
  {
    Orientation3DStamped variable1(rclcpp::Time(12345678, 910111213), uuid_1);
    Orientation3DStamped variable2(rclcpp::Time(12345678, 910111214), uuid_1);
    EXPECT_NE(variable1.uuid(), variable2.uuid());

    Orientation3DStamped variable3(rclcpp::Time(12345678, 910111213), uuid_1);
    Orientation3DStamped variable4(rclcpp::Time(12345679, 910111213), uuid_1);
    EXPECT_NE(variable3.uuid(), variable4.uuid());
  }

  // Verify two orientations with different hardware IDs and different timestamps produce different
  // UUIDs
  {
    Orientation3DStamped variable1(rclcpp::Time(12345678, 910111213), uuid_1);
    Orientation3DStamped variable2(rclcpp::Time(12345678, 910111214), uuid_2);
    EXPECT_NE(variable1.uuid(), variable2.uuid());

    Orientation3DStamped variable3(rclcpp::Time(12345678, 910111213), uuid_1);
    Orientation3DStamped variable4(rclcpp::Time(12345679, 910111213), uuid_2);
    EXPECT_NE(variable3.uuid(), variable4.uuid());
  }
}

template<typename T>
inline static void QuaternionInverse(const T in[4], T out[4])
{
  out[0] = in[0];
  out[1] = -in[1];
  out[2] = -in[2];
  out[3] = -in[3];
}

struct Orientation3DPlus
{
  template<typename T>
  bool operator()(const T * x, const T * delta, T * x_plus_delta) const
  {
    T q_delta[4];
    ceres::AngleAxisToQuaternion(delta, q_delta);
    ceres::QuaternionProduct(x, q_delta, x_plus_delta);
    return true;
  }
};

struct Orientation3DMinus
{
  template<typename T>
  bool operator()(const T * x, const T * y, T * y_minus_x) const
  {
    T x_inverse[4];
    QuaternionInverse(x, x_inverse);
    T q_delta[4];
    ceres::QuaternionProduct(x_inverse, y, q_delta);
    ceres::QuaternionToAngleAxis(q_delta, y_minus_x);
    return true;
  }
};

using Orientation3DLocalParameterization =
  fuse_core::AutoDiffLocalParameterization<Orientation3DPlus, Orientation3DMinus, 4, 3>;

TEST(Orientation3DStamped, Plus)
{
  auto parameterization = Orientation3DStamped(rclcpp::Time(0, 0)).localParameterization();

  double x[4] = {0.842614977, 0.2, 0.3, 0.4};
  double delta[3] = {0.15, -0.2, 0.433012702};
  double result[4] = {0.0, 0.0, 0.0, 0.0};
  bool success = parameterization->Plus(x, delta, result);

  EXPECT_TRUE(success);
  EXPECT_NEAR(0.745561, result[0], 1.0e-5);
  EXPECT_NEAR(0.360184, result[1], 1.0e-5);
  EXPECT_NEAR(0.194124, result[2], 1.0e-5);
  EXPECT_NEAR(0.526043, result[3], 1.0e-5);

  delete parameterization;
}

TEST(Orientation3DStamped, Minus)
{
  auto parameterization = Orientation3DStamped(rclcpp::Time(0, 0)).localParameterization();

  double x1[4] = {0.842614977, 0.2, 0.3, 0.4};
  double x2[4] = {0.745561, 0.360184, 0.194124, 0.526043};
  double result[3] = {0.0, 0.0, 0.0};
  bool success = parameterization->Minus(x1, x2, result);

  EXPECT_TRUE(success);
  EXPECT_NEAR(0.15, result[0], 1.0e-5);
  EXPECT_NEAR(-0.2, result[1], 1.0e-5);
  EXPECT_NEAR(0.433012702, result[2], 1.0e-5);

  delete parameterization;
}

TEST(Orientation3DStamped, PlusJacobian)
{
  auto parameterization = Orientation3DStamped(rclcpp::Time(0, 0)).localParameterization();
  auto reference = Orientation3DLocalParameterization();

  for (double qx = -0.5; qx < 0.5; qx += 0.1) {
    for (double qy = -0.5; qy < 0.5; qy += 0.1) {
      for (double qz = -0.5; qz < 0.5; qz += 0.1) {
        double qw = std::sqrt(1.0 - qx * qx - qy * qy - qz * qz);

        double x[4] = {qw, qx, qy, qz};
        fuse_core::MatrixXd actual(4, 3);
        /* *INDENT-OFF* */
        actual << 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0;
        /* *INDENT-ON* */
        bool success = parameterization->ComputeJacobian(x, actual.data());

        fuse_core::MatrixXd expected(4, 3);
        /* *INDENT-OFF* */
        expected << 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0;
        /* *INDENT-ON* */
        reference.ComputeJacobian(x, expected.data());

        EXPECT_TRUE(success);
        Eigen::IOFormat clean(4, 0, ", ", "\n", "[", "]");
        EXPECT_TRUE(
          expected.isApprox(
            actual,
            1.0e-5)) << "Expected is:\n" << expected.format(clean) << "\n"
                     << "Actual is:\n" << actual.format(clean) << "\n"
                     << "Difference is:\n" << (expected - actual).format(clean)
                     << "\n";
      }
    }
  }

  delete parameterization;
}

TEST(Orientation3DStamped, MinusJacobian)
{
  auto parameterization = Orientation3DStamped(rclcpp::Time(0, 0)).localParameterization();
  auto reference = Orientation3DLocalParameterization();

  for (double qx = -0.5; qx < 0.5; qx += 0.1) {
    for (double qy = -0.5; qy < 0.5; qy += 0.1) {
      for (double qz = -0.5; qz < 0.5; qz += 0.1) {
        double qw = std::sqrt(1.0 - qx * qx - qy * qy - qz * qz);

        double x[4] = {qw, qx, qy, qz};
        fuse_core::MatrixXd actual(3, 4);
        /* *INDENT-OFF* */
        actual << 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0;
        /* *INDENT-ON* */
        bool success = parameterization->ComputeMinusJacobian(x, actual.data());

        fuse_core::MatrixXd expected(3, 4);
        /* *INDENT-OFF* */
        expected << 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0;
        /* *INDENT-ON* */
        reference.ComputeMinusJacobian(x, expected.data());

        EXPECT_TRUE(success);
        Eigen::IOFormat clean(4, 0, ", ", "\n", "[", "]");
        EXPECT_TRUE(
          expected.isApprox(
            actual,
            1.0e-5)) << "Expected is:\n" << expected.format(clean) << "\n"
                     << "Actual is:\n" << actual.format(clean) <<
          "\n"
                     << "Difference is:\n" <<
          (expected - actual).format(clean)
                     << "\n";
      }
    }
  }

  delete parameterization;
}

TEST(Orientation3DStamped, Stamped)
{
  fuse_core::Variable::SharedPtr base = Orientation3DStamped::make_shared(
    rclcpp::Time(12345678, 910111213), fuse_core::uuid::generate("mo"));
  auto derived = std::dynamic_pointer_cast<Orientation3DStamped>(base);
  ASSERT_TRUE(static_cast<bool>(derived));
  EXPECT_EQ(rclcpp::Time(12345678, 910111213), derived->stamp());
  EXPECT_EQ(fuse_core::uuid::generate("mo"), derived->deviceId());

  auto stamped = std::dynamic_pointer_cast<fuse_variables::Stamped>(base);
  ASSERT_TRUE(static_cast<bool>(stamped));
  EXPECT_EQ(rclcpp::Time(12345678, 910111213), stamped->stamp());
  EXPECT_EQ(fuse_core::uuid::generate("mo"), stamped->deviceId());
}

struct QuaternionCostFunction
{
  explicit QuaternionCostFunction(double * observation)
  {
    observation_[0] = observation[0];
    observation_[1] = observation[1];
    observation_[2] = observation[2];
    observation_[3] = observation[3];
  }

  template<typename T>
  bool operator()(const T * quaternion, T * residual) const
  {
    T inverse_quaternion[4] =
    {
      quaternion[0],
      -quaternion[1],
      -quaternion[2],
      -quaternion[3]
    };

    T observation[4] =
    {
      T(observation_[0]),
      T(observation_[1]),
      T(observation_[2]),
      T(observation_[3])
    };

    T output[4];

    ceres::QuaternionProduct(observation, inverse_quaternion, output);

    // Residual can just be the imaginary components
    residual[0] = output[1];
    residual[1] = output[2];
    residual[2] = output[3];

    return true;
  }

  double observation_[4];
};

TEST(Orientation3DStamped, Optimization)
{
  // Create an Orientation3DStamped with R, P, Y values of 10, -20, 30 degrees
  Orientation3DStamped orientation(rclcpp::Time(12345678, 910111213));
  orientation.w() = 0.952;
  orientation.x() = 0.038;
  orientation.y() = -0.189;
  orientation.z() = 0.239;

  // Create a simple a constraint with an identity quaternion
  double target_quat[4] = {1.0, 0.0, 0.0, 0.0};
  ceres::CostFunction * cost_function =
    new ceres::AutoDiffCostFunction<QuaternionCostFunction, 3,
      4>(new QuaternionCostFunction(target_quat));

  // Build the problem.
  ceres::Problem problem;
#if !CERES_SUPPORTS_MANIFOLDS
  problem.AddParameterBlock(
    orientation.data(),
    orientation.size(),
    orientation.localParameterization());
#else
  problem.AddParameterBlock(
    orientation.data(),
    orientation.size(),
    orientation.manifold());
#endif
  std::vector<double *> parameter_blocks;
  parameter_blocks.push_back(orientation.data());
  problem.AddResidualBlock(cost_function, nullptr, parameter_blocks);

  // Run the solver
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // Check
  EXPECT_NEAR(target_quat[0], orientation.w(), 1.0e-3);
  EXPECT_NEAR(target_quat[1], orientation.x(), 1.0e-3);
  EXPECT_NEAR(target_quat[2], orientation.y(), 1.0e-3);
  EXPECT_NEAR(target_quat[3], orientation.z(), 1.0e-3);
}

TEST(Orientation3DStamped, Euler)
{
  const double RAD_TO_DEG = 180.0 / M_PI;

  // Create an Orientation3DStamped with R, P, Y values of 10, -20, 30 degrees
  Orientation3DStamped orientation_r(rclcpp::Time(12345678, 910111213));
  orientation_r.w() = 0.9961947;
  orientation_r.x() = 0.0871557;
  orientation_r.y() = 0.0;
  orientation_r.z() = 0.0;

  EXPECT_NEAR(10.0, RAD_TO_DEG * orientation_r.roll(), 1e-5);

  Orientation3DStamped orientation_p(rclcpp::Time(12345678, 910111213));
  orientation_p.w() = 0.9848078;
  orientation_p.x() = 0.0;
  orientation_p.y() = -0.1736482;
  orientation_p.z() = 0.0;

  EXPECT_NEAR(-20.0, RAD_TO_DEG * orientation_p.pitch(), 1e-5);

  Orientation3DStamped orientation_y(rclcpp::Time(12345678, 910111213));
  orientation_y.w() = 0.9659258;
  orientation_y.x() = 0.0;
  orientation_y.y() = 0.0;
  orientation_y.z() = 0.258819;

  EXPECT_NEAR(30.0, RAD_TO_DEG * orientation_y.yaw(), 1e-5);
}

TEST(Orientation3DStamped, Serialization)
{
  // Create an Orientation3DStamped
  Orientation3DStamped expected(rclcpp::Time(12345678, 910111213));
  expected.w() = 0.952;
  expected.x() = 0.038;
  expected.y() = -0.189;
  expected.z() = 0.239;

  // Serialize the variable into an archive
  std::stringstream stream;
  {
    fuse_core::TextOutputArchive archive(stream);
    expected.serialize(archive);
  }

  // Deserialize a new variable from that same stream
  Orientation3DStamped actual;
  {
    fuse_core::TextInputArchive archive(stream);
    actual.deserialize(archive);
  }

  // Compare
  EXPECT_EQ(expected.deviceId(), actual.deviceId());
  EXPECT_EQ(expected.stamp(), actual.stamp());
  EXPECT_EQ(expected.w(), actual.w());
  EXPECT_EQ(expected.x(), actual.x());
  EXPECT_EQ(expected.y(), actual.y());
  EXPECT_EQ(expected.z(), actual.z());
}

#if CERES_SUPPORTS_MANIFOLDS
#include <ceres/autodiff_manifold.h>

struct Orientation3DFunctor
{
  template<typename T>
  bool Plus(const T * x, const T * delta, T * x_plus_delta) const
  {
    T q_delta[4];
    ceres::AngleAxisToQuaternion(delta, q_delta);
    ceres::QuaternionProduct(x, q_delta, x_plus_delta);
    return true;
  }
  template<typename T>
  bool Minus(const T * y, const T * x, T * y_minus_x) const
  {
    T x_inverse[4];
    QuaternionInverse(x, x_inverse);
    T q_delta[4];
    ceres::QuaternionProduct(x_inverse, y, q_delta);
    ceres::QuaternionToAngleAxis(q_delta, y_minus_x);
    return true;
  }
};

using Orientation3DManifold = ceres::AutoDiffManifold<Orientation3DFunctor, 4, 3>;

TEST(Orientation3DStamped, ManifoldPlus)
{
  auto manifold = Orientation3DStamped(rclcpp::Time(0, 0)).manifold();

  double x[4] = {0.842614977, 0.2, 0.3, 0.4};
  double delta[3] = {0.15, -0.2, 0.433012702};
  double result[4] = {0.0, 0.0, 0.0, 0.0};
  bool success = manifold->Plus(x, delta, result);

  EXPECT_TRUE(success);
  EXPECT_NEAR(0.745561, result[0], 1.0e-5);
  EXPECT_NEAR(0.360184, result[1], 1.0e-5);
  EXPECT_NEAR(0.194124, result[2], 1.0e-5);
  EXPECT_NEAR(0.526043, result[3], 1.0e-5);

  delete manifold;
}

TEST(Orientation3DStamped, ManifoldPlusJacobian)
{
  auto manifold = Orientation3DStamped(rclcpp::Time(0, 0)).manifold();
  auto reference = Orientation3DManifold();

  for (double qx = -0.5; qx < 0.5; qx += 0.1) {
    for (double qy = -0.5; qy < 0.5; qy += 0.1) {
      for (double qz = -0.5; qz < 0.5; qz += 0.1) {
        double qw = std::sqrt(1.0 - qx * qx - qy * qy - qz * qz);

        double x[4] = {qw, qx, qy, qz};
        fuse_core::MatrixXd actual(4, 3);
        /* *INDENT-OFF* */
        actual << 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0;
        /* *INDENT-ON* */
        bool success = manifold->PlusJacobian(x, actual.data());

        fuse_core::MatrixXd expected(4, 3);
        /* *INDENT-OFF* */
        expected << 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0;
        /* *INDENT-ON* */
        reference.PlusJacobian(x, expected.data());

        EXPECT_TRUE(success);
        Eigen::IOFormat clean(4, 0, ", ", "\n", "[", "]");
        EXPECT_TRUE(
          expected.isApprox(
            actual,
            1.0e-5)) << "Expected is:\n" << expected.format(clean) << "\n"
                     << "Actual is:\n" << actual.format(clean) << "\n"
                     << "Difference is:\n" << (expected - actual).format(clean)
                     << "\n";
      }
    }
  }

  delete manifold;
}

TEST(Orientation3DStamped, ManifoldMinus)
{
  double x1[4] = {0.842614977, 0.2, 0.3, 0.4};
  double x2[4] = {0.745561, 0.360184, 0.194124, 0.526043};
  double result[3] = {0.0, 0.0, 0.0};

  auto manifold = Orientation3DStamped(rclcpp::Time(0, 0)).manifold();
  bool success = manifold->Minus(x2, x1, result);

  EXPECT_TRUE(success);
  EXPECT_NEAR(0.15, result[0], 1.0e-5);
  EXPECT_NEAR(-0.2, result[1], 1.0e-5);
  EXPECT_NEAR(0.433012702, result[2], 1.0e-5);

  delete manifold;
}

TEST(Orientation3DStamped, ManifoldMinusJacobian)
{
  auto manifold = Orientation3DStamped(rclcpp::Time(0, 0)).manifold();
  auto reference = Orientation3DManifold();

  for (double qx = -0.5; qx < 0.5; qx += 0.1) {
    for (double qy = -0.5; qy < 0.5; qy += 0.1) {
      for (double qz = -0.5; qz < 0.5; qz += 0.1) {
        double qw = std::sqrt(1.0 - qx * qx - qy * qy - qz * qz);

        double x[4] = {qw, qx, qy, qz};
        fuse_core::MatrixXd actual(3, 4);
        /* *INDENT-OFF* */
        actual << 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0;
        /* *INDENT-ON* */
        bool success = manifold->MinusJacobian(x, actual.data());

        fuse_core::MatrixXd expected(3, 4);
        /* *INDENT-OFF* */
        expected << 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0;
        /* *INDENT-ON* */
        reference.MinusJacobian(x, expected.data());

        EXPECT_TRUE(success);
        Eigen::IOFormat clean(4, 0, ", ", "\n", "[", "]");
        EXPECT_TRUE(
          expected.isApprox(
            actual,
            1.0e-5)) << "Expected is:\n" << expected.format(clean) << "\n"
                     << "Actual is:\n" << actual.format(clean) << "\n"
                     << "Difference is:\n" << (expected - actual).format(clean)
                     << "\n";
      }
    }
  }

  delete manifold;
}

TEST(Orientation3DStamped, ManifoldSerialization)
{
  // Create an Orientation3DStamped
  Orientation3DStamped expected(rclcpp::Time(12345678, 910111213));
  expected.w() = 0.952;
  expected.x() = 0.038;
  expected.y() = -0.189;
  expected.z() = 0.239;

  // Serialize the variable into an archive
  std::stringstream stream;
  {
    fuse_core::TextOutputArchive archive(stream);
    expected.serialize(archive);
  }

  // Deserialize a new variable from that same stream
  Orientation3DStamped actual;
  {
    fuse_core::TextInputArchive archive(stream);
    actual.deserialize(archive);
  }

  // Compare
  EXPECT_EQ(expected.deviceId(), actual.deviceId());
  EXPECT_EQ(expected.stamp(), actual.stamp());
  EXPECT_EQ(expected.w(), actual.w());
  EXPECT_EQ(expected.x(), actual.x());
  EXPECT_EQ(expected.y(), actual.y());
  EXPECT_EQ(expected.z(), actual.z());
}

#endif
