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
#ifndef FUSE_LOSS__LOSS_FUNCTION_HPP_
#define FUSE_LOSS__LOSS_FUNCTION_HPP_

#include <ceres/loss_function.h>

// This provides additional loss functions that are not available in:
// https://github.com/ceres-solver/ceres-solver/blob/master/include/ceres/loss_function.h
//
// Most of them are available in other libraries like:
// * g2o: https://github.com/RainerKuemmerle/g2o/blob/master/g2o/core/robust_kernel_impl.h
// * GTSAM: https://github.com/borglab/gtsam/blob/develop/gtsam/linear/LossFunctions.h
// (formerly in https://github.com/borglab/gtsam/blob/develop/gtsam/linear/NoiseModel.h)
//
// Remember that the loss functions in Ceres solver implement a slight variation of the \rho(r)
// function, defined for multiple M-estimators in table #1 from http://www.audentia-
// gestion.fr/research.microsoft/ZhangIVC-97-01.pdf (p. 24).
//
// In Ceres the loss functions evaluate:
//
//   rho(s) = 2 * \rho(sqrt(s))
//
// where s = r^2, being r the residual.
//
// This is because the output of rho(s) is mutiplied by 0.5 when the residual block cost is
// computed, either with or without loss:
//
// * cost w/o loss:
//   https://github.com/ceres-solver/ceres-solver/blob/master/internal/ceres/residual_block.cc#L159
//
// * cost w/  loss:
//   https://github.com/ceres-solver/ceres-solver/blob/master/internal/ceres/residual_block.cc#L165
namespace ceres
{

// Dynamic Covariance Scaling (DCS), equivalent to Switchable Constraints and similar to the Geman-
// McClure with the tuning constant 'a'.
//
// The term is computed as:
//
//   rho(s) = a * (3 * s  - a) / (a + s)    for s >  a
//   rho(s) = s                             for s <= a
//
// which gives the weight function:
//
//   rho'(s) = min { 1, (2 * a / (a + s))^2 }
//
// as described in Eq. 5.19 and 5.20 in:
//
//   http://www2.informatik.uni-freiburg.de/~agarwal/resources/agarwal-thesis.pdf (p. 89)
//
// that is equal to the square of the scaling factor in Eq. 15 in the original paper:
//
//   http://www2.informatik.uni-freiburg.de/~spinello/agarwalICRA13.pdf (p.3)
//
// which is also reproduced in Eq. 5.15 in:
//
//   http://www2.informatik.uni-freiburg.de/~agarwal/resources/agarwal-thesis.pdf (pp. 85-88)
//
// The M-estimator rho(s) equivalent to DCS is obtained from Eq. 5.18 and 5.19 by integration,
// giving the expression in Eq. 5.20. This way we obtain a valid robust kernel, i.e. one that has
// positive definite weight function rho'(s) >= 0, that can be used in an Iteratively Reweighted
// Least Squares (IRLS) problem. For more details, see:
//
//   http://www2.informatik.uni-freiburg.de/~agarwal/resources/agarwal-thesis.pdf (p. 89)
//
// The relation with the Geman-McClure loss function is explained in:
//
//   http://www2.informatik.uni-freiburg.de/~agarwal/resources/agarwal-thesis.pdf (pp. 90-91)
//
// The DCS rho(s) equation is equivalent to a scaled and translated generalized Geman-McClure for
// s > a (outlier region) with a_GemanMcClure = sqrt(a_DCS), as shown in Eq. 5.28.
// With the rho(s) = 2 * \rho(sqrt(s)) notation, where s = r^2 (r = x in the thesis), we have:
//
//   rho_DCS(s) = 4 * rho_GemanMcClure(s) - a
//
// The implementation in GTSAM has the same weight function rho'(s) we use here:
//
// https://github.com/borglab/gtsam/blob/09b0f03542bfbec5cca62645a60c5d1d4f8f/gtsam/linear/
// LossFunctions.cpp#L348-L357
//
// But its residual rho(r) is not defined as the integral of rho'(r):
//
// https://github.com/borglab/gtsam/blob/09b0f03542bfbec5cca62645a60c5d1d4f8f/gtsam/linear/
// LossFunctions.cpp#L359-L367
//
// Similarly, the implementation in g2o also has the same weight function rho'(s) we use here:
//
// https://github.com/RainerKuemmerle/g2o/blob/fcba4eaca6f20d9a5792404cc8ef303aeb/g2o/core/
// robust_kernel_impl.cpp#L168
//
// But its residual rho(s) is also not defined as the integral of rho'(s):
//
// https://github.com/RainerKuemmerle/g2o/blob/fcba4eaca6f20d9a5792404cc8ef303aeb/g2o/core/
// robust_kernel_impl.cpp#L167
//
// Indeed, the 1st and 2nd derivatives rho'(s) and rho''(s) in g2o seem to consider the scaling
// factor a constant, but that is not correct. However, if they had computed the correct derivatives
// for the rho(s) function they use, then rho'(s) would not be positive definite, which is required
// for robust non-linear least squares problems. This is indeed enforced in the corrector in:
//
// https://github.com/ceres-solver/ceres-
// solver/blob/8e962f37d756272e7019a5d28394fc8f/internal/ceres/corrector.h#L60
//
// which is based on Eq. 10 and 11 from BAMS (Bundle Adjustment -- A Modern Synthesis):
//
//   https://hal.inria.fr/inria-00548290/document
//
// and it requires that rho'(s) >=0 because it is used to compute sqrt(rho'(s)) in the equations
// that correct the residuals and jacobian.
//
// At s = 0: rho = [0, 1, 0].
class DCSLoss : public ceres::LossFunction
{
public:
  explicit DCSLoss(const double a)
  : a_(a)
  {
  }

  void Evaluate(double, double * rho) const override;

private:
  const double a_;
};

// Fair, similar to tthe L1 - L2 estimators, that try to take the advantage of the L1 estimators to
// reduce the influence of large erros and that of L2 estimators to be convex. It behave like L2 for
// small squared residuals 's' and like L1 for large ones.
//
// The term is computed as:
//
//   rho(s) = 2 * b * (r/a - log(1 + r/a))
//
// where b = a * a, being 'a' a tuning constant, and r = sqrt(s) the residual.
//
// At s = 0: rho = [0, 1, -Inf]
class FairLoss : public ceres::LossFunction
{
public:
  explicit FairLoss(const double a)
  : a_(a), b_(a * a)
  {
  }

  void Evaluate(double, double *) const override;

private:
  const double a_;
  const double b_;
};

// Geman-McClure, similarly to Tukey loss, it tries to reduce the effect of large errors, but it
// does not suppress outliers as Tukey might do.
//
// The term is computed as:
//
//   rho(s) = s / (1 + s)
//
// according to table #1 from http://www.audentia-gestion.fr/research.microsoft/ZhangIVC-97-01.pdf
// (p. 24) and Eq. 5.23 in:
//
//   http://www2.informatik.uni-freiburg.de/~agarwal/resources/agarwal-thesis.pdf (pp. 89-90)
//
// where the original Geman-McClure is presented in Eq. 5.21 and the generalized Geman-McClure is
// defined introducing the parametr 'a'. It also shows how it is adapted to be a positive definite
// function.
//
// Remember that in Ceres the implementation of rho(s) must be multiplied by 2 because the cost is
// set as:
//
//   cost = 0.5 * rho(s)
//
// Here we also consider a tuning constant 'a' that scales the residual 'r' doing:
//
//   \hat{r} = r / a
//
// where s = r^2, so we actually have:
//
//   rho(s) = s * b / (b + s)
//
// where b = a^2.
//
// This is equivalent to the implementation in GTSAM:
//
// https://github.com/borglab/gtsam/blob/57da7b31d07a233421d9419b85e6f90bacf0/gtsam/linear/
// LossFunctions.cpp#L314-L325
//
// At s = 0: rho = [0, 1, -2].
class GemanMcClureLoss : public ceres::LossFunction
{
public:
  explicit GemanMcClureLoss(const double a)
  : b_(a * a)
  {
  }

  void Evaluate(double, double *) const override;

private:
  const double b_;
};

// Welsch, similar to Tukey loss, it tries to reduce the effect of large errors, but it does not
// suppress outliers as Tukey might do.
//
// The term is computed as:
//
//   rho(s) = b * (1 - exp(-s/b))
//
// where b = a * a, being 'a' a tuning constant.
//
// At s = 0: rho = [0, 1, -1/b]
class WelschLoss : public ceres::LossFunction
{
public:
  explicit WelschLoss(const double a)
  : b_(a * a), c_(-1.0 / b_)
  {
  }

  void Evaluate(double, double *) const override;

private:
  const double b_;
  const double c_;
};

}  // namespace ceres

#endif  // FUSE_LOSS__LOSS_FUNCTION_HPP_
