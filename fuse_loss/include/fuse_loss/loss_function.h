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
#ifndef FUSE_LOSS_LOSS_FUNCTION_H
#define FUSE_LOSS_LOSS_FUNCTION_H

#include <ceres/loss_function.h>

// This provides additional loss functions that are not available in:
// https://github.com/ceres-solver/ceres-solver/blob/master/include/ceres/loss_function.h
//
// Most of them are available in other libraries like:
// * g2o: https://github.com/RainerKuemmerle/g2o/blob/master/g2o/core/robust_kernel_impl.h
// * GTSAM: https://github.com/borglab/gtsam/blob/develop/gtsam/linear/LossFunctions.h
//          (formerly in https://github.com/borglab/gtsam/blob/develop/gtsam/linear/NoiseModel.h)
//
// Remember that the loss functions in Ceres solver implement a slight variation of the \rho(r) function, defined for
// multiple M-estimators in table #1 from http://www.audentia-gestion.fr/research.microsoft/ZhangIVC-97-01.pdf (p. 24).
//
// In Ceres the loss functions evaluate:
//
//   rho(s) = 2 * \rho(sqrt(s))
//
// where s = r^2, being r the residual.
//
// This is because the output of rho(s) is mutiplied by 0.5 when the residual block cost is computed, either with or
// without loss:
// * cost w/o loss: https://github.com/ceres-solver/ceres-solver/blob/master/internal/ceres/residual_block.cc#L159
// * cost w/  loss: https://github.com/ceres-solver/ceres-solver/blob/master/internal/ceres/residual_block.cc#L165
//
// Note that according to this, it looks like the following loss functions are incorrectly implemented in Ceres:
// * Tukey: It must be multiplied by 2, so instead of dividing by 6 it should divide by 3. See:
//
//            https://github.com/ceres-solver/ceres-solver/blob/master/include/ceres/loss_function.h#L281-L282
//
//          There is an easy workaround for this: combine TukeyLoss with ScaledLoss, using a scaled factor of 2.
//
// * Huber: It is actually a Pseudo-Huber loss function, which is smoother than the original Huber one in table #1 from
//          http://www.audentia-gestion.fr/research.microsoft/ZhangIVC-97-01.pdf (p. 24).
//
//          It also does not look to match the Pseudo-Huber variation suggested in:
//
//            http://en.wikipedia.org/wiki/Huber_Loss_Function
//
//          which is referenced in:
//
//            https://github.com/ceres-solver/ceres-solver/blob/master/include/ceres/loss_function.h#L173
namespace ceres
{

// Dynamic Covariance Scaling (DCS), equivalent to the Geman-McClure with the tuning constant 'a', but scaled by 2.0.
//
// The term is computed as:
//
//   rho(s) = s * 2 * a / (a + s)    for s <  a
//   rho(s) = s                      for s >= a
//
// See http://www2.informatik.uni-freiburg.de/~spinello/agarwalICRA13.pdf (p. 3), where 's' is multiplied by the factor:
//
//   min(1, 2 * a / (a + s))
//
// The g2o implementation seems to use the squared of that factor:
// https://github.com/RainerKuemmerle/g2o/blob/master/g2o/core/robust_kernel_impl.cpp#L167
//
// At s = 0: rho = [0, 1, 0].
class DCSLoss : public ceres::LossFunction
{
public:
  explicit DCSLoss(const double a) : a_(a)
  {
  }

  void Evaluate(double, double* rho) const override;

private:
  const double a_;
};

// Fair, similar to tthe L1 - L2 estimators, that try to take the advantage of the L1 estimators to reduce the influence
// of large erros and that of L2 estimators to be convex. It behave like L2 for small squared residuals 's' and like L1
// for large ones.
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
  explicit FairLoss(const double a) : a_(a), b_(a * a)
  {
  }

  void Evaluate(double, double*) const override;

private:
  const double a_;
  const double b_;
};

// Geman-McClure, similarly to Tukey loss, it tries to reduce the effect of large errors, but it does not suppress
// outliers as Tukey might do.
//
// The term is computed as:
//
//   rho(s) = s / (1 + s)
//
// according to table #1 from http://www.audentia-gestion.fr/research.microsoft/ZhangIVC-97-01.pdf (p. 24).
//
// Remember that in Ceres the implementation of rho(s) must be multiplied by 2 because the cost is set as:
//
//   cost = 0.5 * rho(s)
//
// Here we also consider a tuning constant 'a', so we actually have:
//
//   rho(s) = s * a / (a + s)
//
// At s = 0: rho = [0, 1, -2].
class GemanMcClureLoss : public ceres::LossFunction
{
public:
  explicit GemanMcClureLoss(const double a) : a_(a)
  {
  }

  void Evaluate(double, double*) const override;

private:
  const double a_;
};

// Welsch, similar to Tukey loss, it tries to reduce the effect of large errors, but it does not suppress outliers as
// Tukey might do.
//
// The terms i computed as:
//
//   rho(s) = b * (1 - exp(-s/b))
//
// where b = a * a, being 'a' a tuning constant.
//
// At s = 0: rho = [0, 1, -1/b]
class WelschLoss : public ceres::LossFunction
{
public:
  explicit WelschLoss(const double a) : b_(a * a), c_(-1.0 / b_)
  {
  }

  void Evaluate(double, double*) const override;

private:
  const double b_;
  const double c_;
};

}  // namespace ceres

#endif  // FUSE_LOSS_LOSS_FUNCTION_H
