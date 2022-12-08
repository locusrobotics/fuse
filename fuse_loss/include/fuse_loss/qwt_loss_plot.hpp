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

#ifndef FUSE_LOSS__QWT_LOSS_PLOT_HPP_
#define FUSE_LOSS__QWT_LOSS_PLOT_HPP_

#include <qwt_legend.h>
#include <qwt_plot.h>
#include <qwt_plot_curve.h>
#include <qwt_plot_grid.h>
#include <qwt_plot_magnifier.h>
#include <qwt_plot_panner.h>
#include <qwt_plot_renderer.h>
#include <qwt_plot_zoomer.h>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace fuse_loss
{

/**
 * @brief HSV colormap that varies the hue H in the [0, 1) range with equidistant points for the
 *        given size. The hue H interval is open because the hue is the same for 0 and 1. Therefore,
 *        we never take hue == 1.
 */
class HSVColormap
{
public:
  explicit HSVColormap(const size_t size = 256)
  {
    if (size == 0) {
      return;
    }

    colormap_.reserve(size);

    double hue = 0.0;
    const double hue_increment = 1.0 / size;
    for (size_t i = 0; i < size; ++i, hue += hue_increment) {
      QColor color;
      color.setHsvF(hue, 1.0, 1.0);

      colormap_.push_back(color);
    }
  }

  const QColor & operator[](const size_t i) const
  {
    return colormap_[i];
  }

  size_t size() const
  {
    return colormap_.size();
  }

private:
  std::vector<QColor> colormap_;
};

class LossEvaluator
{
public:
  explicit LossEvaluator(const std::vector<double> & residuals)
  : residuals_(residuals)
  {
  }

  std::vector<double> rho(const ceres::LossFunction * loss_function) const
  {
    std::vector<double> rhos;
    rhos.reserve(residuals_.size());

    // Remember that the loss functions we use are like Ceres solver ones, i.e. they're defined as:
    //
    //   rho(s) = 2 * \rho(sqrt(s))
    //
    // where s = r^2, being r the residual.
    //
    // See: https://github.com/ceres-solver/ceres-
    // solver/blob/master/internal/ceres/residual_block.cc#L165
    std::transform(
      residuals_.begin(), residuals_.end(), std::back_inserter(rhos),
      [&loss_function](const auto & r) {              // NOLINT(whitespace/braces)
        double rho[3];
        loss_function->Evaluate(r * r, rho);
        return 0.5 * rho[0];
      });               // NOLINT(whitespace/braces)

    return rhos;
  }

  std::vector<double> influence(const ceres::LossFunction * loss_function) const
  {
    std::vector<double> influence;
    influence.reserve(residuals_.size());

    // The influence is defined as the first derivative of \rho wrt the residual r:
    //
    //             d \rho(r)
    //   \phi(r) = ---------
    //                dr
    //
    // However, the loss functions we use are like Ceres solver ones, i.e. they compute the first
    // derivative of rho(s) wrt s = r^2 in rho[1] as:
    //
    //             d rho(s)
    //   \phi(s) = --------
    //                ds
    //
    // Therefore, we have the following equivalence:
    //
    //             d \rho(r)   d 0.5 * rho(r^2)         d rho(s)   d r^2
    //   \phi(r) = --------- = ---------------- = 0.5 * -------- * -----
    //                dr             dr                    ds       dr
    //
    //                           d rho(s)       d rho(s)
    //           = 0.5 * 2 * r * -------- = r * --------
    //                              ds             ds
    //
    // where \rho(r) = 0.5 * rho(r^2) is the inverse of rho(s) = 2 * \rho(sqrt(s)),
    // because s = r^2 and r = sqrt(s).
    std::transform(
      residuals_.begin(), residuals_.end(), std::back_inserter(influence),
      [&loss_function](const auto & r) {              // NOLINT(whitespace/braces)
        double rho[3];
        loss_function->Evaluate(r * r, rho);
        return r * rho[1];
      });               // NOLINT(whitespace/braces)

    return influence;
  }

  std::vector<double> weight(const ceres::LossFunction * loss_function) const
  {
    std::vector<double> weight;
    weight.reserve(residuals_.size());

    // The weight is defined as the influence divided by the residual r:
    //
    //   w(r) = \phi(r) / r
    //
    // Since the loss functions we use are like Ceres solver ones, we have:
    //
    //              d rho(s)       d rho(s)
    //   w(r) = r * -------- / r = --------
    //                 ds             ds
    //
    // That is, rho[1].
    std::transform(
      residuals_.begin(), residuals_.end(), std::back_inserter(weight),
      [&loss_function](const auto & r) {              // NOLINT(whitespace/braces)
        double rho[3];
        loss_function->Evaluate(r * r, rho);
        return rho[1];
      });               // NOLINT(whitespace/braces)

    return weight;
  }

  std::vector<double> secondDerivative(const ceres::LossFunction * loss_function) const
  {
    std::vector<double> second_derivative;
    second_derivative.reserve(residuals_.size());

    // The second derivative is defined as:
    //
    //   d^2 rho(s)
    //   ----------
    //      ds^2
    //
    // That is, rho[2].
    std::transform(
      residuals_.begin(), residuals_.end(), std::back_inserter(second_derivative),
      [&loss_function](const auto & r) {              // NOLINT(whitespace/braces)
        double rho[3];
        loss_function->Evaluate(r * r, rho);
        return rho[2];
      });               // NOLINT(whitespace/braces)

    return second_derivative;
  }

  const std::vector<double> & getResiduals() const
  {
    return residuals_;
  }

private:
  std::vector<double> residuals_;
};

class QwtLossPlot
{
public:
  QwtLossPlot(const std::vector<double> & residuals, const HSVColormap & colormap)
  : residuals_(QVector<double>(residuals.begin(), residuals.end()))
    , loss_evaluator_(residuals)
    , colormap_(colormap)
    , magnifier_(plot_.canvas())
    , zoomer_(plot_.canvas())
    , panner_(plot_.canvas())
  {
    // Allocate one curve per colormap entry:
    curves_.reserve(colormap_.size());

    // Set background to white:
    plot_.setCanvasBackground(Qt::white);

    // Add grid:
    grid_.setPen(Qt::lightGray, 0.0, Qt::DotLine);
    grid_.attach(&plot_);

    // Setup panner modifiers:
    panner_.setMouseButton(Qt::LeftButton, Qt::ControlModifier);

    // Add a legend:
    plot_.insertLegend(&legend_);
  }

  static std::string getName(const std::string & type)
  {
    return type.substr(11, type.size() - 15);
  }

  QwtPlotCurve * createCurve(const std::string & name, const std::vector<double> & values)
  {
    QwtPlotCurve * curve = new QwtPlotCurve(name.c_str());

    curve->setSamples(residuals_, QVector<double>(values.begin(), values.end()));

    curve->setPen(colormap_[curves_.size()]);
    curve->attach(&plot_);

    return curve;
  }

  void plotRho(const std::shared_ptr<fuse_core::Loss> & loss)
  {
    curves_.push_back(
      createCurve(
        getName(loss->type()),
        loss_evaluator_.rho(loss->lossFunction())));
  }

  void plotInfluence(const std::shared_ptr<fuse_core::Loss> & loss)
  {
    curves_.push_back(
      createCurve(
        getName(loss->type()),
        loss_evaluator_.influence(loss->lossFunction())));
  }

  void plotWeight(const std::shared_ptr<fuse_core::Loss> & loss)
  {
    curves_.push_back(
      createCurve(
        getName(loss->type()),
        loss_evaluator_.weight(loss->lossFunction())));
  }

  void plotSecondDerivative(const std::shared_ptr<fuse_core::Loss> & loss)
  {
    curves_.push_back(
      createCurve(
        getName(loss->type()),
        loss_evaluator_.secondDerivative(loss->lossFunction())));
  }

  void save(const std::string & filename)
  {
    QwtPlotRenderer renderer;
    renderer.renderDocument(&plot_, filename.c_str(), QSizeF(300, 200));
  }

  QwtPlot & plot()
  {
    return plot_;
  }

private:
  QVector<double> residuals_;
  LossEvaluator loss_evaluator_;

  HSVColormap colormap_;
  // We don't use an std::shared_ptr<QwtPlotCurve> because QwtPlot takes ownership of the curves
  // attached to it and deletes them on destruction
  std::vector<QwtPlotCurve *> curves_;

  QwtPlot plot_;
  QwtPlotGrid grid_;
  QwtLegend legend_;
  QwtPlotMagnifier magnifier_;
  QwtPlotZoomer zoomer_;
  QwtPlotPanner panner_;
};

}  // namespace fuse_loss

#endif  // FUSE_LOSS__QWT_LOSS_PLOT_HPP_
