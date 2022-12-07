/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Clearpath Robotics
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
#include <QApplication>

#include <iostream>
#include <memory>
#include <vector>

#include <fuse_loss/arctan_loss.hpp>
#include <fuse_loss/cauchy_loss.hpp>
#include <fuse_loss/dcs_loss.hpp>
#include <fuse_loss/fair_loss.hpp>
#include <fuse_loss/geman_mcclure_loss.hpp>
#include <fuse_loss/huber_loss.hpp>
#include <fuse_loss/qwt_loss_plot.hpp>
#include <fuse_loss/softlone_loss.hpp>
#include <fuse_loss/tolerant_loss.hpp>
#include <fuse_loss/trivial_loss.hpp>
#include <fuse_loss/tukey_loss.hpp>
#include <fuse_loss/welsch_loss.hpp>

class QwtLossPlotTest : public testing::Test
{
public:
  QwtLossPlotTest()
  {
    // Generate samples:
    const double step{0.01};
    const size_t half_samples{1000};
    const double x_min = -(half_samples * step);

    const size_t samples{2 * half_samples + 1};

    residuals.reserve(samples);

    for (size_t i = 0; i < samples; ++i) {
      residuals.push_back(x_min + i * step);
    }
  }

  std::vector<double> residuals;
};

TEST_F(QwtLossPlotTest, PlotLossQt)
{
  // Create losses
  std::vector<std::shared_ptr<fuse_core::Loss>> losses{{   // NOLINT(whitespace/braces)
    std::make_shared<fuse_loss::ArctanLoss>(),
    std::make_shared<fuse_loss::CauchyLoss>(),
    std::make_shared<fuse_loss::DCSLoss>(),
    std::make_shared<fuse_loss::FairLoss>(),
    std::make_shared<fuse_loss::GemanMcClureLoss>(),
    std::make_shared<fuse_loss::HuberLoss>(),
    std::make_shared<fuse_loss::SoftLOneLoss>(),
    std::make_shared<fuse_loss::TolerantLoss>(),
    std::make_shared<fuse_loss::TrivialLoss>(),
    std::make_shared<fuse_loss::TukeyLoss>(),
    std::make_shared<fuse_loss::WelschLoss>()
  }};

  // Create a Qt application:
  int argc = 0;
  QApplication app(argc, nullptr);

  auto palette = app.palette();
  palette.setColor(QPalette::Window, Qt::white);
  app.setPalette(palette);

  // Create a loss plot for the rho function:
  fuse_loss::HSVColormap colormap(losses.size());
  fuse_loss::QwtLossPlot rho_loss_plot(residuals, colormap);

  auto & plot = rho_loss_plot.plot();
  plot.setTitle("rho function");
  plot.setAxisTitle(QwtPlot::xBottom, "r");
  plot.setAxisTitle(QwtPlot::yLeft, "rho(r)");
  plot.setAxisScale(QwtPlot::yLeft, 0.0, 15.0);

  // Create a curve for each loss rho function:
  for (const auto & loss : losses) {
    rho_loss_plot.plotRho(loss);
  }

  plot.replot();

  // Create a loss plot for the influence function:
  fuse_loss::QwtLossPlot influence_loss_plot(residuals, colormap);

  auto & influence_plot = influence_loss_plot.plot();
  influence_plot.setTitle("influence");

  influence_plot.setAxisTitle(QwtPlot::xBottom, "r");
  influence_plot.setAxisTitle(QwtPlot::yLeft, "phi(r) = r * rho'(r)");
  influence_plot.setAxisScale(QwtPlot::yLeft, -3.0, 3.0);

  // Create a curve for each loss rho function:
  for (const auto & loss : losses) {
    influence_loss_plot.plotInfluence(loss);
  }

  influence_plot.replot();

  // Create a loss plot for the weight function:
  fuse_loss::QwtLossPlot weight_loss_plot(residuals, colormap);

  auto & weight_plot = weight_loss_plot.plot();
  weight_plot.setTitle("weight");

  weight_plot.setAxisTitle(QwtPlot::xBottom, "r");
  weight_plot.setAxisTitle(QwtPlot::yLeft, "w(r) = rho'(r)");
  weight_plot.setAxisScale(QwtPlot::yLeft, 0.0, 1.5);

  // Create a curve for each loss rho function:
  for (const auto & loss : losses) {
    weight_loss_plot.plotWeight(loss);
  }

  weight_plot.replot();

  // Create a loss plot for the second derivative function:
  fuse_loss::QwtLossPlot second_derivative_loss_plot(residuals, colormap);

  auto & second_derivative_plot = second_derivative_loss_plot.plot();
  second_derivative_plot.setTitle("2nd derivative");

  second_derivative_plot.setAxisTitle(QwtPlot::xBottom, "r");
  second_derivative_plot.setAxisTitle(QwtPlot::yLeft, "rho''(r)");
  second_derivative_plot.setAxisScale(QwtPlot::yLeft, -0.15, 0.15);

  // Create a curve for each loss rho function:
  for (const auto & loss : losses) {
    second_derivative_loss_plot.plotSecondDerivative(loss);
  }

  second_derivative_plot.replot();

#ifdef INTERACTIVE_TESTS
  plot.show();
  influence_plot.show();
  weight_plot.show();
  second_derivative_plot.show();
#endif

  // Save as an SVG image, that can be converted to PNG with (e.g. for the weight function SVG image
  // file):
  //
  //   inkscape -z -e weight.png weight.svg
  //
  // In principle, we could use ImageMagick as well, but it might fail:
  //
  //   convert weight.svg weight.png
  rho_loss_plot.save("rho.svg");
  influence_loss_plot.save("influence.svg");
  weight_loss_plot.save("weight.svg");
  second_derivative_loss_plot.save("second_derivative.svg");

#ifdef INTERACTIVE_TESTS
  // Run application:
  // NOTE(CH3): This will block indefinitely until the test windows are closed! Since the tests are
  //            meant to be interactive you MUST close the windows for them to pass!!
  std::cout << "Interactive test active. If test does a timeout, and you did not manually close the"
            << "windows that popped up, the timeout is expected!" << std::endl;
  app.exec();
#endif
}
