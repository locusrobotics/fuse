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
#include <fuse_test/dummy_sensor.h>

#include <fuse_constraints/dummy_constraint.h>
#include <fuse_core/transaction.h>
#include <fuse_variables/dummy_variable.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <string>


// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(fuse_test::DummySensor, fuse_core::SensorModel)

namespace fuse_test
{

DummySensor::DummySensor() :
  fuse_core::AsyncSensorModel(1)
{
}

void DummySensor::onInit()
{
  timer_ = node_handle_.createTimer(ros::Duration(1.0), &DummySensor::timerCallback, this, false, false);
}

void DummySensor::onStart()
{
  timer_.start();
}

void DummySensor::onStop()
{
  timer_.stop();
}

void DummySensor::timerCallback(const ros::TimerEvent& /* event */)
{
  // Create a dummy variable
  auto stamp = ros::Time::now();
  auto quest = std::string("Find the answer to life, the universe, and everything");
  auto variable = fuse_variables::DummyVariable::make_shared(stamp, quest);
  variable->a() = 6;
  variable->b() = 9;

  // Create a dummy constraint
  auto mean = fuse_core::Vector2d();
  mean << 4., 2.;
  auto cov = fuse_core::Matrix2d();
  cov << 1.0, 0.0, 0.0, 1.0;
  auto constraint = fuse_constraints::DummyConstraint::make_shared(*variable, mean, cov);

  // Create and send the transaction
  auto transaction = fuse_core::Transaction::make_shared();
  transaction->stamp(stamp);
  transaction->addInvolvedStamp(stamp);
  transaction->addVariable(variable);
  transaction->addConstraint(constraint);
  sendTransaction(transaction);
}

}  // namespace fuse_test
