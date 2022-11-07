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
#include <fuse_models/unicycle_2d_predict.h>
#include <fuse_models/unicycle_2d_state_kinematic_constraint.h>
#include <fuse_models/unicycle_2d.h>
#include <fuse_models/common/sensor_proc.h>

#include <Eigen/Dense>
#include <fuse_core/async_motion_model.h>
#include <fuse_core/constraint.h>
#include <fuse_core/transaction.h>
#include <fuse_core/uuid.h>
#include <fuse_core/variable.h>
#include <fuse_variables/acceleration_linear_2d_stamped.h>
#include <fuse_variables/orientation_2d_stamped.h>
#include <fuse_variables/position_2d_stamped.h>
#include <fuse_variables/velocity_angular_2d_stamped.h>
#include <fuse_variables/velocity_linear_2d_stamped.h>
#include <fuse_variables/stamped.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf2/utils.h>

#include <stdexcept>
#include <string>
#include <utility>
#include <vector>


// Register this motion model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(fuse_models::Unicycle2D, fuse_core::MotionModel)

namespace std
{

inline bool isfinite(const tf2_2d::Vector2& vector)
{
  return std::isfinite(vector.x()) && std::isfinite(vector.y());
}

inline bool isfinite(const tf2_2d::Transform& transform)
{
  return std::isfinite(transform.x()) && std::isfinite(transform.y()) && std::isfinite(transform.yaw());
}

std::string to_string(const tf2_2d::Vector2& vector)
{
  std::ostringstream oss;
  oss << vector;
  return oss.str();
}

std::string to_string(const tf2_2d::Transform& transform)
{
  std::ostringstream oss;
  oss << transform;
  return oss.str();
}

}  // namespace std

namespace fuse_core
{

template <typename Derived>
inline void validateCovariance(const Eigen::DenseBase<Derived>& covariance,
                               const double precision = Eigen::NumTraits<double>::dummy_precision())
{
  if (!fuse_core::isSymmetric(covariance, precision))
  {
    throw std::runtime_error("Non-symmetric partial covariance matrix\n" +
                             fuse_core::to_string(covariance, Eigen::FullPrecision));
  }

  if (!fuse_core::isPositiveDefinite(covariance))
  {
    throw std::runtime_error("Non-positive-definite partial covariance matrix\n" +
                             fuse_core::to_string(covariance, Eigen::FullPrecision));
  }
}

}  // namespace fuse_core

namespace fuse_models
{

Unicycle2D::Unicycle2D() :
  fuse_core::AsyncMotionModel(1),
  buffer_length_(rclcpp::Duration::max()),
  device_id_(fuse_core::uuid::NIL),
  timestamp_manager_(&Unicycle2D::generateMotionModel, this, rclcpp::Duration::max())
{
}

void Unicycle2D::print(std::ostream& stream) const
{
  stream << "state history:\n";
  for (const auto& state : state_history_)
  {
    stream << "- stamp: " << state.first << "\n";
    state.second.print(stream);
  }
}

void Unicycle2D::StateHistoryElement::print(std::ostream& stream) const
{
  stream << "  position uuid: " << position_uuid << "\n"
         << "  yaw uuid: " << yaw_uuid << "\n"
         << "  velocity linear uuid: " << vel_linear_uuid << "\n"
         << "  velocity yaw uuid: " << vel_yaw_uuid << "\n"
         << "  acceleration linear uuid: " << acc_linear_uuid << "\n"
         << "  pose: " << pose << "\n"
         << "  velocity linear: " << velocity_linear << "\n"
         << "  velocity yaw: " << velocity_yaw << "\n"
         << "  acceleration linear: " << acceleration_linear << "\n";
}

void Unicycle2D::StateHistoryElement::validate() const
{
  if (!std::isfinite(pose))
  {
    throw std::runtime_error("Invalid pose " + std::to_string(pose));
  }

  if (!std::isfinite(velocity_linear))
  {
    throw std::runtime_error("Invalid linear velocity " + std::to_string(velocity_linear));
  }

  if (!std::isfinite(velocity_yaw))
  {
    throw std::runtime_error("Invalid yaw velocity " + std::to_string(velocity_yaw));
  }

  if (!std::isfinite(acceleration_linear))
  {
    throw std::runtime_error("Invalid linear acceleration " + std::to_string(acceleration_linear));
  }
}

bool Unicycle2D::applyCallback(fuse_core::Transaction& transaction)
{
  // Use the timestamp manager to generate just the required motion model segments. The timestamp manager, in turn,
  // makes calls to the generateMotionModel() function.
  try
  {
    // Now actually generate the motion model segments
    timestamp_manager_.query(transaction, true);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 10.0 * 1000,
                                 "An error occurred while completing the motion model query. Error: " << e.what());
    return false;
  }
  return true;
}

void Unicycle2D::onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph)
{
  updateStateHistoryEstimates(*graph, state_history_, buffer_length_);
}

void Unicycle2D::onInit()
{
  std::vector<double> process_noise_diagonal;
  private_node_handle_.param("process_noise_diagonal", process_noise_diagonal, process_noise_diagonal);

  if (process_noise_diagonal.size() != 8)
  {
    throw std::runtime_error("Process noise diagonal must be of length 8!");
  }

  process_noise_covariance_ = fuse_core::Vector8d(process_noise_diagonal.data()).asDiagonal();

  private_node_handle_.param("scale_process_noise", scale_process_noise_, scale_process_noise_);
  private_node_handle_.param("velocity_norm_min", velocity_norm_min_, velocity_norm_min_);

  private_node_handle_.param("disable_checks", disable_checks_, disable_checks_);

  double buffer_length = 3.0;
  private_node_handle_.param("buffer_length", buffer_length, buffer_length);

  if (buffer_length < 0.0)
  {
    throw std::runtime_error("Invalid negative buffer length of " + std::to_string(buffer_length) + " specified.");
  }

  buffer_length_ = (buffer_length == 0.0) ? rclcpp::Duration::max() : rclcpp::Duration::from_seconds(buffer_length);
  timestamp_manager_.bufferLength(buffer_length_);

  device_id_ = fuse_variables::loadDeviceId(private_node_handle_);
}

void Unicycle2D::onStart()
{
  timestamp_manager_.clear();
  state_history_.clear();
}

void Unicycle2D::generateMotionModel(
  const rclcpp::Time& beginning_stamp,
  const rclcpp::Time& ending_stamp,
  std::vector<fuse_core::Constraint::SharedPtr>& constraints,
  std::vector<fuse_core::Variable::SharedPtr>& variables)
{
  assert(beginning_stamp < ending_stamp || (beginning_stamp == ending_stamp && state_history_.empty()));

  StateHistoryElement base_state;
  rclcpp::Time base_time;

  // Find an entry that is > beginning_stamp
  // The entry that is <= will be the one before it
  auto base_state_pair_it = state_history_.upper_bound(beginning_stamp);
  if (base_state_pair_it == state_history_.begin())
  {
    RCLCPP_WARN_STREAM_EXPRESSION(node_->get_logger(), !state_history_.empty(),
                                  "UnicycleModel", "Unable to locate a state in this history with stamp <= "
                                  << beginning_stamp << ". Variables will all be initialized to 0.");
    base_time = beginning_stamp;
  }
  else
  {
    --base_state_pair_it;
    base_time = base_state_pair_it->first;
    base_state = base_state_pair_it->second;
  }

  StateHistoryElement state1;

  // If the nearest state we had was before the beginning stamp, we need to project that state to the beginning stamp
  if (base_time != beginning_stamp)
  {
    predict(
      base_state.pose,
      base_state.velocity_linear,
      base_state.velocity_yaw,
      base_state.acceleration_linear,
      (beginning_stamp - base_time).seconds(),
      state1.pose,
      state1.velocity_linear,
      state1.velocity_yaw,
      state1.acceleration_linear);
  }
  else
  {
    state1 = base_state;
  }

  // If dt is zero, we only need to update the state history:
  const double dt = (ending_stamp - beginning_stamp).seconds();

  if (dt == 0.0)
  {
    state1.position_uuid = fuse_variables::Position2DStamped(beginning_stamp, device_id_).uuid();
    state1.yaw_uuid = fuse_variables::Orientation2DStamped(beginning_stamp, device_id_).uuid();
    state1.vel_linear_uuid = fuse_variables::VelocityLinear2DStamped(beginning_stamp, device_id_).uuid();
    state1.vel_yaw_uuid = fuse_variables::VelocityAngular2DStamped(beginning_stamp, device_id_).uuid();
    state1.acc_linear_uuid = fuse_variables::AccelerationLinear2DStamped(beginning_stamp, device_id_).uuid();

    state_history_.emplace(beginning_stamp, std::move(state1));

    return;
  }

  // Now predict to get an initial guess for the state at the ending stamp
  StateHistoryElement state2;
  predict(
    state1.pose,
    state1.velocity_linear,
    state1.velocity_yaw,
    state1.acceleration_linear,
    dt,
    state2.pose,
    state2.velocity_linear,
    state2.velocity_yaw,
    state2.acceleration_linear);

  // Define the fuse variables required for this constraint
  auto position1 = fuse_variables::Position2DStamped::make_shared(beginning_stamp, device_id_);
  auto yaw1 = fuse_variables::Orientation2DStamped::make_shared(beginning_stamp, device_id_);
  auto velocity_linear1 = fuse_variables::VelocityLinear2DStamped::make_shared(beginning_stamp, device_id_);
  auto velocity_yaw1 = fuse_variables::VelocityAngular2DStamped::make_shared(beginning_stamp, device_id_);
  auto acceleration_linear1 = fuse_variables::AccelerationLinear2DStamped::make_shared(beginning_stamp, device_id_);
  auto position2 = fuse_variables::Position2DStamped::make_shared(ending_stamp, device_id_);
  auto yaw2 = fuse_variables::Orientation2DStamped::make_shared(ending_stamp, device_id_);
  auto velocity_linear2 = fuse_variables::VelocityLinear2DStamped::make_shared(ending_stamp, device_id_);
  auto velocity_yaw2 = fuse_variables::VelocityAngular2DStamped::make_shared(ending_stamp, device_id_);
  auto acceleration_linear2 = fuse_variables::AccelerationLinear2DStamped::make_shared(ending_stamp, device_id_);

  position1->data()[fuse_variables::Position2DStamped::X] = state1.pose.x();
  position1->data()[fuse_variables::Position2DStamped::Y] = state1.pose.y();
  yaw1->data()[fuse_variables::Orientation2DStamped::YAW] = state1.pose.yaw();
  velocity_linear1->data()[fuse_variables::VelocityLinear2DStamped::X] = state1.velocity_linear.x();
  velocity_linear1->data()[fuse_variables::VelocityLinear2DStamped::Y] = state1.velocity_linear.y();
  velocity_yaw1->data()[fuse_variables::VelocityAngular2DStamped::YAW] = state1.velocity_yaw;
  acceleration_linear1->data()[fuse_variables::AccelerationLinear2DStamped::X] = state1.acceleration_linear.x();
  acceleration_linear1->data()[fuse_variables::AccelerationLinear2DStamped::Y] = state1.acceleration_linear.y();
  position2->data()[fuse_variables::Position2DStamped::X] = state2.pose.x();
  position2->data()[fuse_variables::Position2DStamped::Y] = state2.pose.y();
  yaw2->data()[fuse_variables::Orientation2DStamped::YAW] = state2.pose.yaw();
  velocity_linear2->data()[fuse_variables::VelocityLinear2DStamped::X] = state2.velocity_linear.x();
  velocity_linear2->data()[fuse_variables::VelocityLinear2DStamped::Y] = state2.velocity_linear.y();
  velocity_yaw2->data()[fuse_variables::VelocityAngular2DStamped::YAW] = state2.velocity_yaw;
  acceleration_linear2->data()[fuse_variables::AccelerationLinear2DStamped::X] = state2.acceleration_linear.x();
  acceleration_linear2->data()[fuse_variables::AccelerationLinear2DStamped::Y] = state2.acceleration_linear.y();

  state1.position_uuid = position1->uuid();
  state1.yaw_uuid = yaw1->uuid();
  state1.vel_linear_uuid = velocity_linear1->uuid();
  state1.vel_yaw_uuid = velocity_yaw1->uuid();
  state1.acc_linear_uuid = acceleration_linear1->uuid();
  state2.position_uuid = position2->uuid();
  state2.yaw_uuid = yaw2->uuid();
  state2.vel_linear_uuid = velocity_linear2->uuid();
  state2.vel_yaw_uuid = velocity_yaw2->uuid();
  state2.acc_linear_uuid = acceleration_linear2->uuid();

  state_history_.emplace(beginning_stamp, std::move(state1));
  state_history_.emplace(ending_stamp, std::move(state2));

  // Scale process noise covariance pose by the norm of the current state twist
  auto process_noise_covariance = process_noise_covariance_;
  if (scale_process_noise_)
  {
    common::scaleProcessNoiseCovariance(process_noise_covariance, state1.velocity_linear, state1.velocity_yaw,
                                        velocity_norm_min_);
  }

  // Validate
  process_noise_covariance *= dt;

  if (!disable_checks_)
  {
    try
    {
      validateMotionModel(state1, state2, process_noise_covariance);
    }
    catch (const std::runtime_error& ex)
    {
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 10.0 * 1000,
                                   "Invalid '" << name_ << "' motion model: " << ex.what());
      return;
    }
  }

  // Create the constraints for this motion model segment
  auto constraint = fuse_models::Unicycle2DStateKinematicConstraint::make_shared(
    name(),
    *position1,
    *yaw1,
    *velocity_linear1,
    *velocity_yaw1,
    *acceleration_linear1,
    *position2,
    *yaw2,
    *velocity_linear2,
    *velocity_yaw2,
    *acceleration_linear2,
    process_noise_covariance);

  // Update the output variables
  constraints.push_back(constraint);
  variables.push_back(position1);
  variables.push_back(yaw1);
  variables.push_back(velocity_linear1);
  variables.push_back(velocity_yaw1);
  variables.push_back(acceleration_linear1);
  variables.push_back(position2);
  variables.push_back(yaw2);
  variables.push_back(velocity_linear2);
  variables.push_back(velocity_yaw2);
  variables.push_back(acceleration_linear2);
}

void Unicycle2D::updateStateHistoryEstimates(
  const fuse_core::Graph& graph,
  StateHistory& state_history,
  const rclcpp::Duration& buffer_length)
{
  if (state_history.empty())
  {
    return;
  }

  // Compute the expiration time carefully, as ROS can't handle negative times
  const auto& ending_stamp = state_history.rbegin()->first;

  if (ending_stamp.seconds() > buffer_length.seconds()) {
    auto expiration_time = ending_stamp - buffer_length;
  } else {
    // NOTE(CH3): Uninitialized. But okay because it's just used for comparison.
    auto expiration_time = rclcpp::Time(0, 0, ending_stamp.get_clock_type);
  }

  // Remove state history elements before the expiration time.
  // Be careful to ensure that:
  //  - at least one entry remains at all times
  //  - the history covers *at least* until the expiration time. Longer is acceptable.
  auto expiration_iter = state_history.upper_bound(expiration_time);
  if (expiration_iter != state_history.begin())
  {
    // expiration_iter points to the first element > expiration_time.
    // Back up one entry, to a point that is <= expiration_time
    state_history.erase(state_history.begin(), std::prev(expiration_iter));
  }

  // Update the states in the state history with information from the graph
  // If a state is not in the graph yet, predict the state in question from the closest previous state
  for (auto current_iter = state_history.begin(); current_iter != state_history.end(); ++current_iter)
  {
    const auto& current_stamp = current_iter->first;
    auto& current_state = current_iter->second;
    if (graph.variableExists(current_state.position_uuid) &&
        graph.variableExists(current_state.yaw_uuid) &&
        graph.variableExists(current_state.vel_linear_uuid) &&
        graph.variableExists(current_state.vel_yaw_uuid) &&
        graph.variableExists(current_state.acc_linear_uuid))
    {
      // This pose does exist in the graph. Update it directly.
      const auto& position = graph.getVariable(current_state.position_uuid);
      const auto& yaw = graph.getVariable(current_state.yaw_uuid);
      const auto& vel_linear = graph.getVariable(current_state.vel_linear_uuid);
      const auto& vel_yaw = graph.getVariable(current_state.vel_yaw_uuid);
      const auto& acc_linear = graph.getVariable(current_state.acc_linear_uuid);

      current_state.pose.setX(position.data()[fuse_variables::Position2DStamped::X]);
      current_state.pose.setY(position.data()[fuse_variables::Position2DStamped::Y]);
      current_state.pose.setAngle(yaw.data()[fuse_variables::Orientation2DStamped::YAW]);
      current_state.velocity_linear.setX(vel_linear.data()[fuse_variables::VelocityLinear2DStamped::X]);
      current_state.velocity_linear.setY(vel_linear.data()[fuse_variables::VelocityLinear2DStamped::Y]);
      current_state.velocity_yaw = vel_yaw.data()[fuse_variables::VelocityAngular2DStamped::YAW];
      current_state.acceleration_linear.setX(acc_linear.data()[fuse_variables::AccelerationLinear2DStamped::X]);
      current_state.acceleration_linear.setY(acc_linear.data()[fuse_variables::AccelerationLinear2DStamped::Y]);
    }
    else if (current_iter != state_history.begin())
    {
      auto previous_iter = std::prev(current_iter);
      const auto& previous_stamp = previous_iter->first;
      const auto& previous_state = previous_iter->second;

      // This state is not in the graph yet, so we can't update/correct the value in our state history. However, the
      // state *before* this one may have been corrected (or one of its predecessors may have been), so we can use
      // that corrected value, along with our prediction logic, to provide a more accurate update to this state.
      predict(
        previous_state.pose,
        previous_state.velocity_linear,
        previous_state.velocity_yaw,
        previous_state.acceleration_linear,
        (current_stamp - previous_stamp).seconds(),
        current_state.pose,
        current_state.velocity_linear,
        current_state.velocity_yaw,
        current_state.acceleration_linear);
    }
  }
}

void Unicycle2D::validateMotionModel(const StateHistoryElement& state1, const StateHistoryElement& state2,
                                     const fuse_core::Matrix8d& process_noise_covariance)
{
  try
  {
    state1.validate();
  }
  catch (const std::runtime_error& ex)
  {
    throw std::runtime_error("Invalid state #1: " + std::string(ex.what()));
  }

  try
  {
    state2.validate();
  }
  catch (const std::runtime_error& ex)
  {
    throw std::runtime_error("Invalid state #2: " + std::string(ex.what()));
  }

  try
  {
    fuse_core::validateCovariance(process_noise_covariance);
  }
  catch (const std::runtime_error& ex)
  {
    throw std::runtime_error("Invalid process noise covariance: " + std::string(ex.what()));
  }
}

std::ostream& operator<<(std::ostream& stream, const Unicycle2D& unicycle_2d)
{
  unicycle_2d.print(stream);
  return stream;
}

}  // namespace fuse_models
