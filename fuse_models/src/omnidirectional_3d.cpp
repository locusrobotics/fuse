/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, Giacomo Franchini
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
#include <Eigen/Dense>
#include <tf2/utils.h>

#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <fuse_core/async_motion_model.hpp>
#include <fuse_core/constraint.hpp>
#include <fuse_core/parameter.hpp>
#include <fuse_core/transaction.hpp>
#include <fuse_core/uuid.hpp>
#include <fuse_core/variable.hpp>
#include <fuse_models/common/sensor_proc.hpp>
#include <fuse_models/parameters/parameter_base.hpp>
#include <fuse_models/omnidirectional_3d.hpp>
#include <fuse_models/omnidirectional_3d_predict.hpp>
#include <fuse_models/omnidirectional_3d_state_kinematic_constraint.hpp>
#include <fuse_variables/acceleration_linear_3d_stamped.hpp>
#include <fuse_variables/orientation_3d_stamped.hpp>
#include <fuse_variables/position_3d_stamped.hpp>
#include <fuse_variables/stamped.hpp>
#include <fuse_variables/velocity_angular_3d_stamped.hpp>
#include <fuse_variables/velocity_linear_3d_stamped.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

// Register this motion model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(fuse_models::Omnidirectional3D, fuse_core::MotionModel)

namespace std
{

inline bool isfinite(const fuse_core::Vector3d & vector)
{
  return std::isfinite(vector.x()) && std::isfinite(vector.y() && std::isfinite(vector.z()));
}

inline bool isNormalized(const Eigen::Quaterniond & q)
{
  return std::sqrt(q.w() * q.w() + q.x() * q.x() + q.y() * q.y() + q.z() * q.z()) - 1.0 <
         Eigen::NumTraits<double>::dummy_precision();
}

std::string to_string(const fuse_core::Vector3d & vector)
{
  std::ostringstream oss;
  oss << vector;
  return oss.str();
}

std::string to_string(const Eigen::Quaterniond & quaternion)
{
  std::ostringstream oss;
  oss << quaternion;
  return oss.str();
}
}  // namespace std

namespace fuse_core
{

template<typename Derived>
inline void validateCovariance(
  const Eigen::DenseBase<Derived> & covariance,
  const double precision = Eigen::NumTraits<double>::dummy_precision())
{
  if (!fuse_core::isSymmetric(covariance, precision)) {
    throw std::runtime_error(
            "Non-symmetric partial covariance matrix\n" +
            fuse_core::to_string(covariance, Eigen::FullPrecision));
  }

  if (!fuse_core::isPositiveDefinite(covariance)) {
    throw std::runtime_error(
            "Non-positive-definite partial covariance matrix\n" +
            fuse_core::to_string(covariance, Eigen::FullPrecision));
  }
}

}  // namespace fuse_core

namespace fuse_models
{

Omnidirectional3D::Omnidirectional3D()
: fuse_core::AsyncMotionModel(1), // Thread count = 1 for local callback queue
  logger_(rclcpp::get_logger("uninitialized")),
  buffer_length_(rclcpp::Duration::max()),
  device_id_(fuse_core::uuid::NIL),
  timestamp_manager_(&Omnidirectional3D::generateMotionModel, this, rclcpp::Duration::max())
{
}

void Omnidirectional3D::print(std::ostream & stream) const
{
  stream << "state history:\n";
  for (const auto & state : state_history_) {
    stream << "- stamp: " << state.first.nanoseconds() << "\n";
    state.second.print(stream);
  }
}

void Omnidirectional3D::StateHistoryElement::print(std::ostream & stream) const
{
  stream << "  position uuid: " << position_uuid << "\n"
         << "  orientation uuid: " << orientation_uuid << "\n"
         << "  velocity linear uuid: " << vel_linear_uuid << "\n"
         << "  velocity angular uuid: " << vel_angular_uuid << "\n"
         << "  acceleration linear uuid: " << acc_linear_uuid << "\n"
         << "  position: " << position << "\n"
         << "  orientation: " << orientation << "\n"
         << "  velocity linear: " << vel_linear << "\n"
         << "  velocity angular: " << vel_angular << "\n"
         << "  acceleration linear: " << acc_linear << "\n";
}

void Omnidirectional3D::StateHistoryElement::validate() const
{
  if (!std::isfinite(position)) {
    throw std::runtime_error("Invalid position " + std::to_string(position));
  }
  if (!std::isNormalized(orientation)) {
    throw std::runtime_error("Invalid orientation " + std::to_string(orientation));
  }
  if (!std::isfinite(vel_linear)) {
    throw std::runtime_error("Invalid linear velocity " + std::to_string(vel_linear));
  }
  if (!std::isfinite(vel_angular)) {
    throw std::runtime_error("Invalid angular velocity " + std::to_string(vel_angular));
  }
  if (!std::isfinite(acc_linear)) {
    throw std::runtime_error("Invalid linear acceleration " + std::to_string(acc_linear));
  }
}

bool Omnidirectional3D::applyCallback(fuse_core::Transaction & transaction)
{
  // Use the timestamp manager to generate just the required motion model segments. The timestamp
  // manager, in turn, makes calls to the generateMotionModel() function.
  try {
    // Now actually generate the motion model segments
    timestamp_manager_.query(transaction, true); 
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM_THROTTLE(
      logger_, *clock_, 10.0 * 1000,
      "An error occurred while completing the motion model query. Error: " << e.what());
    return false;
  }
  return true;
}

void Omnidirectional3D::onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph)
{
  updateStateHistoryEstimates(*graph, state_history_, buffer_length_);
}

void Omnidirectional3D::initialize(
  fuse_core::node_interfaces::NodeInterfaces<ALL_FUSE_CORE_NODE_INTERFACES> interfaces,
  const std::string & name)
{
  interfaces_ = interfaces;
  fuse_core::AsyncMotionModel::initialize(interfaces, name);
}

void Omnidirectional3D::onInit()
{
  logger_ = interfaces_.get_node_logging_interface()->get_logger();
  clock_ = interfaces_.get_node_clock_interface()->get_clock();

  std::vector<double> process_noise_diagonal;
  process_noise_diagonal =
    fuse_core::getParam(
    interfaces_, fuse_core::joinParameterName(
      name_,
      "process_noise_diagonal"),
    process_noise_diagonal);

  if (process_noise_diagonal.size() != 15) {
    throw std::runtime_error("Process noise diagonal must be of length 15!");
  }

  process_noise_covariance_ = fuse_core::Vector15d(process_noise_diagonal.data()).asDiagonal();

  scale_process_noise_ =
    fuse_core::getParam(
    interfaces_, fuse_core::joinParameterName(
      name_,
      "scale_process_noise"),
    scale_process_noise_);
  velocity_linear_norm_min_ =
    fuse_core::getParam(
    interfaces_, fuse_core::joinParameterName(
      name_,
      "velocity_linear_norm_min"),
    velocity_linear_norm_min_);
  
  velocity_angular_norm_min_ =
    fuse_core::getParam(
    interfaces_, fuse_core::joinParameterName(
      name_,
      "velocity_angular_norm_min"),
    velocity_angular_norm_min_);

  disable_checks_ =
    fuse_core::getParam(
    interfaces_, fuse_core::joinParameterName(
      name_,
      "disable_checks"),
    disable_checks_);

  double buffer_length = 3.0;
  buffer_length =
    fuse_core::getParam(
    interfaces_, fuse_core::joinParameterName(
      name_,
      "buffer_length"),
    buffer_length);

  if (buffer_length < 0.0) {
    throw std::runtime_error(
            "Invalid negative buffer length of " + std::to_string(buffer_length) + " specified.");
  }

  buffer_length_ =
    (buffer_length ==
    0.0) ? rclcpp::Duration::max() : rclcpp::Duration::from_seconds(buffer_length);
  timestamp_manager_.bufferLength(buffer_length_);

  device_id_ = fuse_variables::loadDeviceId(interfaces_);
}

void Omnidirectional3D::onStart()
{
  timestamp_manager_.clear();
  state_history_.clear();
}

void Omnidirectional3D::generateMotionModel(
  const rclcpp::Time & beginning_stamp,
  const rclcpp::Time & ending_stamp,
  std::vector<fuse_core::Constraint::SharedPtr> & constraints,
  std::vector<fuse_core::Variable::SharedPtr> & variables)
{
  assert(
    beginning_stamp < ending_stamp ||
    (beginning_stamp == ending_stamp && state_history_.empty()));

  StateHistoryElement base_state;
  rclcpp::Time base_time{0, 0, RCL_ROS_TIME};

  // Find an entry that is > beginning_stamp
  // The entry that is <= will be the one before it
  auto base_state_pair_it = state_history_.upper_bound(beginning_stamp);
  if (base_state_pair_it == state_history_.begin()) {
    RCLCPP_WARN_STREAM_EXPRESSION(
      logger_, !state_history_.empty(),
      "Unable to locate a state in this history with stamp <= "
        << beginning_stamp.nanoseconds() << ". Variables will all be initialized to 0.");
    base_time = beginning_stamp;
  } else {
    --base_state_pair_it;
    base_time = base_state_pair_it->first;
    base_state = base_state_pair_it->second;
  }

  StateHistoryElement state1;
  // If the nearest state we had was before the beginning stamp, we need to project that state to
  // the beginning stamp
  if (base_time != beginning_stamp) {
    predict(
      base_state.position,
      base_state.orientation,
      base_state.vel_linear,
      base_state.vel_angular,
      base_state.acc_linear,
      (beginning_stamp - base_time).seconds(),
      state1.position,
      state1.orientation,
      state1.vel_linear,
      state1.vel_angular,
      state1.acc_linear);
  } else {
    state1 = base_state;
  }
  
  // If dt is zero, we only need to update the state history:
  const double dt = (ending_stamp - beginning_stamp).seconds();

  if (dt == 0.0) {
    state1.position_uuid = fuse_variables::Position3DStamped(beginning_stamp, device_id_).uuid();
    state1.orientation_uuid = fuse_variables::Orientation3DStamped(beginning_stamp, device_id_).uuid();
    state1.vel_linear_uuid =
      fuse_variables::VelocityLinear3DStamped(beginning_stamp, device_id_).uuid();
    state1.vel_angular_uuid =
      fuse_variables::VelocityAngular3DStamped(beginning_stamp, device_id_).uuid();
    state1.acc_linear_uuid =
      fuse_variables::AccelerationLinear3DStamped(beginning_stamp, device_id_).uuid();
    state_history_.emplace(beginning_stamp, std::move(state1));
    return;
  }

  // Now predict to get an initial guess for the state at the ending stamp
  StateHistoryElement state2;
  predict(
    state1.position,
    state1.orientation,
    state1.vel_linear,
    state1.vel_angular,
    state1.acc_linear,
    dt,
    state2.position,
    state2.orientation,
    state2.vel_linear,
    state2.vel_angular,
    state2.acc_linear);

    // Define the fuse variables required for this constraint
  auto position1 = fuse_variables::Position3DStamped::make_shared(beginning_stamp, device_id_);
  auto orientation1 = fuse_variables::Orientation3DStamped::make_shared(beginning_stamp, device_id_);
  auto velocity_linear1 = fuse_variables::VelocityLinear3DStamped::make_shared(
    beginning_stamp,
    device_id_);
  auto velocity_angular1 = fuse_variables::VelocityAngular3DStamped::make_shared(
    beginning_stamp,
    device_id_);
  auto acceleration_linear1 = fuse_variables::AccelerationLinear3DStamped::make_shared(
    beginning_stamp, device_id_);
  auto position2 = fuse_variables::Position3DStamped::make_shared(ending_stamp, device_id_);
  auto orientation2 = fuse_variables::Orientation3DStamped::make_shared(ending_stamp, device_id_);
  auto velocity_linear2 = fuse_variables::VelocityLinear3DStamped::make_shared(
    ending_stamp,
    device_id_);
  auto velocity_angular2 = fuse_variables::VelocityAngular3DStamped::make_shared(
    ending_stamp,
    device_id_);
  auto acceleration_linear2 = fuse_variables::AccelerationLinear3DStamped::make_shared(
    ending_stamp,
    device_id_);

  position1->data()[fuse_variables::Position3DStamped::X] = state1.position.x();
  position1->data()[fuse_variables::Position3DStamped::Y] = state1.position.y();
  position1->data()[fuse_variables::Position3DStamped::Z] = state1.position.z();

  orientation1->data()[fuse_variables::Orientation3DStamped::X] = state1.orientation.x();
  orientation1->data()[fuse_variables::Orientation3DStamped::Y] = state1.orientation.y();
  orientation1->data()[fuse_variables::Orientation3DStamped::Z] = state1.orientation.z();
  orientation1->data()[fuse_variables::Orientation3DStamped::W] = state1.orientation.w();
  
  velocity_linear1->data()[fuse_variables::VelocityLinear3DStamped::X] = state1.vel_linear.x();
  velocity_linear1->data()[fuse_variables::VelocityLinear3DStamped::Y] = state1.vel_linear.y();
  velocity_linear1->data()[fuse_variables::VelocityLinear3DStamped::Z] = state1.vel_linear.z();

  velocity_angular1->data()[fuse_variables::VelocityAngular3DStamped::ROLL]  = state1.vel_angular.x();
  velocity_angular1->data()[fuse_variables::VelocityAngular3DStamped::PITCH] = state1.vel_angular.y();
  velocity_angular1->data()[fuse_variables::VelocityAngular3DStamped::YAW]   = state1.vel_angular.z();

  acceleration_linear1->data()[fuse_variables::AccelerationLinear3DStamped::X] =
    state1.acc_linear.x();
  acceleration_linear1->data()[fuse_variables::AccelerationLinear3DStamped::Y] =
    state1.acc_linear.y();
  acceleration_linear1->data()[fuse_variables::AccelerationLinear3DStamped::Z] =
    state1.acc_linear.z();
  
  position2->data()[fuse_variables::Position3DStamped::X] = state2.position.x();
  position2->data()[fuse_variables::Position3DStamped::Y] = state2.position.y();
  position2->data()[fuse_variables::Position3DStamped::Z] = state2.position.z();

  orientation2->data()[fuse_variables::Orientation3DStamped::X] = state2.orientation.x();
  orientation2->data()[fuse_variables::Orientation3DStamped::Y] = state2.orientation.y();
  orientation2->data()[fuse_variables::Orientation3DStamped::Z] = state2.orientation.z();
  orientation2->data()[fuse_variables::Orientation3DStamped::W] = state2.orientation.w();
  
  velocity_linear2->data()[fuse_variables::VelocityLinear3DStamped::X] = state2.vel_linear.x();
  velocity_linear2->data()[fuse_variables::VelocityLinear3DStamped::Y] = state2.vel_linear.y();
  velocity_linear2->data()[fuse_variables::VelocityLinear3DStamped::Z] = state2.vel_linear.z();

  velocity_angular2->data()[fuse_variables::VelocityAngular3DStamped::ROLL]  = state2.vel_angular.x();
  velocity_angular2->data()[fuse_variables::VelocityAngular3DStamped::PITCH] = state2.vel_angular.y();
  velocity_angular2->data()[fuse_variables::VelocityAngular3DStamped::YAW]   = state2.vel_angular.z();

  acceleration_linear2->data()[fuse_variables::AccelerationLinear3DStamped::X] =
    state2.acc_linear.x();
  acceleration_linear2->data()[fuse_variables::AccelerationLinear3DStamped::Y] =
    state2.acc_linear.y();
  acceleration_linear2->data()[fuse_variables::AccelerationLinear3DStamped::Z] =
    state2.acc_linear.z();

  state1.position_uuid = position1->uuid();
  state1.orientation_uuid = orientation1->uuid();
  state1.vel_linear_uuid = velocity_linear1->uuid();
  state1.vel_angular_uuid = velocity_angular1->uuid();
  state1.acc_linear_uuid = acceleration_linear1->uuid();

  state2.position_uuid = position2->uuid();
  state2.orientation_uuid = orientation2->uuid();
  state2.vel_linear_uuid = velocity_linear2->uuid();
  state2.vel_angular_uuid = velocity_angular2->uuid();
  state2.acc_linear_uuid = acceleration_linear2->uuid();

  state_history_.emplace(beginning_stamp, std::move(state1));
  state_history_.emplace(ending_stamp, std::move(state2));

  // Scale process noise covariance pose by the norm of the current state twist
  auto process_noise_covariance = process_noise_covariance_;
  if (scale_process_noise_) {
    common::scaleProcessNoiseCovariance(
      process_noise_covariance, state1.vel_linear, state1.vel_angular,
      velocity_linear_norm_min_, velocity_angular_norm_min_);
  }

  // Validate
  process_noise_covariance *= dt;

  if (!disable_checks_) {
    try {
      validateMotionModel(state1, state2, process_noise_covariance);
    } catch (const std::runtime_error & ex) {
      RCLCPP_ERROR_STREAM_THROTTLE(
        logger_, *clock_, 10.0 * 1000,
        "Invalid '" << name_ << "' motion model: " << ex.what());
      return;
    }
  }

  // Create the constraints for this motion model segment
  auto constraint = fuse_models::Omnidirectional3DStateKinematicConstraint::make_shared(
    name(),
    *position1,
    *orientation1,
    *velocity_linear1,
    *velocity_angular1,
    *acceleration_linear1,
    *position2,
    *orientation2,
    *velocity_linear2,
    *velocity_angular2,
    *acceleration_linear2,
    process_noise_covariance);

  // Update the output variables
  constraints.push_back(constraint);
  variables.push_back(position1);
  variables.push_back(orientation1);
  variables.push_back(velocity_linear1);
  variables.push_back(velocity_angular1);
  variables.push_back(acceleration_linear1);
  variables.push_back(position2);
  variables.push_back(orientation2);
  variables.push_back(velocity_linear2);
  variables.push_back(velocity_angular2);
  variables.push_back(acceleration_linear2);
}

void Omnidirectional3D::updateStateHistoryEstimates(
  const fuse_core::Graph & graph,
  StateHistory & state_history,
  const rclcpp::Duration & buffer_length)
{
  if (state_history.empty()) {
    return;
  }

  // Compute the expiration time carefully, as ROS can't handle negative times
  const auto & ending_stamp = state_history.rbegin()->first;

  rclcpp::Time expiration_time;
  if (ending_stamp.seconds() > buffer_length.seconds()) {
    expiration_time = ending_stamp - buffer_length;
  } else {
    // NOTE(CH3): Uninitialized. But okay because it's just used for comparison.
    expiration_time = rclcpp::Time(0, 0, ending_stamp.get_clock_type());
  }

  // Remove state history elements before the expiration time.
  // Be careful to ensure that:
  //  - at least one entry remains at all times
  //  - the history covers *at least* until the expiration time. Longer is acceptable.
  auto expiration_iter = state_history.upper_bound(expiration_time);
  if (expiration_iter != state_history.begin()) {
    // expiration_iter points to the first element > expiration_time.
    // Back up one entry, to a point that is <= expiration_time
    state_history.erase(state_history.begin(), std::prev(expiration_iter));
  }

  // Update the states in the state history with information from the graph
  // If a state is not in the graph yet, predict the state in question from the closest previous
  // state
  for (auto current_iter = state_history.begin(); current_iter != state_history.end();
    ++current_iter)
  {
    const auto & current_stamp = current_iter->first;
    auto & current_state = current_iter->second;
        if (graph.variableExists(current_state.position_uuid) &&
      graph.variableExists(current_state.orientation_uuid) &&
      graph.variableExists(current_state.vel_linear_uuid) &&
      graph.variableExists(current_state.vel_angular_uuid) &&
      graph.variableExists(current_state.acc_linear_uuid))
    {
      // This pose does exist in the graph. Update it directly.
      const auto & position = graph.getVariable(current_state.position_uuid);
      const auto & orientation = graph.getVariable(current_state.orientation_uuid);
      const auto & vel_linear = graph.getVariable(current_state.vel_linear_uuid);
      const auto & vel_angular = graph.getVariable(current_state.vel_angular_uuid);
      const auto & acc_linear = graph.getVariable(current_state.acc_linear_uuid);

      current_state.position.x() = position.data()[fuse_variables::Position3DStamped::X];
      current_state.position.y() = position.data()[fuse_variables::Position3DStamped::Y];
      current_state.position.z() = position.data()[fuse_variables::Position3DStamped::Z];

      current_state.orientation.x() = orientation.data()[fuse_variables::Orientation3DStamped::X];
      current_state.orientation.y() = orientation.data()[fuse_variables::Orientation3DStamped::Y];
      current_state.orientation.z() = orientation.data()[fuse_variables::Orientation3DStamped::Z];
      current_state.orientation.w() = orientation.data()[fuse_variables::Orientation3DStamped::W];

      current_state.vel_linear.x() =
        vel_linear.data()[fuse_variables::VelocityLinear3DStamped::X];
      current_state.vel_linear.y() =
        vel_linear.data()[fuse_variables::VelocityLinear3DStamped::Y];
      current_state.vel_linear.z() =
        vel_linear.data()[fuse_variables::VelocityLinear3DStamped::Z];
      
      current_state.vel_angular.x() = 
        vel_angular.data()[fuse_variables::VelocityAngular3DStamped::ROLL];
      current_state.vel_angular.y() =
        vel_angular.data()[fuse_variables::VelocityAngular3DStamped::PITCH];
      current_state.vel_angular.z() =
        vel_angular.data()[fuse_variables::VelocityAngular3DStamped::YAW];

      current_state.acc_linear.x() =
        acc_linear.data()[fuse_variables::AccelerationLinear3DStamped::X];
      current_state.acc_linear.y() =
        acc_linear.data()[fuse_variables::AccelerationLinear3DStamped::Y];
      current_state.acc_linear.z() =
        acc_linear.data()[fuse_variables::AccelerationLinear3DStamped::Z];
    } else if (current_iter != state_history.begin()) {
      auto previous_iter = std::prev(current_iter);
      const auto & previous_stamp = previous_iter->first;
      const auto & previous_state = previous_iter->second;
      
      // This state is not in the graph yet, so we can't update/correct the value in our state
      // history. However, the state *before* this one may have been corrected (or one of its
      // predecessors may have been), so we can use that corrected value, along with our prediction
      // logic, to provide a more accurate update to this state.
      predict(
        previous_state.position,
        previous_state.orientation,
        previous_state.vel_linear,
        previous_state.vel_angular,
        previous_state.acc_linear,
        (current_stamp - previous_stamp).seconds(),
        current_state.position,
        current_state.orientation,
        current_state.vel_linear,
        current_state.vel_angular,
        current_state.acc_linear);
    }
  }
}

void Omnidirectional3D::validateMotionModel(
  const StateHistoryElement & state1, const StateHistoryElement & state2,
  const fuse_core::Matrix15d & process_noise_covariance)
{
  try {
    state1.validate();
  } catch (const std::runtime_error & ex) {
    throw std::runtime_error("Invalid state #1: " + std::string(ex.what()));
  }

  try {
    state2.validate();
  } catch (const std::runtime_error & ex) {
    throw std::runtime_error("Invalid state #2: " + std::string(ex.what()));
  }
  try {
    fuse_core::validateCovariance(process_noise_covariance);
  } catch (const std::runtime_error & ex) {
    throw std::runtime_error("Invalid process noise covariance: " + std::string(ex.what()));
  }
}

std::ostream & operator<<(std::ostream & stream, const Omnidirectional3D & omnidirectional_3d)
{
  omnidirectional_3d.print(stream);
  return stream;
}

}  // namespace fuse_models
