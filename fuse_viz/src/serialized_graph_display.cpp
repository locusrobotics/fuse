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

#ifndef Q_MOC_RUN
#include <OgreBillboardSet.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/parse_color.hpp>
#include <rviz_common/properties/property.hpp>

#endif  // Q_MOC_RUN

#include <fuse_constraints/relative_pose_2d_stamped_constraint.hpp>
#include <fuse_core/graph.hpp>
#include <fuse_core/uuid.hpp>
#include <fuse_variables/orientation_2d_stamped.hpp>
#include <fuse_variables/position_2d_stamped.hpp>
#include <fuse_viz/pose_2d_stamped_property.hpp>
#include <fuse_viz/pose_2d_stamped_visual.hpp>
#include <fuse_viz/relative_pose_2d_stamped_constraint_property.hpp>
#include <fuse_viz/relative_pose_2d_stamped_constraint_visual.hpp>
#include <fuse_viz/serialized_graph_display.hpp>

namespace fuse_viz
{

using rviz_common::properties::BoolProperty;

SerializedGraphDisplay::SerializedGraphDisplay()
{
  show_variables_property_ =
    new BoolProperty(
    "Variables", true, "The list of all variables.", this,
    SLOT(updateShowVariables()));

  variable_property_ = new Pose2DStampedProperty(
    "pose_2d", true,
    "Pose2DStamped (fuse_variables::Position2DStamped + "
    "fuse_variables::Orientation2DStamped) variable.",
    show_variables_property_, SLOT(queueRender()), this);

  show_constraints_property_ = new BoolProperty(
    "Constraints", true, "The list of all constraints by source.", this,
    SLOT(updateShowConstraints()));
}

SerializedGraphDisplay::~SerializedGraphDisplay()
{
  if (initialized()) {
    clear();

    root_node_->removeAndDestroyAllChildren();
    scene_manager_->destroySceneNode(root_node_->getName());
  }
}

void SerializedGraphDisplay::reset()
{
  MFDClass::reset();
}

void SerializedGraphDisplay::onInitialize()
{
  MFDClass::onInitialize();

  root_node_ = scene_node_->createChildSceneNode();
}

void SerializedGraphDisplay::onEnable()
{
  MFDClass::onEnable();

  root_node_->setVisible(true);

  variable_property_->updateVisibility();

  for (auto & entry : constraint_source_properties_) {
    entry.second->updateVisibility();
  }
}

void SerializedGraphDisplay::onDisable()
{
  MFDClass::onDisable();

  root_node_->setVisible(false);
}

void SerializedGraphDisplay::load(const rviz_common::Config & config)
{
  MFDClass::load(config);

  // Cache constraint config for each source in order to apply it when the
  // RelativePose2DStampedConstraintProperty is created the first time a constraint of each source
  // is present in the graph:
  const auto constraints_config = config.mapGetChild("Constraints");

  for (rviz_common::Config::MapIterator iter = constraints_config.mapIterator(); iter.isValid();
    iter.advance())
  {
    constraint_source_configs_[iter.currentKey().toStdString()] = iter.currentChild();
  }
}

void SerializedGraphDisplay::updateShowVariables()
{
  variable_property_->setBool(show_variables_property_->getBool());
}

void SerializedGraphDisplay::updateShowConstraints()
{
  const auto visible = show_constraints_property_->getBool();

  for (auto & entry : constraint_source_properties_) {
    entry.second->setBool(visible);
  }
}

void SerializedGraphDisplay::clear()
{
  constraint_visuals_.clear();

  delete variable_property_;

  for (auto & entry : constraint_source_properties_) {
    delete entry.second;
  }
  constraint_source_properties_.clear();

  variables_changed_map_.clear();
  constraints_changed_map_.clear();
}

void SerializedGraphDisplay::processMessage(fuse_msgs::msg::SerializedGraph::ConstSharedPtr msg)
{
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(msg->header, position, orientation)) {
    RCLCPP_DEBUG_STREAM(
      rclcpp::get_logger("fuse"),
      "Error transforming from frame '" << msg->header.frame_id << "' to frame '"
                                        << qPrintable(fixed_frame_) << "'");
  }

  root_node_->setPosition(position);
  root_node_->setOrientation(orientation);

  for (auto & entry : variables_changed_map_) {
    entry.second = false;
  }

  for (auto & entry : constraints_changed_map_) {
    entry.second = false;
  }

  const auto graph = graph_deserializer_.deserialize(msg);

  for (const auto & variable : graph->getVariables()) {
    const auto orientation = dynamic_cast<const fuse_variables::Orientation2DStamped *>(&variable);
    if (!orientation) {
      continue;
    }

    const auto position_uuid = fuse_variables::Position2DStamped(
      orientation->stamp(), orientation->deviceId()).uuid();
    if (!graph->variableExists(position_uuid)) {
      continue;
    }

    const auto position =
      dynamic_cast<const fuse_variables::Position2DStamped *>(&graph->getVariable(position_uuid));

    variable_property_->createAndInsertOrUpdateVisual(
      scene_manager_, root_node_, *position,
      *orientation);

    variables_changed_map_[position_uuid] = true;
  }

  for (const auto & constraint : graph->getConstraints()) {
    const auto relative_pose =
      dynamic_cast<const fuse_constraints::RelativePose2DStampedConstraint *>(&constraint);
    if (!relative_pose) {
      continue;
    }

    const auto constraint_uuid = constraint.uuid();
    const auto & constraint_source = constraint.source();

    if (source_color_map_.find(constraint_source) == source_color_map_.end()) {
      // Generate hue color automatically based on the number of sources including the new one (n)
      // The hue is computed in such a way that the (dynamic) colormap is always well spread along
      // the spectrum. This is achieved by traversing a virtual complete binary tree in breadth-
      // first order. Each node represents a sampling position in the hue interval (0, 1) based on
      // the current level and the number of nodes in that level (m)
      const auto n = source_color_map_.size() + 1;
      const size_t level = std::floor(std::log2(n));
      const auto m = n + 1 - std::pow(2, level);
      const auto hue = (2 * (m - 1) + 1) / std::pow(2, level + 1);

      auto & source_color = source_color_map_[constraint_source];
      source_color.setHSB(hue, 1.0, 1.0);

      // Insert constraint sorted alphabetically:
      const auto description = constraint_source + ' ' + constraint.type() + " constraint.";

      const auto constraint_source_property = new RelativePose2DStampedConstraintProperty(
        QString::fromStdString(constraint_source), true, QString::fromStdString(
          description), nullptr,
        SLOT(queueRender()), this);

      const auto result = constraint_source_properties_.insert(
        {constraint_source, constraint_source_property});      // NOLINT(whitespace/braces)

      if (!result.second) {
        delete constraint_source_property;

        throw std::runtime_error("Failed to insert " + description);
      }

      show_constraints_property_->addChild(
        constraint_source_property,
        std::distance(constraint_source_properties_.begin(), result.first));

      if (constraint_source_configs_.find(constraint_source) == constraint_source_configs_.end()) {
        constraint_source_properties_[constraint_source]->setColor(
          rviz_common::properties::ogreToQt(
            source_color));
      } else {
        constraint_source_properties_[constraint_source]->load(
          constraint_source_configs_[
            constraint_source]);
      }
    }

    if (constraint_visuals_.find(constraint_uuid) == constraint_visuals_.end()) {
      constraint_visuals_[constraint_uuid] =
        constraint_source_properties_[constraint_source]->createAndInsertVisual(
        scene_manager_, root_node_, *relative_pose, *graph);
    } else {
      constraint_visuals_[constraint_uuid]->setConstraint(*relative_pose, *graph);
    }

    constraints_changed_map_[constraint_uuid] = true;
  }

  for (const auto & entry : variables_changed_map_) {
    if (!entry.second) {
      variable_property_->eraseVisual(entry.first);
    }
  }

  for (auto it = variables_changed_map_.begin(); it != variables_changed_map_.end(); ) {
    if (it->second) {
      ++it;
    } else {
      it = variables_changed_map_.erase(it);
    }
  }

  for (const auto & entry : constraints_changed_map_) {
    if (!entry.second) {
      constraint_source_properties_[constraint_visuals_[entry.first]->getSource()]->eraseVisual(
        entry.first);
      constraint_visuals_.erase(entry.first);
    }
  }

  for (auto it = constraints_changed_map_.begin(); it != constraints_changed_map_.end(); ) {
    if (it->second) {
      ++it;
    } else {
      it = constraints_changed_map_.erase(it);
    }
  }
}

}  // namespace fuse_viz

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(fuse_viz::SerializedGraphDisplay, rviz_common::Display)
