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

#ifndef FUSE_VIZ__SERIALIZED_GRAPH_DISPLAY_HPP_
#define FUSE_VIZ__SERIALIZED_GRAPH_DISPLAY_HPP_

#ifndef Q_MOC_RUN
#include <fuse_core/graph_deserializer.hpp>
#include <fuse_core/uuid.hpp>
#include <fuse_msgs/msg/serialized_graph.hpp>

#include <rviz_common/message_filter_display.hpp>

#include <OgreColourValue.h>
#include <OgreSceneNode.h>
#endif  // Q_MOC_RUN

#include <map>
#include <memory>
#include <string>
#include <unordered_map>

namespace fuse_viz
{

class Pose2DStampedVisual;
class RelativePose2DStampedConstraintVisual;

using rviz_common::properties::BoolProperty;

class Pose2DStampedProperty;
class RelativePose2DStampedConstraintProperty;

/**
 * @brief An rviz dispaly for fuse_msgs::msg::SerializedGraph messages.
 */
class SerializedGraphDisplay
  : public rviz_common::MessageFilterDisplay<fuse_msgs::msg::SerializedGraph>
{
  Q_OBJECT

public:
  SerializedGraphDisplay();

  ~SerializedGraphDisplay() override;

  void reset() override;

protected:
  void onInitialize() override;

  void onEnable() override;

  void onDisable() override;

  void load(const rviz_common::Config & config) override;

private Q_SLOTS:
  void updateShowVariables();
  void updateShowConstraints();

private:
  using ChangedByUUIDMap = std::unordered_map<fuse_core::UUID, bool, fuse_core::uuid::hash>;
  using ConstraintByUUIDMap =
    std::unordered_map<fuse_core::UUID, std::shared_ptr<RelativePose2DStampedConstraintVisual>,
      fuse_core::uuid::hash>;
  using ColorBySourceMap = std::unordered_map<std::string, Ogre::ColourValue>;
  using ConstraintPropertyBySourceMap = std::map<std::string,
      RelativePose2DStampedConstraintProperty *>;
  using ConfigBySourceMap = std::unordered_map<std::string, rviz_common::Config>;

  void clear();

  void processMessage(fuse_msgs::msg::SerializedGraph::ConstSharedPtr msg) override;

  Ogre::SceneNode * root_node_;

  ConstraintByUUIDMap constraint_visuals_;

  ColorBySourceMap source_color_map_;

  ChangedByUUIDMap variables_changed_map_;
  ChangedByUUIDMap constraints_changed_map_;

  BoolProperty * show_variables_property_;
  BoolProperty * show_constraints_property_;
  Pose2DStampedProperty * variable_property_;
  ConstraintPropertyBySourceMap constraint_source_properties_;

  ConfigBySourceMap constraint_source_configs_;

  fuse_core::GraphDeserializer graph_deserializer_;
};

}  // namespace fuse_viz

#endif  // FUSE_VIZ__SERIALIZED_GRAPH_DISPLAY_HPP_
