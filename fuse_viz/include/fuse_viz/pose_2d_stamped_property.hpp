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

#ifndef FUSE_VIZ__POSE_2D_STAMPED_PROPERTY_HPP_
#define FUSE_VIZ__POSE_2D_STAMPED_PROPERTY_HPP_

#include <memory>
#include <unordered_map>

#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>

#include <fuse_core/uuid.hpp>


namespace Ogre
{

class SceneManager;
class SceneNode;

}  // namespace Ogre

namespace fuse_variables
{

class Position2DStamped;
class Orientation2DStamped;

}  // namespace fuse_variables

namespace fuse_viz
{

class Pose2DStampedVisual;

class Pose2DStampedProperty : public rviz_common::properties::BoolProperty
{
  Q_OBJECT

public:
  using Visual = Pose2DStampedVisual;
  using VisualPtr = std::shared_ptr<Visual>;

  Pose2DStampedProperty(
    const QString & name = "Pose2DStamped", bool default_value = true,
    const QString & description = QString(), Property * parent = NULL,
    const char * changed_slot = NULL, QObject * receiver = NULL);

  ~Pose2DStampedProperty() override = default;

  VisualPtr createAndInsertOrUpdateVisual(
    Ogre::SceneManager * scene_manager, Ogre::SceneNode * parent_node,
    const fuse_variables::Position2DStamped & position,
    const fuse_variables::Orientation2DStamped & orientation);
  void eraseVisual(const fuse_core::UUID & uuid);
  void clearVisual();

public Q_SLOTS:
  void updateVisibility();

private Q_SLOTS:
  void updateAxesAlpha();
  void updateScale();
  void updateShowText();
  void updateSphereColorAlpha();
  void updateTextScale();

private:
  void updateAxesAlpha(const VisualPtr & constraint);
  void updateScale(const VisualPtr & constraint);
  void updateShowText(const VisualPtr & constraint);
  void updateSphereColorAlpha(const VisualPtr & constraint);
  void updateTextScale(const VisualPtr & constraint);
  void updateVisibility(const VisualPtr & constraint);

  std::unordered_map<fuse_core::UUID, VisualPtr, fuse_core::uuid::hash> variables_;

  rviz_common::properties::ColorProperty * color_property_;
  rviz_common::properties::BoolProperty * show_text_property_;
  rviz_common::properties::FloatProperty * sphere_alpha_property_;
  rviz_common::properties::FloatProperty * axes_alpha_property_;
  rviz_common::properties::FloatProperty * scale_property_;
  rviz_common::properties::FloatProperty * text_scale_property_;
};

}  // namespace fuse_viz

#endif  // FUSE_VIZ__POSE_2D_STAMPED_PROPERTY_HPP_
