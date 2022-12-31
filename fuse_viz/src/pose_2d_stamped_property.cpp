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

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <QColor>

#include <fuse_constraints/relative_pose_2d_stamped_constraint.hpp>
#include <fuse_core/uuid.hpp>
#include <fuse_viz/pose_2d_stamped_property.hpp>
#include <fuse_viz/pose_2d_stamped_visual.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/property.hpp>

namespace fuse_viz
{

using rviz_common::properties::ColorProperty;
using rviz_common::properties::FloatProperty;
using rviz_common::properties::Property;

Pose2DStampedProperty::Pose2DStampedProperty(
  const QString & name, bool default_value, const QString & description,
  Property * parent, const char * changed_slot, QObject * receiver)
// NOTE: changed_slot and receiver aren't passed to BoolProperty here, but initialized at the end of
// this constructor
: BoolProperty(name, default_value, description, parent)
{
  color_property_ = new ColorProperty(
    "Color", QColor(255, 0, 0), "Color to draw the variable sphere.", this,
    SLOT(updateSphereColorAlpha()));

  sphere_alpha_property_ =
    new FloatProperty(
    "Sphere Alpha", 1.0, "Alpha of variable sphere.", this,
    SLOT(updateSphereColorAlpha()));
  sphere_alpha_property_->setMin(0.0);
  sphere_alpha_property_->setMax(1.0);

  axes_alpha_property_ =
    new FloatProperty(
    "Axes Alpha", 0.0, "Alpha of variable axes.", this,
    SLOT(updateAxesAlpha()));
  axes_alpha_property_->setMin(0.0);
  axes_alpha_property_->setMax(1.0);

  scale_property_ = new FloatProperty(
    "Scale", 1.0, "Scale of variable sphere and axes.", this, SLOT(
      updateScale()));
  scale_property_->setMin(0.0);

  show_text_property_ =
    new BoolProperty(
    "Show Text", false, "Show variable type and UUID.", this, SLOT(
      updateShowText()));

  text_scale_property_ =
    new FloatProperty(
    "Text Scale", 1.0, "Scale of variable text.", this,
    SLOT(updateTextScale()));
  text_scale_property_->setMin(0.0);

  connect(this, SIGNAL(changed()), this, SLOT(updateVisibility()));

  // Connect changed() signal here instead of doing it through the initialization of BoolProperty().
  // We do this here to make changed_slot be called _after_ updateVisibility()
  if (changed_slot && (parent || receiver)) {
    if (receiver) {
      connect(this, SIGNAL(changed()), receiver, changed_slot);
    } else {
      connect(this, SIGNAL(changed()), parent, changed_slot);
    }
  }

  setDisableChildrenIfFalse(true);
}

Pose2DStampedProperty::VisualPtr Pose2DStampedProperty::createAndInsertOrUpdateVisual(
  Ogre::SceneManager * scene_manager, Ogre::SceneNode * parent_node,
  const fuse_variables::Position2DStamped & position,
  const fuse_variables::Orientation2DStamped & orientation)
{
  auto & visual = variables_[position.uuid()];

  if (visual) {
    visual->setPose2DStamped(position, orientation);
  } else {
    visual.reset(new Visual(scene_manager, parent_node, position, orientation));

    visual->setPose2DStamped(position, orientation);

    updateAxesAlpha(visual);
    updateScale(visual);
    updateShowText(visual);
    updateSphereColorAlpha(visual);
    updateTextScale(visual);
    updateVisibility(visual);
  }

  return visual;
}

void Pose2DStampedProperty::eraseVisual(const fuse_core::UUID & uuid)
{
  variables_.erase(uuid);
}

void Pose2DStampedProperty::clearVisual()
{
  variables_.clear();
}

void Pose2DStampedProperty::updateVisibility()
{
  for (auto & entry : variables_) {
    updateVisibility(entry.second);
  }
}

void Pose2DStampedProperty::updateAxesAlpha()
{
  for (auto & entry : variables_) {
    updateAxesAlpha(entry.second);
  }
}

void Pose2DStampedProperty::updateScale()
{
  for (auto & entry : variables_) {
    updateScale(entry.second);
  }
}

void Pose2DStampedProperty::updateShowText()
{
  for (auto & entry : variables_) {
    updateShowText(entry.second);
  }
}

void Pose2DStampedProperty::updateSphereColorAlpha()
{
  for (auto & entry : variables_) {
    updateSphereColorAlpha(entry.second);
  }
}

void Pose2DStampedProperty::updateTextScale()
{
  for (auto & entry : variables_) {
    updateTextScale(entry.second);
  }
}

void Pose2DStampedProperty::updateAxesAlpha(const VisualPtr & variable)
{
  variable->setAxesAlpha(axes_alpha_property_->getFloat());
}

void Pose2DStampedProperty::updateScale(const VisualPtr & variable)
{
  variable->setScale(Ogre::Vector3{scale_property_->getFloat()});    // NOLINT(whitespace/braces)
}

void Pose2DStampedProperty::updateShowText(const VisualPtr & variable)
{
  variable->setTextVisible(show_text_property_->getBool());
}

void Pose2DStampedProperty::updateSphereColorAlpha(const VisualPtr & variable)
{
  const auto color = color_property_->getColor();

  variable->setSphereColor(
    color.redF(), color.greenF(),
    color.blueF(), sphere_alpha_property_->getFloat());
}

void Pose2DStampedProperty::updateTextScale(const VisualPtr & variable)
{
  variable->setTextScale(Ogre::Vector3{text_scale_property_->getFloat()});  // NOLINT
}

void Pose2DStampedProperty::updateVisibility(const VisualPtr & variable)
{
  const auto visible = getBool();

  variable->setVisible(visible);
  variable->setTextVisible(visible && show_text_property_->getBool());
}

}  // namespace fuse_viz
