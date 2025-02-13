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
#include <fuse_core/graph.hpp>
#include <fuse_viz/mapped_covariance_property.hpp>
#include <fuse_viz/mapped_covariance_visual.hpp>
#include <fuse_viz/relative_pose_2d_stamped_constraint_property.hpp>
#include <fuse_viz/relative_pose_2d_stamped_constraint_visual.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>

namespace fuse_viz
{
using rviz_common::properties::BoolProperty;
using rviz_common::properties::ColorProperty;
using rviz_common::properties::FloatProperty;
using rviz_common::properties::Property;

RelativePose2DStampedConstraintProperty::RelativePose2DStampedConstraintProperty(
  const QString & name, bool default_value, const QString & description, Property * parent,
  const char * changed_slot, QObject * receiver)
// NOTE: changed_slot and receiver aren't passed to BoolProperty here, but initialized at the end of
// this constructor
: BoolProperty(name, default_value, description, parent)
{
  color_property_ =
    new ColorProperty(
    "Color", QColor(255, 255, 255), "Color to draw the constraint relative pose and error lines.",
    this, SLOT(updateColor()));

  relative_pose_axes_alpha_property_ = new FloatProperty(
    "Axes Alpha", 0.0, "Alpha of constraint relative pose axes.",
    this, SLOT(updateRelativePoseAxesAlpha()));
  relative_pose_axes_alpha_property_->setMin(0.0);
  relative_pose_axes_alpha_property_->setMax(1.0);

  relative_pose_axes_scale_property_ = new FloatProperty(
    "Axes Scale", 1.0, "Scale of constraint relative pose axes.",
    this, SLOT(updateRelativePoseAxesScale()));
  relative_pose_axes_scale_property_->setMin(0.0);

  relative_pose_line_alpha_property_ = new FloatProperty(
    "Line Alpha", 1.0, "Alpha of constraint relative pose line.",
    this, SLOT(updateRelativePoseLineAlpha()));
  relative_pose_line_alpha_property_->setMin(0.0);
  relative_pose_line_alpha_property_->setMax(1.0);

  relative_pose_line_width_property_ = new FloatProperty(
    "Line Width", 0.1, "Line width of constraint relative pose line.", this,
    SLOT(updateRelativePoseLineWidth()));
  relative_pose_line_width_property_->setMin(0.0);

  error_line_alpha_property_ =
    new FloatProperty(
    "Error Line Alpha", 0.5, "Alpha of constraint error line.", this,
    SLOT(updateErrorLineAlpha()));
  error_line_alpha_property_->setMin(0.0);
  error_line_alpha_property_->setMax(1.0);

  error_line_width_property_ = new FloatProperty(
    "Error Line Width", 0.1, "Line width of constraint error line.", this,
    SLOT(updateErrorLineWidth()));
  error_line_width_property_->setMin(0.0);

  loss_min_brightness_property_ = new FloatProperty(
    "Loss Min Brightness", 0.25,
    "Min brightness to show the loss impact on the constraint error "
    "line.",
    this, SLOT(updateLossMinBrightness()));
  loss_min_brightness_property_->setMin(0.0);
  loss_min_brightness_property_->setMax(1.0);

  show_text_property_ =
    new BoolProperty(
    "Show Text", false, "Show constraint source, type and UUID.", this,
    SLOT(updateShowText()));

  text_scale_property_ =
    new FloatProperty(
    "Text Scale", 1.0, "Scale of variable text.", this,
    SLOT(updateTextScale()));
  text_scale_property_->setMin(0.0);

  covariance_property_ = new MappedCovarianceProperty(
    "Covariance", true, "Whether or not the constraint covariance should be shown.", this);

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

RelativePose2DStampedConstraintProperty::VisualPtr RelativePose2DStampedConstraintProperty::
createAndInsertVisual(
  Ogre::SceneManager * scene_manager, Ogre::SceneNode * parent_node,
  const fuse_constraints::RelativePose2DStampedConstraint & constraint,
  const fuse_core::Graph & graph)
{
  VisualPtr visual{new Visual(scene_manager, parent_node, constraint)};
  constraints_[constraint.uuid()] = visual;

  visual->setCovariance(
    covariance_property_->createAndInsertVisual(
      fuse_core::uuid::to_string(constraint.uuid()),
      scene_manager, parent_node));
  visual->setConstraint(constraint, graph);

  updateColor(visual);
  updateErrorLineAlpha(visual);
  updateErrorLineWidth(visual);
  updateLossMinBrightness(visual);
  updateRelativePoseAxesAlpha(visual);
  updateRelativePoseAxesScale(visual);
  updateRelativePoseLineAlpha(visual);
  updateRelativePoseLineWidth(visual);
  updateShowText(visual);
  updateTextScale(visual);
  updateVisibility(visual);

  return visual;
}

void RelativePose2DStampedConstraintProperty::eraseVisual(const fuse_core::UUID & uuid)
{
  covariance_property_->eraseVisual(fuse_core::uuid::to_string(uuid));
  constraints_.erase(uuid);
}

void RelativePose2DStampedConstraintProperty::clearVisual()
{
  covariance_property_->clearVisual();
  constraints_.clear();
}

void RelativePose2DStampedConstraintProperty::setColor(const QColor & color)
{
  color_property_->setColor(color);
}

void RelativePose2DStampedConstraintProperty::updateVisibility()
{
  for (auto & entry : constraints_) {
    updateVisibility(entry.second);
  }
}

void RelativePose2DStampedConstraintProperty::updateColor()
{
  for (auto & entry : constraints_) {
    updateColor(entry.second);
  }
}

void RelativePose2DStampedConstraintProperty::updateErrorLineAlpha()
{
  for (auto & entry : constraints_) {
    updateErrorLineAlpha(entry.second);
  }
}

void RelativePose2DStampedConstraintProperty::updateErrorLineWidth()
{
  for (auto & entry : constraints_) {
    updateErrorLineWidth(entry.second);
  }
}

void RelativePose2DStampedConstraintProperty::updateLossMinBrightness()
{
  for (auto & entry : constraints_) {
    updateLossMinBrightness(entry.second);
  }
}

void RelativePose2DStampedConstraintProperty::updateRelativePoseAxesAlpha()
{
  for (auto & entry : constraints_) {
    updateRelativePoseAxesAlpha(entry.second);
  }
}

void RelativePose2DStampedConstraintProperty::updateRelativePoseAxesScale()
{
  for (auto & entry : constraints_) {
    updateRelativePoseAxesScale(entry.second);
  }
}

void RelativePose2DStampedConstraintProperty::updateRelativePoseLineAlpha()
{
  for (auto & entry : constraints_) {
    updateRelativePoseLineAlpha(entry.second);
  }
}

void RelativePose2DStampedConstraintProperty::updateRelativePoseLineWidth()
{
  for (auto & entry : constraints_) {
    updateRelativePoseLineWidth(entry.second);
  }
}

void RelativePose2DStampedConstraintProperty::updateShowText()
{
  for (auto & entry : constraints_) {
    updateShowText(entry.second);
  }
}

void RelativePose2DStampedConstraintProperty::updateTextScale()
{
  for (auto & entry : constraints_) {
    updateTextScale(entry.second);
  }
}

void RelativePose2DStampedConstraintProperty::updateColor(const VisualPtr & constraint)
{
  const auto color = color_property_->getColor();

  constraint->setRelativePoseLineColor(
    color.redF(), color.greenF(), color.blueF(),
    relative_pose_line_alpha_property_->getFloat());
  constraint->setErrorLineColor(
    color.redF(), color.greenF(),
    color.blueF(), error_line_alpha_property_->getFloat());
}

void RelativePose2DStampedConstraintProperty::updateErrorLineAlpha(const VisualPtr & constraint)
{
  const auto color = color_property_->getColor();

  constraint->setErrorLineColor(
    color.redF(), color.greenF(),
    color.blueF(), error_line_alpha_property_->getFloat());
}

void RelativePose2DStampedConstraintProperty::updateErrorLineWidth(const VisualPtr & constraint)
{
  constraint->setErrorLineWidth(error_line_width_property_->getFloat());
}

void RelativePose2DStampedConstraintProperty::updateLossMinBrightness(const VisualPtr & constraint)
{
  constraint->setLossMinBrightness(loss_min_brightness_property_->getFloat());

  updateErrorLineAlpha(constraint);
}

void RelativePose2DStampedConstraintProperty::updateRelativePoseAxesAlpha(
  const VisualPtr & constraint)
{
  constraint->setRelativePoseAxesAlpha(relative_pose_axes_alpha_property_->getFloat());
}

void RelativePose2DStampedConstraintProperty::updateRelativePoseAxesScale(
  const VisualPtr & constraint)
{
  constraint->setRelativePoseAxesScale(
    Ogre::Vector3{relative_pose_axes_scale_property_->getFloat()});
}

void RelativePose2DStampedConstraintProperty::updateRelativePoseLineAlpha(
  const VisualPtr & constraint)
{
  const auto color = color_property_->getColor();

  constraint->setRelativePoseLineColor(
    color.redF(), color.greenF(), color.blueF(),
    relative_pose_line_alpha_property_->getFloat());
}

void RelativePose2DStampedConstraintProperty::updateRelativePoseLineWidth(
  const VisualPtr & constraint)
{
  constraint->setRelativePoseLineWidth(relative_pose_line_width_property_->getFloat());
}

void RelativePose2DStampedConstraintProperty::updateShowText(const VisualPtr & constraint)
{
  constraint->setTextVisible(show_text_property_->getBool());
}

void RelativePose2DStampedConstraintProperty::updateTextScale(const VisualPtr & constraint)
{
  constraint->setTextScale(Ogre::Vector3{text_scale_property_->getFloat()});  // NOLINT
}

void RelativePose2DStampedConstraintProperty::updateVisibility(const VisualPtr & constraint)
{
  const auto visible = getBool();

  constraint->setVisible(visible);
  constraint->setTextVisible(visible && show_text_property_->getBool());
  constraint->getCovariance()->setVisible(visible && covariance_property_->getBool());
}

}  // end namespace fuse_viz
