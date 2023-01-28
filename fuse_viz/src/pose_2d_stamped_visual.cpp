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

#include <OgreQuaternion.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <memory>

#include <fuse_variables/orientation_2d_stamped.hpp>
#include <fuse_variables/position_2d_stamped.hpp>
#include <fuse_viz/conversions.hpp>
#include <fuse_viz/pose_2d_stamped_visual.hpp>
#include <rviz_rendering/objects/axes.hpp>
#include <rviz_rendering/objects/movable_text.hpp>
#include <rviz_rendering/objects/shape.hpp>

namespace fuse_viz
{
using rviz_rendering::MovableText;

Pose2DStampedVisual::Pose2DStampedVisual(
  Ogre::SceneManager * scene_manager, Ogre::SceneNode * parent_node,
  const fuse_variables::Position2DStamped & position,
  const fuse_variables::Orientation2DStamped & orientation, const bool visible)
: Object(scene_manager), root_node_(parent_node->createChildSceneNode()), visible_(visible)
{
  // Create sphere:
  sphere_node_ = root_node_->createChildSceneNode();
  sphere_ = std::make_shared<rviz_rendering::Shape>(
    rviz_rendering::Shape::Sphere, scene_manager_,
    sphere_node_);
  setSphereColor(1.0, 0.0, 0.0, 1.0);

  // Create axes:
  axes_node_ = root_node_->createChildSceneNode();
  axes_ = std::make_shared<rviz_rendering::Axes>(scene_manager_, axes_node_, 10.0, 1.0);

  // Create text:
  const auto caption = position.type() + "::" + fuse_core::uuid::to_string(position.uuid()) + '\n' +
    orientation.type() + "::" + fuse_core::uuid::to_string(orientation.uuid());
  text_ = new MovableText(caption);
  text_->setCaption(caption);
  text_->setTextAlignment(MovableText::H_CENTER, MovableText::V_ABOVE);
  text_->showOnTop();

  text_node_ = root_node_->createChildSceneNode();
  text_node_->attachObject(text_);

  // Set pose for all objects:
  setPose2DStamped(position, orientation);

  // Set the initial visibility:
  // The root node is always visible. The visibility will be updated on its childs.
  root_node_->setVisible(true);
  setVisible(visible_);
}

Pose2DStampedVisual::~Pose2DStampedVisual()
{
  delete text_;
  scene_manager_->destroySceneNode(sphere_node_);
  scene_manager_->destroySceneNode(axes_node_);
  scene_manager_->destroySceneNode(text_node_);
  scene_manager_->destroySceneNode(root_node_);
}

void Pose2DStampedVisual::setPose2DStamped(
  const fuse_variables::Position2DStamped & position,
  const fuse_variables::Orientation2DStamped & orientation)
{
  setPose2DStamped(toOgre(position), toOgre(orientation));
}

void Pose2DStampedVisual::setUserData(const Ogre::Any & data)
{
  axes_->setUserData(data);
  sphere_->setUserData(data);
}

void Pose2DStampedVisual::setSphereColor(const float r, const float g, const float b, const float a)
{
  sphere_->setColor(r, g, b, a);
}

void Pose2DStampedVisual::setAxesAlpha(const float alpha)
{
  static const auto & default_x_color_ = axes_->getDefaultXColor();
  static const auto & default_y_color_ = axes_->getDefaultYColor();
  static const auto & default_z_color_ = axes_->getDefaultZColor();

  axes_->setXColor(
    Ogre::ColourValue(
      default_x_color_.r, default_x_color_.g, default_x_color_.b,
      alpha));                                                                                               // NOLINT
  axes_->setYColor(
    Ogre::ColourValue(
      default_y_color_.r, default_y_color_.g, default_y_color_.b,
      alpha));                                                                                               // NOLINT
  axes_->setZColor(
    Ogre::ColourValue(
      default_z_color_.r, default_z_color_.g, default_z_color_.b,
      alpha));                                                                                               // NOLINT
}

void Pose2DStampedVisual::setScale(const Ogre::Vector3 & scale)
{
  sphere_->setScale(scale);
  axes_->setScale(scale);
}

void Pose2DStampedVisual::setTextScale(const Ogre::Vector3 & scale)
{
  text_node_->setScale(scale);
}

void Pose2DStampedVisual::setTextVisible(const bool visible)
{
  text_->setVisible(visible);
}

void Pose2DStampedVisual::setVisible(const bool visible)
{
  sphere_node_->setVisible(visible);
  axes_node_->setVisible(visible);
}

void Pose2DStampedVisual::setPose2DStamped(
  const Ogre::Vector3 & position,
  const Ogre::Quaternion & orientation)
{
  axes_->setPosition(position);
  axes_->setOrientation(orientation);
  sphere_->setPosition(position);
  text_node_->setPosition(position);
}

const Ogre::Vector3 & Pose2DStampedVisual::getPosition()
{
  return root_node_->getPosition();
}

const Ogre::Quaternion & Pose2DStampedVisual::getOrientation()
{
  return root_node_->getOrientation();
}

void Pose2DStampedVisual::setPosition(const Ogre::Vector3 & position)
{
  root_node_->setPosition(position);
}

void Pose2DStampedVisual::setOrientation(const Ogre::Quaternion & orientation)
{
  root_node_->setOrientation(orientation);
}

}  // namespace fuse_viz
