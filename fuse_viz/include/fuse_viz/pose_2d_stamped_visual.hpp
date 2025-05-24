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

#ifndef FUSE_VIZ__POSE_2D_STAMPED_VISUAL_HPP_
#define FUSE_VIZ__POSE_2D_STAMPED_VISUAL_HPP_

#include <Ogre.h>

#include <memory>

#include <rviz_rendering/objects/axes.hpp>
#include <rviz_rendering/objects/movable_text.hpp>
#include <rviz_rendering/objects/object.hpp>
#include <rviz_rendering/objects/shape.hpp>
#include <tf2/LinearMath/Transform.hpp>


namespace Ogre
{

class SceneManager;
class SceneNode;
class Any;

}  // namespace Ogre

namespace fuse_variables
{

class Position2DStamped;
class Orientation2DStamped;

}  // namespace fuse_variables

namespace fuse_viz
{

/**
 * @class Pose2DStampedVisual
 *
 * @brief 2D pose variable visual consisting on:
 * 1. A sphere representing the 2D position.
 * 2. An axes representing the 2D pose (position + orientation).
 * 3. A text with the variable type and UUID.
 */
class Pose2DStampedVisual : public rviz_rendering::Object
{
private:
  /**
   * @brief Private Constructor
   *
   * Pose2DStampedVisual can only be constructed by friend class
   * Pose2DStampedProperty.
   *
   * @param[in] scene_manager The scene manager to use to construct any necessary objects
   * @param[in] parent_object A rviz object that this constraint will be attached.
   * @param[in] position fuse_variables::Position2DStamped position.
   * @param[in] orientation fuse_variables::Orientation2DStamped orientation.
   * @param[in] visible Initial visibility.
   */
  Pose2DStampedVisual(
    Ogre::SceneManager * scene_manager, Ogre::SceneNode * parent_node,
    const fuse_variables::Position2DStamped & position,
    const fuse_variables::Orientation2DStamped & orientation, const bool visible = true);

public:
  ~Pose2DStampedVisual() override;

  /**
   * @brief Set 2D pose stamped.
   * @param[in] position    2D position stamped variable.
   * @param[in] orientation 2D orientation stamped variable.
   */
  void setPose2DStamped(
    const fuse_variables::Position2DStamped & position,
    const fuse_variables::Orientation2DStamped & orientation);

  /**
   * @brief Get the root scene node of this variable visual.
   * @return the root scene node of this variable visual.
   */
  Ogre::SceneNode * getSceneNode()
  {
    return root_node_;
  }

  /**
   * @brief Sets user data on all ogre objects we own
   */
  void setUserData(const Ogre::Any & data) override;

  void setSphereColor(const float r, const float g, const float b, const float a);

  void setAxesAlpha(const float alpha);

  void setScale(const Ogre::Vector3 & scale) override;

  void setTextScale(const Ogre::Vector3 & scale);

  void setTextVisible(const bool visible);

  /**
   * @brief Sets visibility of this constraint
   *
   * Convenience method that sets visibility
   */
  void setVisible(bool visible);

  /**
   * @brief Sets position of the frame this constraint is attached
   */
  void setPosition(const Ogre::Vector3 & position) override;

  /**
   * @brief Sets orientation of the frame this constraint is attached
   */
  void setOrientation(const Ogre::Quaternion & orientation) override;

private:
  void setPose2DStamped(const Ogre::Vector3 & position, const Ogre::Quaternion & orientation);

  Ogre::SceneNode * root_node_ = nullptr;
  Ogre::SceneNode * sphere_node_ = nullptr;
  Ogre::SceneNode * axes_node_ = nullptr;
  Ogre::SceneNode * text_node_ = nullptr;

  bool visible_;

  std::shared_ptr<rviz_rendering::Axes> axes_;
  std::shared_ptr<rviz_rendering::Shape> sphere_;
  rviz_rendering::MovableText * text_;

private:
  // Hide Object methods we don't want to expose
  // NOTE: Apparently we still need to define them...
  void setColor(float, float, float, float) override {}
  const Ogre::Vector3 & getPosition() override;
  const Ogre::Quaternion & getOrientation() override;

  // Make Pose2DStampedProperty friend class so it create Pose2DStampedVisual objects
  friend class Pose2DStampedProperty;
};

}  // namespace fuse_viz

#endif  // FUSE_VIZ__POSE_2D_STAMPED_VISUAL_HPP_
