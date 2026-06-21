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

#ifndef FUSE_VIZ_RELATIVE_POSE_2D_STAMPED_CONSTRAINT_VISUAL_H
#define FUSE_VIZ_RELATIVE_POSE_2D_STAMPED_CONSTRAINT_VISUAL_H

#include <fuse_viz/mapped_covariance_property.h>

#include <rviz/ogre_helpers/object.h>

#include <OgreColourValue.h>
#include <Ogre.h>

#include <memory>
#include <string>

namespace Ogre
{

class SceneManager;
class SceneNode;
class Any;

}  // namespace Ogre

namespace fuse_core
{

class Graph;

}  // namespace fuse_core

namespace fuse_constraints
{

class RelativePose2DStampedConstraint;

}  // namespace fuse_constraints

namespace rviz
{

class Axes;
class BillboardLine;
class MovableText;

class Pose2DStampedVisual;
class RelativePose2DStampedConstraintProperty;

/**
 * @class RelativePose2DStampedConstraintVisual
 *
 * @brief Relative 2D pose constraint visual consisting on:
 * 1. A line that starts from the first/source variable position and ends at the relative position wrt that variable.
 * 2. An axes that shows the relative pose wrt the first/source variable.
 * 3. A line that starts from the relative position wrt the first/source variable and ends at the second/target variable
 * position, which represents the error.
 * 4. A covariance visual object that represents the relative pose 2D covariance with an ellipse for the position and a
 * cone for the orientation.
 * 5. A text with the constraint source, type and UUID.
 */
class RelativePose2DStampedConstraintVisual : public rviz::Object
{
private:
  /**
   * @brief Private Constructor
   *
   * RelativePose2DStampedConstraintVisual can only be constructed by friend class
   * RelativePose2DStampedConstraintProperty.
   *
   * @param[in] scene_manager The scene manager to use to construct any necessary objects.
   * @param[in] parent_object A rviz object that this constraint will be attached.
   * @param[in] constraint fuse_constraints::RelativePose2DStampedConstraint constraint.
   * @param[in] visible Initial visibility.
   */
  RelativePose2DStampedConstraintVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node,
                                        const fuse_constraints::RelativePose2DStampedConstraint& constraint,
                                        const bool visible = true);

public:
  using CovarianceVisualPtr = MappedCovarianceProperty::MappedCovarianceVisualPtr;

  ~RelativePose2DStampedConstraintVisual() override;

  /**
   * @brief Set the constraint.
   * @param[in] constraint fuse_constraints::RelativePose2DStampedConstraint constraint.
   * @param[in] graph fuse_core::Graph, used to retrieve the first/source and second/target
   * constraint variables pose.
   */
  void setConstraint(const fuse_constraints::RelativePose2DStampedConstraint& constraint,
                     const fuse_core::Graph& graph);

  /**
   * @brief Get the root scene node of this constraint visual
   * @return the root scene node of this constraint visual
   */
  Ogre::SceneNode* getSceneNode()
  {
    return root_node_;
  }

  /**
   * @brief Sets user data on all ogre objects we own
   */
  void setUserData(const Ogre::Any& data) override;

  void setRelativePoseLineWidth(const float line_width);

  void setErrorLineWidth(const float line_width);

  void setLossMinBrightness(const float min_brightness);

  void setRelativePoseLineColor(const float r, const float g, const float b, const float a);

  void setErrorLineColor(const float r, const float g, const float b, const float a);

  void setRelativePoseAxesAlpha(const float alpha);

  void setRelativePoseAxesScale(const Ogre::Vector3& scale);

  void setTextScale(const Ogre::Vector3& scale);

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
  void setPosition(const Ogre::Vector3& position) override;

  /**
   * @brief Sets orientation of the frame this constraint is attached
   */
  void setOrientation(const Ogre::Quaternion& orientation) override;

  const CovarianceVisualPtr& getCovariance() const
  {
    return covariance_;
  }

  void setCovariance(const CovarianceVisualPtr& covariance)
  {
    covariance_ = covariance;
  }

  const std::string& getSource() const
  {
    return source_;
  }

private:
  Ogre::SceneNode* root_node_;
  Ogre::SceneNode* relative_pose_line_node_;
  Ogre::SceneNode* error_line_node_;
  Ogre::SceneNode* relative_pose_axes_node_;
  Ogre::SceneNode* text_node_;

  std::shared_ptr<BillboardLine> relative_pose_line_;
  std::shared_ptr<BillboardLine> error_line_;
  std::shared_ptr<Axes> relative_pose_axes_;
  MovableText* text_;
  CovarianceVisualPtr covariance_;
  std::string source_;

  float loss_scale_{ -1.0 };
  float min_brightness_{ 0.0 };
  Ogre::ColourValue error_line_color_;

  bool visible_;

private:
  // Hide Object methods we don't want to expose
  // NOTE: Apparently we still need to define them...
  void setScale(const Ogre::Vector3& scale) override{};
  void setColor(float r, float g, float b, float a) override{};
  const Ogre::Vector3& getPosition() override;
  const Ogre::Quaternion& getOrientation() override;

  Ogre::ColourValue computeLossErrorLineColor(const Ogre::ColourValue& color, const float loss_scale);

  // Make RelativePose2DStampedConstraintProperty friend class so it create RelativePose2DStampedConstraintVisual
  // objects
  friend class RelativePose2DStampedConstraintProperty;
};

}  // namespace rviz

#endif  // FUSE_VIZ_RELATIVE_POSE_2D_STAMPED_CONSTRAINT_VISUAL_H
