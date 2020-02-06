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

#include <fuse_viz/conversions.h>
#include <fuse_viz/mapped_covariance_visual.h>
#include <fuse_viz/pose_2d_stamped_visual.h>
#include <fuse_viz/relative_pose_2d_stamped_constraint_visual.h>

#include <fuse_constraints/relative_pose_2d_stamped_constraint.h>
#include <fuse_core/graph.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <rviz/ogre_helpers/axes.h>
#include <rviz/ogre_helpers/billboard_line.h>
#include <rviz/ogre_helpers/movable_text.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <OgreQuaternion.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

namespace rviz
{

RelativePose2DStampedConstraintVisual::RelativePose2DStampedConstraintVisual(
    Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node,
    const fuse_constraints::RelativePose2DStampedConstraint& constraint, const bool visible)
  : Object(scene_manager)
  , root_node_(parent_node->createChildSceneNode())
  , source_(constraint.source())
  , visible_(visible)
{
  // Create constraint relative pose line:
  relative_pose_line_node_ = root_node_->createChildSceneNode();
  relative_pose_line_ = std::make_shared<BillboardLine>(scene_manager_, relative_pose_line_node_);
  relative_pose_line_->setMaxPointsPerLine(2);
  relative_pose_line_->setNumLines(1);

  // Create constraint error line:
  error_line_node_ = root_node_->createChildSceneNode();
  error_line_ = std::make_shared<BillboardLine>(scene_manager_, error_line_node_);
  error_line_->setMaxPointsPerLine(2);
  error_line_->setNumLines(1);

  // Create constraint loss error line:
  loss_error_line_node_ = root_node_->createChildSceneNode();
  loss_error_line_ = std::make_shared<BillboardLine>(scene_manager_, loss_error_line_node_);
  loss_error_line_->setMaxPointsPerLine(2);
  loss_error_line_->setNumLines(1);

  // Create constraint relative pose axes:
  relative_pose_axes_node_ = root_node_->createChildSceneNode();
  relative_pose_axes_ = std::make_shared<rviz::Axes>(scene_manager_, relative_pose_axes_node_, 10.0, 1.0);

  // Draw text:
  const auto caption = source_ + "@" + constraint.type() + "::" + fuse_core::uuid::to_string(constraint.uuid());
  text_ = new MovableText(caption);
  text_->setCaption(caption);
  text_->setTextAlignment(MovableText::H_CENTER, MovableText::V_ABOVE);
  text_->showOnTop();

  text_node_ = root_node_->createChildSceneNode();
  text_node_->attachObject(text_);

  // Set the initial visibility:
  // The root node is always visible. The visibility will be updated on its childs.
  root_node_->setVisible(true);
  setVisible(visible_);
}

RelativePose2DStampedConstraintVisual::~RelativePose2DStampedConstraintVisual()
{
  delete text_;
  scene_manager_->destroySceneNode(relative_pose_line_node_->getName());
  scene_manager_->destroySceneNode(error_line_node_->getName());
  scene_manager_->destroySceneNode(loss_error_line_node_->getName());
  scene_manager_->destroySceneNode(relative_pose_axes_node_->getName());
  scene_manager_->destroySceneNode(text_node_->getName());
  scene_manager_->destroySceneNode(root_node_->getName());
}

void RelativePose2DStampedConstraintVisual::setConstraint(
    const fuse_constraints::RelativePose2DStampedConstraint& constraint, const fuse_core::Graph& graph)
{
  // Update constraint relative pose line:
  const auto& variables = constraint.variables();

  const auto pose1 = getPose(graph, variables.at(0), variables.at(1));

  const auto& delta = constraint.delta();
  const tf2::Transform pose_delta{ tf2::Quaternion{ tf2::Vector3{ 0, 0, 1 }, delta[2] },
                                   tf2::Vector3{ delta[0], delta[1], 0 } };
  const auto absolute_pose = pose1 * pose_delta;

  const auto absolute_position_ogre = toOgre(absolute_pose.getOrigin());

  relative_pose_line_->clear();
  relative_pose_line_->addPoint(toOgre(pose1.getOrigin()));
  relative_pose_line_->addPoint(absolute_position_ogre);

  // Update constraint covariance:
  geometry_msgs::PoseWithCovariance relative_pose_msg;
  tf2::toMsg(absolute_pose, relative_pose_msg.pose);
  tf2::toMsg(constraint.covariance(), relative_pose_msg.covariance);

  covariance_->setPosition(absolute_position_ogre);
  covariance_->setOrientation(toOgre(absolute_pose.getRotation()));
  covariance_->setCovariance(relative_pose_msg);

  // Update constraint error line:
  const auto pose2 = getPose(graph, variables.at(2), variables.at(3));

  error_line_->clear();
  error_line_->addPoint(absolute_position_ogre);
  error_line_->addPoint(toOgre(pose2.getOrigin()));

  // Update constraint loss error line:
  loss_error_line_->clear();

  auto loss_function = constraint.lossFunction();
  if (loss_function)
  {
    // Evaluate cost function without loss:
    const double position1[] = { pose1.getOrigin().getX(), pose1.getOrigin().getY() };
    const double yaw1[] = { tf2::getYaw(pose1.getRotation()) };
    const double position2[] = { pose2.getOrigin().getX(), pose2.getOrigin().getY() };
    const double yaw2[] = { tf2::getYaw(pose2.getRotation()) };

    const double* parameters[] = { position1, yaw1, position2, yaw2 };

    auto cost_function = constraint.costFunction();

    fuse_core::VectorXd residuals(cost_function->num_residuals());

    cost_function->Evaluate(parameters, residuals.data(), nullptr);
    delete cost_function;

    // The cost without the loss would be:
    //
    // cost = 0.5 * squared_norm
    //
    // See https://github.com/ceres-solver/ceres-solver/blob/master/internal/ceres/residual_block.cc#L159
    const auto squared_norm = residuals.squaredNorm();

    // Evaluate the loss as in:
    // https://github.com/ceres-solver/ceres-solver/blob/master/internal/ceres/residual_block.cc#L164
    //
    // The cost with the loss would be:
    //
    // loss_cost = 0.5 * rho[0]
    //
    // See https://github.com/ceres-solver/ceres-solver/blob/master/internal/ceres/residual_block.cc#L165
    double rho[3];
    loss_function->Evaluate(squared_norm, rho);
    delete loss_function;

    // Interpolate between the constraint's absolute position and its second variable position by the quotient between
    // the cost with and without loss:
    //
    //              loss_cost      0.5 * rho[0]         rho[0]
    // loss_scale = --------- = ------------------ = ------------
    //                cost      0.5 * squared_norm   squared_norm
    //
    // Remember that in principle `rho[0] <= squared_norm`, with `rho[0] == squared_norm` for the inlier region, and
    // `rho[0] < squared_norm` for the outlier region:
    const auto loss_scale = squared_norm == 0.0 ? 0.0 : rho[0] / squared_norm;

    const auto loss_position = absolute_pose.getOrigin().lerp(pose2.getOrigin(), loss_scale);

    loss_error_line_->addPoint(absolute_position_ogre);
    loss_error_line_->addPoint(toOgre(loss_position));
  }

  // Update constraint relative pose axes:
  relative_pose_axes_->setPosition(absolute_position_ogre);
  relative_pose_axes_->setOrientation(toOgre(absolute_pose.getRotation()));

  // Update text:
  text_node_->setPosition(absolute_position_ogre);
}

void RelativePose2DStampedConstraintVisual::setUserData(const Ogre::Any& data)
{
  relative_pose_line_->setUserData(data);
  error_line_->setUserData(data);
  loss_error_line_->setUserData(data);
  relative_pose_axes_->setUserData(data);
  covariance_->setUserData(data);
}

void RelativePose2DStampedConstraintVisual::setRelativePoseLineWidth(const float line_width)
{
  relative_pose_line_->setLineWidth(line_width);
}

void RelativePose2DStampedConstraintVisual::setErrorLineWidth(const float line_width)
{
  error_line_->setLineWidth(line_width);
}

void RelativePose2DStampedConstraintVisual::setLossErrorLineWidth(const float line_width)
{
  loss_error_line_->setLineWidth(line_width);
}

void RelativePose2DStampedConstraintVisual::setRelativePoseLineColor(const float r, const float g, const float b,
                                                                     const float a)
{
  relative_pose_line_->setColor(r, g, b, a);
}

void RelativePose2DStampedConstraintVisual::setErrorLineColor(const float r, const float g, const float b,
                                                              const float a)
{
  error_line_->setColor(r, g, b, a);
}

void RelativePose2DStampedConstraintVisual::setLossErrorLineColor(const float r, const float g, const float b,
                                                                  const float a)
{
  loss_error_line_->setColor(r, g, b, a);
}

void RelativePose2DStampedConstraintVisual::setRelativePoseAxesAlpha(const float alpha)
{
  static const auto& default_x_color_ = rviz::Axes::getDefaultXColor();
  static const auto& default_y_color_ = rviz::Axes::getDefaultYColor();
  static const auto& default_z_color_ = rviz::Axes::getDefaultZColor();

  relative_pose_axes_->setXColor(
      Ogre::ColourValue{ default_x_color_.r, default_x_color_.g, default_x_color_.b, alpha });  // NOLINT
  relative_pose_axes_->setYColor(
      Ogre::ColourValue{ default_y_color_.r, default_y_color_.g, default_y_color_.b, alpha });  // NOLINT
  relative_pose_axes_->setZColor(
      Ogre::ColourValue{ default_z_color_.r, default_z_color_.g, default_z_color_.b, alpha });  // NOLINT
}

void RelativePose2DStampedConstraintVisual::setRelativePoseAxesScale(const Ogre::Vector3& scale)
{
  relative_pose_axes_->setScale(scale);
}

void RelativePose2DStampedConstraintVisual::setTextScale(const Ogre::Vector3& scale)
{
  text_node_->setScale(scale);
}

void RelativePose2DStampedConstraintVisual::setTextVisible(const bool visible)
{
  text_->setVisible(visible);
}

void RelativePose2DStampedConstraintVisual::setVisible(const bool visible)
{
  relative_pose_line_node_->setVisible(visible);
  error_line_node_->setVisible(visible);
  loss_error_line_node_->setVisible(visible);
  relative_pose_axes_node_->setVisible(visible);
}

const Ogre::Vector3& RelativePose2DStampedConstraintVisual::getPosition()
{
  return root_node_->getPosition();
}

const Ogre::Quaternion& RelativePose2DStampedConstraintVisual::getOrientation()
{
  return root_node_->getOrientation();
}

void RelativePose2DStampedConstraintVisual::setPosition(const Ogre::Vector3& position)
{
  root_node_->setPosition(position);
}

void RelativePose2DStampedConstraintVisual::setOrientation(const Ogre::Quaternion& orientation)
{
  root_node_->setOrientation(orientation);
}

}  // namespace rviz
