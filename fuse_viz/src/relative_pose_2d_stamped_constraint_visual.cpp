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

#include <algorithm>
#include <memory>
#include <string>


namespace rviz
{

/**
 * @brief Generate a constraint name of the form: source@type::uuid
 *
 * @param[in] constraint A constraint
 * @return The constraint name of the form: source@type::uuid
 */
std::string constraint_name(const fuse_constraints::RelativePose2DStampedConstraint& constraint)
{
  return constraint.source() + '@' + constraint.type() + "::" + fuse_core::uuid::to_string(constraint.uuid());
}

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

  // Create constraint relative pose axes:
  relative_pose_axes_node_ = root_node_->createChildSceneNode();
  relative_pose_axes_ = std::make_shared<rviz::Axes>(scene_manager_, relative_pose_axes_node_, 10.0, 1.0);

  // Draw text:
  const auto caption = constraint_name(constraint);
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

  // Set error line color brightness based on the loss function impact on the constraint cost:
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

    if (rho[0] > squared_norm)
    {
      ROS_WARN_STREAM_THROTTLE(10.0, "Detected invalid loss value of "
                                         << rho[0] << " greater than squared residual of " << squared_norm
                                         << " for constraint " << constraint_name(constraint) << " with loss type "
                                         << constraint.loss()->type()
                                         << ". Loss value clamped to the squared residual.");

      rho[0] = squared_norm;
    }

    // Interpolate between the constraint's absolute position and its second variable position by the quotient between
    // the cost with and without loss:
    //
    //              loss_cost      0.5 * rho[0]         rho[0]
    // loss_scale = --------- = ------------------ = ------------
    //                cost      0.5 * squared_norm   squared_norm
    //
    // Remember that in principle `rho[0] <= squared_norm`, with `rho[0] == squared_norm` for the inlier region, and
    // `rho[0] < squared_norm` for the outlier region:
    loss_scale_ = squared_norm == 0.0 ? 1.0 : rho[0] / squared_norm;

    // Compute error line color with the loss function impact:
    const auto loss_error_line_color = computeLossErrorLineColor(error_line_color_, loss_scale_);
    error_line_->setColor(loss_error_line_color.r, loss_error_line_color.g, loss_error_line_color.b,
                          error_line_color_.a);
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

void RelativePose2DStampedConstraintVisual::setLossMinBrightness(const float min_brightness)
{
  min_brightness_ = min_brightness;
}

void RelativePose2DStampedConstraintVisual::setRelativePoseLineColor(const float r, const float g, const float b,
                                                                     const float a)
{
  relative_pose_line_->setColor(r, g, b, a);
}

void RelativePose2DStampedConstraintVisual::setErrorLineColor(const float r, const float g, const float b,
                                                              const float a)
{
  // Cache error line color w/o the loss function impact, so we can change its darkness based on the loss function
  // impact on the constraint cost:
  // Note that we cannot recover/retrieve the color from the Ogre::BillboarrdLine error_line_ because its API does NOT
  // support that.
  error_line_color_.r = r;
  error_line_color_.g = g;
  error_line_color_.b = b;
  error_line_color_.a = a;

  // Compute error line color with the impact of the loss function, in case the constraint has one:
  const auto loss_error_line_color = computeLossErrorLineColor(error_line_color_, loss_scale_);
  error_line_->setColor(loss_error_line_color.r, loss_error_line_color.r, loss_error_line_color.b,
                        loss_error_line_color.a);
}

void RelativePose2DStampedConstraintVisual::setRelativePoseAxesAlpha(const float alpha)
{
  static const auto& default_x_color_ = relative_pose_axes_->getDefaultXColor();
  static const auto& default_y_color_ = relative_pose_axes_->getDefaultYColor();
  static const auto& default_z_color_ = relative_pose_axes_->getDefaultZColor();

  relative_pose_axes_->setXColor(
      Ogre::ColourValue( default_x_color_.r, default_x_color_.g, default_x_color_.b, alpha ));  // NOLINT
  relative_pose_axes_->setYColor(
      Ogre::ColourValue( default_y_color_.r, default_y_color_.g, default_y_color_.b, alpha ));  // NOLINT
  relative_pose_axes_->setZColor(
      Ogre::ColourValue( default_z_color_.r, default_z_color_.g, default_z_color_.b, alpha ));  // NOLINT
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

Ogre::ColourValue RelativePose2DStampedConstraintVisual::computeLossErrorLineColor(const Ogre::ColourValue& color,
                                                                                   const float loss_scale)
{
  // Skip if the loss scale is negative, which means the constraint has no loss:
  if (loss_scale < 0.0)
  {
    return color;
  }

  // Get the error line color as HSB:
  Ogre::ColourValue error_line_color(color.r, color.g, color.b);
  Ogre::Real hue, saturation, brightness;
  error_line_color.getHSB(hue, saturation, brightness);

  // We should correct the color brightness if it is smaller than minimum brightness. Otherwise, we would get an
  // incorrect loss brightness.
  //
  // However, we cannot do this because it changes the color of the error line, which should be consistent for all
  // constraints visuals. Instead, we clamp the minium brightness:
  const auto min_brightness = std::min(min_brightness_, brightness);

  // Scale brightness by the loss scale within the [min_brightness, 1] range:
  const auto loss_brightness = min_brightness + (brightness - min_brightness) * loss_scale;

  // Set error line color with the loss brightness:
  Ogre::ColourValue loss_error_line_color;
  loss_error_line_color.setHSB(hue, saturation, loss_brightness);

  return Ogre::ColourValue(loss_error_line_color.r, loss_error_line_color.g, loss_error_line_color.b, color.a);
}

}  // namespace rviz
