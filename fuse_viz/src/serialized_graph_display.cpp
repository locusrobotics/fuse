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

#include <rviz/ogre_helpers/axes.h>
#include <rviz/ogre_helpers/billboard_line.h>
#include <rviz/ogre_helpers/object.h>
#include <rviz/ogre_helpers/shape.h>

#include <rviz/display_context.h>
#include <rviz/frame_manager.h>

#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#endif  // Q_MOC_RUN

#include <fuse_viz/serialized_graph_display.h>

#include <fuse_core/graph.h>
#include <fuse_variables/orientation_2d_stamped.h>
#include <fuse_variables/position_2d_stamped.h>

namespace rviz
{

SerializedGraphDisplay::SerializedGraphDisplay()
{
  draw_variables_axes_property_ = new BoolProperty("Draw Variables as Axes", true,
                                                   "Whether to draw graph variables axes or not (spheres are always "
                                                   "drawn)",
                                                   this, SLOT(updateDrawVariablesAxesProperty()));

  scale_property_ = new FloatProperty("Scale", 0.1, "Scale of graph markers drawn", this, SLOT(updateScaleProperty()));
}

SerializedGraphDisplay::~SerializedGraphDisplay()
{
  if (initialized())
  {
    clear();
    root_node_->removeAndDestroyAllChildren();
    scene_manager_->destroySceneNode(root_node_->getName());
  }
}

void SerializedGraphDisplay::reset()
{
  MFDClass::reset();
  clear();
}

void SerializedGraphDisplay::onInitialize()
{
  MFDClass::onInitialize();

  root_node_ = scene_node_->createChildSceneNode();

  variables_axes_node_ = root_node_->createChildSceneNode();
  variables_spheres_node_ = root_node_->createChildSceneNode();
}

void SerializedGraphDisplay::onEnable()
{
  MFDClass::onEnable();

  root_node_->setVisible(true);
}

void SerializedGraphDisplay::onDisable()
{
  MFDClass::onDisable();

  root_node_->setVisible(false);
}

void SerializedGraphDisplay::updateDrawVariablesAxesProperty()
{
  variables_axes_node_->setVisible(draw_variables_axes_property_->getBool());
}

void SerializedGraphDisplay::updateScaleProperty()
{
  // TODO(efernandez) Redraw all variables so the scale is applied to the current graph. This is useful if no new
  // message is received
}

void SerializedGraphDisplay::clear()
{
  for (auto& graph_shape : graph_shapes_)
  {
    delete graph_shape;
  }
  graph_shapes_.clear();
}

void SerializedGraphDisplay::processMessage(const fuse_msgs::SerializedGraph::ConstPtr& msg)
{
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(msg->header, position, orientation))
  {
    ROS_DEBUG_STREAM("Error transforming from frame '" << msg->header.frame_id << "' to frame '"
                                                       << qPrintable(fixed_frame_) << "'");
  }

  root_node_->setPosition(position);
  root_node_->setOrientation(orientation);

  clear();

  const auto graph = graph_deserializer_.deserialize(msg);

  const Ogre::Vector3 scale(scale_property_->getFloat());

  for (const auto& variable : graph->getVariables())
  {
    const auto orientation = dynamic_cast<const fuse_variables::Orientation2DStamped*>(&variable);
    if (!orientation)
    {
      continue;
    }

    const auto& stamp = orientation->stamp();
    const auto position_uuid = fuse_variables::Position2DStamped(stamp, orientation->deviceId()).uuid();
    if (!graph->variableExists(position_uuid))
    {
      continue;
    }

    const auto position = dynamic_cast<const fuse_variables::Position2DStamped*>(&graph->getVariable(position_uuid));

    const tf2::Quaternion tf_q(tf2::Vector3(0, 0, 1), orientation->yaw());

    const Ogre::Vector3 p(position->x(), position->y(), 0);
    const Ogre::Quaternion q(tf_q.w(), tf_q.x(), tf_q.y(), tf_q.z());

    // Add sphere:
    Object* sphere = new rviz::Shape(rviz::Shape::Sphere, scene_manager_, variables_spheres_node_);
    sphere->setPosition(p);
    sphere->setScale(scale);
    sphere->setColor(1.0, 0.0, 0.0, 1.0);
    graph_shapes_.push_back(sphere);

    // Add axes:
    Object* axes = new rviz::Axes(scene_manager_, variables_axes_node_, 10.0, 1.0);
    axes->setPosition(p);
    axes->setOrientation(q);
    axes->setScale(scale);
    graph_shapes_.push_back(axes);
  }

  updateDrawVariablesAxesProperty();
}

}  // namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::SerializedGraphDisplay, rviz::Display)
