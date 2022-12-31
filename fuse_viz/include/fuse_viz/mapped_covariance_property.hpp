/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Ellon Paiva Mendes @ LAAS-CNRS
 * All rights reserved.
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

#ifndef FUSE_VIZ__MAPPED_COVARIANCE_PROPERTY_HPP_
#define FUSE_VIZ__MAPPED_COVARIANCE_PROPERTY_HPP_

#include <OgreColourValue.h>
#include <QColor>

#include <memory>
#include <string>
#include <unordered_map>

#include <fuse_viz/mapped_covariance_visual.hpp>

#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/property.hpp>


namespace Ogre
{

class SceneManager;
class SceneNode;

}  // namespace Ogre

namespace fuse_viz
{
/**
 * @brief Property specialized to provide getter for booleans.
 *
 * This is mostly a copy of CovarianceProperty from rviz/default_plugin/covariance_property.h that
 * instead of using an std::deque to store the visuals, it uses an std::unordered_map, so the
 * visuals can be indexed by their UUID.
 */
class MappedCovarianceProperty : public rviz_common::properties::BoolProperty
{
  Q_OBJECT

public:
  typedef std::shared_ptr<MappedCovarianceVisual> MappedCovarianceVisualPtr;

  enum Frame
  {
    Local,
    Fixed,
  };

  enum ColorStyle
  {
    Unique,
    RGB,
  };

  MappedCovarianceProperty(
    const QString & name = "Covariance", bool default_value = false,
    const QString & description = QString(), rviz_common::properties::Property * parent = 0,
    const char * changed_slot = 0, QObject * receiver = 0);

  virtual ~MappedCovarianceProperty();

  bool getPositionBool();
  bool getOrientationBool();

  // Methods to manage the unordered map of Covariance Visuals
  MappedCovarianceVisualPtr createAndInsertVisual(
    const std::string & key, Ogre::SceneManager * scene_manager,
    Ogre::SceneNode * parent_node);
  void eraseVisual(const std::string & key);
  void clearVisual();
  size_t sizeVisual();

public Q_SLOTS:
  void updateVisibility();

private Q_SLOTS:
  void updateColorAndAlphaAndScaleAndOffset();
  void updateOrientationFrame();
  void updateColorStyleChoice();

private:
  void updateColorAndAlphaAndScaleAndOffset(const MappedCovarianceVisualPtr & visual);
  void updateOrientationFrame(const MappedCovarianceVisualPtr & visual);
  void updateVisibility(const MappedCovarianceVisualPtr & visual);

  std::unordered_map<std::string, MappedCovarianceVisualPtr> covariances_;

  rviz_common::properties::BoolProperty * position_property_;
  rviz_common::properties::ColorProperty * position_color_property_;
  rviz_common::properties::FloatProperty * position_alpha_property_;
  rviz_common::properties::FloatProperty * position_scale_property_;
  rviz_common::properties::BoolProperty * orientation_property_;
  rviz_common::properties::EnumProperty * orientation_frame_property_;
  rviz_common::properties::EnumProperty * orientation_colorstyle_property_;
  rviz_common::properties::ColorProperty * orientation_color_property_;
  rviz_common::properties::FloatProperty * orientation_alpha_property_;
  rviz_common::properties::FloatProperty * orientation_offset_property_;
  rviz_common::properties::FloatProperty * orientation_scale_property_;
};

}  // namespace fuse_viz

#endif  // FUSE_VIZ__MAPPED_COVARIANCE_PROPERTY_HPP_
