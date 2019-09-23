/*
 * Copyright (c) 2017, Ellon Paiva Mendes @ LAAS-CNRS
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef FUSE_VIZ_MAPPED_COVARIANCE_VISUAL_H
#define FUSE_VIZ_MAPPED_COVARIANCE_VISUAL_H

#include <rviz/default_plugin/covariance_visual.h>

namespace Ogre
{

class SceneManager;
class SceneNode;

}  // namespace Ogre

namespace rviz
{

class Shape;
class MappedCovarianceProperty;

/**
 * \class MappedCovarianceVisual
 * \brief MappedCovarianceVisual consisting in a ellipse for position and 2D ellipses along the axis for orientation.
 *
 * This is mostly a copy of CovarianceVisual from rviz/default_plugin/covariance_visual.h that allows
 * MappedCovarianceProperty be a friend class of MappedCovarianceVisual, so it can call its constructor.
 */
class MappedCovarianceVisual : public rviz::CovarianceVisual
{
public:
  /**
   * \brief Constructor
   *
   * MappedCovarianceVisual can only be constructed by friend class MappedCovarianceProperty.
   *
   * @param scene_manager The scene manager to use to construct any necessary objects
   * @param parent_object A rviz object that this covariance will be attached.
   * @param is_local_rotation Initial attachment of the rotation part
   * @param is_visible Initial visibility
   * @param pos_scale Scale of the position covariance
   * @param ori_scale Scale of the orientation covariance
   */
  MappedCovarianceVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node, bool is_local_rotation,
                         bool is_visible = true, float pos_scale = 1.0f, float ori_scale = 0.1f,
                         float ori_offset = 0.1f)
    : CovarianceVisual(scene_manager, parent_node, is_local_rotation, is_visible, pos_scale, ori_scale, ori_offset)
  {
  }

private:
  // Make MappedCovarianceProperty friend class so it create MappedCovarianceVisual objects
  friend class MappedCovarianceProperty;
};

}  // namespace rviz

#endif  // FUSE_VIZ_MAPPED_COVARIANCE_VISUAL_H
