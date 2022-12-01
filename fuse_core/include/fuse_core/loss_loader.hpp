/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Locus Robotics
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
#ifndef FUSE_CORE__LOSS_LOADER_HPP_
#define FUSE_CORE__LOSS_LOADER_HPP_

#include <string>

#include <fuse_core/loss.hpp>
#include <pluginlib/class_loader.hpp>

namespace fuse_core
{

/**
 * @brief Load a loss function using pluginlib::ClassLoader
 *
 * The loader objects loads a loss function using pluginlib::ClassLoader. This is typically run
 * from a sensor or motion model on initialization, so the loss functions are later passed to the
 * constraints they create. Since the class loader must outlive the loss objects it creates, this
 * class implements the singleton pattern.
 */
class LossLoader
{
public:
  /**
   * @brief Get singleton instance
   *
   * @return Loss loader singleton instance
   */
  static LossLoader & getInstance()
  {
    static LossLoader instance;
    return instance;
  }

  // Delete copy and move constructors and assign operators
  LossLoader(const LossLoader &) = delete;
  LossLoader(LossLoader &&) = delete;
  LossLoader & operator=(const LossLoader &) = delete;
  LossLoader & operator=(LossLoader &&) = delete;

  /**
   * @brief Create unique instance of a loss function loaded with pluginlib
   *
   * @param[in] lookup_name Loss function lookup name
   * @return Loss function instance handled by an std::unique_ptr<>
   */
  pluginlib::UniquePtr<fuse_core::Loss> createUniqueInstance(const std::string & lookup_name)
  {
    return loss_loader_.createUniqueInstance(lookup_name);
  }

protected:
  /**
   * @brief Constructor
   */
  LossLoader()
  : loss_loader_("fuse_core", "fuse_core::Loss")
  {
  }

private:
  pluginlib::ClassLoader<fuse_core::Loss> loss_loader_;  //!< Pluginlib class loader for Loss types
};

/**
 * @brief Create an unique instance of a loss function loaded with the singleton Loss loader
 *
 * @param[in] lookup_name Loss function lookup name
 * @return Loss function instance handled by an std::unique_ptr<>
 */
inline pluginlib::UniquePtr<fuse_core::Loss> createUniqueLoss(const std::string & lookup_name)
{
  return LossLoader::getInstance().createUniqueInstance(lookup_name);
}

}  // namespace fuse_core

#endif  // FUSE_CORE__LOSS_LOADER_HPP_
