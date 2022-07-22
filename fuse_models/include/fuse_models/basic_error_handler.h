/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Locus Robotics
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
#ifndef FUSE_MODELS_BASIC_ERROR_HANDLER_H
#define FUSE_MODELS_BASIC_ERROR_HANDLER_H

#include <fuse_core/fuse_macros.h>
#include <fuse_core/error_handler.h>

#include <string>


namespace fuse_models
{

/**
 * @brief The interface definition for error handler plugins in the fuse ecosystem.
 *
 * An error handler plugin is responsible for handling any errors that may come up in execution of the program.
 */
class BasicErrorHandler: public fuse_core::ErrorHandler
{
public:
  FUSE_SMART_PTR_ALIASES_ONLY(BasicErrorHandler);

  /**
   * @brief Default Constructor
   */
  BasicErrorHandler();

  /**
   * @brief Destructor
   */
  ~BasicErrorHandler() = default;

  /**
   * @brief Perform any required post-construction initialization, such as subscribing to topics or reading from the
   * parameter server.
   *
   * This will be called on each plugin after construction, and after the ros node has been initialized. Plugins are
   * encouraged to subnamespace any of their parameters to prevent conflicts and allow the same plugin to be used
   * multiple times with different settings and topics.
   *
   * @param[in] name A unique name to give this plugin instance
   */
  void initialize(const std::string& name) override;
  
  /**
   * @brief Handler to be invoked whenever a runtime error occurs
   *
   * @param[in] info Information provided about what specifically occurred
   */
  void runtimeError(const std::string& info) override;

  /**
   * @brief Handler to be invoked whenever invalid arguments are given to a function
   *
   * @param[in] info Information provided about what specifically occurred
   */
  void invalidArgument(const std::string& info) override;

  /**
   * @brief Handler to be invoked whenever an out of range error occurs
   *
   * @param[in] info Information provided about what specifically occurred
   */
  void outOfRangeError(const std::string& info) override;

  /**
   * @brief Handler to be invoked whenever a logic error occurs
   *
   * @param[in] info Information provided about what specifically occurred
   */
  void logicError(const std::string& info) override;


  /**
   * @brief Get the unique name of this error handler
   */
  const std::string& name() const override;
};

}  // namespace fuse_core

#endif  // FUSE_MODELS_BASIC_ERROR_HANDLER_H
