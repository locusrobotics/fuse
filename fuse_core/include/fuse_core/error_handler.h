/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, Locus Robotics
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

#ifndef FUSE_CORE_ERROR_HANDLER_H
#define FUSE_CORE_ERROR_HANDLER_H

#include "ros/ros.h"
#include <string>

/**
 * @brief A class that handles errors that can occur in fuse. 
 * 
 * By default, fuse optimizers handle any errors that come up by throwing a relevant exception. However,
 * while this is generically exceptable, in practical systems it's sometimes not sufficient to just 
 * throw exceptions and hope that the rest of the system goes on while fuse resets and fixes itself.
 * 
 * This class exists to allow for handling generic classes of errors using whatever callback the user sees fit to use. 
 * I recommend registering new callbacks that are bound to an Optimizer instance for more specific functionality.
 * 
 * A note: By default, the base Optimizer class registers the basic callbacks that throw error specific exceptions. 
 * basicCallback below should never be used, and is only provided for simplifying initialisation.
 */

namespace fuse_core
{

/**
 * @brief Used to provide different error handlers for a given context
 */
enum class Context: int
{
  CORE = 0,
  CONSTRAINT = 1,
  GRAPH = 2,
  LOSS = 3,
  MODEL = 4,
  OPTIMIZER = 5,
  PUBLISHER = 6,
  VARIABLE = 7,
  VIZ = 8
};

class ErrorHandler
{
  public:
  static ErrorHandler& getHandler()
  {
    static ErrorHandler handler {};
    return handler;
  }


  void invalidArgument(const std::string& info)
  {
    invalid_argument_cb_(info);
  }

  void runtimeError(const std::string& info)
  {
    runtime_error_cb_(info);
  }

  void outOfRangeError(const std::string& info)
  {
    out_of_range_cb_(info);
  }

  void logicError(const std::string& info)
  {
    logic_error_cb_(info);
  }

  void registerinvalidArgumentErrorCb(std::function<void(const std::string&)> cb)
  {
    invalid_argument_cb_ = cb;
  }

  void registerRuntimeErrorCb(std::function<void(const std::string&)> cb)
  {
      runtime_error_cb_ = cb;
  }

  void registerOutOfRangeErrorCb(std::function<void(const std::string&)> cb)
  {
      out_of_range_cb_ = cb;
  }

  void registerLogicErrorCb(std::function<void(const std::string&)> cb)
  {
      logic_error_cb_ = cb;
  }

  private:
  ErrorHandler()
  {
    invalid_argument_cb_ = invalidArgumentCallback;
    runtime_error_cb_ = runtimeErrorCallback;
    out_of_range_cb_ = outOfRangeErrorCallback;
    logic_error_cb_ = logicErrorCallback;
  }

  ~ErrorHandler() {}
  std::function<void(const std::string&)> invalid_argument_cb_;
  std::function<void(const std::string&)> runtime_error_cb_;
  std::function<void(const std::string&)> out_of_range_cb_;
  std::function<void(const std::string&)> logic_error_cb_;


  /*
  * Default error handling behavior
  */
  static void invalidArgumentCallback(std::string info)
  {
    throw std::invalid_argument(info);
  }
  static void logicErrorCallback(std::string info)
  {
    throw std::logic_error(info);
  }
  static void runtimeErrorCallback(std::string info)
  {
    throw std::runtime_error(info);
  }
  static void outOfRangeErrorCallback(std::string info)
  {
    throw std::out_of_range(info);
  }
};

}  // namespace fuse_core


#endif  // FUSE_CORE_ERROR_HANDLER_H
