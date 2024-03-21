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
#include <utility>
#include <unordered_map>

/**
 * @brief A class that handles errors that can occur in fuse. 
 * 
 * By default, fuse optimizers handle any errors that come up by throwing a relevant exception. However,
 * while this is generically exceptable, in practical systems it's sometimes not sufficient to just 
 * throw exceptions and hope that the rest of the system goes on while fuse resets and fixes itself.
 * 
 * This class exists to allow for handling generic classes of errors using whatever callback the user sees fit to use. 
 * The default behavior is to throw the relevant exception. Any function that has void return and accepts a string
 * can be used for this, however.
 * 
 * The error handler also allows for different error handlers to be used in different contexts. It assumes that unless
 * otherwise specified, all error handling is done generically. However, errors in particular contexts can be handled
 * with unique callbacks as needed. 
 * 
 * It's recommended that you register any needed error non-default callbacks during Optimizer startup.
 */

namespace fuse_core
{

/**
 * @brief Used to provide different error handlers for a given context. 
 */
enum class ErrorContext: int
{
  GENERIC,
  CORE,
  CONSTRAINT,
  GRAPH,
  LOSS,
  MODEL,
  OPTIMIZER,
  PUBLISHER,
  VARIABLE,
  VIZ
};

class ErrorHandler
{
  using ErrorCb = std::function<void(const std::string&)>;
  public:
  /**
   * @brief Gets a reference to the error handler.
   *
   * @param[out] handler - reference to the error handler
   */
  static ErrorHandler& getHandler()
  {
    static ErrorHandler handler {};
    return handler;
  }

  /**
   * @brief Use whatever callback is registered to handle an invalid argument error for a given context.
   *
   * @param[in] info      A string containing information about the error.
   * @param[in] context   The Error Context, which dictates what callback is used. Defaults to ErrorContext::Generic
   * @throws std::out_of_range if no callback has been registered for the provided context
   */
  void invalidArgument(const std::string& info, ErrorContext context = ErrorContext::GENERIC)
  {
    try
    {
      auto cb = invalid_argument_cbs_.at(context);
      cb(info);
    }
    catch(const std::out_of_range& e)
    {
      throw std::out_of_range("Failed to call invalidArgument Error handling, no callback found for error " + info);
    }
  }
  /**
   * @brief Use whatever callback is registered to handle a runtime error for a given context.
   *
   * @param[in] info      A string containing information about the error.
   * @param[in] context   The Error Context, which dictates what callback is used. Defaults to ErrorContext::Generic
   * @throws std::out_of_range if no callback has been registered for the provided context
   */
  void runtimeError(const std::string& info, ErrorContext context = ErrorContext::GENERIC)
  {
    try
    {
      auto cb = runtime_error_cbs_.at(context);
      cb(info);
    }
    catch(const std::out_of_range& e)
    {
      throw std::out_of_range("Failed to call runtimeError Error handling, no callback found for error " + info);
    }
  }

  /**
   * @brief Use whatever callback is registered to handle an out of range error for a given context.
   *
   * @param[in] info      A string containing information about the error.
   * @param[in] context   The Error Context, which dictates what callback is used. Defaults to ErrorContext::Generic
   * @throws std::out_of_range if no callback has been registered for the provided context
   */
  void outOfRangeError(const std::string& info, ErrorContext context = ErrorContext::GENERIC)
  {
    try
    {
      auto cb = out_of_range_cbs_.at(context);
      cb(info);
    }
    catch(const std::out_of_range& e)
    {
      throw std::out_of_range("Failed to call outOfRangeError handling, no callback found for error " + info);
    }
  }

  /**
   * @brief Use whatever callback is registered to handle a logic error for a given context.
   *
   * @param[in] info      A string containing information about the error.
   * @param[in] context   The Error Context, which dictates what callback is used. Defaults to ErrorContext::Generic
   * @throws std::out_of_range if no callback has been registered for the provided context
   */
  void logicError(const std::string& info, ErrorContext context = ErrorContext::GENERIC)
  {
    try
    {
      auto cb = logic_error_cbs_.at(context);
      cb(info);
    }
    catch(const std::out_of_range& e)
    {
      throw std::out_of_range("Failed to call logicError handling, no callback found for error " + info);
    }
  }

  /**
   * @brief Register a callback to be used for invalid argument errors in a particular context
   *
   * @param[in] cb        The callback to be registered
   * @param[in] context   The Error Context, which dictates what callback is used. Defaults to ErrorContext::Generic
   */
  void registerinvalidArgumentErrorCb(ErrorCb cb, ErrorContext context = ErrorContext::GENERIC)
  {
    invalid_argument_cbs_[context] = cb;
  }

  /**
   * @brief Register a callback to be used for runtime errors in a particular context
   *
   * @param[in] cb        The callback to be registered
   * @param[in] context   The Error Context, which dictates what callback is used. Defaults to ErrorContext::Generic
   */
  void registerRuntimeErrorCb(ErrorCb cb, ErrorContext context = ErrorContext::GENERIC)
  {
    runtime_error_cbs_[context] = cb;
  }

  /**
   * @brief Register a callback to be used for out of range errors in a particular context
   *
   * @param[in] cb        The callback to be registered
   * @param[in] context   The Error Context, which dictates what callback is used. Defaults to ErrorContext::Generic
   */
  void registerOutOfRangeErrorCb(ErrorCb cb, ErrorContext context = ErrorContext::GENERIC)
  {
    out_of_range_cbs_[context] = cb;
  }

  /**
   * @brief Register a callback to be used for logic errors in a particular context
   *
   * @param[in] cb        The callback to be registered
   * @param[in] context   The Error Context, which dictates what callback is used. Defaults to ErrorContext::Generic
   */
  void registerLogicErrorCb(ErrorCb cb, ErrorContext context = ErrorContext::GENERIC)
  {
    logic_error_cbs_[context] = cb;
  }

  // Should be used only for teardown and setup during unit tests
  void reset()
  {
    invalid_argument_cbs_.clear();
    runtime_error_cbs_.clear();
    out_of_range_cbs_.clear();
    logic_error_cbs_.clear();

    invalid_argument_cbs_.insert(std::make_pair(ErrorContext::GENERIC, invalidArgumentCallback));
    runtime_error_cbs_.insert(std::make_pair(ErrorContext::GENERIC, runtimeErrorCallback));
    out_of_range_cbs_.insert(std::make_pair(ErrorContext::GENERIC, outOfRangeErrorCallback));
    logic_error_cbs_.insert(std::make_pair(ErrorContext::GENERIC, logicErrorCallback));
  }

  private:
  ErrorHandler()
  {
    invalid_argument_cbs_[ErrorContext::GENERIC] = invalidArgumentCallback;
    runtime_error_cbs_[ErrorContext::GENERIC] = runtimeErrorCallback;
    out_of_range_cbs_[ErrorContext::GENERIC] = outOfRangeErrorCallback;
    logic_error_cbs_[ErrorContext::GENERIC] = logicErrorCallback;
  }

  ~ErrorHandler() {}

  std::unordered_map<ErrorContext, ErrorCb> invalid_argument_cbs_;
  std::unordered_map<ErrorContext, ErrorCb> runtime_error_cbs_;
  std::unordered_map<ErrorContext, ErrorCb> out_of_range_cbs_;
  std::unordered_map<ErrorContext, ErrorCb> logic_error_cbs_;

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
