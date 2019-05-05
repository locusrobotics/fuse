/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Locus Robotics
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
#ifndef FUSE_CORE_CALLBACK_WRAPPER_H
#define FUSE_CORE_CALLBACK_WRAPPER_H

#include <ros/callback_queue_interface.h>

#include <functional>
#include <future>


namespace fuse_core
{

/**
 * @brief Object that wraps a generic function call so that it may be inserted into a ROS callback queue.
 *
 * Once inserted, the function will be called from within the thread that services that callback queue. It will be
 * intermingled with any other standard ROS callbacks (message subscription callbacks, service callbacks, timer
 * callbacks, etc.), analogous to adding callbacks for an additional topic subscriber.
 *
 * boost::bind/std::bind can be used to provide input arguments. Note that bind() uses a pass-by-value mechanism. If
 * arguments should be passed by reference instead (i.e. copies are expensive or the function modifies the input in
 * place), wrap the input arguments with boost::ref() or std::ref().
 *
 * The CallbackWrapper class uses the C++11 promise/future mechanism to provide the return data after the callback has
 * been executed in the callback queue's thread. This is done by getting a future<T> from the CallbackWrapper before it
 * is inserted into the callback queue. You can then block until the return value is ready by calling future.wait(),
 * and the value can be retrieved using future.get(). Even if the function returns void, it can still be useful to get
 * a copy of the CallbackWrapper's future. The future.wait() function is still valid for void types, thus allowing you
 * to block until the callback function has been executed.
 *
 * See C++11 documentation for more details on promises and futures:
 * - http://en.cppreference.com/w/cpp/thread/promise
 * - http://en.cppreference.com/w/cpp/thread/future
 *
 * Example usage:
 * @code{.cpp}
 * class MyClass
 * {
 * public:
 *   double processData(const std::vector<double>& data)
 *   {
 *     return std::accumulate(data.begin(), data.end(), 0.0);
 *   }
 * };
 *
 * MyClass my_object;
 * std::vector<double> really_big_data(1000000);
 * auto callback = boost::make_shared<CallbackWrapper<double> >(
 *   std::bind(&MyClass::processData, &my_object, std::ref(really_big_data)));
 * std::future<double> result = callback->getFuture();
 * ros::getGlobalCallbackQueue()->addCallback(callback);
 * result.wait();
 * ROS_INFO_STREAM("The result is: " << result.get());
 * @endcode
 */
template <typename T>
class CallbackWrapper : public ros::CallbackInterface
{
public:
  using CallbackFunction = std::function<T(void)>;

  /**
   * @brief Constructor
   *
   * @param[in] callback The function to be called from the callback queue
   */
  explicit CallbackWrapper(CallbackFunction callback) :
    callback_(callback)
  {
  }

  /**
   * @brief Get a future<T> object that represents the function's return value
   */
  std::future<T> getFuture()
  {
    return promise_.get_future();
  }

  /**
   * @brief Call this function. This is used by the callback queue.
   */
  CallResult call() override
  {
    promise_.set_value(callback_());
    return Success;
  }

private:
  CallbackFunction callback_;  //!< The function to execute within the
  std::promise<T> promise_;  //!< Promise/Future used to return data after the callback is executed
};

// Specialization to handle 'void' return types
// Specifically, promise_.set_value(callback_()) does not work if callback_() returns void.
template <>
inline ros::CallbackInterface::CallResult CallbackWrapper<void>::call()
{
  callback_();
  promise_.set_value();
  return Success;
}

}  // namespace fuse_core

#endif  // FUSE_CORE_CALLBACK_WRAPPER_H
