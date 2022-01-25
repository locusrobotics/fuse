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
 * based on https://answers.ros.org/question/322815/ros2-how-to-create-custom-waitable/
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
 * auto callback = std::make_shared<CallbackWrapper<double> >(
 *   std::bind(&MyClass::processData, &my_object, std::ref(really_big_data)));
 * std::future<double> result = callback->getFuture();
 * node->get_node_waitables_interface()->add_waitable(callback, (rclcpp::CallbackGroup::SharedPtr) nullptr);
 * result.wait();
 * ROS_INFO_STREAM("The result is: " << result.get());
 * // XXX the shared pointer callback will be erased from the queue if it isn't held in scope under this class
 * // XXX If the callback waitable remains in scope after the future is delivered, it'll get run a second time.
 * // XXX a better fix would be implement a ros1 style callback queue in a ros2 waitable
 * // This waitable wrapper
 * @endcode
 */


class CallbackAdapter : public rclcpp::Waitable
{
  // implement the API of the ros CallbackQueue such that it can be implemented in a ros2 CallbackGroup


};



template <typename T>
class CallbackWrapper : public rclcpp::Waitable
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
    // XXX expose the rclcpp::Context
    std::shared_ptr<rclcpp::Context> context_ptr = 
        rclcpp::contexts::default_context::get_global_default_context();

    rcl_guard_condition_options_t guard_condition_options = 
        rcl_guard_condition_get_default_options();

    // Guard condition is used by the wait set to handle execute-or-not logic
    gc_ = rcl_get_zero_initialized_guard_condition();
    rcl_ret_t ret = rcl_guard_condition_init(
        &gc_, context_ptr->get_rcl_context().get(), guard_condition_options);
  }

  /**
   * @brief tell the CallbackGroup how many guard conditions are ready in this waitable
   */
  size_t get_number_of_ready_guard_conditions() { return 1;}

  /**
   * @brief tell the CallbackGroup that this waitable is ready to execute anything
   */
  bool is_ready(rcl_wait_set_t * wait_set) {
    // XXX return callback_queue_.empty() == false;
    return true;
  }

  /**
   * @brief add_to_wait_set is called by rclcpp during NodeWaitables::add_waitable() and CallbackGroup::add_waitable()
    waitable_ptr = std::make_shared<CallbackWrapper>();
    node->get_node_waitables_interface()->add_waitable(waitable_ptr, (rclcpp::CallbackGroup::SharedPtr) nullptr);
   */
  bool add_to_wait_set(rcl_wait_set_t * wait_set)
  {
    std::lock_guard<std::recursive_mutex> lock(reentrant_mutex_);
    rcl_ret_t ret = rcl_wait_set_add_guard_condition(wait_set, &gc_, NULL);

    //trigger execution immediately
    rcl_trigger_guard_condition(&gc_);
    return RCL_RET_OK == ret;
  }

  /**
   * @brief Get a future<T> object that represents the function's return value
   */
  std::future<T> getFuture()
  {
    return promise_.get_future();
  }

  /**
   * @brief Call this function. This is used by the CallbackGroup.
   */
  void execute() override
  {
    promise_.set_value(callback_());
    return Success;
  }

private:
  std::recursive_mutex reentrant_mutex_;  //!< mutex to allow this callback to be added to multiple callback groups simultaneously
  rcl_guard_condition_t gc_;  //!< guard condition to drive the waitable
  CallbackFunction callback_;  //!< The function to execute within the CallbackGroup
  std::promise<T> promise_;  //!< Promise/Future used to return data after the callback is executed
};

// Specialization to handle 'void' return types
// Specifically, promise_.set_value(callback_()) does not work if callback_() returns void.
template <>
inline void CallbackWrapper<void>::execute()
{
  callback_();
  promise_.set_value();
  return Success;
}

}  // namespace fuse_core

#endif  // FUSE_CORE_CALLBACK_WRAPPER_H
