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
 * auto callback_queue = std::make_shared<CallbackAdapter>(
 *   rclcpp::contexts::default_context::get_global_default_context());
 * node->get_node_waitables_interface()->add_waitable(callback_queue, (rclcpp::CallbackGroup::SharedPtr) nullptr);
 * MyClass my_object;
 * std::vector<double> really_big_data(1000000);
 * auto callback = boost::make_shared<CallbackWrapper<double> >(
 *   std::bind(&MyClass::processData, &my_object, std::ref(really_big_data)));
 * callback_queue->add_callback(callback);
 * std::future<double> result = callback->getFuture();
 * result.wait();
 * RCLCPP_INFO_STREAM(rclcpp::get_logger("fuse"), "The result is: " << result.get());
 * @endcode
 */


template <typename T>
class CallbackWrapper 
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
  void call() override
  {
    promise_.set_value(callback_());
  }

private:
  CallbackFunction callback_;  //!< The function to execute within the
  std::promise<T> promise_;  //!< Promise/Future used to return data after the callback is executed
};

// Specialization to handle 'void' return types
// Specifically, promise_.set_value(callback_()) does not work if callback_() returns void.
template <>
inline void CallbackWrapper<void>::call()
{
  callback_();
  promise_.set_value();
}


class CallbackAdapter : public rclcpp::Waitable
{
public:

  explicit CallbackAdapter(std::shared_ptr<rclcpp::Context> context_ptr){

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
    (void) wait_set;
    // std::lock_guard<std::recursive_mutex> lock(queue_mutex_);
    // a thread glitch isn't a disaster here and a mutex wouldn't stop it anyway
    return !callback_queue_.empty();
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
    return RCL_RET_OK == ret;
  }


  // XXX check this against the threading model of the multi-threaded executor.
  void execute(){
    std::shared_ptr<CallbackWrapper> cb_wrapper = nullptr;
    // fetch the callback ptr and release the lock without spending time in the callback
    {
      std::lock_guard<std::recursive_mutex> lock(queue_mutex_);
      if(!callback_queue_.empty()){
        cb_wrapper = callback_queue_.front();
        callback_queue_.pop();
      }
    }
    //the lock is released and the callback is no longer associated with the queue, run it.
    if(cb_wrapper) {
      cb_wrapper->call();
    }
  }

  void addCallback(const std::shared_ptr<CallbackWrapper> &callback){
    std::lock_guard<std::recursive_mutex> lock(queue_mutex_);
    callback_queue_.push(callback);
    rcl_trigger_guard_condition(&gc_);
  }
  void addCallback(std::shared_ptr<CallbackWrapper> && callback){
    std::lock_guard<std::recursive_mutex> lock(queue_mutex_);
    callback_queue_.push(std::move(callback));
    rcl_trigger_guard_condition(&gc_);
  }

  void removeAllCallbacks(){
    std::lock_guard<std::recursive_mutex> lock(queue_mutex_);
    while(!callback_queue_.empty()){
      callback_queue_.pop();
    }
  }


private:
  std::recursive_mutex reentrant_mutex_;  //!< mutex to allow this callback to be added to multiple callback groups simultaneously
  rcl_guard_condition_t gc_;  //!< guard condition to drive the waitable
  
  std::recursive_mutex queue_mutex_;  //!< mutex to allow this callback to be added to multiple callback groups simultaneously
  std::queue<std::shared_ptr<CallbackWrapper> > callback_queue_;
};


}  // namespace fuse_core

#endif  // FUSE_CORE_CALLBACK_WRAPPER_H
