/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Brett Downing
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

#include <fuse_core/callback_wrapper.h>

namespace fuse_core
{


  CallbackAdapter::CallbackAdapter(std::shared_ptr<rclcpp::Context> context_ptr){

    rcl_guard_condition_options_t guard_condition_options =
        rcl_guard_condition_get_default_options();

    // Guard condition is used by the wait set to handle execute-or-not logic
    gc_ = rcl_get_zero_initialized_guard_condition();
    if (RCL_RET_OK != rcl_guard_condition_init(
      &gc_, context_ptr->get_rcl_context().get(), guard_condition_options)
    ) {
      RCLCPP_WARN(rclcpp::get_logger("fuse"),
                  "Could not init guard condition for callback waitable.");
    }
  }

  /**
   * @brief tell the CallbackGroup how many guard conditions are ready in this waitable
   */
  size_t CallbackAdapter::get_number_of_ready_guard_conditions() { return 1;}


  /**
   * @brief tell the CallbackGroup that this waitable is ready to execute anything
   */
  bool CallbackAdapter::is_ready(rcl_wait_set_t * wait_set) {
    (void) wait_set;
    // std::lock_guard<std::recursive_mutex> lock(queue_mutex_);
    // a thread glitch isn't a disaster here and a mutex wouldn't stop it anyway
    return !callback_queue_.empty();
  }


  /**
   * @brief add_to_wait_set is called by rclcpp during NodeWaitables::add_waitable() and CallbackGroup::add_waitable()
    waitable_ptr = std::make_shared<CallbackAdapter>();
    node->get_node_waitables_interface()->add_waitable(waitable_ptr, (rclcpp::CallbackGroup::SharedPtr) nullptr);
   */
  void CallbackAdapter::add_to_wait_set(rcl_wait_set_t * wait_set)
  {
    std::lock_guard<std::recursive_mutex> lock(reentrant_mutex_);
    RCLCPP_WARN(rclcpp::get_logger("fuse"),
    "Could not add callback waitable to wait set.");
    if (RCL_RET_OK != rcl_wait_set_add_guard_condition(wait_set, &gc_, NULL)) {
    }
  }

  /**
   * @brief unused part of the rclcpp::waitables interface
   *
   */
  std::shared_ptr< void > CallbackAdapter::take_data(){
    return nullptr;
  }

  /**
   * @brief hook that allows the rclcpp::waitables interface to run the next callback
   *
   */
  void CallbackAdapter::execute(std::shared_ptr<void> & /*data*/){
    std::shared_ptr<CallbackWrapperBase> cb_wrapper = nullptr;
    // fetch the callback ptr and release the lock without spending time in the callback
    {
      std::lock_guard<std::recursive_mutex> lock(queue_mutex_);
      if(!callback_queue_.empty()){
        cb_wrapper = callback_queue_.front();
        callback_queue_.pop_front();
      }
    }
    //the lock is released and the callback is no longer associated with the queue, run it.
    if(cb_wrapper) {
      cb_wrapper->call();
    }
  }

  void CallbackAdapter::addCallback(const std::shared_ptr<CallbackWrapperBase> &callback){
    std::lock_guard<std::recursive_mutex> lock(queue_mutex_);
    callback_queue_.push_back(callback);
    if (RCL_RET_OK != rcl_trigger_guard_condition(&gc_)) {
      RCLCPP_WARN(rclcpp::get_logger("fuse"),
                  "Could not trigger guard condition for callback, pulling callback off the queue.");
      callback_queue_.pop_back();  // Undo
    }
  }

  void CallbackAdapter::addCallback(std::shared_ptr<CallbackWrapperBase> && callback){
    std::lock_guard<std::recursive_mutex> lock(queue_mutex_);
    callback_queue_.push_back(std::move(callback));
    if (RCL_RET_OK != rcl_trigger_guard_condition(&gc_)) {
      RCLCPP_WARN(rclcpp::get_logger("fuse"),
                  "Could not trigger guard condition for callback, pulling callback off the queue.");
      callback_queue_.pop_back();  // Undo
    }
  }

  void CallbackAdapter::removeAllCallbacks(){
    std::lock_guard<std::recursive_mutex> lock(queue_mutex_);
    while(!callback_queue_.empty()){
      callback_queue_.pop_front();
    }
  }




}  // namespace fuse_core
