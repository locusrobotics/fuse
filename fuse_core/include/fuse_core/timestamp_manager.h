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
#ifndef FUSE_CORE_TIMESTAMP_MANAGER_H
#define FUSE_CORE_TIMESTAMP_MANAGER_H

#include <fuse_core/constraint.h>
#include <fuse_core/macros.h>
#include <fuse_core/transaction.h>
#include <fuse_core/variable.h>
#include <ros/duration.h>
#include <ros/time.h>

#include <boost/range/any_range.hpp>

#include <functional>
#include <map>
#include <set>
#include <vector>


namespace fuse_core
{

/**
 * @brief A utility class that manages the set of timestamps that have been used to generate motion model constraints.
 *
 * This class keeps track of the timestamps of the previously generated motion model chain, and determines the
 * required start/stop timestamp pairs needed to update the motion model chain to span the queried time range. If the
 * queried time range does not line up with existing constraints, this class with mark some prior constraints for
 * removal and will generate new start/stop timestamp pairs that split the previous range and the requested time. All
 * of the details and bookkeeping needed for creating a proper motion model implementation are handled by this class;
 * derived motion models simply need to include a TimestampManager object and provide a function capable of generating
 * motion model constraints between arbitrary timestamps.
 */
class TimestampManager
{
public:
  SMART_PTR_DEFINITIONS(TimestampManager);

  /**
   * @brief Function that generates motion model constraints between the requested timestamps
   *
   * If the motion models cannot be generated for whatever reason, this function should throw an exception.
   *
   * @param[in]  beginning_stamp The beginning timestamp of the motion model constraints to be generated.
   *                             \p beginning_stamp is guaranteed to be less than \p ending_stamp.
   * @param[in]  ending_stamp    The ending timestamp of the motion model constraints to be generated.
   *                             \p ending_stamp is guaranteed to be greater than \p beginning_stamp.
   * @param[out] constraints     One or more motion model constraints between the requested timestamps.
   * @param[out] variables       One or more variables at both the \p beginning_stamp and \p ending_stamp. The
   *                             variables should include initial values for the optimizer.
   */
  using MotionModelFunction = std::function<void(const ros::Time& beginning_stamp,
                                                 const ros::Time& ending_stamp,
                                                 std::vector<Constraint::SharedPtr>& constraints,
                                                 std::vector<Variable::SharedPtr>& variables)>;

  /**
   * @brief A range of timestamps
   *
   * An object representing a range defined by two iterators. It has begin() and end() methods (which means it can
   * be used in range-based for loops), an empty() method, and a front() method for directly accessing the first
   * member. When dereferenced, an iterator returns a const ros::Time&.
   */
  using stamp_range = boost::any_range<const ros::Time, boost::forward_traversal_tag>;

  /**
   * @brief Constructor that accepts the motion model generator as a std::function object, probably constructed using
   * std::bind()
   *
   * @param[in] generator     A function that generates motion model constraints between the requested timestamps.
   * @param[in] buffer_length The length of the motion model history. If queries arrive involving timestamps
   *                          that are older than the buffer length, an exception will be thrown.
   */
  explicit TimestampManager(MotionModelFunction generator, const ros::Duration& buffer_length = ros::DURATION_MAX);

  /**
   * @brief Constructor that accepts the motion model generator as a member function pointer and object pointer
   *
   * i.e. TimestampManager(&SomeClass::generatorMethod, &some_class_instance)
   *
   * @param[in] fp            A function pointer to a class method that generates motion model constraints between
   *                          the requested timestamps.
   * @param[in] obj           A pointer to the object to be used to generate motion model constraints
   * @param[in] buffer_length The length of the motion model history. If queries arrive involving timestamps
   *                          that are older than the buffer length, an exception will be thrown.
   */
  template<class T>
  TimestampManager(void(T::*fp)(const ros::Time& beginning_stamp,
                                const ros::Time& ending_stamp,
                                std::vector<Constraint::SharedPtr>& constraints,
                                std::vector<Variable::SharedPtr>& variables),
                   T* obj,
                   const ros::Duration& buffer_length = ros::DURATION_MAX);

  /**
   * @brief Constructor that accepts the motion model generator as a const member function pointer and object pointer
   *
   * i.e. TimestampManager(&SomeClass::constGeneratorMethod, &some_class_instance)
   *
   * @param[in] fp            A function pointer to class method that generates motion model constraints between
   *                          the requested timestamps.
   * @param[in] obj           A pointer to the object to be used to generate motion model constraints
   * @param[in] buffer_length The length of the motion model history. If queries arrive involving timestamps
   *                          that are older than the buffer length, an exception will be thrown.
   */
  template<class T>
  TimestampManager(void(T::*fp)(const ros::Time& beginning_stamp,
                                const ros::Time& ending_stamp,
                                std::vector<Constraint::SharedPtr>& constraints,
                                std::vector<Variable::SharedPtr>& variables) const,
                   T* obj,
                   const ros::Duration& buffer_length = ros::DURATION_MAX);

  /**
   * @brief Destructor
   */
  virtual ~TimestampManager() = default;

  /**
   * @brief Read-only access to the buffer length
   */
  const ros::Duration& bufferLength() const
  {
    return buffer_length_;
  }

  /**
   * @brief Write access to the buffer length
   */
  void bufferLength(const ros::Duration& buffer_length)
  {
    buffer_length_ = buffer_length;
  }

  /**
   * @brief Create a transaction structure such that the provided timestamps are connected by motion model constraints.
   *
   * This is not as straightforward as it would seem. Depending on the history of previously generated constraints,
   * fulfilling the request may require removing previously generated constraints and creating several new
   * constraints, such that all timestamps are linked together in a sequential chain. This function calls
   * the \p generator function provided in the constructor to generate the correct set of constraints based on history.
   * This function is designed to be used in the derived MotionModel::queryCallback() implementation -- however this
   * method may throw an exception if it is unable to generate the requested motion models.
   *
   * @param[in]  stamps           The set of timestamps that should be connected by motion model constraints
   * @param[out] transaction      The transaction object that should be augmented with motion model constraints
   * @param[in]  update_variables Update the values of any existing variables with the newly generated values
   * @throws
   */
  void query(const std::set<ros::Time>& stamps, Transaction& transaction, bool update_variables = false);

  /**
   * @brief Read-only access to the current set of timestamps
   *
   * @return An iterator range containing all known timestamps in ascending order
   */
  stamp_range stamps() const;

protected:
  /**
   * @brief Structure used to represent a previously generated motion model constraint
   */
  struct MotionModelSegment
  {
    ros::Time beginning_stamp;
    ros::Time ending_stamp;
    std::vector<Constraint::SharedPtr> constraints;
    std::vector<Variable::SharedPtr> variables;

    MotionModelSegment() = default;

    MotionModelSegment(
      const ros::Time& beginning_stamp,
      const ros::Time& ending_stamp,
      const std::vector<Constraint::SharedPtr>& constraints,
      const std::vector<Variable::SharedPtr>& variables) :
        beginning_stamp(beginning_stamp),
        ending_stamp(ending_stamp),
        constraints(constraints),
        variables(variables)
    {
    }
  };

  /**
   * @brief The set of previously generated motion model segments, sorted by beginning time.
   *
   * The MotionModelHistory will always contain all represented timestamps; the very last entry will be the ending
   * time of the previous MotionModelSegment, and the very last entry will be an empty MotionModelSegment.
   */
  using MotionModelHistory = std::map<ros::Time, MotionModelSegment>;

  MotionModelFunction generator_;  //!< Users upplied function that generates motion model constraints
  ros::Duration buffer_length_;  //!< The length of the motion model history. Segments older than \p buffer_length_
                                 //!< will be removed from the motion model history
  MotionModelHistory motion_model_history_;  //!< Container that stores all previously generated motion models

  /**
   * @brief Helper function used with boost::transform_iterators to convert the internal MotionModelHistory value type
   * into a const ros::Time& iterator compatible with stamp_range
   */
  static const ros::Time& extractStamp(const typename MotionModelHistory::value_type& element);

  /**
   * @brief Create a new MotionModelSegment, updating the provided transaction.
   *
   * The motion_model_history_ container will be updated.
   *
   * @param[in]  beginning_stamp The beginning timestamp of the new segment
   * @param[in]  ending_stamp    The ending timestamp of the new segment
   * @param[out] transaction     A transaction object to be updated with the changes caused by addSegment
   */
  void addSegment(
    const ros::Time& beginning_stamp,
    const ros::Time& ending_stamp,
    Transaction& transaction);

  /**
   * @brief Remove an existing MotionModelSegment, updating the provided transaction.
   *
   * The motion_model_history_ container will be updated.
   *
   * @param[in]  iter        An iterator to the MotionModelSegment to remove. The iterator will be invalid afterwards.
   * @param[out] transaction A transaction object to be updated with the changes caused by removeSegment
   */
  void removeSegment(
    MotionModelHistory::iterator& iter,
    Transaction& transaction);

  /**
   * @brief Split an existing MotionModelSegment into two pieces at the provided timestamp, updating the provided
   * transaction.
   *
   * The motion_model_history_ container will be updated.
   *
   * @param[in]  iter        An iterator to the MotionModelSegment to split. The iterator will be invalid afterwards.
   * @param[in]  stamp       The timestamp where the MotionModelSegment should be split
   * @param[out] transaction A transaction object to be updated with the changes caused by splitSegment
   */
  void splitSegment(
      MotionModelHistory::iterator& iter,
      const ros::Time& stamp,
      Transaction& transaction);

  /**
   * @brief Remove any motion model segments that are older than \p buffer_length_
   */
  void purgeHistory();
};

template<class T>
TimestampManager::TimestampManager(void(T::*fp)(const ros::Time& beginning_stamp,
                                                const ros::Time& ending_stamp,
                                                std::vector<Constraint::SharedPtr>& constraints,
                                                std::vector<Variable::SharedPtr>& variables),
                                   T* obj,
                                   const ros::Duration& buffer_length) :
  TimestampManager(std::bind(fp,
                             obj,
                             std::placeholders::_1,
                             std::placeholders::_2,
                             std::placeholders::_3,
                             std::placeholders::_4),
                   buffer_length)
{
}

template<class T>
TimestampManager::TimestampManager(void(T::*fp)(const ros::Time& beginning_stamp,
                                                const ros::Time& ending_stamp,
                                                std::vector<Constraint::SharedPtr>& constraints,
                                                std::vector<Variable::SharedPtr>& variables) const,
                                   T* obj,
                                   const ros::Duration& buffer_length) :
  TimestampManager(std::bind(fp,
                             obj,
                             std::placeholders::_1,
                             std::placeholders::_2,
                             std::placeholders::_3,
                             std::placeholders::_4),
                   buffer_length)
{
}

}  // namespace fuse_core

#endif  // FUSE_CORE_TIMESTAMP_MANAGER_H
