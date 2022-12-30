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
#ifndef FUSE_PUBLISHERS_STAMPED_VARIABLE_SYNCHRONIZER_H
#define FUSE_PUBLISHERS_STAMPED_VARIABLE_SYNCHRONIZER_H

#include <fuse_core/graph.hpp>
#include <fuse_core/fuse_macros.hpp>
#include <fuse_core/time.hpp>
#include <fuse_core/transaction.hpp>
#include <fuse_core/uuid.hpp>
#include <fuse_core/variable.hpp>
#include <fuse_variables/stamped.hpp>
#include <fuse_core/time.hpp>

#include <type_traits>


namespace fuse_publishers
{

/**
 * @brief A utility class that finds the most recent timestamp shared by a set of stamped variables
 *
 * This is designed to be used by derived fuse_core::Publisher classes. The class remembers the last common timestamp
 * from the previous call, and attempts to find a more recent common timestamp using the provided transaction
 * information. If no common timestamp is found after searching the transaction, a full search of the graph will
 * be conducted. If no common timestamp is found after searching the full graph, a zero timestamp will be returned.
 *
 * The set of variable types are provided in the template parameters. e.g.
 * @code{.cpp}
 * auto synchronizer = StampedVariableSynchronizer<Orientation2DStamped, Position2DStamped>(device_id);
 * @endcode
 *
 * This class only functions with variables derived from the fuse_variables::Stamped base class.
 */
template <typename ...Ts>
class StampedVariableSynchronizer
{
public:
  FUSE_SMART_PTR_DEFINITIONS(StampedVariableSynchronizer)

  /**
   * @brief Construct a synchronizer object
   *
   * @param[in] device_id The device id to use for all variable types
   */
  explicit StampedVariableSynchronizer(const fuse_core::UUID& device_id = fuse_core::uuid::NIL);

  /**
   * @brief Find the latest timestamp for which variables of all the specified template types exist in the graph
   *
   * @param[in] transaction A fuse_core::Transaction object containing the changes to the graph since the last call
   * @param[in] graph       The complete graph
   * @return The latest timestamp shared by all requested variable types
   */
  rclcpp::Time findLatestCommonStamp(const fuse_core::Transaction& transaction, const fuse_core::Graph& graph);

private:
  fuse_core::UUID device_id_;  //!< The device_id to use with the Stamped classes
  rclcpp::Time latest_common_stamp_;  //!< The previously discovered common stamp

  /**
   * @brief Search the variables in the provided range for more recent timestamps. Update the \p latest_common_stamp_
   *        member variable if a newer common timestamp is found.
   *
   * @param[in] variable_range The collection of variables to test
   * @param[in] graph          The complete graph, used to verify that all requested variables exist for a given time
   */
  template <typename VariableRange>
  void updateTime(const VariableRange& variable_range, const fuse_core::Graph& graph);
};

namespace detail
{

/**
 * @brief Some helper structs for testing properties of all types in a template parameter pack
 */
template<bool...> struct bool_pack;
template<bool ...bs>
using all_true_helper = std::is_same<bool_pack<bs..., true>, bool_pack<true, bs...>>;

/**
 * @brief Test if a property is true for all types in a template parameter pack. This is a type.
 */
template <typename ...Ts>
using all_true = all_true_helper<Ts::value...>;

/**
 * @brief Test if a property is true for all types in a template parameter pack. This is a boolean value.
 */
template<typename ...Ts>
constexpr bool allTrue = all_true<Ts...>::value;

/**
 * @brief Test if a class is derived from the fuse_variables::Stamped base class. This is a type.
 */
template<typename T>
using is_stamped = std::is_base_of<fuse_variables::Stamped, T>;

/**
 * @brief Test if a class is derived from the fuse_variables::Stamped base class. This is a boolean value.
 */
template<typename T>
constexpr bool isStamped = is_stamped<T>::value;

/**
 * @brief Test if a class is derived from the fuse_core::Variable base class. This is a type.
 */
template<typename T>
using is_variable = std::is_base_of<fuse_core::Variable, T>;

/**
 * @brief Test if a class is derived from the fuse_core::Variable base class. This is a boolean value.
 */
template<typename T>
constexpr bool isVariable = is_variable<T>::value;

/**
 * @brief Test if a class is derived from both the fuse_core::Variable base class and the fuse_variables::Stamped
 *        base class. This is a type.
 */
template<typename T>
struct is_stamped_variable
{
  static constexpr bool value = isStamped<T> && isVariable<T>;
};

/**
 * @brief Test if a class is derived from both the fuse_core::Variable base class and the fuse_variables::Stamped
 *        base class. This is a boolean value.
 */
template<typename T>
constexpr bool isStampedVariable = is_stamped_variable<T>::value;

/**
 * @brief Test if all of the template parameter pack types are fuse_core::Variable and fuse_variables::Stamped.
 *        This is a type.
 */
template <typename ...Ts>
using all_stamped_variables = all_true<is_stamped_variable<Ts>...>;

/**
 * @brief Test if all of the template parameter pack types are fuse_core::Variable and fuse_variables::Stamped.
 *        This is a boolean value.
 */
template<typename ...Ts>
constexpr bool allStampedVariables = all_stamped_variables<Ts...>::value;

/**
 * @brief Test if instances of all the template parameter pack types exist in the graph
 *
 * This version accepts an empty parameter pack, and is used to terminate the recursive template parameter pack
 * expansion.
 *
 * This would be much easier to write in C++17 using 'if constexpr (sizeof...(Ts) > 0)'
 *
 * @param[in] graph     The complete graph, used to verify the existence of a variable
 * @param[in] stamp     The timestamp used to construct all variable types
 * @param[in] device_id The device id used to construct all variable types
 * @return True if all variables exist, false otherwise
 */
template <typename...>
struct all_variables_exist
{
  static bool value(const fuse_core::Graph& /*graph*/, const rclcpp::Time& /*stamp*/, const fuse_core::UUID& /*device_id*/)
  {
    return true;
  }
};

/**
 * @brief Test if instances of all the template parameter pack types exist in the graph
 *
 * This version accepts two or more template arguments. The template parameter pack is expanded recursively.
 *
 * @param[in] graph     The complete graph, used to verify the existence of a variable
 * @param[in] stamp     The timestamp used to construct all variable types
 * @param[in] device_id The device id used to construct all variable types
 * @return True if all variables exist, false otherwise
 */
template <typename T, typename ...Ts>
struct all_variables_exist<T, Ts...>
{
  static bool value(const fuse_core::Graph& graph, const rclcpp::Time& stamp, const fuse_core::UUID& device_id)
  {
    return graph.variableExists(T(stamp, device_id).uuid()) &&
           all_variables_exist<Ts...>::value(graph, stamp, device_id);
  }
};

/**
 * @brief Test if a given variable is included in the template parameter pack types
 *
 * This version accepts an empty parameter pack, and is used to terminate the recursive template parameter pack
 * expansion.
 *
 * This would be much easier to write in C++17 using 'if constexpr (sizeof...(Ts) > 0)'
 *
 * @param[in] variable The variable to check against the template parameter pack
 * @return True if the variable's type is part of the template parameter pack, false otherwise
 */
template <typename...>
struct is_variable_in_pack
{
  static bool value(const fuse_core::Variable& /*variable*/)
  {
    return false;
  }
};

/**
 * @brief Test if a given variable is included in the template parameter pack types
 *
 * This version accepts two or more template arguments. The template parameter pack is expanded recursively.
 *
 * @param[in] variable The variable to check against the template parameter pack
 * @return True if the variable's type is part of the template parameter pack, false otherwise
 */
template <typename T, typename ...Ts>
struct is_variable_in_pack<T, Ts...>
{
  static bool value(const fuse_core::Variable& variable)
  {
    auto derived = dynamic_cast<const T*>(&variable);
    return static_cast<bool>(derived) ||
           is_variable_in_pack<Ts...>::value(variable);
  }
};

}  // namespace detail

template <typename ...Ts>
StampedVariableSynchronizer<Ts...>::StampedVariableSynchronizer(const fuse_core::UUID& device_id) :
  device_id_(device_id),
  // NOTE(CH3): Uninitialized, for getting latest
  //            We use RCL_ROS_TIME so time comparisons are consistent
  latest_common_stamp_(rclcpp::Time(0, 0, RCL_ROS_TIME))
{
  static_assert(detail::allStampedVariables<Ts...>, "All synchronized types must be derived from both "
                                                    "fuse_core::Variable and fuse_variable::Stamped.");
  static_assert(sizeof...(Ts) > 0, "At least one type must be specified.");
}

template <typename ...Ts>
rclcpp::Time StampedVariableSynchronizer<Ts...>::findLatestCommonStamp(
  const fuse_core::Transaction& transaction,
  const fuse_core::Graph& graph)
{
  // Clear the previous stamp if the variable was deleted
  if (fuse_core::is_valid(latest_common_stamp_) &&
      !detail::all_variables_exist<Ts...>::value(graph, latest_common_stamp_, device_id_))
  {
    latest_common_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }
  // Search the transaction for more recent variables
  updateTime(transaction.addedVariables(), graph);
  // If no common timestamp was found, search the whole graph for the most recent variable set
  if (!fuse_core::is_valid(latest_common_stamp_))
  {
    updateTime(graph.getVariables(), graph);
  }
  return latest_common_stamp_;
}

template <typename ...Ts>
template <typename VariableRange>
void StampedVariableSynchronizer<Ts...>::updateTime(
  const VariableRange& variable_range,
  const fuse_core::Graph& graph)
{
  for (const auto& candidate_variable : variable_range)
  {
    if (detail::is_variable_in_pack<Ts...>::value(candidate_variable))
    {
      const auto& stamped_variable = dynamic_cast<const fuse_variables::Stamped&>(candidate_variable);
      if ((stamped_variable.stamp() > latest_common_stamp_) &&
          (stamped_variable.deviceId() == device_id_) &&
          (detail::all_variables_exist<Ts...>::value(graph, stamped_variable.stamp(), device_id_)))
      {
        latest_common_stamp_ = stamped_variable.stamp();
      }
    }
  }
}

}  // namespace fuse_publishers

#endif  // FUSE_PUBLISHERS_STAMPED_VARIABLE_SYNCHRONIZER_H
