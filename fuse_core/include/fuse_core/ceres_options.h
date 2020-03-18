/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019 Clearpath Robotics
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
#ifndef FUSE_CORE_CERES_OPTIONS_H
#define FUSE_CORE_CERES_OPTIONS_H

#include <fuse_core/ceres_macros.h>

#include <ros/console.h>
#include <ros/node_handle.h>

#include <ceres/version.h>
#include <ceres/covariance.h>
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <ceres/types.h>

#include <string>

/**
 * Defines ToString overloaded function to Ceres Options.
 *
 * For a given Ceres Solver Option <T>, the function ToString calls ceres::<T>ToString
 */
#define CERES_OPTION_TO_STRING_DEFINITION(Option) \
  static inline const char* ToString(ceres::Option value) \
  { \
    return ceres::Option##ToString(value); \
  }

/**
 * Defines FromString overloaded function to Ceres Options.
 *
 * For a given Ceres Solver Option <T>, the function FromString calls ceres::StringTo<T>
 */
#define CERES_OPTION_FROM_STRING_DEFINITION(Option) \
  static inline bool FromString(std::string string_value, ceres::Option* value) \
  { \
    return ceres::StringTo##Option(string_value, value); \
  }

/**
 * Defines ToString and FromString overloaded functions for Ceres Options.
 *
 * See CERES_OPTION_TO_STRING_DEFINITION and CERES_OPTION_FROM_STRING_DEFINITION.
 */
#define CERES_OPTION_STRING_DEFINITIONS(Option) \
  CERES_OPTION_TO_STRING_DEFINITION(Option) \
  CERES_OPTION_FROM_STRING_DEFINITION(Option)

#if !CERES_VERSION_AT_LEAST(2, 0, 0)
/**
 * Patch Ceres versions before 2.0.0 that miss the LoggingType and DumpFormatType ToString and StringTo functions.
 */
#include <algorithm>

namespace ceres
{

#define CASESTR(x) case x: return #x
#define STRENUM(x) if (value == #x) { *type = x; return true;}

static void UpperCase(std::string* input)
{
  // The NOLINT below it's because std::transform requires <algorithm>, which is included inside the #if above, but
  // roslint still complains
  std::transform(input->begin(), input->end(), input->begin(), ::toupper);  // NOLINT(build/include_what_you_use)
}

inline const char* LoggingTypeToString(LoggingType type)
{
  switch (type)
  {
    CASESTR(SILENT);
    CASESTR(PER_MINIMIZER_ITERATION);
    default:
      return "UNKNOWN";
  }
}

inline bool StringToLoggingType(std::string value, LoggingType* type)
{
  UpperCase(&value);
  STRENUM(SILENT);
  STRENUM(PER_MINIMIZER_ITERATION);
  return false;
}

inline const char* DumpFormatTypeToString(DumpFormatType type)
{
  switch (type)
  {
    CASESTR(CONSOLE);
    CASESTR(TEXTFILE);
    default:
      return "UNKNOWN";
  }
}

inline bool StringToDumpFormatType(std::string value, DumpFormatType* type)
{
  UpperCase(&value);
  STRENUM(CONSOLE);
  STRENUM(TEXTFILE);
  return false;
}

#undef CASESTR
#undef STRENUM

}  // namespace ceres
#else
/**
 * Patch Ceres version 2.0.0 that uses lower case for the LoggingType and DumpFormatType StringTo function.
 * See https://github.com/ceres-solver/ceres-solver/blob/master/include/ceres/types.h
 */
namespace ceres
{

inline bool StringToLoggingType(std::string value, LoggingType* type)
{
  return StringtoLoggingType(value, type);
}

inline bool StringToDumpFormatType(std::string value, DumpFormatType* type)
{
  return StringtoDumpFormatType(value, type);
}

}  // namespace ceres
#endif

namespace fuse_core
{

/**
 * String definitions for all Ceres options.
 */
CERES_OPTION_STRING_DEFINITIONS(CovarianceAlgorithmType)
CERES_OPTION_STRING_DEFINITIONS(DenseLinearAlgebraLibraryType)
CERES_OPTION_STRING_DEFINITIONS(DoglegType)
CERES_OPTION_STRING_DEFINITIONS(DumpFormatType)
CERES_OPTION_STRING_DEFINITIONS(LinearSolverType)
CERES_OPTION_STRING_DEFINITIONS(LineSearchDirectionType)
CERES_OPTION_STRING_DEFINITIONS(LineSearchInterpolationType)
CERES_OPTION_STRING_DEFINITIONS(LineSearchType)
CERES_OPTION_STRING_DEFINITIONS(LoggingType)
CERES_OPTION_STRING_DEFINITIONS(MinimizerType)
CERES_OPTION_STRING_DEFINITIONS(NonlinearConjugateGradientType)
CERES_OPTION_STRING_DEFINITIONS(PreconditionerType)
CERES_OPTION_STRING_DEFINITIONS(SparseLinearAlgebraLibraryType)
CERES_OPTION_STRING_DEFINITIONS(TrustRegionStrategyType)
CERES_OPTION_STRING_DEFINITIONS(VisibilityClusteringType)

/**
 * @brief Helper function that loads a Ceres Option (e.g. ceres::LinearSolverType) value from the parameter server
 *
 * @param[in] node_handle - The node handle used to load the parameter
 * @param[in] parameter_name - The parameter name to load
 * @param[in] default_value - A default value to use if the provided parameter name does not exist
 * @return The loaded (or default) value
 */
template <class T>
T getParam(const ros::NodeHandle& node_handle, const std::string& parameter_name, const T& default_value)
{
  const std::string default_string_value{ ToString(default_value) };

  std::string string_value;
  node_handle.param(parameter_name, string_value, default_string_value);

  T value;
  if (!FromString(string_value, &value))
  {
    ROS_WARN_STREAM("The requested " << parameter_name << " (" << string_value
                                     << ") is not supported. Using the default value (" << default_string_value
                                     << ") instead.");
    value = default_value;
  }

  return value;
}

/**
 * @brief Populate a ceres::Covariance::Options object with information from the parameter server
 *
 * @param[in] nh - A node handle in a namespace containing ceres::Covariance::Options settings
 * @param[out] covariance_options - The ceres::Covariance::Options object to update
 */
void loadCovarianceOptionsFromROS(const ros::NodeHandle& nh, ceres::Covariance::Options& covariance_options);

/**
 * @brief Populate a ceres::Problem::Options object with information from the parameter server
 *
 * @param[in] nh - A node handle in a namespace containing ceres::Problem::Options settings
 * @param[out] problem_options - The ceres::Problem::Options object to update
 */
void loadProblemOptionsFromROS(const ros::NodeHandle& nh, ceres::Problem::Options& problem_options);

/**
 * @brief Populate a ceres::Solver::Options object with information from the parameter server
 *
 * @param[in] nh - A node handle in a namespace containing ceres::Solver::Options settings
 * @param[out] solver_options - The ceres::Solver::Options object to update
 */
void loadSolverOptionsFromROS(const ros::NodeHandle& nh, ceres::Solver::Options& solver_options);

}  // namespace fuse_core

#endif  // FUSE_CORE_CERES_OPTIONS_H
