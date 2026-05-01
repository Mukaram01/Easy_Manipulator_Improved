// Copyright 2026 Mukaram
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RUN_GRASP_EXECUTION__HOME_RETURN_UTILS_HPP_
#define RUN_GRASP_EXECUTION__HOME_RETURN_UTILS_HPP_

#include <string>
#include <vector>
#include <optional>
#include <exception>

#include "rclcpp/parameter.hpp"

namespace run_grasp_execution
{

struct HomeReturnStepResult
{
  bool success{false};
  bool executed{false};
  std::string failure_reason;
};

inline std::string default_home_return_failure_reason(
  const std::string & step_name,
  int attempt,
  int max_attempts)
{
  return step_name + " planning failed on attempt " + std::to_string(attempt) +
         "/" + std::to_string(max_attempts) +
         " (no valid plan returned by MoveIt).";
}

inline bool safe_intermediate_enabled(
  bool use_safe_intermediate,
  const std::vector<double> & safe_joint_state)
{
  return use_safe_intermediate && !safe_joint_state.empty();
}

struct SafeJointStateResolution
{
  std::vector<double> value;
  std::optional<std::string> warning_message;
};

inline std::string parameter_type_to_string(const rclcpp::ParameterType type)
{
  switch (type) {
    case rclcpp::ParameterType::PARAMETER_NOT_SET:
      return "not set";
    case rclcpp::ParameterType::PARAMETER_BOOL:
      return "bool";
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      return "integer";
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      return "double";
    case rclcpp::ParameterType::PARAMETER_STRING:
      return "string";
    case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY:
      return "byte_array";
    case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
      return "bool_array";
    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
      return "integer_array";
    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
      return "double_array";
    case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
      return "string_array";
    default:
      return "unknown";
  }
}

inline SafeJointStateResolution parse_safe_joint_state_parameter(
  const rclcpp::Parameter & parameter,
  const std::string & param_name)
{
  const std::vector<double> empty_default;
  if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
    return {empty_default, std::nullopt};
  }

  if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
    return {
      empty_default,
      "Parameter '" + param_name + "' has type '" + parameter_type_to_string(parameter.get_type()) +
      "' but expected 'double_array'. Using empty safe joint state."
    };
  }

  try {
    return {parameter.as_double_array(), std::nullopt};
  } catch (const std::exception & e) {
    return {
      empty_default,
      "Failed to parse parameter '" + param_name + "' as double array (" +
      std::string(e.what()) + "). Using empty safe joint state."
    };
  }
}

inline std::optional<std::string> safe_intermediate_skip_warning(
  bool use_safe_intermediate,
  const std::vector<double> & safe_joint_state,
  size_t manipulator_dof,
  const std::string & planning_group)
{
  if (!use_safe_intermediate) {
    return std::nullopt;
  }
  if (safe_joint_state.empty()) {
    return "[HomeReturn] home_return.use_safe_intermediate=true but home_return.safe_joint_state is empty. "
           "Skipping safe intermediate and continuing to home.";
  }
  if (safe_joint_state.size() != manipulator_dof) {
    return "[HomeReturn] home_return.safe_joint_state has " +
           std::to_string(safe_joint_state.size()) + " values, expected " +
           std::to_string(manipulator_dof) + " for group '" + planning_group +
           "'. Skipping safe intermediate and continuing to home.";
  }
  return std::nullopt;
}

}  // namespace run_grasp_execution

#endif  // RUN_GRASP_EXECUTION__HOME_RETURN_UTILS_HPP_
