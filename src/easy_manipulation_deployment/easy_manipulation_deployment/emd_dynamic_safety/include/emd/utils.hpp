// Copyright 2020 ROS Industrial Consortium Asia Pacific
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

#ifndef EMD__UTILS_HPP_
#define EMD__UTILS_HPP_

#include <iostream>
#include <string>
#include <vector>

#include <rclcpp/node.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace emd
{

template<typename T, typename NodeT>
/// Get parameter (declare if doesn't allow overload)
/**
 * Referenced from moveit servo.
 */
inline void declare_or_get_param(
  T & output_value,
  const std::string & param_name,
  const std::shared_ptr<NodeT> & node,
  const rclcpp::Logger & logger,
  const T & default_value = T{})
{
  try {
    if (node->has_parameter(param_name)) {
      node->template get_parameter_or<T>(param_name, output_value, default_value);
    } else {
      output_value = node->template declare_parameter<T>(param_name, default_value);
    }
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    RCLCPP_ERROR(
      logger,
      "Error getting parameter '%s', check parameter type in YAML file.",
      param_name.c_str());
    throw;
  }
  RCLCPP_DEBUG_STREAM(logger, "param " << param_name << " := " << output_value);
}

}  // namespace emd

#endif  // EMD__UTILS_HPP_
