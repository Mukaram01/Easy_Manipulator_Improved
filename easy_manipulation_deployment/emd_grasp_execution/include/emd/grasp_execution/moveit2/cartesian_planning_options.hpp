// Copyright 2026 OpenAI
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

#ifndef EMD__GRASP_EXECUTION__MOVEIT2__CARTESIAN_PLANNING_OPTIONS_HPP_
#define EMD__GRASP_EXECUTION__MOVEIT2__CARTESIAN_PLANNING_OPTIONS_HPP_

#include <cmath>
#include <cstddef>
#include <limits>
#include <string>

#include "moveit/core/utils.h"
#include "moveit_msgs/msg/constraints.hpp"

namespace grasp_execution
{
namespace moveit2
{

struct CartesianPlanningOptions
{
  bool avoid_collisions{true};
  moveit_msgs::msg::Constraints path_constraints;
  double planning_timeout{0.0};
  int max_ik_attempts{0};
};

enum class CartesianPlanStatus
{
  kOk,
  kInvalidInput,
  kNoSolution,
  kCollisionFilteredOut,
};

struct CartesianInputValidation
{
  CartesianPlanStatus status{CartesianPlanStatus::kOk};
  std::string message;
};

struct CartesianConstraintConfig
{
  bool has_path_constraints{false};
  bool build_constraint_fn{false};
};

inline CartesianInputValidation validate_cartesian_request(
  std::size_t waypoint_count,
  bool link_exists,
  double step,
  double jump_threshold)
{
  if (waypoint_count == 0U) {
    return {CartesianPlanStatus::kInvalidInput, "Waypoint list must be non-empty"};
  }

  if (!link_exists) {
    return {CartesianPlanStatus::kInvalidInput, "Requested link does not exist in the robot model"};
  }

  if (!(step > 0.0)) {
    return {CartesianPlanStatus::kInvalidInput, "Cartesian interpolation step must be > 0"};
  }

  if (!std::isfinite(jump_threshold)) {
    return {CartesianPlanStatus::kInvalidInput, "Jump threshold must be finite"};
  }

  return {CartesianPlanStatus::kOk, ""};
}

inline CartesianConstraintConfig build_cartesian_constraint_config(
  const CartesianPlanningOptions & options)
{
  const bool has_path_constraints = !moveit::core::isEmpty(options.path_constraints);
  return {
    has_path_constraints,
    options.avoid_collisions || has_path_constraints,
  };
}

}  // namespace moveit2
}  // namespace grasp_execution

#endif  // EMD__GRASP_EXECUTION__MOVEIT2__CARTESIAN_PLANNING_OPTIONS_HPP_
