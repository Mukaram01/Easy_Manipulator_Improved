// Copyright 2021 ROS Industrial Consortium Asia Pacific
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

#ifndef EMD__DYNAMIC_SAFETY__REPLANNER_FLATTEN_UTILS_HPP_
#define EMD__DYNAMIC_SAFETY__REPLANNER_FLATTEN_UTILS_HPP_

#include <algorithm>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace dynamic_safety
{

inline bool have_same_joint_names(
  const std::vector<std::string> & expected,
  const std::vector<std::string> & actual)
{
  if (expected.size() != actual.size()) {
    return false;
  }
  std::unordered_map<std::string, size_t> histogram;
  for (const auto & name : expected) {
    histogram[name]++;
  }
  for (const auto & name : actual) {
    auto it = histogram.find(name);
    if (it == histogram.end() || it->second == 0) {
      return false;
    }
    it->second--;
  }
  return std::all_of(
    histogram.begin(), histogram.end(), [](const auto & item) {return item.second == 0;});
}

inline bool reorder_joint(
  const std::vector<std::string> & reference_joint_order,
  std::vector<std::string> & current_joint_order,
  trajectory_msgs::msg::JointTrajectoryPoint & state)
{
  if (!have_same_joint_names(reference_joint_order, current_joint_order)) {
    return false;
  }

  std::unordered_map<std::string, size_t> ref_joint_idx_map;
  std::vector<size_t> joint_permutation;
  ref_joint_idx_map.reserve(reference_joint_order.size());
  joint_permutation.reserve(current_joint_order.size());
  for (size_t i = 0; i < reference_joint_order.size(); i++) {
    ref_joint_idx_map[reference_joint_order[i]] = i;
  }
  for (const auto & joint_name : current_joint_order) {
    joint_permutation.push_back(ref_joint_idx_map[joint_name]);
  }

  for (size_t i = 0; i < current_joint_order.size(); i++) {
    while (joint_permutation[i] != i) {
      std::swap(current_joint_order[joint_permutation[i]], current_joint_order[i]);
      if (!state.positions.empty()) {
        std::swap(state.positions[joint_permutation[i]], state.positions[i]);
      }
      if (!state.velocities.empty()) {
        std::swap(state.velocities[joint_permutation[i]], state.velocities[i]);
      }
      if (!state.accelerations.empty()) {
        std::swap(state.accelerations[joint_permutation[i]], state.accelerations[i]);
      }
      std::swap(joint_permutation[joint_permutation[i]], joint_permutation[i]);
    }
  }

  return true;
}

inline void set_start_kinematics(
  trajectory_msgs::msg::JointTrajectoryPoint & start,
  const trajectory_msgs::msg::JointTrajectoryPoint & current)
{
  if (!current.velocities.empty() && current.velocities.size() == start.positions.size()) {
    start.velocities = current.velocities;
  }
  if (!current.accelerations.empty() && current.accelerations.size() == start.positions.size()) {
    start.accelerations = current.accelerations;
  }
}

inline bool has_monotonic_timestamps(
  const std::vector<trajectory_msgs::msg::JointTrajectoryPoint> & points)
{
  if (points.empty()) {
    return false;
  }
  double prev = -1.0;
  for (const auto & p : points) {
    const double t = rclcpp::Duration(p.time_from_start).seconds();
    if (t < prev) {
      return false;
    }
    prev = t;
  }
  return true;
}

}  // namespace dynamic_safety

#endif  // EMD__DYNAMIC_SAFETY__REPLANNER_FLATTEN_UTILS_HPP_
