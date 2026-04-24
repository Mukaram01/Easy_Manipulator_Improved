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

#include "run_waypoint_execution/target_validation.hpp"

#include "rclcpp/rclcpp.hpp"

namespace run_waypoint_execution
{

bool validate_grasp_target_selection(
  const rclcpp::Logger & logger,
  const emd_msgs::msg::GraspTarget::SharedPtr & target,
  const std::string & target_id,
  const emd_msgs::msg::GraspMethod *& grasp_method,
  const geometry_msgs::msg::PoseStamped *& grasp_pose)
{
  grasp_method = nullptr;
  grasp_pose = nullptr;

  if (!target) {
    RCLCPP_ERROR(
      logger,
      "Planning target validation failed for target_id '%s': missing "
      "target payload (target is null).",
      target_id.c_str());
    return false;
  }

  if (target->grasp_methods.empty()) {
    RCLCPP_ERROR(
      logger,
      "Planning target validation failed for target_id '%s': missing "
      "grasp_methods[0] (grasp_methods is empty).",
      target_id.c_str());
    return false;
  }

  grasp_method = &target->grasp_methods[0];

  if (grasp_method->grasp_poses.empty()) {
    RCLCPP_ERROR(
      logger,
      "Planning target validation failed for target_id '%s': missing "
      "grasp_methods[0].grasp_poses[0] (grasp_poses is empty).",
      target_id.c_str());
    return false;
  }

  grasp_pose = &grasp_method->grasp_poses[0];
  return true;
}

bool resolve_end_effector_mapping(
  const grasp_execution::WorkcellContext & workcell_context,
  const std::string & ee_brand,
  std::string & planning_group,
  std::string & ee_link,
  double & clearance)
{
  planning_group.clear();
  ee_link.clear();
  clearance = 0.0;

  bool found_end_effector = false;
  for (const auto & group : workcell_context.groups) {
    for (const auto & ee : group.second.end_effectors) {
      if (ee.second.brand == ee_brand) {
        planning_group = group.first;
        ee_link = ee.second.link;
        clearance = ee.second.clearance;
        found_end_effector = true;
        break;
      }
    }
    if (found_end_effector) {
      break;
    }
  }

  return found_end_effector && !planning_group.empty() && !ee_link.empty();
}

}  // namespace run_waypoint_execution
