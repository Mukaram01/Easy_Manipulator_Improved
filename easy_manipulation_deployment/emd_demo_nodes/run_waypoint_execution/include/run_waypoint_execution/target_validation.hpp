#pragma once

#include <string>

#include "rclcpp/logger.hpp"

#include "emd/grasp_execution/context.hpp"
#include "emd_msgs/msg/grasp_method.hpp"
#include "emd_msgs/msg/grasp_target.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace run_waypoint_execution
{

bool validate_grasp_target_selection(
  const rclcpp::Logger & logger,
  const emd_msgs::msg::GraspTarget::SharedPtr & target,
  const std::string & target_id,
  const emd_msgs::msg::GraspMethod *& grasp_method,
  const geometry_msgs::msg::PoseStamped *& grasp_pose);

bool resolve_end_effector_mapping(
  const grasp_execution::WorkcellContext & workcell_context,
  const std::string & ee_brand,
  std::string & planning_group,
  std::string & ee_link,
  double & clearance);

}  // namespace run_waypoint_execution
