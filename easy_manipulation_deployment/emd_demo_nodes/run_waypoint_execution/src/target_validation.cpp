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
      "Planning target validation failed for target_id '%s': missing target payload (target is null).",
      target_id.c_str());
    return false;
  }

  if (target->grasp_methods.empty()) {
    RCLCPP_ERROR(
      logger,
      "Planning target validation failed for target_id '%s': missing grasp_methods[0] (grasp_methods is empty).",
      target_id.c_str());
    return false;
  }

  grasp_method = &target->grasp_methods[0];

  if (grasp_method->grasp_poses.empty()) {
    RCLCPP_ERROR(
      logger,
      "Planning target validation failed for target_id '%s': missing grasp_methods[0].grasp_poses[0] (grasp_poses is empty).",
      target_id.c_str());
    return false;
  }

  grasp_pose = &grasp_method->grasp_poses[0];
  return true;
}

}  // namespace run_waypoint_execution
