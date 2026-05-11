#ifndef WORKCELL_BUILDER__ROBOT_TOOL_COMPATIBILITY_HPP_
#define WORKCELL_BUILDER__ROBOT_TOOL_COMPATIBILITY_HPP_

#include <map>
#include <string>
#include <vector>

#include "attributes/workcell.h"

struct CompatibilityIssue
{
  std::string message;
  bool blocker = false;
};

struct RobotProfile
{
  std::string robot_id;
  std::string label;
  std::string description_package;
  std::string moveit_config_package;
  std::string base_link;
  std::string planning_group;
  std::string default_tool_mount_link;
  std::string controller_family;
  std::vector<std::string> supported_tool_types;
  std::vector<std::string> warnings;
};

struct ToolProfile
{
  std::string tool_id;
  std::string label;
  std::string description_package;
  std::string moveit_config_package;
  std::string tool_type;
  std::string mount_link;
  std::string tcp_frame;
  std::string tcp_xyz_rpy;
  std::string controller_hint;
  std::string grasp_strategy_default;
  std::string release_strategy_default;
  bool requires_io = false;
  std::vector<std::string> warnings;
};

struct RobotToolCompatibilityResult
{
  std::string robot_id;
  std::string tool_id;
  std::string status;
  std::string tool_mount_link;
  std::string tcp_frame;
  std::string planning_group;
  std::string controller_hint;
  std::string tool_type;
  std::string grasp_strategy_default;
  std::string release_strategy_default;
  std::vector<CompatibilityIssue> issues;
};

std::map<std::string, RobotProfile> load_robot_profiles(const std::string & config_root);
std::map<std::string, ToolProfile> load_tool_profiles(const std::string & config_root);
RobotToolCompatibilityResult evaluate_robot_tool_compatibility(
  const Scene & scene,
  const std::string & config_root);
void infer_tool_defaults_from_profile(RobotToolCompatibilityResult * result);
std::string compatibility_status_label(const RobotToolCompatibilityResult & result);

#endif
