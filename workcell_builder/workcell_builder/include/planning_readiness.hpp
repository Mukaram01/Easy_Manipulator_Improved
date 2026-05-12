#pragma once

#include <string>
#include <vector>

#include "conveyor_pick_preview.hpp"
#include "workcell_zone_model.hpp"

namespace workcell_builder {

struct PlanningReadinessItem {
  std::string level;
  std::string message;
};

struct PlanningReadinessReport {
  int schema_version{1};
  std::string runtime_mode{"dry_run_readiness_only"};
  std::string robot;
  std::string end_effector;
  std::string task_intent{"task_intent_preview"};
  std::string pick_zone;
  std::string place_zone;
  std::string object_class{"box"};
  std::string planning_group{"manipulator"};
  std::string end_effector_link{"tool0"};
  std::string readiness_status{"WARN"};
  bool can_attempt_plan{false};
  bool can_execute{false};
  std::vector<std::string> blockers;
  std::vector<std::string> warnings;
  bool robot_motion_commanded{false};
  bool moveit_execute_called{false};
  bool gripper_command_sent{false};
  bool real_hardware_commanded{false};
};

struct DryRunPlanningRequest {
  int schema_version{1};
  std::string robot;
  std::string planning_group{"manipulator"};
  std::string start_state{"current_or_named_home"};
  std::string pick_pose_source{"pick_zone_center"};
  std::string place_pose_source{"place_zone_center"};
  std::string frame{"world"};
  std::vector<double> pick_xyz{0.35, 0.0, 0.78};
  std::vector<double> place_xyz{0.25, -0.45, 0.78};
  std::vector<double> rpy{3.14, 0.0, 0.0};
  bool execution_allowed{false};
};

PlanningReadinessReport build_planning_readiness_report(
  const std::string & robot,
  const std::string & end_effector,
  const std::vector<WorkZone> & zones,
  const std::vector<ConveyorFlow> & flows,
  const std::string & moveit_config_package,
  const std::string & planning_group,
  const std::string & end_effector_link);
PlanningReadinessReport validate_planning_readiness(const PlanningReadinessReport & seed);
DryRunPlanningRequest build_dry_run_planning_request(const PlanningReadinessReport & report);
std::string serialize_planning_readiness_yaml(const PlanningReadinessReport & report);
std::string serialize_planning_readiness_json(const PlanningReadinessReport & report);
std::string serialize_dry_run_request_yaml(const DryRunPlanningRequest & request);
void write_planning_readiness_artifacts(
  const std::string & out_dir,
  const PlanningReadinessReport & report,
  const DryRunPlanningRequest & request);

}  // namespace workcell_builder
