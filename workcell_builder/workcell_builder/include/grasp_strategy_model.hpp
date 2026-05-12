#pragma once

#include <string>
#include <vector>

namespace workcell_builder {

struct GraspStrategyRule {
  std::string class_label;
  std::string end_effector_type;
  std::string strategy;
  std::string approach_axis{"z_down"};
  double approach_distance_m{0.10};
  std::string retreat_axis{"z_up"};
  double retreat_distance_m{0.10};
  std::string preferred_pose_source{"live_epd_detection_mapping"};
  std::vector<std::string> required_inputs;
};

struct GraspStrategyTable {
  int schema_version{1};
  std::string runtime_mode{"preview_only"};
  std::string default_strategy{"generic_top_down"};
  std::vector<GraspStrategyRule> strategies;
};

struct EmdGraspPlannerRequest {
  int schema_version{1};
  std::string runtime_mode{"preview_only"};
  std::string source{"workcell_studio"};
  std::string planner_backend{"existing_emd_grasp_planner"};
  std::string perception_source{"live_epd_detection_mapping"};
  std::string robot;
  std::string end_effector;
  std::string end_effector_type;
  std::string object_class;
  std::string object_pose_source{"live_epd_detection_mapping"};
  std::string pick_zone;
  std::string place_zone;
  std::string grasp_strategy;
  double approach_distance_m{0.10};
  double retreat_distance_m{0.10};
  bool point_cloud_required{true};
  bool segmented_cloud_available{false};
  bool can_call_grasp_planner_later{false};
  bool robot_motion_commanded{false};
  bool gripper_command_sent{false};
  bool moveit_execute_called{false};
  bool grasp_execution_called{false};
  std::string emd_planner_config_file;
};

struct GraspStrategyReadinessResult {
  std::string readiness_status{"WARN"};
  std::vector<std::string> infos;
  std::vector<std::string> warnings;
  std::vector<std::string> errors;
  std::string selected_strategy;
  bool end_effector_compatible{false};
};

GraspStrategyTable parse_grasp_strategy_yaml(const std::string & yaml_text);
std::string serialize_grasp_strategy_yaml(const GraspStrategyTable & table);
GraspStrategyReadinessResult validate_grasp_strategy_table(const GraspStrategyTable & table);
GraspStrategyRule select_grasp_strategy_for_detection(const GraspStrategyTable & table, const std::string & object_class, const std::string & end_effector_type, bool * used_default = nullptr);
GraspStrategyReadinessResult validate_grasp_strategy_compatibility(const GraspStrategyRule & rule, const std::string & end_effector_type);
EmdGraspPlannerRequest build_emd_grasp_planner_request(const GraspStrategyRule & rule, const std::string & robot, const std::string & end_effector, const std::string & end_effector_type, const std::string & object_class, const std::string & object_pose_source, const std::string & pick_zone, const std::string & place_zone, bool segmented_cloud_available, bool has_point_cloud_source);
std::string serialize_emd_grasp_planner_request_yaml(const EmdGraspPlannerRequest & request);
std::string serialize_emd_grasp_planner_request_json(const EmdGraspPlannerRequest & request);
void write_emd_grasp_planner_request_artifacts(const std::string & out_dir, const GraspStrategyTable & table, const EmdGraspPlannerRequest & request, const GraspStrategyReadinessResult & readiness);

}  // namespace workcell_builder
