#pragma once

#include <string>
#include <array>
#include <optional>
#include <vector>

namespace workcell_builder
{

struct TaskIntentPose { std::array<double, 3> xyz_m{}; std::array<double, 3> rpy_rad{}; };
struct TaskIntentSelection { std::string source_ref; std::string source_type; std::string zone_ref; std::string class_id; std::optional<std::string> color; std::optional<double> min_confidence; std::optional<double> max_age_seconds; };
struct TaskIntentGrasp { std::string policy{"AUTO"}; std::string required_capability; std::optional<std::string> strategy_ref; std::string approach_axis; double approach_distance_m{0.0}; std::string orientation_mode; std::vector<double> allowed_roll_deg; std::vector<double> allowed_yaw_deg; std::array<double, 3> tolerance_rad{}; std::array<double, 3> tcp_offset_xyz_m{}; std::array<double, 3> tcp_offset_rpy_rad{}; double contact_min_quality{0.0}; double aperture_min_m{0.0}; double aperture_max_m{0.0}; bool contact_required{false}; std::string lift_axis; double lift_distance_m{0.0}; };
struct TaskIntentPlacement { std::string policy{"AUTO"}; std::string asset_ref; std::string region_ref; std::optional<TaskIntentPose> requested_local_pose; std::string orientation_mode; std::array<double, 3> orientation_rpy_rad{}; std::array<double, 3> orientation_tolerance_rad{}; std::string approach_axis; double approach_distance_m{0.0}; double clearance_m{0.0}; std::string retreat_axis; double retreat_distance_m{0.0}; };
struct TaskIntentSafety { std::string execution_mode{"simulation_preview"}; bool require_fake_hardware{true}; bool real_hardware_enabled{false}; std::string preview_policy{"diagnostic_if_unresolved"}; };
struct TaskIntentModel {
  std::string schema{"workcell_builder_task_intent/v2"};
  std::string scene_package; std::string routing_yaml;
  std::string task_id; std::string task_type; std::string task_template;
  TaskIntentSelection pick_selection; TaskIntentGrasp grasp; TaskIntentPlacement place; std::string release_strategy{"tool_release"}; TaskIntentSafety safety;
  std::string migration_provenance;
  bool validated{false};
  std::string normalized_yaml;
  static TaskIntentModel from_yaml(const std::string & yaml_text);
  static std::optional<TaskIntentModel> from_validated_yaml(const std::string & yaml_text);
};

// Canonical semantic JSON is UTF-8, recursively key-sorted, arrays preserved,
// and all finite numbers rendered as unquoted decimal tokens (12 places max). The
// SHA-256 of those bytes is the v2 intent hash shared with Python.
std::string canonical_task_intent_json_for_testing(const std::string & yaml_text);
std::string canonical_task_intent_sha256_for_testing(const std::string & yaml_text);
std::string authoritative_task_intent_sha256(const TaskIntentModel & validated_model);

}  // namespace workcell_builder
