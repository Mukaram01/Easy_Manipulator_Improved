#pragma once

#include <string>
#include <vector>

#include "workcell_perception_snapshot.hpp"
#include "conveyor_pick_preview.hpp"
#include "workcell_zone_model.hpp"

namespace workcell_builder {

struct TaskIntentStep { std::string name; };

struct TaskIntentPreview {
  int schema_version{1};
  std::string runtime_mode{"preview_only"};
  std::string source_detection;
  std::string class_label;
  std::string robot;
  std::string end_effector;
  std::string detection_zone;
  std::string pick_zone;
  std::string place_zone;
  std::string conveyor_flow;
  double time_to_pick_s{0.0};
  bool pick_ready{false};
  bool robot_motion_commanded{false};
  bool moveit_plan_service_called{false};
  bool gripper_command_sent{false};
  std::vector<TaskIntentStep> task_steps;
};

struct TaskIntentReadinessResult {
  std::string status{"OK"};
  std::vector<std::string> blockers;
  std::vector<std::string> warnings;
  std::vector<std::string> infos;
};

TaskIntentPreview build_task_intent_preview(
  const std::string & robot,
  const std::string & end_effector,
  const std::vector<WorkZone> & zones,
  const std::vector<ConveyorFlow> & flows,
  const DetectionAdapterResult & mapping);
TaskIntentReadinessResult validate_task_intent_preview(const TaskIntentPreview & preview, bool task_requires_grasping = true);
std::string serialize_task_intent_preview_yaml(const TaskIntentPreview & preview, const TaskIntentReadinessResult & readiness);
std::string serialize_task_intent_preview_json(const TaskIntentPreview & preview, const TaskIntentReadinessResult & readiness);
void write_task_intent_preview_artifacts(const std::string & out_dir, const TaskIntentPreview & preview, const TaskIntentReadinessResult & readiness);

}  // namespace workcell_builder
