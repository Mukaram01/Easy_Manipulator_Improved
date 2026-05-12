#include "task_intent_readiness.hpp"
#include "class_routing_model.hpp"

#include <boost/filesystem.hpp>
#include <fstream>
#include <sstream>
#include <unordered_set>
#include <yaml-cpp/yaml.h>

namespace fs = boost::filesystem;
namespace workcell_builder {

TaskIntentPreview build_task_intent_preview(const std::string & robot, const std::string & end_effector, const std::vector<WorkZone> & zones, const std::vector<ConveyorFlow> & flows, const DetectionAdapterResult & mapping) {
  TaskIntentPreview p;
  p.source_detection = mapping.detection_id;
  p.class_label = mapping.class_label;
  p.robot = robot;
  p.end_effector = end_effector;
  p.detection_zone = mapping.detection_zone;
  p.pick_zone = mapping.pick_zone;
  p.conveyor_flow = mapping.conveyor_flow;
  p.time_to_pick_s = mapping.time_to_pick_s;
  // class-to-place-zone routing hook: place_zone may be overwritten by routing preview artifacts

  for (const auto & z : zones) {
    if (z.type == "robot_place" && p.place_zone.empty()) p.place_zone = z.name;
    if (p.pick_zone.empty() && z.type == "robot_pick") p.pick_zone = z.name;
  }
  if (p.conveyor_flow.empty() && !flows.empty()) p.conveyor_flow = flows.front().name;

  const bool static_overlap = !p.detection_zone.empty() && p.detection_zone == p.pick_zone;
  p.pick_ready = mapping.pick_ready || static_overlap || p.time_to_pick_s <= 0.0;
  p.task_steps = {{"wait_for_detection"}, {"wait_for_object_in_pick_zone"}, {"approach_pick"}, {"grasp_preview"}, {"retreat_pick"}, {"move_to_place_zone"}, {"release_preview"}, {"complete_preview"}};
  return p;
}

TaskIntentReadinessResult validate_task_intent_preview(const TaskIntentPreview & p, bool task_requires_grasping) {
  TaskIntentReadinessResult r;
  if (p.robot.empty()) r.blockers.push_back("ERROR: no robot configured");
  if (p.pick_zone.empty()) r.blockers.push_back("ERROR: no pick zone configured");
  if (p.place_zone.empty()) r.blockers.push_back("ERROR: no place zone configured");
  if (p.source_detection.empty()) r.blockers.push_back("ERROR: detection mapping missing");
  if (task_requires_grasping && p.end_effector.empty()) r.blockers.push_back("ERROR: no end effector configured when task requires grasping");
  if (!p.pick_ready) r.warnings.push_back("WARN: pick_ready is false because object has not reached pick zone yet");
  r.warnings.push_back("WARN: preview-only, no planning performed");
  r.warnings.push_back("WARN: no approach/retreat offsets configured");
  r.warnings.push_back("WARN: no grasp strategy configured");
  r.infos = {"INFO: no robot motion commanded", "INFO: no MoveIt call", "INFO: no gripper command", "INFO: no EPD runtime launched", "INFO: fake hardware preview recommended"};
  r.status = !r.blockers.empty() ? "ERROR" : (!r.warnings.empty() ? "WARN" : "OK");
  return r;
}

std::string serialize_task_intent_preview_yaml(const TaskIntentPreview & p, const TaskIntentReadinessResult & r) {
  YAML::Emitter out;
  out << YAML::BeginMap << YAML::Key << "task_intent_preview" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "schema_version" << YAML::Value << p.schema_version;
  out << YAML::Key << "runtime_mode" << YAML::Value << p.runtime_mode;
  out << YAML::Key << "source_detection" << YAML::Value << p.source_detection;
  out << YAML::Key << "class_label" << YAML::Value << p.class_label;
  out << YAML::Key << "robot" << YAML::Value << p.robot;
  out << YAML::Key << "end_effector" << YAML::Value << p.end_effector;
  out << YAML::Key << "detection_zone" << YAML::Value << p.detection_zone;
  out << YAML::Key << "pick_zone" << YAML::Value << p.pick_zone;
  out << YAML::Key << "place_zone" << YAML::Value << p.place_zone;
  out << YAML::Key << "conveyor_flow" << YAML::Value << p.conveyor_flow;
  out << YAML::Key << "time_to_pick_s" << YAML::Value << p.time_to_pick_s;
  out << YAML::Key << "pick_ready" << YAML::Value << p.pick_ready;
  out << YAML::Key << "robot_motion_commanded" << YAML::Value << p.robot_motion_commanded;
  out << YAML::Key << "moveit_plan_service_called" << YAML::Value << p.moveit_plan_service_called;
  out << YAML::Key << "gripper_command_sent" << YAML::Value << p.gripper_command_sent;
  out << YAML::Key << "task_steps" << YAML::Value << YAML::BeginSeq; for (const auto & s : p.task_steps) out << s.name; out << YAML::EndSeq;
  out << YAML::Key << "readiness" << YAML::Value << YAML::BeginMap << YAML::Key << "status" << YAML::Value << r.status;
  out << YAML::Key << "blockers" << YAML::Value << r.blockers << YAML::Key << "warnings" << YAML::Value << r.warnings << YAML::EndMap;
  out << YAML::EndMap << YAML::EndMap;
  return out.c_str();
}
std::string serialize_task_intent_preview_json(const TaskIntentPreview & p, const TaskIntentReadinessResult & r) {
  std::ostringstream ss;
  ss << "{\n  \"task_intent_preview\": {\n    \"runtime_mode\": \"preview_only\",\n    \"pick_zone\": \"" << p.pick_zone << "\",\n    \"place_zone\": \"" << p.place_zone << "\",\n    \"robot_motion_commanded\": false,\n    \"moveit_plan_service_called\": false,\n    \"gripper_command_sent\": false,\n    \"task_steps\": [\"wait_for_detection\"],\n    \"readiness_status\": \"" << r.status << "\"\n  }\n}";
  return ss.str();
}
void write_task_intent_preview_artifacts(const std::string & out_dir, const TaskIntentPreview & p, const TaskIntentReadinessResult & r) {
  fs::create_directories(out_dir);
  std::ofstream(out_dir + "/task_intent_preview.yaml") << serialize_task_intent_preview_yaml(p, r);
  std::ofstream(out_dir + "/task_intent_preview.json") << serialize_task_intent_preview_json(p, r);
}

}  // namespace workcell_builder
