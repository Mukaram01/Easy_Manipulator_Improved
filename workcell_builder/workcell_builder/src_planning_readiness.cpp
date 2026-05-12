#include "planning_readiness.hpp"

#include <boost/filesystem.hpp>
#include <fstream>
#include <sstream>
#include <yaml-cpp/yaml.h>

namespace fs = boost::filesystem;
namespace workcell_builder {

PlanningReadinessReport build_planning_readiness_report(const std::string & robot, const std::string & end_effector, const std::vector<WorkZone> & zones, const std::vector<ConveyorFlow> & flows, const std::string & moveit_config_package, const std::string & planning_group, const std::string & end_effector_link) {
  PlanningReadinessReport r;
  r.robot = robot;
  r.end_effector = end_effector;
  r.planning_group = planning_group.empty() ? "manipulator" : planning_group;
  r.end_effector_link = end_effector_link.empty() ? "tool0" : end_effector_link;
  if (moveit_config_package.empty()) r.blockers.push_back("ERROR: no MoveIt config package known");
  for (const auto & z : zones) {
    if (r.pick_zone.empty() && z.type == "robot_pick") r.pick_zone = z.name;
    if (r.place_zone.empty() && z.type == "robot_place") r.place_zone = z.name;
  }
  if (!flows.empty() && flows.front().pick_ready == false) r.warnings.push_back("Pick not ready from conveyor preview");
  r.warnings.push_back("Object pose is zone-center estimate");
  r.warnings.push_back("Grasp strategy is generic");
  r.warnings.push_back("No collision validation against live object mesh");
  return validate_planning_readiness(r);
}

PlanningReadinessReport validate_planning_readiness(const PlanningReadinessReport & seed) {
  PlanningReadinessReport r = seed;
  if (r.robot.empty()) r.blockers.push_back("ERROR: no robot configured");
  if (r.planning_group.empty()) r.blockers.push_back("ERROR: no planning group known");
  if (r.end_effector_link.empty()) r.blockers.push_back("ERROR: no end-effector link known");
  if (r.pick_zone.empty()) r.blockers.push_back("ERROR: no pick zone");
  if (r.place_zone.empty()) r.blockers.push_back("ERROR: no place zone");
  r.can_attempt_plan = r.blockers.empty();
  r.can_execute = false;
  r.readiness_status = !r.blockers.empty() ? "ERROR" : (!r.warnings.empty() ? "WARN" : "OK");
  return r;
}

DryRunPlanningRequest build_dry_run_planning_request(const PlanningReadinessReport & report) {
  DryRunPlanningRequest req;
  req.robot = report.robot;
  req.planning_group = report.planning_group;
  return req;
}

std::string serialize_planning_readiness_yaml(const PlanningReadinessReport & r) {
  YAML::Emitter out;
  out << YAML::BeginMap << YAML::Key << "planning_readiness" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "schema_version" << YAML::Value << r.schema_version;
  out << YAML::Key << "runtime_mode" << YAML::Value << r.runtime_mode;
  out << YAML::Key << "robot" << YAML::Value << r.robot;
  out << YAML::Key << "end_effector" << YAML::Value << r.end_effector;
  out << YAML::Key << "pick_zone" << YAML::Value << r.pick_zone;
  out << YAML::Key << "place_zone" << YAML::Value << r.place_zone;
  out << YAML::Key << "planning_group" << YAML::Value << r.planning_group;
  out << YAML::Key << "end_effector_link" << YAML::Value << r.end_effector_link;
  out << YAML::Key << "readiness_status" << YAML::Value << r.readiness_status;
  out << YAML::Key << "can_attempt_plan" << YAML::Value << r.can_attempt_plan;
  out << YAML::Key << "can_execute" << YAML::Value << r.can_execute;
  out << YAML::Key << "blockers" << YAML::Value << r.blockers;
  out << YAML::Key << "warnings" << YAML::Value << r.warnings;
  out << YAML::Key << "safety" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "robot_motion_commanded" << YAML::Value << false;
  out << YAML::Key << "moveit_execute_called" << YAML::Value << false;
  out << YAML::Key << "gripper_command_sent" << YAML::Value << false;
  out << YAML::Key << "real_hardware_commanded" << YAML::Value << false;
  out << YAML::EndMap << YAML::EndMap << YAML::EndMap;
  return out.c_str();
}
std::string serialize_planning_readiness_json(const PlanningReadinessReport & r) {
  std::ostringstream ss;
  ss << "{\n  \"planning_readiness\": {\n    \"runtime_mode\": \"dry_run_readiness_only\",\n    \"can_attempt_plan\": " << (r.can_attempt_plan ? "true" : "false") << ",\n    \"can_execute\": false,\n    \"safety\": {\n      \"robot_motion_commanded\": false,\n      \"moveit_execute_called\": false,\n      \"gripper_command_sent\": false\n    }\n  }\n}";
  return ss.str();
}
std::string serialize_dry_run_request_yaml(const DryRunPlanningRequest & r) {
  YAML::Emitter out;
  out << YAML::BeginMap << YAML::Key << "dry_run_planning_request" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "schema_version" << YAML::Value << r.schema_version;
  out << YAML::Key << "robot" << YAML::Value << r.robot;
  out << YAML::Key << "planning_group" << YAML::Value << r.planning_group;
  out << YAML::Key << "start_state" << YAML::Value << r.start_state;
  out << YAML::Key << "execution_allowed" << YAML::Value << false;
  out << YAML::EndMap << YAML::EndMap;
  return out.c_str();
}
void write_planning_readiness_artifacts(const std::string & out_dir, const PlanningReadinessReport & report, const DryRunPlanningRequest & request) {
  fs::create_directories(out_dir);
  std::ofstream(out_dir + "/planning_readiness_report.yaml") << serialize_planning_readiness_yaml(report);
  std::ofstream(out_dir + "/planning_readiness_report.json") << serialize_planning_readiness_json(report);
  std::ofstream(out_dir + "/dry_run_planning_request.yaml") << serialize_dry_run_request_yaml(request);
}

}  // namespace workcell_builder
