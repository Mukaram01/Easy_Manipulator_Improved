#include "conveyor_pick_preview.hpp"

#include <cmath>
#include <sstream>
#include <unordered_map>
#include <yaml-cpp/yaml.h>

namespace workcell_builder {
namespace {
std::array<double, 3> vec3(const std::vector<double> & in) {
  if (in.size() < 3) return {{0.0, 0.0, 0.0}};
  return {{in[0], in[1], in[2]}};
}
}

double compute_conveyor_flow_distance(const ConveyorFlow & flow) {
  const auto s = vec3(flow.start_xyz); const auto e = vec3(flow.end_xyz);
  const double dx = e[0] - s[0], dy = e[1] - s[1], dz = e[2] - s[2];
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double compute_time_to_pick(double distance_m, double speed_mps) {
  return speed_mps > 0.0 ? distance_m / speed_mps : 0.0;
}

std::vector<std::array<double, 3>> sample_conveyor_trajectory(const ConveyorFlow & flow, std::size_t samples) {
  std::vector<std::array<double, 3>> out;
  if (samples < 2) samples = 2;
  const auto s = vec3(flow.start_xyz); const auto e = vec3(flow.end_xyz);
  for (std::size_t i = 0; i < samples; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(samples - 1);
    out.push_back({{s[0] + (e[0] - s[0]) * t, s[1] + (e[1] - s[1]) * t, s[2] + (e[2] - s[2]) * t}});
  }
  return out;
}

ConveyorPickPreviewResult validate_conveyor_pick_preview(const std::vector<WorkZone> & zones, const ConveyorFlow & flow) {
  ConveyorPickPreviewResult r;
  r.flow_name = flow.name; r.detection_zone = flow.detection_zone; r.pick_zone = flow.pick_zone; r.speed_mps = flow.speed_mps;
  std::unordered_map<std::string, WorkZone> by_name; for (const auto & z : zones) by_name[z.name] = z;
  if (flow.detection_zone.empty() || by_name.find(flow.detection_zone) == by_name.end()) r.warnings.push_back("ERROR: conveyor_flow references missing detection zone");
  if (flow.pick_zone.empty() || by_name.find(flow.pick_zone) == by_name.end()) r.warnings.push_back("ERROR: conveyor_flow references missing pick zone");
  if (flow.speed_mps <= 0.0) r.warnings.push_back("ERROR: speed_mps <= 0");
  if (flow.start_xyz.size() < 3 || flow.end_xyz.size() < 3) r.warnings.push_back("ERROR: start/end points invalid");
  r.valid = true;
  for (const auto & w : r.warnings) if (w.rfind("ERROR:", 0) == 0) r.valid = false;
  if (r.valid && by_name.count(flow.detection_zone) && by_name.count(flow.pick_zone)) {
    const auto & dz = by_name[flow.detection_zone]; const auto & pz = by_name[flow.pick_zone];
    if (dz.center_xyz == pz.center_xyz) r.warnings.push_back("WARN: detection zone and pick zone overlap");
    if (dz.linked_robot.empty()) r.warnings.push_back("WARN: no linked robot for pick zone");
  }
  return r;
}

ConveyorPickPreviewResult generate_preview_result(const std::vector<WorkZone> & zones, const ConveyorFlow & flow) {
  auto r = validate_conveyor_pick_preview(zones, flow);
  r.distance_m = compute_conveyor_flow_distance(flow);
  r.time_to_pick_s = compute_time_to_pick(r.distance_m, flow.speed_mps);
  r.trajectory_points = sample_conveyor_trajectory(flow, 10);
  r.states = {"detected", "travelling", "entering_pick_zone", "pick_ready"};
  if (r.time_to_pick_s < 0.5) r.warnings.push_back("WARN: time_to_pick_s is very short");
  r.warnings.push_back("INFO: preview_only");
  r.warnings.push_back("INFO: no robot motion commanded");
  r.warnings.push_back("INFO: no real conveyor commanded");
  return r;
}

std::string serialize_preview_to_yaml(const ConveyorPickPreviewResult & result) {
  YAML::Emitter out;
  out << YAML::BeginMap << YAML::Key << "conveyor_pick_preview" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "flow_name" << YAML::Value << result.flow_name;
  out << YAML::Key << "detection_zone" << YAML::Value << result.detection_zone;
  out << YAML::Key << "pick_zone" << YAML::Value << result.pick_zone;
  out << YAML::Key << "speed_mps" << YAML::Value << result.speed_mps;
  out << YAML::Key << "distance_m" << YAML::Value << result.distance_m;
  out << YAML::Key << "time_to_pick_s" << YAML::Value << result.time_to_pick_s;
  out << YAML::Key << "runtime_mode" << YAML::Value << "preview_only";
  out << YAML::Key << "preview_only" << YAML::Value << true;
  out << YAML::Key << "robot_motion_commanded" << YAML::Value << false;
  out << YAML::Key << "real_conveyor_commanded" << YAML::Value << false;
  out << YAML::EndMap << YAML::EndMap;
  return out.c_str();
}
std::string serialize_preview_to_json(const ConveyorPickPreviewResult & result) {
  std::ostringstream ss;
  ss << "{\n  \"conveyor_pick_preview\": {\n"
     << "    \"flow_name\": \"" << result.flow_name << "\",\n"
     << "    \"detection_zone\": \"" << result.detection_zone << "\",\n"
     << "    \"pick_zone\": \"" << result.pick_zone << "\",\n"
     << "    \"speed_mps\": " << result.speed_mps << ",\n"
     << "    \"distance_m\": " << result.distance_m << ",\n"
     << "    \"time_to_pick_s\": " << result.time_to_pick_s << ",\n"
     << "    \"preview_only\": true,\n"
     << "    \"runtime_mode\": \"preview_only\",\n"
     << "    \"robot_motion_commanded\": false,\n"
     << "    \"real_conveyor_commanded\": false\n"
     << "  }\n}";
  return ss.str();
}
}  // namespace workcell_builder
