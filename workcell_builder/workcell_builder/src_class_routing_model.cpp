#include "class_routing_model.hpp"

#include <boost/filesystem.hpp>
#include <fstream>
#include <set>
#include <sstream>
#include <yaml-cpp/yaml.h>

namespace fs = boost::filesystem;
namespace workcell_builder {

static bool zone_exists(const std::vector<WorkZone> & zones, const std::string & name) { for (const auto & z : zones) if (z.name == name) return true; return false; }

ClassRoutingTable parse_class_routing_yaml(const std::string & yaml_path) {
  ClassRoutingTable t;
  const YAML::Node root = YAML::LoadFile(yaml_path);
  const YAML::Node cr = root["class_routing"];
  if (!cr) return t;
  if (cr["schema_version"]) t.schema_version = cr["schema_version"].as<int>();
  if (cr["runtime_mode"]) t.runtime_mode = cr["runtime_mode"].as<std::string>();
  if (cr["default_place_zone"]) t.default_place_zone = cr["default_place_zone"].as<std::string>();
  if (cr["routes"]) for (const auto & n : cr["routes"]) t.routes.push_back({n["class_label"].as<std::string>(""), n["destination"].as<std::string>(""), n["place_zone"].as<std::string>(""), n["priority"].as<int>(0), n["enabled"].as<bool>(true)});
  return t;
}

std::string serialize_class_routing_yaml(const ClassRoutingTable & t) {
  YAML::Emitter out;
  out << YAML::BeginMap << YAML::Key << "class_routing" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "schema_version" << YAML::Value << t.schema_version;
  out << YAML::Key << "runtime_mode" << YAML::Value << t.runtime_mode;
  out << YAML::Key << "default_place_zone" << YAML::Value << t.default_place_zone;
  out << YAML::Key << "routes" << YAML::Value << YAML::BeginSeq;
  for (const auto & r : t.routes) out << YAML::BeginMap << YAML::Key << "class_label" << YAML::Value << r.class_label << YAML::Key << "destination" << YAML::Value << r.destination << YAML::Key << "place_zone" << YAML::Value << r.place_zone << YAML::Key << "priority" << YAML::Value << r.priority << YAML::Key << "enabled" << YAML::Value << r.enabled << YAML::EndMap;
  out << YAML::EndSeq << YAML::EndMap << YAML::EndMap;
  return out.c_str();
}

ClassRoutingResult validate_class_routing(const ClassRoutingTable & t, const std::vector<WorkZone> & zones, bool sorting_scenario) {
  ClassRoutingResult r; r.runtime_mode = t.runtime_mode;
  std::set<std::string> seen;
  for (const auto & it : t.routes) {
    if (!it.enabled) r.warnings.push_back("WARN: route disabled");
    if (it.class_label.empty()) r.warnings.push_back("WARN: class label empty");
    if (it.destination == "bin" || it.destination == "generic") r.warnings.push_back("WARN: destination is generic");
    if (!it.place_zone.empty() && !zone_exists(zones, it.place_zone)) r.errors.push_back("ERROR: route references missing place_zone");
    std::string k = it.class_label + "#" + std::to_string(it.priority);
    if (it.enabled && seen.count(k)) r.errors.push_back("ERROR: duplicate class_label routes with same priority and enabled true");
    seen.insert(k);
  }
  if (sorting_scenario && t.default_place_zone.empty()) r.errors.push_back("ERROR: no default route for unknown class in sorting scenario");
  if (!sorting_scenario && t.default_place_zone.empty()) r.warnings.push_back("WARN: default route missing but not sorting scenario");
  r.routing_status = r.errors.empty() ? (r.warnings.empty() ? "OK" : "WARN") : "ERROR";
  return r;
}

ClassRoutingResult route_detection_class_to_place_zone(const ClassRoutingTable & t, const std::vector<WorkZone> & zones, const DetectionAdapterResult & m, bool sorting_scenario) {
  ClassRoutingResult out = validate_class_routing(t, zones, sorting_scenario);
  out.detection_id = m.detection_id;
  out.class_label = m.class_label;
  int best_priority = -999999;
  for (const auto & route : t.routes) {
    if (route.enabled && route.class_label == m.class_label && route.priority >= best_priority) {
      best_priority = route.priority;
      out.selected_place_zone = route.place_zone;
      out.destination = route.destination;
      out.fallback_used = false;
    }
  }
  if (out.selected_place_zone.empty()) {
    out.selected_place_zone = t.default_place_zone;
    out.destination = "reject_bin";
    out.fallback_used = true;
    out.warnings.push_back("WARN: unknown class used fallback route");
  }
  if (out.selected_place_zone.empty()) out.errors.push_back("ERROR: no place zone available for routed class");
  if (!out.selected_place_zone.empty() && !zone_exists(zones, out.selected_place_zone)) out.errors.push_back("ERROR: route place_zone does not exist");
  out.routing_status = out.errors.empty() ? (out.warnings.empty() ? "OK" : "WARN") : "ERROR";
  return out;
}

std::vector<ClassRoutingResult> route_detection_mapping_results(const ClassRoutingTable & table, const std::vector<WorkZone> & zones, const std::vector<DetectionAdapterResult> & mappings, bool sorting_scenario) {
  std::vector<ClassRoutingResult> out; for (const auto & m : mappings) out.push_back(route_detection_class_to_place_zone(table, zones, m, sorting_scenario)); return out;
}

void write_class_routing_artifacts(const std::string & out_dir, const ClassRoutingTable & t, const ClassRoutingResult & r) {
  fs::create_directories(out_dir);
  std::ofstream(out_dir + "/class_routing_table.yaml") << serialize_class_routing_yaml(t);
  YAML::Emitter y; y << YAML::BeginMap << YAML::Key << "routing_result" << YAML::Value << YAML::BeginMap << YAML::Key << "detection_id" << YAML::Value << r.detection_id << YAML::Key << "class_label" << YAML::Value << r.class_label << YAML::Key << "selected_place_zone" << YAML::Value << r.selected_place_zone << YAML::Key << "destination" << YAML::Value << r.destination << YAML::Key << "routing_status" << YAML::Value << r.routing_status << YAML::Key << "fallback_used" << YAML::Value << r.fallback_used << YAML::Key << "robot_motion_commanded" << YAML::Value << false << YAML::Key << "runtime_mode" << YAML::Value << "preview_only" << YAML::EndMap << YAML::EndMap;
  std::ofstream(out_dir + "/class_routing_result.yaml") << y.c_str();
  std::ofstream(out_dir + "/class_routing_result.json") << "{\n  \"routing_result\": {\n    \"selected_place_zone\": \"" << r.selected_place_zone << "\",\n    \"routing_status\": \"" << r.routing_status << "\",\n    \"robot_motion_commanded\": false,\n    \"runtime_mode\": \"preview_only\"\n  }\n}";
}

}  // namespace workcell_builder
