#pragma once

#include <string>
#include <vector>

#include "workcell_perception_snapshot.hpp"
#include "workcell_zone_model.hpp"

namespace workcell_builder {

struct ClassRoute {
  std::string class_label;
  std::string destination;
  std::string place_zone;
  int priority{0};
  bool enabled{true};
};

struct ClassRoutingTable {
  int schema_version{1};
  std::string runtime_mode{"preview_only"};
  std::string default_place_zone;
  std::vector<ClassRoute> routes;
};

struct ClassRoutingResult {
  std::string detection_id;
  std::string class_label;
  std::string selected_place_zone;
  std::string destination;
  std::string routing_status{"OK"};
  bool fallback_used{false};
  bool robot_motion_commanded{false};
  std::string runtime_mode{"preview_only"};
  std::vector<std::string> warnings;
  std::vector<std::string> errors;
};

ClassRoutingTable parse_class_routing_yaml(const std::string & yaml_path);
std::string serialize_class_routing_yaml(const ClassRoutingTable & table);
ClassRoutingResult validate_class_routing(const ClassRoutingTable & table, const std::vector<WorkZone> & zones, bool sorting_scenario = true);
ClassRoutingResult route_detection_class_to_place_zone(const ClassRoutingTable & table, const std::vector<WorkZone> & zones, const DetectionAdapterResult & mapping, bool sorting_scenario = true);
std::vector<ClassRoutingResult> route_detection_mapping_results(const ClassRoutingTable & table, const std::vector<WorkZone> & zones, const std::vector<DetectionAdapterResult> & mappings, bool sorting_scenario = true);
void write_class_routing_artifacts(const std::string & out_dir, const ClassRoutingTable & table, const ClassRoutingResult & result);

}  // namespace workcell_builder
