#pragma once
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace workcell_builder {
struct WorkZone {
  std::string name;
  std::string type;
  std::string parent_frame{"world"};
  std::vector<double> center_xyz{0.0,0.0,0.8};
  std::vector<double> size_xyz{0.3,0.3,0.02};
  std::vector<double> rpy{0.0,0.0,0.0};
  std::string linked_camera;
  std::string linked_robot;
  std::string destination;
  std::string purpose;
  std::string runtime_mode{"metadata_only"};
};
struct ConveyorFlow {
  std::string name;
  std::string parent_frame{"world"};
  std::vector<double> start_xyz{0.0,0.0,0.82};
  std::vector<double> end_xyz{0.8,0.0,0.82};
  std::vector<double> direction_xyz{1.0,0.0,0.0};
  double speed_mps{0.10};
  std::string detection_zone;
  std::string pick_zone;
  std::string runtime_mode{"metadata_only"};
};
struct WorkZoneValidationResult { bool ok{true}; std::vector<std::string> warnings; std::vector<std::string> errors; std::vector<std::string> infos; };
WorkZone default_detection_zone_for_camera(const std::string & camera_name);
WorkZone default_pick_zone_for_robot(const std::string & robot_name);
WorkZone default_place_zone(const std::string & destination);
void serialize_work_zones_to_yaml(YAML::Emitter *out, const std::vector<WorkZone>& zones, const std::vector<ConveyorFlow>& flows);
void parse_work_zones_from_yaml(const YAML::Node &root, std::vector<WorkZone>* zones, std::vector<ConveyorFlow>* flows);
WorkZoneValidationResult validate_work_zones(const std::vector<WorkZone>& zones, const std::vector<ConveyorFlow>& flows, const std::vector<std::string>& cameras, const std::vector<std::string>& robots);
std::string generate_zone_preview_markers_or_geometry(const WorkZone & zone);
std::string generate_conveyor_flow_preview_arrow(const ConveyorFlow & flow);
}
