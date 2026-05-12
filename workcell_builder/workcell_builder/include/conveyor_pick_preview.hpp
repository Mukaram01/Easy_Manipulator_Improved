#pragma once

#include <array>
#include <string>
#include <vector>

#include "workcell_zone_model.hpp"

namespace workcell_builder {

struct ConveyorPreviewObject {
  std::string id;
  std::string class_label;
  double detection_time_s{0.0};
  std::array<double, 3> start_xyz{{0.0, 0.0, 0.0}};
  std::array<double, 3> pick_xyz{{0.0, 0.0, 0.0}};
};

struct ConveyorPickPreviewResult {
  std::string flow_name;
  std::string detection_zone;
  std::string pick_zone;
  double speed_mps{0.0};
  double distance_m{0.0};
  double time_to_pick_s{0.0};
  std::vector<std::array<double, 3>> trajectory_points;
  std::vector<std::string> states;
  std::vector<std::string> warnings;
  bool valid{false};
};

double compute_conveyor_flow_distance(const ConveyorFlow & flow);
double compute_time_to_pick(double distance_m, double speed_mps);
std::vector<std::array<double, 3>> sample_conveyor_trajectory(const ConveyorFlow & flow, std::size_t samples = 10);
ConveyorPickPreviewResult validate_conveyor_pick_preview(const std::vector<WorkZone> & zones, const ConveyorFlow & flow);
ConveyorPickPreviewResult generate_preview_result(const std::vector<WorkZone> & zones, const ConveyorFlow & flow);
std::string serialize_preview_to_yaml(const ConveyorPickPreviewResult & result);
std::string serialize_preview_to_json(const ConveyorPickPreviewResult & result);

}  // namespace workcell_builder
