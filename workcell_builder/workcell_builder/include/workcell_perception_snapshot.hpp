#pragma once
#include <array>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "workcell_camera_model.hpp"
#include "workcell_zone_model.hpp"
#include "conveyor_pick_preview.hpp"

namespace workcell_builder {

struct PerceptionDetection {
  std::string id, class_label, zone_hint, tracking_id;
  double confidence{0.0};
  std::array<double, 2> center_px{{0.0, 0.0}};
  std::array<double, 4> bbox_px{{0.0, 0.0, 0.0, 0.0}};
  std::array<double, 3> estimated_xyz_camera{{0.0, 0.0, 0.0}};
  std::array<double, 3> estimated_xyz_world{{0.0, 0.0, 0.0}};
  bool has_world_xyz{false};
};

struct PerceptionSnapshot {
  int schema_version{1}; std::string source{"epd"}; std::string runtime_mode{"adapter_metadata_only"};
  std::string camera, camera_frame; double timestamp_sec{0.0}; std::vector<PerceptionDetection> detections;
};

struct DetectionAdapterResult {
  std::string detection_id, class_label, camera, detection_zone, conveyor_flow, pick_zone, state{"travelling"}, runtime_mode{"preview_only"};
  double distance_to_pick_m{0.0}, time_to_pick_s{0.0}; bool pick_ready{false}; bool robot_motion_commanded{false};
  std::vector<std::string> infos, warnings, errors;
};

PerceptionSnapshot parse_detection_snapshot_yaml(const std::string & path);
PerceptionSnapshot parse_detection_snapshot_json(const std::string & path);
std::string serialize_detection_snapshot_yaml(const PerceptionSnapshot & snapshot);
std::string serialize_detection_snapshot_json(const PerceptionSnapshot & snapshot);
std::vector<std::string> validate_detection_snapshot(const PerceptionSnapshot & snapshot, const std::vector<WorkcellCamera> & cameras, const std::vector<WorkZone> & zones, double min_confidence = 0.5);
std::vector<DetectionAdapterResult> map_detection_to_conveyor_preview(const PerceptionSnapshot & snapshot, const std::vector<WorkcellCamera> & cameras, const std::vector<WorkZone> & zones, const std::vector<ConveyorFlow> & flows);
PerceptionSnapshot generate_sample_epd_detection_snapshot(const std::string & camera_name, const std::string & camera_frame);
void write_detection_mapping_artifacts(const std::string & out_dir, const PerceptionSnapshot & snapshot, const std::vector<DetectionAdapterResult> & mapped);

}
