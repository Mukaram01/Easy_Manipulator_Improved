#include <gtest/gtest.h>
#include <fstream>
#include "workcell_perception_snapshot.hpp"
using namespace workcell_builder;

TEST(PerceptionSnapshot, YamlRoundtripAndMapping) {
  auto s = generate_sample_epd_detection_snapshot("realsense_d435i_1", "camera_color_optical_frame");
  auto y = serialize_detection_snapshot_yaml(s); std::ofstream("/tmp/epd_snapshot.yaml") << y;
  auto p = parse_detection_snapshot_yaml("/tmp/epd_snapshot.yaml"); EXPECT_EQ(p.camera, s.camera); EXPECT_EQ(p.detections.size(), 1u);
  WorkcellCamera c; c.name = "realsense_d435i_1";
  WorkZone dz; dz.name="detection_zone_1"; dz.type="camera_detection"; dz.center_xyz={0.6,0.0,0.8}; dz.size_xyz={1.0,1.0,1.0};
  WorkZone pz; pz.name="pick_zone_1"; pz.type="robot_pick";
  ConveyorFlow f; f.name="conveyor_flow_1"; f.detection_zone="detection_zone_1"; f.pick_zone="pick_zone_1"; f.speed_mps=0.1; f.start_xyz={0,0,0}; f.end_xyz={0.85,0,0};
  auto m = map_detection_to_conveyor_preview(p, {c}, {dz,pz}, {f}); EXPECT_EQ(m.front().conveyor_flow, "conveyor_flow_1"); EXPECT_FALSE(m.front().robot_motion_commanded);
}

TEST(PerceptionSnapshot, JsonAndValidation) {
  auto s = generate_sample_epd_detection_snapshot("unknown_cam", "camera_frame");
  auto j = serialize_detection_snapshot_json(s); std::ofstream("/tmp/epd_snapshot.json") << j;
  auto p = parse_detection_snapshot_json("/tmp/epd_snapshot.json"); EXPECT_EQ(p.source, "epd");
  auto errors = validate_detection_snapshot(s, {}, {}, 0.95);
  EXPECT_TRUE(std::any_of(errors.begin(), errors.end(), [](const std::string &e){ return e.find("ERROR: snapshot camera") != std::string::npos; }));
}
