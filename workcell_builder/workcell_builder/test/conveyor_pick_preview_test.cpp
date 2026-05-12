#include <gtest/gtest.h>
#include "conveyor_pick_preview.hpp"

using namespace workcell_builder;

TEST(ConveyorPickPreview, DistanceTime) {
  ConveyorFlow f; f.start_xyz={0,0,0}; f.end_xyz={1,0,0}; f.speed_mps=0.1;
  EXPECT_DOUBLE_EQ(compute_conveyor_flow_distance(f), 1.0);
  EXPECT_DOUBLE_EQ(compute_time_to_pick(1.0,0.1), 10.0);
}

TEST(ConveyorPickPreview, TrajectorySampling) {
  ConveyorFlow f; f.start_xyz={0,0,0}; f.end_xyz={1,0,0};
  auto pts = sample_conveyor_trajectory(f, 5);
  ASSERT_EQ(pts.size(), 5u);
  EXPECT_DOUBLE_EQ(pts.front()[0], 0.0);
  EXPECT_DOUBLE_EQ(pts.back()[0], 1.0);
  for (size_t i=1;i<pts.size();++i) EXPECT_GE(pts[i][0], pts[i-1][0]);
}

TEST(ConveyorPickPreview, ValidationAndSerialization) {
  WorkZone d; d.name="detection_zone_1"; d.type="camera_detection";
  WorkZone p; p.name="pick_zone_1"; p.type="robot_pick";
  ConveyorFlow f; f.name="conveyor_flow_1"; f.detection_zone=d.name; f.pick_zone=p.name; f.speed_mps=0.1; f.start_xyz={0,0,0}; f.end_xyz={1,0,0};
  auto valid = generate_preview_result({d,p}, f);
  EXPECT_TRUE(valid.valid);
  auto y = serialize_preview_to_yaml(valid); auto j=serialize_preview_to_json(valid);
  EXPECT_NE(y.find("flow_name"), std::string::npos);
  EXPECT_NE(y.find("preview_only"), std::string::npos);
  EXPECT_NE(j.find("robot_motion_commanded\": false"), std::string::npos);

  f.detection_zone="missing";
  auto invalid = validate_conveyor_pick_preview({d,p}, f);
  EXPECT_FALSE(invalid.valid);
}
