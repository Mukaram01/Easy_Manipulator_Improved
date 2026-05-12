#include <gtest/gtest.h>
#include "task_intent_readiness.hpp"

using namespace workcell_builder;

TEST(TaskIntentReadiness, BuildValid) {
  DetectionAdapterResult m; m.detection_id="det_001"; m.class_label="box"; m.detection_zone="detection_zone_1"; m.pick_zone="pick_zone_1"; m.time_to_pick_s=1.0;
  std::vector<WorkZone> z{{"detection_zone_1","camera_detection"},{"pick_zone_1","robot_pick"},{"place_zone_1","robot_place"}};
  auto p=build_task_intent_preview("ur5","robotiq_2f_85",z,{},m);
  auto r=validate_task_intent_preview(p,true);
  EXPECT_EQ(p.task_steps.size(),8);
  EXPECT_EQ(r.status,"WARN");
}

TEST(TaskIntentReadiness, MissingRobot) { TaskIntentPreview p; p.source_detection="det"; p.pick_zone="p"; p.place_zone="pl"; auto r=validate_task_intent_preview(p); EXPECT_EQ(r.status,"ERROR"); }
TEST(TaskIntentReadiness, MissingPlaceZone) { TaskIntentPreview p; p.source_detection="det"; p.pick_zone="p"; p.robot="ur5"; p.end_effector="ee"; auto r=validate_task_intent_preview(p); EXPECT_EQ(r.status,"ERROR"); }
TEST(TaskIntentReadiness, ConveyorNotReadyWarn) { TaskIntentPreview p; p.source_detection="det"; p.pick_zone="p"; p.place_zone="pl"; p.robot="ur5"; p.end_effector="ee"; p.pick_ready=false; auto r=validate_task_intent_preview(p); EXPECT_EQ(r.status,"WARN"); }
TEST(TaskIntentReadiness, StaticPickReady) { DetectionAdapterResult m; m.detection_id="det"; m.detection_zone="pick_zone_1"; std::vector<WorkZone> z{{"pick_zone_1","robot_pick"},{"place_zone_1","robot_place"}}; auto p=build_task_intent_preview("ur5","ee",z,{},m); EXPECT_TRUE(p.pick_ready); }
TEST(TaskIntentReadiness, SerializationFields) { TaskIntentPreview p; p.source_detection="det"; p.pick_zone="pz"; p.place_zone="plz"; auto r=validate_task_intent_preview(p,false); auto y=serialize_task_intent_preview_yaml(p,r); EXPECT_NE(y.find("runtime_mode"),std::string::npos); EXPECT_NE(y.find("robot_motion_commanded"),std::string::npos); EXPECT_NE(y.find("moveit_plan_service_called"),std::string::npos); EXPECT_NE(y.find("gripper_command_sent"),std::string::npos); }
