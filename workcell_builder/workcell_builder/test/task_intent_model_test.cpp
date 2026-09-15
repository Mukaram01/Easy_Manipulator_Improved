#include "task_intent_model.hpp"

#include <gtest/gtest.h>

TEST(TaskIntentModel, CanonicalHashGoldenIsStableAcrossYamlFormatting)
{
  const std::string a = "b: 2\na: [0.0, 1]\n";
  const std::string b = "# comment\na: [0, 1.000000]\nb: 2.0\n";
  EXPECT_EQ(workcell_builder::canonical_task_intent_sha256(a), workcell_builder::canonical_task_intent_sha256(b));
  EXPECT_EQ(workcell_builder::canonical_task_intent_sha256(a), "6b5a43d11c069f69bf75b5de430b4caf6a36fb7a8e7e5840c15130ea44f92fd3");
}

TEST(TaskIntentModel, CanonicalRepresentationPreservesScalarTypesAndArrayOrder)
{
  const auto numeric = workcell_builder::canonical_task_intent_json("x: 1\ny: true\nz: null\na: [1, 2]\n");
  const auto strings = workcell_builder::canonical_task_intent_json("x: '1'\ny: 'true'\nz: 'null'\na: [2, 1]\n");
  EXPECT_NE(numeric, strings);
  EXPECT_NE(workcell_builder::canonical_task_intent_sha256("x: 1\n"), workcell_builder::canonical_task_intent_sha256("x: '1'\n"));
  EXPECT_NE(workcell_builder::canonical_task_intent_sha256("x: true\n"), workcell_builder::canonical_task_intent_sha256("x: 'true'\n"));
  EXPECT_NE(workcell_builder::canonical_task_intent_sha256("x: null\n"), workcell_builder::canonical_task_intent_sha256("x: 'null'\n"));
}

TEST(TaskIntentModel, ParsesTheTypedV2AuthoredShape)
{
  const auto model = workcell_builder::TaskIntentModel::from_yaml(
    "schema: workcell_builder_task_intent/v2\n"
    "task: {id: t, type: pick_place, template: pick_place}\n"
    "pick: {selection: {source_ref: detected_objects/v1, source_type: perception, zone_ref: pick_zone_main, object_filter: {class_id: bottle, color: red, min_confidence: 0.8, max_age_seconds: 2.0}}, grasp: {policy: EXACT, required_capability: two_finger_parallel, strategy_ref: top_2f, approach: {axis: z_down, distance_m: 0.12}, orientation: {mode: vertical, allowed_roll_deg: [0, 90], allowed_yaw_deg: [0, 180], tolerance_rad: [0.1, 0.2, 0.3]}, tcp_offset_xyz_m: [1, 2, 3], tcp_offset_rpy_rad: [0.01, 0.02, 0.03], contact: {required: true, min_quality: 0.4}, aperture: {min_m: 0.01, max_m: 0.08}, lift: {axis: x_minus, distance_m: 0.15}}}\n"
    "place: {target: {asset_ref: target_bin_default, region_ref: default_drop_zone}, placement: {policy: EXACT, requested_local_pose: {xyz_m: [0.01, 0.02, 0.03], rpy_rad: [0.1, 0.2, 0.3]}, orientation: {mode: fixed, rpy_rad: [0.2, 0.3, 0.4], tolerance_rad: [0.01, 0.02, 0.03]}, approach: {axis: z_down, distance_m: 0.1}, clearance_m: 0.05, retreat: {axis: y_plus, distance_m: 0.1}}, release: {strategy: tool_release}}\n"
    "routing: {mode: direct}\nscene_package: scenes/ur5_2f_test\nsafety: {execution_mode: simulation_preview, require_fake_hardware: true, real_hardware_enabled: false, preview_policy: diagnostic_if_unresolved}\n");
  EXPECT_EQ(model.task_id, "t");
  EXPECT_EQ(model.pick_selection.zone_ref, "pick_zone_main");
  EXPECT_EQ(model.grasp.strategy_ref, "top_2f");
  EXPECT_EQ(model.place.asset_ref, "target_bin_default");
  EXPECT_EQ(model.release_strategy, "tool_release");
  EXPECT_EQ(model.scene_package, "scenes/ur5_2f_test"); EXPECT_FALSE(model.routing_yaml.empty());
  EXPECT_EQ(model.pick_selection.color, "red"); EXPECT_DOUBLE_EQ(model.pick_selection.min_confidence, 0.8); EXPECT_DOUBLE_EQ(model.pick_selection.max_age_seconds, 2.0);
  EXPECT_EQ(model.grasp.approach_axis, "z_down"); EXPECT_DOUBLE_EQ(model.grasp.approach_distance_m, 0.12); EXPECT_EQ(model.grasp.orientation_mode, "vertical");
  EXPECT_DOUBLE_EQ(model.grasp.tcp_offset_xyz_m[2], 3.0); EXPECT_DOUBLE_EQ(model.grasp.contact_min_quality, 0.4); EXPECT_DOUBLE_EQ(model.grasp.aperture_max_m, 0.08); EXPECT_EQ(model.grasp.lift_axis, "x_minus");
  EXPECT_TRUE(model.place.has_requested_local_pose); EXPECT_DOUBLE_EQ(model.place.requested_local_pose.rpy_rad[2], 0.3); EXPECT_EQ(model.place.orientation_mode, "fixed"); EXPECT_EQ(model.place.approach_axis, "z_down"); EXPECT_EQ(model.place.retreat_axis, "y_plus"); EXPECT_DOUBLE_EQ(model.place.clearance_m, 0.05);
}
