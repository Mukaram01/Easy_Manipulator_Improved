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
    "pick: {selection: {source_ref: detected_objects/v1, source_type: perception, zone_ref: pick_zone_main, object_filter: {class_id: bottle}}, grasp: {policy: EXACT, required_capability: two_finger_parallel, strategy_ref: top_2f}}\n"
    "place: {target: {asset_ref: target_bin_default, region_ref: default_drop_zone}, placement: {policy: EXACT}, release: {strategy: tool_release}}\n");
  EXPECT_EQ(model.task_id, "t");
  EXPECT_EQ(model.pick_selection.zone_ref, "pick_zone_main");
  EXPECT_EQ(model.grasp.strategy_ref, "top_2f");
  EXPECT_EQ(model.place.asset_ref, "target_bin_default");
  EXPECT_EQ(model.release_strategy, "tool_release");
}
