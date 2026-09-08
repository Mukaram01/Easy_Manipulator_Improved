#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include "workcell_yaml_utils.hpp"

TEST(WorkcellYamlUtils, ScalarRobotValue)
{
  const YAML::Node root = YAML::Load("robot: ur5");
  EXPECT_EQ(workcell_builder::yaml_named_or_scalar(root["robot"], "name"), "ur5");
}

TEST(WorkcellYamlUtils, MapRobotValueWithName)
{
  const YAML::Node root = YAML::Load("robot:\n  name: ur5");
  EXPECT_EQ(workcell_builder::yaml_named_or_scalar(root["robot"], "name"), "ur5");
}

TEST(WorkcellYamlUtils, MissingRobotReturnsEmpty)
{
  const YAML::Node root = YAML::Load("scene: demo");
  EXPECT_TRUE(workcell_builder::yaml_named_or_scalar(root["robot"], "name").empty());
}

TEST(WorkcellYamlUtils, LegacyAndNewObjectShapes)
{
  const YAML::Node scalar_obj = YAML::Load("- bin_a")[0];
  const YAML::Node map_obj = YAML::Load("- name: bin_b")[0];
  EXPECT_EQ(workcell_builder::yaml_name_from_node(scalar_obj), "bin_a");
  EXPECT_EQ(workcell_builder::yaml_name_from_node(map_obj), "bin_b");
}

TEST(WorkcellYamlUtils, ScalarPerceptionNodeDoesNotExposeNestedMapKeys)
{
  const YAML::Node root = YAML::Load("perception: disabled");
  const YAML::Node perception_map = workcell_builder::get_map(root, "perception");
  EXPECT_FALSE(perception_map.IsDefined());
  EXPECT_FALSE(workcell_builder::get_scalar(root["perception"], "mode").IsDefined());
}

TEST(WorkcellYamlUtils, BoolLikeHandlesLegacyDisabledTokens)
{
  const YAML::Node disabled = YAML::Load("perception: disabled");
  const YAML::Node false_scalar = YAML::Load("perception: false");
  const YAML::Node none_scalar = YAML::Load("perception: none");
  const YAML::Node empty_map = YAML::Load("perception: {}");
  const YAML::Node missing = YAML::Load("task: pick_place");

  EXPECT_EQ(workcell_builder::bool_like(disabled["perception"]), std::optional<bool>(false));
  EXPECT_EQ(workcell_builder::bool_like(false_scalar["perception"]), std::optional<bool>(false));
  EXPECT_EQ(workcell_builder::bool_like(none_scalar["perception"]), std::nullopt);
  EXPECT_EQ(workcell_builder::get_map(empty_map, "perception").IsMap(), true);
  EXPECT_FALSE(workcell_builder::get_map(missing, "perception").IsDefined());
}

TEST(WorkcellYamlUtils, PerceptionContractSummaryHandlesScalarAndMissing)
{
  const YAML::Node disabled = YAML::Load("task:\n  perception: disabled");
  const YAML::Node false_scalar = YAML::Load("task:\n  perception: false");
  const YAML::Node none_scalar = YAML::Load("task:\n  perception: none");
  const YAML::Node empty_map = YAML::Load("task:\n  perception: {}");
  const YAML::Node missing = YAML::Load("task:\n  type: pick_place");

  const auto a = workcell_builder::parse_perception_contract_summary(disabled);
  const auto b = workcell_builder::parse_perception_contract_summary(false_scalar);
  const auto c = workcell_builder::parse_perception_contract_summary(none_scalar);
  const auto d = workcell_builder::parse_perception_contract_summary(empty_map);
  const auto e = workcell_builder::parse_perception_contract_summary(missing);

  EXPECT_FALSE(a.enabled); EXPECT_EQ(a.mode, "legacy_disabled"); EXPECT_TRUE(a.warning.empty());
  EXPECT_FALSE(b.enabled); EXPECT_EQ(b.mode, "legacy_disabled"); EXPECT_TRUE(b.warning.empty());
  EXPECT_FALSE(c.enabled); EXPECT_EQ(c.mode, "legacy_disabled"); EXPECT_TRUE(c.warning.empty());
  EXPECT_FALSE(d.enabled); EXPECT_EQ(d.mode, "legacy_disabled"); EXPECT_TRUE(d.warning.empty());
  EXPECT_FALSE(e.enabled); EXPECT_EQ(e.mode, "legacy_disabled"); EXPECT_TRUE(e.warning.empty());
}

TEST(WorkcellYamlUtils, NestedV1TaskReadViewPreservesAuthoredDocument)
{
  const YAML::Node root = YAML::Load(R"(
schema: workcell_builder_task_intent/v1
task: {type: pick_place}
pick:
  source: {type: perception, id: detected_objects/v1}
  zone: {id: pick_zone_main}
  object_filter: {class_id: bottle}
place: {target: {id: default_drop_zone}}
grasp: {strategy_ref: top_2f}
perception: {camera: {id: realsense_overhead}}
)");
  const auto before = YAML::Dump(root);
  for (int i = 0; i < 3; ++i) {
    const auto task = workcell_builder::task_intent_view(root);
    EXPECT_EQ(task["type"].as<std::string>(), "pick_place");
    EXPECT_EQ(task["pick"]["source"]["id"].as<std::string>(), "detected_objects/v1");
    EXPECT_EQ(task["pick"]["zone"]["id"].as<std::string>(), "pick_zone_main");
    EXPECT_EQ(task["pick"]["object_filter"]["class_id"].as<std::string>(), "bottle");
    EXPECT_EQ(task["place"]["target"]["id"].as<std::string>(), "default_drop_zone");
    EXPECT_EQ(task["grasp"]["strategy_ref"].as<std::string>(), "top_2f");
    EXPECT_EQ(task["perception"]["camera"]["id"].as<std::string>(), "realsense_overhead");
    EXPECT_TRUE(workcell_builder::parse_perception_contract_summary(root).enabled);
    EXPECT_EQ(YAML::Dump(root), before);
  }
}
