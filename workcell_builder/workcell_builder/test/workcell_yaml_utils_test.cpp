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
