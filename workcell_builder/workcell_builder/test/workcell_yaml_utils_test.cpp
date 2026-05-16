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

