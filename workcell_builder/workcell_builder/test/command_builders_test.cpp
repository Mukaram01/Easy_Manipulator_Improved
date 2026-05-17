#include <gtest/gtest.h>

#include "include/workcell_builder_command_builders.hpp"

TEST(CommandBuildersTest, task_intent_refuses_missing_arguments)
{
  const auto missing_script = workcell_builder::build_task_intent_command_plan("", "/tmp/scene");
  EXPECT_FALSE(missing_script.ready());
  EXPECT_NE(missing_script.missing_fields.indexOf("helper script path"), -1);

  const auto missing_scene = workcell_builder::build_task_intent_command_plan("/tmp/script.py", "");
  EXPECT_FALSE(missing_scene.ready());
  EXPECT_NE(missing_scene.missing_fields.indexOf("scene directory"), -1);
}

TEST(CommandBuildersTest, generate_workcell_refuses_missing_arguments)
{
  const auto plan = workcell_builder::build_generate_workcell_command_plan("/tmp/script.py", "", "", "");
  EXPECT_FALSE(plan.ready());
  EXPECT_NE(plan.missing_fields.indexOf("scene directory"), -1);
  EXPECT_NE(plan.missing_fields.indexOf("output directory"), -1);
  EXPECT_NE(plan.missing_fields.indexOf("scene name"), -1);
}

TEST(CommandBuildersTest, command_text_contains_expected_flags)
{
  const auto plan = workcell_builder::build_generate_workcell_command_plan(
    "/tmp/generate_workcell_from_cell_definition.py", "/tmp/scene", "/tmp/scenes", "scene_a");
  ASSERT_TRUE(plan.ready());
  const auto cmd = plan.display_command();
  EXPECT_NE(cmd.indexOf("cell_definition.yaml"), -1);
  EXPECT_NE(cmd.indexOf("--output-dir"), -1);
  EXPECT_NE(cmd.indexOf("--package-name"), -1);
  EXPECT_NE(cmd.indexOf("scene_a"), -1);
  EXPECT_NE(cmd.indexOf("--force"), -1);
  EXPECT_EQ(cmd.indexOf("--scene-dir"), -1);
  EXPECT_EQ(cmd.indexOf("--scene-name"), -1);
}

TEST(CommandBuildersTest, validate_generated_scene_uses_json_and_positional_scene_path)
{
  const auto plan = workcell_builder::build_validate_generated_scene_command_plan(
    "/tmp/validate_builder_generated_scene.py", "/tmp/scene");
  ASSERT_TRUE(plan.ready());
  const auto cmd = plan.display_command();
  EXPECT_NE(cmd.indexOf("validate_builder_generated_scene.py"), -1);
  EXPECT_NE(cmd.indexOf("--json"), -1);
  EXPECT_NE(cmd.indexOf("/tmp/scene"), -1);
  EXPECT_EQ(cmd.indexOf("--scene-dir"), -1);
  EXPECT_EQ(cmd.indexOf("--scene-name"), -1);
}
