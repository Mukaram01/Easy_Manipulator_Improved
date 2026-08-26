#include <gtest/gtest.h>

#include <boost/filesystem.hpp>
#include <algorithm>
#include <fstream>

#include "workcell_studio_scene_browser.hpp"

namespace fs = boost::filesystem;

TEST(WorkcellSceneIdentity, RefreshAcrossSymlinkPreservesPhysicalSceneAndAuthoredLayout)
{
  const fs::path workspace = fs::temp_directory_path() / fs::unique_path("workcell_identity_%%%%-%%%%");
  const fs::path canonical_root = workspace / "src/easy_manipulation_deployment/scenes";
  const fs::path canonical_scene = canonical_root / "test_scene";
  const fs::path alias_root = workspace / "src/scenes";
  fs::create_directories(canonical_scene / "layout");
  std::ofstream(canonical_scene / "environment.yaml") << "robot:\n  name: test_arm\n";
  const std::string layout = "layout:\n  items:\n    - id: authored_fixture\n";
  std::ofstream(canonical_scene / "layout/workcell_studio_layout.yaml") << layout;
  fs::create_directory_symlink(fs::path("easy_manipulation_deployment/scenes"), alias_root);

  const auto initial = workcell_builder::discover_workcell_studio_scenes(workspace);
  ASSERT_EQ(initial.scenes.size(), 1U);
  const fs::path selected = initial.scenes.front().canonical_scene_dir;

  // Model a refresh returning the supported alias spelling.
  workcell_builder::WorkcellStudioSceneBrowserResult refreshed;
  auto alias_scene = initial.scenes.front();
  alias_scene.scene_dir = alias_root / "test_scene";
  alias_scene.canonical_scene_dir = workcell_builder::canonical_scene_identity(alias_scene.scene_dir);
  refreshed.scenes.push_back(alias_scene);
  EXPECT_EQ(workcell_builder::find_scene_by_identity(refreshed, selected, "test_scene"), 0);
  EXPECT_EQ(alias_scene.canonical_scene_dir, selected);
  EXPECT_TRUE(fs::is_symlink(alias_root));
  EXPECT_EQ(workcell_builder::canonical_scene_identity(alias_root / "test_scene"), selected);
  EXPECT_EQ(refreshed.scenes.size(), 1U);

  std::ifstream layout_file(canonical_scene / "layout/workcell_studio_layout.yaml");
  const std::string preserved((std::istreambuf_iterator<char>(layout_file)), std::istreambuf_iterator<char>());
  EXPECT_EQ(preserved, layout);
  fs::remove_all(workspace);
}

TEST(WorkcellSceneIdentity, DeletedSceneDoesNotMatchAfterRefresh)
{
  workcell_builder::WorkcellStudioSceneBrowserResult refreshed;
  EXPECT_EQ(workcell_builder::find_scene_by_identity(refreshed, "/missing/test_scene", "test_scene"), -1);
}

TEST(WorkcellSceneIdentity, HomeMetadataUsesCanonicalSceneSnapshotSources)
{
  const fs::path workspace = fs::temp_directory_path() / fs::unique_path("workcell_home_metadata_%%%%-%%%%");
  const fs::path scene = workspace / "src/easy_manipulation_deployment/scenes/ur5_2f_test";
  fs::create_directories(scene / "config");
  std::ofstream(scene / "environment.yaml")
    << "scene:\n  name: UR5 2F Test\nrobot:\n  model: ur5\n";
  std::ofstream(scene / "cell_definition.yaml")
    << "end_effector:\n  id: robotiq_85_gripper\n";
  std::ofstream(scene / "config/task_recipe.yaml")
    << "task:\n  type: pick_place\n";

  const auto result = workcell_builder::discover_workcell_studio_scenes(workspace);
  ASSERT_EQ(result.scenes.size(), 1U);
  const auto & metadata = result.scenes.front();
  EXPECT_EQ(metadata.display_name, "UR5 2F Test");
  EXPECT_EQ(metadata.robot_summary, "ur5");
  EXPECT_EQ(metadata.gripper_summary, "robotiq_85_gripper");
  EXPECT_EQ(metadata.task_summary, "pick_place");
  fs::remove_all(workspace);
}

TEST(WorkcellSceneIdentity, GenericTaskConfigurationIsNotInventedAsPickPlace)
{
  const fs::path workspace = fs::temp_directory_path() / fs::unique_path("workcell_home_task_%%%%-%%%%");
  const fs::path scene = workspace / "src/easy_manipulation_deployment/scenes/generic_cell";
  fs::create_directories(scene / "config");
  std::ofstream(scene / "environment.yaml") << "robot:\n  model: demo_robot\n";
  std::ofstream(scene / "config/task_recipe.yaml") << "task:\n  id: configured_task\n";

  const auto result = workcell_builder::discover_workcell_studio_scenes(workspace);
  ASSERT_EQ(result.scenes.size(), 1U);
  EXPECT_EQ(result.scenes.front().task_summary, "Configured");
  fs::remove_all(workspace);
}

TEST(WorkcellSceneIdentity, HomeReadinessUsesCurrentCanonicalAcceptanceNotLegacySmokeExistence)
{
  const fs::path workspace = fs::temp_directory_path() / fs::unique_path("workcell_home_ready_%%%%-%%%%");
  const fs::path scene = workspace / "src/easy_manipulation_deployment/scenes/reference_cell";
  fs::create_directories(scene / "config");
  fs::create_directories(scene / "launch");
  fs::create_directories(scene / "urdf");
  fs::create_directories(scene / "acceptance");
  fs::create_directories(scene / "smoke");
  std::ofstream(scene / "package.xml") << "<package/>\n";
  std::ofstream(scene / "CMakeLists.txt") << "cmake_minimum_required(VERSION 3.8)\n";
  std::ofstream(scene / "environment.yaml") << "robot: {model: ur5}\n";
  std::ofstream(scene / "config/task_recipe.yaml") << "task: {type: pick_place}\n";
  std::ofstream(scene / "config/workcell_builder_task_intent.yaml") << "task: {type: pick_place}\n";
  std::ofstream(scene / "launch/demo.launch.py") << "use_fake_hardware\n";
  std::ofstream(scene / "urdf/scene.urdf.xacro") << "<robot/>\n";
  std::ofstream(scene / "urdf/arm_hand.srdf.xacro") << "<robot/>\n";
  std::ofstream(scene / "smoke/offline_smoke_report.json") << "{\"status\":\"PASS\"}\n";
  const fs::path acceptance = scene / "acceptance/generated_scene_acceptance.json";
  std::ofstream(acceptance) <<
    "{\"scene_name\":\"reference_cell\",\"status\":\"PASS\",\"safety_flags\":"
    "{\"fake_hardware_first\":true,\"runtime_execution_enabled\":false,\"motion_command_sent\":false}}\n";

  auto result = workcell_builder::discover_workcell_studio_scenes(workspace);
  ASSERT_EQ(result.scenes.size(), 1U);
  EXPECT_EQ(result.scenes.front().status, "READY");
  EXPECT_TRUE(result.scenes.front().fake_hardware_ready);

  fs::last_write_time(scene / "environment.yaml", fs::last_write_time(acceptance) + 2);
  result = workcell_builder::discover_workcell_studio_scenes(workspace);
  ASSERT_EQ(result.scenes.size(), 1U);
  EXPECT_EQ(result.scenes.front().status, "WARNINGS");
  EXPECT_FALSE(result.scenes.front().fake_hardware_ready);
  EXPECT_NE(std::find(result.scenes.front().readiness_reasons.begin(),
    result.scenes.front().readiness_reasons.end(), "Scene changed since validation; run validation again"),
    result.scenes.front().readiness_reasons.end());
  fs::remove_all(workspace);
}

TEST(WorkcellSceneIdentity, CanonicalScenesUseTheSameReadyContractAsHome)
{
  const fs::path repo_root(WORKCELL_BUILDER_REPO_ROOT);
  const auto result = workcell_builder::discover_workcell_studio_scenes(repo_root);
  for (const std::string scene_name : {"ur5_2f_test", "suction_test"}) {
    const auto found = std::find_if(result.scenes.begin(), result.scenes.end(),
      [&scene_name](const workcell_builder::WorkcellStudioSceneInfo & scene) {
        return scene.scene_name == scene_name;
      });
    ASSERT_NE(found, result.scenes.end()) << scene_name;
    EXPECT_EQ(found->status, "READY") << scene_name;
    EXPECT_TRUE(found->acceptance_report_current) << scene_name;
    EXPECT_TRUE(found->acceptance_report_passed) << scene_name;
    EXPECT_TRUE(found->fake_hardware_ready) << scene_name;
  }
}
