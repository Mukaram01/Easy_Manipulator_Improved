#include <gtest/gtest.h>

#include <boost/filesystem.hpp>
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
