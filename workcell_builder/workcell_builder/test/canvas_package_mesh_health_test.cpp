#include <gtest/gtest.h>
#include <boost/filesystem.hpp>
#include <fstream>
#include <algorithm>
#include "workcell_studio_canvas_model.hpp"
#include "visual_mesh_source_resolver.hpp"

namespace fs = boost::filesystem;

TEST(CanvasPackageMeshHealth, CanonicalProductAssetsUseRendererPackageOwnership)
{
  const fs::path root = fs::temp_directory_path() / fs::unique_path("canvas-package-health-%%%%-%%%%");
  fs::create_directories(root / "layout");
  fs::create_directories(root / "config");
  std::ofstream(root.string() + "/environment.yaml") << "robot: ur5\n";
  std::ofstream(root.string() + "/scene_manifest.yaml") << "template_name: demo\n";
  std::ofstream(root.string() + "/config/task_recipe.yaml") << "pick_source: pick\nplace_target: place\n";
  const std::vector<std::pair<std::string, std::string>> meshes = {
    {"table", "package://workbench_description/meshes/visual/table.stl"},
    {"bin", "package://sorting_bin_description/meshes/sorting_bin.stl"},
    {"camera", "package://realsense2_description/meshes/d435.dae"},
    {"broken", "package://workbench_description/meshes/visual/absent.stl"}};
  {
    std::ofstream layout(root.string() + "/layout/workcell_studio_layout.yaml");
    layout << "schema_version: workcell_studio_layout/v1\nitems:\n";
    for (const auto & entry : meshes) {
      layout << "  - id: " << entry.first << "\n    type: fixture\n    editable: true\n"
             << "    geometry_type: mesh\n    mesh: {path: '" << entry.second << "'}\n"
             << "    pose: {xyz: [0, 0, 0], rpy: [0, 0, 0]}\n";
    }
  }
  const auto model = workcell_builder::build_workcell_studio_canvas_model(root, "demo");
  for (const auto & item : model.items) {
    EXPECT_NE(item.id, "conveyor");
    EXPECT_NE(item.id, "object_a");
  }
  for (const auto & entry : meshes) {
    const auto item = std::find_if(model.items.begin(), model.items.end(), [&](const auto & candidate) {
      return candidate.id == entry.first;
    });
    ASSERT_NE(item, model.items.end()) << entry.first;
    if (entry.first == "broken") {
      EXPECT_FALSE(item->mesh_available);
      EXPECT_FALSE(item->mesh_load_warning.empty());
    } else {
      const auto expected = workcell_builder::resolve_visual_mesh_source_path(
        QString(), QString::fromStdString(entry.second), root, QString());
      ASSERT_FALSE(expected.isEmpty()) << entry.second;
      EXPECT_TRUE(item->mesh_available) << entry.first;
      EXPECT_EQ(item->mesh_path, expected.toStdString());
      EXPECT_TRUE(item->mesh_load_warning.empty()) << item->mesh_load_warning;
    }
  }
  fs::remove_all(root);
}
