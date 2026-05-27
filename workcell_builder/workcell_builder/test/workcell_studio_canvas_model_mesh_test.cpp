#include <gtest/gtest.h>
#include <boost/filesystem.hpp>
#include <fstream>

#include "workcell_studio_canvas_model.hpp"

namespace fs = boost::filesystem;

static void write_file(const fs::path & path, const std::string & text)
{
  fs::create_directories(path.parent_path());
  std::ofstream out(path.string());
  out << text;
}

TEST(WorkcellStudioCanvasMesh, ResolvesAbsoluteAndPackagePaths)
{
  const fs::path root = fs::temp_directory_path() / "wc_canvas_mesh_abs_pkg";
  fs::remove_all(root);
  fs::create_directories(root);

  const fs::path visual = root / "assets" / "table_description" / "meshes" / "visual" / "table.stl";
  write_file(visual, "solid x\nendsolid x\n");

  write_file(root / "environment.yaml", "mesh_path: package://table_description/meshes/visual/table.stl\n");
  write_file(root / "scene_manifest.yaml", "template_name: demo\n");
  write_file(root / "config" / "task_recipe.yaml", "pick_source: a\nplace_target: b\n");
  write_file(root / "layout" / "workcell_studio_layout.yaml", "schema_version: workcell_studio_layout/v1\nitems: []\n");

  const auto model = workcell_builder::build_workcell_studio_canvas_model(root, "demo");
  bool found = false;
  for (const auto & item : model.items) {
    if (item.id == "table") {
      found = true;
      EXPECT_TRUE(item.mesh_available);
      EXPECT_EQ(item.mesh_path, visual.generic_string());
      EXPECT_TRUE(item.mesh_load_warning.empty());
    }
  }
  EXPECT_TRUE(found);
}

TEST(WorkcellStudioCanvasMesh, FallsBackToCollisionAndWarns)
{
  const fs::path root = fs::temp_directory_path() / "wc_canvas_mesh_collision";
  fs::remove_all(root);
  fs::create_directories(root);

  const fs::path collision = root / "assets" / "environment_objects" / "object_a_description" / "meshes" / "collision" / "object_a.stl";
  write_file(collision, "solid x\nendsolid x\n");

  write_file(root / "environment.yaml", "mesh_path: missing_relative.stl\n");
  write_file(root / "scene_manifest.yaml", "template_name: demo\n");
  write_file(root / "config" / "task_recipe.yaml", "pick_source: a\nplace_target: b\n");
  write_file(root / "layout" / "workcell_studio_layout.yaml", "schema_version: workcell_studio_layout/v1\nitems: []\n");

  const auto model = workcell_builder::build_workcell_studio_canvas_model(root, "demo");
  bool found = false;
  for (const auto & item : model.items) {
    if (item.id == "object_a") {
      found = true;
      EXPECT_TRUE(item.mesh_available);
      EXPECT_EQ(item.mesh_path, collision.generic_string());
      EXPECT_NE(item.mesh_load_warning.find("Mesh preview fallback for object_a"), std::string::npos);
    }
  }
  EXPECT_TRUE(found);
}

TEST(WorkcellStudioCanvasMesh, MissingFileProducesDeterministicWarning)
{
  const fs::path root = fs::temp_directory_path() / "wc_canvas_mesh_missing";
  fs::remove_all(root);
  fs::create_directories(root);

  write_file(root / "environment.yaml", "mesh_path: missing.stl\n");
  write_file(root / "scene_manifest.yaml", "template_name: demo\n");
  write_file(root / "config" / "task_recipe.yaml", "pick_source: a\nplace_target: b\n");
  write_file(root / "layout" / "workcell_studio_layout.yaml", "schema_version: workcell_studio_layout/v1\nitems: []\n");

  const auto model = workcell_builder::build_workcell_studio_canvas_model(root, "demo");
  bool found = false;
  for (const auto & item : model.items) {
    if (item.id == "table") {
      found = true;
      EXPECT_FALSE(item.mesh_available);
      EXPECT_EQ(item.mesh_load_warning, "mesh metadata missing or legacy; using primitive preview");
    }
  }
  EXPECT_TRUE(found);
}

TEST(WorkcellStudioCanvasMesh, LegacyMeshShapesNeverThrowAndFallbackSafely)
{
  const std::vector<std::string> mesh_variants = {
    "", "mesh: none\n", "mesh: false\n", "mesh: package://x/y.stl\n",
    "mesh: {}\n", "mesh:\n", "mesh:\n  path: package://x/y.stl\n  scale: [1, 1, 1]\n"
  };

  for (std::size_t i = 0; i < mesh_variants.size(); ++i) {
    const fs::path root = fs::temp_directory_path() / ("wc_canvas_mesh_legacy_" + std::to_string(i));
    fs::remove_all(root);
    fs::create_directories(root);
    write_file(root / "environment.yaml", "robot: ur5\n");
    write_file(root / "scene_manifest.yaml", "template_name: demo\n");
    write_file(root / "config" / "task_recipe.yaml", "pick_source: a\nplace_target: b\n");
    write_file(root / "layout" / "workcell_studio_layout.yaml",
      "schema_version: workcell_studio_layout/v1\nitems:\n- id: table\n  type: table\n  role: table\n  pose:\n    xyz: [0,0,0]\n    rpy: [0,0,0]\n  " + mesh_variants[i]);

    EXPECT_NO_THROW({
      const auto model = workcell_builder::build_workcell_studio_canvas_model(root, "demo");
      bool found = false;
      for (const auto & item : model.items) {
        if (item.id != "table") continue;
        found = true;
        EXPECT_FALSE(item.mesh_available);
        EXPECT_TRUE(item.mesh_path.empty());
        EXPECT_EQ(item.mesh_load_warning, "mesh metadata missing or legacy; using primitive preview");
      }
      EXPECT_TRUE(found);
    });
  }
}

TEST(WorkcellStudioCanvasMesh, CountEditableLayoutEntriesTreatsEmptyAsZero)
{
  const fs::path root = fs::temp_directory_path() / "wc_layout_count_empty";
  fs::remove_all(root);
  write_file(root / "layout" / "workcell_studio_layout.yaml", "schema_version: workcell_studio_layout/v1\nitems: []\n");
  EXPECT_EQ(workcell_builder::count_editable_layout_entries(root), 0u);
}

TEST(WorkcellStudioCanvasMesh, CountEditableLayoutEntriesWithOneEditable)
{
  const fs::path root = fs::temp_directory_path() / "wc_layout_count_one";
  fs::remove_all(root);
  write_file(root / "layout" / "workcell_studio_layout.yaml", "schema_version: workcell_studio_layout/v1\nitems:\n- id: a\n  editable: true\n");
  EXPECT_EQ(workcell_builder::count_editable_layout_entries(root), 1u);
}

TEST(WorkcellStudioCanvasMesh, BuildStarterLayoutFiltersLockedAndFallbackPreviewItems)
{
  workcell_builder::WorkcellStudioCanvasModel model;
  model.scene_name = "demo";
  workcell_builder::WorkcellStudioCanvasItem safe;
  safe.id = "safe_item";
  safe.type = "object";
  safe.provenance = workcell_builder::WorkcellStudioItemProvenance::GeneratedOrLegacyPreview;
  safe.locked = false;
  model.items.push_back(safe);

  workcell_builder::WorkcellStudioCanvasItem locked = safe;
  locked.id = "locked_item";
  locked.locked = true;
  model.items.push_back(locked);

  workcell_builder::WorkcellStudioCanvasItem fallback = safe;
  fallback.id = "fallback_item";
  fallback.provenance = workcell_builder::WorkcellStudioItemProvenance::StaticFallbackPreview;
  model.items.push_back(fallback);

  const YAML::Node layout = workcell_builder::build_starter_layout_entries_from_preview(model);
  const YAML::Node items = layout["items"];
  ASSERT_TRUE(items && items.IsSequence());
  ASSERT_EQ(items.size(), 1u);
  EXPECT_EQ(items[0]["id"].as<std::string>(), "safe_item");
}
