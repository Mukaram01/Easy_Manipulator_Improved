#include <gtest/gtest.h>
#include <boost/filesystem.hpp>
#include <fstream>
#include <set>

#include "workcell_studio_canvas_model.hpp"

namespace fs = boost::filesystem;

static void write_file(const fs::path & path, const std::string & text)
{
  fs::create_directories(path.parent_path());
  std::ofstream out(path.string());
  out << text;
}

static void copy_directory_tree(const fs::path & source, const fs::path & destination)
{
  fs::create_directories(destination);
  for (fs::recursive_directory_iterator it(source), end; it != end; ++it) {
    const fs::path relative = fs::relative(it->path(), source);
    const fs::path target = destination / relative;
    if (fs::is_directory(it->path())) {
      fs::create_directories(target);
    } else if (fs::is_regular_file(it->path())) {
      fs::create_directories(target.parent_path());
      fs::copy_file(it->path(), target, fs::copy_option::overwrite_if_exists);
    }
  }
}

static YAML::Node load_yaml_file(const fs::path & path)
{
  return YAML::LoadFile(path.string());
}

static void write_yaml_file(const fs::path & path, const YAML::Node & node)
{
  fs::create_directories(path.parent_path());
  std::ofstream out(path.string());
  out << node;
  out << "\n";
}

static std::set<std::string> editable_item_ids(const YAML::Node & layout)
{
  std::set<std::string> ids;
  const YAML::Node items = layout["items"];
  if (!items || !items.IsSequence()) return ids;
  for (const auto & item : items) {
    if (!item || !item.IsMap() || !item["id"]) continue;
    const bool editable = item["editable"] ? item["editable"].as<bool>(false) : false;
    if (editable) ids.insert(item["id"].as<std::string>());
  }
  return ids;
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


TEST(WorkcellStudioCanvasMesh, StarterLayoutAcceptanceCopiesSceneAndFiltersUnsafePreviewItems)
{
#ifndef WORKCELL_BUILDER_REPO_ROOT
  GTEST_SKIP() << "WORKCELL_BUILDER_REPO_ROOT is not configured for scene acceptance coverage";
#else
  const fs::path repo_root = fs::path(WORKCELL_BUILDER_REPO_ROOT);
  fs::path source_scene = repo_root / "scenes" / "ur5_2f_test";
  std::string scene_name = "ur5_2f_test";
  if (!fs::exists(source_scene)) {
    source_scene = repo_root / "scenes" / "suction_test";
    scene_name = "suction_test";
  }
  ASSERT_TRUE(fs::exists(source_scene)) << "expected ur5_2f_test or suction_test scene fixture";

  const fs::path temp_root = fs::temp_directory_path() / fs::unique_path("wc_starter_layout_acceptance_%%%%-%%%%-%%%%");
  const fs::path copied_scene = temp_root / scene_name;
  copy_directory_tree(source_scene, copied_scene);

  const fs::path editable_layout_path = copied_scene / "layout" / "workcell_studio_layout.yaml";
  ASSERT_TRUE(fs::exists(editable_layout_path));
  const YAML::Node before_layout = load_yaml_file(editable_layout_path);
  ASSERT_TRUE(before_layout && before_layout.IsMap());
  ASSERT_EQ(before_layout["schema_version"].as<std::string>(), "workcell_studio_layout/v1");
  ASSERT_TRUE(before_layout["items"] && before_layout["items"].IsSequence());

  const auto valid_model = workcell_builder::build_workcell_studio_canvas_model(copied_scene, scene_name);
  std::set<std::string> safe_unlocked_preview_ids;
  std::set<std::string> locked_preview_ids;
  for (const auto & item : valid_model.items) {
    if (item.locked) {
      locked_preview_ids.insert(item.id);
      continue;
    }
    if (item.provenance != workcell_builder::WorkcellStudioItemProvenance::StaticFallbackPreview) {
      safe_unlocked_preview_ids.insert(item.id);
    }
  }
  ASSERT_FALSE(safe_unlocked_preview_ids.empty()) << "scene should expose at least one safe/unlocked preview item";
  ASSERT_FALSE(locked_preview_ids.empty()) << "scene should expose locked generated preview items";

  const YAML::Node starter_layout = workcell_builder::build_starter_layout_entries_from_preview(valid_model);
  write_yaml_file(editable_layout_path, starter_layout);

  const YAML::Node after_layout = load_yaml_file(editable_layout_path);
  ASSERT_TRUE(after_layout && after_layout.IsMap());
  ASSERT_EQ(after_layout["schema_version"].as<std::string>(), "workcell_studio_layout/v1");
  ASSERT_EQ(after_layout["scene_name"].as<std::string>(), scene_name);
  ASSERT_TRUE(after_layout["items"] && after_layout["items"].IsSequence());

  const std::set<std::string> after_editable_ids = editable_item_ids(after_layout);
  if (safe_unlocked_preview_ids.count("table") > 0) {
    EXPECT_EQ(after_editable_ids.count("table"), 1u)
      << "starter layout should include the expected safe/unlocked table preview item as editable";
  } else {
    bool wrote_expected_safe_editable = false;
    for (const auto & id : safe_unlocked_preview_ids) {
      if (after_editable_ids.count(id) > 0) {
        wrote_expected_safe_editable = true;
        break;
      }
    }
    EXPECT_TRUE(wrote_expected_safe_editable) << "starter layout should include an editable safe/unlocked preview item";
  }
  for (const auto & id : locked_preview_ids) {
    EXPECT_EQ(after_editable_ids.count(id), 0u) << "locked generated preview item was written editable: " << id;
  }

  const fs::path fallback_scene = temp_root / (scene_name + "_fallback_only");
  copy_directory_tree(source_scene, fallback_scene);
  const fs::path fallback_layout_path = fallback_scene / "layout" / "workcell_studio_layout.yaml";
  ASSERT_TRUE(fs::exists(fallback_layout_path));
  const YAML::Node fallback_before_layout = load_yaml_file(fallback_layout_path);
  ASSERT_TRUE(fallback_before_layout && fallback_before_layout.IsMap());
  fs::remove(fallback_layout_path);

  const auto fallback_model = workcell_builder::build_workcell_studio_canvas_model(fallback_scene, scene_name);
  std::set<std::string> static_fallback_ids;
  for (const auto & item : fallback_model.items) {
    if (item.provenance == workcell_builder::WorkcellStudioItemProvenance::StaticFallbackPreview) {
      static_fallback_ids.insert(item.id);
    }
  }
  ASSERT_FALSE(static_fallback_ids.empty()) << "missing layout should force static fallback-only preview metadata";
  const YAML::Node fallback_starter_layout = workcell_builder::build_starter_layout_entries_from_preview(fallback_model);
  write_yaml_file(fallback_layout_path, fallback_starter_layout);
  const YAML::Node fallback_after_layout = load_yaml_file(fallback_layout_path);
  const std::set<std::string> fallback_after_editable_ids = editable_item_ids(fallback_after_layout);
  for (const auto & id : static_fallback_ids) {
    EXPECT_EQ(fallback_after_editable_ids.count(id), 0u) << "static fallback-only preview item was written editable: " << id;
  }

  fs::remove_all(temp_root);
#endif
}
