#include <gtest/gtest.h>
#include <boost/filesystem.hpp>
#include <algorithm>
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

TEST(WorkcellStudioCanvasMesh, BuildStarterLayoutSummarizesSkipsAndEditableItems)
{
  workcell_builder::WorkcellStudioCanvasModel model;
  model.scene_name = "demo";
  workcell_builder::WorkcellStudioCanvasItem safe;
  safe.id = "safe_item";
  safe.type = "object";
  safe.source_file = "layout/workcell_studio_layout.yaml";
  safe.mesh_path = "meshes/visual/safe_item.stl";
  safe.has_mesh_metadata = true;
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

  workcell_builder::WorkcellStudioCanvasItem missing_metadata = safe;
  missing_metadata.id = "missing_metadata_item";
  missing_metadata.has_mesh_metadata = false;
  model.items.push_back(missing_metadata);

  workcell_builder::WorkcellStudioCanvasItem unsafe_warning = safe;
  unsafe_warning.id = "unsafe_warning_item";
  unsafe_warning.mesh_load_warning = "mesh metadata missing or legacy; using primitive preview";
  model.items.push_back(unsafe_warning);

  workcell_builder::WorkcellStudioCanvasItem warning_metadata = safe;
  warning_metadata.id = "warning_metadata_item";
  warning_metadata.warnings.push_back("malformed metadata warning");
  model.items.push_back(warning_metadata);

  const auto summary = workcell_builder::build_starter_layout_entries_from_preview(model);
  EXPECT_EQ(summary.total_preview_items, 6u);
  EXPECT_EQ(summary.skipped_locked_items, 1u);
  EXPECT_EQ(summary.skipped_static_fallback_items, 1u);
  EXPECT_EQ(summary.skipped_unsafe_or_missing_metadata_items, 3u);
  EXPECT_EQ(summary.editable_items_created, 1u);

  EXPECT_FALSE(summary.layout["empty_layout_marker"].as<bool>());
  const YAML::Node items = summary.layout["items"];
  ASSERT_TRUE(items && items.IsSequence());
  ASSERT_EQ(items.size(), 1u);
  EXPECT_EQ(items[0]["id"].as<std::string>(), "safe_item");
  EXPECT_EQ(items[0]["mesh"]["path"].as<std::string>(), "meshes/visual/safe_item.stl");
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
  std::set<std::string> locked_preview_ids;
  for (const auto & item : valid_model.items) {
    if (item.locked) locked_preview_ids.insert(item.id);
  }
  ASSERT_FALSE(locked_preview_ids.empty()) << "scene should expose locked generated preview items";

  const auto starter_layout_summary = workcell_builder::build_starter_layout_entries_from_preview(valid_model);
  EXPECT_EQ(starter_layout_summary.total_preview_items, valid_model.items.size());
  EXPECT_GT(starter_layout_summary.skipped_unsafe_or_missing_metadata_items, 0u)
    << "legacy preview items without explicit safe mesh metadata should be skipped";
  write_yaml_file(editable_layout_path, starter_layout_summary.layout);

  const YAML::Node after_layout = load_yaml_file(editable_layout_path);
  ASSERT_TRUE(after_layout && after_layout.IsMap());
  ASSERT_EQ(after_layout["schema_version"].as<std::string>(), "workcell_studio_layout/v1");
  ASSERT_EQ(after_layout["scene_name"].as<std::string>(), scene_name);
  ASSERT_TRUE(after_layout["items"] && after_layout["items"].IsSequence());

  const std::set<std::string> after_editable_ids = editable_item_ids(after_layout);
  EXPECT_EQ(after_editable_ids.size(), starter_layout_summary.editable_items_created);
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
  const auto fallback_starter_layout_summary = workcell_builder::build_starter_layout_entries_from_preview(fallback_model);
  EXPECT_EQ(fallback_starter_layout_summary.editable_items_created, 0u);
  EXPECT_TRUE(fallback_starter_layout_summary.layout["empty_layout_marker"].as<bool>());
  write_yaml_file(fallback_layout_path, fallback_starter_layout_summary.layout);
  const YAML::Node fallback_after_layout = load_yaml_file(fallback_layout_path);
  const std::set<std::string> fallback_after_editable_ids = editable_item_ids(fallback_after_layout);
  for (const auto & id : static_fallback_ids) {
    EXPECT_EQ(fallback_after_editable_ids.count(id), 0u) << "static fallback-only preview item was written editable: " << id;
  }

  fs::remove_all(temp_root);
#endif
}

TEST(WorkcellStudioCanvasMesh, SaveLayoutReadinessRejectsEmptyEditableLayout)
{
  const fs::path root = fs::temp_directory_path() / "wc_save_layout_empty_ready";
  fs::remove_all(root);
  fs::create_directories(root);

  write_file(root / "layout" / "workcell_studio_layout.yaml",
    "schema_version: workcell_studio_layout/v1\nitems: []\n");
  write_file(root / "environment_layout.yaml", "schema_version: environment_layout/v1\nitems: []\n");
  write_file(root / "environment.yaml", "environment: {}\n");

  const auto inspection = workcell_builder::inspect_editable_layout_entries(root);
  EXPECT_TRUE(inspection.exists);
  EXPECT_TRUE(inspection.valid);
  EXPECT_TRUE(inspection.has_items_sequence);
  EXPECT_EQ(inspection.total_item_entries, 0u);
  EXPECT_EQ(inspection.editable_item_count, 0u);
  EXPECT_FALSE(workcell_builder::is_save_layout_workflow_ready(root));
}

TEST(WorkcellStudioCanvasMesh, SaveLayoutReadinessRejectsLockedOnlyAndFallbackOnlyLayout)
{
  const fs::path root = fs::temp_directory_path() / "wc_save_layout_locked_ready";
  fs::remove_all(root);
  fs::create_directories(root);

  write_file(root / "layout" / "workcell_studio_layout.yaml",
    "schema_version: workcell_studio_layout/v1\n"
    "items:\n"
    "  - id: locked_table\n"
    "    editable: true\n"
    "    locked: true\n"
    "  - id: explicitly_not_editable\n"
    "    editable: false\n"
    "  - id: fallback_table\n"
    "    source_layer: primitive_fallback\n");
  write_file(root / "environment_layout.yaml", "schema_version: environment_layout/v1\nitems: []\n");
  write_file(root / "environment.yaml", "environment: {}\n");

  const auto inspection = workcell_builder::inspect_editable_layout_entries(root);
  EXPECT_EQ(inspection.total_item_entries, 3u);
  EXPECT_EQ(inspection.editable_item_count, 0u);
  EXPECT_FALSE(workcell_builder::is_save_layout_workflow_ready(root));
}

TEST(WorkcellStudioCanvasMesh, SaveLayoutReadinessRejectsEditableLayoutWithMissingEnvironmentFiles)
{
  const fs::path root = fs::temp_directory_path() / "wc_save_layout_missing_env_ready";
  fs::remove_all(root);
  fs::create_directories(root);

  write_file(root / "layout" / "workcell_studio_layout.yaml",
    "schema_version: workcell_studio_layout/v1\n"
    "items:\n"
    "  - id: editable_table\n"
    "    editable: true\n");

  const auto inspection = workcell_builder::inspect_editable_layout_entries(root);
  EXPECT_EQ(inspection.editable_item_count, 1u);
  EXPECT_FALSE(workcell_builder::is_save_layout_workflow_ready(root));
}

TEST(WorkcellStudioCanvasMesh, SaveLayoutReadinessAcceptsEditableLayoutWithRequiredEnvironmentFiles)
{
  const fs::path root = fs::temp_directory_path() / "wc_save_layout_complete_ready";
  fs::remove_all(root);
  fs::create_directories(root);

  write_file(root / "layout" / "workcell_studio_layout.yaml",
    "schema_version: workcell_studio_layout/v1\n"
    "items:\n"
    "  - id: legacy_absent_flags_table\n"
    "  - id: legacy_unlocked_table\n"
    "    locked: false\n"
    "  - id: explicit_editable_zone\n"
    "    editable: true\n");
  write_file(root / "environment_layout.yaml", "schema_version: environment_layout/v1\nitems: []\n");
  write_file(root / "environment.yaml", "environment: {}\n");

  const auto inspection = workcell_builder::inspect_editable_layout_entries(root);
  EXPECT_TRUE(inspection.valid);
  EXPECT_EQ(inspection.total_item_entries, 3u);
  EXPECT_EQ(inspection.editable_item_count, 3u);
  EXPECT_TRUE(workcell_builder::is_save_layout_workflow_ready(root));
}

TEST(WorkcellStudioCanvasMesh, BootstrapEditableLayoutUsesSceneSourcePriority)
{
  const fs::path root = fs::temp_directory_path() / "wc_bootstrap_priority";
  fs::remove_all(root);
  fs::create_directories(root);

  write_file(root / "layout" / "workcell_studio_layout.yaml",
    "schema_version: workcell_studio_layout/v1\n"
    "items:\n"
    "  - id: existing_editable\n"
    "    type: table\n"
    "    editable: true\n"
    "    locked: false\n"
    "  - id: existing_locked\n"
    "    type: table\n"
    "    editable: true\n"
    "    locked: true\n");
  write_file(root / "environment_layout.yaml",
    "schema_version: environment_layout/v1\n"
    "assets:\n"
    "  - id: env_layout_asset\n"
    "    type: bin\n"
    "    pose: {xyz: [1, 2, 3], rpy: [0, 0, 0]}\n");
  write_file(root / "environment.yaml",
    "placed_objects:\n"
    "  - id: env_object\n"
    "    type: box\n");
  write_file(root / "cell_definition.yaml",
    "assets:\n"
    "  - id: cell_asset\n"
    "    type: fixture\n");

  workcell_builder::WorkcellStudioCanvasModel preview;
  preview.scene_name = "wc_bootstrap_priority";
  workcell_builder::WorkcellStudioCanvasItem preview_item;
  preview_item.id = "preview_safe";
  preview_item.type = "object";
  preview_item.source_file = "layout/workcell_studio_layout.yaml";
  preview_item.has_mesh_metadata = true;
  preview_item.mesh_path = "assets/mesh.stl";
  preview.items.push_back(preview_item);

  const auto result = workcell_builder::bootstrap_editable_layout_from_scene_sources(root, "wc_bootstrap_priority", preview);
  EXPECT_EQ(result.source_used, "layout/workcell_studio_layout.yaml");
  EXPECT_EQ(result.editable_items_created, 1u);
  EXPECT_EQ(result.skipped_locked_items, 1u);
  ASSERT_TRUE(result.layout["items"] && result.layout["items"].IsSequence());
  ASSERT_EQ(result.layout["items"].size(), 1u);
  EXPECT_EQ(result.layout["items"][0]["id"].as<std::string>(), "existing_editable");
  EXPECT_TRUE(result.expected_output_file == root / "layout" / "workcell_studio_layout.yaml");

  fs::remove_all(root);
}

TEST(WorkcellStudioCanvasMesh, BootstrapEditableLayoutFallsBackThroughSourcesAndReportsPreviewBlockers)
{
  const fs::path root = fs::temp_directory_path() / "wc_bootstrap_fallbacks";
  fs::remove_all(root);
  fs::create_directories(root);

  write_file(root / "environment_layout.yaml",
    "schema_version: environment_layout/v1\n"
    "assets:\n"
    "  - id: env_layout_asset\n"
    "    type: bin\n"
    "    placeable: true\n"
    "    pose: {xyz: [1, 2, 3], rpy: [0, 0, 0]}\n");
  write_file(root / "environment.yaml", "placed_objects: [{id: env_object, type: box}]\n");
  write_file(root / "cell_definition.yaml", "assets: [{id: cell_asset, type: fixture}]\n");

  workcell_builder::WorkcellStudioCanvasModel preview;
  preview.scene_name = "wc_bootstrap_fallbacks";
  const auto env_layout_result = workcell_builder::bootstrap_editable_layout_from_scene_sources(root, "wc_bootstrap_fallbacks", preview);
  EXPECT_EQ(env_layout_result.source_used, "environment_layout.yaml");
  ASSERT_EQ(env_layout_result.layout["items"].size(), 1u);
  EXPECT_EQ(env_layout_result.layout["items"][0]["id"].as<std::string>(), "env_layout_asset");

  fs::remove(root / "environment_layout.yaml");
  const auto env_result = workcell_builder::bootstrap_editable_layout_from_scene_sources(root, "wc_bootstrap_fallbacks", preview);
  EXPECT_EQ(env_result.source_used, "environment.yaml");
  EXPECT_NE(std::find(env_result.blockers.begin(), env_result.blockers.end(), "no environment_layout.yaml"), env_result.blockers.end());

  fs::remove(root / "environment.yaml");
  const auto cell_result = workcell_builder::bootstrap_editable_layout_from_scene_sources(root, "wc_bootstrap_fallbacks", preview);
  EXPECT_EQ(cell_result.source_used, "cell_definition.yaml");
  EXPECT_NE(std::find(cell_result.blockers.begin(), cell_result.blockers.end(), "no environment.yaml"), cell_result.blockers.end());

  fs::remove(root / "cell_definition.yaml");
  workcell_builder::WorkcellStudioCanvasItem locked_item;
  locked_item.id = "locked_preview";
  locked_item.type = "object";
  locked_item.locked = true;
  preview.items = {locked_item};
  const auto locked_result = workcell_builder::bootstrap_editable_layout_from_scene_sources(root, "wc_bootstrap_fallbacks", preview);
  EXPECT_EQ(locked_result.editable_items_created, 0u);
  EXPECT_NE(std::find(locked_result.blockers.begin(), locked_result.blockers.end(), "no cell_definition.yaml"), locked_result.blockers.end());
  EXPECT_NE(std::find(locked_result.blockers.begin(), locked_result.blockers.end(), "preview locked-only"), locked_result.blockers.end());

  workcell_builder::WorkcellStudioCanvasItem fallback_item;
  fallback_item.id = "fallback_preview";
  fallback_item.type = "object";
  fallback_item.provenance = workcell_builder::WorkcellStudioItemProvenance::StaticFallbackPreview;
  preview.items = {fallback_item};
  const auto fallback_result = workcell_builder::bootstrap_editable_layout_from_scene_sources(root, "wc_bootstrap_fallbacks", preview);
  EXPECT_NE(std::find(fallback_result.blockers.begin(), fallback_result.blockers.end(), "preview fallback-only"), fallback_result.blockers.end());

  workcell_builder::WorkcellStudioCanvasItem unsafe_item;
  unsafe_item.id = "unsafe_preview";
  unsafe_item.type = "object";
  preview.items = {unsafe_item};
  const auto unsafe_result = workcell_builder::bootstrap_editable_layout_from_scene_sources(root, "wc_bootstrap_fallbacks", preview);
  EXPECT_NE(std::find(unsafe_result.blockers.begin(), unsafe_result.blockers.end(), "unsafe/missing mesh metadata"), unsafe_result.blockers.end());

  fs::remove_all(root);
}
