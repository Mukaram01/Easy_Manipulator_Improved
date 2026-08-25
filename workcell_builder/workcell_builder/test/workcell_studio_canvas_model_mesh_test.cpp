#include <gtest/gtest.h>
#include <boost/filesystem.hpp>
#include <algorithm>
#include <fstream>
#include <map>
#include <set>
#include <sstream>

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


TEST(WorkcellStudioCanvasModelLayoutStatus, ReportsCanonicalLayoutSource)
{
  const fs::path root = fs::temp_directory_path() / "wc_canvas_layout_status_canonical";
  fs::remove_all(root);
  fs::create_directories(root);

  write_file(root / "environment.yaml", "robot: ur5\n");
  write_file(root / "scene_manifest.yaml", "template_name: demo\n");
  write_file(root / "config" / "task_recipe.yaml", "pick_source: a\nplace_target: b\n");
  write_file(root / "layout" / "workcell_studio_layout.yaml",
    "schema_version: workcell_studio_layout/v1\n"
    "items:\n"
    "  - id: table\n"
    "    type: table\n"
    "    role: table\n"
    "    editable: true\n"
    "    pose: {xyz: [0.1, 0.2, 0.0], rpy: [0.0, 0.0, 0.0]}\n");

  const auto model = workcell_builder::build_workcell_studio_canvas_model(root, "demo");
  EXPECT_EQ(model.layout_source_kind, "canonical");
  EXPECT_EQ(model.layout_source_path, (root / "layout" / "workcell_studio_layout.yaml").string());
  EXPECT_EQ(model.layout_load_message, "Loaded canonical layout from " + (root / "layout" / "workcell_studio_layout.yaml").string());
}

TEST(WorkcellStudioCanvasModelLayoutStatus, ReportsEmptyCanonicalLayoutMetadata)
{
  const fs::path root = fs::temp_directory_path() / "wc_canvas_layout_status_empty_canonical";
  fs::remove_all(root);
  fs::create_directories(root);

  write_file(root / "environment.yaml", "robot: ur5\n");
  write_file(root / "scene_manifest.yaml", "template_name: demo\n");
  write_file(root / "config" / "task_recipe.yaml", "pick_source: a\nplace_target: b\n");
  write_file(root / "layout" / "workcell_studio_layout.yaml", "schema_version: workcell_studio_layout/v1\nitems: []\n");

  const auto model = workcell_builder::build_workcell_studio_canvas_model(root, "demo");
  EXPECT_EQ(model.layout_source_kind, "canonical");
  EXPECT_EQ(model.layout_load_message, "Loaded empty canonical layout metadata from " + (root / "layout" / "workcell_studio_layout.yaml").string());
}

TEST(WorkcellStudioCanvasModelLayoutStatus, ReportsCanonicalFailureThenLegacyImport)
{
  const fs::path root = fs::temp_directory_path() / "wc_canvas_layout_status_legacy";
  fs::remove_all(root);
  fs::create_directories(root);

  write_file(root / "environment.yaml", "robot: ur5\n");
  write_file(root / "scene_manifest.yaml", "template_name: demo\n");
  write_file(root / "config" / "task_recipe.yaml", "pick_source: a\nplace_target: b\n");
  write_file(root / "layout" / "workcell_studio_layout.yaml", "schema_version: [not valid YAML\n");
  write_file(root / "environment_layout.yaml",
    "schema_version: environment_layout/v1\n"
    "placed_assets:\n"
    "  - id: legacy_table\n"
    "    type: table\n"
    "    role: table\n"
    "    editable: true\n"
    "    pose: {xyz: [0.0, 0.0, 0.0], rpy: [0.0, 0.0, 0.0]}\n");

  const auto model = workcell_builder::build_workcell_studio_canvas_model(root, "demo");
  EXPECT_EQ(model.layout_source_kind, "legacy");
  EXPECT_EQ(model.layout_source_path, (root / "environment_layout.yaml").string());
  EXPECT_NE(model.layout_load_message.find("Failed to load layout " + (root / "layout" / "workcell_studio_layout.yaml").string() + ": "), std::string::npos);
  EXPECT_NE(model.layout_load_message.find("; using next fallback"), std::string::npos);
  EXPECT_NE(model.layout_load_message.find("Imported legacy layout from " + (root / "environment_layout.yaml").string()), std::string::npos);
}

TEST(WorkcellStudioCanvasModelLayoutStatus, ReportsLockedPreviewFallbackWhenNoLayoutSourceExists)
{
  const fs::path root = fs::temp_directory_path() / "wc_canvas_layout_status_no_source";
  fs::remove_all(root);
  fs::create_directories(root);

  write_file(root / "environment.yaml", "robot: ur5\n");
  write_file(root / "scene_manifest.yaml", "template_name: demo\n");
  write_file(root / "config" / "task_recipe.yaml", "pick_source: a\nplace_target: b\n");

  const auto model = workcell_builder::build_workcell_studio_canvas_model(root, "demo");
  EXPECT_EQ(model.layout_source_kind, "locked_preview_fallback");
  EXPECT_EQ(model.layout_load_message, "No editable layout source found; using locked preview fallback");
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

TEST(WorkcellStudioCanvasMesh, UsesEnvironmentLayoutAsEditableFallback)
{
  const fs::path root = fs::temp_directory_path() / "wc_canvas_env_layout_fallback";
  fs::remove_all(root);
  fs::create_directories(root);

  write_file(root / "environment.yaml", "robot: ur5\nend_effector: gripper\n");
  write_file(root / "scene_manifest.yaml", "template_name: demo\n");
  write_file(root / "config" / "task_recipe.yaml", "pick_source: a\nplace_target: b\n");
  write_file(root / "environment_layout.yaml",
    "schema_version: environment_layout/v1\n"
    "items:\n"
    "  - id: table\n"
    "    type: table\n"
    "    pose: {xyz: [1.0, 2.0, 3.0], rpy: [0.1, 0.2, 0.3]}\n"
    "    size: {width: 1.1, depth: 1.2, height: 1.3}\n"
    "assets:\n"
    "  - id: asset_a\n"
    "    type: fixture\n"
    "placed_assets:\n"
    "  - id: placed_a\n"
    "    type: bin\n"
    "objects:\n"
    "  - id: object_b\n"
    "    type: part\n"
    "zones:\n"
    "  - id: zone_a\n"
    "    type: zone\n"
    "targets:\n"
    "  - id: target_a\n"
    "    type: target\n"
    "camera:\n"
    "  id: camera\n"
    "  type: camera\n"
    "  locked: true\n");

  const auto model = workcell_builder::build_workcell_studio_canvas_model(root, "demo");
  std::set<std::string> editable_ids;
  bool saw_table = false;
  bool saw_camera = false;
  for (const auto & item : model.items) {
    if (item.provenance != workcell_builder::WorkcellStudioItemProvenance::EditableLayout) continue;
    editable_ids.insert(item.id);
    EXPECT_EQ(item.source_file, "environment_layout.yaml");
    if (item.id == "table") {
      saw_table = true;
      EXPECT_FALSE(item.locked);
      EXPECT_DOUBLE_EQ(item.x, 1.0);
      EXPECT_DOUBLE_EQ(item.y, 2.0);
      EXPECT_DOUBLE_EQ(item.z, 3.0);
      EXPECT_DOUBLE_EQ(item.roll, 0.1);
      EXPECT_DOUBLE_EQ(item.pitch, 0.2);
      EXPECT_DOUBLE_EQ(item.yaw, 0.3);
      EXPECT_DOUBLE_EQ(item.width, 1.1);
      EXPECT_DOUBLE_EQ(item.depth, 1.2);
      EXPECT_DOUBLE_EQ(item.height, 1.3);
    }
    if (item.id == "camera") {
      saw_camera = true;
      EXPECT_TRUE(item.locked);
    }
  }
  EXPECT_TRUE(saw_table);
  EXPECT_TRUE(saw_camera);
  EXPECT_EQ(editable_ids.count("asset_a"), 1u);
  EXPECT_EQ(editable_ids.count("placed_a"), 1u);
  EXPECT_EQ(editable_ids.count("object_b"), 1u);
  EXPECT_EQ(editable_ids.count("zone_a"), 1u);
  EXPECT_EQ(editable_ids.count("target_a"), 1u);
}

TEST(WorkcellStudioCanvasMesh, UsesSafeManifestLayoutOnlyAfterCanonicalAndLegacyMissing)
{
  const fs::path root = fs::temp_directory_path() / "wc_canvas_manifest_layout_fallback";
  fs::remove_all(root);
  fs::create_directories(root);

  write_file(root / "environment.yaml", "robot: ur5\nend_effector: gripper\n");
  write_file(root / "scene_manifest.yaml",
    "template_name: demo\n"
    "files:\n"
    "  editable_layout: generated/editor_layout.yaml\n");
  write_file(root / "config" / "task_recipe.yaml", "pick_source: a\nplace_target: b\n");
  write_file(root / "generated" / "editor_layout.yaml",
    "schema_version: workcell_studio_layout/v1\n"
    "items:\n"
    "  - id: manifest_item\n"
    "    type: fixture\n"
    "    pose: {xyz: [0.4, 0.5, 0.6], rpy: [0.0, 0.0, 0.0]}\n");

  const auto model = workcell_builder::build_workcell_studio_canvas_model(root, "demo");
  bool found = false;
  for (const auto & item : model.items) {
    if (item.id != "manifest_item") continue;
    found = true;
    EXPECT_EQ(item.provenance, workcell_builder::WorkcellStudioItemProvenance::EditableLayout);
    EXPECT_EQ(item.source_file, "generated/editor_layout.yaml");
  }
  EXPECT_TRUE(found);
}

TEST(WorkcellStudioCanvasMesh, RejectsUnsafeManifestLayoutPath)
{
  const fs::path root = fs::temp_directory_path() / "wc_canvas_manifest_layout_unsafe";
  fs::remove_all(root);
  fs::create_directories(root);

  write_file(root / "environment.yaml", "robot: ur5\nend_effector: gripper\n");
  write_file(root / "scene_manifest.yaml",
    "template_name: demo\n"
    "files:\n"
    "  editable_layout: ../outside.yaml\n");
  write_file(root / "config" / "task_recipe.yaml", "pick_source: a\nplace_target: b\n");

  const auto model = workcell_builder::build_workcell_studio_canvas_model(root, "demo");
  for (const auto & item : model.items) {
    EXPECT_NE(item.id, "outside_item");
    if (item.id == "table") {
      EXPECT_EQ(item.provenance, workcell_builder::WorkcellStudioItemProvenance::StaticFallbackPreview);
    }
  }
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


TEST(WorkcellStudioCanvasMesh, BootstrapFromEnvironmentCreatesEditableLayoutWithoutExistingLayout)
{
  const fs::path root = fs::temp_directory_path() / "wc_bootstrap_no_layout";
  fs::remove_all(root);
  fs::create_directories(root);

  write_file(root / "environment.yaml",
    "schema_version: workcell_scene/v1\n"
    "scene:\n  id: demo\n"
    "placed_objects:\n"
    "  - id: table_main\n"
    "    type: table\n"
    "    mesh: meshes/visual/table_main.stl\n"
    "    pose:\n"
    "      xyz: [0.25, -0.5, 0.75]\n"
    "      rpy: [0.1, 0.2, 0.3]\n"
    "    size: {width: 1.0, depth: 0.6, height: 0.2}\n");
  ASSERT_FALSE(fs::exists(root / "layout" / "workcell_studio_layout.yaml"));

  const auto summary = workcell_builder::bootstrap_editable_layout_from_trusted_canonical_yaml(root, "demo");
  EXPECT_EQ(summary.total_preview_items, 1u);
  EXPECT_EQ(summary.editable_items_created, 1u);
  ASSERT_TRUE(summary.layout["items"] && summary.layout["items"].IsSequence());
  ASSERT_EQ(summary.layout["items"].size(), 1u);
  EXPECT_EQ(summary.layout["items"][0]["id"].as<std::string>(), "table_main");
  EXPECT_TRUE(summary.layout["items"][0]["editable"].as<bool>());
  EXPECT_FALSE(summary.layout["items"][0]["locked"].as<bool>());
}

TEST(WorkcellStudioCanvasMesh, LockedOnlyLayoutBecomesReadyAfterCanonicalBootstrapWrite)
{
  const fs::path root = fs::temp_directory_path() / "wc_bootstrap_locked_only";
  fs::remove_all(root);
  fs::create_directories(root);

  write_file(root / "layout" / "workcell_studio_layout.yaml",
    "schema_version: workcell_studio_layout/v1\n"
    "items:\n"
    "  - id: locked_table\n"
    "    editable: true\n"
    "    locked: true\n");
  write_file(root / "environment_layout.yaml", "schema_version: environment_layout/v1\nitems: []\n");
  write_file(root / "environment.yaml",
    "schema_version: workcell_scene/v1\n"
    "placed_objects:\n"
    "  - id: canonical_table\n"
    "    type: table\n"
    "    pose:\n"
    "      xyz: [1.0, 2.0, 3.0]\n"
    "      rpy: [0.0, 0.1, 0.2]\n");

  EXPECT_FALSE(workcell_builder::is_save_layout_workflow_ready(root));
  const auto summary = workcell_builder::bootstrap_editable_layout_from_trusted_canonical_yaml(root, "demo");
  ASSERT_EQ(summary.editable_items_created, 1u);
  write_yaml_file(root / "layout" / "workcell_studio_layout.yaml", summary.layout);

  const auto inspection = workcell_builder::inspect_editable_layout_entries(root);
  EXPECT_EQ(inspection.editable_item_count, 1u);
  EXPECT_TRUE(workcell_builder::is_save_layout_workflow_ready(root));
}

TEST(WorkcellStudioCanvasMesh, BootstrapSaveReloadPreservesIdsAndPose)
{
  const fs::path root = fs::temp_directory_path() / "wc_bootstrap_save_reload_pose";
  fs::remove_all(root);
  fs::create_directories(root);

  write_file(root / "environment.yaml",
    "schema_version: workcell_scene/v1\n"
    "placed_objects:\n"
    "  - id: precise_fixture\n"
    "    type: fixture\n"
    "    mesh: meshes/visual/precise_fixture.stl\n"
    "    pose:\n"
    "      xyz: [0.125, -0.25, 0.375]\n"
    "      rpy: [1.0, -0.5, 0.25]\n");

  const auto summary = workcell_builder::bootstrap_editable_layout_from_trusted_canonical_yaml(root, "demo");
  ASSERT_EQ(summary.editable_items_created, 1u);
  write_yaml_file(root / "layout" / "workcell_studio_layout.yaml", summary.layout);

  const auto reloaded = workcell_builder::build_workcell_studio_canvas_model(root, "demo");
  bool found = false;
  for (const auto & item : reloaded.items) {
    if (item.id != "precise_fixture") continue;
    found = true;
    EXPECT_EQ(item.provenance, workcell_builder::WorkcellStudioItemProvenance::EditableLayout);
    EXPECT_DOUBLE_EQ(item.x, 0.125);
    EXPECT_DOUBLE_EQ(item.y, -0.25);
    EXPECT_DOUBLE_EQ(item.z, 0.375);
    EXPECT_DOUBLE_EQ(item.roll, 1.0);
    EXPECT_DOUBLE_EQ(item.pitch, -0.5);
    EXPECT_DOUBLE_EQ(item.yaw, 0.25);
  }
  EXPECT_TRUE(found);

  const YAML::Node saved = load_yaml_file(root / "layout" / "workcell_studio_layout.yaml");
  ASSERT_TRUE(saved["items"] && saved["items"].IsSequence());
  ASSERT_EQ(saved["items"].size(), 1u);
  EXPECT_EQ(saved["items"][0]["id"].as<std::string>(), "precise_fixture");
  EXPECT_DOUBLE_EQ(saved["items"][0]["pose"]["xyz"][0].as<double>(), 0.125);
  EXPECT_DOUBLE_EQ(saved["items"][0]["pose"]["xyz"][1].as<double>(), -0.25);
  EXPECT_DOUBLE_EQ(saved["items"][0]["pose"]["xyz"][2].as<double>(), 0.375);
  EXPECT_DOUBLE_EQ(saved["items"][0]["pose"]["rpy"][0].as<double>(), 1.0);
  EXPECT_DOUBLE_EQ(saved["items"][0]["pose"]["rpy"][1].as<double>(), -0.5);
  EXPECT_DOUBLE_EQ(saved["items"][0]["pose"]["rpy"][2].as<double>(), 0.25);
}

TEST(WorkcellStudioCanvasMesh, InvalidLayoutYamlInspectionIsInvalidAndDoesNotThrow)
{
  const fs::path root = fs::temp_directory_path() / "wc_invalid_layout_no_throw";
  fs::remove_all(root);
  fs::create_directories(root);

  write_file(root / "layout" / "workcell_studio_layout.yaml", "schema_version: [oops\nitems: [\n");
  EXPECT_NO_THROW({
    const auto inspection = workcell_builder::inspect_editable_layout_entries(root);
    EXPECT_TRUE(inspection.exists);
    EXPECT_FALSE(inspection.valid);
    EXPECT_EQ(inspection.editable_item_count, 0u);
  });
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

  workcell_builder::WorkcellStudioCanvasItem reach_overlay = safe;
  reach_overlay.id = "robot_reach_overlay";
  reach_overlay.type = "reach";
  reach_overlay.role = "overlay";
  reach_overlay.locked = true;
  model.items.push_back(reach_overlay);

  const auto summary = workcell_builder::build_starter_layout_entries_from_preview(model);
  EXPECT_EQ(summary.total_preview_items, 7u);
  EXPECT_EQ(summary.skipped_locked_items, 0u);
  EXPECT_EQ(summary.skipped_static_fallback_items, 1u);
  EXPECT_EQ(summary.skipped_unsafe_or_missing_metadata_items, 4u);
  EXPECT_EQ(summary.editable_items_created, 2u);

  EXPECT_FALSE(summary.layout["empty_layout_marker"].as<bool>());
  const YAML::Node items = summary.layout["items"];
  ASSERT_TRUE(items && items.IsSequence());
  ASSERT_EQ(items.size(), 2u);
  EXPECT_EQ(items[0]["id"].as<std::string>(), "safe_item__editable_copy");
  EXPECT_EQ(items[0]["mesh"]["path"].as<std::string>(), "meshes/visual/safe_item.stl");
  EXPECT_TRUE(items[1]["editable"].as<bool>());
  EXPECT_FALSE(items[1]["locked"].as<bool>());
  EXPECT_EQ(items[1]["id"].as<std::string>(), "locked_item__editable_copy");
  EXPECT_EQ(items[1]["provenance"]["copy_kind"].as<std::string>(), "editable_layout_copy");
}

TEST(WorkcellStudioCanvasMesh, PreviewToEditableLayoutRoundTripKeepsGeneratedPreviewLocked)
{
  const fs::path root = fs::temp_directory_path() / "wc_preview_to_editable_round_trip";
  fs::remove_all(root);
  fs::create_directories(root / "layout");

  write_file(root / "environment.yaml", "robot: ur5\nend_effector: robotiq_2f\n");
  write_file(root / "scene_manifest.yaml", "template_name: demo\n");
  write_file(root / "config" / "task_recipe.yaml", "pick_source: generated_fixture\nplace_target: bin_a\n");

  workcell_builder::WorkcellStudioCanvasModel preview;
  preview.scene_name = "demo";

  workcell_builder::WorkcellStudioCanvasItem generated_fixture;
  generated_fixture.id = "generated_fixture";
  generated_fixture.label = "Generated Fixture";
  generated_fixture.type = "fixture";
  generated_fixture.category = "fixtures";
  generated_fixture.role = "workholding";
  generated_fixture.x = 0.11;
  generated_fixture.y = -0.22;
  generated_fixture.z = 0.33;
  generated_fixture.roll = 0.44;
  generated_fixture.pitch = -0.55;
  generated_fixture.yaw = 0.66;
  generated_fixture.width = 0.77;
  generated_fixture.depth = 0.88;
  generated_fixture.height = 0.99;
  generated_fixture.source_file = "urdf/scene.urdf.xacro";
  generated_fixture.source_package = "generated_scene_pkg";
  generated_fixture.mesh_path = "package://fixture_asset_pkg/meshes/visual/generated_fixture.stl";
  generated_fixture.mesh_source_package = "fixture_asset_pkg";
  generated_fixture.mesh_scale_x = 1.1;
  generated_fixture.mesh_scale_y = 1.2;
  generated_fixture.mesh_scale_z = 1.3;
  generated_fixture.has_mesh_metadata = true;
  generated_fixture.locked = true;
  generated_fixture.editable = false;
  generated_fixture.provenance = workcell_builder::WorkcellStudioItemProvenance::GeneratedOrLegacyPreview;
  preview.items.push_back(generated_fixture);

  workcell_builder::WorkcellStudioCanvasItem static_fallback = generated_fixture;
  static_fallback.id = "static_fallback_fixture";
  static_fallback.provenance = workcell_builder::WorkcellStudioItemProvenance::StaticFallbackPreview;
  preview.items.push_back(static_fallback);

  workcell_builder::WorkcellStudioCanvasItem overlay_helper = generated_fixture;
  overlay_helper.id = "reach_overlay_helper";
  overlay_helper.type = "reach";
  overlay_helper.category = "overlay";
  overlay_helper.role = "overlay";
  preview.items.push_back(overlay_helper);

  const auto result = workcell_builder::bootstrap_editable_layout_from_scene_sources(root, "demo", preview);
  EXPECT_EQ(result.source_used, "preview_model");
  EXPECT_EQ(result.editable_items_created, 1u);
  EXPECT_EQ(result.skipped_static_fallback_items, 1u);
  EXPECT_EQ(result.skipped_unsafe_or_missing_metadata_items, 1u);
  EXPECT_EQ(result.skipped_locked_items, 0u);

  write_yaml_file(root / "layout" / "workcell_studio_layout.yaml", result.layout);
  const YAML::Node saved = load_yaml_file(root / "layout" / "workcell_studio_layout.yaml");
  const YAML::Node saved_items = saved["items"];
  ASSERT_TRUE(saved_items && saved_items.IsSequence());
  ASSERT_EQ(saved_items.size(), 1u);

  const YAML::Node copied = saved_items[0];
  ASSERT_TRUE(copied && copied.IsMap());
  EXPECT_EQ(copied["id"].as<std::string>(), "generated_fixture__editable_copy");
  EXPECT_NE(copied["id"].as<std::string>(), generated_fixture.id);
  EXPECT_EQ(copied["source_layer"].as<std::string>(), "editable_layout");
  EXPECT_TRUE(copied["editable"].as<bool>());
  EXPECT_FALSE(copied["locked"].as<bool>());
  EXPECT_EQ(copied["display_name"].as<std::string>(), generated_fixture.label);
  EXPECT_EQ(copied["type"].as<std::string>(), generated_fixture.type);
  EXPECT_EQ(copied["category"].as<std::string>(), generated_fixture.category);
  EXPECT_EQ(copied["role"].as<std::string>(), generated_fixture.role);
  EXPECT_DOUBLE_EQ(copied["pose"]["xyz"][0].as<double>(), generated_fixture.x);
  EXPECT_DOUBLE_EQ(copied["pose"]["xyz"][1].as<double>(), generated_fixture.y);
  EXPECT_DOUBLE_EQ(copied["pose"]["xyz"][2].as<double>(), generated_fixture.z);
  EXPECT_DOUBLE_EQ(copied["pose"]["rpy"][0].as<double>(), generated_fixture.roll);
  EXPECT_DOUBLE_EQ(copied["pose"]["rpy"][1].as<double>(), generated_fixture.pitch);
  EXPECT_DOUBLE_EQ(copied["pose"]["rpy"][2].as<double>(), generated_fixture.yaw);
  EXPECT_DOUBLE_EQ(copied["dimensions"][0].as<double>(), generated_fixture.width);
  EXPECT_DOUBLE_EQ(copied["dimensions"][1].as<double>(), generated_fixture.depth);
  EXPECT_DOUBLE_EQ(copied["dimensions"][2].as<double>(), generated_fixture.height);
  EXPECT_EQ(copied["mesh"]["path"].as<std::string>(), generated_fixture.mesh_path);
  EXPECT_EQ(copied["mesh"]["source"].as<std::string>(), generated_fixture.source_file);
  EXPECT_EQ(copied["mesh"]["source_package"].as<std::string>(), generated_fixture.mesh_source_package);
  EXPECT_DOUBLE_EQ(copied["mesh"]["scale"][0].as<double>(), generated_fixture.mesh_scale_x);
  EXPECT_DOUBLE_EQ(copied["mesh"]["scale"][1].as<double>(), generated_fixture.mesh_scale_y);
  EXPECT_DOUBLE_EQ(copied["mesh"]["scale"][2].as<double>(), generated_fixture.mesh_scale_z);
  EXPECT_EQ(copied["source_package"].as<std::string>(), generated_fixture.source_package);
  EXPECT_EQ(copied["mesh_source_package"].as<std::string>(), generated_fixture.mesh_source_package);
  EXPECT_EQ(copied["preview_source_id"].as<std::string>(), generated_fixture.id);
  EXPECT_EQ(copied["provenance"]["original_source_id"].as<std::string>(), generated_fixture.id);
  EXPECT_EQ(copied["provenance"]["preview_source_id"].as<std::string>(), generated_fixture.id);
  EXPECT_EQ(copied["provenance"]["original_source_layer"].as<std::string>(), "locked_generated_urdf_visual");
  EXPECT_EQ(copied["provenance"]["original_source_file"].as<std::string>(), generated_fixture.source_file);

  const auto reloaded = workcell_builder::build_workcell_studio_canvas_model(root, "demo");
  bool found_reloaded_copy = false;
  for (const auto & item : reloaded.items) {
    if (item.id != "generated_fixture__editable_copy") continue;
    found_reloaded_copy = true;
    EXPECT_EQ(item.provenance, workcell_builder::WorkcellStudioItemProvenance::EditableLayout);
    EXPECT_TRUE(item.editable);
    EXPECT_FALSE(item.locked);
    EXPECT_EQ(item.source_file, "editable_layout");
    EXPECT_EQ(item.label, generated_fixture.label);
    EXPECT_EQ(item.type, generated_fixture.type);
    EXPECT_EQ(item.category, generated_fixture.category);
    EXPECT_EQ(item.role, generated_fixture.role);
    EXPECT_DOUBLE_EQ(item.x, generated_fixture.x);
    EXPECT_DOUBLE_EQ(item.y, generated_fixture.y);
    EXPECT_DOUBLE_EQ(item.z, generated_fixture.z);
    EXPECT_DOUBLE_EQ(item.roll, generated_fixture.roll);
    EXPECT_DOUBLE_EQ(item.pitch, generated_fixture.pitch);
    EXPECT_DOUBLE_EQ(item.yaw, generated_fixture.yaw);
    EXPECT_DOUBLE_EQ(item.width, generated_fixture.width);
    EXPECT_DOUBLE_EQ(item.depth, generated_fixture.depth);
    EXPECT_DOUBLE_EQ(item.height, generated_fixture.height);
    EXPECT_EQ(item.mesh_source_package, generated_fixture.mesh_source_package);
    EXPECT_TRUE(item.has_mesh_metadata);
  }
  EXPECT_TRUE(found_reloaded_copy);

  ASSERT_EQ(preview.items[0].id, generated_fixture.id);
  EXPECT_TRUE(preview.items[0].locked);
  EXPECT_FALSE(preview.items[0].editable);
  EXPECT_EQ(preview.items[0].provenance, workcell_builder::WorkcellStudioItemProvenance::GeneratedOrLegacyPreview);

  fs::remove_all(root);
}

TEST(WorkcellStudioCanvasMesh, BootstrapPreviewFallbackCopiesSafeLockedPhysicalItemsOnly)
{
  const fs::path root = fs::temp_directory_path() / "wc_bootstrap_preview_locked_physical";
  fs::remove_all(root);
  fs::create_directories(root);

  workcell_builder::WorkcellStudioCanvasModel model;
  model.scene_name = "demo";

  workcell_builder::WorkcellStudioCanvasItem locked_physical;
  locked_physical.id = "robot_base";
  locked_physical.type = "robot_base";
  locked_physical.role = "robot";
  locked_physical.source_file = "generated/robot.urdf.xacro";
  locked_physical.mesh_path = "meshes/visual/robot_base.stl";
  locked_physical.has_mesh_metadata = true;
  locked_physical.provenance = workcell_builder::WorkcellStudioItemProvenance::GeneratedOrLegacyPreview;
  locked_physical.locked = true;
  model.items.push_back(locked_physical);

  workcell_builder::WorkcellStudioCanvasItem locked_helper = locked_physical;
  locked_helper.id = "robot_reach";
  locked_helper.type = "reach";
  locked_helper.role = "overlay";
  model.items.push_back(locked_helper);

  const auto result = workcell_builder::bootstrap_editable_layout_from_scene_sources(root, "demo", model);
  EXPECT_EQ(result.source_used, "preview_model");
  EXPECT_EQ(result.editable_items_created, 1u);
  EXPECT_EQ(result.skipped_locked_items, 0u);
  EXPECT_EQ(result.skipped_unsafe_or_missing_metadata_items, 1u);

  const YAML::Node items = result.layout["items"];
  ASSERT_TRUE(items && items.IsSequence());
  ASSERT_EQ(items.size(), 1u);
  EXPECT_EQ(items[0]["id"].as<std::string>(), "robot_base__editable_copy");
  EXPECT_TRUE(items[0]["editable"].as<bool>());
  EXPECT_FALSE(items[0]["locked"].as<bool>());
  EXPECT_EQ(items[0]["provenance"]["copy_kind"].as<std::string>(), "editable_layout_copy");

  EXPECT_TRUE(model.items[0].locked) << "preview fallback must not mutate the generated preview item lock guard";

  fs::remove_all(root);
}

TEST(WorkcellStudioCanvasMesh, PreviewStarterLayoutCopiesLockedGeneratedPhysicalItemsWithDistinctEditableProvenance)
{
  const fs::path root = fs::temp_directory_path() / "wc_preview_distinct_editable_copy";
  fs::remove_all(root);
  fs::create_directories(root / "meshes" / "visual");
  const fs::path mesh_path = root / "meshes" / "visual" / "fixture.stl";
  write_file(mesh_path, "solid fixture\nendsolid fixture\n");

  workcell_builder::WorkcellStudioCanvasModel model;
  model.scene_name = "demo";

  workcell_builder::WorkcellStudioCanvasItem generated_fixture;
  generated_fixture.id = "generated_fixture";
  generated_fixture.type = "fixture";
  generated_fixture.category = "fixtures";
  generated_fixture.role = "workholding";
  generated_fixture.label = "Generated Fixture";
  generated_fixture.source_file = "urdf/scene.urdf.xacro";
  generated_fixture.source_package = "generated_scene_pkg";
  generated_fixture.mesh_source_package = "fixture_asset_pkg";
  generated_fixture.x = 0.11;
  generated_fixture.y = 0.22;
  generated_fixture.z = 0.33;
  generated_fixture.roll = 0.44;
  generated_fixture.pitch = 0.55;
  generated_fixture.yaw = 0.66;
  generated_fixture.width = 0.77;
  generated_fixture.depth = 0.88;
  generated_fixture.height = 0.99;
  generated_fixture.mesh_path = mesh_path.string();
  generated_fixture.mesh_scale_x = 1.1;
  generated_fixture.mesh_scale_y = 1.2;
  generated_fixture.mesh_scale_z = 1.3;
  generated_fixture.has_mesh_metadata = true;
  generated_fixture.provenance = workcell_builder::WorkcellStudioItemProvenance::GeneratedOrLegacyPreview;
  generated_fixture.locked = true;
  generated_fixture.editable = false;
  model.items.push_back(generated_fixture);

  workcell_builder::WorkcellStudioCanvasItem static_fallback = generated_fixture;
  static_fallback.id = "static_fallback_fixture";
  static_fallback.provenance = workcell_builder::WorkcellStudioItemProvenance::StaticFallbackPreview;
  model.items.push_back(static_fallback);

  workcell_builder::WorkcellStudioCanvasItem overlay_helper = generated_fixture;
  overlay_helper.id = "reach_overlay_helper";
  overlay_helper.type = "reach";
  overlay_helper.role = "overlay";
  model.items.push_back(overlay_helper);

  const auto summary = workcell_builder::build_starter_layout_entries_from_preview(model);
  EXPECT_EQ(summary.total_preview_items, 3u);
  EXPECT_EQ(summary.skipped_locked_items, 0u);
  EXPECT_EQ(summary.skipped_static_fallback_items, 1u);
  EXPECT_EQ(summary.skipped_unsafe_or_missing_metadata_items, 1u);
  EXPECT_EQ(summary.editable_items_created, 1u);

  const YAML::Node items = summary.layout["items"];
  ASSERT_TRUE(items && items.IsSequence());
  ASSERT_EQ(items.size(), 1u);
  const YAML::Node copied = items[0];
  EXPECT_EQ(copied["id"].as<std::string>(), "generated_fixture__editable_copy");
  EXPECT_NE(copied["id"].as<std::string>(), generated_fixture.id);
  EXPECT_TRUE(copied["editable"].as<bool>());
  EXPECT_FALSE(copied["locked"].as<bool>());
  EXPECT_EQ(copied["source_layer"].as<std::string>(), "editable_layout");
  EXPECT_EQ(copied["display_name"].as<std::string>(), "Generated Fixture");
  EXPECT_EQ(copied["type"].as<std::string>(), "fixture");
  EXPECT_EQ(copied["category"].as<std::string>(), "fixtures");
  EXPECT_EQ(copied["role"].as<std::string>(), "workholding");
  EXPECT_EQ(copied["pose"]["xyz"][0].as<double>(), 0.11);
  EXPECT_EQ(copied["pose"]["rpy"][2].as<double>(), 0.66);
  EXPECT_EQ(copied["dimensions"][0].as<double>(), 0.77);
  EXPECT_EQ(copied["dimensions"][2].as<double>(), 0.99);
  EXPECT_EQ(copied["mesh"]["path"].as<std::string>(), mesh_path.string());
  EXPECT_EQ(copied["mesh"]["source"].as<std::string>(), "urdf/scene.urdf.xacro");
  EXPECT_EQ(copied["mesh"]["source_package"].as<std::string>(), "fixture_asset_pkg");
  EXPECT_EQ(copied["mesh"]["scale"][1].as<double>(), 1.2);
  EXPECT_EQ(copied["source_package"].as<std::string>(), "generated_scene_pkg");
  EXPECT_EQ(copied["mesh_source_package"].as<std::string>(), "fixture_asset_pkg");
  EXPECT_EQ(copied["provenance"]["copy_kind"].as<std::string>(), "editable_layout_copy");
  EXPECT_EQ(copied["provenance"]["original_source_id"].as<std::string>(), "generated_fixture");
  EXPECT_EQ(copied["provenance"]["original_source_layer"].as<std::string>(), "locked_generated_urdf_visual");
  EXPECT_EQ(copied["provenance"]["original_source_file"].as<std::string>(), "urdf/scene.urdf.xacro");
  EXPECT_NE(copied["provenance"]["original_source_id"].as<std::string>(), copied["id"].as<std::string>());
  EXPECT_TRUE(model.items[0].locked);
  EXPECT_FALSE(model.items[0].editable);
  EXPECT_EQ(model.items[0].provenance, workcell_builder::WorkcellStudioItemProvenance::GeneratedOrLegacyPreview);

  write_file(root / "environment.yaml", "robot: ur5\nend_effector: robotiq_2f\n");
  write_file(root / "scene_manifest.yaml", "template_name: demo\n");
  write_file(root / "config" / "task_recipe.yaml", "pick_source: generated_fixture\nplace_target: bin_a\n");
  write_yaml_file(root / "layout" / "workcell_studio_layout.yaml", summary.layout);

  const auto reloaded = workcell_builder::build_workcell_studio_canvas_model(root, "demo");
  bool found_editable_copy = false;
  bool found_original_locked_preview = false;
  for (const auto & item : reloaded.items) {
    if (item.id == "generated_fixture__editable_copy") {
      found_editable_copy = true;
      EXPECT_EQ(item.provenance, workcell_builder::WorkcellStudioItemProvenance::EditableLayout);
      EXPECT_TRUE(item.editable);
      EXPECT_FALSE(item.locked);
      EXPECT_EQ(item.label, "Generated Fixture");
      EXPECT_EQ(item.width, 0.77);
      EXPECT_EQ(item.height, 0.99);
    }
    if (item.id == "generated_fixture") {
      found_original_locked_preview = item.locked && !item.editable &&
        item.provenance == workcell_builder::WorkcellStudioItemProvenance::GeneratedOrLegacyPreview;
    }
  }
  EXPECT_TRUE(found_editable_copy);
  EXPECT_FALSE(found_original_locked_preview) << "the copied layout item must not masquerade as the original generated preview id";

  fs::remove_all(root);
}


TEST(WorkcellStudioCanvasMesh, Ur5CanonicalLayoutSuppressesGenericSemanticPlaceholders)
{
#ifndef WORKCELL_BUILDER_REPO_ROOT
  GTEST_SKIP() << "WORKCELL_BUILDER_REPO_ROOT is not configured for ur5_2f_test layout coverage";
#else
  const fs::path repo_root = fs::path(WORKCELL_BUILDER_REPO_ROOT);
  const fs::path source_scene = repo_root / "scenes" / "ur5_2f_test";
  ASSERT_TRUE(fs::exists(source_scene)) << "expected ur5_2f_test scene fixture";

  const fs::path editable_layout_path = source_scene / "layout" / "workcell_studio_layout.yaml";
  ASSERT_TRUE(fs::exists(editable_layout_path));
  const YAML::Node layout = load_yaml_file(editable_layout_path);
  ASSERT_TRUE(layout && layout.IsMap());
  ASSERT_EQ(layout["schema_version"].as<std::string>(), "workcell_studio_layout/v1");
  ASSERT_TRUE(layout["items"] && layout["items"].IsSequence());
  ASSERT_GT(layout["items"].size(), 0u);

  std::set<std::string> expected_editable_ids;
  for (const auto & item : layout["items"]) {
    ASSERT_TRUE(item && item.IsMap());
    ASSERT_TRUE(item["id"]);
    expected_editable_ids.insert(item["id"].as<std::string>());
  }
  ASSERT_EQ(expected_editable_ids.count("support_surface_table"), 1u);
  ASSERT_EQ(expected_editable_ids.count("pick_zone_commissioning"), 1u);
  ASSERT_EQ(expected_editable_ids.count("place_zone_default"), 1u);
  ASSERT_EQ(expected_editable_ids.count("target_bin_default"), 1u);
  ASSERT_EQ(expected_editable_ids.count("realsense_overhead"), 1u);
  ASSERT_EQ(expected_editable_ids.count("home_pose_safe"), 1u);

  const auto model = workcell_builder::build_workcell_studio_canvas_model(source_scene, "ur5_2f_test");
  std::map<std::string, int> id_counts;
  for (const auto & item : model.items) {
    ++id_counts[item.id];
  }

  for (const auto & id : expected_editable_ids) {
    EXPECT_EQ(id_counts[id], 1) << "canonical editable layout id should appear exactly once: " << id;
  }

  const std::set<std::string> suppressed_generic_placeholder_ids = {
    "table", "pick_zone", "place_zone", "bin_a", "camera", "home_pose"
  };
  for (const auto & id : suppressed_generic_placeholder_ids) {
    EXPECT_EQ(id_counts[id], 0) << "canonical semantic layout should suppress generic placeholder id: " << id;
  }

  EXPECT_EQ(id_counts["robot_base"], 1) << "locked robot preview should remain separate from editable layout items";
  EXPECT_EQ(id_counts["robot_reach"], 1) << "locked reach preview should remain separate from editable layout items";
#endif
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
    const bool helper_locked_preview = id.find("reach") != std::string::npos || id.find("warning") != std::string::npos;
    if (helper_locked_preview) {
      EXPECT_EQ(after_editable_ids.count(id), 0u) << "helper/overlay locked preview item was written editable: " << id;
    }
  }

  const fs::path fallback_scene = temp_root / (scene_name + "_fallback_only");
  copy_directory_tree(source_scene, fallback_scene);
  const fs::path fallback_layout_path = fallback_scene / "layout" / "workcell_studio_layout.yaml";
  ASSERT_TRUE(fs::exists(fallback_layout_path));
  const YAML::Node fallback_before_layout = load_yaml_file(fallback_layout_path);
  ASSERT_TRUE(fallback_before_layout && fallback_before_layout.IsMap());
  fs::remove(fallback_layout_path);
  fs::remove(fallback_scene / "environment_layout.yaml");

  const auto fallback_model = workcell_builder::build_workcell_studio_canvas_model(fallback_scene, scene_name);
  std::set<std::string> static_fallback_ids;
  for (const auto & item : fallback_model.items) {
    if (item.provenance == workcell_builder::WorkcellStudioItemProvenance::StaticFallbackPreview) {
      static_fallback_ids.insert(item.id);
    }
  }
  ASSERT_FALSE(static_fallback_ids.empty()) << "missing editable layout sources should force static fallback-only preview metadata";
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


TEST(WorkcellStudioCanvasMesh, BootstrapEnvironmentLayoutFromEditableLayoutCreatesAndMirrorsPlaceableFields)
{
  const fs::path root = fs::temp_directory_path() / "wc_bootstrap_env_layout_missing";
  fs::remove_all(root);
  fs::create_directories(root / "layout");

  YAML::Node editable(YAML::NodeType::Map);
  editable["schema_version"] = "workcell_studio_layout/v1";
  editable["scene_name"] = "bootstrap_scene";
  YAML::Node item(YAML::NodeType::Map);
  item["id"] = "editable_table";
  item["type"] = "table";
  item["category"] = "support_surface";
  item["pose"]["xyz"].push_back(1.0);
  item["pose"]["xyz"].push_back(2.0);
  item["pose"]["xyz"].push_back(3.0);
  item["pose"]["rpy"].push_back(0.1);
  item["pose"]["rpy"].push_back(0.2);
  item["pose"]["rpy"].push_back(0.3);
  item["dimensions"].push_back(0.4);
  item["dimensions"].push_back(0.5);
  item["dimensions"].push_back(0.6);
  item["source"] = "generated/environment_assets.yaml";
  item["editable"] = true;
  item["locked"] = false;
  item["mesh"]["path"] = "meshes/visual/table.stl";
  editable["items"].push_back(item);
  write_yaml_file(root / "layout" / "workcell_studio_layout.yaml", editable);
  write_file(root / "environment.yaml", "environment: {}\n");

  EXPECT_TRUE(workcell_builder::is_save_layout_workflow_ready(root));
  const auto result = workcell_builder::bootstrap_environment_layout_from_editable_layout(root, "bootstrap_scene", editable);
  EXPECT_TRUE(result.ok) << result.error;
  EXPECT_TRUE(result.created);
  EXPECT_TRUE(result.wrote);
  EXPECT_EQ(result.placed_assets_written, 1u);

  const YAML::Node env = load_yaml_file(root / "environment_layout.yaml");
  ASSERT_EQ(env["schema_version"].as<std::string>(), "environment_layout/v1");
  ASSERT_EQ(env["scene_name"].as<std::string>(), "bootstrap_scene");
  ASSERT_TRUE(env["placed_assets"] && env["placed_assets"].IsSequence());
  ASSERT_EQ(env["placed_assets"].size(), 1u);
  const YAML::Node placed = env["placed_assets"][0];
  EXPECT_EQ(placed["id"].as<std::string>(), item["id"].as<std::string>());
  EXPECT_EQ(YAML::Dump(placed["pose"]["xyz"]), YAML::Dump(item["pose"]["xyz"]));
  EXPECT_EQ(YAML::Dump(placed["pose"]["rpy"]), YAML::Dump(item["pose"]["rpy"]));
  EXPECT_EQ(YAML::Dump(placed["dimensions"]), YAML::Dump(item["dimensions"]));
  EXPECT_EQ(placed["source"].as<std::string>(), item["source"].as<std::string>());
  EXPECT_TRUE(placed["editable"].as<bool>());
  EXPECT_FALSE(placed["locked"].as<bool>());
  EXPECT_TRUE(workcell_builder::is_save_layout_workflow_ready(root));

  fs::remove_all(root);
}

TEST(WorkcellStudioCanvasMesh, BootstrapEnvironmentLayoutPreservesUnrelatedExistingFields)
{
  const fs::path root = fs::temp_directory_path() / "wc_bootstrap_env_layout_preserve";
  fs::remove_all(root);
  fs::create_directories(root);

  YAML::Node editable(YAML::NodeType::Map);
  editable["schema_version"] = "workcell_studio_layout/v1";
  YAML::Node item(YAML::NodeType::Map);
  item["id"] = "shared_asset";
  item["type"] = "fixture";
  item["category"] = "fixture";
  item["pose"]["xyz"].push_back(7.0);
  item["pose"]["xyz"].push_back(8.0);
  item["pose"]["xyz"].push_back(9.0);
  item["pose"]["rpy"].push_back(0.7);
  item["pose"]["rpy"].push_back(0.8);
  item["pose"]["rpy"].push_back(0.9);
  item["dimensions"].push_back(1.1);
  item["dimensions"].push_back(1.2);
  item["dimensions"].push_back(1.3);
  item["source"] = "layout/workcell_studio_layout.yaml";
  item["editable"] = true;
  item["locked"] = false;
  editable["items"].push_back(item);

  write_file(root / "environment_layout.yaml",
    "schema_version: environment_layout/v1\n"
    "scene_name: existing_scene\n"
    "custom_top_level: keep_me\n"
    "placed_assets:\n"
    "  - id: unrelated_asset\n"
    "    custom: untouched\n"
    "  - id: shared_asset\n"
    "    custom_existing: preserved\n"
    "    pose:\n"
    "      xyz: [0, 0, 0]\n"
    "      frame: keep_frame\n");

  const auto result = workcell_builder::bootstrap_environment_layout_from_editable_layout(root, "new_scene", editable);
  EXPECT_TRUE(result.ok) << result.error;
  EXPECT_FALSE(result.created);
  EXPECT_TRUE(result.wrote);

  const YAML::Node env = load_yaml_file(root / "environment_layout.yaml");
  EXPECT_EQ(env["scene_name"].as<std::string>(), "existing_scene");
  EXPECT_EQ(env["custom_top_level"].as<std::string>(), "keep_me");
  ASSERT_EQ(env["placed_assets"].size(), 2u);
  EXPECT_EQ(env["placed_assets"][0]["id"].as<std::string>(), "unrelated_asset");
  EXPECT_EQ(env["placed_assets"][0]["custom"].as<std::string>(), "untouched");
  const YAML::Node shared = env["placed_assets"][1];
  EXPECT_EQ(shared["custom_existing"].as<std::string>(), "preserved");
  EXPECT_EQ(shared["pose"]["frame"].as<std::string>(), "keep_frame");
  EXPECT_EQ(YAML::Dump(shared["pose"]["xyz"]), YAML::Dump(item["pose"]["xyz"]));
  EXPECT_EQ(YAML::Dump(shared["pose"]["rpy"]), YAML::Dump(item["pose"]["rpy"]));
  EXPECT_EQ(YAML::Dump(shared["dimensions"]), YAML::Dump(item["dimensions"]));
  EXPECT_EQ(shared["source"].as<std::string>(), item["source"].as<std::string>());
  EXPECT_TRUE(shared["editable"].as<bool>());
  EXPECT_FALSE(shared["locked"].as<bool>());

  fs::remove_all(root);
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

TEST(WorkcellStudioCanvasMesh, ModernSavedLayoutDoesNotRequireLegacyOrEnvironmentFile)
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
  EXPECT_TRUE(workcell_builder::is_save_layout_workflow_ready(root));

  const auto saved = workcell_builder::inspect_saved_workcell_studio_layout(root);
  EXPECT_TRUE(saved.saved);
  EXPECT_EQ(saved.source, workcell_builder::WorkcellStudioSavedLayoutSource::Canonical);
  EXPECT_EQ(saved.relative_path, "layout/workcell_studio_layout.yaml");
  EXPECT_TRUE(saved.blocker.empty());
  EXPECT_FALSE(fs::exists(root / "environment_layout.yaml"));
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

TEST(WorkcellStudioCanvasMesh, EnsureCanonicalLayoutCreatesSchemaRootWithoutLegacyFile)
{
  const fs::path root = fs::temp_directory_path() / "wc_ensure_canonical_layout";
  fs::remove_all(root);
  fs::create_directories(root);

  const auto result = workcell_builder::ensure_canonical_workcell_studio_layout(root, "modern_scene");
  EXPECT_TRUE(result.ok) << result.error;
  EXPECT_TRUE(result.created);
  EXPECT_FALSE(result.preserved_existing);
  EXPECT_EQ(result.path, root / "layout" / "workcell_studio_layout.yaml");
  EXPECT_FALSE(fs::exists(root / "environment_layout.yaml"));

  const YAML::Node layout = load_yaml_file(result.path);
  EXPECT_EQ(layout["schema_version"].as<std::string>(), "workcell_studio_layout/v1");
  EXPECT_EQ(layout["scene_name"].as<std::string>(), "modern_scene");
  ASSERT_TRUE(layout["items"] && layout["items"].IsSequence());
  EXPECT_EQ(layout["items"].size(), 0u);

  const auto saved = workcell_builder::inspect_saved_workcell_studio_layout(root);
  EXPECT_TRUE(saved.saved);
  EXPECT_EQ(saved.source, workcell_builder::WorkcellStudioSavedLayoutSource::Canonical);

  fs::remove_all(root);
}

TEST(WorkcellStudioCanvasMesh, EnsureCanonicalLayoutNeverOverwritesExistingContent)
{
  const fs::path root = fs::temp_directory_path() / "wc_preserve_canonical_layout";
  fs::remove_all(root);
  fs::create_directories(root / "layout");
  const fs::path layout_path = root / "layout" / "workcell_studio_layout.yaml";
  const std::string authored =
    "schema_version: workcell_studio_layout/v1\n"
    "scene_name: authored_scene\n"
    "authored_marker: preserve_me\n"
    "items: [{id: authored_table, editable: true}]\n";
  write_file(layout_path, authored);

  const auto result = workcell_builder::ensure_canonical_workcell_studio_layout(root, "different_scene");
  EXPECT_TRUE(result.ok) << result.error;
  EXPECT_FALSE(result.created);
  EXPECT_TRUE(result.preserved_existing);

  std::ifstream input(layout_path.string());
  std::stringstream contents;
  contents << input.rdbuf();
  EXPECT_EQ(contents.str(), authored);
  EXPECT_FALSE(fs::exists(root / "environment_layout.yaml"));

  fs::remove_all(root);
}

TEST(WorkcellStudioCanvasMesh, LegacyLayoutRemainsAcceptedAsSavedFallback)
{
  const fs::path root = fs::temp_directory_path() / "wc_saved_layout_legacy_fallback";
  fs::remove_all(root);
  fs::create_directories(root);
  write_file(root / "environment_layout.yaml",
    "schema_version: environment_layout/v1\n"
    "placed_assets: [{id: legacy_table, type: table}]\n");

  const auto saved = workcell_builder::inspect_saved_workcell_studio_layout(root);
  EXPECT_TRUE(saved.saved);
  EXPECT_EQ(saved.source, workcell_builder::WorkcellStudioSavedLayoutSource::LegacyFallback);
  EXPECT_EQ(saved.relative_path, "environment_layout.yaml");
  EXPECT_TRUE(saved.blocker.empty());

  fs::remove_all(root);
}

TEST(WorkcellStudioCanvasMesh, SavedLayoutDetectionKeepsCanonicalPrecedence)
{
  const fs::path root = fs::temp_directory_path() / "wc_saved_layout_canonical_precedence";
  fs::remove_all(root);
  fs::create_directories(root);
  write_file(root / "layout" / "workcell_studio_layout.yaml",
    "schema_version: workcell_studio_layout/v1\nitems: [{id: canonical_table}]\n");
  write_file(root / "environment_layout.yaml",
    "schema_version: environment_layout/v1\nplaced_assets: [{id: legacy_table}]\n");
  write_file(root / "environment.yaml", "placed_objects: [{id: environment_table}]\n");

  const auto saved = workcell_builder::inspect_saved_workcell_studio_layout(root);
  EXPECT_EQ(saved.source, workcell_builder::WorkcellStudioSavedLayoutSource::Canonical);
  const auto model = workcell_builder::build_workcell_studio_canvas_model(root, "precedence_scene");
  EXPECT_EQ(model.layout_source_kind, "canonical");
  EXPECT_EQ(model.layout_source_path, (root / "layout" / "workcell_studio_layout.yaml").string());

  fs::remove_all(root);
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
  locked_item.type = "fixture";
  locked_item.category = "fixtures";
  locked_item.role = "workholding";
  locked_item.source_file = "urdf/scene.urdf.xacro";
  locked_item.mesh_path = "package://asset_pkg/meshes/locked_preview.stl";
  locked_item.has_mesh_metadata = true;
  locked_item.provenance = workcell_builder::WorkcellStudioItemProvenance::GeneratedOrLegacyPreview;
  locked_item.locked = true;
  locked_item.editable = false;
  locked_item.x = 0.1;
  locked_item.y = 0.2;
  locked_item.z = 0.3;
  locked_item.roll = 0.4;
  locked_item.pitch = 0.5;
  locked_item.yaw = 0.6;
  locked_item.width = 0.7;
  locked_item.depth = 0.8;
  locked_item.height = 0.9;
  preview.items = {locked_item};
  const auto locked_result = workcell_builder::bootstrap_editable_layout_from_scene_sources(root, "wc_bootstrap_fallbacks", preview);
  EXPECT_EQ(locked_result.source_used, "preview_model");
  EXPECT_EQ(locked_result.editable_items_created, 1u);
  EXPECT_NE(std::find(locked_result.blockers.begin(), locked_result.blockers.end(), "no cell_definition.yaml"), locked_result.blockers.end());
  EXPECT_EQ(std::find(locked_result.blockers.begin(), locked_result.blockers.end(), "preview locked-only"), locked_result.blockers.end());
  EXPECT_EQ(std::find(locked_result.blockers.begin(), locked_result.blockers.end(), "unsafe/helper/missing mesh metadata"), locked_result.blockers.end());
  ASSERT_TRUE(locked_result.layout["items"] && locked_result.layout["items"].IsSequence());
  ASSERT_EQ(locked_result.layout["items"].size(), 1u);
  const YAML::Node locked_copy = locked_result.layout["items"][0];
  EXPECT_EQ(locked_copy["id"].as<std::string>(), "locked_preview__editable_copy");
  EXPECT_NE(locked_copy["id"].as<std::string>(), "locked_preview");
  EXPECT_EQ(locked_copy["type"].as<std::string>(), "fixture");
  EXPECT_EQ(locked_copy["category"].as<std::string>(), "fixtures");
  EXPECT_EQ(locked_copy["role"].as<std::string>(), "workholding");
  EXPECT_EQ(locked_copy["pose"]["xyz"][0].as<double>(), 0.1);
  EXPECT_EQ(locked_copy["pose"]["rpy"][2].as<double>(), 0.6);
  EXPECT_EQ(locked_copy["dimensions"][2].as<double>(), 0.9);
  EXPECT_EQ(locked_copy["mesh"]["path"].as<std::string>(), "package://asset_pkg/meshes/locked_preview.stl");
  EXPECT_EQ(locked_copy["source_layer"].as<std::string>(), "editable_layout");
  EXPECT_TRUE(locked_copy["editable"].as<bool>());
  EXPECT_FALSE(locked_copy["locked"].as<bool>());
  EXPECT_EQ(locked_copy["provenance"]["original_source_id"].as<std::string>(), "locked_preview");
  EXPECT_EQ(locked_copy["provenance"]["original_source_layer"].as<std::string>(), "locked_generated_urdf_visual");
  EXPECT_TRUE(preview.items[0].locked);

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
  EXPECT_NE(std::find(unsafe_result.blockers.begin(), unsafe_result.blockers.end(), "unsafe/helper/missing mesh metadata"), unsafe_result.blockers.end());

  workcell_builder::WorkcellStudioCanvasItem safe_preview;
  safe_preview.id = "safe_preview";
  safe_preview.type = "fixture";
  safe_preview.category = "fixtures";
  safe_preview.role = "workholding";
  safe_preview.label = "Safe Preview Fixture";
  safe_preview.source_file = "urdf/scene.urdf.xacro";
  safe_preview.source_package = "generated_scene_pkg";
  safe_preview.mesh_source_package = "asset_pkg";
  safe_preview.x = 0.1;
  safe_preview.y = 0.2;
  safe_preview.z = 0.3;
  safe_preview.roll = 0.4;
  safe_preview.pitch = 0.5;
  safe_preview.yaw = 0.6;
  safe_preview.width = 0.7;
  safe_preview.depth = 0.8;
  safe_preview.height = 0.9;
  safe_preview.primitive_geometry_type = "cylinder";
  safe_preview.primitive_radius = 0.11;
  safe_preview.primitive_length = 0.22;
  safe_preview.has_mesh_metadata = true;
  safe_preview.mesh_path = "package://asset_pkg/meshes/fixture.stl";
  safe_preview.mesh_scale_x = 1.5;
  safe_preview.mesh_scale_y = 2.5;
  safe_preview.mesh_scale_z = 3.5;
  safe_preview.confidence = 0.42;
  safe_preview.tracking_id = "track-should-not-copy";
  safe_preview.snapshot_source_file = "snapshot-should-not-copy.yaml";
  preview.items = {safe_preview};
  const auto preview_result = workcell_builder::bootstrap_editable_layout_from_scene_sources(root, "wc_bootstrap_fallbacks", preview);
  EXPECT_EQ(preview_result.source_used, "preview_model");
  ASSERT_EQ(preview_result.layout["items"].size(), 1u);
  const YAML::Node copied = preview_result.layout["items"][0];
  EXPECT_EQ(copied["id"].as<std::string>(), "safe_preview__editable_copy");
  EXPECT_EQ(copied["display_name"].as<std::string>(), "Safe Preview Fixture");
  EXPECT_EQ(copied["type"].as<std::string>(), "fixture");
  EXPECT_EQ(copied["category"].as<std::string>(), "fixtures");
  EXPECT_EQ(copied["role"].as<std::string>(), "workholding");
  EXPECT_EQ(copied["pose"]["xyz"][0].as<double>(), 0.1);
  EXPECT_EQ(copied["pose"]["rpy"][2].as<double>(), 0.6);
  EXPECT_EQ(copied["dimensions"][2].as<double>(), 0.9);
  EXPECT_EQ(copied["primitive_geometry_type"].as<std::string>(), "cylinder");
  EXPECT_EQ(copied["primitive_radius"].as<double>(), 0.11);
  EXPECT_EQ(copied["primitive_length"].as<double>(), 0.22);
  EXPECT_EQ(copied["mesh"]["path"].as<std::string>(), "package://asset_pkg/meshes/fixture.stl");
  EXPECT_EQ(copied["mesh"]["source"].as<std::string>(), "urdf/scene.urdf.xacro");
  EXPECT_EQ(copied["mesh"]["source_package"].as<std::string>(), "asset_pkg");
  EXPECT_EQ(copied["mesh"]["scale"][1].as<double>(), 2.5);
  EXPECT_EQ(copied["source_package"].as<std::string>(), "generated_scene_pkg");
  EXPECT_EQ(copied["mesh_source_package"].as<std::string>(), "asset_pkg");
  EXPECT_EQ(copied["preview_source_id"].as<std::string>(), "safe_preview");
  EXPECT_EQ(copied["source_layer"].as<std::string>(), "editable_layout");
  EXPECT_TRUE(copied["editable"].as<bool>());
  EXPECT_FALSE(copied["locked"].as<bool>());
  EXPECT_EQ(copied["provenance"]["mode"].as<std::string>(), "created_from_generated_preview");
  EXPECT_EQ(copied["provenance"]["source_layer"].as<std::string>(), "locked_generated_urdf_visual");
  EXPECT_EQ(copied["provenance"]["preview_source_id"].as<std::string>(), "safe_preview");
  EXPECT_EQ(copied["provenance"]["copy_kind"].as<std::string>(), "editable_layout_copy");
  EXPECT_EQ(copied["provenance"]["original_source_id"].as<std::string>(), "safe_preview");
  EXPECT_EQ(copied["provenance"]["original_source_layer"].as<std::string>(), "locked_generated_urdf_visual");
  EXPECT_EQ(copied["provenance"]["original_source_file"].as<std::string>(), "urdf/scene.urdf.xacro");
  EXPECT_FALSE(copied["confidence"].IsDefined());
  EXPECT_FALSE(copied["tracking_id"].IsDefined());
  EXPECT_FALSE(copied["snapshot_source_file"].IsDefined());

  fs::remove_all(root);
}

TEST(WorkcellStudioCanvasModelMeshTest, DirtyAuthoringSessionSurvivesRepeatedRefreshUndoRedoAndInstances)
{
  using workcell_builder::WorkcellStudioCanvasItem;
  using workcell_builder::WorkcellStudioCanvasModel;
  using workcell_builder::WorkcellStudioItemProvenance;

  WorkcellStudioCanvasItem authored;
  authored.id = "table_01";
  authored.provenance = WorkcellStudioItemProvenance::EditableLayout;
  WorkcellStudioCanvasItem generated;
  generated.id = "robot_preview";
  generated.provenance = WorkcellStudioItemProvenance::GeneratedOrLegacyPreview;
  WorkcellStudioCanvasModel stale_disk_model;
  stale_disk_model.scene_name = "scene";

  auto stale_refresh = [&]() {
    WorkcellStudioCanvasModel model = stale_disk_model;
    model.items = {authored, generated};
    return model;
  };
  auto imported_instance = [](const std::string & id, double x) {
    WorkcellStudioCanvasItem item;
    item.id = id;
    item.label = "2068_001_24 (Imported)";
    item.source_file = "assets/imported/2068_001_24.stl";
    item.mesh_path = item.source_file;
    item.x = x;
    item.roll = 0.1;
    item.width = item.depth = item.height = 1.0;
    item.provenance = WorkcellStudioItemProvenance::EditableLayout;
    item.editable = true;
    item.locked = false;
    return item;
  };

  const auto object_01 = imported_instance("object_01", 0.25);
  std::vector<WorkcellStudioCanvasItem> session{authored, object_01};
  auto refreshed = stale_refresh();
  workcell_builder::merge_dirty_editable_layout_session(refreshed, session, {});
  ASSERT_EQ(refreshed.items.size(), 3U);
  auto find_id = [&](const WorkcellStudioCanvasModel & model, const std::string & id) {
    return std::find_if(model.items.begin(), model.items.end(), [&](const auto & item) { return item.id == id; });
  };
  auto placed = find_id(refreshed, "object_01");
  ASSERT_NE(placed, refreshed.items.end());
  EXPECT_EQ(placed->source_file, object_01.source_file);
  EXPECT_TRUE(placed->editable);
  EXPECT_FALSE(placed->locked);
  EXPECT_DOUBLE_EQ(placed->x, 0.25);
  EXPECT_DOUBLE_EQ(placed->roll, 0.1);

  // Undo removes the created instance from the canonical session; another
  // stale disk refresh cannot resurrect it. Redo restores the same identity.
  session = {authored};
  refreshed = stale_refresh();
  workcell_builder::merge_dirty_editable_layout_session(refreshed, session, {});
  EXPECT_EQ(find_id(refreshed, "object_01"), refreshed.items.end());
  session.push_back(object_01);
  refreshed = stale_refresh();
  workcell_builder::merge_dirty_editable_layout_session(refreshed, session, {});
  EXPECT_NE(find_id(refreshed, "object_01"), refreshed.items.end());

  // Catalog identity is deliberately not a reconciliation key: two scene
  // instance IDs backed by the same imported mesh remain independent.
  session.push_back(imported_instance("object_02", 0.50));
  refreshed = stale_refresh();
  workcell_builder::merge_dirty_editable_layout_session(refreshed, session, {});
  EXPECT_NE(find_id(refreshed, "object_01"), refreshed.items.end());
  EXPECT_NE(find_id(refreshed, "object_02"), refreshed.items.end());
  EXPECT_EQ(refreshed.items.size(), 4U);
}
