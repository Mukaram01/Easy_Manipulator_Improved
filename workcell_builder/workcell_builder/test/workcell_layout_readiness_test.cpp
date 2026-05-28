#include <gtest/gtest.h>

#include <boost/filesystem.hpp>
#include <fstream>
#include <string>

#include "workcell_layout_readiness.hpp"

namespace fs = boost::filesystem;

namespace {

void write_file(const fs::path & path, const std::string & text)
{
  fs::create_directories(path.parent_path());
  std::ofstream out(path.string());
  out << text;
}

fs::path fresh_scene(const std::string & name)
{
  const fs::path root = fs::temp_directory_path() / name;
  fs::remove_all(root);
  fs::create_directories(root);
  return root;
}

}  // namespace

TEST(WorkcellLayoutReadiness, EmptyLayoutStaysNonReady)
{
  const fs::path root = fresh_scene("wc_layout_readiness_empty");
  write_file(root / "layout" / "workcell_studio_layout.yaml",
    "schema_version: workcell_studio_layout/v1\nitems: []\n");
  write_file(root / "environment_layout.yaml", "items: []\n");
  write_file(root / "environment.yaml", "robot: ur5\n");

  const auto state = workcell_builder::derive_layout_state_model(root, 0, true);
  EXPECT_EQ(state.state, workcell_builder::LayoutStateModel::EMPTY_LAYOUT);
  EXPECT_EQ(state.editable_item_count, 0u);
  EXPECT_FALSE(workcell_builder::save_layout_workflow_ready(root, state));
}

TEST(WorkcellLayoutReadiness, LockedOnlyLayoutStaysNonReady)
{
  const fs::path root = fresh_scene("wc_layout_readiness_locked_only");
  write_file(root / "layout" / "workcell_studio_layout.yaml",
    "schema_version: workcell_studio_layout/v1\n"
    "items:\n"
    "- id: locked_table\n"
    "  editable: true\n"
    "  locked: true\n"
    "- id: disabled_bin\n"
    "  editable: false\n"
    "  locked: false\n"
    "- id: fallback_preview\n"
    "  source_layer: primitive_fallback\n");
  write_file(root / "environment_layout.yaml", "items: []\n");
  write_file(root / "environment.yaml", "robot: ur5\n");

  const auto state = workcell_builder::derive_layout_state_model(root, 3, true);
  EXPECT_EQ(state.state, workcell_builder::LayoutStateModel::PREVIEW_ONLY_AVAILABLE);
  EXPECT_EQ(state.editable_item_count, 0u);
  EXPECT_FALSE(workcell_builder::save_layout_workflow_ready(root, state));
}

TEST(WorkcellLayoutReadiness, EditableLayoutWithMissingEnvironmentFilesStaysNonReady)
{
  const fs::path root = fresh_scene("wc_layout_readiness_missing_environment_files");
  write_file(root / "layout" / "workcell_studio_layout.yaml",
    "schema_version: workcell_studio_layout/v1\n"
    "items:\n"
    "- id: editable_table\n"
    "  editable: true\n"
    "  locked: false\n");

  const auto state = workcell_builder::derive_layout_state_model(root, 1, true);
  EXPECT_EQ(state.state, workcell_builder::LayoutStateModel::EDITABLE_LAYOUT_PRESENT);
  EXPECT_EQ(state.editable_item_count, 1u);
  EXPECT_FALSE(workcell_builder::save_layout_workflow_ready(root, state));
}

TEST(WorkcellLayoutReadiness, EditableLayoutPlusRequiredEnvironmentFilesBecomesReady)
{
  const fs::path root = fresh_scene("wc_layout_readiness_ready");
  write_file(root / "layout" / "workcell_studio_layout.yaml",
    "schema_version: workcell_studio_layout/v1\n"
    "items:\n"
    "- id: legacy_effectively_editable\n"
    "  locked: false\n"
    "- id: explicit_editable\n"
    "  editable: true\n");
  write_file(root / "environment_layout.yaml", "items: []\n");
  write_file(root / "environment.yaml", "robot: ur5\n");

  const auto state = workcell_builder::derive_layout_state_model(root, 2, true);
  EXPECT_EQ(state.state, workcell_builder::LayoutStateModel::EDITABLE_LAYOUT_PRESENT);
  EXPECT_EQ(state.editable_item_count, 2u);
  EXPECT_TRUE(workcell_builder::save_layout_workflow_ready(root, state));
}

TEST(WorkcellLayoutReadiness, InvalidLayoutYamlIsNonReady)
{
  const fs::path root = fresh_scene("wc_layout_readiness_invalid_yaml");
  write_file(root / "layout" / "workcell_studio_layout.yaml", "items: [unterminated\n");
  write_file(root / "environment_layout.yaml", "items: []\n");
  write_file(root / "environment.yaml", "robot: ur5\n");

  const auto state = workcell_builder::derive_layout_state_model(root, 1, true);
  EXPECT_EQ(state.state, workcell_builder::LayoutStateModel::INVALID_LAYOUT_YAML);
  EXPECT_EQ(state.editable_item_count, 0u);
  EXPECT_FALSE(workcell_builder::save_layout_workflow_ready(root, state));
}
