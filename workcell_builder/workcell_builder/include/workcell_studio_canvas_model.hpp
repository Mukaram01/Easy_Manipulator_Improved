#pragma once
#include <boost/filesystem.hpp>
#include <cstddef>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace workcell_builder {
enum class WorkcellStudioItemProvenance
{
  EditableLayout,
  GeneratedOrLegacyPreview,
  StaticFallbackPreview
};

struct WorkcellStudioProvenanceStatus
{
  int editable_layout_count{0};
  int generated_or_legacy_preview_count{0};
  int static_fallback_preview_count{0};
  std::string summary;
};

struct WorkcellStudioCanvasItem {
  std::string id; std::string type; std::string category; std::string role; std::string label; std::string source_file;
  std::string catalog_asset_id;
  std::string source_package; std::string mesh_source_package;
  double x{0.0}, y{0.0}, z{0.0}, roll{0.0}, pitch{0.0}, yaw{0.0}, width{0.25}, depth{0.25}, height{0.25}, radius{0.0};
  std::string mesh_path;
  std::string mesh_type;
  std::string primitive_geometry_type;
  double primitive_radius{0.0};
  double primitive_length{0.0};
  bool has_material_color{false};
  double material_r{0.0}, material_g{0.0}, material_b{0.0}, material_a{1.0};
  std::string material_name;
  double mesh_scale_x{1.0}, mesh_scale_y{1.0}, mesh_scale_z{1.0};
  double mesh_r{0.0}, mesh_p{0.0}, mesh_y{0.0};
  bool has_mesh_metadata{false};
  bool has_origin_offset{false};
  double origin_offset_x{0.0}, origin_offset_y{0.0}, origin_offset_z{0.0};
  bool mesh_available{false};
  std::string mesh_load_warning;
  WorkcellStudioItemProvenance provenance{WorkcellStudioItemProvenance::GeneratedOrLegacyPreview};
  bool locked{false};
  bool editable{true};
  std::string camera_id;
  std::string frame_id;
  std::string detection_label;
  double confidence{-1.0};
  std::string tracking_id;
  std::string snapshot_source_file;
  std::string alignment_warning;
  std::vector<std::string> warnings;
};
struct WorkcellStudioCanvasModel {
  std::string scene_name, template_name, robot_summary, tool_summary, status;
  std::string layout_source_path, layout_source_kind, layout_load_message;
  bool fake_hardware_first{true}, no_robot_motion{true}, has_warnings{false};
  std::string pick_source, grasp_strategy, place_target, release_strategy;
  WorkcellStudioProvenanceStatus provenance_status;
  std::vector<std::string> warnings; std::vector<WorkcellStudioCanvasItem> items;
};

// Lightweight, read-only projection of the same revision-aware metadata
// snapshot used to prepare Product View. Home consumes this instead of
// maintaining a second YAML interpretation path.
struct WorkcellStudioSceneMetadataSummary
{
  std::string display_name;
  std::string robot;
  std::string tool;
  std::string task;
  std::string revision;
  bool has_parse_warning{false};
};

struct WorkcellStudioStarterLayoutSummary
{
  std::size_t total_preview_items{0};
  std::size_t skipped_locked_items{0};
  std::size_t skipped_static_fallback_items{0};
  std::size_t skipped_unsafe_or_missing_metadata_items{0};
  std::size_t editable_items_created{0};
  YAML::Node layout;
};

struct WorkcellStudioEditableLayoutBootstrapResult
{
  std::string source_used;
  std::size_t editable_items_created{0};
  std::size_t skipped_locked_items{0};
  std::size_t skipped_static_fallback_items{0};
  std::size_t skipped_unsafe_or_missing_metadata_items{0};
  std::vector<std::string> blockers;
  YAML::Node layout;
  boost::filesystem::path expected_output_dir;
  boost::filesystem::path expected_output_file;
  bool expected_output_dir_exists{false};
  bool expected_output_file_exists{false};
};

struct WorkcellStudioEditableLayoutInspection
{
  bool exists{false};
  bool valid{false};
  bool has_items_sequence{false};
  std::size_t total_item_entries{0};
  std::size_t editable_item_count{0};
};

struct WorkcellStudioEnvironmentLayoutBootstrapResult
{
  bool ok{false};
  bool wrote{false};
  bool created{false};
  std::size_t placed_assets_written{0};
  std::string error;
};

WorkcellStudioCanvasModel build_workcell_studio_canvas_model(const boost::filesystem::path & scene_dir, const std::string & scene_name);
WorkcellStudioSceneMetadataSummary load_workcell_studio_scene_metadata_summary(
  const boost::filesystem::path & scene_dir, const std::string & scene_name);
// Reconcile a disk-derived preview with the authoritative unsaved authoring
// session. Editable disk rows are replaced; generated/runtime rows remain.
void merge_dirty_editable_layout_session(
  WorkcellStudioCanvasModel & model,
  const std::vector<WorkcellStudioCanvasItem> & session_items,
  const std::vector<std::string> & deleted_item_ids);
void invalidate_workcell_studio_scene_metadata_snapshot(const boost::filesystem::path & scene_dir, const std::string & reason);
WorkcellStudioEditableLayoutInspection inspect_editable_layout_entries(const boost::filesystem::path & scene_dir);
std::size_t count_editable_layout_entries(const boost::filesystem::path & scene_dir);
bool is_save_layout_workflow_ready(const boost::filesystem::path & scene_dir);
WorkcellStudioStarterLayoutSummary bootstrap_editable_layout_from_trusted_canonical_yaml(const boost::filesystem::path & scene_dir, const std::string & scene_name);
WorkcellStudioStarterLayoutSummary build_starter_layout_entries_from_preview(const WorkcellStudioCanvasModel & model);
WorkcellStudioEditableLayoutBootstrapResult bootstrap_editable_layout_from_scene_sources(
  const boost::filesystem::path & scene_dir,
  const std::string & scene_name,
  const WorkcellStudioCanvasModel & preview_model);
WorkcellStudioEnvironmentLayoutBootstrapResult bootstrap_environment_layout_from_editable_layout(
  const boost::filesystem::path & scene_dir, const std::string & scene_name, const YAML::Node & editable_layout);
}
