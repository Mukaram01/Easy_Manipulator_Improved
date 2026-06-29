// Copyright 2020 Advanced Remanufacturing and Technology Centre
// Copyright 2020 ROS-Industrial Consortium Asia Pacific Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__GUI__SCENE_SELECT_H_
#define EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__GUI__SCENE_SELECT_H_

#include <QDialog>
#include <QStringList>
#include <QListWidget>
#include <QInputDialog>
#include <boost/filesystem.hpp>
#include <string>
#include <vector>

#include <map>

struct TaskGraspEditorState
{
  std::string task_type{"pick_place"};
  std::string pick_source_type{"selected_object"};
  std::string pick_source_id;
  std::string place_target_type{"selected_bin"};
  std::string place_target_id;
  std::string selected_object_id;
  std::string selected_bin_id;
  std::string selected_pick_zone_id;
  std::string selected_place_zone_id;
  std::string grasp_strategy{"finger_top"};
  std::string orientation_mode{"vertical"};
  std::string approach_axis{"z_down"};
  std::string retreat_axis{"z_up"};
  double approach_distance_m{0.12};
  double retreat_distance_m{0.10};
  double place_clearance_m{0.05};
  std::string allowed_yaw_angles_deg{"-180..180"};
  std::string allowed_roll_angles_deg{"-10..10"};
  std::string tcp_offset_xyz{"0 0 0"};
  std::string tcp_offset_rpy{"0 0 0"};
  std::string release_strategy{"open_gripper"};
  std::string tool_profile{"unknown"};
  bool requires_io{false};
  bool preview_only{false};
  bool unsaved_task_edits{false};
  std::vector<std::string> warnings;
  std::vector<std::string> blockers;
  std::map<std::string,std::string> class_routing;
};

#include "yaml-cpp/yaml.h"
#include "attributes/workcell.h"
#include "conveyor_sorting_run_console.h"
#include "conveyor_sorting_scenario_wizard.h"
#include "scene_select_paths.h"
#include "validation_dashboard_model.hpp"
#include "workcell_scene_status.hpp"
#include "offline_smoke_check_model.hpp"
#include "supported_scene_readiness_loader.hpp"

namespace Ui
{
  class SceneSelect; // NOLINT
}

class SceneSelect: public QDialog
{
  Q_OBJECT

public:
  boost::filesystem::path scenes_path;
  boost::filesystem::path templates_path;
  boost::filesystem::path workcell_path;
  boost::filesystem::path assets_path;

  Workcell workcell;
  void generate_scene_package(
    boost::filesystem::path scene_filepath, std::string scene_name,
    int ros_ver, const std::string & ros_distro);
  void generate_scene_files(Scene scene);
  bool check_yaml();
  bool check_files(bool strict = true);
  bool check_scene(bool strict = true);
  void load_workcell(Workcell workcell);
  bool load_scene_from_yaml(Scene * input_scene);
  void refresh_scenes(int latest_scene, bool scaffold_only_status = false);
  void keyPressEvent(QKeyEvent * e);

  explicit SceneSelect(QWidget * parent = nullptr);
  ~SceneSelect();

private slots:
  void on_add_scene_clicked();
  void on_delete_scene_clicked();
  void on_edit_scene_clicked();
  void on_generate_yaml_clicked();
  void on_scene_list_currentIndexChanged(int index);
  void on_generate_files_clicked();
  void on_generate_full_scene_package_start_clicked();
  void on_open_scene_folder_clicked();
  void on_back_clicked();
  void on_exit_clicked();
  void on_clear_logs_clicked();
  void on_validate_cell_clicked();
  void on_generate_canonical_files_clicked();
  void on_generate_workcell_package_clicked();
  void on_generate_studio_pack_clicked();
  void on_open_preview_clicked();
  void on_open_output_folder_clicked();
  void on_show_readiness_report_clicked();
  void on_copy_fake_hardware_launch_command_clicked();
  void refresh_preview_status();
  void export_preview_layout();
  void on_browse_scenes_folder_clicked();
  void on_refresh_scenes_button_clicked();
  void on_export_scene_bundle_clicked();
  void on_export_open_web_3d_viewer_clicked();
  void on_validate_web_edit_patch_clicked();
  void on_dry_run_web_edit_patch_clicked();
  void on_apply_web_edit_patch_clicked();
  void on_generate_validate_after_web_edit_clicked();
  void on_import_scene_bundle_clicked();
  void on_refresh_status_button_clicked();
  void on_validate_scene_button_clicked();
  void on_copy_build_command_button_clicked();
  void on_copy_launch_command_button_clicked();
  void on_create_scenario_template_clicked();
  void on_create_conveyor_sorting_live_epd_preview_clicked();
  void on_use_recommended_layout_clicked();
  void on_repair_scene_yaml_clicked();
  void on_open_conveyor_sorting_run_console_button_clicked();
  void on_template_catalog_selection_changed();
  void on_demo_one_click_button_clicked();
  void on_demo_create_scene_button_clicked();
  void on_demo_validate_button_clicked();
  void on_demo_generate_button_clicked();
  void on_demo_export_preview_button_clicked();
  void on_demo_copy_build_command_button_clicked();
  void on_demo_copy_launch_command_button_clicked();
  void on_demo_open_scene_folder_button_clicked();
  void on_demo_open_readiness_report_button_clicked();
  void on_run_offline_smoke_check_clicked();
  void on_open_smoke_report_clicked();
  void on_export_smoke_report_clicked();
  void on_copy_smoke_summary_clicked();
  void on_run_all_scenes_readiness_clicked();

private:
  enum class MessageLevel { Info, Warning, Error, Success };

  void append_message(MessageLevel level, const std::string & message);
  void append_info(const std::string & message);
  void append_warning(const std::string & message);
  void append_error(const std::string & message);
  void append_success(const std::string & message);
  void clear_messages();
  void refresh_scene_status(bool strict, const std::string & trigger);
  std::string build_workcell_readiness_report(const Scene & scene, const boost::filesystem::path & scene_dir, bool strict, bool * blocked = nullptr);
  void write_workcell_studio_summary(const Scene & scene, const boost::filesystem::path & scene_dir, const std::string & readiness_status);
  void refresh_validation_dashboard_table(const workcell_builder::ValidationDashboardResult & result);
  bool export_workcell_layout_preview(const Scene & scene, const boost::filesystem::path & scene_dir, bool open_after_export);
  bool open_existing_scene(const boost::filesystem::path & scene_dir, Scene * output_scene, std::string * status);
  bool save_scene(Scene scene, const boost::filesystem::path & scene_dir, std::string * backup_path);
  bool duplicate_scene(const boost::filesystem::path & source_scene_dir, const std::string & new_scene_name, boost::filesystem::path * duplicated_dir, std::string * reason);
  bool regenerate_scene(const boost::filesystem::path & scene_dir, const std::string & scene_name, std::string * launch_command, std::string * reason);
  void refresh_canvas_from_scene(const Scene & scene);
  bool create_scene_from_template(const std::string & template_id, const std::string & scene_name, const boost::filesystem::path & output_root, boost::filesystem::path * scene_dir);
  bool apply_recommended_layout_to_scene(const boost::filesystem::path & scene_dir, const std::string & template_id);
  bool save_new_scene_yaml(const boost::filesystem::path & scene_dir, const Scene & scene_model);
  bool validate_new_scene(const boost::filesystem::path & scene_dir);
  bool generate_full_scene_package_from_scene(const boost::filesystem::path & scene_dir);
  void update_new_scene_lifecycle_and_canvas(const boost::filesystem::path & scene_dir);
  std::string sanitize_scene_name(const std::string & raw_name) const;
  void initialize_task_grasp_editor();
  void sync_task_editor_from_model();
  void sync_task_model_from_editor();
  void apply_tool_defaults(bool force = false);
  void rerun_task_validation();
  bool assign_selected_canvas_item(const std::string & role);

  Ui::SceneSelect * ui;
  QListWidget * scenario_template_catalog_ = nullptr;
  QString selected_template_;
  QString last_status_message_;
  int current_scene_index() const;
  void initialize_template_catalog();
  void initialize_demo_mode_catalog();
  void refresh_demo_mode_catalog();
  bool run_demo_action(bool validate, bool generate);
  void initialize_asset_library();
  QString ensure_selected_template();
  boost::filesystem::path scene_dir_for_current_selection() const;
  bool validate_description_xacros(const Scene & scene, const std::string & context_label);
  bool run_web_edit_patch_workflow(bool validate_only, bool write);
  bool run_generate_validate_after_web_edit();
  bool execute_generate_validate_after_web_edit(const boost::filesystem::path & repo_root, const boost::filesystem::path & scene_dir, QString * output);
  bool execute_web_edit_patch_workflow(const boost::filesystem::path & repo_root, const boost::filesystem::path & scene_dir, const boost::filesystem::path & patch_path, bool validate_only, bool write, QString * output);
  void configure_startup_fallback_paths();
  void show_invalid_workcell_error(const std::string & error_message);
  void discover_scene_packages_on_startup();
  void update_scene_browser_status(const std::string & note = "");

  int scaffold_scene_index_ = -1;
  workcell_builder::ValidationDashboardResult latest_dashboard_result_;
  void render_workcell_studio_status(const workcell_builder::SceneStatusReport & report);
  workcell_builder::SceneStatusReport latest_scene_status_report_;
  std::string latest_demo_scene_name_;
  boost::filesystem::path latest_demo_scene_dir_;
  TaskGraspEditorState task_editor_state_;
  std::string selected_canvas_item_id_;
  std::string selected_canvas_item_type_;
  workcell_builder::OfflineSmokeCheckResult latest_offline_smoke_result_;
  std::vector<workcell_builder::SupportedSceneRegistryEntry> supported_scene_registry_;
  workcell_builder::AllScenesReadinessData all_scenes_readiness_;
};

#endif  // EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__GUI__SCENE_SELECT_H_
