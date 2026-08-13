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

#ifndef EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__GUI__MAINWINDOW_H_
#define EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__GUI__MAINWINDOW_H_

#include <QFutureWatcher>
#include <QStackedWidget>
#include <QMainWindow>
#include <QTableWidget>
#include <QLabel>
#include <QString>
#include <QJsonObject>
#include <QMap>
#include <QSet>
#include <QLineEdit>
#include <QList>
#include <QPointF>
#include <atomic>
#include <QProcess>
#include <boost/filesystem.hpp>
#include <unordered_map>
#include <string>
#include <vector>

#include "attributes/workcell.h"
#include "workcell_studio_scene_browser.hpp"
#include "workcell_studio_layout_merge.hpp"
#include "studio_log_issue_tracker.hpp"
#include "gui/scene_preview_widget.h"

namespace fs = boost::filesystem;

namespace workcell_builder {
struct WorkcellStudioCanvasModel;
}

QT_BEGIN_NAMESPACE
namespace Ui {class MainWindow;}
QT_END_NAMESPACE
class QProgressDialog;
class QListWidget;
class QPushButton;
class QTextEdit;
class QPlainTextEdit;
class QGraphicsView;
class QGraphicsScene;
class QGraphicsItem;
class QGraphicsRectItem;
class QCheckBox;
class QPushButton;
class QDoubleSpinBox;
class QGraphicsSceneMouseEvent;
class QTreeWidget;
class QLineEdit;
class QTreeWidgetItem;
class QComboBox;
class QToolButton;
class QHBoxLayout;
class QMenu;
class QAction;
class QSplitter;
class QFrame;
class QDialog;
class QGridLayout;
class QMessageBox;
class QCloseEvent;

class MainWindow: public QMainWindow
{
  Q_OBJECT

public:
  friend class TestMain;
  boost::filesystem::path workcell_path;
  Workcell workcell;
  bool success;
  // Supported ROS 2 distributions detected at runtime.
  std::vector<std::string> ros_dist;
  bool is_good_scene(boost::filesystem::path original_path, std::string scene_name);

  explicit MainWindow(const QString & startup_workspace = QString(), const QString & startup_ros_distro = QString(), QWidget * parent = nullptr);
  ~MainWindow();
  ScenePreviewWidget * active_scene_preview_widget() const;
  bool load_scene_for_scene3d_smoke(const QString & scene_name, const QString & explicit_scene_path = QString(), QStringList * blockers = nullptr, QJsonObject * diagnostics = nullptr);
  void refresh_scene_builder_state_from_active_scene();
  QJsonObject scene3d_filter_diagnostics() const;
  static bool parse_transform_clipboard_text(
    const QString & text, double * x, double * y, double * z, double * r, double * p, double * yaw, QString * error = nullptr);

private slots:
  void on_load_workcell_clicked();

  void on_next_clicked();

  void on_change_workcell_clicked();
  void run_preview_build();
  void run_fake_hardware_preview();
  void stop_preview_process();
  void handle_preview_stdout();
  void handle_preview_stderr();
  void handle_preview_started();
  void handle_preview_error(QProcess::ProcessError error);
  void handle_preview_finished(int exit_code, QProcess::ExitStatus exit_status);

private:
  bool eventFilter(QObject * watched, QEvent * event) override;
  void closeEvent(QCloseEvent * event) override;
  enum class StudioPage : int {
    DashboardPage = 0,
    SceneBuilderPage,
    ExistingScenesPage,
    DemoModePage,
    PlanSimulatePage,
    DiagnosticsPage,
    ValidationPage,
    ExportPage
  };
  void show_studio_page(StudioPage page);
  bool open_scene_builder_for_selected_scene(const QString & source_action);
  void open_new_scene_creation_flow();
  bool has_selected_ros_distro() const;
  void update_next_button_state();
  void toggle_full_screen();
  void setup_studio_shell();
  void build_studio_header_actions();
  void apply_studio_theme();
  void append_studio_log(
    const QString & message,
    workcell_builder::StudioLogSeverity severity = workcell_builder::StudioLogSeverity::Info,
    const QString & issue_key = QString());
  bool append_scene_diagnostic_log_once(const QString & event, int payload_revision, int payload_count, const QString & message);
  bool scene3d_debug_logging_enabled() const;
  void show_not_wired_message(const QString & action_label);
  QString selected_scene_name() const;
  QString selected_scene_path() const;
  bool has_selected_scene() const;
  QString compact_scene_path_context(const QString & scene_name, const QString & full_path) const;
  void update_scene_builder_path_header(const QString & scene_name, const QString & full_path);
  struct SelectedSceneState
  {
    bool valid{ false };
    int index{ -1 };
    QString name;
    QString path;
    QString status;
    QString robot_summary;
    QString end_effector_summary;
    QString tool_mount_summary;
    QString grasp_frame_summary;
    bool launchable{ false };
  };
  void sync_selected_scene_state();
  void sync_selected_item_state();
  void refresh_selected_scene_metadata_panel();
  void refresh_scene_builder_selection_state_ui();
  void refresh_scene_builder_selected_scene_ui();
  void refresh_scene_browser_ui();
  void populate_scene_files_tab();
  void refresh_studio_home_scene_table();
  void refresh_selected_scene_details_card();
  void select_scene_by_row(int row);
  void open_selected_scene_artifact(const QString & artifact);
  void delete_selected_scene();
  bool is_safe_scene_path_for_trash_move(const boost::filesystem::path & scene_path, QString * reason = nullptr) const;
  void refresh_task_intent_panel();
  void refresh_new_cell_checklist();
  QString scene_workflow_checklist_html() const;
  enum class SceneWorkflowStepStatus { Done, Current, NeedsAction, Blocked, Warning };
  struct SceneWorkflowStep { QString label; SceneWorkflowStepStatus status{ SceneWorkflowStepStatus::NeedsAction }; QString detail; };
  std::vector<SceneWorkflowStep> scene_workflow_steps() const;
  SceneWorkflowStep compute_scene_workflow_step(
    const QString & label,
    bool ready,
    const QString & ready_detail,
    const QString & missing_detail,
    const QStringList & prerequisites,
    const QMap<QString, bool> & prerequisite_states,
    SceneWorkflowStepStatus ready_status = SceneWorkflowStepStatus::Done) const;
  QString scene_workflow_status_text(SceneWorkflowStepStatus status) const;
  QString scene_workflow_status_chip(SceneWorkflowStepStatus status) const;
  QString scene_workflow_compact_summary(const SceneWorkflowStep & step) const;
  QString scene_workflow_details_tooltip(const std::vector<SceneWorkflowStep> & steps) const;
  void refresh_scene_workflow_rail();
  QAction * scene_builder_action(const QString & key) const;
  void register_scene_builder_action(const QString & key, QAction * action);
  enum class RecommendedWorkflowActionHandler {
    OpenOrCreateScene,
    AddAsset,
    CreateEditableLayoutFromPreview,
    SaveLayout,
    GenerateYaml,
    ValidateScene,
    GenerateScenePackage,
    PlanSimulate,
    ExportBundle
  };
  struct RecommendedWorkflowAction
  {
    QString token;
    QString label;
    bool enabled{ false };
    QString blocker_reason_tooltip;
    QString explanatory_text;
    RecommendedWorkflowActionHandler handler{ RecommendedWorkflowActionHandler::OpenOrCreateScene };
  };
  RecommendedWorkflowAction resolve_recommended_workflow_action() const;
  std::vector<RecommendedWorkflowAction> resolve_recommended_workflow_actions() const;
  void trigger_recommended_workflow_action(RecommendedWorkflowActionHandler handler);
  void refresh_run_next_menu(const std::vector<RecommendedWorkflowAction> & actions);
  void validate_task_intent_for_selected_scene();
  void generate_or_update_task_intent_for_selected_scene();
  void generate_yaml_draft_for_selected_scene();
  void generate_scene_package_for_selected_scene();
  void validate_generated_scene_for_selected_scene();
  void copy_build_launch_commands_for_selected_scene();
  void open_selected_task_file();
  void copy_selected_task_summary();
  void preview_offline_plan_for_selected_scene();
  void bind_selected_item_as_pick_zone();
  void bind_selected_item_as_place_zone();
  void bind_selected_item_as_camera();
  struct SelectedSceneItemState
  {
    bool valid{ false };
    QString id;
    QString display_name;
    QString role_or_category;
    QString role;
    QString category;
    QString type;
    QString source_path;
    QString asset_id;
    QString visual_uri;
    QString tags;
    QString icon_key;
    QString source_layer;
    QString active_visual_source;
    bool editable{ true };
    bool locked{ false };
    QString lock_reason;
    bool linked_to_editable_layout_state{ false };
    QString visual_backing_status;
    bool generated_visual{ false };
    QString item_type_classification;
    QString camera_id;
    QString frame_id;
    QString detection_label;
    double confidence{ -1.0 };
    QString tracking_id;
    QString snapshot_source_file;
    QString alignment_warning;
    bool pose_available{ false };
    double pose_x{ 0.0 };
    double pose_y{ 0.0 };
    double pose_z{ 0.0 };
    double roll{ 0.0 };
    double pitch{ 0.0 };
    double yaw{ 0.0 };
    double dim_x{ 0.0 };
    double dim_y{ 0.0 };
    double dim_z{ 0.0 };
    QString pose_text;
  };
  struct EditableLayoutSelectionTarget
  {
    bool ok{ false };
    SelectedSceneItemState state;
    QString source_path;
    QString asset_id;
    QString visual_uri;
    QString tags;
    QString icon_key;
    QString blocker;
    QGraphicsItem * fallback_item{ nullptr };
  };
  bool is_pick_source_candidate(const SelectedSceneItemState & state) const;
  bool is_place_target_candidate(const SelectedSceneItemState & state) const;
  bool is_camera_candidate(const SelectedSceneItemState & state) const;
  void refresh_selection_binding_actions(const SelectedSceneItemState & state);
  SelectedSceneItemState current_selected_scene_item() const;
  void refresh_selected_scene_item_labels(const SelectedSceneItemState & state);
  QString selected_scene_binding_id() const;
  bool update_selected_scene_task_intent_binding(
    const QString & binding_label,
    const std::vector<std::string> & key_path,
    const QString & selected_id);
  bool update_selected_scene_task_intent_bindings(
    const QString & binding_label,
    const std::vector<std::vector<std::string>> & key_paths,
    const QString & selected_id);
  void refresh_after_task_binding_change(const QString & binding_label, const QString & selected_id);
  QString selected_scene_launch_command() const;
  QString selected_scene_build_command() const;
  QString selected_scene_source_command() const;
  QString selected_scene_preview_command_block() const;
  bool selected_scene_preview_ready(QStringList * blockers = nullptr) const;
  bool preview_command_is_safe(const QString & command, QStringList * blockers = nullptr) const;
  void refresh_preview_launch_ui();
  void write_preview_launch_transcript(bool ran_process, const QString & command, const QString & event, int exit_code = -1);
  QString detect_workspace_root() const;
  void apply_startup_selection();
  void set_preview_state(const QString & state);
  void rebuild_digital_twin_canvas();
  void rebuild_canvas_inspector();
  enum class CanvasInteractionMode { Select, Place, Move, Inspect };
  void set_canvas_interaction_mode(CanvasInteractionMode mode);
  QPointF snap_canvas_position(const QPointF & pos) const;
  double current_nudge_step_m(Qt::KeyboardModifiers modifiers) const;
  void refresh_minimap_card();
  void update_minimap_backend_presentation();
  void select_canvas_item(QGraphicsItem * item);
  void apply_scene_selection(const QString & id, const QString & role, bool intentional_clear = false, bool center_canvas = true);
  void mark_layout_dirty(const QString & reason);
  void capture_active_editable_layout_session();
  void merge_active_editable_layout_session(
    workcell_builder::WorkcellStudioCanvasModel * model) const;
  void save_layout_changes();
  void create_starter_layout_from_preview();
  void refresh_create_starter_layout_action();
  void revert_layout_changes();
  void on_canvas_selection_changed();
  void on_canvas_item_moved(QGraphicsItem * item, const QPointF & old_pos, const QPointF & new_pos, const QString & reason);
  void apply_inspector_pose_to_item();
  void apply_selection_transform_from_editor();
  void revert_selection_transform_editor();
  void copy_selection_transform_to_clipboard();
  void paste_selection_transform_from_clipboard();
  void refresh_selection_transform_editor_from_item(QGraphicsItem * item);
  void refresh_selection_transform_editor_from_state(const SelectedSceneItemState & state);
  EditableLayoutSelectionTarget resolve_selected_editable_layout_target() const;
  QGraphicsItem * find_canvas_item_by_stable_id(const QString & id) const;
  void refresh_robot_base_pose_inspector();
  void apply_robot_base_pose_from_inspector();
  void reset_robot_base_pose_from_snapshot();
  QGraphicsItem * find_authoring_robot_base_item() const;
  void undo_layout_edit();
  void redo_layout_edit();
  void duplicate_selected_item();
  bool selected_item_can_be_duplicated() const;
  void refresh_duplicate_selected_action();
  bool selected_item_can_be_deleted() const;
  void refresh_delete_selected_action();
  void delete_selected_item();
  void add_asset_to_canvas_from_catalog(const QString & category, const QString & display_name, const QString & source_path);
  bool place_catalog_asset_at_world_position(
    const QString & asset_id, double world_x_m, double world_y_m, double world_z_m,
    bool configure_transform);
  QPointF compute_default_canvas_pose(const QString & category, const QString & display_name) const;
  bool arm_place_asset_mode(const QString & asset_id);
  void commit_armed_asset_placement(const QPointF & canvas_pos_px);
  bool configure_asset_placement_transform(const QString & category, const QString & display_name);
  bool validate_armed_asset_transform(QString * error_message = nullptr);
  void clear_armed_asset_placement();
  void reset_armed_asset_transform_to_defaults();
  void update_arm_transform_validation_ui();
  void update_scene_builder_top_controls_overflow();
  void apply_scene_builder_panel_visibility(bool left_visible, bool right_visible, bool persist = true);
  void save_scene_builder_panel_state();
  void sync_scene_builder_view_actions();
  double default_asset_pose_z(const QString & category, const QString & display_name) const;

  struct AssetCatalogEntry
  {
    QString asset_type;
    QString display_name;
    QString role;
    QString dimensions;
    QString default_pose;
    QString source_path;
    QString asset_id;
    QString visual_uri;
    QString tags;
    QString icon_key;
    bool editable{ true };
    QString availability_status;
    QString disabled_reason;
    QString category;
    double scale{ 1.0 };
  };

  void run_layout_merge_for_selected_scene(bool from_generate_scene = false);
  void open_layout_merge_report();
  void copy_layout_merge_summary();
  void export_canvas_snapshot();
  void refresh_scene_bundle_export_panel();
  void export_scene_bundle_for_selected_scene();
  void import_scene_bundle_into_scenes_root();
  void open_scene_bundle_export_folder();
  bool selected_scene_layout_merge_ready(QStringList * blockers = nullptr) const;
  void refresh_diagnostics_quick_status();
  void refresh_scene_builder_left_explorer();
  void refresh_scene_builder_view_chips();
  void populate_scene_hierarchy();
  void apply_scene3d_product_view_layer_defaults_and_commit();
  void apply_scene3d_preview_layer_filters(bool log_change = false);
  void refresh_scene3d_product_view_status_and_audit();
  void keyPressEvent(QKeyEvent * event) override;
  void populate_asset_catalog();
  void import_stl_to_asset_library();
  void on_hierarchy_item_selected(QTreeWidgetItem * item);
  void refresh_selected_item_card();
  void on_asset_filter_changed(int index);
  void open_add_asset_dialog();
  void refresh_add_asset_dialog_details();
  void place_selected_asset_from_dialog();
  void validate_asset_catalog_selection();
  void update_asset_library_preview();
  QString selected_catalog_item_path() const;
  void run_diagnostics_self_test();
  void run_diagnostics_golden_flow_dry_run();
  void run_offline_validation();
  void run_layout_validation_only();
  void check_canvas_generated_parity();
  void open_validation_report();
  void copy_validation_summary();
  void generate_readiness_pack();
  void open_readiness_dashboard();
  void copy_diagnostics_report();
  void open_diagnostics_folder();
  bool helper_script_exists(const QString & script_name, QString * path = nullptr) const;
  QStringList helper_script_search_paths(const QString & script_name) const;
  QString resolve_scene3d_extractor_script_path(const fs::path & selected_scene_dir) const;
  QString resolve_workcell_studio_repo_root(const fs::path & selected_scene_dir) const;
  QString find_repo_root_with_extractor(const QStringList & candidate_roots) const;
  QStringList candidate_repo_roots_for_scene(const fs::path & selected_scene_dir) const;
  QString diagnostics_output_root() const;
  QString diagnostics_status_from_counts(int blocked, int warn) const;
  enum class CanvasGeneratedParityState {
    NotChecked,
    PreGenerationOk,
    PreGenerationWarnings,
    PostGenerationPassed,
    PostGenerationWarnings,
    PostGenerationBlocked
  };
  enum class CanvasGeneratedParityMode { PreGeneration, PostGeneration };
  QString canvas_generated_parity_state_text(CanvasGeneratedParityState state) const;
  void refresh_canvas_generated_parity_ui();
  bool run_canvas_generated_parity_check(
    CanvasGeneratedParityMode mode, QString * user_warning = nullptr, bool * severe_mismatch = nullptr);
  bool parse_canvas_generated_parity_report(
    const QString & report_path, int * mismatches, int * warnings, int * blockers) const;
  void append_diagnostics_row(const QString & name, const QString & status, const QString & details, const QString & fix, const QString & related_path);
  void write_diagnostics_report(const QJsonObject & report, const QString & summary, const QString & dashboard_html);
  Ui::MainWindow * ui;
  QStackedWidget * studio_pages_{ nullptr };
  QListWidget * studio_nav_{ nullptr };
  QPushButton * full_screen_button_{ nullptr };
  QTextEdit * studio_log_{ nullptr };
  QTableWidget * dashboard_scene_table_{ nullptr };
  QListWidget * dashboard_library_list_{ nullptr };
  QLineEdit * dashboard_library_search_{ nullptr };
  QComboBox * dashboard_library_status_filter_{ nullptr };
  QLineEdit * dashboard_scene_search_{ nullptr };
  QComboBox * dashboard_scene_status_filter_{ nullptr };
  QLabel * dashboard_total_scenes_card_{ nullptr };
  QLabel * dashboard_ready_scenes_card_{ nullptr };
  QLabel * dashboard_warning_scenes_card_{ nullptr };
  QLabel * dashboard_last_updated_card_{ nullptr };
  QLabel * dashboard_empty_state_title_{ nullptr };
  QLabel * dashboard_empty_state_hint_{ nullptr };
  QFrame * dashboard_empty_state_card_{ nullptr };
  QFrame * dashboard_selected_scene_card_{ nullptr };
  QLabel * dashboard_selected_scene_details_{ nullptr };
  QToolButton * dashboard_scene_actions_button_{ nullptr };
  QMenu * dashboard_scene_actions_menu_{ nullptr };
  QAction * dashboard_open_scene_action_{ nullptr };
  QAction * dashboard_validate_action_{ nullptr };
  QAction * dashboard_plan_action_{ nullptr };
  QAction * dashboard_export_action_{ nullptr };
  QAction * dashboard_delete_action_{ nullptr };
  QTableWidget * existing_scene_table_{ nullptr };
  QLabel * dashboard_summary_label_{ nullptr };
  QLabel * scene_builder_title_{ nullptr };
  QLabel * scene_builder_launch_chip_{ nullptr };
  QLabel * scene_builder_safety_chip_{ nullptr };
  QPushButton * scene_builder_generate_launch_button_{ nullptr };
  bool scene_builder_is_3d_view_{ true };
  QLabel * scene_builder_path_label_{ nullptr };
  QToolButton * scene_builder_copy_path_button_{ nullptr };
  QLabel * selection_scene_name_label_{ nullptr };
  QLabel * selection_scene_status_label_{ nullptr };
  QLabel * selection_scene_robot_label_{ nullptr };
  QLabel * selection_scene_end_effector_label_{ nullptr };
  QLabel * selection_scene_launch_label_{ nullptr };
  QLabel * inspector_label_{ nullptr };
  QDoubleSpinBox * inspector_x_{ nullptr };
  QDoubleSpinBox * inspector_y_{ nullptr };
  QDoubleSpinBox * inspector_z_{ nullptr };
  QDoubleSpinBox * inspector_roll_{ nullptr };
  QDoubleSpinBox * inspector_pitch_{ nullptr };
  QDoubleSpinBox * inspector_yaw_{ nullptr };
  QDoubleSpinBox * inspector_dim_x_{ nullptr };
  QDoubleSpinBox * inspector_dim_y_{ nullptr };
  QDoubleSpinBox * inspector_dim_z_{ nullptr };
  QLineEdit * inspector_display_name_{ nullptr };
  QLineEdit * inspector_role_{ nullptr };
  QLineEdit * inspector_category_{ nullptr };
  QLineEdit * inspector_type_{ nullptr };
  QCheckBox * inspector_live_update_box_{ nullptr };
  QPushButton * inspector_apply_button_{ nullptr };
  QPushButton * inspector_revert_button_{ nullptr };
  QPushButton * inspector_copy_transform_button_{ nullptr };
  QPushButton * inspector_paste_transform_button_{ nullptr };
  QLabel * inspector_warning_label_{ nullptr };
  QLabel * inspector_advanced_details_label_{ nullptr };
  QLabel * robot_pose_source_label_{ nullptr };
  QLabel * robot_pose_message_label_{ nullptr };
  QDoubleSpinBox * robot_base_x_{ nullptr };
  QDoubleSpinBox * robot_base_y_{ nullptr };
  QDoubleSpinBox * robot_base_z_{ nullptr };
  QDoubleSpinBox * robot_base_roll_{ nullptr };
  QDoubleSpinBox * robot_base_pitch_{ nullptr };
  QDoubleSpinBox * robot_base_yaw_{ nullptr };
  QPushButton * robot_base_apply_button_{ nullptr };
  QPushButton * robot_base_reset_button_{ nullptr };
  QLabel * live_coordinate_label_{ nullptr };
  QLabel * readiness_label_{ nullptr };
  QLabel * task_flow_label_{ nullptr };
  QLabel * new_cell_checklist_label_{ nullptr };
  QFrame * scene_builder_command_preview_card_{ nullptr };
  QLabel * scene_builder_build_command_label_{ nullptr };
  QLabel * scene_builder_launch_command_label_{ nullptr };
  QLabel * task_intent_details_label_{ nullptr };
  QLabel * pick_place_details_label_{ nullptr };
  QLabel * grasp_details_label_{ nullptr };
  QLabel * approach_retreat_details_label_{ nullptr };
  QLabel * preview_actions_label_{ nullptr };
  QLabel * mode_chip_label_{ nullptr };
  QLabel * asset_catalog_panel_label_{ nullptr };
  QLabel * scene_files_summary_label_{ nullptr };
  QLabel * scene_workflow_rail_label_{ nullptr };
  QLabel * scene_workflow_recommendation_label_{ nullptr };
  QPushButton * scene_workflow_recommendation_button_{ nullptr };
  QMenu * scene_workflow_recommendation_menu_{ nullptr };
  QTreeWidget * scene_hierarchy_tree_{ nullptr };
  QLabel * selected_item_name_label_{ nullptr };
  QLabel * selected_item_summary_label_{ nullptr };
  QLabel * selected_item_id_label_{ nullptr };
  QLabel * selected_item_reason_label_{ nullptr };
  QPushButton * scene_move_mode_button_{ nullptr };
  QPushButton * scene_rotate_mode_button_{ nullptr };
  QTreeWidget * asset_catalog_tree_{ nullptr };
  QTreeWidget * scene_files_tree_{ nullptr };
  QComboBox * asset_filter_combo_{ nullptr };
  QLineEdit * asset_library_search_{ nullptr };
  ScenePreviewWidget * asset_library_preview_{ nullptr };
  QLabel * asset_library_preview_status_{ nullptr };
  QPushButton * add_to_canvas_button_{ nullptr };
  QPushButton * add_asset_button_{ nullptr };
  QPushButton * pick_source_button_{ nullptr };
  QPushButton * place_target_button_{ nullptr };
  QPushButton * camera_button_{ nullptr };
  QLabel * canvas_legend_label_{ nullptr };
  QLabel * canvas_mode_label_{ nullptr };
  QWidget * scene_builder_top_controls_host_{ nullptr };
  QHBoxLayout * scene_builder_primary_controls_layout_{ nullptr };
  QToolButton * scene_builder_camera_view_button_{ nullptr };
  QToolButton * scene_builder_overlays_button_{ nullptr };
  QToolButton * scene_builder_canvas_more_button_{ nullptr };
  QToolButton * scene_builder_visual_modes_button_{ nullptr };
  QToolButton * scene_builder_secondary_overflow_button_{ nullptr };
  QMenu * scene_builder_secondary_overflow_menu_{ nullptr };
  QMap<QString, QAction *> scene_builder_action_registry_;
  // Unified action registry (header menus + side-panel buttons share these actions).
  // Workspace
  QAction * action_workspace_studio_home_{ nullptr };
  QAction * action_workspace_new_cell_{ nullptr };
  QAction * action_workspace_open_scene_builder_{ nullptr };
  // Layout
  QAction * action_layout_save_{ nullptr };
  QAction * action_layout_revert_{ nullptr };
  // Generate
  QAction * action_generate_package_{ nullptr };
  QAction * action_generate_yaml_{ nullptr };
  QAction * action_generate_task_intent_{ nullptr };
  // Validate
  QAction * action_validate_offline_{ nullptr };
  QAction * action_validate_generated_scene_{ nullptr };
  QAction * action_validate_open_report_{ nullptr };
  QAction * action_validate_open_readiness_{ nullptr };
  // Simulate
  QAction * action_simulate_plan_preview_{ nullptr };
  // Export
  QAction * action_export_open_page_{ nullptr };
  // View
  QAction * action_view_demo_mode_{ nullptr };
  QAction * action_view_diagnostics_page_{ nullptr };
  // Diagnostics
  QAction * action_diagnostics_run_self_test_{ nullptr };
  QAction * action_diagnostics_run_golden_flow_{ nullptr };
  QAction * action_diagnostics_copy_report_{ nullptr };
  QAction * action_diagnostics_open_folder_{ nullptr };
  QAction * action_diagnostics_copy_build_launch_commands_{ nullptr };
  QLabel * snap_step_label_{ nullptr };
  QGraphicsView * digital_twin_canvas_{ nullptr };
  ScenePreviewWidget * scene_preview_widget_{ nullptr };
  QGraphicsScene * digital_twin_scene_{ nullptr };
  QGraphicsScene * minimap_scene_{ nullptr };
  QCheckBox * toggle_grid_box_{ nullptr };
  QCheckBox * snap_to_grid_box_{ nullptr };
  QCheckBox * fine_move_mode_box_{ nullptr };
  QCheckBox * unlock_robot_base_box_{ nullptr };
  QCheckBox * toggle_labels_box_{ nullptr };
  QCheckBox * toggle_warnings_box_{ nullptr };
  QCheckBox * show_reach_overlay_box_{ nullptr };
  QCheckBox * show_camera_fov_overlay_box_{ nullptr };
  QCheckBox * show_pick_place_overlay_box_{ nullptr };
  QCheckBox * show_trajectory_overlay_box_{ nullptr };
  QCheckBox * show_minimap_box_{ nullptr };
  QCheckBox * place_mode_persistent_box_{ nullptr };
  QCheckBox * preview_layer_editable_layout_box_{ nullptr };
  QCheckBox * preview_layer_generated_urdf_visual_box_{ nullptr };
  QCheckBox * preview_layer_mesh_preview_box_{ nullptr };
  QCheckBox * preview_layer_primitive_fallback_box_{ nullptr };
  QCheckBox * preview_layer_overlays_helpers_box_{ nullptr };
  QCheckBox * preview_layer_warnings_missing_assets_box_{ nullptr };
  QVector<ScenePreviewWidget::PreviewItem> all_scene_preview_items_;
  // Canonical authored state for the active, unsaved editing session. Preview
  // rebuilds consume this snapshot; only Save Layout persists it to YAML.
  QVector<ScenePreviewWidget::PreviewItem> editable_layout_session_items_;
  QString editable_layout_session_scene_name_;
  QJsonObject scene3d_filter_diagnostics_;
  QJsonObject scene3d_visual_ingestion_diagnostics_;
  QLabel * layout_state_label_{ nullptr };
  QPushButton * undo_layout_button_{ nullptr };
  QPushButton * redo_layout_button_{ nullptr };
  QPushButton * duplicate_layout_button_{ nullptr };
  QPushButton * delete_layout_button_{ nullptr };
  QPushButton * save_layout_button_{ nullptr };
  QPushButton * create_starter_layout_button_{ nullptr };
  QGraphicsView * minimap_view_{ nullptr };
  bool minimap_requested_visible_{ true };
  QVector<AssetCatalogEntry> asset_catalog_entries_;
  QPoint catalog_drag_start_;
  QDialog * add_asset_dialog_{ nullptr };
  QTableWidget * add_asset_dialog_table_{ nullptr };
  QLabel * add_asset_dialog_details_label_{ nullptr };
  QPushButton * add_asset_dialog_place_button_{ nullptr };
  QGraphicsRectItem * ghost_preview_item_{ nullptr };
  CanvasInteractionMode canvas_mode_{ CanvasInteractionMode::Select };
  bool place_asset_armed_{ false };
  QString armed_asset_id_;
  QString armed_asset_category_;
  QString armed_asset_display_name_;
  QString armed_asset_source_path_;
  double armed_asset_scale_{ 1.0 };
  bool armed_asset_use_clicked_xy_{ true };
  QPointF armed_asset_default_xy_px_{ 0.0, 0.0 };
  bool armed_asset_transform_valid_{ true };
  QString armed_asset_transform_error_;
  double armed_asset_x_m_{ 0.0 };
  double armed_asset_y_m_{ 0.0 };
  double armed_asset_z_m_{ 0.0 };
  double armed_asset_roll_rad_{ 0.0 };
  double armed_asset_pitch_rad_{ 0.0 };
  double armed_asset_yaw_rad_{ 0.0 };
  double snap_step_m_{ 0.05 };
  bool layout_dirty_{ false };
  bool layout_saved_{ false };
  bool validation_stale_{ true };
  bool launch_artifacts_ready_{ false };
  CanvasGeneratedParityState canvas_generated_parity_state_{ CanvasGeneratedParityState::NotChecked };
  QString canvas_generated_parity_report_path_;
  int canvas_generated_parity_mismatches_{ 0 };
  int canvas_generated_parity_warnings_{ 0 };
  int canvas_generated_parity_blockers_{ 0 };
  bool inspector_update_guard_{ false };
  bool robot_base_update_guard_{ false };
  bool robot_base_snapshot_valid_{ false };
  QString robot_base_pose_source_{ "blocked" };
  double robot_base_snapshot_x_{ 0.0 };
  double robot_base_snapshot_y_{ 0.0 };
  double robot_base_snapshot_z_{ 0.0 };
  double robot_base_snapshot_roll_{ 0.0 };
  double robot_base_snapshot_pitch_{ 0.0 };
  double robot_base_snapshot_yaw_{ 0.0 };
  bool selection_update_guard_{ false };
  SelectedSceneState selected_scene_state_;
  SelectedSceneItemState selected_item_state_;
  QString current_selected_scene_item_id_;
  struct CanvasEditCommand
  {
    CanvasEditCommand(
      const QString & kind,
      const QString & item_id,
      const QPointF & old_pos,
      const QPointF & new_pos,
      bool created,
      bool deleted,
      const QVector<ScenePreviewWidget::PreviewItem> & preview_items)
    : kind(kind),
      item_id(item_id),
      old_pos(old_pos),
      new_pos(new_pos),
      created(created),
      deleted(deleted),
      preview_items(preview_items)
    {
    }

    QString kind;
    QString item_id;
    QPointF old_pos;
    QPointF new_pos;
    bool created;
    bool deleted;
    QVector<ScenePreviewWidget::PreviewItem> preview_items;
  };
  QSet<QString> deleted_layout_item_ids_;
  std::vector<CanvasEditCommand> undo_stack_;
  std::vector<CanvasEditCommand> redo_stack_;
  QLabel * preview_scene_label_{ nullptr };
  QTableWidget * diagnostics_table_{ nullptr };
  QLabel * diagnostics_status_label_{ nullptr };
  QLabel * diagnostics_summary_label_{ nullptr };
  QLabel * diagnostics_indicator_label_{ nullptr };
  QPushButton * run_self_test_button_{ nullptr };
  QPushButton * run_golden_flow_button_{ nullptr };
  QPushButton * copy_diagnostics_report_button_{ nullptr };
  QPushButton * open_diagnostics_report_button_{ nullptr };
  QLabel * preview_status_label_{ nullptr };
  QLabel * preview_safety_label_{ nullptr };
  QTextEdit * preview_commands_{ nullptr };
  QPlainTextEdit * preview_log_{ nullptr };
  QPushButton * run_preview_button_{ nullptr };
  QPushButton * run_build_button_{ nullptr };
  QPushButton * stop_preview_button_{ nullptr };
  QPushButton * copy_build_button_{ nullptr };
  QPushButton * copy_source_button_{ nullptr };
  QPushButton * copy_launch_button_{ nullptr };
  QPushButton * copy_all_button_{ nullptr };
  QPushButton * open_preview_folder_button_{ nullptr };
  QPushButton * open_preview_transcript_button_{ nullptr };
  QToolButton * scene_builder_more_actions_button_{ nullptr };
  QSplitter * scene_builder_splitter_{ nullptr };
  QSplitter * scene_builder_center_splitter_{ nullptr };
  QFrame * scene_builder_left_panel_{ nullptr };
  QFrame * scene_builder_right_panel_{ nullptr };
  QAction * scene_builder_show_left_panel_action_{ nullptr };
  QAction * scene_builder_show_right_panel_action_{ nullptr };
  QAction * scene_builder_focus_3d_action_{ nullptr };
  QList<int> scene_builder_last_splitter_sizes_{320, 900, 360};
  int scene_builder_preferred_left_width_{320};
  int scene_builder_preferred_right_width_{360};
  bool scene_builder_panel_state_syncing_{false};
  bool scene_builder_focus_3d_active_{ false };
  bool scene_builder_focus_restore_left_visible_{ true };
  bool scene_builder_focus_restore_right_visible_{ true };
  QTabWidget * scene_builder_left_tabs_{ nullptr };
  QTabWidget * scene_builder_inspector_tabs_{ nullptr };
  QPushButton * scene_builder_log_toggle_button_{ nullptr };
  QFrame * scene_builder_log_panel_{ nullptr };
  QLabel * scene_builder_status_message_label_{ nullptr };
  QLabel * scene_builder_issue_count_label_{ nullptr };
  workcell_builder::StudioLogIssueTracker studio_log_issue_tracker_;
  bool scene_builder_log_collapsed_{ false };
  QToolButton * preview_more_actions_button_{ nullptr };
  QToolButton * validation_more_actions_button_{ nullptr };
  QToolButton * export_more_actions_button_{ nullptr };
  QLabel * validation_summary_label_{ nullptr };
  QLabel * validation_blockers_label_{ nullptr };
  QLabel * validation_warnings_label_{ nullptr };
  QLabel * validation_required_files_label_{ nullptr };
  QLabel * validation_task_intent_status_label_{ nullptr };
  QLabel * validation_scene_package_status_label_{ nullptr };
  QLabel * validation_next_fix_label_{ nullptr };
  QLabel * scene_bundle_selected_scene_label_{ nullptr };
  QLabel * scene_bundle_destination_label_{ nullptr };
  QLabel * scene_bundle_contents_label_{ nullptr };
  QString last_scene_bundle_export_folder_;
  QProcess * preview_process_{ nullptr };
  QString preview_state_{ "IDLE" };
  QString active_preview_command_;
  QString preview_running_scene_key_;
  workcell_builder::WorkcellStudioSceneInfo active_preview_scene_;
  QString active_preview_workspace_root_;
  QString preview_output_tail_;
  bool preview_stop_requested_{ false };
  bool close_after_preview_stop_{ false };
  workcell_builder::WorkcellStudioSceneBrowserResult scene_browser_result_;
  int selected_scene_index_{ -1 };
  struct WorkcellLoadResult
  {
    bool success{ false };
    bool cancelled{ false };
    QString error;
    Workcell workcell;
    boost::filesystem::path workcell_path;
    QString workcell_file;
    QString workcell_root_label;
  };
  QFutureWatcher<WorkcellLoadResult> * load_watcher_{ nullptr };
  QProgressDialog * progress_dialog_{ nullptr };
  std::atomic<bool> cancel_requested_{ false };
  QString startup_workspace_;
  QString startup_ros_distro_;
  QString selected_workspace_;
  QString last_perception_summary_log_;
  QString last_camera_summary_log_;
  QString last_preview_summary_log_;
  QString last_layout_load_message_log_;
  QSet<QString> emitted_scene_diagnostic_log_keys_;
  int scene_diagnostic_payload_revision_{ 0 };
  QString visual_index_script_missing_reported_scene_key_;
  QString visual_index_regen_failure_reported_scene_key_;
  bool visual_index_regen_throttle_session_active_{ false };
  int editable_layout_item_count_{ 0 };
  int preview_fallback_item_count_{ 0 };
  QString preview_provenance_summary_;
  bool scene3d_clean_product_view_{ false };
  QStringList preview_warning_details_;
  QStringList readiness_warning_details_;
};
#endif  // EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__GUI__MAINWINDOW_H_
