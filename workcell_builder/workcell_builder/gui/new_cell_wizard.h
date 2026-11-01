#pragma once

#include <QDialog>
#include <QJsonObject>
#include <QList>
#include <QStringList>
#include <boost/filesystem.hpp>

class QListWidget;
class QStackedWidget;
class QLineEdit;
class QTextEdit;
class QComboBox;
class QLabel;
class QPushButton;
class QCheckBox;
class QDoubleSpinBox;
class QTableWidget;
class QGroupBox;
class QListWidgetItem;

struct NewCellWizardResult {
  bool created{false};
  bool open_in_scene_builder{false};
  QString scene_name;
  boost::filesystem::path scene_dir;
};

struct NewCellScenarioChoice {
  QString id;
  QString label;
  QString category;
  int order{0};
};

class NewCellWizard : public QDialog {
  Q_OBJECT
public:
  explicit NewCellWizard(const QString &workspace_root, QWidget *parent = nullptr);
  void set_output_root(const QString & path);
  NewCellWizardResult result() const { return result_; }

  static bool is_valid_package_name(const QString &name);
  static QStringList recommended_environment_assets();
  static QList<NewCellScenarioChoice> load_industrial_scenario_choices(const QString &catalog_path);
  // Object materials qualified for the physics simulator (catalog/simulation/qualified_physics.yaml).
  static QStringList load_qualified_physics_materials(const QString &catalog_path);
  QString selected_scenario_id() const;
  bool select_scenario_by_id(const QString &scenario_id);
  // Scenarios the wizard configures into a runnable cell: Pick & Place, and Sorting (its reviewed two-bin layout).
  bool configured_scenario() const;
  bool sorting_scenario() const;
  QString review_text() const;
  bool pick_place_configuration_available() const;
  // Robot and tool choices come from the capability catalog joined with the installed models and the reviewed
  // New Cell profiles (instantiate_workcell_studio_profile.py --equipment); a combination's status is
  // PLAN_CAPABLE, CONFIGURABLE, UNCOMMISSIONED or UNSUPPORTED, with its reason.
  QString selected_robot_id() const;
  QString selected_tool_id() const;
  bool select_equipment(const QString &robot_id, const QString &tool_id);
  QJsonObject equipment_combination() const;
  QStringList equipment_blockers() const;
  // Everything that keeps Create disabled, each with the wizard page that fixes it (Basics, Robot, ...).
  struct CreationBlocker { int page; QString field; QString text; };
  QList<CreationBlocker> creation_blockers() const;
  QString selected_object_source_id() const;
  bool select_object_source_by_id(const QString &source_id);
  bool manual_object_geometry_valid() const;

protected:
  void showEvent(QShowEvent *event) override;

private:
  void fit_available_screen();
  void build_ui();
  void refresh_validation();
  void refresh_summary();
  void go_to_blocker(const QString &link);
  void refresh_environment_parent_options();
  void apply_scenario_defaults();
  void refresh_scenario_ui();
  void refresh_object_source_ui();
  void add_test_object_row(const QString &id, const QString &appearance, double x, double y, double yaw_deg);
  QStringList test_object_blockers() const;
  QString scenario_catalog_path() const;
  QString qualified_physics_catalog_path() const;
  QString selected_scenario_category() const;
  QString selected_scenario_label() const;
  QStringList readiness_warnings() const;
  QStringList readiness_blockers() const;
  void apply_recommended_environment_layout();
  void refresh_environment_review_table();
  void add_environment_asset_row(const QString &id, const QString &asset, const QString &parent, const QString &parent_link, const QString &child_link, const QString &role);
  void select_environment_substep(int index);
  QStringList discover_environment_asset_catalog() const;
  void load_equipment();
  QJsonObject equipment_record(const char *kind, const QString &id) const;
  QJsonObject equipment_profile() const;
  void refresh_robot_model_options(const QString &robot_family);
  void refresh_tool_model_options(const QString &tool_family);
  // Frames, planning group, controller, tool type, release and grasp strategies of the selected pair.
  void apply_equipment_selection();
  bool create_scene_scaffold(bool open_in_builder);
  QString scene_name_error() const;
  QString scene_name_warning() const;
  boost::filesystem::path scenes_root_path() const;
  // The repository's scenes folder: the only place New Cell creates cells (Scene Library, 3D Scene Builder).
  boost::filesystem::path canonical_scenes_root() const;

  NewCellWizardResult result_;
  QString workspace_root_;
  QString recommended_profile_;
  QString recommended_profile_error_;
  QJsonObject equipment_;
  QString equipment_error_;
  QString profile_helper_path() const;

  QListWidget *steps_{nullptr};
  QStackedWidget *stack_{nullptr};

  QLineEdit *display_name_{nullptr};
  QLineEdit *scene_name_{nullptr};
  QTextEdit *description_{nullptr};
  QComboBox *application_scenario_{nullptr};
  QLineEdit *output_path_{nullptr};
  QLabel *scene_error_{nullptr};
  QLabel *scene_warning_{nullptr};

  QComboBox *robot_family_{nullptr};
  QComboBox *robot_{nullptr};
  QDoubleSpinBox *robot_x_{nullptr};
  QDoubleSpinBox *robot_y_{nullptr};
  QDoubleSpinBox *robot_z_{nullptr};
  QDoubleSpinBox *robot_roll_{nullptr};
  QDoubleSpinBox *robot_pitch_{nullptr};
  QDoubleSpinBox *robot_yaw_{nullptr};
  QLabel *robot_warning_{nullptr};
  QLabel *robot_readiness_banner_{nullptr};
  QComboBox *robot_base_link_{nullptr};
  QComboBox *robot_tip_link_{nullptr};
  QComboBox *robot_planning_group_{nullptr};
  QLineEdit *robot_controller_name_{nullptr};
  QGroupBox *robot_advanced_group_{nullptr};

  QComboBox *ee_{nullptr};
  QComboBox *ee_family_{nullptr};
  QDoubleSpinBox *ee_x_{nullptr};
  QDoubleSpinBox *ee_y_{nullptr};
  QDoubleSpinBox *ee_z_{nullptr};
  QDoubleSpinBox *ee_roll_{nullptr};
  QDoubleSpinBox *ee_pitch_{nullptr};
  QDoubleSpinBox *ee_yaw_{nullptr};
  QComboBox *ee_attach_link_{nullptr};
  QComboBox *ee_tcp_link_{nullptr};
  QComboBox *ee_type_{nullptr};
  QGroupBox *ee_advanced_group_{nullptr};
  QLabel *ee_readiness_banner_{nullptr};

  QListWidget *env_substeps_{nullptr};
  QStackedWidget *env_substep_stack_{nullptr};
  QTableWidget *env_objects_table_{nullptr};
  QTableWidget *env_review_table_{nullptr};
  QComboBox *env_add_asset_combo_{nullptr};
  QComboBox *env_parent_combo_{nullptr};
  QComboBox *env_parent_link_combo_{nullptr};
  QComboBox *env_child_link_combo_{nullptr};
  QComboBox *env_role_combo_{nullptr};
  QComboBox *env_frame_combo_{nullptr};
  QPushButton *env_edit_selected_button_{nullptr};
  QLabel *env_preview_{nullptr};

  QLabel *task_scenario_{nullptr};
  QLabel *scenario_configuration_notice_{nullptr};
  QGroupBox *pick_config_card_{nullptr};
  QGroupBox *place_config_card_{nullptr};
  QGroupBox *grasp_motion_card_{nullptr};
  QComboBox *pick_zone_source_{nullptr};
  QComboBox *pick_camera_{nullptr};
  QComboBox *object_source_mode_{nullptr};
  QLineEdit *perception_binding_{nullptr};
  QLineEdit *required_object_class_{nullptr};
  QDoubleSpinBox *minimum_confidence_{nullptr};
  QLabel *dynamic_object_notice_{nullptr};
  QGroupBox *manual_object_card_{nullptr};
  QGroupBox *test_objects_card_{nullptr};
  QTableWidget *test_objects_table_{nullptr};
  QJsonObject scenario_layout_;
  QLineEdit *manual_object_id_{nullptr};
  QComboBox *manual_object_shape_{nullptr};
  QLineEdit *manual_object_class_{nullptr};
  QComboBox *manual_object_material_{nullptr};
  QDoubleSpinBox *manual_object_x_{nullptr};
  QDoubleSpinBox *manual_object_y_{nullptr};
  QDoubleSpinBox *manual_object_z_{nullptr};
  QLineEdit *manual_object_frame_{nullptr};
  QDoubleSpinBox *manual_pose_x_{nullptr};
  QDoubleSpinBox *manual_pose_y_{nullptr};
  QDoubleSpinBox *manual_pose_z_{nullptr};
  QComboBox *pick_source_{nullptr};
  QComboBox *pick_zone_frame_{nullptr};
  QComboBox *place_target_{nullptr};
  QComboBox *place_frame_link_{nullptr};
  QComboBox *placement_mode_{nullptr};
  QComboBox *placement_alignment_{nullptr};
  QComboBox *grasp_strategy_{nullptr};
  QComboBox *approach_axis_{nullptr};
  QComboBox *release_strategy_{nullptr};
  QDoubleSpinBox *approach_distance_{nullptr};
  QDoubleSpinBox *retreat_distance_{nullptr};
  QTextEdit *task_intent_text_{nullptr};
  QLabel *task_warning_{nullptr};
  QLabel *task_readiness_label_{nullptr};

  QLabel *summary_{nullptr};
  QLabel *create_blocked_{nullptr};
  bool scene_name_edited_{false};

  QPushButton *back_{nullptr};
  QPushButton *next_{nullptr};
  QPushButton *create_{nullptr};
  QPushButton *create_open_{nullptr};
};
