#pragma once

#include <QDialog>
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

class NewCellWizard : public QDialog {
  Q_OBJECT
public:
  explicit NewCellWizard(const QString &workspace_root, QWidget *parent = nullptr);
  NewCellWizardResult result() const { return result_; }

  static bool is_valid_package_name(const QString &name);
  static QString default_gripper_rpy_text();
  static QStringList recommended_environment_assets();
  static QString default_robot_base_link(const QString &robot_family, const QString &robot_model);
  static QString default_robot_tip_link(const QString &robot_family, const QString &robot_model);
  static QString default_robot_planning_group(const QString &robot_family, const QString &robot_model);
  static QString default_end_effector_attach_link(const QString &model);
  static QString default_end_effector_tcp_link(const QString &model);
  static QString default_end_effector_type(const QString &model);

private:
  void build_ui();
  void refresh_validation();
  void refresh_summary();
  void refresh_environment_parent_options();
  void apply_task_family_defaults();
  QStringList readiness_warnings() const;
  QStringList readiness_blockers() const;
  void apply_recommended_environment_layout();
  void refresh_environment_review_table();
  void add_environment_asset_row(const QString &id, const QString &asset, const QString &parent, const QString &parent_link, const QString &child_link, const QString &role);
  void select_environment_substep(int index);
  QStringList discover_environment_asset_catalog() const;
  QStringList discover_robot_models_for_family(const QString &robot_family) const;
  QStringList discover_robot_models_from_assets(const QString &robot_family) const;
  QStringList discover_robot_models_from_catalog(const QString &robot_family) const;
  QString normalize_robot_family(const QString &raw_family) const;
  void refresh_robot_model_options(const QString &robot_family);
  void refresh_robot_links_for_selection();
  bool create_scene_scaffold(bool open_in_builder);
  QString scene_name_error() const;
  QString scene_name_warning() const;
  boost::filesystem::path scenes_root_path() const;

  NewCellWizardResult result_;
  QString workspace_root_;

  QListWidget *steps_{nullptr};
  QStackedWidget *stack_{nullptr};

  QLineEdit *display_name_{nullptr};
  QLineEdit *scene_name_{nullptr};
  QTextEdit *description_{nullptr};
  QComboBox *template_{nullptr};
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

  QComboBox *task_family_{nullptr};
  QComboBox *pick_zone_source_{nullptr};
  QComboBox *pick_camera_{nullptr};
  QComboBox *pick_detection_source_{nullptr};
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

  QPushButton *back_{nullptr};
  QPushButton *next_{nullptr};
  QPushButton *create_{nullptr};
  QPushButton *create_open_{nullptr};
};
