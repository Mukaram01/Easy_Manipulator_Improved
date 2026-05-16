#pragma once
#include <QDialog>
#include <QStringList>
#include <boost/filesystem.hpp>
class QListWidget; class QStackedWidget; class QLineEdit; class QTextEdit; class QComboBox; class QLabel; class QPushButton; class QCheckBox; class QDoubleSpinBox;

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
private:
  void build_ui();
  void refresh_validation();
  void refresh_summary();
  bool create_scene_scaffold(bool open_in_builder);
  QString scene_name_error() const;
  boost::filesystem::path scenes_root_path() const;
  NewCellWizardResult result_;
  QString workspace_root_;
  QListWidget *steps_{nullptr}; QStackedWidget *stack_{nullptr};
  QLineEdit *display_name_{nullptr}; QLineEdit *scene_name_{nullptr}; QTextEdit *description_{nullptr}; QComboBox *template_{nullptr}; QLineEdit *output_path_{nullptr}; QLabel *scene_error_{nullptr};
  QComboBox *robot_{nullptr}; QLabel *robot_warning_{nullptr};
  QComboBox *ee_{nullptr}; QDoubleSpinBox *ee_roll_{nullptr}; QDoubleSpinBox *ee_pitch_{nullptr}; QDoubleSpinBox *ee_yaw_{nullptr};
  QCheckBox *env_table_{nullptr}; QCheckBox *env_source_{nullptr}; QCheckBox *env_place_{nullptr}; QCheckBox *env_reject_{nullptr}; QCheckBox *env_conveyor_{nullptr}; QCheckBox *env_camera_{nullptr}; QCheckBox *env_safety_{nullptr}; QLabel *env_preview_{nullptr};
  QComboBox *task_family_{nullptr}; QLineEdit *pick_source_{nullptr}; QLineEdit *place_target_{nullptr};
  QLabel *summary_{nullptr}; QLabel *task_warning_{nullptr};
  QPushButton *back_{nullptr}; QPushButton *next_{nullptr}; QPushButton *create_{nullptr}; QPushButton *create_open_{nullptr};
};
