#include "gui/new_cell_wizard.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QStackedWidget>
#include <QTextEdit>
#include <QVBoxLayout>

#include <fstream>
#include <regex>

namespace fs = boost::filesystem;

NewCellWizard::NewCellWizard(const QString &workspace_root, QWidget *parent)
: QDialog(parent), workspace_root_(workspace_root)
{
  build_ui();
  refresh_validation();
}

bool NewCellWizard::is_valid_package_name(const QString &name)
{
  static const std::regex re("^[a-z][a-z0-9_]*$");
  return std::regex_match(name.toStdString(), re);
}

QString NewCellWizard::default_gripper_rpy_text() { return "-1.5708, -1.5708, 0"; }

QStringList NewCellWizard::recommended_environment_assets()
{
  return {"Work table", "Source bin", "Place fixture", "RealSense D435i camera", "Safety zone"};
}

void NewCellWizard::build_ui()
{
  setWindowTitle("New Cell");
  resize(1040, 720);

  auto *root = new QVBoxLayout(this);
  root->addWidget(new QLabel(
    "<h2>New Cell</h2>"
    "<p>Create a new robotic workcell from a guided setup.</p>"
    "<p><b>Safety badge:</b> Fake hardware default / Real robot locked</p>"));

  auto *body = new QHBoxLayout();
  root->addLayout(body, 1);

  steps_ = new QListWidget(this);
  steps_->addItems({"1 Basics", "2 Robot", "3 End Effector", "4 Environment", "5 Task Intent", "6 Review"});
  steps_->setFixedWidth(220);
  body->addWidget(steps_);

  stack_ = new QStackedWidget(this);
  body->addWidget(stack_, 1);

  auto mk_page = [&](QWidget *w) {
    auto *c = new QWidget(this);
    auto *l = new QVBoxLayout(c);
    l->addWidget(w);
    l->addStretch(1);
    stack_->addWidget(c);
  };

  auto mk_pose_spin = []() {
    auto *s = new QDoubleSpinBox();
    s->setDecimals(4);
    s->setRange(-1000.0, 1000.0);
    s->setSingleStep(0.01);
    return s;
  };

  // Step 1
  auto *s1 = new QWidget(this);
  auto *f1 = new QFormLayout(s1);
  display_name_ = new QLineEdit();
  scene_name_ = new QLineEdit();
  description_ = new QTextEdit();
  template_ = new QComboBox();
  template_->addItems({"Pick & Place", "Sorting", "Inspection", "Machine Tending", "Conveyor Picking", "Blank Cell"});
  output_path_ = new QLineEdit(QString::fromStdString(scenes_root_path().string()));
  scene_error_ = new QLabel();
  scene_error_->setStyleSheet("color:#b00020");
  scene_warning_ = new QLabel();
  scene_warning_->setStyleSheet("color:#8a6d00");
  f1->addRow("Cell display name", display_name_);
  f1->addRow("Scene/package name", scene_name_);
  f1->addRow("", scene_error_);
  f1->addRow("", scene_warning_);
  f1->addRow("Description", description_);
  f1->addRow("Template", template_);
  f1->addRow("Output scenes path", output_path_);
  mk_page(s1);

  // Step 2
  auto *s2 = new QWidget(this);
  auto *f2 = new QFormLayout(s2);
  robot_ = new QComboBox();
  robot_->addItems({"UR5", "UR10", "UR3", "Generic Cartesian Placeholder", "Generic Delta Placeholder"});
  auto *robot_pose = new QWidget(this);
  auto *grid2 = new QGridLayout(robot_pose);
  robot_x_ = mk_pose_spin(); robot_y_ = mk_pose_spin(); robot_z_ = mk_pose_spin();
  robot_roll_ = mk_pose_spin(); robot_pitch_ = mk_pose_spin(); robot_yaw_ = mk_pose_spin();
  grid2->addWidget(new QLabel("x"),0,0); grid2->addWidget(robot_x_,0,1);
  grid2->addWidget(new QLabel("y"),0,2); grid2->addWidget(robot_y_,0,3);
  grid2->addWidget(new QLabel("z"),0,4); grid2->addWidget(robot_z_,0,5);
  grid2->addWidget(new QLabel("roll"),1,0); grid2->addWidget(robot_roll_,1,1);
  grid2->addWidget(new QLabel("pitch"),1,2); grid2->addWidget(robot_pitch_,1,3);
  grid2->addWidget(new QLabel("yaw"),1,4); grid2->addWidget(robot_yaw_,1,5);
  robot_warning_ = new QLabel();
  robot_warning_->setWordWrap(true);
  robot_warning_->setStyleSheet("color:#8a6d00");
  f2->addRow("Robot", robot_);
  f2->addRow("Robot base pose", robot_pose);
  f2->addRow("Hardware mode", new QLabel("Fake hardware default / Real robot locked"));
  f2->addRow("", robot_warning_);
  mk_page(s2);

  // Step 3
  auto *s3 = new QWidget(this);
  auto *f3 = new QFormLayout(s3);
  ee_ = new QComboBox();
  ee_->addItems({"robotiq_85", "single_suction", "parallel_gripper", "vacuum_array_placeholder"});
  auto *ee_pose = new QWidget(this);
  auto *grid3 = new QGridLayout(ee_pose);
  ee_x_ = mk_pose_spin(); ee_y_ = mk_pose_spin(); ee_z_ = mk_pose_spin();
  ee_roll_ = mk_pose_spin(); ee_pitch_ = mk_pose_spin(); ee_yaw_ = mk_pose_spin();
  ee_roll_->setValue(-1.5708); ee_pitch_->setValue(-1.5708); ee_yaw_->setValue(0.0);
  grid3->addWidget(new QLabel("x"),0,0); grid3->addWidget(ee_x_,0,1);
  grid3->addWidget(new QLabel("y"),0,2); grid3->addWidget(ee_y_,0,3);
  grid3->addWidget(new QLabel("z"),0,4); grid3->addWidget(ee_z_,0,5);
  grid3->addWidget(new QLabel("roll"),1,0); grid3->addWidget(ee_roll_,1,1);
  grid3->addWidget(new QLabel("pitch"),1,2); grid3->addWidget(ee_pitch_,1,3);
  grid3->addWidget(new QLabel("yaw"),1,4); grid3->addWidget(ee_yaw_,1,5);
  f3->addRow("End effector", ee_);
  f3->addRow("End effector mount pose", ee_pose);
  f3->addRow("", new QLabel("Default gripper orientation uses known working generated-scene orientation."));
  mk_page(s3);

  // Step 4
  auto *s4 = new QWidget(this);
  auto *v4 = new QVBoxLayout(s4);
  env_table_ = new QCheckBox("Work table / workbench");
  env_source_ = new QCheckBox("Source bin");
  env_place_ = new QCheckBox("Place fixture");
  env_reject_ = new QCheckBox("Reject bin");
  env_conveyor_ = new QCheckBox("Conveyor placeholder");
  env_camera_ = new QCheckBox("RealSense D435i camera");
  env_safety_ = new QCheckBox("Safety zone");
  for (auto *c : {env_table_, env_source_, env_place_, env_reject_, env_conveyor_, env_camera_, env_safety_}) v4->addWidget(c);
  auto *btns = new QHBoxLayout();
  auto *rec = new QPushButton("Use Recommended Layout");
  auto *clr = new QPushButton("Clear Layout");
  auto *pv = new QPushButton("Preview Layout Summary");
  btns->addWidget(rec); btns->addWidget(clr); btns->addWidget(pv);
  v4->addLayout(btns);
  env_preview_ = new QLabel();
  env_preview_->setWordWrap(true);
  v4->addWidget(env_preview_);
  mk_page(s4);

  // Step 5
  auto *s5 = new QWidget(this);
  auto *f5 = new QFormLayout(s5);
  task_family_ = new QComboBox();
  task_family_->addItems({"pick_place", "sorting", "inspection", "machine_tending", "conveyor_picking", "blank"});
  pick_source_ = new QLineEdit();
  place_target_ = new QLineEdit();
  grasp_strategy_ = new QComboBox();
  grasp_strategy_->addItems({"auto", "top_down_2f", "suction_top", "side_grasp_placeholder"});
  approach_distance_ = mk_pose_spin();
  retreat_distance_ = mk_pose_spin();
  approach_distance_->setValue(0.10);
  retreat_distance_->setValue(0.10);
  task_intent_text_ = new QTextEdit();
  task_warning_ = new QLabel();
  task_warning_->setWordWrap(true);
  task_warning_->setStyleSheet("color:#8a6d00");
  f5->addRow("Task family", task_family_);
  f5->addRow("Pick source", pick_source_);
  f5->addRow("Place target", place_target_);
  f5->addRow("Grasp strategy", grasp_strategy_);
  f5->addRow("Approach distance", approach_distance_);
  f5->addRow("Retreat distance", retreat_distance_);
  f5->addRow("Natural language task intent", task_intent_text_);
  f5->addRow("", task_warning_);
  mk_page(s5);

  // Step 6
  auto *s6 = new QWidget(this);
  auto *v6 = new QVBoxLayout(s6);
  summary_ = new QLabel();
  summary_->setWordWrap(true);
  summary_->setFrameShape(QFrame::StyledPanel);
  v6->addWidget(summary_);
  mk_page(s6);

  auto *nav = new QHBoxLayout();
  back_ = new QPushButton("Back");
  next_ = new QPushButton("Next");
  create_ = new QPushButton("Create Cell");
  create_open_ = new QPushButton("Create and Open");
  auto *cancel = new QPushButton("Cancel");
  nav->addWidget(back_);
  nav->addWidget(next_);
  nav->addStretch(1);
  nav->addWidget(create_);
  nav->addWidget(create_open_);
  nav->addWidget(cancel);
  root->addLayout(nav);

  connect(steps_, &QListWidget::currentRowChanged, stack_, &QStackedWidget::setCurrentIndex);
  connect(steps_, &QListWidget::currentRowChanged, this, &NewCellWizard::refresh_validation);
  steps_->setCurrentRow(0);

  connect(back_, &QPushButton::clicked, this, [this] { steps_->setCurrentRow(std::max(0, steps_->currentRow() - 1)); });
  connect(next_, &QPushButton::clicked, this, [this] { steps_->setCurrentRow(std::min(5, steps_->currentRow() + 1)); });
  connect(cancel, &QPushButton::clicked, this, &QDialog::reject);
  connect(create_, &QPushButton::clicked, this, [this] { if (create_scene_scaffold(false)) accept(); });
  connect(create_open_, &QPushButton::clicked, this, [this] { if (create_scene_scaffold(true)) accept(); });

  auto refresh_all = [this] { refresh_validation(); refresh_summary(); };
  connect(scene_name_, &QLineEdit::textChanged, this, refresh_all);
  connect(template_, &QComboBox::currentTextChanged, this, refresh_all);
  connect(robot_, &QComboBox::currentTextChanged, this, refresh_all);
  connect(task_family_, &QComboBox::currentTextChanged, this, refresh_all);
  connect(pick_source_, &QLineEdit::textChanged, this, refresh_all);
  connect(place_target_, &QLineEdit::textChanged, this, refresh_all);
  for (auto *c : {env_table_, env_source_, env_place_, env_reject_, env_conveyor_, env_camera_, env_safety_}) connect(c, &QCheckBox::toggled, this, refresh_all);

  connect(rec, &QPushButton::clicked, this, [this] {
    env_table_->setChecked(true); env_source_->setChecked(true); env_place_->setChecked(true);
    env_reject_->setChecked(false); env_conveyor_->setChecked(false); env_camera_->setChecked(true); env_safety_->setChecked(true);
    refresh_summary();
  });
  connect(clr, &QPushButton::clicked, this, [this] {
    for (auto *c : {env_table_, env_source_, env_place_, env_reject_, env_conveyor_, env_camera_, env_safety_}) c->setChecked(false);
    refresh_summary();
  });
  connect(pv, &QPushButton::clicked, this, &NewCellWizard::refresh_summary);
}

fs::path NewCellWizard::scenes_root_path() const { return fs::path(workspace_root_.toStdString()) / "src" / "scenes"; }

QString NewCellWizard::scene_name_error() const
{
  const QString n = scene_name_->text().trimmed();
  if (n.isEmpty()) return "Scene/package name is required.";
  if (!is_valid_package_name(n)) return "Use lowercase letters, numbers, underscores only; must start with a letter.";
  return "";
}

QString NewCellWizard::scene_name_warning() const
{
  const QString n = scene_name_->text().trimmed();
  if (n.isEmpty()) return "";
  const fs::path p = scenes_root_path() / n.toStdString();
  if (fs::exists(p)) return "Warning: output folder already exists. Creation is blocked to avoid overwrite.";
  return "";
}

void NewCellWizard::refresh_validation()
{
  const int row = steps_->currentRow();
  const QString err = scene_name_error();
  const QString warn = scene_name_warning();
  scene_error_->setText(err);
  scene_warning_->setText(warn);

  const bool name_valid = err.isEmpty() && warn.isEmpty();
  back_->setEnabled(row > 0);
  next_->setEnabled(row < 5 && (row != 0 || name_valid));
  create_->setEnabled(name_valid);
  create_open_->setEnabled(name_valid);

  const QString robot_txt = robot_->currentText().toLower();
  if (robot_txt.contains("placeholder")) {
    robot_warning_->setText("Preview/scaffold only: placeholder robot metadata may be incomplete.");
  } else {
    robot_warning_->setText("Known robot selected. Unknown robot metadata/config should be reviewed before runtime.");
  }

  const bool needs_pick_place = template_->currentText().contains("Pick") || template_->currentText().contains("Sorting");
  const bool missing_bindings = pick_source_->text().trimmed().isEmpty() || place_target_->text().trimmed().isEmpty();
  task_warning_->setText((needs_pick_place && missing_bindings)
    ? "Warning: pick/place template is incomplete (missing pick source or place target). Scaffold creation allowed with warning."
    : "");

  refresh_summary();
}

void NewCellWizard::refresh_summary()
{
  QStringList env;
  if (env_table_->isChecked()) env << "work_table";
  if (env_source_->isChecked()) env << "source_bin";
  if (env_place_->isChecked()) env << "place_fixture";
  if (env_reject_->isChecked()) env << "reject_bin";
  if (env_conveyor_->isChecked()) env << "conveyor_placeholder";
  if (env_camera_->isChecked()) env << "realsense_d435i_camera";
  if (env_safety_->isChecked()) env << "safety_zone";
  env_preview_->setText("Layout summary: " + (env.isEmpty() ? QString("(none)") : env.join(", ")));

  const bool has_warnings = !scene_name_warning().isEmpty() || !task_warning_->text().isEmpty() || robot_->currentText().contains("Placeholder");
  const QString readiness = scene_name_error().isEmpty() ? (has_warnings ? "scaffold / warnings" : "ready") : "invalid";

  summary_->setText(QString(
    "<b>Scene/package name:</b> %1<br/>"
    "<b>Template:</b> %2<br/>"
    "<b>Robot:</b> %3<br/>"
    "<b>End effector:</b> %4<br/>"
    "<b>End effector RPY:</b> %5, %6, %7<br/>"
    "<b>Environment assets:</b> %8<br/>"
    "<b>Task intent:</b> %9 | %10 -> %11 | %12<br/>"
    "<b>Output path:</b> %13<br/>"
    "<b>Safety mode:</b> Fake hardware default / Real robot locked<br/>"
    "<b>Expected readiness:</b> %14")
    .arg(scene_name_->text().trimmed(), template_->currentText(), robot_->currentText(), ee_->currentText(),
      ee_roll_->text(), ee_pitch_->text(), ee_yaw_->text(),
      env.isEmpty() ? QString("(none)") : env.join(", "),
      task_family_->currentText(), pick_source_->text().trimmed(), place_target_->text().trimmed(),
      task_intent_text_->toPlainText().trimmed(), output_path_->text().trimmed(), readiness));
}

bool NewCellWizard::create_scene_scaffold(bool open_in_builder)
{
  const QString err = scene_name_error();
  if (!err.isEmpty() || !scene_name_warning().isEmpty()) return false;

  const fs::path scene_dir = scenes_root_path() / scene_name_->text().trimmed().toStdString();
  boost::system::error_code ec;
  fs::create_directories(scene_dir / "config", ec);
  if (ec) return false;

  std::ofstream out((scene_dir / "environment.yaml").string());
  out << "scene_name: " << scene_name_->text().trimmed().toStdString() << "\n";
  out << "display_name: \"" << display_name_->text().trimmed().toStdString() << "\"\n";
  out << "description: \"" << description_->toPlainText().trimmed().toStdString() << "\"\n";
  out << "template: " << template_->currentText().toStdString() << "\n";
  out << "robot: " << robot_->currentText().toStdString() << "\n";
  out << "end_effector: " << ee_->currentText().toStdString() << "\n";
  out << "gripper_mount_rpy: [" << ee_roll_->value() << ", " << ee_pitch_->value() << ", " << ee_yaw_->value() << "]\n";
  out << "fake_hardware_first: true\n";
  out << "real_robot_locked: true\n";
  out << "runtime_execution_enabled: false\n";
  out << "scaffold_only: true\n";
  out.close();

  result_.created = true;
  result_.open_in_scene_builder = open_in_builder;
  result_.scene_name = scene_name_->text().trimmed();
  result_.scene_dir = scene_dir;
  return true;
}
