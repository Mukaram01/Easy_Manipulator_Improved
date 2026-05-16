#include "gui/new_cell_wizard.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QListWidget>
#include <QStackedWidget>
#include <QLineEdit>
#include <QTextEdit>
#include <QComboBox>
#include <QLabel>
#include <QPushButton>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QFrame>
#include <fstream>
#include <regex>

namespace fs = boost::filesystem;

NewCellWizard::NewCellWizard(const QString &workspace_root, QWidget *parent): QDialog(parent), workspace_root_(workspace_root) { build_ui(); refresh_validation(); }

bool NewCellWizard::is_valid_package_name(const QString &name){ static const std::regex re("^[a-z][a-z0-9_]*$"); return std::regex_match(name.toStdString(), re); }
QString NewCellWizard::default_gripper_rpy_text(){ return "-1.5708, -1.5708, 0"; }
QStringList NewCellWizard::recommended_environment_assets(){ return {"Work table", "Source bin", "Place fixture", "RealSense D435i", "Safety zone"}; }

void NewCellWizard::build_ui(){ setWindowTitle("New Cell Wizard"); resize(1000,700); auto *root=new QVBoxLayout(this); root->addWidget(new QLabel("<h2>New Cell</h2><p>Create a new robotic workcell in a guided setup flow.</p><p><b>Safety:</b> Fake hardware default / Real robot locked</p>"));
 auto *body=new QHBoxLayout(); root->addLayout(body,1); steps_=new QListWidget(this); steps_->addItems({"Basics","Robot","End Effector","Environment","Task Intent","Review & Create"}); steps_->setFixedWidth(220); body->addWidget(steps_);
 stack_=new QStackedWidget(this); body->addWidget(stack_,1);
 auto mk=[&](QWidget *w){ auto *c=new QWidget(this); auto *l=new QVBoxLayout(c); l->addWidget(w); l->addStretch(1); stack_->addWidget(c); };
 auto *s1=new QWidget(this); auto *f1=new QFormLayout(s1); display_name_=new QLineEdit; scene_name_=new QLineEdit; description_=new QTextEdit; template_=new QComboBox; template_->addItems({"Pick & Place","Sorting","Inspection","Machine Tending","Conveyor Picking","Blank Cell"}); output_path_=new QLineEdit(QString::fromStdString(scenes_root_path().string())); scene_error_=new QLabel; scene_error_->setStyleSheet("color:#b00020"); f1->addRow("Cell display name",display_name_); f1->addRow("Scene/package name",scene_name_); f1->addRow("",scene_error_); f1->addRow("Description",description_); f1->addRow("Template",template_); f1->addRow("Scenes output path",output_path_); mk(s1);
 auto *s2=new QWidget(this); auto *f2=new QFormLayout(s2); robot_=new QComboBox; robot_->addItems({"UR5","UR10","UR3","Generic Cartesian Placeholder","Generic Delta Placeholder"}); robot_warning_=new QLabel("Placeholders are scaffold only."); f2->addRow("Robot",robot_); f2->addRow("",robot_warning_); mk(s2);
 auto *s3=new QWidget(this); auto *f3=new QFormLayout(s3); ee_=new QComboBox; ee_->addItems({"robotiq_85 / Robotiq 2F","single_suction","parallel gripper","vacuum placeholder"}); ee_roll_=new QDoubleSpinBox; ee_pitch_=new QDoubleSpinBox; ee_yaw_=new QDoubleSpinBox; for (auto *b:{ee_roll_,ee_pitch_,ee_yaw_}){b->setDecimals(4);b->setRange(-6.2832,6.2832);} ee_roll_->setValue(-1.5708); ee_pitch_->setValue(-1.5708); ee_yaw_->setValue(0.0); f3->addRow("End effector",ee_); f3->addRow("Roll",ee_roll_); f3->addRow("Pitch",ee_pitch_); f3->addRow("Yaw",ee_yaw_); mk(s3);
 auto *s4=new QWidget(this); auto *v4=new QVBoxLayout(s4); env_table_=new QCheckBox("Work table / workbench"); env_source_=new QCheckBox("Source bin"); env_place_=new QCheckBox("Place fixture / target table"); env_reject_=new QCheckBox("Reject bin"); env_conveyor_=new QCheckBox("Conveyor placeholder"); env_camera_=new QCheckBox("RealSense D435i / camera"); env_safety_=new QCheckBox("Safety zone"); for(auto*c:{env_table_,env_source_,env_place_,env_reject_,env_conveyor_,env_camera_,env_safety_}) v4->addWidget(c); auto *r=new QHBoxLayout(); auto *rec=new QPushButton("Use recommended layout"); auto *clr=new QPushButton("Clear layout"); auto *pv=new QPushButton("Preview layout summary"); r->addWidget(rec);r->addWidget(clr);r->addWidget(pv); v4->addLayout(r); env_preview_=new QLabel; v4->addWidget(env_preview_); mk(s4);
 auto *s5=new QWidget(this); auto *f5=new QFormLayout(s5); task_family_=new QComboBox; task_family_->addItems({"pick_place","sorting","inspection","machine_tending","conveyor_picking"}); pick_source_=new QLineEdit; place_target_=new QLineEdit; task_warning_=new QLabel; task_warning_->setStyleSheet("color:#b36b00"); f5->addRow("Task family",task_family_); f5->addRow("Pick source",pick_source_); f5->addRow("Place target",place_target_); f5->addRow("",task_warning_); mk(s5);
 auto *s6=new QWidget(this); auto *v6=new QVBoxLayout(s6); summary_=new QLabel; summary_->setWordWrap(true); summary_->setFrameShape(QFrame::StyledPanel); v6->addWidget(summary_); mk(s6);
 auto *nav=new QHBoxLayout(); root->addLayout(nav); back_=new QPushButton("Back"); next_=new QPushButton("Next"); create_=new QPushButton("Create Cell"); create_open_=new QPushButton("Create and Open in Scene Builder"); auto *cancel=new QPushButton("Cancel"); nav->addWidget(back_); nav->addWidget(next_); nav->addStretch(1); nav->addWidget(create_); nav->addWidget(create_open_); nav->addWidget(cancel);
 connect(steps_, &QListWidget::currentRowChanged, stack_, &QStackedWidget::setCurrentIndex); connect(steps_, &QListWidget::currentRowChanged, this, &NewCellWizard::refresh_validation); steps_->setCurrentRow(0);
 connect(back_, &QPushButton::clicked, this, [this]{ steps_->setCurrentRow(std::max(0, steps_->currentRow()-1));}); connect(next_, &QPushButton::clicked, this, [this]{ steps_->setCurrentRow(std::min(5, steps_->currentRow()+1));});
 connect(cancel,&QPushButton::clicked,this,&QDialog::reject);
 auto refresh_all=[this]{refresh_validation(); refresh_summary();}; connect(scene_name_, &QLineEdit::textChanged, this, refresh_all); connect(task_family_, &QComboBox::currentTextChanged, this, refresh_all); connect(pick_source_, &QLineEdit::textChanged, this, refresh_all); connect(place_target_, &QLineEdit::textChanged, this, refresh_all);
 connect(rec,&QPushButton::clicked,this,[this]{env_table_->setChecked(true);env_source_->setChecked(true);env_place_->setChecked(true);env_reject_->setChecked(false);env_conveyor_->setChecked(false);env_camera_->setChecked(true);env_safety_->setChecked(true);refresh_summary();});
 connect(clr,&QPushButton::clicked,this,[this]{for(auto*c:{env_table_,env_source_,env_place_,env_reject_,env_conveyor_,env_camera_,env_safety_}) c->setChecked(false);refresh_summary();});
 connect(pv,&QPushButton::clicked,this,[this]{refresh_summary();});
 connect(create_, &QPushButton::clicked, this, [this]{ if(create_scene_scaffold(false)) accept(); });
 connect(create_open_, &QPushButton::clicked, this, [this]{ if(create_scene_scaffold(true)) accept(); });
}

boost::filesystem::path NewCellWizard::scenes_root_path() const { return fs::path(workspace_root_.toStdString()) / "src" / "scenes"; }
QString NewCellWizard::scene_name_error() const { const QString n=scene_name_->text().trimmed(); if(n.isEmpty()) return "Scene/package name is required."; if(!is_valid_package_name(n)) return "Use lowercase letters, digits, underscores; start with a letter."; const fs::path p=scenes_root_path()/n.toStdString(); if(fs::exists(p)) return "A scene with this name already exists."; return ""; }
void NewCellWizard::refresh_validation(){ const int row=steps_->currentRow(); back_->setEnabled(row>0); next_->setEnabled(row<5 && (row!=0 || scene_name_error().isEmpty())); scene_error_->setText(scene_name_error()); const bool valid=scene_name_error().isEmpty(); create_->setEnabled(valid); create_open_->setEnabled(valid); const bool needs_pick_place=task_family_->currentText().contains("pick")||task_family_->currentText().contains("sort"); task_warning_->setText((needs_pick_place && (pick_source_->text().trimmed().isEmpty()||place_target_->text().trimmed().isEmpty()))?"Warning: incomplete pick/place bindings; scaffold allowed.":""); refresh_summary(); }
void NewCellWizard::refresh_summary(){ QStringList env; if(env_table_->isChecked()) env<<"table"; if(env_source_->isChecked()) env<<"source_bin"; if(env_place_->isChecked()) env<<"place_fixture"; if(env_reject_->isChecked()) env<<"reject_bin"; if(env_conveyor_->isChecked()) env<<"conveyor"; if(env_camera_->isChecked()) env<<"realsense_d435i"; if(env_safety_->isChecked()) env<<"safety_zone"; env_preview_->setText("Enabled: "+env.join(", "));
 summary_->setText(QString("<b>scene name</b>: %1<br/><b>template</b>: %2<br/><b>robot</b>: %3<br/><b>end effector</b>: %4<br/><b>gripper mount RPY</b>: %5, %6, %7<br/><b>enabled environment assets</b>: %8<br/><b>task</b>: %9 (%10 → %11)<br/><b>fake hardware</b>: default / real robot locked<br/><b>output path</b>: %12")
 .arg(scene_name_->text(),template_->currentText(),robot_->currentText(),ee_->currentText()).arg(ee_roll_->text(),ee_pitch_->text(),ee_yaw_->text(),env.join(", "),task_family_->currentText(),pick_source_->text(),place_target_->text(),output_path_->text())); }
bool NewCellWizard::create_scene_scaffold(bool open_in_builder){ const QString err=scene_name_error(); if(!err.isEmpty()) return false; const fs::path scene_dir=scenes_root_path()/scene_name_->text().trimmed().toStdString(); boost::system::error_code ec; fs::create_directories(scene_dir/"config", ec); if(ec) return false; std::ofstream env((scene_dir/"environment.yaml").string()); env<<"scene_name: "<<scene_name_->text().trimmed().toStdString()<<"\n"; env<<"template: "<<template_->currentText().toStdString()<<"\n"; env<<"robot: "<<robot_->currentText().toStdString()<<"\n"; env<<"end_effector: "<<ee_->currentText().toStdString()<<"\n"; env<<"gripper_mount_rpy: ["<<ee_roll_->value()<<", "<<ee_pitch_->value()<<", "<<ee_yaw_->value()<<"]\n"; env<<"fake_hardware_first: true\n"; env<<"runtime_execution_enabled: false\n"; env.close(); result_.created=true; result_.open_in_scene_builder=open_in_builder; result_.scene_name=scene_name_->text().trimmed(); result_.scene_dir=scene_dir; return true; }
