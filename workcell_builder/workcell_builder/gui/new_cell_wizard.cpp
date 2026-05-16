#include "gui/new_cell_wizard.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QStackedWidget>
#include <QTableWidget>
#include <QTextEdit>
#include <QVBoxLayout>

#include <fstream>
#include <regex>

namespace fs = boost::filesystem;

NewCellWizard::NewCellWizard(const QString &workspace_root, QWidget *parent): QDialog(parent), workspace_root_(workspace_root){ build_ui(); refresh_validation(); }

bool NewCellWizard::is_valid_package_name(const QString &name){ static const std::regex re("^[a-z][a-z0-9_]*$"); return std::regex_match(name.toStdString(), re);} 
QString NewCellWizard::default_gripper_rpy_text(){ return "-1.5708, -1.5708, 0"; }
QStringList NewCellWizard::recommended_environment_assets(){ return {"workbench_01","source_bin_01","place_fixture_01","camera_01","safety_zone_01"}; }
QString NewCellWizard::default_robot_base_link(const QString &){ return "base_link"; }
QString NewCellWizard::default_robot_tip_link(const QString &r){ return r.toLower().contains("placeholder")?"tool0_placeholder":"ee_link"; }
QString NewCellWizard::default_robot_planning_group(const QString &r){ return r.toLower().contains("placeholder")?"preview_group":"manipulator"; }
QString NewCellWizard::default_end_effector_attach_link(const QString &m){ return m=="robotiq_85"?"gripper_base_link":"tool_mount_link"; }
QString NewCellWizard::default_end_effector_tcp_link(const QString &m){ return m=="single_suction"?"suction_tcp":"tool0"; }
QString NewCellWizard::default_end_effector_type(const QString &m){ if(m=="robotiq_85") return "finger"; if(m=="single_suction") return "suction"; if(m.contains("vacuum")) return "vacuum_array"; return "placeholder"; }

void NewCellWizard::build_ui(){
 setWindowTitle("New Cell"); resize(1040,720); auto *root=new QVBoxLayout(this);
 root->addWidget(new QLabel("<h2>New Cell</h2><p><b>Safety badge:</b> Fake hardware default / Real robot locked</p>"));
 auto *body=new QHBoxLayout(); root->addLayout(body,1);
 steps_=new QListWidget(this); steps_->addItems({"1 Basics","2 Robot","3 End Effector","4 Environment","5 Task Intent","6 Review"}); steps_->setFixedWidth(220); body->addWidget(steps_);
 stack_=new QStackedWidget(this); body->addWidget(stack_,1);
 auto mk_page=[&](QWidget*w){auto*c=new QWidget(this); auto*l=new QVBoxLayout(c); l->addWidget(w); l->addStretch(1); stack_->addWidget(c);};
 auto mk_pose_spin=[](){auto*s=new QDoubleSpinBox(); s->setDecimals(4); s->setRange(-1000,1000); return s;};

 auto *s1=new QWidget(this); auto *f1=new QFormLayout(s1); display_name_=new QLineEdit(); scene_name_=new QLineEdit(); description_=new QTextEdit(); template_=new QComboBox(); template_->addItems({"Pick & Place","Sorting","Inspection","Machine Tending","Conveyor Picking","Blank Cell"}); output_path_=new QLineEdit(QString::fromStdString(scenes_root_path().string())); scene_error_=new QLabel(); scene_warning_=new QLabel(); f1->addRow("Cell display name",display_name_); f1->addRow("Scene/package name",scene_name_); f1->addRow("",scene_error_); f1->addRow("",scene_warning_); f1->addRow("Description",description_); f1->addRow("Template",template_); f1->addRow("Output scenes path",output_path_); mk_page(s1);

 auto *s2=new QWidget(this); auto *f2=new QFormLayout(s2); robot_=new QComboBox(); robot_->addItems({"UR5","UR10","UR3","Generic Cartesian Placeholder"}); robot_base_link_=new QLineEdit(); robot_tip_link_=new QLineEdit(); robot_planning_group_=new QLineEdit(); robot_controller_name_=new QLineEdit("scaled_joint_trajectory_controller"); robot_controller_name_->setReadOnly(true);
 auto *robot_pose=new QWidget(this); auto*grid2=new QGridLayout(robot_pose); robot_x_=mk_pose_spin(); robot_y_=mk_pose_spin(); robot_z_=mk_pose_spin(); robot_roll_=mk_pose_spin(); robot_pitch_=mk_pose_spin(); robot_yaw_=mk_pose_spin(); grid2->addWidget(robot_x_,0,1); grid2->addWidget(robot_y_,0,3); grid2->addWidget(robot_z_,0,5); grid2->addWidget(robot_roll_,1,1); grid2->addWidget(robot_pitch_,1,3); grid2->addWidget(robot_yaw_,1,5);
 robot_advanced_group_=new QGroupBox("Advanced Robot Frames"); robot_advanced_group_->setCheckable(true); robot_advanced_group_->setChecked(false); auto*raf=new QFormLayout(robot_advanced_group_); raf->addRow("Robot base link",robot_base_link_); raf->addRow("Robot end-effector/tip link",robot_tip_link_); raf->addRow("Planning group",robot_planning_group_); raf->addRow("Controller name",robot_controller_name_);
 robot_warning_=new QLabel(); robot_warning_->setWordWrap(true);
 f2->addRow("Robot",robot_); f2->addRow("Robot base pose",robot_pose); f2->addRow(robot_advanced_group_); f2->addRow("",robot_warning_); mk_page(s2);

 auto *s3=new QWidget(this); auto *f3=new QFormLayout(s3); ee_=new QComboBox(); ee_->addItems({"robotiq_85","single_suction","parallel_gripper","vacuum_array_placeholder"}); ee_attach_link_=new QLineEdit(); ee_tcp_link_=new QLineEdit(); ee_type_=new QComboBox(); ee_type_->addItems({"finger","suction","vacuum_array","placeholder"});
 auto *ee_pose=new QWidget(this); auto *grid3=new QGridLayout(ee_pose); ee_x_=mk_pose_spin(); ee_y_=mk_pose_spin(); ee_z_=mk_pose_spin(); ee_roll_=mk_pose_spin(); ee_pitch_=mk_pose_spin(); ee_yaw_=mk_pose_spin(); ee_roll_->setValue(-1.5708); ee_pitch_->setValue(-1.5708); grid3->addWidget(ee_x_,0,1); grid3->addWidget(ee_y_,0,3); grid3->addWidget(ee_z_,0,5); grid3->addWidget(ee_roll_,1,1); grid3->addWidget(ee_pitch_,1,3); grid3->addWidget(ee_yaw_,1,5);
 ee_advanced_group_=new QGroupBox("Advanced Tool Frames"); ee_advanced_group_->setCheckable(true); ee_advanced_group_->setChecked(false); auto*eaf=new QFormLayout(ee_advanced_group_); eaf->addRow("End-effector attach/base link",ee_attach_link_); eaf->addRow("End-effector TCP/tool link",ee_tcp_link_); eaf->addRow("End-effector type",ee_type_);
 f3->addRow("End effector",ee_); f3->addRow("End effector mount pose",ee_pose); f3->addRow(ee_advanced_group_); mk_page(s3);

 auto *s4=new QWidget(this); auto *v4=new QVBoxLayout(s4); v4->addWidget(new QLabel("Parent/child links define how each object is attached in the URDF tree. Use defaults unless you know the object link names. World is the root parent."));
 env_objects_table_=new QTableWidget(0,14,this); env_objects_table_->setHorizontalHeaderLabels({"Enabled","Object ID","Asset Type","Parent Object","Parent Link","Child Link","Joint Type","X","Y","Z","Roll","Pitch","Yaw","Semantic Role"}); env_objects_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch); v4->addWidget(env_objects_table_);
 auto*rec=new QPushButton("Use Recommended Layout"); v4->addWidget(rec); env_preview_=new QLabel(); v4->addWidget(env_preview_); mk_page(s4);

 auto *s5=new QWidget(this); auto *f5=new QFormLayout(s5); task_family_=new QComboBox(); task_family_->addItems({"pick_place","sorting","inspection","machine_tending","conveyor_picking","blank"}); pick_zone_source_=new QComboBox(); pick_zone_source_->addItems({"Camera view zone","Source bin","Conveyor tracking zone","Manual zone"}); pick_camera_=new QComboBox(); pick_detection_source_=new QComboBox(); pick_detection_source_->addItems({"EPD/RealSense","PointCloud2 direct","Simulated/manual"}); pick_source_=new QComboBox(); pick_zone_frame_=new QLineEdit("camera_01_frame"); place_target_=new QComboBox(); place_frame_link_=new QComboBox(); placement_mode_=new QComboBox(); placement_mode_->addItems({"top_of_surface","inside_bin","fixture_alignment","conveyor_dropoff","manual_pose"}); placement_alignment_=new QComboBox(); placement_alignment_->addItems({"none","feature_based","fixed_yaw","match_object_orientation"});
 grasp_strategy_=new QComboBox(); grasp_strategy_->addItems({"auto","top_down_2f","suction_top"}); approach_distance_=mk_pose_spin(); retreat_distance_=mk_pose_spin(); approach_distance_->setValue(0.10); retreat_distance_->setValue(0.10); task_intent_text_=new QTextEdit(); task_warning_=new QLabel(); task_warning_->setWordWrap(true);
 f5->addRow("Task family",task_family_); f5->addRow("Pick zone source",pick_zone_source_); f5->addRow("Camera",pick_camera_); f5->addRow("Detection source",pick_detection_source_); f5->addRow("Pick object source",pick_source_); f5->addRow("Zone frame",pick_zone_frame_); f5->addRow(new QLabel("Pick zone is defined by the selected camera view. Source bin/conveyor defines where objects are expected, but perception comes from the camera zone.")); f5->addRow("Place target",place_target_); f5->addRow("Place frame/link",place_frame_link_); f5->addRow("Placement mode",placement_mode_); f5->addRow("Alignment",placement_alignment_); f5->addRow("Grasp strategy",grasp_strategy_); f5->addRow("Approach distance",approach_distance_); f5->addRow("Retreat distance",retreat_distance_); f5->addRow("Task intent",task_intent_text_); f5->addRow("",task_warning_); mk_page(s5);

 auto *s6=new QWidget(this); auto *v6=new QVBoxLayout(s6); summary_=new QLabel(); summary_->setWordWrap(true); summary_->setFrameShape(QFrame::StyledPanel); v6->addWidget(summary_); mk_page(s6);

 auto*nav=new QHBoxLayout(); back_=new QPushButton("Back"); next_=new QPushButton("Next"); create_=new QPushButton("Create Cell"); create_open_=new QPushButton("Create and Open"); auto*cancel=new QPushButton("Cancel"); nav->addWidget(back_); nav->addWidget(next_); nav->addStretch(1); nav->addWidget(create_); nav->addWidget(create_open_); nav->addWidget(cancel); root->addLayout(nav);
 connect(steps_,&QListWidget::currentRowChanged,stack_,&QStackedWidget::setCurrentIndex); connect(steps_,&QListWidget::currentRowChanged,this,&NewCellWizard::refresh_validation); steps_->setCurrentRow(0);
 connect(back_,&QPushButton::clicked,this,[this]{steps_->setCurrentRow(std::max(0,steps_->currentRow()-1));}); connect(next_,&QPushButton::clicked,this,[this]{steps_->setCurrentRow(std::min(5,steps_->currentRow()+1));}); connect(cancel,&QPushButton::clicked,this,&QDialog::reject);
 connect(create_,&QPushButton::clicked,this,[this]{if(create_scene_scaffold(false))accept();}); connect(create_open_,&QPushButton::clicked,this,[this]{if(create_scene_scaffold(true))accept();});
 connect(rec,&QPushButton::clicked,this,&NewCellWizard::apply_recommended_environment_layout);
 connect(robot_,&QComboBox::currentTextChanged,this,[this](const QString&r){robot_base_link_->setText(default_robot_base_link(r)); robot_tip_link_->setText(default_robot_tip_link(r)); robot_planning_group_->setText(default_robot_planning_group(r)); refresh_validation();});
 connect(ee_,&QComboBox::currentTextChanged,this,[this](const QString&m){ee_attach_link_->setText(default_end_effector_attach_link(m)); ee_tcp_link_->setText(default_end_effector_tcp_link(m)); ee_type_->setCurrentText(default_end_effector_type(m)); if(m=="robotiq_85"||m=="single_suction"){ee_roll_->setValue(-1.5708);ee_pitch_->setValue(-1.5708);ee_yaw_->setValue(0);} refresh_validation();});
 robot_->setCurrentText("UR5"); ee_->setCurrentText("robotiq_85"); apply_recommended_environment_layout();
}

void NewCellWizard::apply_recommended_environment_layout(){ env_objects_table_->setRowCount(0); auto add=[this](QString id,QString asset,QString parent,QString plink,QString clink,QString role){int r=env_objects_table_->rowCount(); env_objects_table_->insertRow(r); env_objects_table_->setCellWidget(r,0,new QCheckBox()); static_cast<QCheckBox*>(env_objects_table_->cellWidget(r,0))->setChecked(true); env_objects_table_->setItem(r,1,new QTableWidgetItem(id)); env_objects_table_->setItem(r,2,new QTableWidgetItem(asset)); env_objects_table_->setItem(r,3,new QTableWidgetItem(parent)); env_objects_table_->setItem(r,4,new QTableWidgetItem(plink)); env_objects_table_->setItem(r,5,new QTableWidgetItem(clink)); env_objects_table_->setItem(r,6,new QTableWidgetItem("fixed")); for(int c=7;c<=12;++c) env_objects_table_->setItem(r,c,new QTableWidgetItem("0")); env_objects_table_->setItem(r,13,new QTableWidgetItem(role));};
 add("workbench_01","workbench/table","world","world","table_link","support_surface"); add("source_bin_01","bin","workbench_01","table_link","bin_link","pick_source"); add("place_fixture_01","fixture/table","workbench_01","table_link","fixture_link","place_target"); add("reject_bin_01","bin","workbench_01","table_link","bin_link","reject_target"); add("conveyor_01","conveyor","world","world","conveyor_link","conveyor/source_line"); add("camera_01","RealSense D435i","workbench_01","table_link","camera_link","camera_view_zone"); add("safety_zone_01","safety_zone","world","world","safety_zone_link","safety_guard"); refresh_environment_parent_options(); refresh_summary(); }

void NewCellWizard::refresh_environment_parent_options(){ pick_camera_->clear(); pick_source_->clear(); place_target_->clear(); place_frame_link_->clear(); for(int r=0;r<env_objects_table_->rowCount();++r){auto*cb=qobject_cast<QCheckBox*>(env_objects_table_->cellWidget(r,0)); if(!cb||!cb->isChecked()) continue; auto role=env_objects_table_->item(r,13)->text(); auto id=env_objects_table_->item(r,1)->text(); if(role.contains("camera_view_zone")) pick_camera_->addItem(id); if(role.contains("pick_source")||role.contains("conveyor")) pick_source_->addItem(id); if(role.contains("place_target")||role.contains("reject_target")||role.contains("table")||role.contains("conveyor")) place_target_->addItem(id);} pick_source_->addItem("manual"); if(pick_camera_->findText("camera_01")>=0) pick_camera_->setCurrentText("camera_01"); if(pick_camera_->count()>0) pick_zone_source_->setCurrentText("Camera view zone"); }

fs::path NewCellWizard::scenes_root_path() const { return fs::path(workspace_root_.toStdString()) / "src" / "scenes"; }
QString NewCellWizard::scene_name_error() const { const QString n=scene_name_->text().trimmed(); if(n.isEmpty()) return "Scene/package name is required."; if(!is_valid_package_name(n)) return "Use lowercase letters, numbers, underscores only; must start with a letter."; return ""; }
QString NewCellWizard::scene_name_warning() const { const QString n=scene_name_->text().trimmed(); if(n.isEmpty()) return ""; const fs::path p=scenes_root_path()/n.toStdString(); if(fs::exists(p)) return "Warning: output folder already exists. Creation is blocked to avoid overwrite."; return ""; }

void NewCellWizard::refresh_validation(){ const int row=steps_->currentRow(); const QString err=scene_name_error(); scene_error_->setText(err); scene_warning_->setText(scene_name_warning()); const bool name_valid=err.isEmpty()&&scene_name_warning().isEmpty(); back_->setEnabled(row>0); next_->setEnabled(row<5&&(row!=0||name_valid)); create_->setEnabled(name_valid); create_open_->setEnabled(name_valid); refresh_environment_parent_options(); bool unknown=robot_tip_link_->text().trimmed().isEmpty()||ee_attach_link_->text().trimmed().isEmpty()||ee_tcp_link_->text().trimmed().isEmpty(); task_warning_->setText((place_target_->currentText().isEmpty())?"Warning: place target missing; READY blocked but scaffold creation allowed.":(unknown?"Warning: unknown links/default editable text used.":"")); refresh_summary(); }

void NewCellWizard::refresh_summary(){ QString readiness="READY"; if(!scene_name_error().isEmpty()) readiness="BLOCKED"; else if(task_warning_->text().contains("missing")) readiness="BLOCKED"; else if(task_warning_->text().contains("unknown")) readiness="WARNINGS"; summary_->setText(QString("<b>Robot:</b> %1 | base=%2 tip=%3 planning=%4<br/><b>Tool:</b> %5 | %3 -> %6 | tcp=%7 | mount rpy=%8,%9,%10<br/><b>Task:</b> pick zone=%11 camera=%12 pick source=%13 place target=%14 place link=%15<br/><b>Readiness:</b> %16").arg(robot_->currentText(),robot_base_link_->text(),robot_tip_link_->text(),robot_planning_group_->text(),ee_->currentText(),ee_attach_link_->text(),ee_tcp_link_->text(),ee_roll_->text(),ee_pitch_->text(),ee_yaw_->text(),pick_zone_source_->currentText(),pick_camera_->currentText(),pick_source_->currentText(),place_target_->currentText(),place_frame_link_->currentText(),readiness)); }

bool NewCellWizard::create_scene_scaffold(bool open_in_builder){ if(!scene_name_error().isEmpty()||!scene_name_warning().isEmpty()) return false; const fs::path scene_dir=scenes_root_path()/scene_name_->text().trimmed().toStdString(); boost::system::error_code ec; fs::create_directories(scene_dir/"config",ec); if(ec) return false; std::ofstream out((scene_dir/"environment.yaml").string()); out<<"scene_name: "<<scene_name_->text().trimmed().toStdString()<<"\n"; out<<"robot: "<<robot_->currentText().toStdString()<<"\n"; out<<"end_effector: "<<ee_->currentText().toStdString()<<"\n"; out<<"workcell_studio:\n  frames:\n    robot_base_link: "<<robot_base_link_->text().toStdString()<<"\n    robot_tip_link: "<<robot_tip_link_->text().toStdString()<<"\n    end_effector_attach_link: "<<ee_attach_link_->text().toStdString()<<"\n    end_effector_tcp_link: "<<ee_tcp_link_->text().toStdString()<<"\n  pick_zone:\n    source: "<<pick_zone_source_->currentText().toStdString()<<"\n    camera: "<<pick_camera_->currentText().toStdString()<<"\n    pick_object_source: "<<pick_source_->currentText().toStdString()<<"\n  place_zone:\n    target: "<<place_target_->currentText().toStdString()<<"\n";
 out<<"fake_hardware_first: true\nreal_robot_locked: true\nruntime_execution_enabled: false\nscaffold_only: true\n"; out.close(); result_.created=true; result_.open_in_scene_builder=open_in_builder; result_.scene_name=scene_name_->text().trimmed(); result_.scene_dir=scene_dir; return true; }
