#include "gui/new_cell_wizard.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QDir>
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
#include <QScrollArea>
#include <QVBoxLayout>

#include <array>
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
QString NewCellWizard::default_end_effector_tcp_link(const QString &m){ return m=="robotiq_85"?"ee_palm":(m=="single_suction"?"tcp_link":"tool0"); }
QString NewCellWizard::default_end_effector_type(const QString &m){ if(m=="robotiq_85") return "finger"; if(m=="single_suction") return "suction"; if(m.contains("vacuum")) return "vacuum_array"; return "placeholder"; }

void NewCellWizard::build_ui(){
 setWindowTitle("New Cell"); resize(1040,720); auto *root=new QVBoxLayout(this);
 root->addWidget(new QLabel("<h2>New Cell</h2><p><b>Safety badge:</b> Fake hardware default / Real robot locked</p>"));
 auto *body=new QHBoxLayout(); root->addLayout(body,1);
 steps_=new QListWidget(this); steps_->addItems({"1 Basics","2 Robot","3 End Effector","4 Environment","5 Task Intent","6 Review"}); steps_->setFixedWidth(220); body->addWidget(steps_);
 stack_=new QStackedWidget(this); body->addWidget(stack_,1);
 auto mk_page=[&](QWidget*w){auto*c=new QWidget(this); auto*l=new QVBoxLayout(c); l->addWidget(w); l->addStretch(1); stack_->addWidget(c);};
 auto mk_pose_spin=[](int decimals, const QString &suffix, const QString &tooltip){auto*s=new QDoubleSpinBox(); s->setDecimals(decimals); s->setRange(-1000,1000); s->setSuffix(suffix); s->setToolTip(tooltip); return s;};
 auto mk_pose_editor=[&](QDoubleSpinBox*&x,QDoubleSpinBox*&y,QDoubleSpinBox*&z,QDoubleSpinBox*&r,QDoubleSpinBox*&p,QDoubleSpinBox*&yaw){
   auto *wrap = new QWidget(this); auto *v = new QVBoxLayout(wrap);
   auto *pos = new QGroupBox("Position", wrap); auto *posGrid = new QGridLayout(pos);
   x=mk_pose_spin(3," m","X: forward/back"); y=mk_pose_spin(3," m","Y: left/right"); z=mk_pose_spin(3," m","Z: up/down");
   posGrid->addWidget(new QLabel("X (forward/back)"),0,0); posGrid->addWidget(x,0,1);
   posGrid->addWidget(new QLabel("Y (left/right)"),1,0); posGrid->addWidget(y,1,1);
   posGrid->addWidget(new QLabel("Z (up/down)"),2,0); posGrid->addWidget(z,2,1);
   auto *ori = new QGroupBox("Orientation", wrap); auto *oriGrid = new QGridLayout(ori);
   r=mk_pose_spin(4," rad","Roll: rotate around X"); p=mk_pose_spin(4," rad","Pitch: rotate around Y"); yaw=mk_pose_spin(4," rad","Yaw: rotate around Z");
   oriGrid->addWidget(new QLabel("Roll (around X)"),0,0); oriGrid->addWidget(r,0,1);
   oriGrid->addWidget(new QLabel("Pitch (around Y)"),1,0); oriGrid->addWidget(p,1,1);
   oriGrid->addWidget(new QLabel("Yaw (around Z)"),2,0); oriGrid->addWidget(yaw,2,1);
   v->addWidget(pos); v->addWidget(ori); return wrap;
 };

 auto *s1=new QWidget(this); auto *f1=new QFormLayout(s1); display_name_=new QLineEdit(); scene_name_=new QLineEdit(); description_=new QTextEdit(); template_=new QComboBox(); template_->addItems({"Pick & Place","Sorting","Inspection","Machine Tending","Conveyor Picking","Blank Cell"}); output_path_=new QLineEdit(QString::fromStdString(scenes_root_path().string())); scene_error_=new QLabel(); scene_warning_=new QLabel(); f1->addRow("Cell display name",display_name_); f1->addRow("Scene/package name",scene_name_); f1->addRow("",scene_error_); f1->addRow("",scene_warning_); f1->addRow("Description",description_); f1->addRow("Template",template_); f1->addRow("Output scenes path",output_path_); mk_page(s1);

 auto *s2=new QWidget(this); auto *f2=new QFormLayout(s2); robot_=new QComboBox(); robot_->addItems({"UR5","UR10","UR3","Generic Cartesian Placeholder"}); robot_base_link_=new QComboBox(); robot_tip_link_=new QComboBox(); robot_planning_group_=new QComboBox();
 for(auto*c:{robot_base_link_,robot_tip_link_,robot_planning_group_}) c->setEditable(true);
 robot_controller_name_=new QLineEdit("scaled_joint_trajectory_controller"); robot_controller_name_->setReadOnly(true);
 auto *robot_pose=mk_pose_editor(robot_x_,robot_y_,robot_z_,robot_roll_,robot_pitch_,robot_yaw_);
 robot_advanced_group_=new QGroupBox("Advanced Robot Frames"); robot_advanced_group_->setCheckable(true); robot_advanced_group_->setChecked(false); auto*raf=new QFormLayout(robot_advanced_group_); raf->addRow("Robot base link",robot_base_link_); raf->addRow("Robot end-effector/tip link",robot_tip_link_); raf->addRow("Planning group",robot_planning_group_); raf->addRow("Controller name",robot_controller_name_);
 robot_warning_=new QLabel(); robot_warning_->setWordWrap(true);
 f2->addRow("Robot",robot_); f2->addRow("Robot base pose",robot_pose); f2->addRow(robot_advanced_group_); f2->addRow("",robot_warning_); mk_page(s2);

 auto *s3=new QWidget(this); auto *f3=new QFormLayout(s3); ee_=new QComboBox(); ee_->addItems({"robotiq_85","single_suction","parallel_gripper","vacuum_array_placeholder"}); ee_attach_link_=new QComboBox(); ee_tcp_link_=new QComboBox(); ee_attach_link_->setEditable(true); ee_tcp_link_->setEditable(true); ee_type_=new QComboBox(); ee_type_->addItems({"finger","suction","vacuum_array","placeholder"});
 auto *ee_pose=mk_pose_editor(ee_x_,ee_y_,ee_z_,ee_roll_,ee_pitch_,ee_yaw_); ee_roll_->setValue(-1.5708); ee_pitch_->setValue(-1.5708);
 ee_advanced_group_=new QGroupBox("Advanced Tool Frames"); ee_advanced_group_->setCheckable(true); ee_advanced_group_->setChecked(false); auto*eaf=new QFormLayout(ee_advanced_group_); eaf->addRow("End-effector attach/base link",ee_attach_link_); eaf->addRow("End-effector TCP/tool link",ee_tcp_link_); eaf->addRow("End-effector type",ee_type_);
 f3->addRow("End effector",ee_); f3->addRow("End effector mount pose",ee_pose); f3->addRow(ee_advanced_group_); mk_page(s3);

 auto *s4=new QWidget(this); auto *v4=new QVBoxLayout(s4); v4->addWidget(new QLabel("Environment Assets and relationships"));
 auto *split=new QHBoxLayout(); env_substeps_=new QListWidget(this); env_substeps_->addItems({"1 Select Assets","2 Attachments & Links","3 Frames & Role Mapping","4 Poses","5 Review"}); env_substeps_->setFixedWidth(220); split->addWidget(env_substeps_);
 env_substep_stack_=new QStackedWidget(this); split->addWidget(env_substep_stack_,1); v4->addLayout(split);

 env_objects_table_=new QTableWidget(0,9,this); env_objects_table_->setHorizontalHeaderLabels({"Enabled","Object ID","Asset Type","Parent","Parent Link","Child Link","Joint","Semantic Role","Pose Summary"});

 auto *env1=new QWidget(this); auto *env1v=new QVBoxLayout(env1);
 env_add_asset_combo_=new QComboBox(this); env_add_asset_combo_->addItems({"workbench_01","source_bin_01","place_fixture_01","reject_bin_01","conveyor_01","camera_01","camera_mount_01","safety_zone_01","custom_01"}); env_add_asset_combo_->addItems(discover_environment_asset_catalog());
 auto *env1btn=new QHBoxLayout(); auto*add_asset=new QPushButton("Add Asset"); auto*rec=new QPushButton("Use Recommended Layout"); auto*clear=new QPushButton("Clear Layout"); env1btn->addWidget(env_add_asset_combo_); env1btn->addWidget(add_asset); env1btn->addWidget(rec); env1btn->addWidget(clear); env1btn->addStretch(1); env1v->addLayout(env1btn); env1v->addWidget(env_objects_table_);
 env_substep_stack_->addWidget(env1);

 auto *env2=new QWidget(this); auto *env2f=new QFormLayout(env2); env_parent_combo_=new QComboBox(this); env_parent_combo_->setEditable(true); env_parent_link_combo_=new QComboBox(this); env_parent_link_combo_->setEditable(true); env_child_link_combo_=new QComboBox(this); env_child_link_combo_->setEditable(true); env2f->addRow("Parent object",env_parent_combo_); env2f->addRow("Parent link",env_parent_link_combo_); env2f->addRow("Child link",env_child_link_combo_); env_substep_stack_->addWidget(env2);
 auto *env3=new QWidget(this); auto *env3f=new QFormLayout(env3); env_frame_combo_=new QComboBox(this); env_frame_combo_->setEditable(true); env_frame_combo_->addItems({"world","base_link","camera_link","table_link"}); env_role_combo_=new QComboBox(this); env_role_combo_->setEditable(true); env_role_combo_->addItems({"pick_source","place_target","reject_target","camera_view_zone","support_surface","safety_guard","conveyor/source_line"}); env3f->addRow("Reference frame",env_frame_combo_); env3f->addRow("Semantic role",env_role_combo_); env_substep_stack_->addWidget(env3);
 auto *env4=new QWidget(this); auto *env4v=new QVBoxLayout(env4); env4v->addWidget(new QLabel("Pose editing uses the same labeled pose editor widget in this wizard.")); env_substep_stack_->addWidget(env4);
 auto *env5=new QWidget(this); auto *env5v=new QVBoxLayout(env5); env_review_table_=new QTableWidget(0,6,this); env_review_table_->setHorizontalHeaderLabels({"Object ID","Asset Type","Parent.Link -> Child Link","Semantic Role","Pose Summary","Status"}); env_review_table_->setWordWrap(true); env_review_table_->horizontalHeader()->setSectionResizeMode(0,QHeaderView::ResizeToContents); env_review_table_->horizontalHeader()->setSectionResizeMode(1,QHeaderView::ResizeToContents); env_review_table_->horizontalHeader()->setSectionResizeMode(2,QHeaderView::Stretch); env_review_table_->horizontalHeader()->setSectionResizeMode(3,QHeaderView::ResizeToContents); env_review_table_->horizontalHeader()->setSectionResizeMode(4,QHeaderView::Stretch); env_review_table_->horizontalHeader()->setSectionResizeMode(5,QHeaderView::ResizeToContents); env_edit_selected_button_=new QPushButton("Edit Selected Asset"); env5v->addWidget(env_review_table_); env5v->addWidget(env_edit_selected_button_); env_substep_stack_->addWidget(env5);
 env_preview_=new QLabel(); env_preview_->setWordWrap(true); v4->addWidget(env_preview_); mk_page(s4);

 auto *s5=new QWidget(this); auto *v5=new QVBoxLayout(s5); task_family_=new QComboBox(); task_family_->addItems({"pick_place","sorting","inspection","machine_tending","conveyor_picking","blank"});
 task_intent_text_=new QTextEdit();
 auto *task_template_card=new QGroupBox("Task Template",s5); auto *task_template_form=new QFormLayout(task_template_card); task_template_form->addRow("Task family",task_family_); task_template_form->addRow("Task intent",task_intent_text_);
 pick_zone_source_=new QComboBox(); pick_zone_source_->addItems({"Camera view zone","Source bin","Conveyor tracking zone","Manual bounds"}); pick_camera_=new QComboBox(); pick_detection_source_=new QComboBox(); pick_detection_source_->addItems({"EPD / RealSense","PointCloud2 direct","Simulated/manual"}); pick_source_=new QComboBox(); pick_zone_frame_=new QComboBox(); pick_zone_frame_->addItems({"camera frame","source object frame","world"}); pick_zone_frame_->setEditable(true);
 auto *pick_config_card=new QGroupBox("Pick Configuration",s5); auto *pick_form=new QFormLayout(pick_config_card); pick_form->addRow("Pick zone source",pick_zone_source_); pick_form->addRow("Camera",pick_camera_); pick_form->addRow("Detection source",pick_detection_source_); pick_form->addRow("Pick object source",pick_source_); pick_form->addRow("Zone frame",pick_zone_frame_); pick_form->addRow(new QLabel("Pick zone is where perception searches for objects. By default this is the selected camera view zone. The source bin or conveyor defines where objects are expected."));
 place_target_=new QComboBox(); place_target_->setEditable(true); place_frame_link_=new QComboBox(); place_frame_link_->setEditable(true); placement_mode_=new QComboBox(); placement_mode_->addItems({"top_of_surface","inside_bin","fixture_alignment","conveyor_dropoff","manual_pose"}); placement_alignment_=new QComboBox(); placement_alignment_->addItems({"none","fixed_yaw","match_object_orientation","feature_based"});
 auto *place_config_card=new QGroupBox("Place Configuration",s5); auto *place_form=new QFormLayout(place_config_card); place_form->addRow("Place target",place_target_); place_form->addRow("Place frame/link",place_frame_link_); place_form->addRow("Placement mode",placement_mode_); place_form->addRow("Alignment",placement_alignment_);
 grasp_strategy_=new QComboBox(); grasp_strategy_->addItems({"auto","top_down_2f","suction_top","side_grasp_placeholder"}); approach_axis_=new QComboBox(); approach_axis_->addItems({"z_down","z_up","x_forward","y_left"}); release_strategy_=new QComboBox(); release_strategy_->addItems({"open_gripper","vacuum_off","none/scaffold"}); approach_distance_=mk_pose_spin(3," m","Approach distance"); retreat_distance_=mk_pose_spin(3," m","Retreat distance"); approach_distance_->setValue(0.10); retreat_distance_->setValue(0.10); task_warning_=new QLabel(); task_warning_->setWordWrap(true); task_readiness_label_=new QLabel(); task_readiness_label_->setWordWrap(true);
 auto *grasp_motion_card=new QGroupBox("Grasp and Motion Intent",s5); auto *grasp_form=new QFormLayout(grasp_motion_card); grasp_form->addRow("Grasp strategy",grasp_strategy_); grasp_form->addRow("Approach axis",approach_axis_); grasp_form->addRow("Release strategy",release_strategy_); grasp_form->addRow("Approach distance",approach_distance_); grasp_form->addRow("Retreat distance",retreat_distance_); grasp_form->addRow("Readiness",task_readiness_label_); grasp_form->addRow("Warnings / blockers",task_warning_);
 v5->addWidget(task_template_card); v5->addWidget(pick_config_card); v5->addWidget(place_config_card); v5->addWidget(grasp_motion_card); mk_page(s5);

 auto *s6=new QWidget(this); auto *v6=new QVBoxLayout(s6); summary_=new QLabel(); summary_->setWordWrap(true); summary_->setFrameShape(QFrame::StyledPanel); v6->addWidget(summary_); mk_page(s6);

 auto*nav=new QHBoxLayout(); back_=new QPushButton("Back"); next_=new QPushButton("Next"); create_=new QPushButton("Create Cell"); create_open_=new QPushButton("Create and Open"); auto*cancel=new QPushButton("Cancel"); nav->addWidget(back_); nav->addWidget(next_); nav->addStretch(1); nav->addWidget(create_); nav->addWidget(create_open_); nav->addWidget(cancel); root->addLayout(nav);
 connect(steps_,&QListWidget::currentRowChanged,stack_,&QStackedWidget::setCurrentIndex); connect(steps_,&QListWidget::currentRowChanged,this,&NewCellWizard::refresh_validation); steps_->setCurrentRow(0);
 connect(back_,&QPushButton::clicked,this,[this]{steps_->setCurrentRow(std::max(0,steps_->currentRow()-1));}); connect(next_,&QPushButton::clicked,this,[this]{steps_->setCurrentRow(std::min(5,steps_->currentRow()+1));}); connect(cancel,&QPushButton::clicked,this,&QDialog::reject);
 connect(create_,&QPushButton::clicked,this,[this]{if(create_scene_scaffold(false))accept();}); connect(create_open_,&QPushButton::clicked,this,[this]{if(create_scene_scaffold(true))accept();});
 connect(rec,&QPushButton::clicked,this,&NewCellWizard::apply_recommended_environment_layout);
 connect(clear,&QPushButton::clicked,this,[this]{env_objects_table_->setRowCount(0); refresh_environment_parent_options(); refresh_summary();});
 connect(preview,&QPushButton::clicked,this,&NewCellWizard::refresh_summary);
 connect(task_family_,&QComboBox::currentTextChanged,this,[this](const QString&){apply_task_family_defaults(); refresh_validation();});
 connect(add_asset,&QPushButton::clicked,this,[this]{ const QString id=env_add_asset_combo_->currentText().trimmed(); if(id.isEmpty()) return; add_environment_asset_row(id,id, "world","world","asset_link","custom"); refresh_environment_parent_options(); refresh_environment_review_table(); });
 connect(env_substeps_,&QListWidget::currentRowChanged,this,&NewCellWizard::select_environment_substep);
 connect(env_edit_selected_button_,&QPushButton::clicked,this,[this]{ if(env_review_table_->currentRow()>=0){ env_substeps_->setCurrentRow(1); }});
 connect(robot_,&QComboBox::currentTextChanged,this,[this](const QString&r){robot_base_link_->clear(); robot_tip_link_->clear(); robot_planning_group_->clear(); robot_base_link_->addItems({"base_link","base","world"}); robot_tip_link_->addItems({"ee_link","tool0","wrist_3_link"}); robot_planning_group_->addItems({"manipulator","arm","preview_group"}); robot_base_link_->setCurrentText(default_robot_base_link(r)); robot_tip_link_->setCurrentText(default_robot_tip_link(r)); robot_planning_group_->setCurrentText(default_robot_planning_group(r)); refresh_validation();});
 connect(ee_,&QComboBox::currentTextChanged,this,[this](const QString&m){ee_attach_link_->clear(); ee_tcp_link_->clear(); ee_attach_link_->addItems({"gripper_base_link","robotiq_arg2f_base_link","ee_palm","tool0","tcp_link","suction_cup_link"}); ee_tcp_link_->addItems({"ee_palm","tcp_link","tool0","suction_cup_link"}); ee_attach_link_->setCurrentText(default_end_effector_attach_link(m)); ee_tcp_link_->setCurrentText(default_end_effector_tcp_link(m)); ee_type_->setCurrentText(default_end_effector_type(m)); if(m=="robotiq_85"||m=="single_suction"){ee_roll_->setValue(-1.5708);ee_pitch_->setValue(-1.5708);ee_yaw_->setValue(0);} refresh_validation();});
 robot_->setCurrentText("UR5"); ee_->setCurrentText("robotiq_85"); env_substeps_->setCurrentRow(0); apply_recommended_environment_layout();
}

void NewCellWizard::add_environment_asset_row(const QString &id,const QString &asset,const QString &parent,const QString &plink,const QString &clink,const QString &role){int r=env_objects_table_->rowCount(); env_objects_table_->insertRow(r); env_objects_table_->setCellWidget(r,0,new QCheckBox()); static_cast<QCheckBox*>(env_objects_table_->cellWidget(r,0))->setChecked(true); env_objects_table_->setItem(r,1,new QTableWidgetItem(id)); env_objects_table_->setItem(r,2,new QTableWidgetItem(asset)); env_objects_table_->setItem(r,3,new QTableWidgetItem(parent)); env_objects_table_->setItem(r,4,new QTableWidgetItem(plink)); env_objects_table_->setItem(r,5,new QTableWidgetItem(clink)); env_objects_table_->setItem(r,6,new QTableWidgetItem("fixed")); env_objects_table_->setItem(r,7,new QTableWidgetItem(role)); env_objects_table_->setItem(r,8,new QTableWidgetItem("x=0.000, y=0.000, z=0.000, r=0.0000, p=0.0000, y=0.0000")); }
void NewCellWizard::apply_recommended_environment_layout(){ env_objects_table_->setRowCount(0); add_environment_asset_row("workbench_01","workbench/table","world","world","table_link","support_surface"); add_environment_asset_row("source_bin_01","bin","workbench_01","table_link","bin_link","pick_source"); add_environment_asset_row("place_fixture_01","fixture/table","workbench_01","table_link","fixture_link","place_target"); add_environment_asset_row("reject_bin_01","bin","workbench_01","table_link","bin_link","reject_target"); add_environment_asset_row("conveyor_01","conveyor","world","world","conveyor_link","conveyor/source_line"); add_environment_asset_row("camera_01","RealSense D435i","workbench_01","table_link","camera_link","camera_view_zone"); add_environment_asset_row("camera_mount_01","camera_mount","workbench_01","table_link","mount_link","camera_support"); add_environment_asset_row("safety_zone_01","safety_zone","world","world","safety_zone_link","safety_guard"); refresh_environment_parent_options(); refresh_environment_review_table(); refresh_summary(); }
QStringList NewCellWizard::discover_environment_asset_catalog() const { QStringList found; for(const QString &rel:{"assets/environment","assets/environment_objects"}){ QDir dir(workspace_root_+"/"+rel); const auto entries=dir.entryList(QDir::Dirs|QDir::NoDotAndDotDot); for(const auto &e:entries) found<<e; } found.removeDuplicates(); return found; }
void NewCellWizard::select_environment_substep(int index){ if(env_substep_stack_) env_substep_stack_->setCurrentIndex(std::max(0,index)); }
void NewCellWizard::refresh_environment_review_table(){ env_review_table_->setRowCount(0); for(int r=0;r<env_objects_table_->rowCount();++r){ int rr=env_review_table_->rowCount(); env_review_table_->insertRow(rr); auto id=env_objects_table_->item(r,1)->text(); auto type=env_objects_table_->item(r,2)->text(); auto rel=env_objects_table_->item(r,3)->text()+"."+env_objects_table_->item(r,4)->text()+" -> "+env_objects_table_->item(r,5)->text(); auto role=env_objects_table_->item(r,7)->text(); auto pose=env_objects_table_->item(r,8)->text(); auto *cb=qobject_cast<QCheckBox*>(env_objects_table_->cellWidget(r,0)); auto status=(cb&&cb->isChecked())?"enabled":"disabled"; env_review_table_->setItem(rr,0,new QTableWidgetItem(id)); env_review_table_->setItem(rr,1,new QTableWidgetItem(type)); env_review_table_->setItem(rr,2,new QTableWidgetItem(rel)); env_review_table_->setItem(rr,3,new QTableWidgetItem(role)); env_review_table_->setItem(rr,4,new QTableWidgetItem(pose)); env_review_table_->setItem(rr,5,new QTableWidgetItem(status)); }}

void NewCellWizard::refresh_environment_parent_options(){
 pick_camera_->clear(); pick_source_->clear(); place_target_->clear(); place_frame_link_->clear();
 for(int r=0;r<env_objects_table_->rowCount();++r){
  auto*cb=qobject_cast<QCheckBox*>(env_objects_table_->cellWidget(r,0)); if(!cb||!cb->isChecked()) continue;
  const auto role=env_objects_table_->item(r,7)->text().toLower(); const auto id=env_objects_table_->item(r,1)->text();
  if(role.contains("camera")) pick_camera_->addItem(id);
  if(role.contains("pick_source")||role.contains("conveyor")) pick_source_->addItem(id);
  if(role.contains("place_target")||role.contains("reject_target")||role.contains("output_bin")||role.contains("fixture")||role.contains("support")) place_target_->addItem(id);
 }
 pick_source_->addItem("manual");
 place_frame_link_->addItems({"target_link","top_surface","world"});
 if(pick_camera_->findText("camera_01")>=0){ pick_zone_source_->setCurrentText("Camera view zone"); pick_camera_->setCurrentText("camera_01"); }
 if(pick_source_->findText("source_bin_01")>=0) pick_source_->setCurrentText("source_bin_01");
 apply_task_family_defaults();
}

void NewCellWizard::apply_task_family_defaults(){
 const auto family=task_family_->currentText();
 if(family=="pick_place"){ if(place_target_->findText("place_fixture_01")>=0) place_target_->setCurrentText("place_fixture_01"); }
 else if(family=="sorting"){ if(place_target_->findText("reject_bin_01")>=0) place_target_->setCurrentText("reject_bin_01"); }
 else if(family=="inspection"){ if(place_target_->findText("place_fixture_01")>=0) place_target_->setCurrentText("place_fixture_01"); }
 else if(family=="conveyor_picking"){ if(place_target_->findText("place_fixture_01")>=0) place_target_->setCurrentText("place_fixture_01"); if(pick_source_->findText("conveyor_01")>=0) pick_source_->setCurrentText("conveyor_01"); pick_zone_source_->setCurrentText("Conveyor tracking zone"); }
}

QStringList NewCellWizard::readiness_warnings() const{
 QStringList warnings;
 const bool unknown=robot_tip_link_->currentText().trimmed().isEmpty()||ee_attach_link_->currentText().trimmed().isEmpty()||ee_tcp_link_->currentText().trimmed().isEmpty();
 if(unknown) warnings<<"Unknown robot/tool links still editable placeholders.";
 if(pick_camera_->currentText().isEmpty() && pick_zone_source_->currentText()=="Camera view zone") warnings<<"Camera view zone selected but no enabled camera asset.";
 return warnings;
}

QStringList NewCellWizard::readiness_blockers() const{
 QStringList blockers;
 const auto family=task_family_->currentText();
 if(pick_source_->currentText().isEmpty()) blockers<<"Pick object source is required.";
 if((family=="pick_place"||family=="sorting"||family=="inspection"||family=="conveyor_picking") && place_target_->currentText().isEmpty()) blockers<<"Place target is required for selected task family.";
 if(family=="inspection" && pick_camera_->currentText().isEmpty()) blockers<<"Inspection requires at least one camera.";
 if(family=="conveyor_picking" && !pick_zone_source_->currentText().contains("Conveyor")) blockers<<"Conveyor picking requires conveyor tracking zone.";
 return blockers;
}
void NewCellWizard::refresh_environment_parent_options(){ pick_camera_->clear(); pick_source_->clear(); place_target_->clear(); place_frame_link_->clear(); env_parent_combo_->clear(); for(int r=0;r<env_objects_table_->rowCount();++r){auto*cb=qobject_cast<QCheckBox*>(env_objects_table_->cellWidget(r,0)); if(!cb||!cb->isChecked()) continue; auto role=env_objects_table_->item(r,7)->text(); auto id=env_objects_table_->item(r,1)->text(); env_parent_combo_->addItem(id); if(role.contains("camera_view_zone")) pick_camera_->addItem(id); if(role.contains("pick_source")||role.contains("conveyor")) pick_source_->addItem(id); if(role.contains("place_target")||role.contains("reject_target")||role.contains("support_surface")||role.contains("conveyor")) place_target_->addItem(id);} pick_source_->addItem("manual"); place_frame_link_->addItems({"target_link","top_surface","world"}); if(pick_camera_->findText("camera_01")>=0) pick_camera_->setCurrentText("camera_01"); if(pick_camera_->count()>0) pick_zone_source_->setCurrentText("Camera view zone"); if(pick_source_->findText("source_bin_01")>=0) pick_source_->setCurrentText("source_bin_01"); refresh_environment_review_table(); }

fs::path NewCellWizard::scenes_root_path() const { return fs::path(workspace_root_.toStdString()) / "src" / "scenes"; }
QString NewCellWizard::scene_name_error() const { const QString n=scene_name_->text().trimmed(); if(n.isEmpty()) return "Scene/package name is required."; if(!is_valid_package_name(n)) return "Use lowercase letters, numbers, underscores only; must start with a letter."; return ""; }
QString NewCellWizard::scene_name_warning() const { const QString n=scene_name_->text().trimmed(); if(n.isEmpty()) return ""; const fs::path p=scenes_root_path()/n.toStdString(); if(fs::exists(p)) return "Warning: output folder already exists. Creation is blocked to avoid overwrite."; return ""; }

void NewCellWizard::refresh_validation(){
 const int row=steps_->currentRow(); const QString err=scene_name_error(); scene_error_->setText(err); scene_warning_->setText(scene_name_warning());
 const bool name_valid=err.isEmpty()&&scene_name_warning().isEmpty(); back_->setEnabled(row>0); next_->setEnabled(row<5&&(row!=0||name_valid));
 const auto blockers=readiness_blockers(); const auto warnings=readiness_warnings();
 QString readiness="READY"; if(!blockers.isEmpty()) readiness=(task_family_->currentText()=="blank")?"SCAFFOLD":"BLOCKED"; else if(!warnings.isEmpty()) readiness="WARNINGS";
 task_readiness_label_->setText(readiness);
 task_warning_->setText((blockers+warnings).join("\n"));
 create_->setEnabled(name_valid); create_open_->setEnabled(name_valid);
 refresh_summary();
}

void NewCellWizard::refresh_summary(){
 QString readiness=task_readiness_label_?task_readiness_label_->text():"READY";
 summary_->setText(QString("<b>Robot:</b> %1 | base=%2 tip=%3 planning=%4<br/><b>Tool:</b> %5 | attach=%6 | tcp=%7 | type=%8 | mount rpy=%9,%10,%11<br/><b>Task:</b> family=%12 pick zone=%13 camera=%14 pick source=%15 place target=%16 place link=%17 grasp=%18 approach=%19m retreat=%20m<br/><b>Readiness:</b> %21<br/><b>Warnings/Blockers:</b><br/>%22").arg(robot_->currentText(),robot_base_link_->currentText(),robot_tip_link_->currentText(),robot_planning_group_->currentText(),ee_->currentText(),ee_attach_link_->currentText(),ee_tcp_link_->currentText(),ee_type_->currentText(),ee_roll_->text(),ee_pitch_->text(),ee_yaw_->text(),task_family_->currentText(),pick_zone_source_->currentText(),pick_camera_->currentText(),pick_source_->currentText(),place_target_->currentText(),place_frame_link_->currentText(),grasp_strategy_->currentText(),approach_distance_->text(),retreat_distance_->text(),readiness,task_warning_->text().toHtmlEscaped().replace("\n","<br/>")));
}

bool NewCellWizard::create_scene_scaffold(bool open_in_builder){
 if(!scene_name_error().isEmpty()||!scene_name_warning().isEmpty()) return false;
 const fs::path scene_dir=scenes_root_path()/scene_name_->text().trimmed().toStdString();
 boost::system::error_code ec;
 fs::create_directories(scene_dir/"config",ec);
 if(ec) return false;

 auto yaml_scalar=[](const QString &value){
   std::string s=value.toStdString();
   size_t pos=0;
   while((pos=s.find("'",pos))!=std::string::npos){ s.insert(pos,"'"); pos+=2; }
   return std::string("'")+s+"'";
 };
 auto pose_number=[](QDoubleSpinBox *spin){ return spin->value(); };

 const QString warning_text=task_warning_->text().trimmed();
 const bool place_target_missing=place_target_->currentText().trimmed().isEmpty();
 const bool unknown_links=warning_text.contains("unknown");
 QString readiness="READY";
 if(place_target_missing) readiness="BLOCKED";
 else if(unknown_links) readiness="WARNINGS";

 std::ofstream out((scene_dir/"environment.yaml").string());
 out<<"scene_name: "<<scene_name_->text().trimmed().toStdString()<<"\n";
 out<<"robot: "<<robot_->currentText().toStdString()<<"\n";
 out<<"end_effector: "<<ee_->currentText().toStdString()<<"\n";
 out<<"workcell_studio:\n";
 out<<"  frames:\n";
 out<<"    robot_base_link: "<<yaml_scalar(robot_base_link_->currentText())<<"\n";
 out<<"    robot_tip_link: "<<yaml_scalar(robot_tip_link_->currentText())<<"\n";
 out<<"    planning_group: "<<yaml_scalar(robot_planning_group_->currentText())<<"\n";
 out<<"    end_effector_attach_link: "<<yaml_scalar(ee_attach_link_->currentText())<<"\n";
 out<<"    end_effector_tcp_link: "<<yaml_scalar(ee_tcp_link_->currentText())<<"\n";
 out<<"    robot_mount_pose:\n";
 out<<"      xyz: ["<<pose_number(robot_x_)<<", "<<pose_number(robot_y_)<<", "<<pose_number(robot_z_)<<"]\n";
 out<<"      rpy: ["<<pose_number(robot_roll_)<<", "<<pose_number(robot_pitch_)<<", "<<pose_number(robot_yaw_)<<"]\n";
 out<<"    tool_mount_pose:\n";
 out<<"      xyz: ["<<pose_number(ee_x_)<<", "<<pose_number(ee_y_)<<", "<<pose_number(ee_z_)<<"]\n";
 out<<"      rpy: ["<<pose_number(ee_roll_)<<", "<<pose_number(ee_pitch_)<<", "<<pose_number(ee_yaw_)<<"]\n";

 out<<"  environment_objects:\n";
 for(int r=0;r<env_objects_table_->rowCount();++r){
   auto*cb=qobject_cast<QCheckBox*>(env_objects_table_->cellWidget(r,0));
   if(!cb||!cb->isChecked()) continue;
   auto cell=[this,r](int c){ auto*item=env_objects_table_->item(r,c); return item?item->text().trimmed():QString(); };
   out<<"    - id: "<<yaml_scalar(cell(1))<<"\n";
   out<<"      asset_type: "<<yaml_scalar(cell(2))<<"\n";
   out<<"      parent_object: "<<yaml_scalar(cell(3))<<"\n";
   out<<"      parent_link: "<<yaml_scalar(cell(4))<<"\n";
   out<<"      child_link: "<<yaml_scalar(cell(5))<<"\n";
   out<<"      joint_type: "<<yaml_scalar(cell(6))<<"\n";
   out<<"      semantic_role: "<<yaml_scalar(cell(7))<<"\n";
   std::array<double,6> pose_values{0.0,0.0,0.0,0.0,0.0,0.0};
   const std::string pose_summary=cell(8).toStdString();
   std::regex number_re(R"(([+-]?\d*\.?\d+))");
   auto begin=std::sregex_iterator(pose_summary.begin(),pose_summary.end(),number_re);
   auto end=std::sregex_iterator();
   size_t i=0;
   for(auto it=begin;it!=end && i<pose_values.size();++it,++i) pose_values[i]=std::stod((*it)[1].str());
   out<<"      pose:\n";
   out<<"        xyz: ["<<pose_values[0]<<", "<<pose_values[1]<<", "<<pose_values[2]<<"]\n";
   out<<"        rpy: ["<<pose_values[3]<<", "<<pose_values[4]<<", "<<pose_values[5]<<"]\n";
 }

 out<<"  pick_zone:\n";
 out<<"    source: "<<yaml_scalar(pick_zone_source_->currentText())<<"\n";
 out<<"    camera: "<<yaml_scalar(pick_camera_->currentText())<<"\n";
 out<<"    pick_object_source: "<<yaml_scalar(pick_source_->currentText())<<"\n";
 out<<"    zone_frame: "<<yaml_scalar(pick_zone_frame_->currentText())<<"\n";
 out<<"    detection_source: "<<yaml_scalar(pick_detection_source_->currentText())<<"\n";

 out<<"  place_zone:\n";
 out<<"    target: "<<yaml_scalar(place_target_->currentText())<<"\n";
 out<<"    place_frame_link: "<<yaml_scalar(place_frame_link_->currentText())<<"\n";
 out<<"    placement_mode: "<<yaml_scalar(placement_mode_->currentText())<<"\n";
 out<<"    placement_alignment: "<<yaml_scalar(placement_alignment_->currentText())<<"\n";

 out<<"  task_intent:\n";
 out<<"    task_family: "<<yaml_scalar(task_family_->currentText())<<"\n";
 out<<"    intent_text: "<<yaml_scalar(task_intent_text_->toPlainText().trimmed())<<"\n";
 out<<"    grasp_strategy: "<<yaml_scalar(grasp_strategy_->currentText())<<"\n";
 out<<"    approach_axis: "<<yaml_scalar(approach_axis_->currentText())<<"\n";
 out<<"    release_strategy: "<<yaml_scalar(release_strategy_->currentText())<<"\n";
 out<<"    approach_distance_m: "<<approach_distance_->value()<<"\n";
 out<<"    retreat_distance_m: "<<retreat_distance_->value()<<"\n";
 out<<"    readiness: "<<yaml_scalar(readiness)<<"\n";

 out<<"  warnings:\n";
 if(warning_text.isEmpty()) out<<"    - "<<yaml_scalar("none")<<"\n";
 else out<<"    - "<<yaml_scalar(warning_text)<<"\n";

 out<<"fake_hardware_first: true\nreal_robot_locked: true\nruntime_execution_enabled: false\nscaffold_only: true\n";
 out.close();
 result_.created=true; result_.open_in_scene_builder=open_in_builder; result_.scene_name=scene_name_->text().trimmed(); result_.scene_dir=scene_dir; return true;
}
