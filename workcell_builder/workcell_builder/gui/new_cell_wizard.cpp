#include "gui/new_cell_wizard.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QDir>
#include <QFormLayout>
#include <QFrame>
#include <QFile>
#include <QFileInfo>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QRegularExpression>
#include <QStackedWidget>
#include <QTableWidget>
#include <QTextEdit>
#include <QScrollArea>
#include <QSet>
#include <QVBoxLayout>

#include <array>
#include <algorithm>
#include <fstream>
#include <regex>

namespace fs = boost::filesystem;

NewCellWizard::NewCellWizard(const QString &workspace_root, QWidget *parent): QDialog(parent), workspace_root_(workspace_root){ build_ui(); refresh_validation(); }

bool NewCellWizard::is_valid_package_name(const QString &name){ static const std::regex re("^[a-z][a-z0-9_]*$"); return std::regex_match(name.toStdString(), re);} 
QString NewCellWizard::default_gripper_rpy_text(){ return "-1.5708, -1.5708, 0"; }
QStringList NewCellWizard::recommended_environment_assets(){ return {"workbench_01","source_bin_01","place_fixture_01","camera_01","safety_zone_01"}; }
QString NewCellWizard::default_robot_base_link(const QString &family, const QString &){ if (family == "Franka / Panda") return "panda_link0"; return "base_link"; }
QString NewCellWizard::default_robot_tip_link(const QString &family, const QString &r){ if (family == "Franka / Panda") return "panda_hand"; return r.toLower().contains("placeholder")?"tool0_placeholder":"ee_link"; }
QString NewCellWizard::default_robot_planning_group(const QString &family, const QString &r){ if (family == "Franka / Panda") return "panda_arm"; return r.toLower().contains("placeholder")?"preview_group":"manipulator"; }
QString NewCellWizard::default_end_effector_attach_link(const QString &m){ return m=="robotiq_85"?"gripper_base_link":"tool_mount_link"; }
QString NewCellWizard::default_end_effector_tcp_link(const QString &m){ return m=="robotiq_85"?"ee_palm":(m=="single_suction"?"tcp_link":"tool0"); }
QString NewCellWizard::default_end_effector_type(const QString &m){ if(m=="robotiq_85") return "finger"; if(m=="single_suction") return "suction"; if(m.contains("vacuum")) return "vacuum_array"; return "placeholder"; }
QString NewCellWizard::default_end_effector_family_readiness(const QString &family){ return family.contains("Custom") ? "SCAFFOLD" : "READY"; }

QList<NewCellScenarioChoice> NewCellWizard::load_industrial_scenario_choices(const QString &catalog_path) {
  QList<NewCellScenarioChoice> choices;
  QFile file(catalog_path);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return choices;
  NewCellScenarioChoice current;
  auto commit = [&] {
    if (!current.id.isEmpty() && !current.label.isEmpty() && current.order > 0) choices.push_back(current);
    current = {};
  };
  const QRegularExpression field_re("^\\s*(id|category|new_cell_wizard_label|new_cell_wizard_order):\\s*(.*?)\\s*$");
  while (!file.atEnd()) {
    const QString line = QString::fromUtf8(file.readLine());
    if (line.startsWith("- id:")) commit();
    const auto match = field_re.match(line.startsWith("- ") ? line.mid(2) : line);
    if (!match.hasMatch()) continue;
    QString value = match.captured(2).trimmed();
    if ((value.startsWith('"') && value.endsWith('"')) || (value.startsWith('\'') && value.endsWith('\''))) value = value.mid(1, value.size() - 2);
    const QString key = match.captured(1);
    if (key == "id") current.id = value;
    else if (key == "category") current.category = value;
    else if (key == "new_cell_wizard_label") current.label = value;
    else if (key == "new_cell_wizard_order") current.order = value.toInt();
  }
  commit();
  std::sort(choices.begin(), choices.end(), [](const auto &a, const auto &b) { return a.order < b.order; });
  return choices;
}

QString NewCellWizard::scenario_catalog_path() const {
  const QStringList candidates = {
    workspace_root_ + "/catalog/scenarios/industrial_scenarios.yaml",
    workspace_root_ + "/src/easy_manipulation_deployment/catalog/scenarios/industrial_scenarios.yaml",
    QDir::current().absoluteFilePath("catalog/scenarios/industrial_scenarios.yaml")};
  for (const QString &candidate : candidates) if (QFileInfo::exists(candidate)) return candidate;
  return candidates.front();
}

QString NewCellWizard::selected_scenario_id() const { return application_scenario_ ? application_scenario_->currentData(Qt::UserRole).toString() : QString(); }
QString NewCellWizard::selected_scenario_category() const { return application_scenario_ ? application_scenario_->currentData(Qt::UserRole + 1).toString() : QString(); }
QString NewCellWizard::selected_scenario_label() const { return application_scenario_ ? application_scenario_->currentText() : QString(); }
bool NewCellWizard::select_scenario_by_id(const QString &scenario_id) {
  if (!application_scenario_) return false;
  const int index = application_scenario_->findData(scenario_id, Qt::UserRole);
  if (index < 0) return false;
  application_scenario_->setCurrentIndex(index);
  return true;
}
QString NewCellWizard::review_text() const { return summary_ ? summary_->text() : QString(); }
bool NewCellWizard::pick_place_configuration_available() const { return pick_config_card_ && !pick_config_card_->isHidden(); }
QString NewCellWizard::selected_object_source_id() const { return object_source_mode_ ? object_source_mode_->currentData().toString() : QString(); }
bool NewCellWizard::select_object_source_by_id(const QString &source_id) {
  if (!object_source_mode_) return false;
  const int index = object_source_mode_->findData(source_id);
  if (index < 0) return false;
  object_source_mode_->setCurrentIndex(index);
  return true;
}
bool NewCellWizard::manual_object_geometry_valid() const {
  if (selected_object_source_id() != "manual_simulated") return true;
  return manual_object_id_ && !manual_object_id_->text().trimmed().isEmpty()
    && manual_object_frame_ && !manual_object_frame_->text().trimmed().isEmpty()
    && manual_object_x_->value() > 0.0 && manual_object_y_->value() > 0.0 && manual_object_z_->value() > 0.0;
}

void NewCellWizard::build_ui(){
 setWindowTitle("New Cell"); resize(1040,720); auto *root=new QVBoxLayout(this);
 root->addWidget(new QLabel("<h2>New Cell</h2><p><b>Safety badge:</b> Fake hardware default / Real robot locked</p>"));
 auto *body=new QHBoxLayout(); root->addLayout(body,1);
 steps_=new QListWidget(this); steps_->setObjectName("newCellWizardSteps"); steps_->addItems({"1 Basics","2 Robot","3 End Effector","4 Environment","5 Task Intent","6 Review"}); steps_->setFixedWidth(220); body->addWidget(steps_);
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

 auto *s1=new QWidget(this); auto *f1=new QFormLayout(s1); display_name_=new QLineEdit(); scene_name_=new QLineEdit(); description_=new QTextEdit(); application_scenario_=new QComboBox(); application_scenario_->setObjectName("applicationScenarioCombo");
 const auto scenario_choices = load_industrial_scenario_choices(scenario_catalog_path());
 for (const auto &choice : scenario_choices) { application_scenario_->addItem(choice.label, choice.id); application_scenario_->setItemData(application_scenario_->count()-1, choice.category, Qt::UserRole+1); }
 output_path_=new QLineEdit(QString::fromStdString(scenes_root_path().string())); scene_error_=new QLabel(); scene_warning_=new QLabel(); f1->addRow("Cell display name",display_name_); f1->addRow("Scene/package name",scene_name_); f1->addRow("",scene_error_); f1->addRow("",scene_warning_); f1->addRow("Description",description_); f1->addRow("Application / Scenario",application_scenario_); f1->addRow("Output scenes path",output_path_); mk_page(s1);

 auto *s2=new QWidget(this); auto *f2=new QFormLayout(s2); robot_family_=new QComboBox(); robot_family_->addItems({"Universal Robots / UR","Franka / Panda","Fanuc","ABB","Cartesian / Gantry","Delta","Custom / Placeholder"}); robot_=new QComboBox(); robot_base_link_=new QComboBox(); robot_tip_link_=new QComboBox(); robot_planning_group_=new QComboBox();
 for(auto*c:{robot_base_link_,robot_tip_link_,robot_planning_group_}) c->setEditable(true);
 robot_controller_name_=new QLineEdit("scaled_joint_trajectory_controller"); robot_controller_name_->setReadOnly(true);
 auto *robot_pose=mk_pose_editor(robot_x_,robot_y_,robot_z_,robot_roll_,robot_pitch_,robot_yaw_);
 robot_advanced_group_=new QGroupBox("Advanced Robot Frames"); robot_advanced_group_->setCheckable(true); robot_advanced_group_->setChecked(false); auto*raf=new QFormLayout(robot_advanced_group_); raf->addRow("Robot base link",robot_base_link_); raf->addRow("Robot end-effector/tip link",robot_tip_link_); raf->addRow("Planning group",robot_planning_group_); raf->addRow("Controller name",robot_controller_name_);
 robot_warning_=new QLabel(); robot_warning_->setWordWrap(true); robot_readiness_banner_=new QLabel(); robot_readiness_banner_->setWordWrap(true);
 f2->addRow("Robot family",robot_family_); f2->addRow("Robot model",robot_); f2->addRow("Robot base pose",robot_pose); f2->addRow(robot_advanced_group_); f2->addRow("Robot readiness",robot_readiness_banner_); f2->addRow("",robot_warning_); mk_page(s2);

 auto *s3=new QWidget(this); auto *f3=new QFormLayout(s3); ee_family_=new QComboBox(); ee_family_->addItems({"Robotiq","Suction","Vacuum Array","Parallel Gripper","Custom / Placeholder"}); ee_=new QComboBox(); ee_attach_link_=new QComboBox(); ee_tcp_link_=new QComboBox(); ee_attach_link_->setEditable(true); ee_tcp_link_->setEditable(true); ee_type_=new QComboBox(); ee_type_->addItems({"finger","suction","vacuum_array","placeholder"});
 auto *ee_pose=mk_pose_editor(ee_x_,ee_y_,ee_z_,ee_roll_,ee_pitch_,ee_yaw_); ee_roll_->setValue(-1.5708); ee_pitch_->setValue(-1.5708);
 ee_advanced_group_=new QGroupBox("Advanced Tool Frames"); ee_advanced_group_->setCheckable(true); ee_advanced_group_->setChecked(false); auto*eaf=new QFormLayout(ee_advanced_group_); eaf->addRow("End-effector attach/base link",ee_attach_link_); eaf->addRow("End-effector TCP/tool link",ee_tcp_link_); eaf->addRow("End-effector type",ee_type_);
 ee_readiness_banner_=new QLabel(); ee_readiness_banner_->setWordWrap(true);
 f3->addRow("Tool family",ee_family_); f3->addRow("Tool model",ee_); f3->addRow("End effector mount pose",ee_pose); f3->addRow(ee_advanced_group_); f3->addRow("Tool readiness",ee_readiness_banner_); mk_page(s3);

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

 auto *s5=new QWidget(this); auto *v5=new QVBoxLayout(s5); task_scenario_=new QLabel(); task_scenario_->setObjectName("derivedTaskScenario"); task_scenario_->setWordWrap(true); scenario_configuration_notice_=new QLabel(); scenario_configuration_notice_->setObjectName("scenarioConfigurationNotice"); scenario_configuration_notice_->setWordWrap(true);
 task_intent_text_=new QTextEdit();
 auto *task_template_card=new QGroupBox("Application Intent",s5); auto *task_template_form=new QFormLayout(task_template_card); task_template_form->addRow("Application / Scenario",task_scenario_); task_template_form->addRow("Task intent",task_intent_text_); task_template_form->addRow("",scenario_configuration_notice_);
 pick_zone_source_=new QComboBox(); pick_zone_source_->addItems({"Camera view zone","Source bin","Conveyor tracking zone","Manual bounds"}); pick_camera_=new QComboBox();
 object_source_mode_=new QComboBox(); object_source_mode_->setObjectName("objectSourceMode"); object_source_mode_->addItem("Live EPD / RealSense","live_epd_realsense"); object_source_mode_->addItem("Recorded / replayed perception","recorded_perception"); object_source_mode_->addItem("Manual / simulated object","manual_simulated");
 perception_binding_=new QLineEdit("/easy_perception_deployment/epd_localize_output"); perception_binding_->setObjectName("perceptionBinding"); required_object_class_=new QLineEdit(); required_object_class_->setPlaceholderText("Optional class label"); minimum_confidence_=new QDoubleSpinBox(); minimum_confidence_->setRange(0.0,1.0); minimum_confidence_->setSingleStep(0.05); minimum_confidence_->setValue(0.5); dynamic_object_notice_=new QLabel(); dynamic_object_notice_->setObjectName("dynamicObjectNotice"); dynamic_object_notice_->setWordWrap(true);
 pick_source_=new QComboBox(); pick_zone_frame_=new QComboBox(); pick_zone_frame_->addItems({"camera frame","source object frame","world"}); pick_zone_frame_->setEditable(true);
 pick_config_card_=new QGroupBox("Pick Configuration",s5); auto *pick_form=new QFormLayout(pick_config_card_); pick_form->addRow("Object source",object_source_mode_); pick_form->addRow("Perception binding",perception_binding_); pick_form->addRow("Required class / filter",required_object_class_); pick_form->addRow("Minimum confidence",minimum_confidence_); pick_form->addRow("Pick zone source",pick_zone_source_); pick_form->addRow("Camera",pick_camera_); pick_form->addRow("Source zone / object",pick_source_); pick_form->addRow("Zone frame",pick_zone_frame_); pick_form->addRow(dynamic_object_notice_); pick_form->addRow(new QLabel("The source zone says where to search. Runtime perception owns the observed object's identity, pose, dimensions, shape, and confidence."));
 manual_object_card_=new QGroupBox("Manual / Simulated Object Fallback",s5); manual_object_card_->setObjectName("manualObjectCard"); auto *manual_form=new QFormLayout(manual_object_card_); manual_object_id_=new QLineEdit("manual_object_01"); manual_object_id_->setObjectName("manualObjectId"); manual_object_shape_=new QComboBox(); manual_object_shape_->addItems({"box","cylinder","sphere"}); manual_object_x_=mk_pose_spin(3," m","Manual object X dimension"); manual_object_y_=mk_pose_spin(3," m","Manual object Y dimension"); manual_object_z_=mk_pose_spin(3," m","Manual object Z dimension"); manual_object_x_->setObjectName("manualObjectDimensionX"); manual_object_y_->setObjectName("manualObjectDimensionY"); manual_object_z_->setObjectName("manualObjectDimensionZ"); manual_object_x_->setRange(0.0,1000.0); manual_object_y_->setRange(0.0,1000.0); manual_object_z_->setRange(0.0,1000.0); manual_object_x_->setValue(0.05); manual_object_y_->setValue(0.05); manual_object_z_->setValue(0.05); manual_object_frame_=new QLineEdit("world"); manual_object_frame_->setObjectName("manualObjectFrame"); manual_pose_x_=mk_pose_spin(3," m","Manual object pose X"); manual_pose_y_=mk_pose_spin(3," m","Manual object pose Y"); manual_pose_z_=mk_pose_spin(3," m","Manual object pose Z"); auto *manual_dims=new QWidget(); auto *manual_dims_layout=new QHBoxLayout(manual_dims); manual_dims_layout->setContentsMargins(0,0,0,0); manual_dims_layout->addWidget(manual_object_x_); manual_dims_layout->addWidget(manual_object_y_); manual_dims_layout->addWidget(manual_object_z_); auto *manual_pose=new QWidget(); auto *manual_pose_layout=new QHBoxLayout(manual_pose); manual_pose_layout->setContentsMargins(0,0,0,0); manual_pose_layout->addWidget(manual_pose_x_); manual_pose_layout->addWidget(manual_pose_y_); manual_pose_layout->addWidget(manual_pose_z_); manual_form->addRow("Object ID / name",manual_object_id_); manual_form->addRow("Primitive shape",manual_object_shape_); manual_form->addRow("Dimensions X / Y / Z",manual_dims); manual_form->addRow("Pose frame",manual_object_frame_); manual_form->addRow("Pose X / Y / Z",manual_pose);
 place_target_=new QComboBox(); place_target_->setEditable(true); place_frame_link_=new QComboBox(); place_frame_link_->setEditable(true); placement_mode_=new QComboBox(); placement_mode_->addItems({"top_of_surface","inside_bin","fixture_alignment","conveyor_dropoff","manual_pose"}); placement_alignment_=new QComboBox(); placement_alignment_->addItems({"none","fixed_yaw","match_object_orientation","feature_based"});
 place_config_card_=new QGroupBox("Place Configuration",s5); auto *place_form=new QFormLayout(place_config_card_); place_form->addRow("Place target",place_target_); place_form->addRow("Place frame/link",place_frame_link_); place_form->addRow("Placement mode",placement_mode_); place_form->addRow("Alignment",placement_alignment_);
 grasp_strategy_=new QComboBox(); grasp_strategy_->addItems({"auto","top_down_2f","suction_top","side_grasp_placeholder"}); approach_axis_=new QComboBox(); approach_axis_->addItems({"z_down","z_up","x_forward","y_left"}); release_strategy_=new QComboBox(); release_strategy_->addItems({"open_gripper","vacuum_off","none/scaffold"}); approach_distance_=mk_pose_spin(3," m","Approach distance"); retreat_distance_=mk_pose_spin(3," m","Retreat distance"); approach_distance_->setValue(0.10); retreat_distance_->setValue(0.10); task_warning_=new QLabel(); task_warning_->setWordWrap(true); task_readiness_label_=new QLabel(); task_readiness_label_->setWordWrap(true);
 grasp_motion_card_=new QGroupBox("Grasp and Motion Intent",s5); auto *grasp_form=new QFormLayout(grasp_motion_card_); grasp_form->addRow("Grasp strategy",grasp_strategy_); grasp_form->addRow("Approach axis",approach_axis_); grasp_form->addRow("Release strategy",release_strategy_); grasp_form->addRow("Approach distance",approach_distance_); grasp_form->addRow("Retreat distance",retreat_distance_); grasp_form->addRow("Readiness",task_readiness_label_); grasp_form->addRow("Warnings / blockers",task_warning_);
 v5->addWidget(task_template_card); v5->addWidget(pick_config_card_); v5->addWidget(manual_object_card_); v5->addWidget(place_config_card_); v5->addWidget(grasp_motion_card_); mk_page(s5);

 auto *s6=new QWidget(this); auto *v6=new QVBoxLayout(s6); summary_=new QLabel(); summary_->setWordWrap(true); summary_->setFrameShape(QFrame::StyledPanel); v6->addWidget(summary_); mk_page(s6);

 auto*nav=new QHBoxLayout(); back_=new QPushButton("Back"); next_=new QPushButton("Next"); create_=new QPushButton("Create Cell"); create_open_=new QPushButton("Create and Open"); auto*cancel=new QPushButton("Cancel"); nav->addWidget(back_); nav->addWidget(next_); nav->addStretch(1); nav->addWidget(create_); nav->addWidget(create_open_); nav->addWidget(cancel); root->addLayout(nav);
 connect(steps_,&QListWidget::currentRowChanged,stack_,&QStackedWidget::setCurrentIndex); connect(steps_,&QListWidget::currentRowChanged,this,&NewCellWizard::refresh_validation); steps_->setCurrentRow(0);
 connect(back_,&QPushButton::clicked,this,[this]{steps_->setCurrentRow(std::max(0,steps_->currentRow()-1));}); connect(next_,&QPushButton::clicked,this,[this]{steps_->setCurrentRow(std::min(5,steps_->currentRow()+1));}); connect(cancel,&QPushButton::clicked,this,&QDialog::reject);
 connect(create_,&QPushButton::clicked,this,[this]{if(create_scene_scaffold(false))accept();}); connect(create_open_,&QPushButton::clicked,this,[this]{if(create_scene_scaffold(true))accept();});
 connect(rec,&QPushButton::clicked,this,&NewCellWizard::apply_recommended_environment_layout);
 connect(clear,&QPushButton::clicked,this,[this]{env_objects_table_->setRowCount(0); refresh_environment_parent_options(); refresh_summary();});
 connect(env_edit_selected_button_,&QPushButton::clicked,this,&NewCellWizard::refresh_summary);
 connect(application_scenario_,QOverload<int>::of(&QComboBox::currentIndexChanged),this,[this](int){apply_scenario_defaults(); refresh_scenario_ui(); refresh_validation();});
 connect(object_source_mode_,QOverload<int>::of(&QComboBox::currentIndexChanged),this,[this](int){refresh_object_source_ui(); refresh_validation();});
 for(auto *spin:{manual_object_x_,manual_object_y_,manual_object_z_}) connect(spin,QOverload<double>::of(&QDoubleSpinBox::valueChanged),this,[this](double){refresh_validation();});
 connect(manual_object_id_,&QLineEdit::textChanged,this,[this](const QString&){refresh_validation();}); connect(manual_object_frame_,&QLineEdit::textChanged,this,[this](const QString&){refresh_validation();});
 connect(add_asset,&QPushButton::clicked,this,[this]{ const QString id=env_add_asset_combo_->currentText().trimmed(); if(id.isEmpty()) return; add_environment_asset_row(id,id, "world","world","asset_link","custom"); refresh_environment_parent_options(); refresh_environment_review_table(); });
 connect(env_substeps_,&QListWidget::currentRowChanged,this,&NewCellWizard::select_environment_substep);
 connect(env_edit_selected_button_,&QPushButton::clicked,this,[this]{ if(env_review_table_->currentRow()>=0){ env_substeps_->setCurrentRow(1); }});
 connect(robot_family_,&QComboBox::currentTextChanged,this,[this](const QString&f){ refresh_robot_model_options(f); refresh_robot_links_for_selection(); refresh_validation();});
 connect(robot_,&QComboBox::currentTextChanged,this,[this](const QString&){ refresh_robot_links_for_selection(); refresh_validation();});
 connect(ee_family_,&QComboBox::currentTextChanged,this,[this](const QString&f){ refresh_tool_model_options(f); apply_tool_profile_selection(); refresh_validation();});
 connect(ee_,&QComboBox::currentTextChanged,this,[this](const QString&){ apply_tool_profile_selection(); refresh_validation();});
 robot_family_->setCurrentText("Universal Robots / UR"); refresh_robot_model_options(robot_family_->currentText()); robot_->setCurrentText("UR5"); refresh_robot_links_for_selection(); ee_family_->setCurrentText("Robotiq"); refresh_tool_model_options(ee_family_->currentText()); ee_->setCurrentText("robotiq_85"); apply_tool_profile_selection(); env_substeps_->setCurrentRow(0); apply_recommended_environment_layout(); select_scenario_by_id("static_table_pick_place"); refresh_scenario_ui(); refresh_object_source_ui();
}
QString NewCellWizard::normalize_tool_family(const QString &raw_family) const {
  const QString f = raw_family.toLower();
  if (f.contains("robotiq")) return "robotiq";
  if (f.contains("suction")) return "suction";
  if (f.contains("vacuum")) return "vacuum_array";
  if (f.contains("parallel")) return "parallel_gripper";
  return "placeholder";
}
QStringList NewCellWizard::builtin_tool_models_for_family(const QString &tool_family) const {
  if (tool_family == "Robotiq") return {"robotiq_85","robotiq_2f_140"};
  if (tool_family == "Suction") return {"single_suction","onrobot_airpick4"};
  if (tool_family == "Vacuum Array") return {"vacuum_array_placeholder","multi_cup_array"};
  if (tool_family == "Parallel Gripper") return {"parallel_gripper","rg2"};
  return {"custom_tool_placeholder"};
}
QStringList NewCellWizard::discover_tool_models_from_catalog(const QString &tool_family) const {
  QStringList out; QDir dir(workspace_root_ + "/catalog/capabilities/end_effectors"); if (!dir.exists()) return out;
  const QString key = normalize_tool_family(tool_family);
  for (const auto &fn : dir.entryList(QStringList() << "*.yaml", QDir::Files)) {
    QFile f(dir.absoluteFilePath(fn)); if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) continue;
    const QString text = QString::fromUtf8(f.readAll()).toLower();
    if (!(text.contains(key) || fn.toLower().contains(key))) continue;
    QRegularExpression id_re("id:\\s*([a-z0-9_\\-]+)"); auto m=id_re.match(text);
    out << (m.hasMatch() ? m.captured(1) : QFileInfo(fn).baseName());
  }
  out.removeDuplicates(); return out;
}
void NewCellWizard::refresh_tool_model_options(const QString &tool_family) {
  ee_->clear();
  QStringList models = builtin_tool_models_for_family(tool_family);
  models << discover_tool_models_from_catalog(tool_family);
  models.removeDuplicates();
  ee_->addItems(models);
}
NewCellWizard::ToolModelProfile NewCellWizard::tool_profile_for_selection(const QString &tool_family, const QString &tool_model) const {
  ToolModelProfile p; p.family=tool_family; p.model=tool_model;
  if (tool_family == "Robotiq" && tool_model == "robotiq_85") { p.ee_type="finger"; p.attach_link="gripper_base_link"; p.tcp_link="ee_palm"; }
  else if (tool_family == "Suction") { p.ee_type="suction"; p.attach_link="suction_base_link"; p.tcp_link="tcp_link"; }
  else if (tool_family == "Vacuum Array") { p.ee_type="vacuum_array"; p.attach_link="vacuum_array_base_link"; p.tcp_link="tcp_link"; }
  else if (tool_family == "Parallel Gripper") { p.ee_type="finger"; p.attach_link="gripper_base_link"; p.tcp_link="tcp_link"; p.readiness="WARNINGS"; p.reason="Parallel gripper may require model-specific controller metadata."; }
  else { p.ee_type="placeholder"; p.attach_link="tool_mount_link"; p.tcp_link="tool0"; p.readiness="SCAFFOLD"; p.reason="Custom/placeholder tool requires manual metadata."; }
  return p;
}
void NewCellWizard::apply_tool_profile_selection() {
  const auto p = tool_profile_for_selection(ee_family_->currentText(), ee_->currentText());
  ee_attach_link_->clear(); ee_tcp_link_->clear();
  ee_attach_link_->addItems({p.attach_link,"gripper_base_link","tool_mount_link","suction_base_link","vacuum_array_base_link","tool0"});
  ee_tcp_link_->addItems({p.tcp_link,"ee_palm","tcp_link","suction_cup_link","tool0"});
  ee_attach_link_->setCurrentText(p.attach_link); ee_tcp_link_->setCurrentText(p.tcp_link); ee_type_->setCurrentText(p.ee_type);
  ee_roll_->setValue(p.roll); ee_pitch_->setValue(p.pitch); ee_yaw_->setValue(p.yaw);
  const auto readiness = evaluate_tool_readiness();
  ee_readiness_banner_->setText(readiness.status + " - " + readiness.reason);
}
NewCellWizard::ToolSelectionReadiness NewCellWizard::evaluate_tool_readiness() const {
  const auto p = tool_profile_for_selection(ee_family_->currentText(), ee_->currentText());
  return {p.readiness, p.reason};
}

QString NewCellWizard::normalize_robot_family(const QString &raw_family) const {
  const QString f = raw_family.toLower();
  if (f.contains("ur") || f.contains("universal")) return "ur";
  if (f.contains("panda") || f.contains("franka")) return "panda";
  if (f.contains("fanuc")) return "fanuc";
  if (f.contains("abb")) return "abb";
  if (f.contains("gantry") || f.contains("cartesian")) return "gantry";
  if (f.contains("delta")) return "delta";
  return "custom";
}
QStringList NewCellWizard::discover_robot_models_from_assets(const QString &robot_family) const {
  QStringList out; QDir dir(workspace_root_ + "/assets/robots"); if (!dir.exists()) return out;
  const QString key = normalize_robot_family(robot_family);
  for (const auto &e : dir.entryList(QDir::Dirs | QDir::NoDotAndDotDot)) { if (e.toLower().contains(key)) out << e; }
  return out;
}
QStringList NewCellWizard::discover_robot_models_from_catalog(const QString &robot_family) const {
  QStringList out; QDir dir(workspace_root_ + "/catalog/capabilities/robots"); if (!dir.exists()) return out;
  const QString key = normalize_robot_family(robot_family);
  for (const auto &fn : dir.entryList(QStringList() << "*.yaml", QDir::Files)) {
    QFile f(dir.absoluteFilePath(fn)); if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) continue;
    const QString text = QString::fromUtf8(f.readAll()).toLower();
    if (text.contains("family: " + key) || text.contains("brand: " + key) || fn.toLower().contains(key)) {
      QRegularExpression id_re("id:\\s*([a-z0-9_\\-]+)"); auto m=id_re.match(text); out << (m.hasMatch() ? m.captured(1) : QFileInfo(fn).baseName().replace("robot_",""));
    }
  }
  return out;
}
QStringList NewCellWizard::discover_robot_models_for_family(const QString &robot_family) const {
  QStringList models;
  if (robot_family == "Universal Robots / UR") models << "UR5" << "UR10" << "UR3";
  else if (robot_family == "Franka / Panda") models << "Panda" << "FR3";
  else if (robot_family == "Fanuc") models << "M-10iA" << "LR Mate 200iD";
  else if (robot_family == "ABB") models << "IRB 120" << "IRB 2600";
  else if (robot_family == "Cartesian / Gantry") models << "Generic Cartesian Placeholder" << "Generic Gantry";
  else if (robot_family == "Delta") models << "Generic Delta" << "Delta Placeholder";
  else models << "Custom Placeholder";
  models << discover_robot_models_from_assets(robot_family) << discover_robot_models_from_catalog(robot_family);
  models.removeDuplicates(); return models;
}
void NewCellWizard::refresh_robot_model_options(const QString &robot_family) { robot_->clear(); robot_->addItems(discover_robot_models_for_family(robot_family)); }
void NewCellWizard::refresh_robot_links_for_selection() {
  const QString family = robot_family_->currentText(); const QString model = robot_->currentText(); const bool scaffold = model.toLower().contains("placeholder") || family=="Custom / Placeholder";
  robot_base_link_->clear(); robot_tip_link_->clear(); robot_planning_group_->clear();
  if (family == "Franka / Panda") { robot_base_link_->addItems({"panda_link0","base_link"}); robot_tip_link_->addItems({"panda_hand","panda_link8","ee_link"}); robot_planning_group_->addItems({"panda_arm","manipulator"}); }
  else { robot_base_link_->addItems({"base_link","base","world"}); robot_tip_link_->addItems({"ee_link","tool0","wrist_3_link"}); robot_planning_group_->addItems({"manipulator","arm","preview_group"}); }
  robot_base_link_->setCurrentText(default_robot_base_link(family, model)); robot_tip_link_->setCurrentText(default_robot_tip_link(family, model)); robot_planning_group_->setCurrentText(default_robot_planning_group(family, model));
  const QString status = scaffold ? "SCAFFOLD" : "READY";
  const QString reason = scaffold ? "Placeholder family/model without rich metadata." : "Robot metadata available or defaults applied.";
  robot_readiness_banner_->setText(status + " - " + reason);
  robot_warning_->setText(scaffold ? "Using scaffold robot setup. Review links/planning group." : "");
}

void NewCellWizard::add_environment_asset_row(const QString &id,const QString &asset,const QString &parent,const QString &plink,const QString &clink,const QString &role){int r=env_objects_table_->rowCount(); env_objects_table_->insertRow(r); env_objects_table_->setCellWidget(r,0,new QCheckBox()); static_cast<QCheckBox*>(env_objects_table_->cellWidget(r,0))->setChecked(true); env_objects_table_->setItem(r,1,new QTableWidgetItem(id)); env_objects_table_->setItem(r,2,new QTableWidgetItem(asset)); env_objects_table_->setItem(r,3,new QTableWidgetItem(parent)); env_objects_table_->setItem(r,4,new QTableWidgetItem(plink)); env_objects_table_->setItem(r,5,new QTableWidgetItem(clink)); env_objects_table_->setItem(r,6,new QTableWidgetItem("fixed")); env_objects_table_->setItem(r,7,new QTableWidgetItem(role)); env_objects_table_->setItem(r,8,new QTableWidgetItem("x=0.000, y=0.000, z=0.000, r=0.0000, p=0.0000, y=0.0000")); }
void NewCellWizard::apply_recommended_environment_layout(){ env_objects_table_->setRowCount(0); add_environment_asset_row("workbench_01","workbench/table","world","world","table_link","support_surface"); add_environment_asset_row("source_bin_01","bin","workbench_01","table_link","bin_link","pick_source"); add_environment_asset_row("place_fixture_01","fixture/table","workbench_01","table_link","fixture_link","place_target"); add_environment_asset_row("reject_bin_01","bin","workbench_01","table_link","bin_link","reject_target"); add_environment_asset_row("conveyor_01","conveyor","world","world","conveyor_link","conveyor/source_line"); add_environment_asset_row("camera_01","RealSense D435i","workbench_01","table_link","camera_link","camera_view_zone"); add_environment_asset_row("camera_mount_01","camera_mount","workbench_01","table_link","mount_link","camera_support"); add_environment_asset_row("safety_zone_01","safety_zone","world","world","safety_zone_link","safety_guard"); refresh_environment_parent_options(); refresh_environment_review_table(); refresh_summary(); }
QStringList NewCellWizard::discover_environment_asset_catalog() const { QStringList found; const QStringList roots={"assets/environment","assets/environment_objects"}; for(const QString &rel:roots){ QDir dir(workspace_root_+"/"+rel); const auto entries=dir.entryList(QDir::Dirs|QDir::NoDotAndDotDot); for(const auto &e:entries) found<<e; } found.removeDuplicates(); return found; }
void NewCellWizard::select_environment_substep(int index){ if(env_substep_stack_) env_substep_stack_->setCurrentIndex(std::max(0,index)); }
void NewCellWizard::refresh_environment_review_table(){ env_review_table_->setRowCount(0); for(int r=0;r<env_objects_table_->rowCount();++r){ int rr=env_review_table_->rowCount(); env_review_table_->insertRow(rr); auto id=env_objects_table_->item(r,1)->text(); auto type=env_objects_table_->item(r,2)->text(); auto rel=env_objects_table_->item(r,3)->text()+"."+env_objects_table_->item(r,4)->text()+" -> "+env_objects_table_->item(r,5)->text(); auto role=env_objects_table_->item(r,7)->text(); auto pose=env_objects_table_->item(r,8)->text(); auto *cb=qobject_cast<QCheckBox*>(env_objects_table_->cellWidget(r,0)); auto status=(cb&&cb->isChecked())?"enabled":"disabled"; env_review_table_->setItem(rr,0,new QTableWidgetItem(id)); env_review_table_->setItem(rr,1,new QTableWidgetItem(type)); env_review_table_->setItem(rr,2,new QTableWidgetItem(rel)); env_review_table_->setItem(rr,3,new QTableWidgetItem(role)); env_review_table_->setItem(rr,4,new QTableWidgetItem(pose)); env_review_table_->setItem(rr,5,new QTableWidgetItem(status)); }}

void NewCellWizard::apply_scenario_defaults(){
 if(selected_scenario_id()=="static_table_pick_place" && place_target_->findText("place_fixture_01")>=0) place_target_->setCurrentText("place_fixture_01");
}

void NewCellWizard::refresh_scenario_ui(){
 const bool pick_place = selected_scenario_id()=="static_table_pick_place";
 task_scenario_->setText(QString("%1 (%2)").arg(selected_scenario_label(), selected_scenario_id()));
 scenario_configuration_notice_->setText(pick_place ? QString() : QStringLiteral("Scenario configuration coming in next implementation phase"));
 pick_config_card_->setVisible(pick_place);
 place_config_card_->setVisible(pick_place);
 grasp_motion_card_->setVisible(pick_place);
 manual_object_card_->setVisible(pick_place && selected_object_source_id()=="manual_simulated");
}

void NewCellWizard::refresh_object_source_ui(){
 const bool manual=selected_object_source_id()=="manual_simulated";
 const bool replay=selected_object_source_id()=="recorded_perception";
 manual_object_card_->setVisible(manual && selected_scenario_id()=="static_table_pick_place");
 perception_binding_->setVisible(!manual); minimum_confidence_->setVisible(!manual); required_object_class_->setVisible(!manual);
 if(replay && perception_binding_->text()=="/easy_perception_deployment/epd_localize_output") perception_binding_->setText("generated/detected_objects_replay.yaml");
 else if(!replay && !manual && perception_binding_->text()=="generated/detected_objects_replay.yaml") perception_binding_->setText("/easy_perception_deployment/epd_localize_output");
 dynamic_object_notice_->setText(manual
   ? "Manual fallback: authored primitive geometry and pose are required because perception is absent."
   : "Dynamic observation: object ID, pose/centroid, dimensions, shape, and confidence come from perception when available. Fixed manual dimensions are not required.");
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
 if(selected_scenario_id()!="static_table_pick_place") return blockers;
 if(pick_source_->currentText().isEmpty()) blockers<<"Pick object source is required.";
 if(selected_object_source_id()=="manual_simulated" && !manual_object_geometry_valid()) blockers<<"Manual / simulated object requires an ID, frame, and positive primitive dimensions.";
 if(selected_object_source_id()!="manual_simulated" && perception_binding_->text().trimmed().isEmpty()) blockers<<"Perception binding is required for live or recorded object sources.";
 if(place_target_->currentText().isEmpty()) blockers<<"Place target is required for Pick & Place.";
 return blockers;
}
void NewCellWizard::refresh_environment_parent_options(){ pick_camera_->clear(); pick_source_->clear(); place_target_->clear(); place_frame_link_->clear(); env_parent_combo_->clear(); for(int r=0;r<env_objects_table_->rowCount();++r){auto*cb=qobject_cast<QCheckBox*>(env_objects_table_->cellWidget(r,0)); if(!cb||!cb->isChecked()) continue; auto role=env_objects_table_->item(r,7)->text().toLower(); auto id=env_objects_table_->item(r,1)->text(); env_parent_combo_->addItem(id); if(role.contains("camera")) pick_camera_->addItem(id); if(role.contains("pick_source")||role.contains("conveyor")) pick_source_->addItem(id); if(role.contains("place_target")||role.contains("reject_target")||role.contains("output_bin")||role.contains("fixture")||role.contains("support")) place_target_->addItem(id);} pick_source_->addItem("manual"); place_frame_link_->addItems({"target_link","top_surface","world"}); if(pick_camera_->findText("camera_01")>=0){ pick_zone_source_->setCurrentText("Camera view zone"); pick_camera_->setCurrentText("camera_01"); } if(pick_source_->findText("source_bin_01")>=0) pick_source_->setCurrentText("source_bin_01"); apply_scenario_defaults(); refresh_environment_review_table(); }

fs::path NewCellWizard::scenes_root_path() const { return fs::path(workspace_root_.toStdString()) / "src" / "scenes"; }
QString NewCellWizard::scene_name_error() const { const QString n=scene_name_->text().trimmed(); if(n.isEmpty()) return "Scene/package name is required."; if(!is_valid_package_name(n)) return "Use lowercase letters, numbers, underscores only; must start with a letter."; return ""; }
QString NewCellWizard::scene_name_warning() const { const QString n=scene_name_->text().trimmed(); if(n.isEmpty()) return ""; const fs::path p=scenes_root_path()/n.toStdString(); if(fs::exists(p)) return "Warning: output folder already exists. Creation is blocked to avoid overwrite."; return ""; }

void NewCellWizard::refresh_validation(){
 const int row=steps_->currentRow(); const QString err=scene_name_error(); scene_error_->setText(err); scene_warning_->setText(scene_name_warning());
 const bool name_valid=err.isEmpty()&&scene_name_warning().isEmpty(); back_->setEnabled(row>0); next_->setEnabled(row<5&&(row!=0||name_valid));
 const auto blockers=readiness_blockers(); const auto warnings=readiness_warnings();
 QString readiness=selected_scenario_id()=="static_table_pick_place"?"READY":"AUTHORING ONLY"; if(!blockers.isEmpty()) readiness="BLOCKED"; else if(!warnings.isEmpty() && selected_scenario_id()=="static_table_pick_place") readiness="WARNINGS";
 task_readiness_label_->setText(readiness);
 task_warning_->setText((blockers+warnings).join("\n"));
 create_->setEnabled(name_valid); create_open_->setEnabled(name_valid);
 refresh_summary();
}

void NewCellWizard::refresh_summary(){
 QString readiness=task_readiness_label_?task_readiness_label_->text():"READY";
 summary_->setText(QString("<b>Application / Scenario:</b> %1 (%2)<br/><b>Object source:</b> %3 (%4)<br/><b>Robot:</b> family=%5 model=%6 | base=%7 tip=%8 planning=%9<br/><b>Tool:</b> family=%10 model=%11 | attach=%12 | tcp=%13 | type=%14 | mount rpy=%15,%16,%17 | readiness=%18<br/><b>Authoring status:</b> %19<br/><b>Warnings/Blockers:</b><br/>%20").arg(selected_scenario_label(),selected_scenario_id(),object_source_mode_->currentText(),selected_object_source_id(),robot_family_->currentText(),robot_->currentText(),robot_base_link_->currentText(),robot_tip_link_->currentText(),robot_planning_group_->currentText(),ee_family_->currentText(),ee_->currentText(),ee_attach_link_->currentText(),ee_tcp_link_->currentText(),ee_type_->currentText(),ee_roll_->text(),ee_pitch_->text(),ee_yaw_->text(),ee_readiness_banner_?ee_readiness_banner_->text():"READY",readiness,task_warning_->text().toHtmlEscaped().replace("\n","<br/>")));
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
 const bool pick_place_scenario=selected_scenario_id()=="static_table_pick_place";
 const bool place_target_missing=pick_place_scenario && place_target_->currentText().trimmed().isEmpty();
 const bool unknown_links=warning_text.contains("unknown");
 QString readiness=pick_place_scenario?"READY":"AUTHORING_ONLY";
 if(place_target_missing) readiness="BLOCKED";
 else if(unknown_links) readiness="WARNINGS";

 std::ofstream out((scene_dir/"environment.yaml").string());
 out<<"scene_name: "<<scene_name_->text().trimmed().toStdString()<<"\n";
 out<<"robot: "<<robot_->currentText().toStdString()<<"\n";
 out<<"end_effector: "<<ee_->currentText().toStdString()<<"\n";
out<<"workcell_studio:\n";
 out<<"  scenario:\n";
 out<<"    id: "<<yaml_scalar(selected_scenario_id())<<"\n";
 out<<"    category: "<<yaml_scalar(selected_scenario_category())<<"\n";
 out<<"    label: "<<yaml_scalar(selected_scenario_label())<<"\n";
 out<<"    authoring_selection_only: true\n";
 out<<"    runtime_ready: false\n";
 out<<"  robot:\n";
 out<<"    family: "<<yaml_scalar(robot_family_->currentText())<<"\n";
 out<<"    model: "<<yaml_scalar(robot_->currentText())<<"\n";
 out<<"    base: "<<yaml_scalar(robot_base_link_->currentText())<<"\n";
 out<<"    tip: "<<yaml_scalar(robot_tip_link_->currentText())<<"\n";
 out<<"    planning_group: "<<yaml_scalar(robot_planning_group_->currentText())<<"\n";
 out<<"    readiness: "<<yaml_scalar(robot_readiness_banner_->text())<<"\n";
 out<<"  tool:\n";
 out<<"    family: "<<yaml_scalar(ee_family_->currentText())<<"\n";
 out<<"    model: "<<yaml_scalar(ee_->currentText())<<"\n";
 out<<"    attach: "<<yaml_scalar(ee_attach_link_->currentText())<<"\n";
 out<<"    tcp: "<<yaml_scalar(ee_tcp_link_->currentText())<<"\n";
 out<<"    type: "<<yaml_scalar(ee_type_->currentText())<<"\n";
 out<<"    mount_pose:\n";
 out<<"      xyz: ["<<pose_number(ee_x_)<<", "<<pose_number(ee_y_)<<", "<<pose_number(ee_z_)<<"]\n";
 out<<"      rpy: ["<<pose_number(ee_roll_)<<", "<<pose_number(ee_pitch_)<<", "<<pose_number(ee_yaw_)<<"]\n";
 out<<"    readiness: "<<yaml_scalar(ee_readiness_banner_ ? ee_readiness_banner_->text() : "READY")<<"\n";
 out<<"  create_cell_metadata:\n";
 out<<"    robot_family: "<<yaml_scalar(robot_family_->currentText())<<"\n";
 out<<"    robot_model: "<<yaml_scalar(robot_->currentText())<<"\n";
 out<<"    robot_base_link: "<<yaml_scalar(robot_base_link_->currentText())<<"\n";
 out<<"    robot_tip_link: "<<yaml_scalar(robot_tip_link_->currentText())<<"\n";
 out<<"    robot_planning_group: "<<yaml_scalar(robot_planning_group_->currentText())<<"\n";
 out<<"    robot_readiness: "<<yaml_scalar(robot_readiness_banner_->text())<<"\n";
 out<<"  frames:\n";
 out<<"    robot_base_link: "<<yaml_scalar(robot_base_link_->currentText())<<"\n";
 out<<"    robot_tip_link: "<<yaml_scalar(robot_tip_link_->currentText())<<"\n";
 out<<"    planning_group: "<<yaml_scalar(robot_planning_group_->currentText())<<"\n";
 out<<"    end_effector_attach_link: "<<yaml_scalar(ee_attach_link_->currentText())<<"\n";
 out<<"    end_effector_tcp_link: "<<yaml_scalar(ee_tcp_link_->currentText())<<"\n";
 out<<"    end_effector_family: "<<yaml_scalar(ee_family_->currentText())<<"\n";
 out<<"    end_effector_model: "<<yaml_scalar(ee_->currentText())<<"\n";
 out<<"    end_effector_type: "<<yaml_scalar(ee_type_->currentText())<<"\n";
 out<<"    end_effector_readiness: "<<yaml_scalar(ee_readiness_banner_ ? ee_readiness_banner_->text() : "READY")<<"\n";
 out<<"    robot_mount_pose:\n";
 out<<"      xyz: ["<<pose_number(robot_x_)<<", "<<pose_number(robot_y_)<<", "<<pose_number(robot_z_)<<"]\n";
 out<<"      rpy: ["<<pose_number(robot_roll_)<<", "<<pose_number(robot_pitch_)<<", "<<pose_number(robot_yaw_)<<"]\n";
 out<<"    tool_mount_pose:\n";
   out<<"      xyz: ["<<pose_number(ee_x_)<<", "<<pose_number(ee_y_)<<", "<<pose_number(ee_z_)<<"]\n";
   out<<"      rpy: ["<<pose_number(ee_roll_)<<", "<<pose_number(ee_pitch_)<<", "<<pose_number(ee_yaw_)<<"]\n";

 out<<"  perception:\n";
 out<<"    mode: "<<yaml_scalar(selected_object_source_id())<<"\n";
 out<<"    selected_topics:\n";
 out<<"      detection_source: "<<yaml_scalar(selected_object_source_id()=="manual_simulated" ? QString() : perception_binding_->text().trimmed())<<"\n";
 out<<"      camera: "<<yaml_scalar(pick_camera_->currentText())<<"\n";
 out<<"    profile_metadata:\n";
 out<<"      pick_zone_source: "<<yaml_scalar(pick_zone_source_->currentText())<<"\n";
 out<<"      scene_readiness: "<<yaml_scalar(readiness)<<"\n";

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

 if(pick_place_scenario){
  out<<"  pick_zone:\n";
  out<<"    source: "<<yaml_scalar(pick_zone_source_->currentText())<<"\n";
  out<<"    camera: "<<yaml_scalar(pick_camera_->currentText())<<"\n";
  out<<"    pick_object_source: "<<yaml_scalar(pick_source_->currentText())<<"\n";
  out<<"    zone_frame: "<<yaml_scalar(pick_zone_frame_->currentText())<<"\n";
  out<<"    detection_source: "<<yaml_scalar(selected_object_source_id())<<"\n";
  out<<"    object_source:\n";
  out<<"      mode: "<<yaml_scalar(selected_object_source_id())<<"\n";
  out<<"      observation_contract: "<<yaml_scalar("workcell_perception_snapshot/v1")<<"\n";
  out<<"      source_zone_ref: "<<yaml_scalar(pick_source_->currentText())<<"\n";
  out<<"      geometry_origin: "<<yaml_scalar(selected_object_source_id()=="manual_simulated" ? "manual_authored" : "perception_observation")<<"\n";
  out<<"      dynamic_pose: "<<(selected_object_source_id()=="manual_simulated"?"false":"true")<<"\n";
  out<<"      dynamic_geometry: "<<(selected_object_source_id()=="manual_simulated"?"false":"true")<<"\n";
  if(selected_object_source_id()!="manual_simulated"){
   out<<"      perception_binding: "<<yaml_scalar(perception_binding_->text().trimmed())<<"\n";
   out<<"      required_class: "<<yaml_scalar(required_object_class_->text().trimmed())<<"\n";
   out<<"      minimum_confidence: "<<minimum_confidence_->value()<<"\n";
  } else {
   out<<"      manual_observation:\n";
   out<<"        object_id: "<<yaml_scalar(manual_object_id_->text().trimmed())<<"\n";
   out<<"        shape: "<<yaml_scalar(manual_object_shape_->currentText())<<"\n";
   out<<"        dimensions: ["<<manual_object_x_->value()<<", "<<manual_object_y_->value()<<", "<<manual_object_z_->value()<<"]\n";
   out<<"        pose:\n";
   out<<"          frame_id: "<<yaml_scalar(manual_object_frame_->text().trimmed())<<"\n";
   out<<"          xyz: ["<<manual_pose_x_->value()<<", "<<manual_pose_y_->value()<<", "<<manual_pose_z_->value()<<"]\n";
   out<<"          rpy: [0, 0, 0]\n";
  }
  out<<"      planning_scene_conversion: pending_runtime_conversion\n";

  out<<"  place_zone:\n";
  out<<"    target: "<<yaml_scalar(place_target_->currentText())<<"\n";
  out<<"    place_frame_link: "<<yaml_scalar(place_frame_link_->currentText())<<"\n";
  out<<"    placement_mode: "<<yaml_scalar(placement_mode_->currentText())<<"\n";
  out<<"    placement_alignment: "<<yaml_scalar(placement_alignment_->currentText())<<"\n";
 }

 out<<"  task_intent:\n";
 out<<"    scenario_id: "<<yaml_scalar(selected_scenario_id())<<"\n";
 out<<"    task_family: "<<yaml_scalar(selected_scenario_category())<<"\n";
 out<<"    configuration_status: "<<yaml_scalar(pick_place_scenario?"configured":"coming_next_implementation_phase")<<"\n";
 out<<"    intent_text: "<<yaml_scalar(task_intent_text_->toPlainText().trimmed())<<"\n";
 if(pick_place_scenario){
  out<<"    grasp_strategy: "<<yaml_scalar(grasp_strategy_->currentText())<<"\n";
  out<<"    object_binding_ref: "<<yaml_scalar("workcell_studio.pick_zone.object_source")<<"\n";
  out<<"    approach_axis: "<<yaml_scalar(approach_axis_->currentText())<<"\n";
  out<<"    release_strategy: "<<yaml_scalar(release_strategy_->currentText())<<"\n";
  out<<"    approach_distance_m: "<<approach_distance_->value()<<"\n";
  out<<"    retreat_distance_m: "<<retreat_distance_->value()<<"\n";
 }
 out<<"    readiness: "<<yaml_scalar(readiness)<<"\n";

 out<<"  warnings:\n";
 if(warning_text.isEmpty()) out<<"    - "<<yaml_scalar("none")<<"\n";
 else out<<"    - "<<yaml_scalar(warning_text)<<"\n";

 out<<"fake_hardware_first: true\nreal_robot_locked: true\nruntime_execution_enabled: false\nscaffold_only: true\n";
 out.close();
 result_.created=true; result_.open_in_scene_builder=open_in_builder; result_.scene_name=scene_name_->text().trimmed(); result_.scene_dir=scene_dir; return true;
}
