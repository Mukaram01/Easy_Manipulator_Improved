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
#include <QScreen>
#include <QShowEvent>
#include <QStyle>
#include <QWindow>
#include <QSet>
#include <QVBoxLayout>

#include <array>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <QSaveFile>
#include <QTemporaryDir>
#include <QProcess>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QtMath>
#include <iomanip>
#include "scene_select_paths.h"
#include <regex>

namespace fs = boost::filesystem;

NewCellWizard::NewCellWizard(const QString &workspace_root, QWidget *parent): QDialog(parent), workspace_root_(workspace_root){ build_ui(); refresh_validation(); }

void NewCellWizard::fit_available_screen()
{
  auto *display = screen();
  if (!display) return;
  connect(display, &QScreen::availableGeometryChanged, this,
    &NewCellWizard::fit_available_screen, Qt::UniqueConnection);
  const auto available = display->availableGeometry();
  const auto frame = windowHandle() ? windowHandle()->frameMargins() : QMargins();
  // Qt screen geometry is in logical pixels, including when desktop scaling is enabled.
  const int title_height = std::max(frame.top(), style()->pixelMetric(QStyle::PM_TitleBarHeight));
  const QSize limit(std::max(1, available.width() - frame.left() - frame.right()),
    std::max(1, available.height() - title_height - frame.bottom()));
  setMinimumWidth(std::min(800, limit.width()));
  setMaximumSize(limit);
  resize(size().boundedTo(limit));
  const auto bounds = frameGeometry();
  move(pos() + QPoint(std::max(available.left(), std::min(bounds.left(), available.right() - bounds.width() + 1)) - bounds.left(),
    std::max(available.top(), std::min(bounds.top(), available.bottom() - bounds.height() + 1)) - bounds.top()));
}

void NewCellWizard::showEvent(QShowEvent *event)
{
  QDialog::showEvent(event);
  if (windowHandle()) connect(windowHandle(), &QWindow::screenChanged, this,
    &NewCellWizard::fit_available_screen, Qt::UniqueConnection);
  fit_available_screen();
}

bool NewCellWizard::is_valid_package_name(const QString &name){ static const std::regex re("^[a-z][a-z0-9_]*$"); return std::regex_match(name.toStdString(), re);} 
QStringList NewCellWizard::recommended_environment_assets(){ return {"workbench_01","source_bin_01","place_fixture_01","camera_01","safety_zone_01"}; }

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

QStringList NewCellWizard::load_qualified_physics_materials(const QString &catalog_path) {
  QStringList materials;
  QFile file(catalog_path);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return materials;
  bool in_materials = false;
  const QRegularExpression key_re("^  ([A-Za-z][A-Za-z0-9_]*):\\s*$");
  while (!file.atEnd()) {
    const QString line = QString::fromUtf8(file.readLine()).replace('\n', QString());
    if (line.trimmed().isEmpty() || line.trimmed().startsWith('#')) continue;
    if (!line.startsWith(' ')) { in_materials = line.startsWith("materials:"); continue; }
    const auto match = key_re.match(line);
    if (in_materials && match.hasMatch()) materials << match.captured(1);
  }
  return materials;
}

QString NewCellWizard::qualified_physics_catalog_path() const {
  const QStringList candidates = {
    workspace_root_ + "/catalog/simulation/qualified_physics.yaml",
    workspace_root_ + "/src/easy_manipulation_deployment/catalog/simulation/qualified_physics.yaml",
    QDir::current().absoluteFilePath("catalog/simulation/qualified_physics.yaml")};
  for (const QString &candidate : candidates) if (QFileInfo::exists(candidate)) return candidate;
  return candidates.front();
}

QString NewCellWizard::selected_scenario_id() const { return application_scenario_ ? application_scenario_->currentData(Qt::UserRole).toString() : QString(); }
QString NewCellWizard::selected_scenario_category() const { return application_scenario_ ? application_scenario_->currentData(Qt::UserRole + 1).toString() : QString(); }
QString NewCellWizard::selected_scenario_label() const { return application_scenario_ ? application_scenario_->currentText() : QString(); }
bool NewCellWizard::sorting_scenario() const { return selected_scenario_id() == "multi_bin_sorting_cell"; }
bool NewCellWizard::configured_scenario() const { return selected_scenario_id() == "static_table_pick_place" || sorting_scenario(); }
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
 auto mk_page=[&](QWidget*w){
   for (auto *label : w->findChildren<QLabel *>()) label->setWordWrap(true);
   for (auto *form : w->findChildren<QFormLayout *>()) form->setRowWrapPolicy(QFormLayout::WrapLongRows);
   auto *content = new QWidget;
   auto *layout = new QVBoxLayout(content);
   layout->addWidget(w); layout->addStretch(1);
   auto *scroll = new QScrollArea(stack_);
   scroll->setWidgetResizable(true);
   scroll->setFrameShape(QFrame::NoFrame);
   scroll->setWidget(content);
   stack_->addWidget(scroll);
 };
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

 auto *s1=new QWidget(this); auto *f1=new QFormLayout(s1); display_name_=new QLineEdit(); scene_name_=new QLineEdit(); display_name_->setObjectName("newCellDisplayName"); scene_name_->setObjectName("newCellSceneName"); description_=new QTextEdit(); application_scenario_=new QComboBox(); application_scenario_->setObjectName("applicationScenarioCombo");
 const auto scenario_choices = load_industrial_scenario_choices(scenario_catalog_path());
 for (const auto &choice : scenario_choices) { application_scenario_->addItem(choice.label, choice.id); application_scenario_->setItemData(application_scenario_->count()-1, choice.category, Qt::UserRole+1); }
 output_path_=new QLineEdit(QString::fromStdString(scenes_root_path().string())); scene_error_=new QLabel(); scene_warning_=new QLabel(); scene_name_->setPlaceholderText("e.g. my_pick_cell (lowercase letters, numbers, _)"); scene_error_->setObjectName("newCellSceneNameError"); f1->addRow(new QLabel("Fields marked <b>*</b> are required.")); f1->addRow("Cell display name",display_name_); f1->addRow("Scene/package name *",scene_name_); f1->addRow("",scene_error_); f1->addRow("",scene_warning_); f1->addRow("Description",description_); f1->addRow("Application / Scenario",application_scenario_); f1->addRow("Output scenes path",output_path_);
 // The only supported location; a cell kept elsewhere would not be discovered or editable in the 3D Scene Builder.
 output_path_->setReadOnly(true);
 auto *local_note=new QLabel("The cell is saved on this PC in the Workcell Studio scenes folder. New cells are not tracked by "
   "Git: they are not uploaded to GitHub and do not sync to other computers. To use a cell on another PC, copy its folder "
   "or export it.");
 local_note->setObjectName("newCellLocalStorageNote"); f1->addRow("",local_note); mk_page(s1);

 auto *s2=new QWidget(this); auto *f2=new QFormLayout(s2); robot_family_=new QComboBox(); robot_family_->setObjectName("newCellRobotFamily"); robot_=new QComboBox(); robot_->setObjectName("newCellRobotModel"); robot_base_link_=new QComboBox(); robot_tip_link_=new QComboBox(); robot_planning_group_=new QComboBox();
 for(auto*c:{robot_base_link_,robot_tip_link_,robot_planning_group_}) c->setEditable(true);
 robot_controller_name_=new QLineEdit(); robot_controller_name_->setReadOnly(true);
 auto *robot_pose=mk_pose_editor(robot_x_,robot_y_,robot_z_,robot_roll_,robot_pitch_,robot_yaw_);
 robot_advanced_group_=new QGroupBox("Advanced Robot Frames"); robot_advanced_group_->setCheckable(true); robot_advanced_group_->setChecked(false); auto*raf=new QFormLayout(robot_advanced_group_); raf->addRow("Robot base link",robot_base_link_); raf->addRow("Robot end-effector/tip link",robot_tip_link_); raf->addRow("Planning group",robot_planning_group_); raf->addRow("Controller name",robot_controller_name_);
 robot_warning_=new QLabel(); robot_warning_->setWordWrap(true); robot_readiness_banner_=new QLabel(); robot_readiness_banner_->setWordWrap(true); robot_readiness_banner_->setObjectName("newCellRobotReadiness");
 f2->addRow("Robot family",robot_family_); f2->addRow("Robot model",robot_); f2->addRow("Robot base pose",robot_pose); f2->addRow(robot_advanced_group_); f2->addRow("Robot readiness",robot_readiness_banner_); f2->addRow("",robot_warning_); mk_page(s2);

 auto *s3=new QWidget(this); auto *f3=new QFormLayout(s3); ee_family_=new QComboBox(); ee_family_->setObjectName("newCellToolFamily"); ee_=new QComboBox(); ee_->setObjectName("newCellToolModel"); ee_attach_link_=new QComboBox(); ee_tcp_link_=new QComboBox(); ee_attach_link_->setEditable(true); ee_tcp_link_->setEditable(true); ee_type_=new QComboBox();
 auto *ee_pose=mk_pose_editor(ee_x_,ee_y_,ee_z_,ee_roll_,ee_pitch_,ee_yaw_);
 ee_advanced_group_=new QGroupBox("Advanced Tool Frames"); ee_advanced_group_->setCheckable(true); ee_advanced_group_->setChecked(false); auto*eaf=new QFormLayout(ee_advanced_group_); eaf->addRow("End-effector attach/base link",ee_attach_link_); eaf->addRow("End-effector TCP/tool link",ee_tcp_link_); eaf->addRow("End-effector type",ee_type_);
 ee_readiness_banner_=new QLabel(); ee_readiness_banner_->setWordWrap(true); ee_readiness_banner_->setObjectName("newCellToolReadiness");
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
 manual_object_card_=new QGroupBox("Manual / Simulated Object Fallback",s5); manual_object_card_->setObjectName("manualObjectCard"); auto *manual_form=new QFormLayout(manual_object_card_); manual_object_id_=new QLineEdit("manual_object_01"); manual_object_id_->setObjectName("manualObjectId"); manual_object_shape_=new QComboBox(); manual_object_shape_->addItems({"box","cylinder","sphere"}); manual_object_x_=mk_pose_spin(3," m","Manual object X dimension"); manual_object_y_=mk_pose_spin(3," m","Manual object Y dimension"); manual_object_z_=mk_pose_spin(3," m","Manual object Z dimension"); manual_object_x_->setObjectName("manualObjectDimensionX"); manual_object_y_->setObjectName("manualObjectDimensionY"); manual_object_z_->setObjectName("manualObjectDimensionZ"); manual_object_x_->setRange(0.0,1000.0); manual_object_y_->setRange(0.0,1000.0); manual_object_z_->setRange(0.0,1000.0); manual_object_x_->setValue(0.05); manual_object_y_->setValue(0.05); manual_object_z_->setValue(0.05); manual_object_frame_=new QLineEdit("world"); manual_object_frame_->setObjectName("manualObjectFrame"); manual_pose_x_=mk_pose_spin(3," m","Manual object pose X"); manual_pose_y_=mk_pose_spin(3," m","Manual object pose Y"); manual_pose_z_=mk_pose_spin(3," m","Manual object pose Z"); auto *manual_dims=new QWidget(); auto *manual_dims_layout=new QHBoxLayout(manual_dims); manual_dims_layout->setContentsMargins(0,0,0,0); manual_dims_layout->addWidget(manual_object_x_); manual_dims_layout->addWidget(manual_object_y_); manual_dims_layout->addWidget(manual_object_z_); auto *manual_pose=new QWidget(); auto *manual_pose_layout=new QHBoxLayout(manual_pose); manual_pose_layout->setContentsMargins(0,0,0,0); manual_pose_layout->addWidget(manual_pose_x_); manual_pose_layout->addWidget(manual_pose_y_); manual_pose_layout->addWidget(manual_pose_z_); manual_object_class_=new QLineEdit(); manual_object_class_->setObjectName("manualObjectClass"); manual_object_class_->setPlaceholderText("e.g. part (used by the task class filter)"); manual_object_material_=new QComboBox(); manual_object_material_->setObjectName("manualObjectMaterial"); manual_object_material_->addItem("Not physics-simulated", QString()); for(const QString &material: load_qualified_physics_materials(qualified_physics_catalog_path())) manual_object_material_->addItem(material, material); manual_object_material_->setToolTip("Qualified physics material. Without one the cell can be planned but not run in the physics simulation."); manual_form->addRow("Object ID / name",manual_object_id_); manual_form->addRow("Object class",manual_object_class_); manual_form->addRow("Primitive shape",manual_object_shape_); manual_form->addRow("Physics material",manual_object_material_); manual_form->addRow("Dimensions X / Y / Z",manual_dims); manual_form->addRow("Pose frame",manual_object_frame_); manual_form->addRow("Pose X / Y / Z",manual_pose);
 // Sorting: the cell's simulated objects. Each is the qualified physics test block; its appearance is only what the
 // cell camera renders, and the class always comes from perception (EPD).
 test_objects_card_=new QGroupBox("Simulated Test Objects",s5); test_objects_card_->setObjectName("testObjectsCard"); auto *test_objects_layout=new QVBoxLayout(test_objects_card_);
 test_objects_table_=new QTableWidget(0,5,test_objects_card_); test_objects_table_->setObjectName("testObjectsTable"); test_objects_table_->setHorizontalHeaderLabels({"Object ID","Appearance","X (m)","Y (m)","Yaw (deg)"}); test_objects_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch); test_objects_table_->setMinimumHeight(160);
 auto *test_object_buttons=new QHBoxLayout(); auto *add_test_object=new QPushButton("Add Object"); auto *remove_test_object=new QPushButton("Remove Object"); test_object_buttons->addWidget(add_test_object); test_object_buttons->addWidget(remove_test_object); test_object_buttons->addStretch(1);
 test_objects_layout->addWidget(new QLabel("Each object is the qualified 25 mm physics test block resting on the table. Its appearance is what the cell camera renders; the class always comes from perception (EPD)."));
 test_objects_layout->addWidget(test_objects_table_); test_objects_layout->addLayout(test_object_buttons);
 place_target_=new QComboBox(); place_target_->setEditable(true); place_frame_link_=new QComboBox(); place_frame_link_->setEditable(true); placement_mode_=new QComboBox(); placement_mode_->addItems({"top_of_surface","inside_bin","fixture_alignment","conveyor_dropoff","manual_pose"}); placement_alignment_=new QComboBox(); placement_alignment_->addItems({"none","fixed_yaw","match_object_orientation","feature_based"});
 place_config_card_=new QGroupBox("Place Configuration",s5); auto *place_form=new QFormLayout(place_config_card_); place_form->addRow("Place target",place_target_); place_form->addRow("Place frame/link",place_frame_link_); place_form->addRow("Placement mode",placement_mode_); place_form->addRow("Alignment",placement_alignment_);
 grasp_strategy_=new QComboBox(); grasp_strategy_->setObjectName("graspStrategyCombo"); approach_axis_=new QComboBox(); approach_axis_->addItems({"z_down","z_up","x_forward","y_left"}); release_strategy_=new QComboBox(); approach_distance_=mk_pose_spin(3," m","Approach distance"); retreat_distance_=mk_pose_spin(3," m","Retreat distance"); approach_distance_->setValue(0.10); retreat_distance_->setValue(0.10); task_warning_=new QLabel(); task_warning_->setWordWrap(true); task_readiness_label_=new QLabel(); task_readiness_label_->setWordWrap(true);
 grasp_motion_card_=new QGroupBox("Grasp and Motion Intent",s5); auto *grasp_form=new QFormLayout(grasp_motion_card_); grasp_form->addRow("Grasp strategy",grasp_strategy_); grasp_form->addRow("Approach axis",approach_axis_); grasp_form->addRow("Release strategy",release_strategy_); grasp_form->addRow("Approach distance",approach_distance_); grasp_form->addRow("Retreat distance",retreat_distance_); grasp_form->addRow("Readiness",task_readiness_label_); grasp_form->addRow("Warnings / blockers",task_warning_);
 v5->addWidget(task_template_card); v5->addWidget(pick_config_card_); v5->addWidget(manual_object_card_); v5->addWidget(test_objects_card_); v5->addWidget(place_config_card_); v5->addWidget(grasp_motion_card_); mk_page(s5);

 auto *s6=new QWidget(this); auto *v6=new QVBoxLayout(s6); summary_=new QLabel(); summary_->setWordWrap(true); summary_->setFrameShape(QFrame::StyledPanel); v6->addWidget(summary_); mk_page(s6);

 auto*nav=new QHBoxLayout(); back_=new QPushButton("Back"); next_=new QPushButton("Next"); create_=new QPushButton("Create Cell"); create_open_=new QPushButton("Create and Open"); auto*cancel=new QPushButton("Cancel"); create_blocked_=new QLabel(); create_blocked_->setObjectName("newCellCreateBlocked"); create_blocked_->setWordWrap(true); create_blocked_->setTextFormat(Qt::RichText); nav->addWidget(back_); nav->addWidget(next_); nav->addWidget(create_blocked_,1); nav->addWidget(create_); nav->addWidget(create_open_); nav->addWidget(cancel); root->addLayout(nav);
 connect(steps_,&QListWidget::currentRowChanged,stack_,&QStackedWidget::setCurrentIndex); connect(steps_,&QListWidget::currentRowChanged,this,&NewCellWizard::refresh_validation); steps_->setCurrentRow(0);
 connect(scene_name_, &QLineEdit::textChanged, this, [this](const QString &) { refresh_validation(); });
 // Until the package name is typed, propose one from the display name.
 connect(scene_name_, &QLineEdit::textEdited, this, [this](const QString &) { scene_name_edited_ = true; });
 connect(display_name_, &QLineEdit::textEdited, this, [this](const QString &text) {
   if (scene_name_edited_) return;
   QString slug = text.trimmed().toLower();
   slug.replace(QRegularExpression("[^a-z0-9]+"), "_");
   slug.remove(QRegularExpression("^[^a-z]+|_+$"));
   scene_name_->setText(slug);
 });
 summary_->setTextFormat(Qt::RichText); summary_->setObjectName("newCellReviewSummary");
 connect(summary_, &QLabel::linkActivated, this, &NewCellWizard::go_to_blocker);
 connect(create_blocked_, &QLabel::linkActivated, this, &NewCellWizard::go_to_blocker);
 connect(output_path_, &QLineEdit::textChanged, this, [this](const QString &) { refresh_validation(); });
 connect(back_,&QPushButton::clicked,this,[this]{steps_->setCurrentRow(std::max(0,steps_->currentRow()-1));}); connect(next_,&QPushButton::clicked,this,[this]{steps_->setCurrentRow(std::min(5,steps_->currentRow()+1));}); connect(cancel,&QPushButton::clicked,this,&QDialog::reject);
 connect(create_,&QPushButton::clicked,this,[this]{if(create_scene_scaffold(false))accept();}); connect(create_open_,&QPushButton::clicked,this,[this]{if(create_scene_scaffold(true))accept();});
 connect(rec,&QPushButton::clicked,this,&NewCellWizard::apply_recommended_environment_layout);
 connect(clear,&QPushButton::clicked,this,[this]{recommended_profile_.clear(); env_objects_table_->setRowCount(0); refresh_environment_parent_options(); refresh_summary();});
 connect(env_edit_selected_button_,&QPushButton::clicked,this,&NewCellWizard::refresh_summary);
 connect(application_scenario_,QOverload<int>::of(&QComboBox::currentIndexChanged),this,[this](int){if(!recommended_profile_.isEmpty()) apply_recommended_environment_layout(); apply_scenario_defaults(); refresh_scenario_ui(); refresh_validation();});
 connect(add_test_object,&QPushButton::clicked,this,[this]{ add_test_object_row(QString("object_%1").arg(test_objects_table_->rowCount()+1),"cup",0.36,-0.33,90.0); refresh_validation(); });
 connect(remove_test_object,&QPushButton::clicked,this,[this]{ if(test_objects_table_->currentRow()>=0) test_objects_table_->removeRow(test_objects_table_->currentRow()); refresh_validation(); });
 connect(test_objects_table_,&QTableWidget::itemChanged,this,[this](QTableWidgetItem*){ refresh_validation(); });
 connect(object_source_mode_,QOverload<int>::of(&QComboBox::currentIndexChanged),this,[this](int){refresh_object_source_ui(); refresh_validation();});
 for(auto *spin:{manual_object_x_,manual_object_y_,manual_object_z_}) connect(spin,QOverload<double>::of(&QDoubleSpinBox::valueChanged),this,[this](double){refresh_validation();});
 connect(manual_object_id_,&QLineEdit::textChanged,this,[this](const QString&){refresh_validation();}); connect(manual_object_frame_,&QLineEdit::textChanged,this,[this](const QString&){refresh_validation();});
 connect(add_asset,&QPushButton::clicked,this,[this]{ const QString id=env_add_asset_combo_->currentText().trimmed(); if(id.isEmpty()) return; add_environment_asset_row(id,id, "world","world","asset_link","custom"); refresh_environment_parent_options(); refresh_environment_review_table(); });
 connect(env_substeps_,&QListWidget::currentRowChanged,this,&NewCellWizard::select_environment_substep);
 connect(env_edit_selected_button_,&QPushButton::clicked,this,[this]{ if(env_review_table_->currentRow()>=0){ env_substeps_->setCurrentRow(1); }});
 // A robot or tool change re-derives the pair's frames and readiness, and its reviewed layout when it has one.
 auto equipment_changed=[this]{ apply_equipment_selection(); apply_recommended_environment_layout(); refresh_validation(); };
 connect(robot_family_,QOverload<int>::of(&QComboBox::currentIndexChanged),this,[this,equipment_changed](int){ refresh_robot_model_options(QString()); refresh_tool_model_options(QString()); equipment_changed(); });
 connect(robot_,QOverload<int>::of(&QComboBox::currentIndexChanged),this,[this,equipment_changed](int){ refresh_tool_model_options(QString()); equipment_changed(); });
 connect(ee_family_,QOverload<int>::of(&QComboBox::currentIndexChanged),this,[this,equipment_changed](int){ refresh_tool_model_options(QString()); equipment_changed(); });
 connect(ee_,QOverload<int>::of(&QComboBox::currentIndexChanged),this,[equipment_changed](int){ equipment_changed(); });
 for(auto *combo:{robot_base_link_,robot_tip_link_,robot_planning_group_,ee_attach_link_,ee_tcp_link_}) connect(combo,&QComboBox::currentTextChanged,this,[this](const QString&){ refresh_validation(); });
 // Default: the first plan-capable (reviewed) pair, else the first discovered one.
 load_equipment();
 QJsonObject first;
 for(const auto &c : equipment_.value("combinations").toArray()) if(first.isEmpty() || (c.toObject().value("status")=="PLAN_CAPABLE" && first.value("status")!="PLAN_CAPABLE")) first=c.toObject();
 env_substeps_->setCurrentRow(0);
 if(!select_equipment(first.value("robot").toString(), first.value("tool").toString())) { refresh_robot_model_options(QString()); refresh_tool_model_options(QString()); apply_equipment_selection(); apply_recommended_environment_layout(); } select_object_source_by_id("manual_simulated"); select_scenario_by_id("static_table_pick_place"); refresh_scenario_ui(); refresh_object_source_ui();
}
void NewCellWizard::load_equipment() {
  equipment_ = {}; equipment_error_.clear();
  QProcess helper;
  helper.start("python3", {profile_helper_path(), "--equipment"});
  if (!helper.waitForFinished(30000)) { helper.kill(); helper.waitForFinished(); }
  if (helper.exitStatus() == QProcess::NormalExit && helper.exitCode() == 0)
    equipment_ = QJsonDocument::fromJson(helper.readAllStandardOutput()).object();
  if (equipment_.value("schema").toString() != "new_cell_equipment/v1")
    equipment_error_ = "Robot and tool choices could not be loaded from the capability catalog (" + profile_helper_path() +
      " --equipment): " + QString::fromUtf8(helper.readAllStandardError()).trimmed() + " Check the catalog and reopen New Cell.";
  auto families = [](const QJsonArray &items) {
    QStringList out; for (const auto &item : items) { const auto f = item.toObject().value("family").toString(); if (!out.contains(f)) out << f; } return out; };
  for (auto [combo, items] : {std::pair{robot_family_, equipment_.value("robots").toArray()}, std::pair{ee_family_, equipment_.value("tools").toArray()}}) {
    const QSignalBlocker block(combo); combo->clear();
    for (const auto &family : families(items)) combo->addItem(QString(family).replace('_', ' '), family);
  }
}
QJsonObject NewCellWizard::equipment_record(const char *kind, const QString &id) const {
  for (const auto &item : equipment_.value(kind).toArray()) if (item.toObject().value("id").toString() == id) return item.toObject();
  return {};
}
QString NewCellWizard::selected_robot_id() const { return robot_ ? robot_->currentData().toString() : QString(); }
QString NewCellWizard::selected_tool_id() const { return ee_ ? ee_->currentData().toString() : QString(); }
QJsonObject NewCellWizard::equipment_combination() const {
  for (const auto &item : equipment_.value("combinations").toArray()) {
    const auto c = item.toObject();
    if (c.value("robot").toString() == selected_robot_id() && c.value("tool").toString() == selected_tool_id()) return c;
  }
  return {};
}
QJsonObject NewCellWizard::equipment_profile() const {
  const QString id = equipment_combination().value("profile").toString();
  for (const auto &item : equipment_.value("profiles").toArray()) if (item.toObject().value("id").toString() == id) return item.toObject();
  return {};
}
bool NewCellWizard::select_equipment(const QString &robot_id, const QString &tool_id) {
  const auto robot = equipment_record("robots", robot_id), tool = equipment_record("tools", tool_id);
  if (robot.isEmpty() || tool.isEmpty()) return false;
  {
    const QSignalBlocker a(robot_family_), b(robot_), c(ee_family_), d(ee_);
    robot_family_->setCurrentIndex(robot_family_->findData(robot.value("family").toString()));
    refresh_robot_model_options(QString()); robot_->setCurrentIndex(robot_->findData(robot_id));
    ee_family_->setCurrentIndex(ee_family_->findData(tool.value("family").toString()));
    refresh_tool_model_options(QString()); ee_->setCurrentIndex(ee_->findData(tool_id));
  }
  apply_equipment_selection(); apply_recommended_environment_layout(); refresh_validation();
  return selected_robot_id() == robot_id && selected_tool_id() == tool_id;
}
void NewCellWizard::refresh_robot_model_options(const QString &) {
  const QSignalBlocker block(robot_); const QString keep = selected_robot_id(); robot_->clear();
  for (const auto &item : equipment_.value("robots").toArray()) {
    const auto r = item.toObject();
    if (r.value("family").toString() != robot_family_->currentData().toString()) continue;
    robot_->addItem(r.value("label").toString() + (r.value("installed").toBool() ? QString() : " (not installed)"), r.value("id").toString());
    robot_->setItemData(robot_->count() - 1, r.value("source").toString(), Qt::ToolTipRole);
  }
  robot_->setCurrentIndex(std::max(0, robot_->findData(keep)));
}
void NewCellWizard::refresh_tool_model_options(const QString &) {
  // Each tool says how far it goes with the selected robot.
  static const QMap<QString, QString> label{{"PLAN_CAPABLE", "plan-capable"}, {"CONFIGURABLE", "authoring only"},
    {"UNCOMMISSIONED", "not installed"}, {"UNSUPPORTED", "unsupported"}};
  const QSignalBlocker block(ee_); const QString keep = selected_tool_id(); ee_->clear();
  for (const auto &item : equipment_.value("tools").toArray()) {
    const auto t = item.toObject();
    if (t.value("family").toString() != ee_family_->currentData().toString()) continue;
    QString status;
    for (const auto &c : equipment_.value("combinations").toArray())
      if (c.toObject().value("robot").toString() == selected_robot_id() && c.toObject().value("tool").toString() == t.value("id").toString())
        status = label.value(c.toObject().value("status").toString());
    ee_->addItem(t.value("label").toString() + (status.isEmpty() ? QString() : " (" + status + ")"), t.value("id").toString());
    ee_->setItemData(ee_->count() - 1, t.value("source").toString(), Qt::ToolTipRole);
  }
  ee_->setCurrentIndex(std::max(0, ee_->findData(keep)));
}
void NewCellWizard::apply_equipment_selection() {
  const auto robot = equipment_record("robots", selected_robot_id()), tool = equipment_record("tools", selected_tool_id());
  const auto profile = equipment_profile(), reviewed = profile.value("defaults").toObject();
  // Reviewed profile values first, then the catalog and installed-model candidates; Advanced may pick any listed
  // one or type another, which validation checks against these lists.
  auto fill = [](QComboBox *combo, const QStringList &choices, const QString &current) {
    combo->clear(); QStringList items = choices; if (!current.isEmpty() && !items.contains(current)) items.prepend(current);
    items.removeAll(QString()); combo->addItems(items); combo->setCurrentText(current);
  };
  auto strings = [](const QJsonValue &v) { QStringList out; for (const auto &s : v.toArray()) out << s.toString(); return out; };
  auto pick = [&](const char *key, const QJsonObject &record, const char *fallback) {
    const QString value = reviewed.value(key).toString(); return value.isEmpty() ? record.value(fallback).toString() : value; };
  fill(robot_base_link_, strings(robot.value("base_frames")), pick("base", robot, "default_base"));
  fill(robot_tip_link_, strings(robot.value("tip_frames")), pick("tip", robot, "default_tip"));
  fill(robot_planning_group_, strings(robot.value("planning_groups")), pick("planning_group", robot, "default_planning_group"));
  fill(ee_attach_link_, strings(tool.value("attach_links")), tool.value("default_attach").toString());
  fill(ee_tcp_link_, strings(tool.value("tcp_frames")), pick("tcp", tool, "default_tcp"));
  fill(ee_type_, {tool.value("type").toString()}, reviewed.value("type").toString(tool.value("type").toString()));
  robot_controller_name_->setText(profile.value("arm_controller").toString());
  robot_controller_name_->setPlaceholderText("No reviewed controller binding for this robot and tool");
  // The reviewed mount; a configurable tool takes the mount of the supported scene that already assembles it (its
  // 3D authoring model), otherwise it starts unrotated on the flange (review it in Advanced).
  const auto combination = equipment_combination();
  const auto assembly = combination.value("assembly").toObject();
  auto xyz = reviewed.value("mount_pose_xyz").toArray(), rpy = reviewed.value("mount_pose_rpy").toArray();
  if (profile.isEmpty() && !assembly.isEmpty()) {
    xyz = assembly.value("tool_mount").toObject().value("xyz").toArray();
    rpy = assembly.value("tool_mount").toObject().value("rpy").toArray();
    robot_tip_link_->setCurrentText(assembly.value("tool_parent").toString());
  }
  std::array<QDoubleSpinBox*,6> mount{ee_x_,ee_y_,ee_z_,ee_roll_,ee_pitch_,ee_yaw_};
  for (int i = 0; i < 6; ++i) mount[i]->setValue(i < 3 ? xyz.at(i).toDouble() : rpy.at(i - 3).toDouble());
  // Grasp strategies whose catalog record names this tool; AUTO is the reviewed task authoring's own choice.
  QStringList strategies = strings(tool.value("grasp_strategies"));
  if (!profile.isEmpty()) strategies.prepend("auto");
  fill(grasp_strategy_, strategies, strategies.value(0));
  fill(release_strategy_, {tool.value("release").toString()}, tool.value("release").toString());
  const QString status = combination.value("status").toString("UNSUPPORTED");
  auto describe = [](const QJsonObject &record) {
    QStringList lines{QString("%1 (%2) — %3").arg(record.value("label").toString(), record.value("id").toString(), record.value("source").toString())};
    lines << QString("Model: %1").arg(record.value("model").toString().isEmpty()
      ? "not installed (missing " + record.value("missing").toVariant().toStringList().join(", ") + ")" : record.value("model").toString());
    lines << QString("Catalog status: support %1, runtime %2, fake hardware %3, real hardware %4.").arg(
      record.value("support_status").toString(), record.value("runtime_status").toString(),
      record.value("fake_hardware_status").toString(), record.value("real_hardware_status").toString());
    return lines.join("\n");
  };
  QString verdict = status + " - " + combination.value("reason").toString(equipment_error_.isEmpty()
    ? "Select a robot and a tool." : equipment_error_) + "\nReal hardware: not commissioned (locked).";
  QString model_line;
  if (!profile.isEmpty()) model_line = "3D model: reviewed profile " + profile.value("id").toString() + ".";
  else if (!assembly.isEmpty()) {
    const auto mount_xyz = assembly.value("tool_mount").toObject().value("xyz").toArray(), mount_rpy = assembly.value("tool_mount").toObject().value("rpy").toArray();
    auto triple = [](const QJsonArray &v) { return QString("%1, %2, %3").arg(v.at(0).toDouble()).arg(v.at(1).toDouble()).arg(v.at(2).toDouble()); };
    model_line = QString("3D model (layout authoring only): the installed robot and tool geometry, mounted on %1 as supported scene %2 does (xyz %3 m, rpy %4 rad).")
      .arg(assembly.value("tool_parent").toString(), assembly.value("scene").toString(), triple(mount_xyz), triple(mount_rpy));
  } else if (status == "CONFIGURABLE") model_line = "3D model: " + combination.value("assembly_reason").toString();
  if (!model_line.isEmpty()) verdict += "\n" + model_line;
  robot_readiness_banner_->setText(robot.isEmpty() ? verdict : describe(robot) + "\n" + verdict);
  ee_readiness_banner_->setText(tool.isEmpty() ? verdict : describe(tool) + "\n" + verdict);
  robot_warning_->setText(equipment_.value("notes").toVariant().toStringList().join("\n"));
}

void NewCellWizard::add_environment_asset_row(const QString &id,const QString &asset,const QString &parent,const QString &plink,const QString &clink,const QString &role){int r=env_objects_table_->rowCount(); env_objects_table_->insertRow(r); env_objects_table_->setCellWidget(r,0,new QCheckBox()); static_cast<QCheckBox*>(env_objects_table_->cellWidget(r,0))->setChecked(true); env_objects_table_->setItem(r,1,new QTableWidgetItem(id)); env_objects_table_->setItem(r,2,new QTableWidgetItem(asset)); env_objects_table_->setItem(r,3,new QTableWidgetItem(parent)); env_objects_table_->setItem(r,4,new QTableWidgetItem(plink)); env_objects_table_->setItem(r,5,new QTableWidgetItem(clink)); env_objects_table_->setItem(r,6,new QTableWidgetItem("fixed")); env_objects_table_->setItem(r,7,new QTableWidgetItem(role)); env_objects_table_->setItem(r,8,new QTableWidgetItem("x=0.000, y=0.000, z=0.000, r=0.0000, p=0.0000, y=0.0000")); }
QString NewCellWizard::profile_helper_path() const {
 const auto resolved=workcell_builder::resolve_scene_select_paths(Workcell{}, fs::path(workspace_root_.toStdString()));
 const auto candidate=QString::fromStdString((resolved.paths.workcell_path/"scripts/instantiate_workcell_studio_profile.py").string());
 return QFileInfo::exists(candidate) ? candidate : QDir::current().absoluteFilePath("scripts/instantiate_workcell_studio_profile.py");
}
void NewCellWizard::apply_recommended_environment_layout(){
 recommended_profile_.clear();
 recommended_profile_error_.clear();
 // The reviewed physical layout of the selected robot + tool, when a New Cell profile reviews that pair.
 const QString reviewed_profile=equipment_combination().value("profile").toString();
 if(!reviewed_profile.isEmpty()) {
 QProcess profile;
 QStringList describe{profile_helper_path(), "--profile", reviewed_profile, "--describe"};
 if(sorting_scenario()) describe << "--scenario" << selected_scenario_id();
 profile.start("python3", describe);
 if (profile.waitForFinished(30000) && profile.exitCode()==0) {
   const auto defaults=QJsonDocument::fromJson(profile.readAllStandardOutput()).object();
   if(defaults.value("id").toString()==reviewed_profile) {
     recommended_profile_=defaults.value("id").toString();
     env_objects_table_->setRowCount(0);
     for(const auto &value : defaults.value("rows").toArray()) {
       const auto row=value.toObject(); const auto id=row.value("id").toString();
       add_environment_asset_row(id,row.value("asset_type").toString(),"world","world",id,row.value("role").toString());
       const auto pose=row.value("pose").toObject(); const auto xyz=pose.value("xyz").toArray(), rpy=pose.value("rpy").toArray();
       QStringList values;
       for(const auto &v : xyz) values << QString::number(v.toDouble(),'g',17);
       for(const auto &v : rpy) values << QString::number(v.toDouble(),'g',17);
       env_objects_table_->item(env_objects_table_->rowCount()-1,8)->setText(
         QString("x=%1, y=%2, z=%3, r=%4, p=%5, y=%6").arg(values[0],values[1],values[2],values[3],values[4],values[5]));
     }
     const auto robot=defaults.value("robot").toObject(), tool=defaults.value("tool").toObject();
     const auto set_pose=[](const QJsonArray &xyz,const QJsonArray &rpy,std::array<QDoubleSpinBox*,6> fields){
       for(int i=0;i<6;++i){ fields[i]->setDecimals(12); fields[i]->setValue(i<3?xyz[i].toDouble():rpy[i-3].toDouble()); }
     };
     set_pose(robot.value("pose_xyz").toArray(),robot.value("pose_rpy").toArray(),{robot_x_,robot_y_,robot_z_,robot_roll_,robot_pitch_,robot_yaw_});
     set_pose(tool.value("mount_pose_xyz").toArray(),tool.value("mount_pose_rpy").toArray(),{ee_x_,ee_y_,ee_z_,ee_roll_,ee_pitch_,ee_yaw_});
     robot_tip_link_->setCurrentText(robot.value("tool_mount_link").toString());
     refresh_environment_parent_options();
     pick_source_->setCurrentText(defaults.value("pick_zone").toString());
     place_target_->setCurrentText(defaults.value("place_asset").toString());
     placement_mode_->setCurrentText("inside_bin");
     scenario_layout_=defaults.value("scenario").toObject();
     test_objects_table_->setRowCount(0);
     for(const auto &value : scenario_layout_.value("test_objects").toArray()) {
       const auto row=value.toObject(); const auto xy=row.value("xy").toArray();
       add_test_object_row(row.value("object_id").toString(), row.value("appearance").toString(), xy.at(0).toDouble(), xy.at(1).toDouble(), row.value("yaw_deg").toDouble());
     }
     refresh_summary(); return;
   }
 } else {
   if(profile.state()!=QProcess::NotRunning) { profile.kill(); profile.waitForFinished(); }
   recommended_profile_error_="Reviewed physical layout " + reviewed_profile + " could not be loaded: " + QString::fromUtf8(profile.readAllStandardError());
   env_objects_table_->setRowCount(0); refresh_validation(); return;
 }
 recommended_profile_error_="Reviewed physical layout " + reviewed_profile + " is invalid or incompatible.";
 env_objects_table_->setRowCount(0); refresh_validation(); return;
 }
  env_objects_table_->setRowCount(0); add_environment_asset_row("workbench_01","workbench/table","world","world","table_link","support_surface"); add_environment_asset_row("source_bin_01","bin","workbench_01","table_link","bin_link","pick_source"); add_environment_asset_row("place_fixture_01","fixture/table","workbench_01","table_link","fixture_link","place_target"); add_environment_asset_row("reject_bin_01","bin","workbench_01","table_link","bin_link","reject_target"); add_environment_asset_row("conveyor_01","conveyor","world","world","conveyor_link","conveyor/source_line"); add_environment_asset_row("camera_01","RealSense D435i","workbench_01","table_link","camera_link","camera_view_zone"); add_environment_asset_row("camera_mount_01","camera_mount","workbench_01","table_link","mount_link","camera_support"); add_environment_asset_row("safety_zone_01","safety_zone","world","world","safety_zone_link","safety_guard"); refresh_environment_parent_options(); refresh_environment_review_table(); refresh_summary();
}

QStringList NewCellWizard::discover_environment_asset_catalog() const { QStringList found; const QStringList roots={"assets/environment","assets/environment_objects"}; for(const QString &rel:roots){ QDir dir(workspace_root_+"/"+rel); const auto entries=dir.entryList(QDir::Dirs|QDir::NoDotAndDotDot); for(const auto &e:entries) found<<e; } found.removeDuplicates(); return found; }
void NewCellWizard::select_environment_substep(int index){ if(env_substep_stack_) env_substep_stack_->setCurrentIndex(std::max(0,index)); }
void NewCellWizard::refresh_environment_review_table(){ env_review_table_->setRowCount(0); for(int r=0;r<env_objects_table_->rowCount();++r){ int rr=env_review_table_->rowCount(); env_review_table_->insertRow(rr); auto id=env_objects_table_->item(r,1)->text(); auto type=env_objects_table_->item(r,2)->text(); auto rel=env_objects_table_->item(r,3)->text()+"."+env_objects_table_->item(r,4)->text()+" -> "+env_objects_table_->item(r,5)->text(); auto role=env_objects_table_->item(r,7)->text(); auto pose=env_objects_table_->item(r,8)->text(); auto *cb=qobject_cast<QCheckBox*>(env_objects_table_->cellWidget(r,0)); auto status=(cb&&cb->isChecked())?"enabled":"disabled"; env_review_table_->setItem(rr,0,new QTableWidgetItem(id)); env_review_table_->setItem(rr,1,new QTableWidgetItem(type)); env_review_table_->setItem(rr,2,new QTableWidgetItem(rel)); env_review_table_->setItem(rr,3,new QTableWidgetItem(role)); env_review_table_->setItem(rr,4,new QTableWidgetItem(pose)); env_review_table_->setItem(rr,5,new QTableWidgetItem(status)); }}

void NewCellWizard::apply_scenario_defaults(){
 if(selected_scenario_id()=="static_table_pick_place" && place_target_->findText("place_fixture_01")>=0) place_target_->setCurrentText("place_fixture_01");
}

void NewCellWizard::refresh_scenario_ui(){
 const bool configured = configured_scenario();
 const bool manual = selected_object_source_id()=="manual_simulated";
 task_scenario_->setText(QString("%1 (%2)").arg(selected_scenario_label(), selected_scenario_id()));
 scenario_configuration_notice_->setText(sorting_scenario()
   ? QStringLiteral("Sorting: every object the cell camera classifies runs the Pick & Place cycle below, into the destination its "
                    "class is routed to. The reviewed layout adds Destination Bin B with its own placement region. After "
                    "creating the cell, route classes to bins in Scene Builder > Task > Where > Route by class.")
   : configured ? QString() : QStringLiteral("Scenario configuration coming in next implementation phase"));
 pick_config_card_->setVisible(configured);
 place_config_card_->setVisible(configured);
 grasp_motion_card_->setVisible(configured);
 manual_object_card_->setVisible(configured && !sorting_scenario() && manual);
 test_objects_card_->setVisible(sorting_scenario() && manual);
}

void NewCellWizard::refresh_object_source_ui(){
 const bool manual=selected_object_source_id()=="manual_simulated";
 const bool replay=selected_object_source_id()=="recorded_perception";
 manual_object_card_->setVisible(manual && selected_scenario_id()=="static_table_pick_place");
 test_objects_card_->setVisible(manual && sorting_scenario());
 perception_binding_->setVisible(!manual); minimum_confidence_->setVisible(!manual); required_object_class_->setVisible(!manual);
 if(replay && perception_binding_->text()=="/easy_perception_deployment/epd_localize_output") perception_binding_->setText("generated/detected_objects_replay.yaml");
 else if(!replay && !manual && perception_binding_->text()=="generated/detected_objects_replay.yaml") perception_binding_->setText("/easy_perception_deployment/epd_localize_output");
 dynamic_object_notice_->setText(manual
   ? "Manual fallback: authored primitive geometry and pose are required because perception is absent."
   : "Dynamic observation: object ID, pose/centroid, dimensions, shape, and confidence come from perception when available. Fixed manual dimensions are not required.");
}

QStringList NewCellWizard::equipment_blockers() const{
 if(!equipment_error_.isEmpty()) return {equipment_error_};
 const auto combination=equipment_combination();
 if(combination.isEmpty()) return {"Select a robot and a tool from the capability catalog (Robot and End Effector pages)."};
 QStringList blockers;
 const QString status=combination.value("status").toString();
 if(status=="UNSUPPORTED"||status=="UNCOMMISSIONED") blockers<<status+": "+combination.value("reason").toString();
 // Advanced overrides must name what the installed model and its catalog record declare.
 const auto robot=equipment_record("robots",selected_robot_id()), tool=equipment_record("tools",selected_tool_id());
 auto check=[&](QComboBox *field, const QJsonObject &record, const char *key, const QString &what, const QString &page){
   const QString value=field->currentText().trimmed();
   const QStringList known=record.value(key).toVariant().toStringList();
   if(value.isEmpty()) blockers<<QString("%1 is required (%2).").arg(what,page);
   else if(!known.isEmpty() && !known.contains(value))
     blockers<<QString("%1 '%2' is not declared by %3 (known: %4). Select a listed value in %5.").arg(
       what,value,record.value("label").toString(),known.join(", "),page);
 };
 check(robot_base_link_,robot,"base_frames","Robot base link","Robot > Advanced Robot Frames");
 check(robot_tip_link_,robot,"tip_frames","Robot tip (tool mount) link","Robot > Advanced Robot Frames");
 check(robot_planning_group_,robot,"planning_groups","Planning group","Robot > Advanced Robot Frames");
 check(ee_attach_link_,tool,"attach_links","Tool attach link","End Effector > Advanced Tool Frames");
 // The 3D authoring model reuses a reference mount, which is only valid on the robot link it was measured from.
 const auto assembly=combination.value("assembly").toObject();
 if(!assembly.isEmpty() && robot_tip_link_->currentText().trimmed()!=assembly.value("tool_parent").toString())
   blockers<<QString("The 3D model mounts %1 on %2 (as supported scene %3 does); select that robot tip (Robot > Advanced Robot Frames).")
     .arg(tool.value("label").toString(), assembly.value("tool_parent").toString(), assembly.value("scene").toString());
 static const QRegularExpression frame_re("^[A-Za-z_][A-Za-z0-9_]*$");
 if(!frame_re.match(ee_tcp_link_->currentText().trimmed()).hasMatch())
   blockers<<"Tool TCP link must be a frame name (letters, digits, underscores) (End Effector > Advanced Tool Frames).";
 return blockers;
}

QStringList NewCellWizard::readiness_warnings() const{
 QStringList warnings;
 const auto combination=equipment_combination();
 if(combination.value("status").toString()=="CONFIGURABLE")
   warnings<<"Authoring only: "+combination.value("reason").toString()+" No task is authored for this cell.";
 const auto tool=equipment_record("tools",selected_tool_id());
 const QString tcp=ee_tcp_link_->currentText().trimmed();
 if(!tcp.isEmpty() && !tool.value("tcp_frames").toVariant().toStringList().contains(tcp))
   warnings<<QString("TCP frame '%1' is not declared by %2; the cell defines it at the tool's TCP.").arg(tcp,tool.value("label").toString());
 else if(!tcp.isEmpty() && combination.value("profile").toString().isEmpty() && !tool.value("model_links").toVariant().toStringList().contains(tcp))
   warnings<<QString("TCP '%1' is declared by %2 but its installed model does not define it, so the 3D view cannot place it; it stays unmeasured.").arg(tcp,tool.value("label").toString());
 const bool unknown=robot_tip_link_->currentText().trimmed().isEmpty()||ee_attach_link_->currentText().trimmed().isEmpty()||ee_tcp_link_->currentText().trimmed().isEmpty();
 if(unknown) warnings<<"Unknown robot/tool links still editable placeholders.";
 if(pick_camera_->currentText().isEmpty() && pick_zone_source_->currentText()=="Camera view zone") warnings<<"Camera view zone selected but no enabled camera asset.";
 return warnings;
}

QStringList NewCellWizard::readiness_blockers() const{
 QStringList blockers;
 blockers << equipment_blockers();
 if(!recommended_profile_error_.isEmpty()) blockers << recommended_profile_error_;
 if(!configured_scenario()) return blockers;
 if(sorting_scenario() && recommended_profile_.isEmpty())
   blockers<<(equipment_combination().value("status").toString()=="PLAN_CAPABLE"
     ? QString("Sorting uses the reviewed two-bin layout of the selected robot and tool: Environment > Use Recommended Layout.")
     : "Sorting needs a plan-capable robot and tool with a reviewed two-bin layout; " + equipment_combination().value("reason").toString());
 if(pick_source_->currentText().isEmpty()) blockers<<"Pick object source is required.";
 if(selected_object_source_id()=="manual_simulated" && sorting_scenario()) blockers<<test_object_blockers();
 else if(selected_object_source_id()=="manual_simulated" && !manual_object_geometry_valid()) blockers<<"Manual / simulated object requires an ID, frame, and positive primitive dimensions.";
 if(selected_object_source_id()!="manual_simulated" && perception_binding_->text().trimmed().isEmpty()) blockers<<"Perception binding is required for live or recorded object sources.";
 if(place_target_->currentText().isEmpty()) blockers<<"Place target is required for Pick & Place.";
 return blockers;
}
void NewCellWizard::refresh_environment_parent_options(){ pick_camera_->clear(); pick_source_->clear(); place_target_->clear(); place_frame_link_->clear(); env_parent_combo_->clear(); for(int r=0;r<env_objects_table_->rowCount();++r){auto*cb=qobject_cast<QCheckBox*>(env_objects_table_->cellWidget(r,0)); if(!cb||!cb->isChecked()) continue; auto role=env_objects_table_->item(r,7)->text().toLower(); auto id=env_objects_table_->item(r,1)->text(); env_parent_combo_->addItem(id); if(role.contains("camera")) pick_camera_->addItem(id); if(role.contains("pick_source")||role.contains("conveyor")) pick_source_->addItem(id); if(role.contains("place_target")||role.contains("reject_target")||role.contains("output_bin")||role.contains("fixture")||role.contains("support")) place_target_->addItem(id);} pick_source_->addItem("manual"); place_frame_link_->addItems({"target_link","top_surface","world"}); if(pick_camera_->findText("camera_01")>=0){ pick_zone_source_->setCurrentText("Camera view zone"); pick_camera_->setCurrentText("camera_01"); } if(pick_source_->findText("source_bin_01")>=0) pick_source_->setCurrentText("source_bin_01"); apply_scenario_defaults(); refresh_environment_review_table(); }

void NewCellWizard::set_output_root(const QString & path) { if (!path.isEmpty()) output_path_->setText(path); }
fs::path NewCellWizard::scenes_root_path() const {
 if (output_path_) return fs::absolute(fs::path(output_path_->text().trimmed().toStdString()));
 return canonical_scenes_root();
}
fs::path NewCellWizard::canonical_scenes_root() const {
 const auto resolved = workcell_builder::resolve_scene_select_paths(Workcell{}, fs::path(workspace_root_.toStdString()));
 return resolved.success ? resolved.paths.scenes_path : fs::path(workspace_root_.toStdString()) / "src" / "scenes";
}
QString NewCellWizard::scene_name_error() const {
 const QString n=scene_name_->text().trimmed();
 if(n.isEmpty()) return "Scene/package name is required.";
 if(output_path_ && output_path_->text().trimmed().isEmpty()) return "Output scenes path is required.";
 if(!is_valid_package_name(n)) return "Use lowercase letters, numbers, underscores only; must start with a letter.";
 boost::system::error_code ec;
 const fs::path chosen=fs::weakly_canonical(scenes_root_path(),ec), canonical=fs::weakly_canonical(canonical_scenes_root(),ec);
 if(chosen!=canonical)
   return QString("%1 is not supported. New cells are saved only in the Workcell Studio scenes folder %2, where Workcell "
                  "Builder lists, edits, generates and validates them. To move a cell elsewhere, copy or export it.")
     .arg(QString::fromStdString(scenes_root_path().string()), QString::fromStdString(canonical_scenes_root().string()));
 return "";
}
QString NewCellWizard::scene_name_warning() const { const QString n=scene_name_->text().trimmed(); if(n.isEmpty()) return ""; const fs::path p=scenes_root_path()/n.toStdString(); if(fs::exists(p)) return "Warning: output folder already exists. Creation is blocked to avoid overwrite."; return ""; }

void NewCellWizard::refresh_validation(){
 const int row=steps_->currentRow(); const QString err=scene_name_error(); scene_error_->setText(err); scene_warning_->setText(scene_name_warning());
 const bool name_valid=err.isEmpty()&&scene_name_warning().isEmpty(); back_->setEnabled(row>0); next_->setEnabled(row<5&&(row!=0||name_valid));
 const auto blockers=readiness_blockers(); const auto warnings=readiness_warnings();
 QString readiness=configured_scenario()?"READY":"AUTHORING ONLY"; if(!blockers.isEmpty()) readiness="BLOCKED"; else if(!warnings.isEmpty() && configured_scenario()) readiness="WARNINGS";
 task_readiness_label_->setText(readiness);
 task_warning_->setText((blockers+warnings).join("\n"));
 const auto creation=creation_blockers(); const bool creatable=creation.isEmpty();
 // The name field is the one required free-text input: mark it until it is valid.
 scene_name_->setStyleSheet(name_valid ? QString() : QString("QLineEdit { border: 2px solid #e5484d; border-radius: 4px; }"));
 for (auto *label : {scene_error_, scene_warning_}) { label->setStyleSheet(label->text().isEmpty() ? QString() : QString("color: #ff6b6b; font-weight: 600;")); label->setVisible(!label->text().isEmpty()); }
 create_->setEnabled(creatable); create_open_->setEnabled(creatable);
 QStringList reasons; for (const auto &b : creation) reasons << b.text;
 const QString blocked=creatable ? QString() : "Create is disabled until these are fixed:\n" + reasons.join("\n");
 create_->setToolTip(blocked); create_open_->setToolTip(blocked);
 create_blocked_->setText(creatable ? QString() : QString("<span style='color:#ff6b6b'>Create disabled: %1</span> <a href='%2'>Fix</a>")
   .arg(creation.size()==1 ? creation.first().text.toHtmlEscaped() : QString("%1 items to fix (see Review)").arg(creation.size()),
        QString("goto:%1:%2").arg(creation.isEmpty() ? 0 : creation.first().page).arg(creation.isEmpty() ? QString() : creation.first().field)));
 refresh_summary();
}

QList<NewCellWizard::CreationBlocker> NewCellWizard::creation_blockers() const{
 QList<CreationBlocker> out;
 if(const QString e=scene_name_error(); !e.isEmpty()) out << CreationBlocker{0, "newCellSceneName", "Basics > Scene/package name: " + e};
 if(const QString w=scene_name_warning(); !w.isEmpty()) out << CreationBlocker{0, "newCellSceneName", "Basics > Scene/package name: " + w};
 for(const QString &b : equipment_blockers()) {
   const bool tool=b.contains("End Effector") || b.contains("tool", Qt::CaseInsensitive);
   out << CreationBlocker{tool && !b.contains("Robot >") ? 2 : 1, tool ? "newCellToolModel" : "newCellRobotModel", b};
 }
 if(!recommended_profile_error_.isEmpty()) out << CreationBlocker{3, QString(), "Environment: " + recommended_profile_error_};
 return out;
}

void NewCellWizard::go_to_blocker(const QString &link){
 const QStringList parts=link.split(':');
 if(parts.size()<2 || parts.first()!="goto") return;
 steps_->setCurrentRow(std::clamp(parts.at(1).toInt(), 0, 5));
 if(parts.size()>2 && !parts.at(2).isEmpty())
   if(auto *field=findChild<QWidget *>(parts.at(2))) field->setFocus(Qt::OtherFocusReason);
}

void NewCellWizard::refresh_summary(){
 QString readiness=task_readiness_label_?task_readiness_label_->text():"READY";
 const auto creation=creation_blockers();
 // Overall creation readiness comes first and is never READY while Create is disabled.
 QString creation_html;
 if(creation.isEmpty()) creation_html="<p><b>Cell creation:</b> <span style='color:#3fb950'>READY</span> &mdash; Create Cell and Create and Open are enabled.</p>";
 else {
   creation_html=QString("<p><b>Cell creation:</b> <span style='color:#ff6b6b'>BLOCKED</span> &mdash; fix %1 item(s) to enable Create:</p><ul>").arg(creation.size());
   for(const auto &b : creation) creation_html+=QString("<li>%1 <a href='goto:%2:%3'>Go to field</a></li>").arg(b.text.toHtmlEscaped()).arg(b.page).arg(b.field);
   creation_html+="</ul>";
   if(readiness=="READY") readiness="Task inputs complete; the cell is not created until the items above are fixed";
 }
 summary_->setText(creation_html + QString("<b>Application / Scenario:</b> %1 (%2)<br/><b>Object source:</b> %3 (%4)<br/><b>Robot:</b> family=%5 model=%6 | base=%7 tip=%8 planning=%9<br/><b>Tool:</b> family=%10 model=%11 | attach=%12 | tcp=%13 | type=%14 | mount rpy=%15,%16,%17 <br/><b>Equipment readiness:</b> %18<br/><b>Task authoring:</b> %19<br/><b>Warnings/Blockers:</b><br/>%20").arg(selected_scenario_label(),selected_scenario_id(),object_source_mode_->currentText(),selected_object_source_id(),robot_family_->currentText(),robot_->currentText(),robot_base_link_->currentText(),robot_tip_link_->currentText(),robot_planning_group_->currentText(),ee_family_->currentText(),ee_->currentText(),ee_attach_link_->currentText(),ee_tcp_link_->currentText(),ee_type_->currentText(),ee_roll_->text(),ee_pitch_->text(),ee_yaw_->text(),(equipment_combination().value("status").toString()+" - "+equipment_combination().value("reason").toString()).toHtmlEscaped(),readiness,task_warning_->text().toHtmlEscaped().replace("\n","<br/>")));
}

bool NewCellWizard::create_scene_scaffold(bool open_in_builder){
 if(!scene_name_error().isEmpty()||!scene_name_warning().isEmpty()||!recommended_profile_error_.isEmpty()||!equipment_blockers().isEmpty()) return false;
 fs::path destination=fs::absolute(scenes_root_path())/scene_name_->text().trimmed().toStdString();
 auto fail = [&](const QString & detail) {
   steps_->setCurrentRow(0);
   // Changing pages refreshes validation; retain the actionable save error.
   scene_error_->setText("Save failed at " + QString::fromStdString(destination.string()) + ": " + detail +
     ". Check the output folder and retry; no cell was created.");
   return false;
 };
 boost::system::error_code ec;
 fs::create_directories(destination.parent_path(),ec);
 if(ec) return fail(QString::fromStdString(ec.message()));
 destination=fs::canonical(destination.parent_path())/destination.filename();
 QTemporaryDir staging(QString::fromStdString((destination.parent_path()/".new-cell-XXXXXX").string()));
 if(!staging.isValid()) return fail("Cannot create temporary authoring folder");
 const fs::path scene_dir=fs::path(staging.path().toStdString())/destination.filename();
 fs::create_directories(scene_dir/"config",ec);
 if(ec) return fail(QString::fromStdString(ec.message()));

 auto yaml_scalar=[](const QString &value){
   std::string s=value.toStdString();
   size_t pos=0;
   while((pos=s.find("'",pos))!=std::string::npos){ s.insert(pos,"'"); pos+=2; }
   return std::string("'")+s+"'";
 };
 auto pose_number=[](QDoubleSpinBox *spin){ return spin->value(); };

 const QString warning_text=task_warning_->text().trimmed();
 const bool pick_place_scenario=configured_scenario();
 const auto combination=equipment_combination();
 const QString equipment_status=combination.value("status").toString();   // Pick & Place, or Sorting (the same cycle per object)
 if(sorting_scenario() && !readiness_blockers().isEmpty()) return fail(readiness_blockers().join(" "));
 const bool place_target_missing=pick_place_scenario && place_target_->currentText().trimmed().isEmpty();
 const bool unknown_links=warning_text.contains("unknown");
 QString readiness=pick_place_scenario && equipment_status=="PLAN_CAPABLE"?"READY":"AUTHORING_ONLY";
 if(place_target_missing) readiness="BLOCKED";
 else if(unknown_links) readiness="WARNINGS";

 std::ostringstream out;
 out<<std::setprecision(17);
 out<<"scene_name: "<<scene_name_->text().trimmed().toStdString()<<"\n";
 // The scene browser and header read scene.display_name; Description is kept beside it.
 out<<"scene:\n";
 out<<"  id: "<<yaml_scalar(scene_name_->text().trimmed())<<"\n";
 if(!display_name_->text().trimmed().isEmpty()) out<<"  display_name: "<<yaml_scalar(display_name_->text().trimmed())<<"\n";
 if(!description_->toPlainText().trimmed().isEmpty()) out<<"  description: "<<yaml_scalar(description_->toPlainText().trimmed())<<"\n";
 // Equipment identities are capability ids (catalog/capabilities); export refuses a pair that is not plan-capable.
 out<<"robot: "<<yaml_scalar(selected_robot_id())<<"\n";
 out<<"end_effector: "<<yaml_scalar(selected_tool_id())<<"\n";
out<<"workcell_studio:\n";
 out<<"  scenario:\n";
 out<<"    id: "<<yaml_scalar(selected_scenario_id())<<"\n";
 out<<"    category: "<<yaml_scalar(selected_scenario_category())<<"\n";
 out<<"    label: "<<yaml_scalar(selected_scenario_label())<<"\n";
 out<<"    authoring_selection_only: true\n";
 out<<"    runtime_ready: false\n";
 out<<"  equipment:\n";
 out<<"    status: "<<yaml_scalar(equipment_status)<<"\n";
 out<<"    reason: "<<yaml_scalar(combination.value("reason").toString())<<"\n";
 out<<"    profile: "<<yaml_scalar(combination.value("profile").toString())<<"\n";
 out<<"    robot_capability: "<<yaml_scalar(selected_robot_id())<<"\n";
 out<<"    tool_capability: "<<yaml_scalar(selected_tool_id())<<"\n";
 out<<"    real_hardware: 'not_commissioned'\n";
 out<<"  robot:\n";
 out<<"    family: "<<yaml_scalar(robot_family_->currentData().toString())<<"\n";
 out<<"    model: "<<yaml_scalar(selected_robot_id())<<"\n";
 out<<"    capability: "<<yaml_scalar(selected_robot_id())<<"\n";
 out<<"    label: "<<yaml_scalar(equipment_record("robots",selected_robot_id()).value("label").toString())<<"\n";
 out<<"    base: "<<yaml_scalar(robot_base_link_->currentText())<<"\n";
 out<<"    tip: "<<yaml_scalar(robot_tip_link_->currentText())<<"\n";
 out<<"    planning_group: "<<yaml_scalar(robot_planning_group_->currentText())<<"\n";
 out<<"    controller: "<<yaml_scalar(robot_controller_name_->text())<<"\n";
 out<<"    readiness: "<<yaml_scalar(equipment_status)<<"\n";
 out<<"  tool:\n";
 out<<"    family: "<<yaml_scalar(ee_family_->currentData().toString())<<"\n";
 out<<"    model: "<<yaml_scalar(selected_tool_id())<<"\n";
 out<<"    capability: "<<yaml_scalar(selected_tool_id())<<"\n";
 out<<"    label: "<<yaml_scalar(equipment_record("tools",selected_tool_id()).value("label").toString())<<"\n";
 out<<"    attach: "<<yaml_scalar(ee_attach_link_->currentText())<<"\n";
 out<<"    tcp: "<<yaml_scalar(ee_tcp_link_->currentText())<<"\n";
 out<<"    type: "<<yaml_scalar(ee_type_->currentText())<<"\n";
 out<<"    mount_pose:\n";
 out<<"      xyz: ["<<pose_number(ee_x_)<<", "<<pose_number(ee_y_)<<", "<<pose_number(ee_z_)<<"]\n";
 out<<"      rpy: ["<<pose_number(ee_roll_)<<", "<<pose_number(ee_pitch_)<<", "<<pose_number(ee_yaw_)<<"]\n";
 out<<"    readiness: "<<yaml_scalar(equipment_status)<<"\n";
 out<<"  create_cell_metadata:\n";
 out<<"    robot_family: "<<yaml_scalar(robot_family_->currentData().toString())<<"\n";
 out<<"    robot_model: "<<yaml_scalar(selected_robot_id())<<"\n";
 out<<"    robot_base_link: "<<yaml_scalar(robot_base_link_->currentText())<<"\n";
 out<<"    robot_tip_link: "<<yaml_scalar(robot_tip_link_->currentText())<<"\n";
 out<<"    robot_planning_group: "<<yaml_scalar(robot_planning_group_->currentText())<<"\n";
 out<<"    robot_readiness: "<<yaml_scalar(equipment_status)<<"\n";
 out<<"  frames:\n";
 out<<"    robot_base_link: "<<yaml_scalar(robot_base_link_->currentText())<<"\n";
 out<<"    robot_tip_link: "<<yaml_scalar(robot_tip_link_->currentText())<<"\n";
 out<<"    planning_group: "<<yaml_scalar(robot_planning_group_->currentText())<<"\n";
 out<<"    end_effector_attach_link: "<<yaml_scalar(ee_attach_link_->currentText())<<"\n";
 out<<"    end_effector_tcp_link: "<<yaml_scalar(ee_tcp_link_->currentText())<<"\n";
 out<<"    end_effector_family: "<<yaml_scalar(ee_family_->currentData().toString())<<"\n";
 out<<"    end_effector_model: "<<yaml_scalar(selected_tool_id())<<"\n";
 out<<"    end_effector_type: "<<yaml_scalar(ee_type_->currentText())<<"\n";
 out<<"    end_effector_readiness: "<<yaml_scalar(equipment_status)<<"\n";
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
  } else if(sorting_scenario()) {
   // Height, size and material are the qualified test block's (the profile helper completes each body).
   out<<"      manual_observations:\n";
   for(int r=0;r<test_objects_table_->rowCount();++r){
    auto number=[this,r](int c){ return test_objects_table_->item(r,c)->text().trimmed().toDouble(); };
    auto *look=qobject_cast<QComboBox*>(test_objects_table_->cellWidget(r,1));
    out<<"        - object_id: "<<yaml_scalar(test_objects_table_->item(r,0)->text().trimmed())<<"\n";
    out<<"          appearance: "<<yaml_scalar(look ? look->currentData().toString() : QString())<<"\n";
    out<<"          pose:\n";
    out<<"            frame_id: 'world'\n";
    out<<"            xyz: ["<<number(2)<<", "<<number(3)<<", 0]\n";
    out<<"            rpy: [0, 0, "<<qDegreesToRadians(number(4))<<"]\n";
   }
  } else {
   out<<"      manual_observation:\n";
   out<<"        object_id: "<<yaml_scalar(manual_object_id_->text().trimmed())<<"\n";
   out<<"        shape: "<<yaml_scalar(manual_object_shape_->currentText())<<"\n";
   if(!manual_object_class_->text().trimmed().isEmpty()) out<<"        class_id: "<<yaml_scalar(manual_object_class_->text().trimmed())<<"\n";
   if(!manual_object_material_->currentData().toString().isEmpty()) out<<"        material: "<<yaml_scalar(manual_object_material_->currentData().toString())<<"\n";
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
 auto save = [](const fs::path & path, const QByteArray & bytes, QString * error) {
   QSaveFile file(QString::fromStdString(path.string()));
   if(!file.open(QIODevice::WriteOnly) || file.write(bytes)!=bytes.size() || !file.commit()) {
     *error=file.errorString(); return false;
   }
   return true;
 };
 QString error;
 if(!save(scene_dir/"environment.yaml", QByteArray::fromStdString(out.str()), &error)) return fail(error);
 if(equipment_status=="CONFIGURABLE" && !combination.value("assembly").toObject().isEmpty()) {
   // Authoring-only 3D model of the installed robot and tool (no MoveIt, controller or task binding).
   QProcess assemble;
   assemble.start("python3", {profile_helper_path(), "--authoring-model", "--scene", QString::fromStdString(scene_dir.string())});
   if(!assemble.waitForFinished(180000)) { assemble.kill(); assemble.waitForFinished(); return fail("3D equipment model assembly did not finish"); }
   if(assemble.exitStatus()!=QProcess::NormalExit || assemble.exitCode()!=0)
     return fail("3D equipment model could not be assembled: " + QString::fromUtf8(assemble.readAllStandardError()).trimmed());
 }
 if(pick_place_scenario && !recommended_profile_.isEmpty()) {
   QProcess materialize;
   materialize.start("python3", {profile_helper_path(), "--profile", recommended_profile_,
     "--scene", QString::fromStdString(scene_dir.string()), "--destination", QString::fromStdString(destination.string())});
   if(!materialize.waitForFinished(180000)) { materialize.kill(); materialize.waitForFinished(); return fail("Physical scene preparation did not finish"); }
   if(materialize.exitStatus()!=QProcess::NormalExit || materialize.exitCode()!=0)
     return fail("Physical scene preparation failed: " + QString::fromUtf8(materialize.readAllStandardError()) + QString::fromUtf8(materialize.readAllStandardOutput()));
 } else if(pick_place_scenario && equipment_status=="PLAN_CAPABLE") {
   const auto resolved=workcell_builder::resolve_scene_select_paths(Workcell{}, fs::path(workspace_root_.toStdString()));
   QString helper=QString::fromStdString((resolved.paths.workcell_path/"scripts/task_intent_authoring.py").string());
   if(!QFileInfo::exists(helper)) helper=QDir::current().absoluteFilePath("scripts/task_intent_authoring.py");
   QProcess authoring;
   authoring.start("python3", {helper, QString::fromStdString(scene_dir.string())});
   if(!authoring.waitForFinished(30000)) { authoring.kill(); authoring.waitForFinished(); return fail("Task authoring helper did not finish"); }
   const auto report=QJsonDocument::fromJson(authoring.readAllStandardOutput()).object();
   auto intent=report.value("task_intent").toObject();
   if(authoring.exitStatus()!=QProcess::NormalExit || authoring.exitCode()!=0 || intent.isEmpty())
     return fail("Task intent could not be saved: " + QString::fromUtf8(QJsonDocument(report).toJson(QJsonDocument::Compact)) + QString::fromUtf8(authoring.readAllStandardError()));
   intent.insert("scene_package", QString::fromStdString(destination.string()));
   if(!save(scene_dir/"config/workcell_builder_task_intent.yaml", QJsonDocument(intent).toJson(), &error)) return fail(error);
 }
 if(fs::exists(destination)) return fail("Output folder already exists; choose a different name");
 fs::rename(scene_dir, destination, ec);
 if(ec) return fail(QString::fromStdString(ec.message()));
 result_.created=true; result_.open_in_scene_builder=open_in_builder; result_.scene_name=scene_name_->text().trimmed(); result_.scene_dir=destination; return true;
}

void NewCellWizard::add_test_object_row(const QString &id, const QString &appearance, double x, double y, double yaw_deg){
 const QSignalBlocker block(test_objects_table_);
 const int r=test_objects_table_->rowCount(); test_objects_table_->insertRow(r);
 test_objects_table_->setItem(r,0,new QTableWidgetItem(id));
 auto *look=new QComboBox(test_objects_table_);
 const auto appearances=scenario_layout_.value("appearances").toObject();
 for(auto it=appearances.begin(); it!=appearances.end(); ++it) look->addItem(it.value().toObject().value("label").toString(it.key()), it.key());
 look->setCurrentIndex(std::max(0, look->findData(appearance)));
 test_objects_table_->setCellWidget(r,1,look);
 test_objects_table_->setItem(r,2,new QTableWidgetItem(QString::number(x,'f',3)));
 test_objects_table_->setItem(r,3,new QTableWidgetItem(QString::number(y,'f',3)));
 test_objects_table_->setItem(r,4,new QTableWidgetItem(QString::number(yaw_deg,'f',1)));
}

QStringList NewCellWizard::test_object_blockers() const{
 QStringList blockers; QSet<QString> ids;
 static const QRegularExpression id_re("^[A-Za-z][A-Za-z0-9_]*$");
 if(test_objects_table_->rowCount()==0) blockers<<"Sorting needs at least one simulated test object (Add Object).";
 for(int r=0;r<test_objects_table_->rowCount();++r){
   auto text=[this,r](int c){ auto *item=test_objects_table_->item(r,c); return item ? item->text().trimmed() : QString(); };
   const QString id=text(0);
   if(!id_re.match(id).hasMatch() || ids.contains(id)) blockers<<QString("Test object %1: use a unique ID of letters, digits and underscores.").arg(r+1);
   ids.insert(id);
   for(int c=2;c<5;++c){
     bool ok=false; text(c).toDouble(&ok);
     if(!ok){ blockers<<QString("Test object %1: %2 must be a number.").arg(id.isEmpty()?QString::number(r+1):id, test_objects_table_->horizontalHeaderItem(c)->text()); break; }
   }
 }
 return blockers;
}
