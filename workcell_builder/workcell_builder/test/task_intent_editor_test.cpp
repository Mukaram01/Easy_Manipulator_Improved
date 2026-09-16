#include <QFileInfo>
#include <gtest/gtest.h>
#include <QApplication>
#include <QComboBox>
#include <QDir>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QProcess>
#include <QLineEdit>
#include <QLabel>
#include <QTemporaryDir>
#include "gui/task_intent_editor.h"

namespace {
void application() {
  if (QCoreApplication::instance()) return;
  if (qEnvironmentVariableIsEmpty("QT_QPA_PLATFORM")) qputenv("QT_QPA_PLATFORM", "offscreen");
  static int argc = 1; static char name[] = "task_editor_test"; static char * argv[] = {name, nullptr};
  static QApplication app(argc, argv);
}
void write(const QString & path, const QByteArray & bytes) { QFile f(path); ASSERT_TRUE(f.open(QIODevice::WriteOnly)); f.write(bytes); }
QByteArray read(const QString & path) { QFile f(path); if (!f.open(QIODevice::ReadOnly)) return {}; return f.readAll(); }
void seed(const QString & scene) {
  QDir().mkpath(scene + "/config");
  write(scene + "/environment.yaml", R"(end_effector: {id: robotiq_85_gripper}
robot: {name: ur5}
assets:
- {id: bin, frame: world, pose_xyz: [1, 2, 3], pose_rpy: [0, 0, 0], collision: {enabled: true}, usable_placement: {pose_xyz: [0, 0, 0.1], pose_rpy: [0, 0, 0], dimensions: [0.4, 0.3, 0.2]}}
task_zones:
- {id: pick, type: pick_zone}
- {id: drop, target_ref: bin, frame: world, pose_xyz: [1.05, 2, 3.1], pose_rpy: [0, 0, 0], dimensions: [0.2, 0.1, 0.1], placement_local: {pose_xyz: [0.05, 0, 0.1], pose_rpy: [0, 0, 0], dimensions: [0.2, 0.1, 0.1]}}
)");
  write(scene + "/config/workcell_builder_task_intent.yaml", R"(schema: workcell_builder_task_intent/v2
task: {id: t, type: pick_place, template: pick_place}
pick: {selection: {source_ref: source, source_type: perception, zone_ref: pick, object_filter: {class_id: bottle, min_confidence: null}}, grasp: {policy: EXACT, required_capability: two_finger_parallel, strategy_ref: top_2f, approach: {axis: z_down, distance_m: 0.123456789123}, orientation: {mode: vertical}, lift: {axis: z_up, distance_m: 0.15}}}
place: {target: {asset_ref: bin, region_ref: drop}, placement: {policy: EXACT, requested_local_pose: {xyz_m: [0.05, 0, 0.1], rpy_rad: [-0.2, 0, 0]}, orientation: {mode: fixed}, approach: {axis: z_down, distance_m: 0.1}, retreat: {axis: z_up, distance_m: 0.1}}, release: {strategy: tool_release}}
safety: {require_fake_hardware: true, real_hardware_enabled: false}
provenance: {custom: retain}
)");
}
QString helper() { return QStringLiteral(WORKCELL_BUILDER_REPO_ROOT) + "/scripts/task_intent_authoring.py"; }
void edit(TaskIntentEditor & editor, const char * name, const QString & value) {
  auto * field = editor.findChild<QLineEdit *>(name); ASSERT_NE(field, nullptr);
  field->setText(value); field->textEdited(value);
}
}
TEST(TaskIntentEditor, WidgetEditsAtomicSaveAndReopenPreserveHashAndBindings) {
  application(); QTemporaryDir dir; seed(dir.path());
  const auto environment = read(dir.path() + "/environment.yaml");
  TaskIntentEditor editor; ASSERT_TRUE(editor.load_scene(dir.path(), helper()));
  edit(editor, "taskTargetClass", "part");
  editor.findChild<QComboBox *>("taskGraspPolicy")->setCurrentText("PREFERRED");
  edit(editor, "taskLocalXYZ", "0.04, 0.01, 0.12");
  ASSERT_TRUE(editor.dirty()); ASSERT_TRUE(editor.save());
  auto before = workcell_builder::TaskIntentModel::from_validated_yaml(editor.model().to_yaml()); ASSERT_TRUE(before);
  TaskIntentEditor reopened; ASSERT_TRUE(reopened.load_scene(dir.path(), helper()));
  auto after = workcell_builder::TaskIntentModel::from_validated_yaml(reopened.model().to_yaml()); ASSERT_TRUE(after);
  EXPECT_EQ(workcell_builder::authoritative_task_intent_sha256(*before), workcell_builder::authoritative_task_intent_sha256(*after));
  EXPECT_EQ(after->pick_selection.class_id, "part"); EXPECT_EQ(after->pick_selection.source_ref, "source");
  EXPECT_EQ(after->place.asset_ref, "bin"); EXPECT_EQ(after->place.region_ref, "drop");
  EXPECT_FALSE(after->pick_selection.min_confidence.has_value());
  EXPECT_DOUBLE_EQ(after->grasp.approach_distance_m, 0.123456789123);
  EXPECT_DOUBLE_EQ(after->place.requested_local_pose->rpy_rad[0], -0.2);
  EXPECT_EQ(environment, read(dir.path() + "/environment.yaml"));
}
TEST(TaskIntentEditor, InvalidExactRemainsUnchangedAndBlocksAfterReopen) {
  application(); QTemporaryDir dir; seed(dir.path()); TaskIntentEditor editor;
  ASSERT_TRUE(editor.load_scene(dir.path(), helper()));
  edit(editor, "taskLocalXYZ", "0.30, 0, 0.1");
  ASSERT_TRUE(editor.save());
  EXPECT_TRUE(editor.blocker().contains("PLACE_LOCAL_POSE_OUTSIDE_REGION"));
  TaskIntentEditor reopened; ASSERT_TRUE(reopened.load_scene(dir.path(), helper()));
  EXPECT_EQ(reopened.model().place.policy, "EXACT");
  EXPECT_DOUBLE_EQ(reopened.model().place.requested_local_pose->xyz_m[0], 0.30);
  EXPECT_TRUE(reopened.blocker().contains("PLACE_LOCAL_POSE_OUTSIDE_REGION"));
}
TEST(TaskIntentEditor, MalformedInputAndExternalChangesCannotSilentlySave) {
  application(); QTemporaryDir dir; seed(dir.path()); TaskIntentEditor editor;
  ASSERT_TRUE(editor.load_scene(dir.path(), helper()));
  auto path = dir.path() + "/config/workcell_builder_task_intent.yaml"; auto before = read(path);
  edit(editor, "taskLocalXYZ", "broken"); EXPECT_FALSE(editor.save()); EXPECT_EQ(read(path), before);
  edit(editor, "taskLocalXYZ", "0.05, 0, 0.1");
  write(path, before + "\n# another editor\n"); EXPECT_FALSE(editor.save()); EXPECT_EQ(read(path), before + "\n# another editor\n");
}

#include <QFormLayout>
#include <QPushButton>
#include "gui/new_cell_wizard.h"

TEST(TaskIntentEditor, ExistingNewCellWizardCreatesEditableV2DraftWithoutFileRepair) {
  application(); QTemporaryDir workspace;
  QDir().mkpath(workspace.path() + "/src/easy_manipulation_deployment/scenes");
  QDir().mkpath(workspace.path() + "/src/easy_manipulation_deployment/assets");
  NewCellWizard wizard(workspace.path());
  ASSERT_TRUE(wizard.select_scenario_by_id("static_table_pick_place"));
  ASSERT_TRUE(wizard.select_object_source_by_id("manual_simulated"));
  bool named = false;
  for (auto * form : wizard.findChildren<QFormLayout *>()) {
    for (auto * field : wizard.findChildren<QLineEdit *>()) {
      auto * label = qobject_cast<QLabel *>(form->labelForField(field));
      if (label && label->text() == "Scene/package name") { field->setText("r20c_fresh_cell"); named = true; }
    }
  }
  ASSERT_TRUE(named);
  for (auto * button : wizard.findChildren<QPushButton *>()) if (button->text() == "Create and Open") button->click();
  ASSERT_TRUE(wizard.result().created);
  const auto scene = QString::fromStdString(wizard.result().scene_dir.string());
  const auto environment = read(scene + "/environment.yaml");
  workcell_builder::TaskIntentModel before;
  {
    TaskIntentEditor editor; ASSERT_TRUE(editor.load_scene(scene, helper()));
    EXPECT_EQ(editor.model().grasp.policy, "AUTO");
    EXPECT_EQ(editor.model().pick_selection.source_ref, "pick_zone_main");
    EXPECT_EQ(editor.model().place.asset_ref, "target_bin_default");
    edit(editor, "taskTargetClass", "widget");
    editor.findChild<QComboBox *>("taskGraspPolicy")->setCurrentText("PREFERRED");
    editor.findChild<QComboBox *>("taskGraspIntent")->setCurrentText("top_2f");
    ASSERT_TRUE(editor.save());
    auto validated = workcell_builder::TaskIntentModel::from_validated_yaml(editor.model().to_yaml());
    ASSERT_TRUE(validated); before = *validated;
    EXPECT_TRUE(editor.findChild<QLabel *>("taskValidationStatus")->text().contains(
      QString::fromStdString(workcell_builder::authoritative_task_intent_sha256(before))));
  }  // Close the editor before opening the saved task in a new instance.
  TaskIntentEditor reopened; ASSERT_TRUE(reopened.load_scene(scene, helper()));
  const auto after = *workcell_builder::TaskIntentModel::from_validated_yaml(reopened.model().to_yaml());
  EXPECT_EQ(before.normalized_yaml, after.normalized_yaml);
  EXPECT_EQ(workcell_builder::authoritative_task_intent_sha256(before), workcell_builder::authoritative_task_intent_sha256(after));
  EXPECT_EQ(read(scene + "/environment.yaml"), environment);
  // The reviewed profile supplies physical containment; this is not motion readiness.
  EXPECT_TRUE(reopened.blocker().isEmpty());
}

#include <QDoubleSpinBox>
TEST(TaskIntentEditor, PolicyChangesAndAdvancedFieldsKeepAuthoredConstraints) {
  application(); QTemporaryDir dir; seed(dir.path()); TaskIntentEditor editor;
  ASSERT_TRUE(editor.load_scene(dir.path(), helper()));
  const auto pose = editor.model().place.requested_local_pose->xyz_m;
  auto * policy = editor.findChild<QComboBox *>("taskPlacePolicy");
  for (const auto & mode : {"AUTO", "PREFERRED", "EXACT"}) {
    policy->setCurrentText(mode); EXPECT_EQ(editor.model().place.policy, mode);
    EXPECT_EQ(editor.model().place.requested_local_pose->xyz_m, pose);
  }
  editor.findChild<QDoubleSpinBox *>("taskPlaceApproach")->setValue(0.17);
  editor.findChild<QDoubleSpinBox *>("taskPlaceRetreat")->setValue(0.23);
  editor.findChild<QDoubleSpinBox *>("taskApertureMax")->setValue(0.081);
  edit(editor, "taskTcpXYZ", "0.01, -0.02, 0.03");
  EXPECT_DOUBLE_EQ(editor.model().place.approach_distance_m, 0.17);
  EXPECT_DOUBLE_EQ(editor.model().place.retreat_distance_m, 0.23);
  EXPECT_DOUBLE_EQ(editor.model().grasp.aperture_max_m, 0.081);
  EXPECT_DOUBLE_EQ(editor.model().grasp.tcp_offset_xyz_m[1], -0.02);
  ASSERT_TRUE(editor.save());
  TaskIntentEditor reopened; ASSERT_TRUE(reopened.load_scene(dir.path(), helper()));
  EXPECT_DOUBLE_EQ(reopened.model().grasp.tcp_offset_xyz_m[1], -0.02);
  EXPECT_DOUBLE_EQ(reopened.model().place.retreat_distance_m, 0.23);
}

#include "workcell_studio_scene_browser.hpp"
namespace {
void creation_chain(const QString & layout) {
  application(); QTemporaryDir workspace;
  const auto repo = workspace.path() + "/src/easy_manipulation_deployment";
  QDir().mkpath(repo + "/scenes/existing"); QDir().mkpath(repo + "/assets");
  write(repo + "/scenes/existing/environment.yaml", "robot: {model: ur5}\n");
  const auto output = layout == "custom" ? workspace.path() + "/custom output" : workspace.path() + "/src/scenes";
  if (layout == "alias") boost::filesystem::create_directory_symlink(
    (repo + "/scenes").toStdString(), output.toStdString());
  else QDir().mkpath(output);
  NewCellWizard wizard(workspace.path());
  ASSERT_TRUE(wizard.select_scenario_by_id("static_table_pick_place"));
  ASSERT_TRUE(wizard.select_object_source_by_id("manual_simulated"));
  for (auto * form : wizard.findChildren<QFormLayout *>())
    for (auto * field : wizard.findChildren<QLineEdit *>()) {
      auto * label = qobject_cast<QLabel *>(form->labelForField(field));
      if (!label) continue;
      if (label->text() == "Scene/package name") field->setText("created");
      if (label->text() == "Output scenes path") {
        EXPECT_EQ(field->text(), repo + "/scenes");
        field->setText(output);
      }
    }
  for (auto * button : wizard.findChildren<QPushButton *>())
    if (button->text() == "Create and Open") button->click();
  ASSERT_TRUE(wizard.result().created);
  const auto scene = wizard.result().scene_dir;
  EXPECT_EQ(scene, workcell_builder::canonical_scene_identity((output + "/created").toStdString()));
  EXPECT_TRUE(QFile::exists(QString::fromStdString(scene.string()) + "/config/workcell_builder_task_intent.yaml"));
  const auto saved = read(QString::fromStdString(scene.string()) + "/config/workcell_builder_task_intent.yaml");
  EXPECT_EQ(QJsonDocument::fromJson(saved).object().value("scene_package").toString().toStdString(), scene.string());
  // Retrying the same wizard must not replace the saved cell.
  for (auto * button : wizard.findChildren<QPushButton *>())
    if (button->text() == "Create and Open") button->click();
  EXPECT_EQ(saved, read(QString::fromStdString(scene.string()) + "/config/workcell_builder_task_intent.yaml"));
  if (layout != "alias") {
    QDir().mkpath(repo + "/scenes/created");
    write(repo + "/scenes/created/environment.yaml", "robot: {model: other_robot}\n");
  }
  const auto home = workcell_builder::discover_workcell_studio_scenes(workspace.path().toStdString(),
    layout == "custom" ? output.toStdString() : std::string());
  EXPECT_EQ(home.scenes.size(), layout == "alias" ? 2U : 3U);
  EXPECT_EQ(workcell_builder::find_scene_by_identity(home, {}, "created"),
    layout == "alias" ? workcell_builder::find_scene_by_identity(home, scene) : -1);
  EXPECT_EQ(workcell_builder::find_scene_by_identity(home, "/missing/created", "created"), -1);
  const int index = workcell_builder::find_scene_by_identity(home, scene);
  ASSERT_GE(index, 0);
  EXPECT_TRUE(home.scenes[index].has_package_xml);
  TaskIntentEditor editor;
  ASSERT_TRUE(editor.load_scene(QString::fromStdString(home.scenes[index].scene_dir.string()), helper()));
  EXPECT_EQ(editor.model().grasp.policy, "AUTO");
  EXPECT_EQ(editor.model().place.asset_ref, "target_bin_default");
}

}
TEST(TaskIntentEditor, CreateDiscoverOpenAcrossSeparateSceneRoots) { creation_chain("separate"); }
TEST(TaskIntentEditor, CreateDiscoverOpenThroughNormalAlias) { creation_chain("alias"); }
TEST(TaskIntentEditor, CreateDiscoverOpenThroughCustomOutput) { creation_chain("custom"); }
TEST(TaskIntentEditor, SaveFailureDoesNotPublishCellOrClaimCreation) {
  application(); QTemporaryDir workspace;
  write(workspace.path() + "/not_a_directory", "preserve me");
  NewCellWizard wizard(workspace.path());
  for (auto * form : wizard.findChildren<QFormLayout *>())
    for (auto * field : wizard.findChildren<QLineEdit *>()) {
      auto * label = qobject_cast<QLabel *>(form->labelForField(field));
      if (label && label->text() == "Scene/package name") field->setText("created");
    }
  wizard.set_output_root(workspace.path() + "/not_a_directory");
  for (auto * button : wizard.findChildren<QPushButton *>())
    if (button->text() == "Create and Open") button->click();
  EXPECT_FALSE(wizard.result().created);
  EXPECT_EQ(read(workspace.path() + "/not_a_directory"), "preserve me");
  bool save_error = false;
  for (auto * label : wizard.findChildren<QLabel *>())
    if (label->text().contains("Save failed at") && label->text().contains("not_a_directory")) save_error = true;
  EXPECT_TRUE(save_error);
}
TEST(TaskIntentEditor, TaskPersistenceFailureLeavesNoPartialScene) {
  application(); QTemporaryDir workspace;
  const auto repo = workspace.path() + "/src/easy_manipulation_deployment";
  QDir().mkpath(repo + "/scenes"); QDir().mkpath(repo + "/assets"); QDir().mkpath(repo + "/scripts");
  const auto profile_helper = QFileInfo(helper()).dir().filePath("instantiate_workcell_studio_profile.py");
  write(repo + "/scripts/instantiate_workcell_studio_profile.py",
    ("import sys, subprocess\nif '--describe' in sys.argv:\n subprocess.run([sys.executable, '" + profile_helper +
      "'] + sys.argv[1:], check=True)\nelse:\n print('Authoring unavailable', file=sys.stderr)\n sys.exit(1)\n").toUtf8());
  NewCellWizard wizard(workspace.path());
  ASSERT_TRUE(wizard.select_scenario_by_id("static_table_pick_place"));
  for (auto * form : wizard.findChildren<QFormLayout *>())
    for (auto * field : wizard.findChildren<QLineEdit *>()) {
      auto * label = qobject_cast<QLabel *>(form->labelForField(field));
      if (label && label->text() == "Scene/package name") field->setText("created");
    }
  for (auto * button : wizard.findChildren<QPushButton *>())
    if (button->text() == "Create and Open") button->click();
  EXPECT_FALSE(wizard.result().created);
  EXPECT_FALSE(QFile::exists(repo + "/scenes/created"));
  EXPECT_TRUE(QDir(repo + "/scenes").entryList(QDir::AllEntries | QDir::Hidden | QDir::NoDotAndDotDot).isEmpty());
  bool task_error = false;
  for (auto * label : wizard.findChildren<QLabel *>())
    if (label->text().contains("Physical scene preparation failed") && label->text().contains("Authoring unavailable")) task_error = true;
  EXPECT_TRUE(task_error);
}

#include <QMessageBox>
#include <QTimer>
TEST(TaskIntentEditor, CancelSceneSwitchPreservesUnsavedDraft) {
  application(); QTemporaryDir first; QTemporaryDir second; seed(first.path()); seed(second.path());
  TaskIntentEditor editor; ASSERT_TRUE(editor.load_scene(first.path(), helper()));
  edit(editor, "taskTargetClass", "unsaved_part");
  QTimer cancel; cancel.setSingleShot(true);
  QObject::connect(&cancel, &QTimer::timeout, [] {
    for (auto * widget : QApplication::topLevelWidgets())
      if (auto * dialog = qobject_cast<QMessageBox *>(widget)) dialog->button(QMessageBox::Cancel)->click();
  });
  cancel.start(0);
  EXPECT_FALSE(editor.load_scene(second.path(), helper()));
  EXPECT_EQ(editor.scene(), first.path());
  EXPECT_TRUE(editor.dirty());
  EXPECT_EQ(editor.model().pick_selection.class_id, "unsaved_part");
}
TEST(TaskIntentEditor, ExternalChangeBlocksValidationAndCleanReopenReloads) {
  application(); QTemporaryDir dir; seed(dir.path()); TaskIntentEditor editor;
  ASSERT_TRUE(editor.load_scene(dir.path(), helper()));
  const auto path = dir.path() + "/config/workcell_builder_task_intent.yaml";
  auto bytes = read(path); bytes.replace("class_id: bottle", "class_id: changed_on_disk"); write(path, bytes);
  editor.validate_now();
  EXPECT_FALSE(editor.blocker().isEmpty());
  ASSERT_TRUE(editor.load_scene(dir.path(), helper()));
  EXPECT_EQ(editor.model().pick_selection.class_id, "changed_on_disk");
  EXPECT_FALSE(editor.dirty());
  EXPECT_TRUE(editor.blocker().isEmpty());
}
TEST(TaskIntentEditor, RefreshPreservesDirtyDraftWhenDiskChanges) {
  application(); QTemporaryDir dir; seed(dir.path()); TaskIntentEditor editor;
  ASSERT_TRUE(editor.load_scene(dir.path(), helper()));
  edit(editor, "taskTargetClass", "unsaved_part");
  const auto path = dir.path() + "/config/workcell_builder_task_intent.yaml";
  auto bytes = read(path); bytes.replace("class_id: bottle", "class_id: external_part"); write(path, bytes);
  ASSERT_TRUE(editor.load_scene(dir.path(), helper()));
  EXPECT_EQ(editor.model().pick_selection.class_id, "unsaved_part");
  EXPECT_TRUE(editor.dirty());
  EXPECT_FALSE(editor.save());
  EXPECT_EQ(read(path), bytes);
}

TEST(TaskIntentEditor, CanonicalSceneSaveCloseReopenPreservesPhysicalTruth) {
  application(); QTemporaryDir dir; QDir().mkpath(dir.path() + "/config");
  const auto source = QStringLiteral(WORKCELL_BUILDER_REPO_ROOT) + "/scenes/ur5_2f_test";
  const auto environment = read(source + "/environment.yaml");
  const auto intent = read(source + "/config/workcell_builder_task_intent.yaml");
  ASSERT_FALSE(environment.isEmpty()); ASSERT_FALSE(intent.isEmpty());
  write(dir.path() + "/environment.yaml", environment);
  write(dir.path() + "/config/workcell_builder_task_intent.yaml", intent);
  std::string normalized, hash;
  {
    TaskIntentEditor editor; ASSERT_TRUE(editor.load_scene(dir.path(), helper()));
    editor.show(); QApplication::processEvents();
    edit(editor, "taskTargetClass", "acceptance_part");
    editor.findChild<QDoubleSpinBox *>("taskApproachDistance")->setValue(0.13);
    editor.findChild<QComboBox *>("taskPlacePolicy")->setCurrentText("EXACT");
    edit(editor, "taskLocalXYZ", "0.01, 0, 0.01");
    edit(editor, "taskLocalRPY", "0, 0, 0");
    editor.findChild<QPushButton *>("taskSave")->click();
    ASSERT_FALSE(editor.dirty()); ASSERT_TRUE(editor.blocker().isEmpty());
    const auto model = workcell_builder::TaskIntentModel::from_validated_yaml(editor.model().to_yaml());
    ASSERT_TRUE(model); normalized = model->normalized_yaml;
    hash = workcell_builder::authoritative_task_intent_sha256(*model);
    EXPECT_EQ(model->place.asset_ref, "target_bin_default");
    EXPECT_EQ(model->place.region_ref, "default_drop_zone");
    EXPECT_DOUBLE_EQ(model->place.requested_local_pose->xyz_m[0], 0.01);
    const auto output = qEnvironmentVariable("WORKCELL_TASK_EVIDENCE_DIR");
    if (!output.isEmpty()) {
      QDir().mkpath(output); QApplication::processEvents();
      EXPECT_TRUE(editor.grab().save(output + "/canonical-editor.png"));
      write(output + "/normalized-intent.json", QByteArray::fromStdString(normalized));
      write(output + "/intent.sha256", QByteArray::fromStdString(hash));
    }
  }
  TaskIntentEditor reopened; ASSERT_TRUE(reopened.load_scene(dir.path(), helper()));
  const auto model = workcell_builder::TaskIntentModel::from_validated_yaml(reopened.model().to_yaml());
  ASSERT_TRUE(model); EXPECT_EQ(model->normalized_yaml, normalized);
  EXPECT_EQ(workcell_builder::authoritative_task_intent_sha256(*model), hash);
  EXPECT_EQ(model->pick_selection.class_id, "acceptance_part");
  EXPECT_DOUBLE_EQ(model->grasp.approach_distance_m, 0.13);
  EXPECT_EQ(read(dir.path() + "/environment.yaml"), environment);
  EXPECT_EQ(read(source + "/environment.yaml"), environment);
  EXPECT_EQ(read(source + "/config/workcell_builder_task_intent.yaml"), intent);
}

TEST(TaskIntentEditor, SaveAndDiscardSceneSwitchDecisionsAreHonored) {
  application();
  for (const auto answer : {QMessageBox::Save, QMessageBox::Discard}) {
    QTemporaryDir first; QTemporaryDir second; seed(first.path()); seed(second.path());
    TaskIntentEditor editor; ASSERT_TRUE(editor.load_scene(first.path(), helper()));
    const auto path = first.path() + "/config/workcell_builder_task_intent.yaml";
    const auto original = read(path);
    edit(editor, "taskTargetClass", "saved_part");
    QTimer reply; reply.setSingleShot(true);
    QObject::connect(&reply, &QTimer::timeout, [answer] {
      for (auto * widget : QApplication::topLevelWidgets())
        if (auto * dialog = qobject_cast<QMessageBox *>(widget)) dialog->button(answer)->click();
    });
    reply.start(0);
    ASSERT_TRUE(editor.load_scene(second.path(), helper()));
    EXPECT_EQ(editor.scene(), second.path()); EXPECT_FALSE(editor.dirty());
    ASSERT_TRUE(editor.load_scene(first.path(), helper()));
    EXPECT_EQ(editor.model().pick_selection.class_id, answer == QMessageBox::Save ? "saved_part" : "bottle");
    if (answer == QMessageBox::Discard) EXPECT_EQ(read(path), original);
  }
}

TEST(TaskIntentEditor, RuntimeParityCreatesFreshCellAndSavesReplayTask) {
  application();
  const auto workspace = qEnvironmentVariable("R20DE_ACCEPTANCE_WORKSPACE");
  if (workspace.isEmpty()) GTEST_SKIP() << "Set R20DE_ACCEPTANCE_WORKSPACE for retained runtime acceptance cells";
  QDir().mkpath(workspace + "/src/easy_manipulation_deployment/scenes");
  QDir().mkpath(workspace + "/src/easy_manipulation_deployment/assets");
  NewCellWizard wizard(workspace);
  ASSERT_TRUE(wizard.select_scenario_by_id("static_table_pick_place"));
  ASSERT_TRUE(wizard.select_object_source_by_id("manual_simulated"));
  for (auto * form : wizard.findChildren<QFormLayout *>())
    for (auto * field : wizard.findChildren<QLineEdit *>()) {
      auto * label = qobject_cast<QLabel *>(form->labelForField(field));
      if (label && label->text() == "Scene/package name") field->setText("r20de_fresh_cell");
    }
  for (auto * button : wizard.findChildren<QPushButton *>())
    if (button->text() == "Create and Open") button->click();
  ASSERT_TRUE(wizard.result().created);
  const auto fresh = QString::fromStdString(wizard.result().scene_dir.string());
  const auto canonical = workspace + "/src/easy_manipulation_deployment/scenes/ur5_2f_test";
  for (const auto & scene : {canonical, fresh}) {
    const auto environment = read(scene + "/environment.yaml");
    std::string before_hash;
    {
      TaskIntentEditor editor; ASSERT_TRUE(editor.load_scene(scene, helper()));
      edit(editor, "taskTargetClass", "cup");
      editor.findChild<QComboBox *>("taskGraspPolicy")->setCurrentText("AUTO");
      editor.findChild<QComboBox *>("taskPlacePolicy")->setCurrentText("AUTO");
      editor.findChild<QDoubleSpinBox *>("taskMaximumAge")->setValue(300.0);
      editor.findChild<QDoubleSpinBox *>("taskPlaceClearance")->setValue(0.001);
      ASSERT_TRUE(editor.save());
      auto model = workcell_builder::TaskIntentModel::from_validated_yaml(editor.model().to_yaml());
      ASSERT_TRUE(model);
      before_hash = workcell_builder::authoritative_task_intent_sha256(*model);
    }
    TaskIntentEditor reopened; ASSERT_TRUE(reopened.load_scene(scene, helper()));
    auto model = workcell_builder::TaskIntentModel::from_validated_yaml(reopened.model().to_yaml());
    ASSERT_TRUE(model);
    const auto reopened_hash = workcell_builder::authoritative_task_intent_sha256(*model);
    EXPECT_EQ(before_hash, reopened_hash);
    QProcess python;
    python.start("python3", {helper(), scene});
    ASSERT_TRUE(python.waitForFinished(10000));
    const auto report = QJsonDocument::fromJson(python.readAllStandardOutput()).object();
    EXPECT_EQ(report.value("normalized_intent_sha256").toString().toStdString(), reopened_hash);
    QJsonObject evidence{{"scene", QFileInfo(scene).fileName()},
      {"cpp_saved_sha256", QString::fromStdString(before_hash)},
      {"cpp_reopened_sha256", QString::fromStdString(reopened_hash)},
      {"python_normalized_sha256", report.value("normalized_intent_sha256")},
      {"environment_unchanged", environment == read(scene + "/environment.yaml")}};
    write(workspace + "/" + QFileInfo(scene).fileName() + "-authoring.json", QJsonDocument(evidence).toJson());
    EXPECT_EQ(environment, read(scene + "/environment.yaml"));
    EXPECT_TRUE(reopened.blocker().isEmpty());
  }
}
