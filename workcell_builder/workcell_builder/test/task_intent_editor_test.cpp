#include <QFileInfo>
#include <gtest/gtest.h>
#include <QListWidget>
#include <QApplication>
#include <QComboBox>
#include <QDir>
#include <QFile>
#include <QCryptographicHash>
#include <QJsonArray>
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
- {id: pick, type: pick_zone, frame: world, pose_xyz: [0.3, 0, 0.2], pose_rpy: [0, 0, 0], dimensions: [1, 1, 1]}
- {id: drop, target_ref: bin, frame: world, pose_xyz: [1.05, 2, 3.1], pose_rpy: [0, 0, 0], dimensions: [0.2, 0.1, 0.1], placement_local: {pose_xyz: [0.05, 0, 0.1], pose_rpy: [0, 0, 0], dimensions: [0.2, 0.1, 0.1]}}
)");
  write(scene + "/config/workcell_builder_task_intent.yaml", R"(schema: workcell_builder_task_intent/v2
task: {id: t, type: pick_place, template: pick_place}
pick: {selection: {source_ref: source, source_type: perception, zone_ref: pick, object_filter: {class_id: bottle, min_confidence: null, max_age_seconds: 2.0}}, grasp: {policy: EXACT, required_capability: two_finger_parallel, strategy_ref: top_2f, approach: {axis: z_down, distance_m: 0.123456789123}, orientation: {mode: vertical}, lift: {axis: z_up, distance_m: 0.15}}}
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
  // A new task without explicit observations is a savable draft, never plan-ready.
  EXPECT_TRUE(reopened.blocker().contains("TASK_INPUT_UNAVAILABLE"));
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
      auto * input_mode = editor.findChild<QComboBox *>("taskPerceptionMode");
      input_mode->setCurrentIndex(input_mode->findData("replayed_snapshot"));
      edit(editor, "taskReplayPath", QStringLiteral(WORKCELL_BUILDER_REPO_ROOT) + "/config/runtime/r1_4_replay.yaml");
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

TEST(TaskIntentEditor, MultiObjectPreviewIsSavedReplayOnlyAndDoesNotLaunchOnSave) {
  application(); QTemporaryDir dir; seed(dir.path());
  TaskIntentEditor editor; ASSERT_TRUE(editor.load_scene(dir.path(), helper()));
  auto * preview = editor.findChild<QPushButton *>("taskSimulationBatchPreview");
  auto * mode = editor.findChild<QComboBox *>("taskPerceptionMode");
  ASSERT_NE(preview, nullptr); ASSERT_NE(mode, nullptr);
  ASSERT_NE(editor.findChild<QLabel *>("taskSimulationBatchStatus"), nullptr);
  EXPECT_FALSE(preview->isEnabled());  // No user-selected input; no implicit default.
  mode->setCurrentIndex(mode->findData("replayed_snapshot"));
  edit(editor, "taskReplayPath",
       QStringLiteral(WORKCELL_BUILDER_REPO_ROOT) + "/config/runtime/r1_4_replay.yaml");
  EXPECT_FALSE(preview->isEnabled());  // Unsaved task cannot launch preview.
  ASSERT_TRUE(editor.save());
  EXPECT_TRUE(preview->isEnabled());  // Enabled, not automatically executed.
  edit(editor, "taskTargetClass", "cup");
  EXPECT_FALSE(preview->isEnabled());
  ASSERT_TRUE(editor.save());
  EXPECT_TRUE(preview->isEnabled());
  mode->setCurrentIndex(mode->findData("live_epd"));
  ASSERT_TRUE(editor.save());
  EXPECT_FALSE(preview->isEnabled());  // Live camera never masquerades as commissioning.
  mode->setCurrentIndex(mode->findData("off"));
  ASSERT_TRUE(editor.save());
  EXPECT_TRUE(preview->isEnabled());  // Explicit saved replay remains a commissioning input.
  edit(editor, "taskReplayPath", "");
  ASSERT_TRUE(editor.save());
  EXPECT_FALSE(preview->isEnabled());
}

TEST(TaskIntentEditor, PlanCheckIsSavedReplayOnlyAndNeverStartsByItself) {
  application(); QTemporaryDir dir; seed(dir.path());
  TaskIntentEditor editor; ASSERT_TRUE(editor.load_scene(dir.path(), helper()));
  auto * check = editor.findChild<QPushButton *>("taskSimulationBatchPlanCheck");
  auto * cancel = editor.findChild<QPushButton *>("taskSimulationBatchPlanCancel");
  auto * mode = editor.findChild<QComboBox *>("taskPerceptionMode");
  ASSERT_NE(check, nullptr); ASSERT_NE(cancel, nullptr);
  EXPECT_FALSE(check->isEnabled());
  EXPECT_FALSE(cancel->isEnabled());
  mode->setCurrentIndex(mode->findData("replayed_snapshot"));
  edit(editor, "taskReplayPath",
       QStringLiteral(WORKCELL_BUILDER_REPO_ROOT) + "/config/runtime/r1_4_replay.yaml");
  EXPECT_FALSE(check->isEnabled());  // Unsaved edits never reach MoveIt.
  ASSERT_TRUE(editor.save());
  EXPECT_TRUE(check->isEnabled());
  EXPECT_FALSE(cancel->isEnabled());  // Saving does not start a check.
  edit(editor, "taskTargetClass", "cup");
  EXPECT_FALSE(check->isEnabled());
  mode->setCurrentIndex(mode->findData("live_epd"));
  ASSERT_TRUE(editor.save());
  EXPECT_FALSE(check->isEnabled());  // Live input is not a commissioning plan check.
}

TEST(TaskIntentEditor, SequenceCheckIsSavedReplayOnlyAndOrderClearsOnSceneLoad) {
  application(); QTemporaryDir dir; seed(dir.path());
  TaskIntentEditor editor; ASSERT_TRUE(editor.load_scene(dir.path(), helper()));
  auto * sequence = editor.findChild<QPushButton *>("taskSimulationSequenceCheck");
  auto * order = editor.findChild<QListWidget *>("taskSequenceOrder");
  auto * mode = editor.findChild<QComboBox *>("taskPerceptionMode");
  ASSERT_NE(sequence, nullptr); ASSERT_NE(order, nullptr);
  EXPECT_FALSE(sequence->isEnabled());
  EXPECT_EQ(order->dragDropMode(), QAbstractItemView::InternalMove);  // Operator reordering.
  mode->setCurrentIndex(mode->findData("replayed_snapshot"));
  edit(editor, "taskReplayPath",
       QStringLiteral(WORKCELL_BUILDER_REPO_ROOT) + "/config/runtime/r1_4_replay.yaml");
  EXPECT_FALSE(sequence->isEnabled());
  ASSERT_TRUE(editor.save());
  EXPECT_TRUE(sequence->isEnabled());  // Enabled, never started by saving.
  EXPECT_FALSE(editor.findChild<QPushButton *>("taskSimulationBatchPlanCancel")->isEnabled());
  order->addItem("runtime::sample-cup");
  edit(editor, "taskTargetClass", "cup");
  EXPECT_FALSE(sequence->isEnabled());
  QTemporaryDir other; seed(other.path());
  ASSERT_TRUE(editor.save());
  ASSERT_TRUE(editor.load_scene(other.path(), helper()));
  EXPECT_EQ(order->count(), 0);  // A queue from another cell never carries over.
}

namespace {
// A saved two-cup sequence: cup-02 Automatic first, then cup-01 at the far-end place.
QString seed_sequence(const QString & scene) {
  seed(scene);
  const QByteArray recording = R"(schema_version: detected_objects/v1
source: {mode: replayed_snapshot, description: two cups}
objects:
- {object_id: cup-01, class_id: cup, confidence: 0.99, age_seconds: 0, pose: {frame_id: world, xyz: [0.3, -0.1, 0.05], rpy: [0, 0, 0]}, dimensions: [0.06, 0.06, 0.1]}
- {object_id: cup-02, class_id: cup, confidence: 0.98, age_seconds: 0, pose: {frame_id: world, xyz: [0.4, -0.1, 0.05], rpy: [0, 0, 0]}, dimensions: [0.06, 0.06, 0.1]}
)";
  write(scene + "/recording.yaml", recording);
  const QString sha = QCryptographicHash::hash(recording, QCryptographicHash::Sha256).toHex();
  write(scene + "/config/workcell_builder_task_intent.yaml", QString(R"(schema: workcell_builder_task_intent/v2
task: {id: t, type: pick_place, template: pick_place}
pick: {selection: {source_ref: detected_objects/v1, source_type: perception, zone_ref: pick, object_filter: {class_id: cup, min_confidence: null}}, grasp: {policy: AUTO, required_capability: two_finger_parallel, approach: {axis: z_down, distance_m: 0.1}, lift: {axis: z_up, distance_m: 0.15}}}
place: {target: {asset_ref: bin, region_ref: drop}, placement: {policy: AUTO, clearance_m: 0.0, orientation: {mode: target_default}, approach: {axis: z_down, distance_m: 0.1}, retreat: {axis: z_up, distance_m: 0.1}}, release: {strategy: tool_release}}
perception: {mode: replayed_snapshot, replay_path: recording.yaml}
safety: {require_fake_hardware: true, real_hardware_enabled: false}
sequence:
  source_sha256: %1
  items:
  - {object_id: 'runtime::cup-02', class_id: cup, observed_xyz_m: [0.4, -0.1, 0.05], dimensions_m: [0.06, 0.06, 0.1], placement: {mode: AUTO}}
  - {object_id: 'runtime::cup-01', class_id: cup, observed_xyz_m: [0.3, -0.1, 0.05], dimensions_m: [0.06, 0.06, 0.1], placement: {mode: MANUAL, asset_ref: bin, region_ref: drop, local_pose: {xyz_m: [0.117, 0, 0.1], rpy_rad: [0, 0, 0]}}}
)").arg(sha).toUtf8());
  return sha;
}
void choose_place(TaskIntentEditor & editor, const QString & label) {
  auto * place = editor.findChild<QComboBox *>("taskSequencePlace");
  const int index = place->findText(label); ASSERT_GE(index, 0) << label.toStdString();
  place->setCurrentIndex(index); emit place->activated(index);
}
}  // namespace

TEST(TaskIntentEditor, SavedSequenceReopensEditsRevertsAndSaves) {
  application(); QTemporaryDir dir; seed_sequence(dir.path());
  TaskIntentEditor editor; ASSERT_TRUE(editor.load_scene(dir.path(), helper()));
  auto * order = editor.findChild<QListWidget *>("taskSequenceOrder");
  auto * summary = editor.findChild<QLabel *>("taskSequenceSummary");
  auto * revert = editor.findChild<QPushButton *>("taskRevert");
  ASSERT_NE(order, nullptr); ASSERT_NE(summary, nullptr); ASSERT_NE(revert, nullptr);
  // Reopened exactly as saved, without a Preview: order, bindings and places.
  ASSERT_EQ(order->count(), 2);
  EXPECT_TRUE(order->item(0)->text().startsWith("1. cup-02 (cup)  →  Automatic")) << order->item(0)->text().toStdString();
  EXPECT_EQ(order->item(1)->text(), "2. cup-01 (cup)  →  Place 3 of 3 — far end");
  EXPECT_TRUE(summary->text().contains("Ready for the full-sequence check")) << summary->text().toStdString();
  EXPECT_FALSE(editor.dirty()); EXPECT_FALSE(revert->isEnabled());
  order->setCurrentRow(1);
  EXPECT_EQ(editor.findChild<QComboBox *>("taskSequencePlace")->currentText(), "Place 3 of 3 — far end");
  EXPECT_TRUE(editor.findChild<QLabel *>("taskSequencePlaceDetail")->text().contains("x 0.117, y 0.000, z 0.100 m"));
  // Choose another place: unsaved, revertible.
  choose_place(editor, "Place 1 of 3 — near end");
  EXPECT_TRUE(editor.dirty()); EXPECT_TRUE(revert->isEnabled());
  EXPECT_EQ(order->item(1)->text(), "2. cup-01 (cup)  →  Place 1 of 3 — near end");
  editor.revert();
  EXPECT_FALSE(editor.dirty());
  EXPECT_EQ(order->item(1)->text(), "2. cup-01 (cup)  →  Place 3 of 3 — far end");
  // Reorder by dragging (a model row move), save, reopen.
  ASSERT_TRUE(order->model()->moveRow(QModelIndex(), 1, QModelIndex(), 0));
  EXPECT_TRUE(editor.dirty());
  ASSERT_TRUE(editor.save());
  TaskIntentEditor reopened; ASSERT_TRUE(reopened.load_scene(dir.path(), helper()));
  auto * again = reopened.findChild<QListWidget *>("taskSequenceOrder");
  ASSERT_EQ(again->count(), 2);
  EXPECT_EQ(again->item(0)->text(), "1. cup-01 (cup)  →  Place 3 of 3 — far end");
  EXPECT_TRUE(again->item(1)->text().startsWith("2. cup-02 (cup)  →  Automatic"));
  const auto saved = read(dir.path() + "/config/workcell_builder_task_intent.yaml");
  EXPECT_LT(saved.indexOf("runtime::cup-01"), saved.indexOf("runtime::cup-02"));
  EXPECT_FALSE(saved.contains("Place 3"));  // Labels are never the stored truth.
}

TEST(TaskIntentEditor, SequenceProblemsAreShownAndNeverSilentlyRebound) {
  application(); QTemporaryDir dir; const QString sha = seed_sequence(dir.path());
  TaskIntentEditor editor; ASSERT_TRUE(editor.load_scene(dir.path(), helper()));
  auto * order = editor.findChild<QListWidget *>("taskSequenceOrder");
  auto * summary = editor.findChild<QLabel *>("taskSequenceSummary");
  order->setCurrentRow(0);
  choose_place(editor, "Place 3 of 3 — far end");  // Same place as cup-01.
  editor.validate_now();
  EXPECT_TRUE(summary->text().startsWith("Unsaved changes — NEEDS REVIEW")) << summary->text().toStdString();
  EXPECT_TRUE(order->item(1)->text().startsWith("⚠ 2. cup-01")) << order->item(1)->text().toStdString();
  EXPECT_TRUE(order->item(1)->toolTip().startsWith("PLACE_CONFLICT"));
  editor.revert();
  // Same object IDs, different detection: blocked, and the saved binding stays.
  QByteArray recording = read(dir.path() + "/recording.yaml");
  write(dir.path() + "/recording.yaml", recording.replace("xyz: [0.4, -0.1, 0.05]", "xyz: [0.45, -0.1, 0.05]"));
  TaskIntentEditor reopened; ASSERT_TRUE(reopened.load_scene(dir.path(), helper()));
  auto * blocked = reopened.findChild<QLabel *>("taskSequenceSummary");
  EXPECT_TRUE(blocked->text().startsWith("BLOCKED")) << blocked->text().toStdString();
  EXPECT_TRUE(blocked->text().contains("SEQUENCE_SOURCE_CHANGED"));
  EXPECT_TRUE(reopened.findChild<QListWidget *>("taskSequenceOrder")->item(0)->text().endsWith("BLOCKED"));
  EXPECT_FALSE(reopened.dirty());
  EXPECT_TRUE(read(dir.path() + "/config/workcell_builder_task_intent.yaml").contains(sha.toUtf8()));
}

TEST(TaskIntentEditor, TaskWithoutSequenceLoadsWithEmptyList) {
  application(); QTemporaryDir dir; seed(dir.path());
  TaskIntentEditor editor; ASSERT_TRUE(editor.load_scene(dir.path(), helper()));
  EXPECT_EQ(editor.findChild<QListWidget *>("taskSequenceOrder")->count(), 0);
  EXPECT_TRUE(editor.findChild<QLabel *>("taskSequenceSummary")->text().startsWith("No saved sequence"));
  EXPECT_FALSE(editor.findChild<QComboBox *>("taskSequencePlace")->isEnabled());
  EXPECT_FALSE(editor.findChild<QPushButton *>("taskRevert")->isEnabled());
}

TEST(TaskIntentEditor, PerceptionBindingPersistsAndMissingInputBlocksPlanning) {
  application(); QTemporaryDir dir; seed(dir.path()); TaskIntentEditor editor;
  ASSERT_TRUE(editor.load_scene(dir.path(), helper()));
  auto * mode = editor.findChild<QComboBox *>("taskPerceptionMode"); ASSERT_NE(mode, nullptr);
  mode->setCurrentIndex(mode->findData("replayed_snapshot"));
  edit(editor, "taskReplayPath", QStringLiteral(WORKCELL_BUILDER_REPO_ROOT) + "/config/runtime/r1_4_replay.yaml");
  ASSERT_TRUE(editor.save()); EXPECT_TRUE(editor.blocker().isEmpty()) << editor.blocker().toStdString();
  editor.bind_pick("pick");
  EXPECT_EQ(editor.model().pick_selection.source_ref, "detected_objects/v1");
  ASSERT_TRUE(editor.save());
  TaskIntentEditor reopened; ASSERT_TRUE(reopened.load_scene(dir.path(), helper()));
  EXPECT_EQ(reopened.findChild<QComboBox *>("taskPerceptionMode")->currentData().toString(), "replayed_snapshot");
  edit(reopened, "taskReplayPath", "missing-recording.yaml");
  ASSERT_TRUE(reopened.save()); EXPECT_TRUE(reopened.blocker().contains("TASK_INPUT_UNAVAILABLE"));
  EXPECT_TRUE(reopened.blocker(false).isEmpty());  // Physical-only Generate must remain available.
  mode = reopened.findChild<QComboBox *>("taskPerceptionMode"); mode->setCurrentIndex(mode->findData("off"));
  EXPECT_EQ(reopened.model().pick_selection.source_type, "manual_simulated");
  EXPECT_EQ(reopened.model().pick_selection.source_ref, "pick");
}


TEST(TaskIntentEditor, ExplicitLiveCameraSourceSurvivesReopenAndClearsOutsideLiveMode) {
  application(); QTemporaryDir dir; seed(dir.path());
  TaskIntentEditor editor; ASSERT_TRUE(editor.load_scene(dir.path(), helper()));
  auto * mode = editor.findChild<QComboBox *>("taskPerceptionMode");
  auto * camera = editor.findChild<QComboBox *>("taskCameraSource");
  ASSERT_NE(mode, nullptr); ASSERT_NE(camera, nullptr);
  auto * capture = editor.findChild<QPushButton *>("taskCameraCapture");
  auto * preview = editor.findChild<QLabel *>("taskCameraCaptureStatus");
  ASSERT_NE(capture, nullptr); ASSERT_NE(preview, nullptr);
  auto * live_epd = editor.findChild<QPushButton *>("taskCaptureLiveEpd");
  ASSERT_NE(live_epd, nullptr);
  ASSERT_NE(editor.findChild<QLabel *>("taskCaptureLiveEpdStatus"), nullptr);
  auto * preflight = editor.findChild<QPushButton *>("taskLiveScenePreflight");
  ASSERT_NE(preflight, nullptr);
  ASSERT_NE(editor.findChild<QLabel *>("taskLiveScenePreflightStatus"), nullptr);
  EXPECT_FALSE(preflight->isEnabled());
  EXPECT_FALSE(capture->isEnabled());
  EXPECT_FALSE(live_epd->isEnabled());
  mode->setCurrentIndex(mode->findData("live_epd"));
  ASSERT_TRUE(camera->isEnabled());
  camera->setCurrentIndex(camera->findData("simulated_rgbd"));
  EXPECT_FALSE(capture->isEnabled());  // Unsaved camera edits cannot probe.
  EXPECT_FALSE(live_epd->isEnabled());
  EXPECT_FALSE(preflight->isEnabled());
  ASSERT_TRUE(editor.save());
  EXPECT_TRUE(preflight->isEnabled());  // Check is explicit; no ROS starts by saving.
  EXPECT_TRUE(capture->isEnabled());  // No ROS launched until explicit click.
  EXPECT_TRUE(live_epd->isEnabled());  // No EPD subscribed until explicit click.
  EXPECT_TRUE(editor.blocker().contains("TASK_INPUT_UNAVAILABLE"));

  TaskIntentEditor reopened; ASSERT_TRUE(reopened.load_scene(dir.path(), helper()));
  auto * source = reopened.findChild<QComboBox *>("taskCameraSource"); ASSERT_NE(source, nullptr);
  EXPECT_EQ(source->currentData().toString(), "simulated_rgbd");
  auto * live_mode = reopened.findChild<QComboBox *>("taskPerceptionMode");
  EXPECT_EQ(live_mode->currentData().toString(), "live_epd");
  source->setCurrentIndex(source->findData("realsense_d435i"));
  ASSERT_TRUE(reopened.save());

  TaskIntentEditor d435i; ASSERT_TRUE(d435i.load_scene(dir.path(), helper()));
  auto * physical = d435i.findChild<QComboBox *>("taskCameraSource");
  ASSERT_NE(physical, nullptr);
  EXPECT_EQ(physical->currentData().toString(), "realsense_d435i");
  auto * selected_mode = d435i.findChild<QComboBox *>("taskPerceptionMode");
  selected_mode->setCurrentIndex(selected_mode->findData("off"));
  EXPECT_FALSE(physical->isEnabled());
  EXPECT_FALSE(d435i.findChild<QPushButton *>("taskCameraCapture")->isEnabled());
  EXPECT_FALSE(d435i.findChild<QPushButton *>("taskCaptureLiveEpd")->isEnabled());
  EXPECT_FALSE(d435i.findChild<QPushButton *>("taskLiveScenePreflight")->isEnabled());
  ASSERT_TRUE(d435i.save());
  const auto task = QJsonDocument::fromJson(QByteArray::fromStdString(
    workcell_builder::canonical_task_intent_json_for_testing(d435i.model().to_yaml()))).object();
  EXPECT_TRUE(task.value("perception").toObject().value("camera_source").isNull());
}

TEST(TaskIntentEditor, DestinationChangeSurvivesReorderToggleSaveUntilPlaceReselected) {
  application(); QTemporaryDir dir; seed_sequence(dir.path());
  auto environment = read(dir.path() + "/environment.yaml");
  environment.replace("assets:\n", "assets:\n- {id: bin2, frame: world, pose_xyz: [1, 2, 3], pose_rpy: [0, 0, 0], collision: {enabled: true}, usable_placement: {pose_xyz: [0, 0, 0.1], pose_rpy: [0, 0, 0], dimensions: [0.4, 0.3, 0.2]}}\n");
  environment += "- {id: drop2, target_ref: bin2, frame: world, pose_xyz: [1.05, 2, 3.1], pose_rpy: [0, 0, 0], dimensions: [0.2, 0.1, 0.1], placement_local: {pose_xyz: [0.05, 0, 0.1], pose_rpy: [0, 0, 0], dimensions: [0.2, 0.1, 0.1]}}\n";
  write(dir.path() + "/environment.yaml", environment);
  TaskIntentEditor editor; ASSERT_TRUE(editor.load_scene(dir.path(), helper()));
  editor.bind_destination("drop2");
  editor.validate_now();
  auto * order = editor.findChild<QListWidget *>("taskSequenceOrder");
  ASSERT_EQ(order->count(), 2);
  ASSERT_TRUE(order->model()->moveRow(QModelIndex(), 1, QModelIndex(), 0));
  order->item(1)->setCheckState(Qt::Unchecked);
  order->item(1)->setCheckState(Qt::Checked);
  editor.validate_now();
  const auto root = QJsonDocument::fromJson(QByteArray::fromStdString(
      workcell_builder::canonical_task_intent_json_for_testing(editor.model().to_yaml()))).object();
  const auto placement = root["sequence"].toObject()["items"].toArray()[0].toObject()["placement"].toObject();
  EXPECT_EQ(placement["asset_ref"].toString(), "bin");
  EXPECT_EQ(placement["region_ref"].toString(), "drop");
  EXPECT_TRUE(editor.findChild<QLabel *>("taskSequenceSummary")->text().contains("DESTINATION_CHANGED"));
  ASSERT_TRUE(editor.save());
  TaskIntentEditor reopened; ASSERT_TRUE(reopened.load_scene(dir.path(), helper()));
  EXPECT_TRUE(reopened.findChild<QLabel *>("taskSequenceSummary")->text().contains("DESTINATION_CHANGED"));
  auto * again = reopened.findChild<QListWidget *>("taskSequenceOrder");
  again->setCurrentRow(0);
  choose_place(reopened, "Place 3 of 3 — far end");
  ASSERT_TRUE(reopened.save());
  const auto updated = QJsonDocument::fromJson(QByteArray::fromStdString(
      workcell_builder::canonical_task_intent_json_for_testing(reopened.model().to_yaml()))).object();
  const auto selected = updated["sequence"].toObject()["items"].toArray()[0].toObject()["placement"].toObject();
  EXPECT_EQ(selected["asset_ref"].toString(), "bin2");
  EXPECT_EQ(selected["region_ref"].toString(), "drop2");
}

TEST(TaskIntentEditor, ExecutionBackendIsAnExplicitAuthoredChoiceThatSurvivesReopen) {
  application(); QTemporaryDir dir; seed(dir.path());
  TaskIntentEditor editor; ASSERT_TRUE(editor.load_scene(dir.path(), helper()));
  auto * backend = editor.findChild<QComboBox *>("taskExecutionBackend"); ASSERT_NE(backend, nullptr);
  EXPECT_EQ(backend->currentData().toString(), "fake");  // absent backend: MoveIt fake hardware, as before
  backend->setCurrentIndex(backend->findData("simulator"));
  ASSERT_TRUE(editor.dirty()); ASSERT_TRUE(editor.save());
  const auto saved = QJsonDocument::fromJson(read(dir.path() + "/config/workcell_builder_task_intent.yaml")).object()["safety"].toObject();
  EXPECT_EQ(saved["execution_backend"].toString(), "simulator");
  EXPECT_FALSE(saved["require_fake_hardware"].toBool(true));
  EXPECT_FALSE(saved["real_hardware_enabled"].toBool(true));
  TaskIntentEditor reopened; ASSERT_TRUE(reopened.load_scene(dir.path(), helper()));
  EXPECT_EQ(reopened.findChild<QComboBox *>("taskExecutionBackend")->currentData().toString(), "simulator");
  auto * again = reopened.findChild<QComboBox *>("taskExecutionBackend");
  again->setCurrentIndex(again->findData("fake")); ASSERT_TRUE(reopened.save());
  const auto reverted = QJsonDocument::fromJson(read(dir.path() + "/config/workcell_builder_task_intent.yaml")).object()["safety"].toObject();
  EXPECT_EQ(reverted["execution_backend"].toString(), "fake");
  EXPECT_TRUE(reverted["require_fake_hardware"].toBool(false));
}
