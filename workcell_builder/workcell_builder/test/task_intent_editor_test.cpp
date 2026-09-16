#include <gtest/gtest.h>
#include <QApplication>
#include <QComboBox>
#include <QDir>
#include <QFile>
#include <QJsonDocument>
#include <QLineEdit>
#include <QLabel>
#include <QTemporaryDir>
#include "gui/task_intent_editor.h"

namespace {
void application() {
  if (QCoreApplication::instance()) return;
  qputenv("QT_QPA_PLATFORM", "offscreen");
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
    EXPECT_EQ(editor.model().pick_selection.source_ref, "source_bin_01");
    EXPECT_EQ(editor.model().place.asset_ref, "place_fixture_01");
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
  // A fresh scaffold has no authored R1.9 containment yet: never claim planning readiness.
  EXPECT_FALSE(reopened.blocker().isEmpty());
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
