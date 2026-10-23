#include <gtest/gtest.h>

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QTemporaryDir>

#include <csignal>
#include <signal.h>
#include <sys/types.h>

#include "operator_hmi_model.hpp"
#include "operator_preview_launcher.hpp"
#include "operator_preview_request.hpp"

using namespace workcell_builder;

namespace
{

// The application is owned by a gtest environment and destroyed inside main(),
// never by an exit handler.
class CoreApplicationEnvironment : public ::testing::Environment
{
public:
  void SetUp() override
  {
    static int argc = 1;
    static char name[] = "operator_hmi_test";
    static char * argv[] = {name, nullptr};
    app_ = new QCoreApplication(argc, argv);
  }
  void TearDown() override { delete app_; app_ = nullptr; }

private:
  QCoreApplication * app_{nullptr};
};
[[maybe_unused]] ::testing::Environment * const core_application_environment =
  ::testing::AddGlobalTestEnvironment(new CoreApplicationEnvironment);

// Shape produced by scripts/operator_preview_projection.py for ur5_2f_test.
QJsonObject projection(bool draft, bool with_replay, const QString & validation = "PASS")
{
  QJsonObject task{
    {"id", "default_task"}, {"type", "pick_place"},
    {"target", QJsonObject{{"class_id", draft ? "cup" : "bottle"}, {"max_age_seconds", 2.0},
                           {"min_confidence", QJsonValue()}, {"pick_zone", "pick_zone_main"},
                           {"source_type", "perception"}, {"source_ref", "detected_objects/v1"}}},
    {"not_targeted", QJsonObject{{"rule", "Objects that do not match the pick filter are not picked."}}},
    {"destinations", QJsonArray{QJsonObject{{"id", "default_drop_zone"}, {"target_ref", "target_bin_default"}}}},
    {"routing_rules", QJsonArray{QJsonObject{{"id", "default_place"},
                                             {"when", QJsonObject{{"always", true}}},
                                             {"destination", "default_drop_zone"}}}},
    {"grasp", QJsonObject{{"policy", "EXACT"}, {"strategy_ref", "top_2f"}}}};
  QJsonObject out{
    {"schema", "workcell_operator_preview/v1"},
    {"scene", QJsonObject{{"name", "ur5_2f_test"}, {"path", "/ws/scenes/ur5_2f_test"}}},
    {"revision", QJsonObject{{"kind", draft ? "draft" : "saved"}, {"normalized_intent_sha256", "aa"},
                             {"saved_normalized_intent_sha256", "bb"},
                             {"draft_differs_from_saved", draft ? QJsonValue(true) : QJsonValue()}}},
    {"cell", QJsonObject{{"robot", QJsonObject{{"model", "ur5"}}},
                         {"end_effector", QJsonObject{{"model", "robotiq_85_gripper"},
                                                      {"capabilities", QJsonArray{"two_finger_parallel"}}}}}},
    {"task", task},
    {"validation", QJsonObject{{"status", validation}, {"errors", QJsonArray{}},
                               {"readiness_classification", "task_intent_ready_offline"}}},
    {"resolution", QJsonObject{{"status", "BLOCKED"}, {"primary_code", "CYCLE_EVALUATION_REQUIRED"}}},
    {"input", QJsonObject{{"kind", "replay"}, {"path", "/ws/config/runtime/r1_4_replay.yaml"}}},
    {"approval", QJsonObject{{"status", "NOT_APPROVED"}}},
    {"zones", QJsonObject{{"pick", QJsonObject{{"id", "pick_zone_main"}, {"pose_xyz", QJsonArray{0.4, -0.2, 0.3}},
                                               {"dimensions", QJsonArray{0.35, 0.3, 0.6}}}}}},
    {"execution", QJsonObject{{"backend", "UNAVAILABLE"}, {"motion_authorised", false},
                              {"reason", "No qualified execution backend."}}},
    {"generated_bundle", QJsonObject{{"dry_run_available", false}, {"reason", "Generate first."}}},
    {"events", QJsonArray{QJsonObject{{"level", "info"}, {"code", "TASK_SELECTED"}, {"message", "Task"}}}},
  };
  if (with_replay) {
    out.insert("observations", QJsonObject{
        {"selected_object", "sample-bottle"},
        {"objects", QJsonArray{
          QJsonObject{{"object_id", "sample-cup"}, {"class_id", "cup"}, {"confidence", 0.99}, {"accepted", false},
                      {"reason_code", "CLASS_NOT_TARGETED"}, {"pose_xyz", QJsonArray{0.26, -0.08, 0.05}},
                      {"dimensions", QJsonArray{0.06, 0.06, 0.1}}},
          QJsonObject{{"object_id", "sample-bottle"}, {"class_id", "bottle"}, {"confidence", 0.99},
                      {"accepted", true}, {"reason_code", "SELECTED"}}}}});
  }
  return out;
}

QString write_fake_hmi(const QTemporaryDir & dir)
{
  const QString path = dir.filePath("fake_hmi.sh");
  QFile file(path);
  file.open(QIODevice::WriteOnly);
  file.write("#!/bin/sh\nexec sleep 30\n");
  file.close();
  file.setPermissions(QFile::ReadOwner | QFile::WriteOwner | QFile::ExeOwner);
  return path;
}

}  // namespace

TEST(OperatorPreviewRequest, RoundTripsAndRejectsDirtyTaskWithoutDraft)
{
  QTemporaryDir dir;
  OperatorPreviewRequest request;
  request.revision = 4;
  request.scene_dir = "/ws/scenes/ur5_2f_test";
  request.scene_name = "ur5_2f_test";
  request.task_dirty = true;
  request.task_draft = QJsonObject{{"schema", "workcell_builder_task_intent/v2"}};
  request.layout_dirty = true;
  const QString path = dir.filePath("request.json");
  QString error;
  ASSERT_TRUE(write_operator_preview_request(path, request, &error)) << error.toStdString();
  const auto read = read_operator_preview_request(path, &error);
  ASSERT_TRUE(read.has_value()) << error.toStdString();
  EXPECT_EQ(read->revision, 4);
  EXPECT_TRUE(read->task_dirty);
  EXPECT_TRUE(read->layout_dirty);
  EXPECT_EQ(read->task_draft.value("schema").toString(), "workcell_builder_task_intent/v2");

  auto json = request.to_json();
  json.remove("task_draft");
  EXPECT_FALSE(OperatorPreviewRequest::from_json(json, &error).has_value());
  EXPECT_TRUE(error.contains("no draft"));
}

TEST(OperatorHmiModel, ParsesSavedProjectionWithReplayDecisions)
{
  const auto view = parse_operator_preview(projection(false, true), false);
  ASSERT_TRUE(view.usable);
  EXPECT_FALSE(view.draft);
  EXPECT_EQ(view.target_class, "bottle");
  EXPECT_EQ(view.destinations, QStringList{"default_drop_zone → target_bin_default"});
  EXPECT_FALSE(view.min_confidence.has_value());
  ASSERT_EQ(view.objects.size(), 2);
  EXPECT_FALSE(view.objects.at(0).accepted);
  EXPECT_EQ(view.objects.at(0).reason_code, "CLASS_NOT_TARGETED");
  EXPECT_TRUE(view.objects.at(0).has_geometry);
  EXPECT_FALSE(view.objects.at(1).has_geometry);  // missing geometry is not invented
  EXPECT_TRUE(view.pick_zone_geometry.valid);
  EXPECT_FALSE(view.place_zone_geometry.valid);
  EXPECT_EQ(operator_mode(view, true), OperatorMode::Replay);
  EXPECT_EQ(operator_mode(view, false), OperatorMode::Offline);
}

TEST(OperatorHmiModel, DraftOutranksReplayAndIsNeverAuthorised)
{
  const auto view = parse_operator_preview(projection(true, true), true);
  EXPECT_TRUE(view.draft);
  EXPECT_EQ(view.target_class, "cup");
  EXPECT_EQ(operator_mode(view, true), OperatorMode::Draft);
  const auto readiness = operator_readiness(view);
  EXPECT_TRUE(readiness.offline_checks_allowed);
  EXPECT_TRUE(readiness.production_blockers.join('\n').contains("Unsaved draft"));
  EXPECT_TRUE(readiness.production_blockers.join('\n').contains("Unsaved layout edits"));
  EXPECT_TRUE(readiness.production_blockers.join('\n').contains("No qualified execution backend"));
}

TEST(OperatorHmiModel, InvalidTaskBlocksOfflineChecksAndMissingProjectionIsUnusable)
{
  const auto invalid = parse_operator_preview(projection(false, false, "FAIL"), false);
  EXPECT_FALSE(operator_readiness(invalid).offline_checks_allowed);

  const auto missing = parse_operator_preview(QJsonObject{}, false);
  EXPECT_FALSE(missing.usable);
  EXPECT_FALSE(missing.errors.isEmpty());
  EXPECT_FALSE(operator_readiness(missing).offline_checks_allowed);
  EXPECT_FALSE(operator_readiness(missing).production_blockers.isEmpty());
}

TEST(OperatorHmiModel, ParsesGatedDryRunBlockers)
{
  const QJsonObject report{
    {"operator_summary", QJsonObject{{"status", "FAIL"}, {"safe_for_robot_motion", false},
                                     {"next_recommended_action", "Resolve preflight blockers."}}},
    {"preflight", QJsonObject{{"status", "FAIL"},
                              {"blockers", QJsonArray{"install/setup.bash missing under workspace root"}}}},
    {"cycle", QJsonObject{{"status", "SKIPPED"}, {"note", "preflight failed"}}}};
  const auto result = parse_gated_dry_run(report);
  EXPECT_TRUE(result.parsed);
  EXPECT_EQ(result.status, "FAIL");
  EXPECT_FALSE(result.safe_for_robot_motion);
  EXPECT_EQ(result.blockers, QStringList{"install/setup.bash missing under workspace root"});
  EXPECT_EQ(result.cycle_status, "SKIPPED");
  EXPECT_FALSE(parse_gated_dry_run(QJsonObject{}).parsed);
}

TEST(OperatorPreviewLauncher, OwnsOneProcessRefreshesAndStopsIt)
{
  QTemporaryDir dir;
  QDir(dir.path()).mkpath("scene");
  OperatorPreviewLauncher launcher;
  launcher.set_hmi_program(write_fake_hmi(dir));
  OperatorPreviewRequest request;
  request.scene_dir = dir.filePath("scene");
  request.scene_name = "scene";
  QString error;
  ASSERT_TRUE(launcher.open_or_refresh(request, &error)) << error.toStdString();
  ASSERT_TRUE(launcher.running());
  const qint64 pid = launcher.process_id();
  EXPECT_FALSE(launcher.request_path().startsWith(request.scene_dir));  // never inside the scene

  auto read = read_operator_preview_request(launcher.request_path(), &error);
  ASSERT_TRUE(read.has_value());
  EXPECT_EQ(read->revision, 1);
  EXPECT_TRUE(read->activate);

  launcher.publish_update(request);
  read = read_operator_preview_request(launcher.request_path(), &error);
  EXPECT_EQ(read->revision, 2);
  EXPECT_FALSE(read->activate);

  ASSERT_TRUE(launcher.open_or_refresh(request, &error));  // repeated launch refreshes
  EXPECT_EQ(launcher.process_id(), pid);
  read = read_operator_preview_request(launcher.request_path(), &error);
  EXPECT_EQ(read->revision, 3);
  EXPECT_TRUE(read->activate);

  launcher.shutdown(2000);
  EXPECT_FALSE(launcher.running());
  EXPECT_NE(::kill(static_cast<pid_t>(pid), 0), 0);  // no orphan left behind
}

TEST(OperatorPreviewLauncher, RefusesMissingSceneAndMissingProgram)
{
  QTemporaryDir dir;
  OperatorPreviewLauncher launcher;
  launcher.set_hmi_program(dir.filePath("does_not_exist"));
  OperatorPreviewRequest request;
  request.scene_dir = dir.filePath("missing_scene");
  QString error;
  EXPECT_FALSE(launcher.open_or_refresh(request, &error));
  EXPECT_TRUE(error.contains("Select a saved cell"));
  QDir(dir.path()).mkpath("scene");
  request.scene_dir = dir.filePath("scene");
  EXPECT_FALSE(launcher.open_or_refresh(request, &error));
  EXPECT_TRUE(error.contains("not found"));
  EXPECT_FALSE(launcher.running());
}
