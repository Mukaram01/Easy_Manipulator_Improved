#include <gtest/gtest.h>

#include "readiness_pack_command.hpp"

using workcell_builder::classify_readiness_pack_outcome;
using workcell_builder::readiness_pack_command;

TEST(ReadinessPackCommand, UsesTheScriptsNamedArgumentsNotAPositionalPath)
{
  const auto command = readiness_pack_command(
    QStringLiteral("/repo/scripts/generate_workcell_studio_readiness_pack.py"),
    QStringLiteral("/repo/scenes/ur5_2f_test"), QStringLiteral("ur5_2f_test"));
  EXPECT_EQ(command.program, QStringLiteral("python3"));
  ASSERT_GE(command.arguments.size(), 8);
  EXPECT_EQ(command.arguments.at(0), QStringLiteral("/repo/scripts/generate_workcell_studio_readiness_pack.py"));
  const int scene_at = command.arguments.indexOf(QStringLiteral("--scene-package"));
  const int output_at = command.arguments.indexOf(QStringLiteral("--output-dir"));
  const int project_at = command.arguments.indexOf(QStringLiteral("--project-name"));
  ASSERT_GT(scene_at, 0);
  ASSERT_GT(output_at, 0);
  ASSERT_GT(project_at, 0);
  EXPECT_EQ(command.arguments.at(scene_at + 1), QStringLiteral("/repo/scenes/ur5_2f_test"));
  EXPECT_EQ(command.arguments.at(output_at + 1), QStringLiteral("/repo/scenes/ur5_2f_test/readiness"));
  EXPECT_EQ(command.arguments.at(project_at + 1), QStringLiteral("ur5_2f_test"));
  EXPECT_TRUE(command.arguments.contains(QStringLiteral("--json")));
  // The old defect: the scene directory passed positionally right after the script.
  EXPECT_NE(command.arguments.at(1), QStringLiteral("/repo/scenes/ur5_2f_test"));
  EXPECT_EQ(command.manifest_path, QStringLiteral("/repo/scenes/ur5_2f_test/readiness/readiness_pack_manifest.json"));
  // Never destroy a previous pack implicitly.
  EXPECT_FALSE(command.arguments.contains(QStringLiteral("--force")));
}

TEST(ReadinessPackOutcome, ArgparseRejectionIsAnErrorWithTheCause)
{
  const auto outcome = classify_readiness_pack_outcome(
    true, false, true, 2, QString(),
    QStringLiteral("usage: generate_workcell_studio_readiness_pack.py [-h] --scene-package\n"
                   "generate_workcell_studio_readiness_pack.py: error: the following arguments are required: --scene-package, --output-dir, --project-name"),
    false, QStringLiteral("/s/readiness/readiness_pack_manifest.json"), 180);
  EXPECT_FALSE(outcome.success);
  EXPECT_EQ(outcome.kind, QStringLiteral("error"));
  EXPECT_TRUE(outcome.message.contains(QStringLiteral("exit code 2")));
  EXPECT_TRUE(outcome.message.contains(QStringLiteral("the following arguments are required")));
}

TEST(ReadinessPackOutcome, StartingIsNotSuccess)
{
  // Process started, exit 0, but nothing was written by this run.
  const auto outcome = classify_readiness_pack_outcome(
    true, false, true, 0, QStringLiteral("{\"result\": \"PASS\"}"), QString(),
    false, QStringLiteral("/s/readiness/readiness_pack_manifest.json"), 180);
  EXPECT_FALSE(outcome.success);
  EXPECT_EQ(outcome.kind, QStringLiteral("error"));
  EXPECT_TRUE(outcome.message.contains(QStringLiteral("wrote no manifest")));

  const auto not_started = classify_readiness_pack_outcome(
    false, false, true, -1, QString(), QString(), false, QString(), 180);
  EXPECT_FALSE(not_started.success);
  EXPECT_EQ(not_started.kind, QStringLiteral("warning"));
  EXPECT_TRUE(not_started.message.contains(QStringLiteral("did not start")));

  const auto timed_out = classify_readiness_pack_outcome(
    true, true, true, 0, QString(), QString(), true, QString(), 180);
  EXPECT_FALSE(timed_out.success);
  EXPECT_TRUE(timed_out.message.contains(QStringLiteral("timed out after 180 s")));
}

TEST(ReadinessPackOutcome, FinalReadinessFailIsReportedAsFailure)
{
  const auto outcome = classify_readiness_pack_outcome(
    true, false, true, 2, QStringLiteral("{\"result\": \"FAIL\", \"manifest\": \"/s/readiness/readiness_pack_manifest.json\"}"),
    QString(), true, QStringLiteral("/s/readiness/readiness_pack_manifest.json"), 180);
  EXPECT_FALSE(outcome.success);
  EXPECT_EQ(outcome.result, QStringLiteral("FAIL"));
  EXPECT_TRUE(outcome.message.contains(QStringLiteral("final readiness is FAIL")));
  EXPECT_TRUE(outcome.message.contains(QStringLiteral("/s/readiness/readiness_pack_manifest.json")));
  EXPECT_EQ(outcome.kind, QStringLiteral("error"));
}

TEST(ReadinessPackOutcome, WrittenManifestWithExitZeroIsSuccess)
{
  const auto pass = classify_readiness_pack_outcome(
    true, false, true, 0, QStringLiteral("{\"result\": \"PASS\"}"), QString(),
    true, QStringLiteral("/s/readiness/readiness_pack_manifest.json"), 180);
  EXPECT_TRUE(pass.success);
  EXPECT_EQ(pass.kind, QStringLiteral("success"));
  EXPECT_TRUE(pass.message.contains(QStringLiteral("PASS")));
  EXPECT_TRUE(pass.message.contains(QStringLiteral("/s/readiness/readiness_pack_manifest.json")));

  const auto warn = classify_readiness_pack_outcome(
    true, false, true, 0, QStringLiteral("{\"result\": \"WARN\"}"), QString(),
    true, QStringLiteral("/s/readiness/readiness_pack_manifest.json"), 180);
  EXPECT_TRUE(warn.success);
  EXPECT_EQ(warn.kind, QStringLiteral("warning"));
}
