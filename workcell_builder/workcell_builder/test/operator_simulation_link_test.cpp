#include <gtest/gtest.h>

#include <QTemporaryDir>

#include "operator_preview_request.hpp"
#include "operator_simulation_link.hpp"

using namespace workcell_builder;

namespace
{
struct Fixture
{
  QTemporaryDir dir;
  OperatorLinkState state;
  Fixture()
  {
    QDir(dir.path()).mkpath("cell_a");
    QDir(dir.path()).mkpath("cell_b");
    state.control_token = "token-1";
    state.scene_dir = dir.path() + "/cell_a";
    state.intent_sha256 = "sha-saved";
    state.request_revision = 7;
  }
  QJsonObject run(const QString & token = "token-1", const QString & cell = "cell_a", const QString & sha = "sha-saved", int revision = 7)
  {
    return make_operator_command(1, "run_simulation", token, dir.path() + "/" + cell, sha, revision);
  }
};
}  // namespace

TEST(OperatorSimulationLink, OnlyTheLaunchedHmiOnTheSelectedSavedCellMayStart)
{
  Fixture f;
  EXPECT_TRUE(check_operator_command(f.run(), f.state).accepted);
  EXPECT_FALSE(check_operator_command(f.run("other"), f.state).accepted);              // not this Builder's HMI
  EXPECT_FALSE(check_operator_command(f.run(""), f.state).accepted);
  EXPECT_FALSE(check_operator_command(f.run("token-1", "cell_b"), f.state).accepted);  // another cell
  EXPECT_FALSE(check_operator_command(f.run("token-1", "cell_a", "sha-draft"), f.state).accepted);  // not the generated task
  EXPECT_FALSE(check_operator_command(f.run("token-1", "cell_a", "sha-saved", 6), f.state).accepted);  // stale view
  auto standalone = f.state;
  standalone.control_token.clear();                                                       // no trusted control session
  EXPECT_FALSE(check_operator_command(f.run(""), standalone).accepted);
  auto ungenerated = f.state;
  ungenerated.intent_sha256.clear();
  EXPECT_FALSE(check_operator_command(f.run(), ungenerated).accepted);
}

TEST(OperatorSimulationLink, OneSimulationAtATimeAndCancelOnlyTheRunningSession)
{
  Fixture f;
  f.state.session_active = true;
  f.state.session_id = "session-9";
  EXPECT_EQ(check_operator_command(f.run(), f.state).reason, "a session is already running; one simulation at a time");
  auto cancel = make_operator_command(2, "cancel_simulation", "token-1", f.state.scene_dir, "sha-saved", 7, "session-9");
  EXPECT_TRUE(check_operator_command(cancel, f.state).accepted);
  cancel["session_id"] = "session-8";                                                    // a previous session
  EXPECT_FALSE(check_operator_command(cancel, f.state).accepted);
  cancel["session_id"] = "session-9";
  f.state.cancel_requested = true;
  EXPECT_FALSE(check_operator_command(cancel, f.state).accepted);
  f.state.session_active = false; f.state.session_id.clear(); f.state.cancel_requested = false;   // nothing running now
  EXPECT_EQ(check_operator_command(cancel, f.state).reason, "no simulation is running");
  auto unknown = f.run();
  unknown["action"] = "start_production";
  EXPECT_FALSE(check_operator_command(unknown, f.state).accepted);                       // production is never a command
}

TEST(OperatorSimulationLink, CommandLinesAreRecognisedOnlyWithPrefixAndSchema)
{
  const auto command = make_operator_command(3, "run_simulation", "t", "/c", "s", 1);
  QJsonObject parsed;
  ASSERT_TRUE(parse_operator_command_line(operator_command_line(command), &parsed));
  EXPECT_EQ(parsed, command);
  EXPECT_FALSE(parse_operator_command_line("[GUI] log line\n", &parsed));
  EXPECT_FALSE(parse_operator_command_line("WORKCELL_OPERATOR_COMMAND {\"action\":\"run_simulation\"}\n", &parsed));
  EXPECT_EQ(operator_status_file("/tmp/x/operator_preview_request.json"), "/tmp/x/operator_simulation_status.json");
}

TEST(OperatorSimulationLink, TheRequestCarriesTheControlToken)
{
  OperatorPreviewRequest request;
  request.scene_dir = "/c";
  request.control_token = "token-1";
  QString error;
  const auto back = OperatorPreviewRequest::from_json(request.to_json(), &error);
  ASSERT_TRUE(back.has_value()) << error.toStdString();
  EXPECT_EQ(back->control_token, "token-1");
}
