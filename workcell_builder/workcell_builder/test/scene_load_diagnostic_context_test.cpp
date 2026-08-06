#include <gtest/gtest.h>

#include <QJsonObject>

#include "scene_load_diagnostic_context.h"

TEST(SceneLoadDiagnosticContext, RepeatedPollingAndRefreshCallbacksEmitOnce)
{
  SceneLoadDiagnosticContext context;
  const SceneLoadDiagnosticContext::Identity load{"ur5_2f_test", "builder_revision:41", 7};
  const QJsonObject ready{{"state", "scene_ready"}, {"rendered", 9}};
  EXPECT_TRUE(context.observe(load, "terminal_readiness", ready, true).should_emit);
  EXPECT_FALSE(context.observe(load, "terminal_readiness", ready, true).should_emit);
  EXPECT_FALSE(context.observe(load, "terminal_readiness", QJsonObject{{"rendered", 9}, {"state", "scene_ready"}}, true).should_emit);
}

TEST(SceneLoadDiagnosticContext, RevisionAndMaterialFailureChangesEmit)
{
  SceneLoadDiagnosticContext context;
  const QJsonObject failure{{"state", "scene_failed"}, {"reason", "mesh missing"}};
  const SceneLoadDiagnosticContext::Identity rev1{"ur5_2f_test", "builder_revision:41", 7};
  EXPECT_TRUE(context.observe(rev1, "terminal_readiness", failure, true).should_emit);
  EXPECT_FALSE(context.observe(rev1, "terminal_readiness", failure, true).should_emit);
  EXPECT_TRUE(context.observe(rev1, "terminal_readiness",
    QJsonObject{{"state", "scene_failed"}, {"reason", "package unresolved"}}, true).should_emit);
  const SceneLoadDiagnosticContext::Identity rev2{"ur5_2f_test", "builder_revision:42", 8};
  EXPECT_TRUE(context.observe(rev2, "terminal_readiness", failure, true).should_emit);
}

TEST(SceneLoadDiagnosticContext, DebugModeEmitsIdenticalObservations)
{
  SceneLoadDiagnosticContext context;
  context.set_debug_enabled(true);
  const SceneLoadDiagnosticContext::Identity load{"ur5_2f_test", "scene.json", 3};
  EXPECT_TRUE(context.observe(load, "asset_discovery", QJsonObject{{"assets", 4}}).should_emit);
  EXPECT_TRUE(context.observe(load, "asset_discovery", QJsonObject{{"assets", 4}}).should_emit);
}
