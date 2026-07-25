#include <gtest/gtest.h>

#include "studio_log_issue_tracker.hpp"

using workcell_builder::StudioLogIssueTracker;
using workcell_builder::StudioLogSeverity;

TEST(StudioLogIssueTracker, DiagnosticTextIsInfoByDefault)
{
  StudioLogIssueTracker tracker;
  tracker.set_scope("scene_a");
  for (const char * diagnostic : {"failed_required_item_count=0", "parse_error=0",
      "missing_chain_warnings=0", "Metadata warnings: 0"}) {
    tracker.report(StudioLogSeverity::Info, diagnostic);
  }
  EXPECT_EQ(tracker.error_count(), 0);
  EXPECT_EQ(tracker.warning_count(), 0);
}

TEST(StudioLogIssueTracker, KeyedIssuesIncrementOnlyOnce)
{
  StudioLogIssueTracker tracker;
  tracker.set_scope("scene_a");
  EXPECT_TRUE(tracker.report(StudioLogSeverity::Warning, "missing_camera_metadata"));
  EXPECT_FALSE(tracker.report(StudioLogSeverity::Warning, "missing_camera_metadata"));
  EXPECT_EQ(tracker.warning_count(), 1);
  EXPECT_TRUE(tracker.report(StudioLogSeverity::Error, "scene_preparation_failed"));
  EXPECT_FALSE(tracker.report(StudioLogSeverity::Error, "scene_preparation_failed"));
  EXPECT_EQ(tracker.error_count(), 1);
}

TEST(StudioLogIssueTracker, SceneScopeClearsVisibleCounts)
{
  StudioLogIssueTracker tracker;
  tracker.set_scope("scene_a");
  tracker.report(StudioLogSeverity::Warning, "missing_camera_metadata");
  tracker.report(StudioLogSeverity::Error, "scene_preparation_failed");
  tracker.set_scope("scene_b");
  EXPECT_EQ(tracker.warning_count(), 0);
  EXPECT_EQ(tracker.error_count(), 0);
}
