#include <gtest/gtest.h>

#include "preview_process_state.hpp"

using workcell_builder::PreviewProcessOutcome;
using workcell_builder::classify_preview_process_failure;

TEST(PreviewProcessState, UserStopIsExpected)
{
  EXPECT_EQ(classify_preview_process_failure(true, true, false, true), PreviewProcessOutcome::ExpectedStop);
}

TEST(PreviewProcessState, UnexpectedRunningProcessDeathIsAnError)
{
  EXPECT_EQ(classify_preview_process_failure(false, false, false, false), PreviewProcessOutcome::UnexpectedExit);
}

TEST(PreviewProcessState, BuildAndLaunchFailuresRemainErrors)
{
  EXPECT_EQ(classify_preview_process_failure(false, false, true, false), PreviewProcessOutcome::BuildFailure);
  EXPECT_EQ(classify_preview_process_failure(false, false, false, true), PreviewProcessOutcome::LaunchFailure);
}
