#include <gtest/gtest.h>

#include <vector>

#include "run_grasp_execution/home_return_utils.hpp"

TEST(TestHomeReturnUtils, SafeIntermediateEnabledOnlyWithValues)
{
  EXPECT_FALSE(run_grasp_execution::safe_intermediate_enabled(false, {0.1, 0.2}));
  EXPECT_FALSE(run_grasp_execution::safe_intermediate_enabled(true, {}));
  EXPECT_TRUE(run_grasp_execution::safe_intermediate_enabled(true, {0.1, 0.2}));
}

TEST(TestHomeReturnUtils, FailureReasonIncludesAttemptAndStep)
{
  const auto reason = run_grasp_execution::default_home_return_failure_reason(
    "Move back to home", 2, 5);
  EXPECT_NE(reason.find("Move back to home"), std::string::npos);
  EXPECT_NE(reason.find("2/5"), std::string::npos);
  EXPECT_NE(reason.find("no valid plan returned by MoveIt"), std::string::npos);
}
