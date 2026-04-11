#include <gtest/gtest.h>

#include "emd/dynamic_safety/goal_time_tolerance_utils.hpp"

TEST(GoalTimeToleranceUtils, EnforceStrictToleranceAtNominalScale)
{
  EXPECT_TRUE(dynamic_safety::goal_time_tolerance::should_enforce_strict_tolerance(1.0, 0.05));
}

TEST(GoalTimeToleranceUtils, DisableStrictToleranceAtLowScale)
{
  EXPECT_FALSE(dynamic_safety::goal_time_tolerance::should_enforce_strict_tolerance(0.01, 0.05));
}

TEST(GoalTimeToleranceUtils, KeepToleranceAtScaleOne)
{
  constexpr double base_tolerance = 0.2;
  const double inflated = dynamic_safety::goal_time_tolerance::inflated_tolerance_from_average_scale(
    base_tolerance, 1.0, 1e-3);
  EXPECT_DOUBLE_EQ(inflated, base_tolerance);
}

TEST(GoalTimeToleranceUtils, InflateToleranceAtHalfScale)
{
  constexpr double base_tolerance = 0.2;
  const double inflated = dynamic_safety::goal_time_tolerance::inflated_tolerance_from_average_scale(
    base_tolerance, 0.5, 1e-3);
  EXPECT_DOUBLE_EQ(inflated, 0.4);
}

TEST(GoalTimeToleranceUtils, InflateToleranceWithVaryingScaleProfile)
{
  constexpr double base_tolerance = 0.2;
  constexpr double elapsed_wall = 8.0;
  constexpr double elapsed_effective = 6.0;  // average scale = 0.75
  const double average_scale = elapsed_effective / elapsed_wall;
  const double inflated = dynamic_safety::goal_time_tolerance::inflated_tolerance_from_average_scale(
    base_tolerance, average_scale, 1e-3);
  EXPECT_NEAR(inflated, 0.2666666667, 1e-9);
}
