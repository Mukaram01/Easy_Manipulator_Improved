#include <gtest/gtest.h>
#include "planning_readiness.hpp"

TEST(PlanningReadiness, BasicReadinessNoExecution) {
  workcell_builder::PlanningReadinessReport r;
  r.robot = "ur5";
  r.pick_zone = "pick_zone_1";
  r.place_zone = "place_zone_1";
  r = workcell_builder::validate_planning_readiness(r);
  EXPECT_TRUE(r.can_attempt_plan);
  EXPECT_FALSE(r.can_execute);
}
