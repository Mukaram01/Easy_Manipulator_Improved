#include <gtest/gtest.h>

#include "emd/dynamic_safety/zone_decision_policy.hpp"

using dynamic_safety::ReplannerStatus;
using dynamic_safety::SafetyZone;
using dynamic_safety::ZoneDecisionInput;
using dynamic_safety::ZoneDecisionPolicy;
using dynamic_safety::ZonePolicyParameters;

TEST(ZoneDecisionPolicy, NoisyCollisionStreamDoesNotFlapReplanner)
{
  ZonePolicyParameters params;
  params.min_replan_interval = 0.25;
  params.collision_persistence_window = 0.2;
  params.replan_start_time_epsilon = 0.05;
  ZoneDecisionPolicy policy(params);

  int start_calls = 0;
  int terminate_calls = 0;
  ReplannerStatus status = ReplannerStatus::IDLE;

  double now = 0.0;
  for (int i = 0; i < 80; ++i) {
    now += 0.05;
    const bool noisy_collision = (i % 3 != 0);
    const uint8_t zone = noisy_collision ? SafetyZone::REPLAN : SafetyZone::SAFE;

    const auto decision = policy.decide(ZoneDecisionInput{
      now,
      0.05,
      2.0,
      6.0,
      noisy_collision ? 2.5 : -1.0,
      zone,
      noisy_collision,
      true,
      1.0,
      0.5,
      1.0,
      status
    });

    if (decision.start_replanner) {
      ++start_calls;
      status = ReplannerStatus::ONGOING;
    }
    if (decision.terminate_replanner) {
      ++terminate_calls;
      status = ReplannerStatus::IDLE;
    }
  }

  EXPECT_LE(start_calls, 2);
  EXPECT_LE(terminate_calls, 1);
}

TEST(ZoneDecisionPolicy, ReplanStartTimeIsClampedAndDeduplicated)
{
  ZonePolicyParameters params;
  params.min_replan_interval = 0.3;
  params.replan_start_time_epsilon = 0.1;
  ZoneDecisionPolicy policy(params);

  const auto d1 = policy.decide(ZoneDecisionInput{
    1.0,
    0.1,
    4.8,
    5.0,
    5.5,
    SafetyZone::REPLAN,
    true,
    true,
    1.0,
    0.4,
    1.0,
    ReplannerStatus::IDLE
  });
  EXPECT_TRUE(d1.start_replanner);
  EXPECT_LE(d1.start_state_time, 5.0);

  const auto d2 = policy.decide(ZoneDecisionInput{
    1.05,
    0.05,
    4.8,
    5.0,
    5.45,
    SafetyZone::REPLAN,
    true,
    true,
    1.0,
    0.4,
    1.0,
    ReplannerStatus::IDLE
  });
  EXPECT_FALSE(d2.start_replanner);
}
