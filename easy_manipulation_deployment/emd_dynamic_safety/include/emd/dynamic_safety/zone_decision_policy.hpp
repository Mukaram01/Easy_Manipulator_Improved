#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

#include "emd/dynamic_safety/replanner.hpp"
#include "emd/dynamic_safety/safety_zone.hpp"

namespace dynamic_safety
{

struct ZonePolicyParameters
{
  double min_replan_interval{0.25};
  double emergency_hysteresis{0.05};
  double slowdown_hysteresis{0.08};
  double replan_hysteresis{0.10};
  double safe_hysteresis{0.0};
  double scale_floor{0.0001};
  double scale_ceiling{1.0};
  double scale_ramp_down_rate{1.5};
  double scale_ramp_up_rate{0.75};
  double collision_persistence_window{0.10};
  double replan_start_time_epsilon{0.02};
};

struct ZoneDecisionInput
{
  double now{0.0};
  double control_period{0.01};
  double current_time{0.0};
  double full_duration{0.0};
  double collision_time_point{-1.0};
  uint8_t raw_zone{SafetyZone::SAFE};
  bool collision_detected{false};
  bool allow_replan{false};
  double current_scale{1.0};
  double emergency_zone_limit{0.0};
  double slowdown_zone_limit{0.0};
  ReplannerStatus replanner_status{ReplannerStatus::IDLE};
};

struct ZoneDecision
{
  uint8_t zone{SafetyZone::SAFE};
  double next_scale{1.0};
  bool start_replanner{false};
  bool terminate_replanner{false};
  bool consume_replan_result{false};
  double start_state_time{0.0};
  std::string action{"none"};
};

class ZoneDecisionPolicy
{
public:
  explicit ZoneDecisionPolicy(const ZonePolicyParameters & parameters)
  : parameters_(parameters) {}

  void reset();
  ZoneDecision decide(const ZoneDecisionInput & input);

private:
  double get_hysteresis(uint8_t zone) const;

  ZonePolicyParameters parameters_;
  bool initialized_{false};
  double last_decision_time_{0.0};
  double last_collision_observed_time_{-std::numeric_limits<double>::infinity()};
  double last_replan_start_time_{-std::numeric_limits<double>::infinity()};
  double last_replan_wall_time_{-std::numeric_limits<double>::infinity()};
  uint8_t prior_zone_{SafetyZone::SAFE};
};

inline void ZoneDecisionPolicy::reset()
{
  initialized_ = false;
  prior_zone_ = SafetyZone::SAFE;
  last_decision_time_ = 0.0;
  last_collision_observed_time_ = -std::numeric_limits<double>::infinity();
  last_replan_start_time_ = -std::numeric_limits<double>::infinity();
  last_replan_wall_time_ = -std::numeric_limits<double>::infinity();
}

inline double ZoneDecisionPolicy::get_hysteresis(uint8_t zone) const
{
  if (zone <= SafetyZone::EMERGENCY) {
    return parameters_.emergency_hysteresis;
  }
  if (zone == SafetyZone::SLOWDOWN) {
    return parameters_.slowdown_hysteresis;
  }
  if (zone == SafetyZone::REPLAN) {
    return parameters_.replan_hysteresis;
  }
  return parameters_.safe_hysteresis;
}

inline ZoneDecision ZoneDecisionPolicy::decide(const ZoneDecisionInput & input)
{
  ZoneDecision out;
  const double dt = initialized_ ? std::max(input.now - last_decision_time_, 1e-6) :
    std::max(input.control_period, 1e-6);
  initialized_ = true;
  last_decision_time_ = input.now;

  if (input.collision_detected) {
    last_collision_observed_time_ = input.now;
  }
  const bool collision_persistent =
    (input.now - last_collision_observed_time_) <= parameters_.collision_persistence_window;

  out.zone = input.raw_zone;
  const double ttc = input.collision_time_point - input.current_time;
  if (collision_persistent && input.raw_zone > prior_zone_) {
    const double hysteresis_limit = get_hysteresis(prior_zone_);
    if (prior_zone_ == SafetyZone::EMERGENCY && ttc <= input.emergency_zone_limit + hysteresis_limit) {
      out.zone = SafetyZone::EMERGENCY;
    } else if (prior_zone_ == SafetyZone::SLOWDOWN &&
      ttc <= input.slowdown_zone_limit + hysteresis_limit)
    {
      out.zone = SafetyZone::SLOWDOWN;
    } else if (prior_zone_ == SafetyZone::REPLAN &&
      ttc <= input.slowdown_zone_limit + hysteresis_limit)
    {
      out.zone = SafetyZone::REPLAN;
    }
  }

  out.next_scale = std::clamp(input.current_scale, parameters_.scale_floor, parameters_.scale_ceiling);

  if (collision_persistent) {
    if (out.zone <= SafetyZone::EMERGENCY) {
      out.next_scale = parameters_.scale_floor;
      out.action = "emergency_stop";
    } else if (out.zone == SafetyZone::SLOWDOWN) {
      out.next_scale = std::max(parameters_.scale_floor, input.current_scale - parameters_.scale_ramp_down_rate * dt);
      out.action = "slowdown";
    }
  } else {
    out.next_scale = std::min(parameters_.scale_ceiling, input.current_scale + parameters_.scale_ramp_up_rate * dt);
    out.action = "recover";
  }

  if (input.allow_replan) {
    if (collision_persistent && (out.zone == SafetyZone::SLOWDOWN || out.zone == SafetyZone::REPLAN)) {
      double start_state_time = input.current_time;
      if (out.zone == SafetyZone::SLOWDOWN) {
        start_state_time = (input.current_time + input.emergency_zone_limit + input.collision_time_point) / 2.0;
      } else {
        start_state_time = (input.current_time + input.slowdown_zone_limit + input.collision_time_point) / 2.0;
      }
      start_state_time = std::clamp(start_state_time, input.current_time, input.full_duration);
      const bool min_interval_ok =
        (input.now - last_replan_wall_time_) >= parameters_.min_replan_interval;
      const bool not_duplicate =
        std::fabs(start_state_time - last_replan_start_time_) > parameters_.replan_start_time_epsilon;
      if (min_interval_ok && not_duplicate &&
        (input.replanner_status == ReplannerStatus::IDLE ||
        input.replanner_status == ReplannerStatus::TIMEOUT))
      {
        out.start_replanner = true;
        out.start_state_time = start_state_time;
        out.action = "start_replanner";
        last_replan_wall_time_ = input.now;
        last_replan_start_time_ = start_state_time;
      }
    } else if (!collision_persistent || out.zone == SafetyZone::SAFE) {
      if (input.replanner_status == ReplannerStatus::ONGOING ||
        input.replanner_status == ReplannerStatus::TIMEOUT)
      {
        out.terminate_replanner = true;
        out.action = "terminate_replanner";
      } else if (input.replanner_status == ReplannerStatus::SUCCEED) {
        out.consume_replan_result = true;
        out.action = "consume_replanner_result";
      }
    }
  }

  prior_zone_ = out.zone;
  return out;
}

}  // namespace dynamic_safety
