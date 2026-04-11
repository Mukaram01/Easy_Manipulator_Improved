#ifndef EMD__DYNAMIC_SAFETY__GOAL_TIME_TOLERANCE_UTILS_HPP_
#define EMD__DYNAMIC_SAFETY__GOAL_TIME_TOLERANCE_UTILS_HPP_

#include <algorithm>

namespace dynamic_safety
{
namespace goal_time_tolerance
{
inline bool should_enforce_strict_tolerance(
  const double current_scale,
  const double minimum_scale_for_strict_checks)
{
  return current_scale >= minimum_scale_for_strict_checks;
}

inline double inflated_tolerance_from_average_scale(
  const double base_tolerance,
  const double average_scale,
  const double minimum_average_scale)
{
  const double clamped_average_scale = std::max(average_scale, minimum_average_scale);
  return base_tolerance / clamped_average_scale;
}
}  // namespace goal_time_tolerance
}  // namespace dynamic_safety

#endif  // EMD__DYNAMIC_SAFETY__GOAL_TIME_TOLERANCE_UTILS_HPP_
