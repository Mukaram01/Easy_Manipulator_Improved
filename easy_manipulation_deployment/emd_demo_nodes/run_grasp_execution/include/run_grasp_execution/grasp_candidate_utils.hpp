#ifndef RUN_GRASP_EXECUTION__GRASP_CANDIDATE_UTILS_HPP_
#define RUN_GRASP_EXECUTION__GRASP_CANDIDATE_UTILS_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace run_grasp_execution
{

inline std::string pose_dedup_key(const geometry_msgs::msg::PoseStamped & pose)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(5)
      << pose.pose.position.x << "," << pose.pose.position.y << "," << pose.pose.position.z << ","
      << pose.pose.orientation.x << "," << pose.pose.orientation.y << ","
      << pose.pose.orientation.z << "," << pose.pose.orientation.w;
  return oss.str();
}

inline std::vector<geometry_msgs::msg::PoseStamped> expand_grasp_candidates_with_fallbacks(
  const std::vector<geometry_msgs::msg::PoseStamped> & input_candidates,
  std::size_t min_candidate_count)
{
  std::vector<geometry_msgs::msg::PoseStamped> candidates = input_candidates;
  if (candidates.empty() || candidates.size() >= min_candidate_count) {
    return candidates;
  }

  std::set<std::string> seen_keys;
  for (const auto & candidate : candidates) {
    seen_keys.insert(pose_dedup_key(candidate));
  }

  const std::vector<double> yaw_offsets_deg = {15.0, -15.0, 30.0, -30.0, 45.0, -45.0, 60.0, -60.0};
  const std::vector<double> pitch_offsets_deg = {0.0, 8.0, -8.0, 12.0, -12.0};
  const std::vector<double> z_offsets_m = {0.0, 0.01, 0.02};
  constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

  for (const auto & base : input_candidates) {
    tf2::Quaternion base_q;
    tf2::fromMsg(base.pose.orientation, base_q);
    if (base_q.length2() <= 0.0) {
      continue;
    }
    base_q.normalize();
    for (const double z_offset : z_offsets_m) {
      for (const double pitch_deg : pitch_offsets_deg) {
        for (const double yaw_deg : yaw_offsets_deg) {
          if (candidates.size() >= min_candidate_count) {
            return candidates;
          }

          tf2::Quaternion yaw_q;
          yaw_q.setRPY(0.0, 0.0, yaw_deg * kDegToRad);
          tf2::Quaternion pitch_q;
          pitch_q.setRPY(0.0, pitch_deg * kDegToRad, 0.0);
          tf2::Quaternion candidate_q = base_q * yaw_q * pitch_q;
          candidate_q.normalize();

          geometry_msgs::msg::PoseStamped variant = base;
          variant.pose.position.z += z_offset;
          variant.pose.orientation = tf2::toMsg(candidate_q);
          const std::string key = pose_dedup_key(variant);
          if (seen_keys.insert(key).second) {
            candidates.push_back(std::move(variant));
          }
        }
      }
    }
  }

  return candidates;
}

inline std::string categorize_candidate_rejection_reason(const std::string & reason)
{
  if (reason.find("IK failed") != std::string::npos ||
    reason.find("invalid candidate quaternion") != std::string::npos ||
    reason.find("planning group not found") != std::string::npos ||
    reason.find("could not retrieve current robot state") != std::string::npos)
  {
    return "IK failure";
  }

  if (reason.find("IK solution in collision:") != std::string::npos) {
    const bool has_table = reason.find("table_") != std::string::npos;
    const bool has_octomap = reason.find("<octomap>") != std::string::npos;
    const bool has_finger_tip = reason.find("finger_tip_link") != std::string::npos;
    if (has_table) {
      return "broad robot/table collision";
    }
    if (has_octomap && !has_finger_tip) {
      return "broad robot/octomap collision";
    }
    return "self-collision";
  }

  if (reason.find("outside workspace bounds") != std::string::npos) {
    return "planner failure";
  }

  return "planner failure";
}

}  // namespace run_grasp_execution

#endif  // RUN_GRASP_EXECUTION__GRASP_CANDIDATE_UTILS_HPP_
