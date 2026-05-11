#include "camera_perception_profile.hpp"
#include <algorithm>
#include <cmath>

namespace workcell_builder
{
std::vector<CameraProfile> load_camera_profiles(const std::string &)
{
  return {default_realsense_d435i_profile()};
}

CameraProfile default_realsense_d435i_profile() { return CameraProfile{}; }

void normalize_camera_topics(CameraProfile & profile)
{
  auto norm = [](std::string & s) {
    if (!s.empty() && s.front() != '/') s = "/" + s;
  };
  norm(profile.rgb_topic); norm(profile.depth_topic); norm(profile.camera_info_topic); norm(profile.pointcloud_topic);
}

CameraValidationResult validate_camera_placement(const CameraProfile & profile, bool strict)
{
  CameraValidationResult out;
  if (profile.camera_id.empty()) out.blockers.emplace_back("camera_id must be non-empty");
  if (profile.frame_id.empty()) out.blockers.emplace_back("frame_id must be non-empty");
  if (profile.pose.size() != 6) out.blockers.emplace_back("pose must contain six finite numbers");
  else for (double v : profile.pose) if (!std::isfinite(v)) { out.blockers.emplace_back("pose must contain six finite numbers"); break; }
  if (profile.pointcloud_topic.empty()) out.warnings.emplace_back("missing pointcloud topic");
  if (profile.rgb_topic.empty() || profile.depth_topic.empty()) {
    (strict ? out.blockers : out.warnings).emplace_back("missing rgb/depth topics");
  }
  out.ok = out.blockers.empty();
  return out;
}

std::string camera_status_label(const CameraValidationResult & result)
{
  return result.blockers.empty() ? (result.warnings.empty() ? "Camera PASS" : "Camera WARN") : "Camera FAIL";
}
}  // namespace workcell_builder
