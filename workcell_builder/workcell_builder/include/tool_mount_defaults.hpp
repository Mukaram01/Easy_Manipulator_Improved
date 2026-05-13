#ifndef WORKCELL_BUILDER__TOOL_MOUNT_DEFAULTS_HPP_
#define WORKCELL_BUILDER__TOOL_MOUNT_DEFAULTS_HPP_

#include <algorithm>
#include <array>
#include <cctype>
#include <string>

namespace workcell_builder
{
struct ToolMountProfile
{
  std::string profile_name;
  std::array<float, 3> xyz{{0.0F, 0.0F, 0.0F}};
  std::array<float, 3> rpy{{-1.5708F, -1.5708F, 0.0F}};
  bool apply_default{false};
  bool unknown_profile{false};
};

inline std::string lower_copy(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return value;
}

inline ToolMountProfile resolve_tool_mount_profile(const std::string & ee_name, const std::string & ee_type)
{
  const std::string name = lower_copy(ee_name);
  const std::string type = lower_copy(ee_type);
  ToolMountProfile profile;

  if (name.find("robotiq_85") != std::string::npos || name.find("robotiq_2f") != std::string::npos ||
    name.find("2f") != std::string::npos)
  {
    profile.profile_name = "robotiq_2f";
    profile.apply_default = true;
    return profile;
  }
  if (name.find("robotiq_3f") != std::string::npos || name.find("3f") != std::string::npos) {
    profile.profile_name = "robotiq_3f";
    profile.apply_default = true;
    return profile;
  }
  if (type == "suction" || name.find("suction") != std::string::npos || name.find("airpick") != std::string::npos || name.find("vacuum") != std::string::npos) {
    profile.profile_name = "suction_airpick_vacuum";
    profile.apply_default = true;
    return profile;
  }
  if (type == "finger") {
    profile.profile_name = "unknown_gripper";
    profile.apply_default = true;
    profile.unknown_profile = true;
    return profile;
  }

  profile.profile_name = "non_gripper";
  return profile;
}
}  // namespace workcell_builder

#endif
