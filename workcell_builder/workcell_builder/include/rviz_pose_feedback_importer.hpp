#pragma once

#include <string>
#include <vector>

namespace workcell_builder
{

struct RvizPoseFeedbackObjectEntry
{
  std::string name;
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};
  std::string status;
  std::string original_mesh;
  std::vector<std::string> warnings;
  std::vector<std::string> errors;
  bool valid{false};
};

struct RvizPoseFeedbackImportSummary
{
  std::string scene_name;
  std::string source;
  bool safe_for_robot_motion{true};
  std::vector<std::string> errors;
  std::vector<std::string> warnings;
  std::vector<RvizPoseFeedbackObjectEntry> entries;

  bool has_fatal_error() const;
};

RvizPoseFeedbackImportSummary parse_rviz_pose_feedback_file(
  const std::string & scene_name,
  const std::string & feedback_yaml_path) noexcept;

}  // namespace workcell_builder
