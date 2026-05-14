#pragma once
#include <string>
#include <vector>
#include <boost/filesystem.hpp>

namespace workcell_builder {
struct LayoutMergeResult {
  bool status{false};
  bool layout_applied{false};
  bool generated_from_saved_layout{false};
  std::vector<std::string> warnings;
  std::vector<std::string> blockers;
  std::vector<std::string> generated_artifacts;
  std::string report_path;
  std::string summary_path;
  std::string stdout_log;
  std::string stderr_log;
};

LayoutMergeResult merge_workcell_studio_layout(const boost::filesystem::path& scene_dir);
}
