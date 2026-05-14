#pragma once
#include <string>
#include <vector>
#include <boost/filesystem.hpp>

namespace workcell_builder {
struct LayoutMergeResult {
  bool layout_applied{false};
  std::vector<std::string> warnings;
  std::vector<std::string> blockers;
};

LayoutMergeResult merge_workcell_studio_layout(const boost::filesystem::path& scene_dir);
}
