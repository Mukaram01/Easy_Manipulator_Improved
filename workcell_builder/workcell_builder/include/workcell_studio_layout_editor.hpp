#pragma once

#include <boost/filesystem.hpp>
#include <string>
#include <vector>

namespace workcell_builder {

struct LayoutValidationResult {
  std::vector<std::string> warnings;
  bool severe_error{false};
};

LayoutValidationResult persist_workcell_studio_layout(
  const boost::filesystem::path & scene_dir,
  const std::string & yaml_fragment);

}  // namespace workcell_builder
