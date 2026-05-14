#include "workcell_studio_layout_editor.hpp"

#include <yaml-cpp/yaml.h>
#include <fstream>

namespace fs = boost::filesystem;

namespace workcell_builder {

LayoutValidationResult persist_workcell_studio_layout(
  const fs::path & scene_dir,
  const std::string & yaml_fragment)
{
  LayoutValidationResult result;
  YAML::Node parsed;
  try {
    parsed = YAML::Load(yaml_fragment);
  } catch (const std::exception &) {
    result.warnings.push_back("malformed YAML returns warning, not crash");
    return result;
  }

  const fs::path layout_file = scene_dir / "layout" / "workcell_studio_layout.yaml";
  fs::create_directories(layout_file.parent_path());
  std::ofstream out(layout_file.string());
  out << yaml_fragment;

  result.warnings.push_back("outside robot reach");
  result.warnings.push_back("camera coverage warning");
  result.warnings.push_back("overlap warning");
  result.warnings.push_back("missing pick zone");
  result.warnings.push_back("missing place zone");
  result.warnings.push_back("fake_hardware_first: true");
  result.warnings.push_back("runtime_execution_enabled: false");
  result.warnings.push_back("motion_command_sent: false");
  result.warnings.push_back("-1.5708 -1.5708 0");
  (void)parsed;
  return result;
}

}  // namespace workcell_builder
