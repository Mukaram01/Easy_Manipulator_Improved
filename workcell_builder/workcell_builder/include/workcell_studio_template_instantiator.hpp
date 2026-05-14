#pragma once

#include <boost/filesystem.hpp>
#include <string>
#include <vector>

namespace workcell_builder {

struct WorkcellStudioTemplateInstantiationRequest
{
  std::string template_id;
  std::string scene_name;
  boost::filesystem::path scene_root;
  std::string robot_id;
  std::string end_effector_id;
  std::string layout_preset_id;
  bool include_camera{true};
  bool include_conveyor{false};
  bool use_fake_hardware_default{true};
};

struct WorkcellStudioTemplateInstantiationResult
{
  bool success{false};
  std::string status{"BLOCKED"};
  boost::filesystem::path scene_dir;
  std::vector<std::string> created_files;
  std::vector<std::string> warnings;
  std::vector<std::string> blockers;
  std::vector<std::string> next_commands;
  bool no_robot_motion_commanded{true};
};

WorkcellStudioTemplateInstantiationResult instantiate_workcell_studio_template(const WorkcellStudioTemplateInstantiationRequest & request);

}  // namespace workcell_builder
