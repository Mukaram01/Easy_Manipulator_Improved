#pragma once

#include <string>
#include <vector>

namespace workcell_builder
{
std::string load_scene_templates(const std::string & catalog_path);
std::vector<std::string> list_scene_templates(const std::string & catalog_text);
std::string instantiate_scene_template(const std::string & catalog_text, const std::string & template_id, const std::string & scene_name);
bool validate_scene_template(const std::string & template_text, std::string & message);
std::string scene_template_status_label(bool valid);
}  // namespace workcell_builder
