#include "scene_template_library.hpp"

#include <fstream>
#include <sstream>

namespace workcell_builder
{
std::string load_scene_templates(const std::string & catalog_path)
{
  std::ifstream input(catalog_path);
  std::stringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
}

std::vector<std::string> list_scene_templates(const std::string & catalog_text)
{
  std::vector<std::string> ids;
  std::size_t pos = 0;
  const std::string needle = "\"template_id\":\"";
  while ((pos = catalog_text.find(needle, pos)) != std::string::npos)
  {
    const std::size_t start = pos + needle.size();
    const std::size_t end = catalog_text.find("\"", start);
    if (end == std::string::npos) break;
    ids.emplace_back(catalog_text.substr(start, end - start));
    pos = end + 1;
  }
  return ids;
}

std::string instantiate_scene_template(const std::string &, const std::string &, const std::string &)
{
  return "Template instantiation is performed by scripts/generate_workcell_scene_from_template.py";
}

bool validate_scene_template(const std::string & template_text, std::string & message)
{
  const bool ok = template_text.find("workcell_scene/v1") != std::string::npos || template_text.find("template_id") != std::string::npos;
  message = ok ? "Template Validation Status: PASS" : "Template Validation Status: FAIL";
  return ok;
}

std::string scene_template_status_label(bool valid) { return valid ? "Template Validation Status: PASS" : "Template Validation Status: FAIL"; }
}  // namespace workcell_builder
