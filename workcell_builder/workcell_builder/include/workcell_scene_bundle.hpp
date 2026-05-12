#pragma once

#include <boost/filesystem.hpp>
#include <string>
#include <vector>

namespace workcell_builder
{
struct SceneBundleExportOptions
{
  boost::filesystem::path workcell_path;
  boost::filesystem::path scenes_path;
  boost::filesystem::path assets_path;
  std::string scene_name;
  boost::filesystem::path output_dir;
  bool include_generated_environment_assets{true};
  bool overwrite{false};
};

struct SceneBundleImportOptions
{
  boost::filesystem::path workcell_path;
  boost::filesystem::path scenes_path;
  boost::filesystem::path assets_path;
  boost::filesystem::path bundle_dir;
  bool overwrite{false};
  bool rename_on_conflict{true};
};

struct SceneBundleResult
{
  bool ok{false};
  std::string message;
  std::string scene_name;
  boost::filesystem::path output_path;
  std::vector<std::string> warnings;
  std::vector<std::string> dependency_notes;
};

SceneBundleResult export_scene_bundle(const SceneBundleExportOptions & options);
SceneBundleResult import_scene_bundle(const SceneBundleImportOptions & options);
}  // namespace workcell_builder
