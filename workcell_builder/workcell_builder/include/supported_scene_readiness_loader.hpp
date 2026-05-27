#pragma once
#include <boost/filesystem.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace workcell_builder {

struct SupportedSceneRegistryEntry {
  std::string name;
  std::string support_level{"supported"};
};

struct SceneReadinessSummary {
  std::string scene_name;
  std::string readiness_status{"UNKNOWN"};
  std::string blockers_summary;
  std::string required_files_status{"UNKNOWN"};
  std::string static_validation_status{"UNKNOWN"};
  std::string guided_build_launch_readiness{"UNKNOWN"};
  std::string fake_hardware_launch_readiness{"UNKNOWN"};
  std::vector<std::string> blockers;
  std::vector<std::string> warnings;
};

struct AllScenesReadinessData {
  std::string report_path;
  std::unordered_map<std::string, SceneReadinessSummary> by_scene;
};

std::vector<SupportedSceneRegistryEntry> load_supported_scene_registry(const boost::filesystem::path & repo_root, std::string * error);
AllScenesReadinessData load_latest_all_scenes_readiness_report(const boost::filesystem::path & repo_root, std::string * error);

} // namespace workcell_builder
