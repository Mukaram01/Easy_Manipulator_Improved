#include "supported_scene_readiness_loader.hpp"
#include <yaml-cpp/yaml.h>

namespace fs = boost::filesystem;
namespace workcell_builder {

static std::string scalar_or(const YAML::Node & n, const char * key, const std::string & d="UNKNOWN") {
  if (!n || !n[key] || !n[key].IsScalar()) return d;
  const auto v = n[key].as<std::string>();
  return v.empty()?d:v;
}

std::vector<SupportedSceneRegistryEntry> load_supported_scene_registry(const fs::path & repo_root, std::string * error) {
  std::vector<SupportedSceneRegistryEntry> out;
  const fs::path p = repo_root / "workcell_studio_supported_scenes.yaml";
  if (!fs::exists(p)) { if (error) *error = "Supported-scene registry missing: " + p.string(); return out; }
  try {
    const auto root = YAML::LoadFile(p.string());
    const auto scenes = root["scenes"];
    if (scenes && scenes.IsSequence()) {
      for (const auto & s : scenes) {
        SupportedSceneRegistryEntry e;
        e.name = scalar_or(s, "name", "");
        e.support_level = scalar_or(s, "support_level", "supported");
        if (!e.name.empty()) out.push_back(e);
      }
    }
  } catch (const std::exception & e) { if (error) *error = std::string("Failed parsing supported-scene registry: ")+e.what(); }
  return out;
}

AllScenesReadinessData load_latest_all_scenes_readiness_report(const fs::path & repo_root, std::string * error) {
  AllScenesReadinessData out;
  const fs::path p = repo_root / "reports" / "all_scenes_readiness_latest.json";
  if (!fs::exists(p)) { if (error) *error = "All-scenes readiness report missing: " + p.string(); return out; }
  out.report_path = p.string();
  try {
    const auto root = YAML::LoadFile(p.string());
    YAML::Node scenes = root["scenes"];
    if (!scenes && root.IsSequence()) scenes = root;
    if (scenes && scenes.IsSequence()) {
      for (const auto & s : scenes) {
        SceneReadinessSummary r;
        r.scene_name = scalar_or(s, "scene", scalar_or(s, "scene_name", ""));
        r.readiness_status = scalar_or(s, "status", scalar_or(s, "readiness_status", "UNKNOWN"));
        r.blockers_summary = scalar_or(s, "blockers_summary", "");
        r.required_files_status = scalar_or(s, "required_files_status", "UNKNOWN");
        r.static_validation_status = scalar_or(s, "static_validation_status", "UNKNOWN");
        r.guided_build_launch_readiness = scalar_or(s, "guided_build_launch_readiness", "UNKNOWN");
        r.fake_hardware_launch_readiness = scalar_or(s, "fake_hardware_launch_readiness", "UNKNOWN");
        if (s["blockers"] && s["blockers"].IsSequence()) for (const auto & b : s["blockers"]) r.blockers.push_back(b.as<std::string>());
        if (s["warnings"] && s["warnings"].IsSequence()) for (const auto & w : s["warnings"]) r.warnings.push_back(w.as<std::string>());
        if (!r.scene_name.empty()) out.by_scene[r.scene_name] = r;
      }
    }
  } catch (const std::exception & e) { if (error) *error = std::string("Failed parsing all-scenes readiness report: ")+e.what(); }
  return out;
}

} // namespace workcell_builder
