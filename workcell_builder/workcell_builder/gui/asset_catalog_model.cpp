#include "include/asset_catalog_model.h"

#include <boost/filesystem.hpp>
#include <fstream>
#include <map>
#include <ctime>

namespace fs = boost::filesystem;

namespace {

std::vector<std::string> candidate_roots(const std::string & workspace_root, const std::string & repo_root)
{
  std::vector<std::string> roots;
  const char * env_root = std::getenv("WORKCELL_BUILDER_ASSET_ROOT");
  if (env_root && *env_root) roots.emplace_back(env_root);
  roots.emplace_back(workspace_root + "/src/easy_manipulation_deployment/assets");
  roots.emplace_back(workspace_root + "/src/assets");
  roots.emplace_back(repo_root + "/assets");
  roots.emplace_back(repo_root + "/workcell_builder/workcell_builder/assets");
  return roots;
}

AssetCatalogEntry make_seed(const std::string & name, const std::string & category, const std::string & readiness, const std::string & icon)
{
  AssetCatalogEntry e;
  e.id = name;
  e.display_name = name;
  e.category = category;
  e.source = "fallback_seed";
  e.readiness = readiness;
  e.icon_key = icon;
  e.suggested_action = "Preview and replace with workspace asset when available.";
  e.warnings.push_back("fallback_seed");
  return e;
}
}

AssetCatalogModel discover_asset_catalog(const std::string & workspace_root, const std::string & repo_root)
{
  AssetCatalogModel m;
  for (const auto & root : candidate_roots(workspace_root, repo_root)) {
    m.discovered_roots.push_back(root);
    boost::system::error_code ec;
    if (!fs::exists(root, ec) || ec) continue;

    for (fs::recursive_directory_iterator it(root, ec), end; it != end && !ec; it.increment(ec)) {
      if (!fs::is_regular_file(it->path(), ec)) continue;
      const std::string ext = it->path().extension().string();
      if (ext != ".urdf" && ext != ".xacro" && ext != ".stl") continue;

      AssetCatalogEntry e;
      e.id = it->path().stem().string();
      e.display_name = it->path().stem().string();
      e.path = it->path().string();
      e.discovered_files.push_back(e.path);
      e.source = (root.find(repo_root) == 0) ? "repo" : "workspace";
      e.compatible_templates = {"Pick and Place Cell: UR5 + Robotiq 2F + table + cube + bin"};

      const auto lower = e.id;
      if (lower.find("ur") != std::string::npos || lower.find("fanuc") != std::string::npos || lower.find("panda") != std::string::npos) {
        e.category = "robot";
        e.role_hints = {"robot_base"};
        e.icon_key = "🤖";
        e.readiness = (e.path.find("moveit") != std::string::npos) ? "READY" : "MISSING_MOVEIT_CONFIG";
        if (e.readiness != "READY") e.blockers.push_back("MoveIt config not detected");
        e.suggested_action = "Set as Robot";
      } else if (lower.find("robotiq") != std::string::npos || lower.find("gripper") != std::string::npos || lower.find("suction") != std::string::npos) {
        e.category = "end_effector";
        e.role_hints = {lower.find("suction") != std::string::npos ? "suction_tool" : "gripper"};
        e.icon_key = (lower.find("suction") != std::string::npos) ? "🧲" : "🦾";
        e.readiness = "READY_WITH_WARNINGS";
        e.warnings.push_back("Mount RPY default: -1.5708 -1.5708 0");
        e.suggested_action = "Set as End Effector";
      } else if (lower.find("camera") != std::string::npos || lower.find("realsense") != std::string::npos) {
        e.category = "camera_sensor";
        e.role_hints = {"camera"};
        e.icon_key = "📷";
        e.readiness = "PREVIEW_ONLY";
        e.warnings.push_back("metadata/preview asset");
        e.suggested_action = "Add as camera metadata";
      } else {
        e.category = "environment_object";
        e.role_hints = {"support_surface"};
        e.icon_key = "🧱";
        e.readiness = ext == ".stl" ? "PREVIEW_ONLY" : "READY";
        e.suggested_action = "Add as support surface/object";
      }
      e.can_add_to_scene = (e.readiness == "READY" || e.readiness == "PREVIEW_ONLY" || e.readiness == "READY_WITH_WARNINGS");
      m.assets.push_back(e);
    }
  }

  if (m.assets.empty()) {
    m.assets.push_back(make_seed("UR5", "robot", "PREVIEW_ONLY", "🤖"));
    m.assets.push_back(make_seed("Robotiq 2F", "end_effector", "PREVIEW_ONLY", "🦾"));
    m.assets.push_back(make_seed("simple_conveyor", "conveyor", "PREVIEW_ONLY", "➡️"));
    m.assets.push_back(make_seed("RealSense D435i", "camera_sensor", "PREVIEW_ONLY", "📷"));
  }
  return m;
}

bool export_asset_catalog_report(const AssetCatalogModel & model, const std::string & output_dir, std::string * json_path, std::string * html_path, std::string * error)
{
  boost::system::error_code ec;
  fs::create_directories(output_dir, ec);
  if (ec) { if (error) *error = ec.message(); return false; }
  const fs::path j = fs::path(output_dir) / "asset_catalog.json";
  const fs::path h = fs::path(output_dir) / "asset_catalog.html";
  std::ofstream jout(j.string());
  if (!jout) return false;
  jout << "{\n  \"discovered_roots\": [";
  for (size_t i = 0; i < model.discovered_roots.size(); ++i) { if (i) jout << ","; jout << "\"" << model.discovered_roots[i] << "\""; }
  jout << "],\n  \"assets\": [";
  for (size_t i = 0; i < model.assets.size(); ++i) { if (i) jout << ","; jout << "{\"name\":\"" << model.assets[i].display_name << "\",\"readiness\":\"" << model.assets[i].readiness << "\"}"; }
  jout << "],\n  \"note\": \"fake_hardware_first=true,no_runtime_motion=true\"\n}\n";
  std::ofstream hout(h.string());
  hout << "<html><body><h1>Asset Catalog Report</h1><p>fake_hardware_first / no-runtime-motion preserved</p><ul>";
  for (const auto & r : model.discovered_roots) hout << "<li>" << r << "</li>";
  hout << "</ul></body></html>";
  if (json_path) *json_path = j.string();
  if (html_path) *html_path = h.string();
  return true;
}
