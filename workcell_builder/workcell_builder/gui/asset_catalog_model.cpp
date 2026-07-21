#include "include/asset_catalog_model.h"

#include <boost/filesystem.hpp>

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <set>
#include <string>
#include <tuple>
#include <vector>

namespace fs = boost::filesystem;

namespace {

struct RankedAsset
{
  AssetCatalogEntry entry;
  int rank{0};
  std::string package_key;
};

std::string lower_copy(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return value;
}

bool has_path_component(const fs::path & path, const std::string & component)
{
  const std::string expected = lower_copy(component);
  for (const auto & part : path) {
    if (lower_copy(part.string()) == expected) return true;
  }
  return false;
}

bool is_supported_visual_mesh(const fs::path & path)
{
  const std::string ext = lower_copy(path.extension().string());
  return ext == ".stl" || ext == ".dae" || ext == ".obj" || ext == ".glb" || ext == ".gltf";
}

bool is_description_file(const fs::path & path)
{
  const std::string ext = lower_copy(path.extension().string());
  return ext == ".urdf" || ext == ".xacro";
}

bool path_is_collision(const fs::path & path)
{
  return has_path_component(path, "collision");
}

bool path_is_non_product_support_file(const fs::path & path)
{
  return has_path_component(path, "tests") ||
         has_path_component(path, "test") ||
         has_path_component(path, "rviz") ||
         has_path_component(path, "launch");
}

bool path_is_internal_placeholder(const fs::path & path)
{
  const std::string normalized = lower_copy(path.generic_string());
  return normalized.find("/workcell_builder/workcell_builder/assets/environment/") != std::string::npos ||
         normalized.find("placeholder") != std::string::npos;
}

bool path_is_canonical_asset(const fs::path & path)
{
  const std::string normalized = lower_copy(path.generic_string());
  return normalized.find("/assets/environment/") != std::string::npos ||
         normalized.find("/assets/robots/") != std::string::npos ||
         normalized.find("/assets/end_effectors/") != std::string::npos;
}

bool path_is_visual_mesh(const fs::path & path)
{
  return is_supported_visual_mesh(path) && has_path_component(path, "visual");
}

fs::path package_directory_for(const fs::path & file, const fs::path & root)
{
  fs::path current = file.parent_path();
  const std::string root_text = root.generic_string();
  while (!current.empty()) {
    if (fs::exists(current / "package.xml")) return current;
    if (current == root || current.generic_string().size() < root_text.size()) break;
    const fs::path parent = current.parent_path();
    if (parent == current) break;
    current = parent;
  }
  return fs::path();
}

std::string display_name_from_id(std::string id)
{
  const std::string suffix = "_description";
  if (id.size() > suffix.size() && id.compare(id.size() - suffix.size(), suffix.size(), suffix) == 0) {
    id.erase(id.size() - suffix.size());
  }
  std::replace(id.begin(), id.end(), '_', ' ');
  bool upper_next = true;
  for (char & c : id) {
    if (c == ' ') {
      upper_next = true;
    } else if (upper_next) {
      c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
      upper_next = false;
    }
  }
  return id;
}

int root_rank(const fs::path & root, const std::string & workspace_root, const std::string & repo_root)
{
  const std::string normalized = lower_copy(root.generic_string());
  if (normalized.find("/workcell_builder/workcell_builder/assets") != std::string::npos) return 80;
  if (!repo_root.empty() && normalized.find(lower_copy(repo_root + "/assets")) == 0) return 0;
  if (!workspace_root.empty() && normalized.find(lower_copy(workspace_root)) == 0) return 10;
  return 20;
}

int file_rank(
  const fs::path & path,
  const fs::path & root,
  const std::string & workspace_root,
  const std::string & repo_root)
{
  int rank = root_rank(root, workspace_root, repo_root);
  const std::string ext = lower_copy(path.extension().string());
  const std::string filename = lower_copy(path.filename().string());

  if (path_is_internal_placeholder(path)) rank += 200;
  if (is_description_file(path)) rank += 60;
  if (is_supported_visual_mesh(path)) rank += 20;
  if (path_is_visual_mesh(path)) rank -= 15;
  if (ext == ".dae") rank -= 5;
  if (filename == "d435.dae") rank -= 20;
  return rank;
}

std::vector<fs::path> candidate_roots(const std::string & workspace_root, const std::string & repo_root)
{
  std::vector<fs::path> roots;
  const char * env_root = std::getenv("WORKCELL_BUILDER_ASSET_ROOT");
  if (env_root && *env_root) roots.emplace_back(env_root);

  // Canonical repository/workspace packages must be considered before the
  // lightweight builder-internal primitive placeholders.
  roots.emplace_back(workspace_root + "/src/easy_manipulation_deployment/assets");
  roots.emplace_back(repo_root + "/assets");
  roots.emplace_back(workspace_root + "/src/assets");
  roots.emplace_back(repo_root + "/workcell_builder/workcell_builder/assets");

  std::vector<fs::path> deduped;
  std::set<std::string> seen;
  for (const auto & root : roots) {
    boost::system::error_code ec;
    const fs::path normalized = fs::exists(root, ec) && !ec ? fs::canonical(root, ec) : root.lexically_normal();
    const std::string key = normalized.generic_string();
    if (seen.insert(key).second) deduped.push_back(normalized);
  }
  return deduped;
}

void classify_entry(AssetCatalogEntry & entry, const fs::path & path, bool placeholder, bool canonical)
{
  const std::string identity = lower_copy(entry.id + " " + entry.display_name + " " + path.generic_string());
  const std::string ext = lower_copy(path.extension().string());

  entry.source = placeholder ? "generated_placeholder" : (canonical ? "canonical_asset" : "discovered_asset");
  entry.compatible_templates = {"Pick and Place Cell: UR5 + Robotiq 2F + table + cube + bin"};

  if (identity.find("robotiq") != std::string::npos ||
      identity.find("gripper") != std::string::npos ||
      identity.find("suction") != std::string::npos ||
      identity.find("airpick") != std::string::npos)
  {
    entry.category = "end_effector";
    entry.role_hints = {
      identity.find("suction") != std::string::npos || identity.find("airpick") != std::string::npos ?
      "suction_tool" : "gripper"};
    entry.icon_key = identity.find("suction") != std::string::npos ? "🧲" : "🦾";
    entry.readiness = placeholder ? "PREVIEW_ONLY" : "READY_WITH_WARNINGS";
    if (!placeholder) entry.warnings.push_back("Verify mount frame and RPY against the selected robot description.");
    entry.suggested_action = "Set as End Effector";
  } else if (identity.find("camera") != std::string::npos ||
             identity.find("realsense") != std::string::npos ||
             identity.find("d435") != std::string::npos)
  {
    entry.category = "camera_sensor";
    entry.role_hints = {"camera"};
    entry.icon_key = "📷";
    entry.readiness = placeholder ? "PREVIEW_ONLY" : "READY";
    entry.suggested_action = "Add as camera";
  } else if (identity.find("ur_description") != std::string::npos ||
             identity.find("universal_robot") != std::string::npos ||
             identity.find("fanuc") != std::string::npos ||
             identity.find("panda") != std::string::npos)
  {
    entry.category = "robot";
    entry.role_hints = {"robot_base"};
    entry.icon_key = "🤖";
    entry.readiness = identity.find("moveit") != std::string::npos ? "READY" : "MISSING_MOVEIT_CONFIG";
    if (entry.readiness != "READY") entry.blockers.push_back("MoveIt config not detected");
    entry.suggested_action = "Set as Robot";
  } else {
    entry.category = "environment_object";
    const bool support_surface =
      identity.find("table") != std::string::npos || identity.find("workbench") != std::string::npos;
    entry.role_hints = {support_surface ? "support_surface" : "environment_object"};
    entry.icon_key = "🧱";
    entry.readiness = placeholder ? "PREVIEW_ONLY" : "READY";
    entry.suggested_action = support_surface ? "Add as support surface" : "Add as environment object";
  }

  if (placeholder) {
    entry.warnings.push_back(
      "Generated primitive placeholder; prefer the canonical ROS description package visual when available.");
  } else if (canonical && ext == ".dae") {
    entry.warnings.push_back("Canonical COLLADA visual selected to preserve authored materials and appearance.");
  }
  entry.can_add_to_scene =
    entry.readiness == "READY" ||
    entry.readiness == "PREVIEW_ONLY" ||
    entry.readiness == "READY_WITH_WARNINGS";
}

AssetCatalogEntry make_seed(
  const std::string & name,
  const std::string & category,
  const std::string & readiness,
  const std::string & icon)
{
  AssetCatalogEntry entry;
  entry.id = name;
  entry.display_name = name;
  entry.category = category;
  entry.source = "fallback_seed";
  entry.readiness = readiness;
  entry.icon_key = icon;
  entry.suggested_action = "Preview and replace with workspace asset when available.";
  entry.warnings.push_back("fallback_seed");
  entry.can_add_to_scene = readiness == "PREVIEW_ONLY";
  return entry;
}

}  // namespace

AssetCatalogModel discover_asset_catalog(const std::string & workspace_root, const std::string & repo_root)
{
  AssetCatalogModel model;
  std::vector<RankedAsset> candidates;
  std::set<std::string> seen_files;

  for (const auto & root : candidate_roots(workspace_root, repo_root)) {
    model.discovered_roots.push_back(root.string());
    boost::system::error_code ec;
    if (!fs::exists(root, ec) || ec) continue;

    for (fs::recursive_directory_iterator it(root, ec), end; it != end && !ec; it.increment(ec)) {
      if (!fs::is_regular_file(it->path(), ec) || ec) continue;
      const fs::path path = it->path();
      if ((!is_supported_visual_mesh(path) && !is_description_file(path)) ||
          path_is_collision(path) ||
          path_is_non_product_support_file(path))
      {
        continue;
      }

      boost::system::error_code canonical_ec;
      const fs::path resolved = fs::canonical(path, canonical_ec);
      const std::string file_key = (canonical_ec ? path : resolved).generic_string();
      if (!seen_files.insert(file_key).second) continue;

      const fs::path package_dir = package_directory_for(path, root);
      const std::string package_name = package_dir.empty() ? std::string() : package_dir.filename().string();
      if (lower_copy(package_name).find("_moveit_config") != std::string::npos) continue;

      RankedAsset candidate;
      candidate.package_key = package_dir.empty() ? std::string() : package_dir.generic_string();
      candidate.rank = file_rank(path, root, workspace_root, repo_root);
      candidate.entry.id = package_name.empty() ? path.stem().string() : package_name;
      candidate.entry.display_name = display_name_from_id(candidate.entry.id);
      candidate.entry.path = path.string();
      candidate.entry.discovered_files = {candidate.entry.path};
      classify_entry(
        candidate.entry,
        path,
        path_is_internal_placeholder(path),
        path_is_canonical_asset(path) && !path_is_internal_placeholder(path));
      candidates.push_back(candidate);
    }
  }

  std::sort(
    candidates.begin(), candidates.end(),
    [](const RankedAsset & lhs, const RankedAsset & rhs) {
      return std::tie(lhs.rank, lhs.entry.category, lhs.entry.display_name, lhs.entry.path) <
             std::tie(rhs.rank, rhs.entry.category, rhs.entry.display_name, rhs.entry.path);
    });

  std::set<std::string> emitted_packages;
  for (const auto & candidate : candidates) {
    if (!candidate.package_key.empty() && !emitted_packages.insert(candidate.package_key).second) continue;
    model.assets.push_back(candidate.entry);
  }

  if (model.assets.empty()) {
    model.assets.push_back(make_seed("UR5", "robot", "PREVIEW_ONLY", "🤖"));
    model.assets.push_back(make_seed("Robotiq 2F", "end_effector", "PREVIEW_ONLY", "🦾"));
    model.assets.push_back(make_seed("simple_conveyor", "conveyor", "PREVIEW_ONLY", "➡️"));
    model.assets.push_back(make_seed("RealSense D435i", "camera_sensor", "PREVIEW_ONLY", "📷"));
  }
  return model;
}

bool export_asset_catalog_report(
  const AssetCatalogModel & model,
  const std::string & output_dir,
  std::string * json_path,
  std::string * html_path,
  std::string * error)
{
  boost::system::error_code ec;
  fs::create_directories(output_dir, ec);
  if (ec) {
    if (error) *error = ec.message();
    return false;
  }
  const fs::path json_file = fs::path(output_dir) / "asset_catalog.json";
  const fs::path html_file = fs::path(output_dir) / "asset_catalog.html";
  std::ofstream json(json_file.string());
  if (!json) return false;
  json << "{\n  \"discovered_roots\": [";
  for (size_t i = 0; i < model.discovered_roots.size(); ++i) {
    if (i) json << ",";
    json << "\"" << model.discovered_roots[i] << "\"";
  }
  json << "],\n  \"assets\": [";
  for (size_t i = 0; i < model.assets.size(); ++i) {
    if (i) json << ",";
    json << "{\"name\":\"" << model.assets[i].display_name
         << "\",\"readiness\":\"" << model.assets[i].readiness
         << "\",\"source\":\"" << model.assets[i].source << "\"}";
  }
  json << "],\n  \"note\": \"fake_hardware_first=true,no_runtime_motion=true\"\n}\n";

  std::ofstream html(html_file.string());
  html << "<html><body><h1>Asset Catalog Report</h1>"
       << "<p>fake_hardware_first / no-runtime-motion preserved</p><ul>";
  for (const auto & root : model.discovered_roots) html << "<li>" << root << "</li>";
  html << "</ul></body></html>";
  if (json_path) *json_path = json_file.string();
  if (html_path) *html_path = html_file.string();
  return true;
}
