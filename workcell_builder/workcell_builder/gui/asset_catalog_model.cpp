#include "include/asset_catalog_model.h"
#include "gui/asset_catalog_discovery.h"

#include <boost/filesystem.hpp>

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iterator>
#include <map>
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

// Identity of the geometry a library entry shows: the bytes of its visual mesh,
// or, for a description file without a mesh of its own, of the first
// package:// mesh it references. Empty when that geometry cannot be read.
std::string geometry_content_key(
  const fs::path & path, const std::map<std::string, fs::path> & package_dirs)
{
  const auto read = [](const fs::path & file) {
    std::ifstream in(file.string(), std::ios::binary);
    return in ? std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>()) : std::string();
  };
  fs::path mesh = path;
  const std::string extension = fs::extension(path);
  if (extension == ".xacro" || extension == ".urdf") {
    const std::string text = read(path);
    const std::string scheme = "package://";
    const auto start = text.find("filename=\"" + scheme);
    if (start == std::string::npos) return {};
    const auto uri_start = start + 10 + scheme.size();
    const auto uri = text.substr(uri_start, text.find('"', uri_start) - uri_start);
    const auto slash = uri.find('/');
    const auto owner = package_dirs.find(uri.substr(0, slash));
    if (slash == std::string::npos || owner == package_dirs.end()) return {};
    mesh = owner->second / uri.substr(slash + 1);
  }
  const std::string bytes = read(mesh);
  return bytes.empty() ? std::string() : std::to_string(bytes.size()) + ":" + std::to_string(std::hash<std::string>{}(bytes));
}

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

// Groups files of one asset when its directory has no package.xml (the
// builder-internal "<name>_description/{meshes,urdf}" assets), so the mesh and
// its xacro are one library entry instead of two.
fs::path description_directory_for(const fs::path & file, const fs::path & root)
{
  for (fs::path current = file.parent_path(); !current.empty() && current != root; current = current.parent_path()) {
    const std::string name = current.filename().string();
    if (name.size() > 12 && name.compare(name.size() - 12, 12, "_description") == 0) return current;
    if (current.parent_path() == current) break;
  }
  return fs::path();
}

// A package can withhold itself from the Asset Library while staying on disk
// (placeholder or broken geometry awaiting real CAD):
//   <export><workcell_asset_library status="replace" reason="..."/></export>
bool package_withheld_from_library(const fs::path & package_dir, std::string * reason)
{
  std::ifstream in((package_dir / "package.xml").string());
  if (!in) return false;
  const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  const auto tag = text.find("<workcell_asset_library");
  if (tag == std::string::npos) return false;
  const auto end = text.find('>', tag);
  const std::string element = text.substr(tag, end == std::string::npos ? std::string::npos : end - tag);
  if (element.find("status=\"replace\"") == std::string::npos &&
      element.find("status=\"remove\"") == std::string::npos) return false;
  if (reason) {
    const auto r = element.find("reason=\"");
    *reason = r == std::string::npos ? std::string() :
      element.substr(r + 8, element.find('"', r + 8) - (r + 8));
  }
  return true;
}

std::string humanize_asset_identifier(std::string value)
{
  value = fs::path(value).filename().string();
  const std::string extension = fs::path(value).extension().string();
  if (!extension.empty()) value.erase(value.size() - extension.size());
  for (const std::string suffix : {"_moveit_config", "_description"}) {
    if (value.size() > suffix.size() &&
      value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0)
    {
      value.erase(value.size() - suffix.size());
      break;
    }
  }
  std::replace(value.begin(), value.end(), '_', ' ');
  std::string collapsed;
  bool previous_space = true;
  for (const unsigned char c : value) {
    if (std::isspace(c)) {
      if (!previous_space) collapsed.push_back(' ');
      previous_space = true;
    } else {
      collapsed.push_back(static_cast<char>(c));
      previous_space = false;
    }
  }
  if (!collapsed.empty() && collapsed.back() == ' ') collapsed.pop_back();
  bool upper_next = true;
  for (char & c : collapsed) {
    if (c == ' ') {
      upper_next = true;
    } else if (upper_next) {
      c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
      upper_next = false;
    }
  }
  return collapsed;
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
  // Builder-internal assets contribute only the curated generic environment
  // primitives. Its robots/end_effectors trees are stale package copies kept for
  // the template instantiator, not library items.
  roots.emplace_back(repo_root + "/workcell_builder/workcell_builder/assets/environment");

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
  } else if (identity.find("camera") != std::string::npos &&
             identity.find("realsense") == std::string::npos && identity.find("d435") == std::string::npos &&
             (identity.find("stand") != std::string::npos || identity.find("mount") != std::string::npos ||
              identity.find("bracket") != std::string::npos || identity.find("gantry") != std::string::npos ||
              identity.find("clamp") != std::string::npos || identity.find("camera_frame") != std::string::npos))
  {
    // A camera stand/mount carries the camera: it is a physical fixture, not a
    // second camera sensor (the cell has one physical camera owner).
    entry.category = "environment_object";
    entry.role_hints = {"fixture"};
    entry.icon_key = "🧱";
    entry.readiness = placeholder ? "PREVIEW_ONLY" : "READY";
    entry.suggested_action = "Add as fixture";
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

std::string asset_provenance(const AssetCatalogEntry & entry)
{
  return entry.source == "scene_imported_asset" || entry.source == "local_imported_asset" ? "imported" : "built_in";
}

std::string asset_display_name(const AssetCatalogEntry & entry)
{
  if (!entry.display_name.empty()) {
    // Catalog manifests sometimes populate display_name with a filename or ROS
    // package identifier. Preserve genuine authored product names verbatim,
    // while treating identifier-shaped values as fallback metadata.
    if (entry.display_name.find('_') == std::string::npos &&
      fs::path(entry.display_name).extension().empty())
    {
      return entry.display_name;
    }
    return humanize_asset_identifier(entry.display_name);
  }
  if (!entry.model.empty()) return entry.model;
  const std::string fallback = !entry.path.empty() ? fs::path(entry.path).filename().string() : entry.id;
  const std::string humanized = humanize_asset_identifier(fallback);
  return humanized.empty() ? std::string("—") : humanized;
}

std::string asset_provenance_label(const AssetCatalogEntry & entry)
{
  return asset_provenance(entry) == "imported" ? "IMPORTED" : "BUILT-IN";
}

std::string normalize_asset_category(const AssetCatalogEntry & entry)
{
  const std::string key = lower_copy(entry.category + " " +
    (entry.role_hints.empty() ? std::string() : entry.role_hints.front()));
  if (key.find("robot") != std::string::npos) return "robot";
  if (key.find("gripper") != std::string::npos || key.find("end_effector") != std::string::npos ||
    key.find("end effector") != std::string::npos || key.find("suction") != std::string::npos) return "gripper";
  if (key.find("table") != std::string::npos || key.find("support_surface") != std::string::npos) return "table";
  if (key.find("camera") != std::string::npos || key.find("sensor") != std::string::npos) return "camera";
  if (key.find("environment") != std::string::npos || key.find("conveyor") != std::string::npos ||
    key.find("bin") != std::string::npos || key.find("fixture") != std::string::npos) return "environment";
  if (asset_provenance(entry) == "imported" || key.find("object") != std::string::npos) return "object";
  return "other";
}

std::string asset_category_label(const AssetCatalogEntry & entry)
{
  const std::string category = normalize_asset_category(entry);
  if (category == "robot") return "Robot";
  if (category == "gripper") return "Gripper";
  if (category == "table") return "Table";
  if (category == "camera") return "Camera";
  if (category == "environment") return "Environment";
  if (category == "object") return "Object";
  return "Other";
}

std::string asset_format_label(const AssetCatalogEntry & entry)
{
  const std::string extension = lower_copy(fs::path(entry.path).extension().string());
  if (extension == ".stl") return "STL";
  if (extension == ".dae") return "COLLADA";
  if (extension == ".obj") return "OBJ";
  if (extension == ".glb") return "GLB";
  if (extension == ".gltf") return "glTF";
  return std::string();
}

std::string asset_package_hint(const AssetCatalogEntry & entry)
{
  fs::path path(entry.path);
  fs::path current = path.parent_path();
  while (!current.empty()) {
    if (fs::exists(current / "package.xml")) return current.filename().string();
    const fs::path parent = current.parent_path();
    if (parent == current) break;
    current = parent;
  }
  return path.filename().string();
}

std::string asset_source_hint(const AssetCatalogEntry & entry)
{
  const std::string package = asset_package_hint(entry);
  if (!package.empty()) return package;
  return entry.path.empty() ? std::string() : fs::path(entry.path).filename().string();
}

bool asset_library_matches(
  const AssetCatalogEntry & entry, const std::string & query,
  const std::string & normalized_filter)
{
  const std::string filter = lower_copy(normalized_filter);
  const bool filter_match = filter.empty() || filter == "all" ||
    (filter == "imported" ? asset_provenance(entry) == "imported" : normalize_asset_category(entry) == filter);
  if (!filter_match) return false;
  const std::string needle = lower_copy(query);
  if (needle.empty()) return true;
  const std::string haystack = lower_copy(asset_display_name(entry) + " " + entry.display_name + " " + entry.id + " " +
    entry.category + " " + entry.vendor + " " + entry.model + " " + entry.path + " " +
    asset_package_hint(entry));
  return haystack.find(needle) != std::string::npos;
}

std::vector<size_t> filter_asset_catalog(
  const std::vector<AssetCatalogEntry> & assets, const std::string & query,
  const std::string & normalized_filter)
{
  std::vector<size_t> matches;
  for (size_t i = 0; i < assets.size(); ++i) {
    if (asset_library_matches(assets[i], query, normalized_filter)) matches.push_back(i);
  }
  return matches;
}

std::vector<std::string> record_recent_asset_id(
  const std::vector<std::string> & recent_ids, const std::string & asset_id,
  size_t maximum)
{
  std::vector<std::string> updated;
  if (!asset_id.empty() && maximum > 0) updated.push_back(asset_id);
  for (const auto & id : recent_ids) {
    if (id.empty() || id == asset_id || updated.size() >= maximum) continue;
    updated.push_back(id);
  }
  return updated;
}

std::vector<size_t> filter_recent_asset_catalog(
  const std::vector<AssetCatalogEntry> & assets,
  const std::vector<std::string> & recent_ids, const std::string & query)
{
  std::vector<size_t> matches;
  for (const auto & id : recent_ids) {
    const auto match = std::find_if(assets.cbegin(), assets.cend(), [&id](const auto & entry) {
      return entry.id == id;
    });
    if (match != assets.cend() && asset_library_matches(*match, query, "all")) {
      matches.push_back(static_cast<size_t>(std::distance(assets.cbegin(), match)));
    }
  }
  return matches;
}

AssetCatalogModel discover_asset_catalog(
  const std::string & workspace_root,
  const std::string & repo_root,
  const std::string & scene_catalog_root,
  const std::vector<AssetCatalogEntry> & library_imports)
{
  AssetCatalogModel model;
  std::vector<RankedAsset> candidates;
  std::set<std::string> seen_files;
  std::map<std::string, int> visual_meshes_per_package;

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
      std::string withheld_reason;
      if (!package_dir.empty() && package_withheld_from_library(package_dir, &withheld_reason)) {
        const std::string note = package_name + ": " + withheld_reason;
        if (std::find(model.withheld_packages.begin(), model.withheld_packages.end(), note) == model.withheld_packages.end()) {
          model.withheld_packages.push_back(note);
        }
        continue;
      }

      RankedAsset candidate;
      const fs::path group_dir = package_dir.empty() ? description_directory_for(path, root) : package_dir;
      candidate.package_key = group_dir.empty() ? std::string() : group_dir.generic_string();
      if (!package_dir.empty() && is_supported_visual_mesh(path)) ++visual_meshes_per_package[candidate.package_key];
      candidate.rank = file_rank(path, root, workspace_root, repo_root);
      candidate.entry.id = package_name.empty() ? path.stem().string() : package_name;
      candidate.entry.path = path.string();
      candidate.entry.discovered_files = {candidate.entry.path};
      classify_entry(
        candidate.entry,
        path,
        path_is_internal_placeholder(path),
        path_is_canonical_asset(path) && !path_is_internal_placeholder(path));
      // A package is one product: name it from the package, not from whichever
      // of its meshes ranked first ("Link 0" for Panda, "Finger" for the 3F).
      std::string product_name = package_name;
      const std::string suffix = "_description";
      if (product_name.size() > suffix.size() &&
          product_name.compare(product_name.size() - suffix.size(), suffix.size(), suffix) == 0) {
        product_name.resize(product_name.size() - suffix.size());
      }
      candidate.entry.display_name = product_name.empty()
        ? asset_display_name(candidate.entry) : humanize_asset_identifier(product_name);
      candidates.push_back(candidate);
    }
  }

  // Feed scene-local authored imports through the same model as every other
  // catalog source. Re-running discovery reads the manifest from disk, so an
  // import written immediately before a refresh is visible in this call.
  if (!scene_catalog_root.empty()) {
    boost::system::error_code root_ec;
    const fs::path requested_scene_root(scene_catalog_root);
    const fs::path normalized_scene_root =
      fs::exists(requested_scene_root, root_ec) && !root_ec ?
      fs::canonical(requested_scene_root, root_ec) : requested_scene_root.lexically_normal();
    model.discovered_roots.push_back(normalized_scene_root.string());
    const auto discovered = workcell_builder::discover_asset_catalog_entries(
      fs::path(repo_root), fs::path(workspace_root), normalized_scene_root);
    for (const auto & imported : discovered) {
      if (imported.source_kind != "manifest" ||
          fs::path(imported.source_path).parent_path() != normalized_scene_root)
      {
        continue;
      }
      RankedAsset candidate;
      candidate.rank = -100;
      candidate.entry.id = imported.asset_id;
      candidate.entry.display_name = imported.display_name;
      candidate.entry.category = imported.category;
      candidate.entry.path = imported.source_path;
      candidate.entry.source = "scene_imported_asset";
      candidate.entry.role_hints = {imported.role_hint};
      candidate.entry.readiness = imported.availability == "ready" ? "PREVIEW_ONLY" : "INCOMPLETE";
      candidate.entry.icon_key = "🧱";
      candidate.entry.scale = imported.scale;
      candidate.entry.can_add_to_scene = imported.availability == "ready";
      if (!imported.reason.empty()) candidate.entry.blockers.push_back(imported.reason);
      candidate.entry.discovered_files = {imported.source_path};
      candidates.push_back(candidate);
    }
  }

  // Imports from this PC's imported-asset library serve every cell. The caller
  // reads its catalog from disk for each discovery, so imports survive a restart.
  for (const auto & imported : library_imports) {
    RankedAsset candidate;
    candidate.rank = -100;
    candidate.entry = imported;
    candidate.entry.category = "Imported";
    candidate.entry.source = "local_imported_asset";
    if (candidate.entry.role_hints.empty()) candidate.entry.role_hints = {"object"};
    candidate.entry.icon_key = "🧱";
    candidate.entry.discovered_files = {candidate.entry.path};
    candidates.push_back(candidate);
  }

  std::sort(
    candidates.begin(), candidates.end(),
    [](const RankedAsset & lhs, const RankedAsset & rhs) {
      return std::tie(lhs.rank, lhs.entry.category, lhs.entry.display_name, lhs.entry.path) <
             std::tie(rhs.rank, rhs.entry.category, rhs.entry.display_name, rhs.entry.path);
    });

  std::map<std::string, fs::path> package_dirs;
  for (const auto & candidate : candidates) {
    if (!candidate.package_key.empty())
      package_dirs.emplace(fs::path(candidate.package_key).filename().string(), fs::path(candidate.package_key));
  }
  std::set<std::string> emitted_packages;
  std::set<std::string> emitted_id_paths;
  std::set<std::string> emitted_geometry;
  for (const auto & candidate : candidates) {
    if (!candidate.package_key.empty() && !emitted_packages.insert(candidate.package_key).second) continue;
    boost::system::error_code ec;
    const fs::path path = fs::path(candidate.entry.path);
    const fs::path canonical = fs::exists(path, ec) && !ec ? fs::canonical(path, ec) : path.lexically_normal();
    const std::string identity = candidate.entry.id + "|" + canonical.generic_string();
    if (!emitted_id_paths.insert(identity).second) continue;
    // Two packages that ship byte-identical geometry are one product; list it once
    // (both stay on disk for the cells that reference them).
    const std::string geometry = geometry_content_key(path, package_dirs);
    if (!geometry.empty() && !emitted_geometry.insert(geometry).second) continue;
    model.assets.push_back(candidate.entry);
    // Placing a multi-part tool adds one loose link mesh, not the assembled
    // gripper. It is configured as the cell's end effector instead.
    const auto meshes = visual_meshes_per_package.find(candidate.package_key);
    auto & placed = model.assets.back();
    if (placed.category == "end_effector" && meshes != visual_meshes_per_package.end() && meshes->second > 1) {
      placed.readiness = "ASSIGN_AS_END_EFFECTOR";
      placed.can_add_to_scene = false;
      placed.blockers.insert(placed.blockers.begin(), "Multi-part tool (" + std::to_string(meshes->second) +
        " meshes): choose it as the cell's end effector in New Cell > End Effector. Placing it here would add one loose part, not the assembled tool.");
    }
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
