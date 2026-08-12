#include "gui/asset_catalog_discovery.h"
#include "workcell_yaml_utils.hpp"
#include "workcell_warning_once.hpp"

#include <yaml-cpp/yaml.h>
#include <boost/algorithm/string.hpp>
#include <algorithm>
#include <set>

namespace fs = boost::filesystem;

namespace workcell_builder {
namespace {
std::string infer_role_hint(const std::string & category, const std::string & source_kind)
{
  const std::string key = boost::algorithm::to_lower_copy(category + " " + source_kind);
  if (key.find("robot") != std::string::npos) return "robot";
  if (key.find("end_effector") != std::string::npos || key.find("end effector") != std::string::npos) return "end_effector";
  if (key.find("table") != std::string::npos) return "table";
  if (key.find("conveyor") != std::string::npos) return "conveyor";
  if (key.find("camera") != std::string::npos || key.find("sensor") != std::string::npos) return "camera";
  if (key.find("bin") != std::string::npos || key.find("place") != std::string::npos) return "place_target";
  return "object";
}

bool has_supported_geometry(const fs::path & dir)
{
  boost::system::error_code ec;
  for (fs::recursive_directory_iterator it(dir, ec), end; it != end && !ec; it.increment(ec)) {
    if (!fs::is_regular_file(it->path(), ec) || ec) continue;
    const std::string ext = it->path().extension().string();
    if (ext == ".stl" || ext == ".dae" || ext == ".obj" || ext == ".urdf" || ext == ".xacro") return true;
  }
  return false;
}

std::string category_for_root(const fs::path & root)
{
  const std::string n = root.filename().string();
  if (n == "robots") return "Robots";
  if (n == "end_effectors") return "End Effectors";
  return "Environment Objects";
}

void add_entry(
  std::vector<DiscoveredAssetCatalogEntry> & out,
  std::set<std::string> & dedupe,
  const DiscoveredAssetCatalogEntry & entry)
{
  const std::string key = entry.source_path + "|" + entry.asset_id;
  if (dedupe.insert(key).second) out.push_back(entry);
}

void parse_manifest_file(
  const fs::path & manifest,
  const std::string & default_category,
  std::vector<DiscoveredAssetCatalogEntry> & out,
  std::set<std::string> & dedupe,
  bool require_declared_id = false)
{
  YAML::Node root;
  try { root = YAML::LoadFile(manifest.string()); } catch (const std::exception & e) {
    workcell_builder::log_warning_once_per_context_path_reason(
      "asset_catalog_manifest_parse", manifest, std::string("YAML parse failed: ") + e.what());
    return;
  }

  const YAML::Node assets = workcell_builder::yaml_map_key(root, "assets");
  if (!assets || !assets.IsSequence()) return;
  for (const auto & asset : assets) {
    if (!asset.IsMap()) continue;
    const std::string id = workcell_builder::yaml_map_value_or_empty(asset, "id");
    if (require_declared_id && id.empty()) continue;
    const std::string name = workcell_builder::yaml_map_value_or_empty(asset, "display_name");
    const std::string rel_path = workcell_builder::yaml_map_value_or_empty(asset, "path");
    const std::string declared_category = workcell_builder::yaml_map_value_or_empty(asset, "category");
    const fs::path resolved_path = rel_path.empty() ? manifest.parent_path() : (manifest.parent_path() / rel_path);
    DiscoveredAssetCatalogEntry entry;
    entry.asset_id = id.empty() ? resolved_path.filename().string() : id;
    entry.display_name = name.empty() ? entry.asset_id : name;
    entry.category = declared_category.empty() ? default_category : declared_category;
    boost::system::error_code source_ec;
    const fs::path canonical_path = fs::exists(resolved_path, source_ec) ? fs::canonical(resolved_path, source_ec) : resolved_path;
    entry.source_path = canonical_path.string();
    entry.source_kind = "manifest";
    entry.role_hint = infer_role_hint(entry.category, entry.source_kind);
    const bool ready = fs::exists(resolved_path) && (fs::is_regular_file(resolved_path) || has_supported_geometry(resolved_path));
    entry.availability = ready ? "ready" : "incomplete";
    entry.reason = ready ? std::string() : "manifest path missing or lacks supported geometry";
    const YAML::Node scale = workcell_builder::yaml_map_key(asset, "scale");
    if (scale && scale.IsScalar()) {
      try { entry.scale = scale.as<double>(); } catch (const YAML::Exception &) {}
    }
    add_entry(out, dedupe, entry);
  }
}

void discover_from_root(const fs::path & root, std::vector<DiscoveredAssetCatalogEntry> & out, std::set<std::string> & dedupe)
{
  boost::system::error_code ec;
  if (!fs::exists(root, ec) || ec || !fs::is_directory(root, ec) || ec) return;

  const std::string category = category_for_root(root);
  for (fs::directory_iterator it(root, ec), end; it != end && !ec; it.increment(ec)) {
    if (!fs::is_directory(it->path(), ec) || ec) continue;
    const fs::path candidate = it->path();
    const fs::path m1 = candidate / "asset_manifest.yaml";
    const fs::path m2 = candidate / "assets_manifest.yaml";
    if (fs::exists(m1)) parse_manifest_file(m1, category, out, dedupe);
    if (fs::exists(m2)) parse_manifest_file(m2, category, out, dedupe);

    DiscoveredAssetCatalogEntry inferred;
    inferred.asset_id = candidate.filename().string();
    inferred.display_name = inferred.asset_id;
    inferred.category = category;
    inferred.source_kind = "inferred";
    inferred.role_hint = infer_role_hint(inferred.category, inferred.source_kind);
    boost::system::error_code source_ec;
    inferred.source_path = fs::canonical(candidate, source_ec).string();
    if (source_ec) inferred.source_path = candidate.string();
    const bool ready = has_supported_geometry(candidate);
    inferred.availability = ready ? "ready" : "incomplete";
    inferred.reason = ready ? std::string() : "no .stl/.dae/.obj/.urdf/.xacro found";
    add_entry(out, dedupe, inferred);
  }
}

void add_template_asset_refs(const fs::path & repo_root, std::vector<DiscoveredAssetCatalogEntry> & out, std::set<std::string> & dedupe)
{
  const fs::path template_index = repo_root / "catalog" / "workcell_studio_demos.yaml";
  if (!fs::exists(template_index)) return;
  YAML::Node root;
  try { root = YAML::LoadFile(template_index.string()); } catch (const std::exception & e) {
    workcell_builder::log_warning_once_per_context_path_reason(
      "asset_catalog_template_parse", template_index, std::string("YAML parse failed: ") + e.what());
    return;
  }
  const YAML::Node templates = workcell_builder::yaml_map_key(root, "templates");
  if (!templates || !templates.IsSequence()) return;

  for (const auto & t : templates) {
    if (!t || !t.IsMap()) continue;
    const YAML::Node refs = workcell_builder::yaml_map_key(t, "asset_references");
    if (!refs || !refs.IsSequence()) continue;
    for (const auto & ref : refs) {
      if (!ref || !ref.IsMap()) continue;
      const std::string key = workcell_builder::yaml_map_value_or_empty(ref, "id");
      const std::string path = workcell_builder::yaml_map_value_or_empty(ref, "path");
      if (key.empty() && path.empty()) continue;
      DiscoveredAssetCatalogEntry entry;
      entry.asset_id = key.empty() ? fs::path(path).stem().string() : key;
      entry.display_name = entry.asset_id;
      entry.category = "Template References";
      entry.source_kind = "scene_template";
      entry.role_hint = infer_role_hint(entry.category, entry.source_kind);
      entry.source_path = path.empty() ? key : path;
      const fs::path resolved = path.empty() ? fs::path() : (repo_root / path);
      const bool ready = !path.empty() && fs::exists(resolved);
      entry.availability = ready ? "ready" : "incomplete";
      entry.reason = ready ? std::string() : "template reference path missing";
      add_entry(out, dedupe, entry);
    }
  }
}

}  // namespace

std::vector<DiscoveredAssetCatalogEntry> discover_asset_catalog_entries(
  const fs::path & repo_root,
  const fs::path & workspace_root,
  const fs::path & scene_catalog_root)
{
  std::vector<DiscoveredAssetCatalogEntry> out;
  std::set<std::string> dedupe;

  discover_from_root(repo_root / "assets" / "environment_objects", out, dedupe);
  discover_from_root(repo_root / "assets" / "robots", out, dedupe);
  discover_from_root(repo_root / "assets" / "end_effectors", out, dedupe);

  const fs::path workspace_assets = workspace_root / "src" / "easy_manipulation_deployment" / "assets";
  discover_from_root(workspace_assets / "environment_objects", out, dedupe);
  discover_from_root(workspace_assets / "robots", out, dedupe);
  discover_from_root(workspace_assets / "end_effectors", out, dedupe);

  // Scene imports are authored by their manifest. Do not infer identities from
  // filenames here: the declared ID is the stable catalog/placement identity.
  if (!scene_catalog_root.empty()) {
    const fs::path manifest = scene_catalog_root / "asset_manifest.yaml";
    boost::system::error_code ec;
    if (fs::exists(manifest, ec) && !ec) {
      parse_manifest_file(manifest, "Imported", out, dedupe, true);
    }
  }

  add_template_asset_refs(repo_root, out, dedupe);
  return out;
}

}  // namespace workcell_builder
