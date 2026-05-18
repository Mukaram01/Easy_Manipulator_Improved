#include "gui/asset_catalog_discovery.h"

#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <set>

namespace fs = boost::filesystem;

namespace workcell_builder {
namespace {

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
  const std::string & category,
  std::vector<DiscoveredAssetCatalogEntry> & out,
  std::set<std::string> & dedupe)
{
  YAML::Node root;
  try { root = YAML::LoadFile(manifest.string()); } catch (...) { return; }

  const YAML::Node assets = root["assets"];
  if (!assets || !assets.IsSequence()) return;
  for (const auto & asset : assets) {
    if (!asset.IsMap()) continue;
    const std::string id = asset["id"] ? asset["id"].as<std::string>() : std::string();
    const std::string name = asset["display_name"] ? asset["display_name"].as<std::string>() : id;
    const std::string rel_path = asset["path"] ? asset["path"].as<std::string>() : std::string();
    const fs::path resolved_path = rel_path.empty() ? manifest.parent_path() : (manifest.parent_path() / rel_path);
    DiscoveredAssetCatalogEntry entry;
    entry.asset_id = id.empty() ? resolved_path.filename().string() : id;
    entry.display_name = name.empty() ? entry.asset_id : name;
    entry.category = category;
    boost::system::error_code source_ec;
    const fs::path canonical_path = fs::exists(resolved_path, source_ec) ? fs::canonical(resolved_path, source_ec) : resolved_path;
    entry.source_path = canonical_path.string();
    entry.source_kind = "manifest";
    const bool ready = fs::exists(resolved_path) && (fs::is_regular_file(resolved_path) || has_supported_geometry(resolved_path));
    entry.availability = ready ? "ready" : "incomplete";
    entry.reason = ready ? std::string() : "manifest path missing or lacks supported geometry";
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
  try { root = YAML::LoadFile(template_index.string()); } catch (...) { return; }
  const YAML::Node templates = root["templates"];
  if (!templates || !templates.IsSequence()) return;

  for (const auto & t : templates) {
    const YAML::Node refs = t["asset_references"];
    if (!refs || !refs.IsSequence()) continue;
    for (const auto & ref : refs) {
      const std::string key = ref["id"] ? ref["id"].as<std::string>() : std::string();
      const std::string path = ref["path"] ? ref["path"].as<std::string>() : std::string();
      if (key.empty() && path.empty()) continue;
      DiscoveredAssetCatalogEntry entry;
      entry.asset_id = key.empty() ? fs::path(path).stem().string() : key;
      entry.display_name = entry.asset_id;
      entry.category = "Template References";
      entry.source_kind = "scene_template";
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
  const fs::path & workspace_root)
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

  add_template_asset_refs(repo_root, out, dedupe);
  return out;
}

}  // namespace workcell_builder
