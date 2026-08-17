#pragma once

#include <cstddef>
#include <string>
#include <vector>

struct AssetCatalogEntry
{
  std::string id;
  std::string display_name;
  std::string category;
  std::string vendor;
  std::string model;
  std::string path;
  std::string source;
  std::vector<std::string> role_hints;
  std::string readiness;
  std::vector<std::string> warnings;
  std::vector<std::string> blockers;
  std::vector<std::string> compatible_templates;
  std::string icon_key;
  std::vector<std::string> discovered_files;
  std::string suggested_action;
  double scale{1.0};
  bool can_add_to_scene = false;
};

// Presentation semantics for the Asset Library. These deliberately operate on
// the already-discovered catalog; filtering never re-scans the filesystem.
std::string normalize_asset_category(const AssetCatalogEntry & entry);
std::string asset_provenance(const AssetCatalogEntry & entry);
std::string asset_package_hint(const AssetCatalogEntry & entry);
bool asset_library_matches(
  const AssetCatalogEntry & entry, const std::string & query,
  const std::string & normalized_filter);
std::vector<size_t> filter_asset_catalog(
  const std::vector<AssetCatalogEntry> & assets, const std::string & query,
  const std::string & normalized_filter);
std::vector<std::string> record_recent_asset_id(
  const std::vector<std::string> & recent_ids, const std::string & asset_id,
  size_t maximum = 8);
std::vector<size_t> filter_recent_asset_catalog(
  const std::vector<AssetCatalogEntry> & assets,
  const std::vector<std::string> & recent_ids, const std::string & query);

struct AssetCatalogModel
{
  std::vector<std::string> discovered_roots;
  std::vector<AssetCatalogEntry> assets;
};

AssetCatalogModel discover_asset_catalog(
  const std::string & workspace_root,
  const std::string & repo_root,
  const std::string & scene_catalog_root = std::string());
bool export_asset_catalog_report(const AssetCatalogModel & model, const std::string & output_dir, std::string * json_path, std::string * html_path, std::string * error);
