#pragma once

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
