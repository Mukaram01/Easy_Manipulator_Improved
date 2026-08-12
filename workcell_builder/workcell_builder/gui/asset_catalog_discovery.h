#pragma once

#include <boost/filesystem.hpp>
#include <string>
#include <vector>

namespace workcell_builder {

struct DiscoveredAssetCatalogEntry
{
  std::string asset_id;
  std::string display_name;
  std::string category;
  std::string source_path;
  std::string source_kind;
  std::string role_hint;
  std::string availability;
  std::string reason;
  double scale{1.0};
};

std::vector<DiscoveredAssetCatalogEntry> discover_asset_catalog_entries(
  const boost::filesystem::path & repo_root,
  const boost::filesystem::path & workspace_root,
  const boost::filesystem::path & scene_catalog_root = boost::filesystem::path());

}  // namespace workcell_builder
