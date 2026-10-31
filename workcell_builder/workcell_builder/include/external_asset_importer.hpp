#pragma once
#include <QJsonObject>
#include <string>
#include <vector>

namespace workcell_builder {
struct ImportedAssetRequest {
  std::string source_path;
  std::string asset_name;
  std::string label;
  std::string asset_category;
  std::string asset_type;
  double dim_x{1.0}, dim_y{1.0}, dim_z{1.0};
  double pose_x{0.0}, pose_y{0.0}, pose_z{0.0}, roll{0.0}, pitch{0.0}, yaw{0.0};
  double default_z_hint{0.0};
  std::string license;
  std::string source_note;
  std::string tags;
  bool allow_overwrite{false};
};

std::string sanitize_imported_asset_name(const std::string &name);
bool validate_imported_asset_request(const ImportedAssetRequest &request, std::string *reason);
std::string normalize_imported_asset_path(const std::string &path);
std::string imported_asset_status_label(bool ok, const std::string &reason);
bool import_external_asset(const std::string &repo_root, const ImportedAssetRequest &request, std::string *summary);

// The local imported-asset library: imported CAD and its one catalog stay on this
// PC (Git-ignored). The tracked config/asset_profiles catalog is a read-only legacy
// bootstrap: its entries are adopted by the first import and it is never written.
std::string imported_asset_library_dir(const std::string &repo_root);
std::string imported_asset_catalog_path(const std::string &repo_root);

struct ImportedAssetCatalogEntry {
  std::string asset_id;
  std::string label;
  std::string category;
  std::string mesh_path;           // repository-relative, as catalogued
  std::string resolved_mesh_path;  // absolute; empty when the entry is unsafe
  double unit_scale_to_m{1.0};
  bool available{false};
  std::string problem;
};

struct ImportedAssetCatalog {
  bool ok{true};     // false: the catalog exists but is not a JSON array of assets
  std::string path;  // the file read: the local catalog, or the legacy bootstrap
  std::string error;
  std::vector<ImportedAssetCatalogEntry> entries;
};

ImportedAssetCatalog load_imported_asset_catalog(const std::string &repo_root);

struct ImportedAssetPublication {
  bool ok{false};
  bool reused{false};  // the same geometry in the same units was already in the library
  std::string asset_id;
  std::string mesh_path;
  std::string resolved_mesh_path;
  std::string error;
};

// Copies one .stl/.obj/.dae file into the library and records metadata (label,
// category, units, bounds, licence) in the catalog. The mesh is published before
// the catalog names it and removed again if the catalog cannot be written; an
// existing file or catalog entry is never overwritten.
ImportedAssetPublication publish_imported_asset(
  const std::string &repo_root, const std::string &source_path, const std::string &name, QJsonObject metadata);
}  // namespace workcell_builder
