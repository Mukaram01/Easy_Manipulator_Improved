#pragma once
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
bool write_imported_asset_catalog_entry(const std::string &catalog_path, const std::string &entry_json, std::string *reason);
bool import_external_asset(const ImportedAssetRequest &request, std::string *summary);
}  // namespace workcell_builder
