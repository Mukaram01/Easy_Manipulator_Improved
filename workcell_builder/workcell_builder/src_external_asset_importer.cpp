#include "external_asset_importer.hpp"
#include <algorithm>

namespace workcell_builder {
// External Asset Import Wizard
// Import External Asset
// Select STL / URDF
// Asset Name
// Asset Category
// Asset Type
// Default Dimensions
// Default Pose
// Default Z Hint
// License / Source Note
// Validate Imported Asset
// Add to Asset Library
// Import and Place
// Import Summary

std::string sanitize_imported_asset_name(const std::string &name) {
  std::string out;
  for (char c : name) {
    if ((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_') out.push_back(c);
    else if (c >= 'A' && c <= 'Z') out.push_back(static_cast<char>(c - 'A' + 'a'));
    else if (c == ' ' || c == '-') out.push_back('_');
  }
  return out;
}

bool validate_imported_asset_request(const ImportedAssetRequest &request, std::string *reason) {
  if (sanitize_imported_asset_name(request.asset_name).empty()) { if (reason) *reason = "unsafe imported asset name"; return false; }
  if (request.license.empty() || request.source_note.empty()) { if (reason) *reason = "license/source note required"; return false; }
  return true;
}
std::string normalize_imported_asset_path(const std::string &path) { return path; }
std::string imported_asset_status_label(bool ok, const std::string &reason) { return ok ? "Imported Asset OK" : ("Imported Asset Error: " + reason); }
bool write_imported_asset_catalog_entry(const std::string &, const std::string &, std::string *reason) { if (reason) *reason = "catalog write placeholder"; return true; }
bool import_external_asset(const ImportedAssetRequest &request, std::string *summary) {
  std::string reason; bool ok = validate_imported_asset_request(request, &reason);
  if (summary) *summary = imported_asset_status_label(ok, reason);
  return ok;
}
}  // namespace workcell_builder
