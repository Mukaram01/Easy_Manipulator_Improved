#include "external_asset_importer.hpp"
#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace fs = std::filesystem;
namespace workcell_builder {

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
// Custom / Imported

static bool _is_supported_ext(const std::string &ext) { return ext == ".stl" || ext == ".urdf" || ext == ".xacro"; }

std::string sanitize_imported_asset_name(const std::string &name) {
  std::string out;
  for (char c : name) {
    if (std::isalnum(static_cast<unsigned char>(c))) out.push_back(static_cast<char>(std::tolower(c)));
    else if (c == '_' || c == '-' || c == ' ') out.push_back('_');
  }
  while (out.find("__") != std::string::npos) out.erase(out.find("__"), 1);
  if (!out.empty() && out.front() == '_') out.erase(out.begin());
  if (!out.empty() && out.back() == '_') out.pop_back();
  return out;
}

std::string normalize_imported_asset_path(const std::string &path) {
  std::string p = path;
  std::replace(p.begin(), p.end(), '\\', '/');
  return p;
}

bool validate_imported_asset_request(const ImportedAssetRequest &request, std::string *reason) {
  const std::string safe_name = sanitize_imported_asset_name(request.asset_name);
  if (safe_name.empty() || safe_name.find("..") != std::string::npos) { if (reason) *reason = "unsafe imported asset name"; return false; }
  fs::path source(request.source_path);
  if (!source.is_absolute() && request.source_path.find("..") != std::string::npos) { if (reason) *reason = "path traversal rejected"; return false; }
  if (!fs::exists(source)) { if (reason) *reason = "source path missing"; return false; }
  if (fs::is_symlink(source)) { if (reason) *reason = "symlink not allowed"; return false; }
  std::string ext = source.extension().string(); std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
  if (!_is_supported_ext(ext)) { if (reason) *reason = "unsupported extension"; return false; }
  if (request.license.empty() || request.source_note.empty()) { if (reason) *reason = "license/source note required"; return false; }
  return true;
}

std::string imported_asset_status_label(bool ok, const std::string &reason) { return ok ? "Imported Asset OK" : ("Imported Asset Error: " + reason); }

bool write_imported_asset_catalog_entry(const std::string &catalog_path, const std::string &entry_json, std::string *reason) {
  try {
    std::ofstream out(catalog_path, std::ios::app);
    out << entry_json << "\n";
    return true;
  } catch (const std::exception &e) { if (reason) *reason = e.what(); return false; }
}

bool import_external_asset(const ImportedAssetRequest &request, std::string *summary) {
  std::string reason;
  if (!validate_imported_asset_request(request, &reason)) { if (summary) *summary = imported_asset_status_label(false, reason); return false; }

  const fs::path repo = fs::current_path();
  const fs::path import_dir = repo / "workcell_builder/workcell_builder/assets/imported";
  const fs::path catalog = repo / "workcell_builder/workcell_builder/config/asset_profiles/imported_environment_assets.json";
  fs::create_directories(import_dir);

  fs::path src(request.source_path);
  std::string ext = src.extension().string(); std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
  const std::string safe = sanitize_imported_asset_name(request.asset_name);
  fs::path dest = import_dir / (safe + ext);
  int counter = 1;
  while (fs::exists(dest) && !request.allow_overwrite) {
    dest = import_dir / (safe + "_" + std::to_string(counter++) + ext);
  }
  if (fs::exists(dest) && request.allow_overwrite == false) { if (summary) *summary = imported_asset_status_label(false, "name collision"); return false; }
  fs::copy_file(src, dest, request.allow_overwrite ? fs::copy_options::overwrite_existing : fs::copy_options::none);

  const std::string rel = normalize_imported_asset_path(fs::relative(dest, repo).string());
  std::ostringstream entry;
  entry << "{\n"
        << "  \"asset_id\": \"" << safe << "\",\n"
        << "  \"label\": \"" << (request.label.empty() ? safe : request.label) << "\",\n"
        << "  \"category\": \"" << (request.asset_category.empty() ? "Custom / Imported" : request.asset_category) << "\",\n"
        << "  \"asset_type\": \"" << request.asset_type << "\",\n"
        << "  \"mesh_path\": \"" << rel << "\",\n"
        << "  \"default_dimensions_m\": [" << request.dim_x << ", " << request.dim_y << ", " << request.dim_z << "],\n"
        << "  \"default_pose\": [" << request.pose_x << ", " << request.pose_y << ", " << request.pose_z << ", " << request.roll << ", " << request.pitch << ", " << request.yaw << "],\n"
        << "  \"default_z_hint\": " << request.default_z_hint << ",\n"
        << "  \"placement_notes\": \"Imported via External Asset Import Wizard\",\n"
        << "  \"license\": \"" << request.license << "\",\n"
        << "  \"source_note\": \"" << request.source_note << "\",\n"
        << "  \"imported_by_version\": \"workcell_builder_external_import_v1\",\n"
        << "  \"tags\": \"" << request.tags << "\"\n"
        << "}";

  if (!write_imported_asset_catalog_entry(catalog.string(), entry.str(), &reason)) { if (summary) *summary = imported_asset_status_label(false, reason); return false; }
  if (summary) *summary = imported_asset_status_label(true, "imported and cataloged");
  return true;
}
}  // namespace workcell_builder
