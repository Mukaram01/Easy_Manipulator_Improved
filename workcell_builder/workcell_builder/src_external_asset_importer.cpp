#include "external_asset_importer.hpp"

#include <QCryptographicHash>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QSaveFile>
#include <QSet>

#include <algorithm>
#include <cctype>

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

namespace {
const QString kLibraryDir = QStringLiteral("workcell_builder/workcell_builder/assets/imported");
const QString kCatalogName = QStringLiteral("imported_environment_assets.json");
const QString kLegacyCatalog = QStringLiteral("workcell_builder/workcell_builder/config/asset_profiles/imported_environment_assets.json");
const QStringList kMeshFormats{QStringLiteral("stl"), QStringLiteral("obj"), QStringLiteral("dae")};

QString qstr(const std::string &value) { return QString::fromStdString(value); }

QString file_sha256(const QString &path) {
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) return QString();
  QCryptographicHash hash(QCryptographicHash::Sha256);
  if (!hash.addData(&file)) return QString();
  return QString::fromLatin1(hash.result().toHex());
}

// A catalog is a JSON array of asset objects. The old importer appended objects
// after the array ("[]\n{...}\n{...}"): recover those, never drop them.
bool read_catalog(const QString &path, QJsonArray *assets, QString *error) {
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) { *error = QStringLiteral("cannot be read"); return false; }
  const QByteArray bytes = file.readAll();
  if (bytes.trimmed().isEmpty()) { *assets = QJsonArray(); return true; }
  QJsonParseError parse;
  const QJsonDocument document = QJsonDocument::fromJson(bytes, &parse);
  if (parse.error == QJsonParseError::NoError) {
    if (!document.isArray()) { *error = QStringLiteral("is not a JSON array"); return false; }
    *assets = document.array();
    return true;
  }
  QJsonArray recovered;
  int depth = 0, start = -1;
  bool in_string = false, escaped = false;
  for (int i = 0; i < bytes.size(); ++i) {
    const char c = bytes[i];
    if (in_string) {
      if (escaped) escaped = false;
      else if (c == '\\') escaped = true;
      else if (c == '"') in_string = false;
      continue;
    }
    if (c == '"') { in_string = true; continue; }
    if (c == '{' || c == '[') { if (depth++ == 0) start = i; continue; }
    if (c != '}' && c != ']') continue;
    if (--depth < 0) break;
    if (depth > 0) continue;
    const QJsonDocument part = QJsonDocument::fromJson(bytes.mid(start, i - start + 1), &parse);
    if (parse.error != QJsonParseError::NoError) break;
    if (part.isObject()) recovered.append(part.object());
    else for (const auto &value : part.array()) recovered.append(value);
  }
  if (depth != 0 || parse.error != QJsonParseError::NoError) {
    *error = QStringLiteral("is not valid JSON (%1)").arg(parse.errorString());
    return false;
  }
  *assets = recovered;
  return true;
}

QString library_dir(const std::string &repo_root) { return QDir(qstr(repo_root)).filePath(kLibraryDir); }
QString local_catalog(const std::string &repo_root) { return QDir(library_dir(repo_root)).filePath(kCatalogName); }

// The local catalog once it exists; before the first import, the legacy bootstrap.
bool read_current_catalog(const std::string &repo_root, QJsonArray *assets, QString *path, QString *error) {
  *path = local_catalog(repo_root);
  if (!QFileInfo::exists(*path)) {
    const QString legacy = QDir(qstr(repo_root)).filePath(kLegacyCatalog);
    if (!QFileInfo::exists(legacy)) return true;
    *path = legacy;
  }
  return read_catalog(*path, assets, error);
}
}  // namespace

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
  const QFileInfo source(qstr(request.source_path));
  if (!source.isAbsolute() && request.source_path.find("..") != std::string::npos) { if (reason) *reason = "path traversal rejected"; return false; }
  if (source.isSymLink()) { if (reason) *reason = "symlink not allowed"; return false; }
  if (!source.exists()) { if (reason) *reason = "source path missing"; return false; }
  if (!kMeshFormats.contains(source.suffix().toLower())) { if (reason) *reason = "unsupported extension"; return false; }
  if (request.license.empty() || request.source_note.empty()) { if (reason) *reason = "license/source note required"; return false; }
  return true;
}

std::string imported_asset_status_label(bool ok, const std::string &reason) { return ok ? "Imported Asset OK" : ("Imported Asset Error: " + reason); }

std::string imported_asset_library_dir(const std::string &repo_root) { return library_dir(repo_root).toStdString(); }
std::string imported_asset_catalog_path(const std::string &repo_root) { return local_catalog(repo_root).toStdString(); }

ImportedAssetCatalog load_imported_asset_catalog(const std::string &repo_root) {
  ImportedAssetCatalog catalog;
  QJsonArray assets;
  QString path, error;
  catalog.ok = read_current_catalog(repo_root, &assets, &path, &error);
  catalog.path = path.toStdString();
  if (!catalog.ok) { catalog.error = QStringLiteral("%1 %2").arg(path, error).toStdString(); return catalog; }
  const QString library = QFileInfo(library_dir(repo_root)).canonicalFilePath();
  for (const auto &value : assets) {
    const QJsonObject asset = value.toObject();
    ImportedAssetCatalogEntry entry;
    entry.asset_id = asset.value("asset_id").toString().toStdString();
    entry.label = asset.value("label").toString(qstr(entry.asset_id)).toStdString();
    entry.category = asset.value("category").toString(QStringLiteral("Custom / Imported")).toStdString();
    entry.mesh_path = asset.value("mesh_path").toString().toStdString();
    entry.unit_scale_to_m = asset.value("unit_scale_to_m").toDouble(1.0);
    const QFileInfo mesh(QDir(qstr(repo_root)).filePath(qstr(entry.mesh_path)));
    const QString resolved = mesh.canonicalFilePath();
    if (entry.asset_id.empty() || sanitize_imported_asset_name(entry.asset_id) != entry.asset_id) {
      entry.problem = "the catalogued asset_id is empty or unsafe";
    } else if (entry.mesh_path.empty() || QDir::isAbsolutePath(qstr(entry.mesh_path))) {
      entry.problem = "the catalogued mesh path must be repository-relative";
    } else if (!mesh.exists()) {
      entry.problem = "the imported mesh file is missing";
    } else if (mesh.isSymLink() || !mesh.isFile() || library.isEmpty() || !resolved.startsWith(library + QLatin1Char('/'))) {
      entry.problem = "the imported mesh must be a regular file inside the imported-asset library";
    } else if (!kMeshFormats.contains(mesh.suffix().toLower())) {
      entry.problem = "only imported .stl, .obj and .dae meshes can be placed";
    } else {
      entry.resolved_mesh_path = resolved.toStdString();
      entry.available = true;
    }
    catalog.entries.push_back(entry);
  }
  return catalog;
}

ImportedAssetPublication publish_imported_asset(
  const std::string &repo_root, const std::string &source_path, const std::string &name, QJsonObject metadata) {
  ImportedAssetPublication result;
  auto fail = [&result](const QString &reason) { result.error = reason.toStdString(); return result; };
  const QFileInfo source(qstr(source_path));
  if (source.isSymLink()) return fail(QStringLiteral("symbolic-link sources are not allowed"));
  if (!source.isFile() || !source.isReadable()) return fail(QStringLiteral("the source is not a readable regular file"));
  const QString extension = source.suffix().toLower();
  if (!kMeshFormats.contains(extension)) return fail(QStringLiteral("unsupported extension; choose one .stl, .obj, or .dae file"));
  const QString stem = qstr(sanitize_imported_asset_name(name.empty() ? source.completeBaseName().toStdString() : name));
  if (stem.isEmpty()) return fail(QStringLiteral("the asset name has no safe characters"));
  if (metadata.value("license").toString().trimmed().isEmpty()) return fail(QStringLiteral("license/source note required"));

  const QString repo = QFileInfo(qstr(repo_root)).canonicalFilePath();
  const QString library = library_dir(repo_root);
  if (repo.isEmpty() || !QDir().mkpath(library)) return fail(QStringLiteral("cannot create the imported-asset library %1").arg(library));
  // A symlinked library would publish customer CAD outside the Git-ignored folder.
  if (QFileInfo(library).canonicalFilePath() != QDir(repo).filePath(kLibraryDir))
    return fail(QStringLiteral("the imported-asset library %1 must be a real folder inside the repository").arg(library));

  QJsonArray assets;
  QString catalog_read, error;
  if (!read_current_catalog(repo_root, &assets, &catalog_read, &error))
    return fail(QStringLiteral("the imported-asset catalog %1 %2; nothing was imported. Repair it or move it aside.").arg(catalog_read, error));

  const QString sha256 = file_sha256(source.absoluteFilePath());
  if (sha256.isEmpty()) return fail(QStringLiteral("cannot hash the source file"));
  const double scale = metadata.value("unit_scale_to_m").toDouble(1.0);
  QSet<QString> ids;
  for (const auto &value : assets) {
    const QJsonObject asset = value.toObject();
    ids.insert(asset.value("asset_id").toString());
    const QString mesh = QDir(repo).filePath(asset.value("mesh_path").toString());
    if (asset.value("source_sha256").toString() == sha256 && asset.value("unit_scale_to_m").toDouble(1.0) == scale &&
        QFileInfo(mesh).isFile()) {
      result.ok = result.reused = true;
      result.asset_id = asset.value("asset_id").toString().toStdString();
      result.mesh_path = asset.value("mesh_path").toString().toStdString();
      result.resolved_mesh_path = QFileInfo(mesh).canonicalFilePath().toStdString();
      return result;
    }
  }
  QString base = stem, asset_id, destination;
  for (int n = 2;; ++n) {
    asset_id = QStringLiteral("imported_") + base;
    destination = QDir(library).filePath(base + QLatin1Char('.') + extension);
    if (!ids.contains(asset_id) && !QFileInfo::exists(destination)) break;
    base = QStringLiteral("%1_%2").arg(stem).arg(n);
  }

  // Publish the mesh under a hidden name, verify it, then give it its final name.
  const QString partial = QDir(library).filePath(QStringLiteral(".%1.%2.importing").arg(base, extension));
  QFile::remove(partial);
  if (!QFile::copy(source.absoluteFilePath(), partial)) return fail(QStringLiteral("copying the source into the library failed"));
  if (file_sha256(partial) != sha256) { QFile::remove(partial); return fail(QStringLiteral("the source changed while it was copied")); }
  if (!QFile::rename(partial, destination)) { QFile::remove(partial); return fail(QStringLiteral("publishing %1 failed").arg(destination)); }

  const QString mesh_path = kLibraryDir + QLatin1Char('/') + QFileInfo(destination).fileName();
  QJsonObject entry = metadata;
  entry.insert("asset_id", asset_id);
  entry.insert("mesh_path", mesh_path);
  entry.insert("visual_type", extension);
  entry.insert("original_filename", source.fileName());
  entry.insert("source_sha256", sha256);
  entry.insert("unit_scale_to_m", scale);
  entry.insert("imported_at_utc", QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs));
  entry.insert("imported_by_version", QStringLiteral("workcell_studio_mesh_import/v3"));
  auto default_to = [&entry](const char *key, const QJsonValue &value) { if (!entry.contains(key)) entry.insert(key, value); };
  default_to("label", base);
  default_to("category", QStringLiteral("Custom / Imported"));
  default_to("asset_type", QStringLiteral("custom"));
  default_to("default_pose", QJsonArray{0, 0, 0, 0, 0, 0});
  default_to("default_z_hint", 0.0);
  default_to("source_note", QStringLiteral("Imported in Workcell Builder from %1").arg(source.fileName()));
  assets.append(entry);

  QSaveFile catalog(local_catalog(repo_root));
  if (!catalog.open(QIODevice::WriteOnly) ||
      catalog.write(QJsonDocument(assets).toJson(QJsonDocument::Indented)) < 0 || !catalog.commit()) {
    QFile::remove(destination);
    return fail(QStringLiteral("writing the imported-asset catalog failed; the copied file was removed"));
  }
  result.ok = true;
  result.asset_id = asset_id.toStdString();
  result.mesh_path = mesh_path.toStdString();
  result.resolved_mesh_path = QFileInfo(destination).canonicalFilePath().toStdString();
  return result;
}

bool import_external_asset(const std::string &repo_root, const ImportedAssetRequest &request, std::string *summary) {
  std::string reason;
  if (!validate_imported_asset_request(request, &reason)) { if (summary) *summary = imported_asset_status_label(false, reason); return false; }
  QJsonObject metadata{
    {"label", qstr(request.label.empty() ? sanitize_imported_asset_name(request.asset_name) : request.label)},
    {"category", qstr(request.asset_category.empty() ? "Custom / Imported" : request.asset_category)},
    {"asset_type", qstr(request.asset_type)},
    {"default_dimensions_m", QJsonArray{request.dim_x, request.dim_y, request.dim_z}},
    {"default_pose", QJsonArray{request.pose_x, request.pose_y, request.pose_z, request.roll, request.pitch, request.yaw}},
    {"default_z_hint", request.default_z_hint},
    {"placement_notes", "Imported via External Asset Import Wizard"},
    {"license", qstr(request.license)},
    {"source_note", qstr(request.source_note)},
    {"tags", qstr(request.tags)}};
  const auto published = publish_imported_asset(repo_root, request.source_path, request.asset_name, metadata);
  if (summary) *summary = imported_asset_status_label(published.ok, published.ok ? "imported and cataloged" : published.error);
  return published.ok;
}
}  // namespace workcell_builder
