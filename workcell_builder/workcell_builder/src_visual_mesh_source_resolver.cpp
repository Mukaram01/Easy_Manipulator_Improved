// Copyright 2026 Mukaram01
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "visual_mesh_source_resolver.hpp"

#include <QDir>
#include <QDirIterator>
#include <QFileInfo>
#include <QHash>
#include <QMap>
#include <QProcessEnvironment>
#include <QStringList>

namespace fs = boost::filesystem;

namespace workcell_builder
{

boost::filesystem::path workcell_builder_repo_root_from_source()
{
  fs::path cursor = fs::path(__FILE__).parent_path();
  for (int depth = 0; depth < 8 && !cursor.empty(); ++depth) {
    if (fs::exists(cursor / "assets") && fs::exists(cursor / "workcell_builder")) {
      return cursor;
    }
    if (cursor == cursor.parent_path()) break;
    cursor = cursor.parent_path();
  }
  return fs::path(__FILE__).parent_path().parent_path().parent_path().parent_path();
}

static bool visual_mesh_package_root_exists(const fs::path & package_root)
{
  return fs::exists(package_root / "package.xml") ||
         fs::exists(package_root / "meshes") ||
         fs::exists(package_root / "urdf") ||
         fs::exists(package_root / "dae") ||
         fs::exists(package_root / "stl");
}

static QString canonical_or_absolute(const QString & path)
{
  const QFileInfo info(path);
  const QString canonical = info.canonicalFilePath();
  return canonical.isEmpty() ? info.absoluteFilePath() : canonical;
}

QString mesh_asset_category_for_path(const QString & mesh_path)
{
  const QString lower = mesh_path.toLower();
  if (lower.contains(QStringLiteral("/robots/")) || lower.contains(QStringLiteral("ur_description")) ||
      lower.contains(QStringLiteral("universal_robot")) || lower.contains(QStringLiteral("/ur5")) ||
      lower.contains(QStringLiteral("/ur3")) || lower.contains(QStringLiteral("/ur10"))) {
    return QStringLiteral("robot");
  }
  if (lower.contains(QStringLiteral("/end_effectors/")) || lower.contains(QStringLiteral("gripper")) ||
      lower.contains(QStringLiteral("robotiq")) || lower.contains(QStringLiteral("suction"))) {
    return QStringLiteral("gripper");
  }
  if (lower.contains(QStringLiteral("workbench")) || lower.contains(QStringLiteral("table"))) {
    return QStringLiteral("table/workbench");
  }
  if (lower.contains(QStringLiteral("realsense")) || lower.contains(QStringLiteral("camera")) || lower.contains(QStringLiteral("d435"))) {
    return QStringLiteral("camera");
  }
  if (lower.contains(QStringLiteral("/environment/"))) {
    return QStringLiteral("environment");
  }
  return QStringLiteral("other");
}

QStringList product_asset_roots(const QString & workspace_root)
{
  QStringList roots;
  const QString explicit_root = QProcessEnvironment::systemEnvironment().value(QStringLiteral("WORKCELL_ASSET_ROOT")).trimmed();
  if (!explicit_root.isEmpty()) roots << explicit_root;
  const QString ws = workspace_root.trimmed();
  if (!ws.isEmpty()) roots << (ws + QStringLiteral("/src/assets"));
  roots << QString::fromStdString((workcell_builder_repo_root_from_source() / "assets").string());
  QStringList existing;
  for (const QString & root : roots) {
    if (root.trimmed().isEmpty()) continue;
    const QFileInfo info(root);
    if (!info.exists() || !info.isDir()) continue;
    const QString canonical = canonical_or_absolute(root);
    if (!existing.contains(canonical)) existing << canonical;
  }
  return existing;
}

QMap<QString, int> discover_visual_mesh_asset_category_counts(
  const QStringList & asset_roots, int * total_mesh_count, QStringList * mesh_paths)
{
  QMap<QString, int> counts;
  for (const QString & category : {QStringLiteral("robot"), QStringLiteral("gripper"),
      QStringLiteral("table/workbench"), QStringLiteral("camera"), QStringLiteral("environment"),
      QStringLiteral("other")}) {
    counts.insert(category, 0);
  }
  int total = 0;
  for (const QString & root : asset_roots) {
    QDirIterator it(root, QStringList() << QStringLiteral("*.stl") << QStringLiteral("*.STL")
                                      << QStringLiteral("*.dae") << QStringLiteral("*.DAE")
                                      << QStringLiteral("*.obj") << QStringLiteral("*.OBJ"),
                    QDir::Files, QDirIterator::Subdirectories);
    while (it.hasNext()) {
      const QString mesh = canonical_or_absolute(it.next());
      if (mesh_paths) mesh_paths->push_back(mesh);
      const QString category = mesh_asset_category_for_path(mesh);
      counts[category] = counts.value(category) + 1;
      ++total;
    }
  }
  if (mesh_paths) mesh_paths->removeDuplicates();
  if (total_mesh_count) *total_mesh_count = total;
  return counts;
}

QMap<QString, QString> discover_visual_mesh_package_map(const fs::path & scene_dir, const QString & workspace_root, QStringList * detected_asset_roots)
{
  Q_UNUSED(scene_dir);
  QMap<QString, QString> package_map;
  auto add_package_root = [&](const QString & package_name, const fs::path & package_root, bool authoritative = false) {
    if (package_name.trimmed().isEmpty()) return;
    if (!visual_mesh_package_root_exists(package_root)) return;
    const QString root = canonical_or_absolute(QString::fromStdString(package_root.string()));
    if (authoritative || !package_map.contains(package_name)) {
      package_map.insert(package_name, root);
    }
  };
  const QStringList asset_roots = product_asset_roots(workspace_root);
  if (detected_asset_roots) *detected_asset_roots << asset_roots;
  for (const QString & root : asset_roots) {
    QDirIterator it(root, QStringList() << "package.xml", QDir::Files, QDirIterator::Subdirectories);
    while (it.hasNext()) {
      const QFileInfo info(it.next());
      const QString package_root = info.dir().absolutePath();
      const QString package_name = QFileInfo(package_root).fileName();
      if (!package_name.trimmed().isEmpty() && !package_map.contains(package_name)) {
        package_map.insert(package_name, package_root);
      }
    }
  }

  const fs::path repo_root = workcell_builder_repo_root_from_source();
  // These shipped product assets are the RViz/source-of-truth packages used by
  // generated Scene3D rows.  Prefer them over stale workspace copies so package
  // URIs such as package://workbench_description/... cannot resolve to an older
  // or same-named mesh from another workspace and explode the table/camera view.
  add_package_root(
    QStringLiteral("robotiq_85_description"),
    repo_root / "assets/end_effectors/robotiq_85_gripper/robotiq_85_description", true);
  add_package_root(
    QStringLiteral("workbench_description"),
    repo_root / "assets/environment/workbench_description", true);
  add_package_root(
    QStringLiteral("realsense2_description"),
    repo_root / "assets/environment/realsense2_description", true);
  const fs::path ur_description_assets = repo_root / "assets/robots/universal_robot/ur_description";
  if (fs::exists(ur_description_assets / "meshes/ur5/visual")) {
    package_map.insert(QStringLiteral("ur_description"), canonical_or_absolute(QString::fromStdString(ur_description_assets.string())));
  }
  return package_map;
}

QString resolve_visual_mesh_source_path(
  const QString & raw_path, const QString & package_uri, const fs::path & scene_dir,
  const QString & workspace_root, QStringList * tried_candidates)
{
  const auto add_candidate = [&](const QString & candidate) {
    if (tried_candidates) tried_candidates->push_back(candidate);
    const QFileInfo info(candidate);
    if (!info.exists() || !info.isFile()) return QString();
    const QString canonical = info.canonicalFilePath();
    return canonical.isEmpty() ? info.absoluteFilePath() : canonical;
  };
  auto as_repo_relative = [&](const QString & rel) {
    return QString::fromStdString((workcell_builder_repo_root_from_source() / rel.toStdString()).string());
  };
  auto as_scene_relative = [&](const QString & rel) {
    return QString::fromStdString((scene_dir / rel.toStdString()).string());
  };

  const QString trimmed_uri = package_uri.trimmed();
  QString package_name;
  QString package_rel;
  if (trimmed_uri.startsWith("package://")) {
    const QString package_tail = trimmed_uri.mid(QString("package://").size());
    const int slash = package_tail.indexOf('/');
    package_name = (slash >= 0) ? package_tail.left(slash) : package_tail;
    package_rel = (slash >= 0) ? package_tail.mid(slash + 1) : QString();
  }

  // Package URIs are authoritative for generated visual rows.  A cached
  // resolved_source_path can be stale or can point at a different package with
  // the same mesh filename, for example table_description/.../table.stl for a
  // package://workbench_description/.../table.stl row.  Resolve the declared
  // package first so Scene3D uses the intended mesh and URDF mesh scale.
  if (trimmed_uri.startsWith("package://") && !package_name.isEmpty() && !package_rel.isEmpty()) {
    static QHash<QString, QMap<QString, QString>> workspace_cache;
    const QString cache_key = QString::fromStdString(scene_dir.string()) + "::" + workspace_root;
    if (!workspace_cache.contains(cache_key)) workspace_cache.insert(cache_key, discover_visual_mesh_package_map(scene_dir, workspace_root, nullptr));
    const QMap<QString, QString> package_map = workspace_cache.value(cache_key);
    const QString package_root = package_map.value(package_name);
    if (!package_root.trimmed().isEmpty()) {
      const QString resolved = add_candidate(QDir(package_root).filePath(package_rel));
      if (!resolved.isEmpty()) return resolved;
    }

    // Fallback asset scan must still respect the requested package name.  Do
    // not accept any mesh that merely ends in /meshes/visual/table.stl because
    // several packages can legitimately contain identically named STL files.
    QStringList meshes; int total = 0;
    discover_visual_mesh_asset_category_counts(product_asset_roots(workspace_root), &total, &meshes);
    Q_UNUSED(total);
    const QString package_suffix = QStringLiteral("/") + package_name + QStringLiteral("/") + package_rel;
    for (const QString & mesh : meshes) {
      if (mesh.endsWith(package_suffix)) {
        const QString resolved = add_candidate(mesh);
        if (!resolved.isEmpty()) return resolved;
      }
    }
  }

  // Explicit local asset_path / mesh_path is only used after package:// has had
  // a chance to resolve the owning package. This keeps hand-authored local files
  // working while preventing stale generated paths from overriding package_uri.
  const QString trimmed_raw = raw_path.trimmed();
  if (!trimmed_raw.isEmpty()) {
    QFileInfo raw_info(trimmed_raw);
    if (raw_info.isAbsolute()) {
      const QString resolved = add_candidate(raw_info.absoluteFilePath());
      if (!resolved.isEmpty()) return resolved;
    } else {
      const QString repo_resolved = add_candidate(as_repo_relative(trimmed_raw));
      if (!repo_resolved.isEmpty()) return repo_resolved;
      const QString scene_resolved = add_candidate(as_scene_relative(trimmed_raw));
      if (!scene_resolved.isEmpty()) return scene_resolved;
    }
  }

  if (trimmed_uri.startsWith("file://")) {
    const QString resolved = add_candidate(trimmed_uri.mid(7));
    if (!resolved.isEmpty()) return resolved;
  } else if (trimmed_uri.startsWith("/")) {
    const QString resolved = add_candidate(trimmed_uri);
    if (!resolved.isEmpty()) return resolved;
  }
  return QString();
}

}  // namespace workcell_builder
