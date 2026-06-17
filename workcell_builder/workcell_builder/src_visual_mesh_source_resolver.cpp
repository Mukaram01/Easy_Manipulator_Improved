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

QMap<QString, QString> discover_visual_mesh_package_map(const fs::path & scene_dir, const QString & workspace_root, QStringList * detected_asset_roots)
{
  QMap<QString, QString> package_map;
  auto add_package_root = [&](const QString & package_name, const fs::path & package_root) {
    if (package_name.trimmed().isEmpty()) return;
    if (!visual_mesh_package_root_exists(package_root)) return;
    if (!package_map.contains(package_name)) {
      package_map.insert(package_name, QString::fromStdString(package_root.string()));
    }
  };
  auto scan_root = [&](const QString & root, bool report_asset_root = false) {
    if (root.trimmed().isEmpty()) return;
    const QDir base(root);
    if (!base.exists()) return;
    if (report_asset_root && detected_asset_roots) {
      const QString canonical = QFileInfo(root).canonicalFilePath();
      detected_asset_roots->push_back(canonical.isEmpty() ? base.absolutePath() : canonical);
    }
    QDirIterator it(root, QStringList() << "package.xml", QDir::Files, QDirIterator::Subdirectories);
    while (it.hasNext()) {
      const QString package_xml = it.next();
      const QFileInfo info(package_xml);
      const QString package_root = info.dir().absolutePath();
      const QString package_name = QFileInfo(package_root).fileName();
      if (package_name.trimmed().isEmpty()) continue;
      if (!package_map.contains(package_name)) package_map.insert(package_name, package_root);
    }
  };

  const fs::path repo_root = workcell_builder_repo_root_from_source();
  const QString ws = workspace_root.trimmed();
  // Product preview priority: local ROS-style asset workspace first, repo assets second,
  // then generated/workspace/ament fallbacks for legacy scenes.
  scan_root(ws + "/src/assets", true);
  scan_root(QString::fromStdString((repo_root / "assets").string()), true);
  scan_root(QString::fromStdString((scene_dir / "generated").string()));
  scan_root(ws + "/install/share");
  scan_root(ws + "/src/easy_manipulation_deployment/assets");
  scan_root(ws + "/src");
  scan_root(QString::fromStdString((repo_root / "assets/environment").string()));
  scan_root(QString::fromStdString((repo_root / "assets/robots").string()));
  scan_root(QString::fromStdString((repo_root / "assets/end_effectors").string()));
  add_package_root(
    QStringLiteral("robotiq_85_description"),
    repo_root / "assets/end_effectors/robotiq_85_gripper/robotiq_85_description");
  add_package_root(
    QStringLiteral("workbench_description"),
    repo_root / "assets/environment/workbench_description");
  add_package_root(
    QStringLiteral("realsense2_description"),
    repo_root / "assets/environment/realsense2_description");
  const fs::path universal_robot_assets = repo_root / "assets/robots/universal_robot";
  const fs::path ur_description_assets = universal_robot_assets / "ur_description";
  if (!package_map.contains(QStringLiteral("ur_description"))) {
    if (fs::exists(ur_description_assets / "meshes/ur5/visual")) {
      package_map.insert(QStringLiteral("ur_description"), QString::fromStdString(ur_description_assets.string()));
    } else if (fs::exists(universal_robot_assets / "meshes/ur5/visual")) {
      package_map.insert(QStringLiteral("ur_description"), QString::fromStdString(universal_robot_assets.string()));
    }
  }
  scan_root(QStringLiteral("/opt/ros/humble/share"));
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
  if (trimmed_uri.startsWith("package://")) {
    const QString package_tail = trimmed_uri.mid(QString("package://").size());
    const int slash = package_tail.indexOf('/');
    const QString package_name = (slash >= 0) ? package_tail.left(slash) : package_tail;
    const QString package_rel = (slash >= 0) ? package_tail.mid(slash + 1) : QString();
    static QHash<QString, QMap<QString, QString>> workspace_cache;
    const QString cache_key = QString::fromStdString(scene_dir.string()) + "::" + workspace_root;
    if (!workspace_cache.contains(cache_key)) workspace_cache.insert(cache_key, discover_visual_mesh_package_map(scene_dir, workspace_root, nullptr));
    const QMap<QString, QString> package_map = workspace_cache.value(cache_key);
    const QString package_root = package_map.value(package_name);
    if (!package_root.trimmed().isEmpty()) {
      const QString resolved = add_candidate(QDir(package_root).filePath(package_rel));
      if (!resolved.isEmpty()) return resolved;
    }
  }

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
