#include "live_mesh_staging.hpp"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QSaveFile>
#include <QStringList>

namespace workcell_builder
{
namespace
{

// Must match STAGED_MESH_ROOTS in workcell_studio_web/viewer/viewer.js.
const QStringList kBrowserServedRoots{
  QStringLiteral("build/workcell_studio_web_scene/assets/"), QStringLiteral("workcell_studio_web/"),
  QStringLiteral("assets/"), QStringLiteral("scenes/")};
// Must match the asset root of _stage_visual_meshes in scripts/export_workcell_studio_web_scene_impl.py.
const QString kStagedAssetRoot = QStringLiteral("build/workcell_studio_web_scene/assets");

LiveMeshStaging refused(const QString & status, const QString & reason)
{
  return {QString(), status, reason};
}

}  // namespace

LiveMeshStaging stage_live_authoring_mesh(
  const QString & repo_root, const QString & scene_dir, const QString & scene_id,
  const QString & mesh_source)
{
  const QString source = mesh_source.trimmed();
  if (source.isEmpty()) return refused(QStringLiteral("no_mesh_uri"), QStringLiteral("the item has no mesh path"));
  const QString suffix = QFileInfo(source).suffix().toLower();
  if (suffix != QStringLiteral("stl") && suffix != QStringLiteral("dae") && suffix != QStringLiteral("obj")) {
    return refused(QStringLiteral("unsupported_format"),
      QStringLiteral("Web3D draws .stl, .dae and .obj meshes, not '%1'").arg(source));
  }
  const QString root = QFileInfo(repo_root.trimmed()).canonicalFilePath();
  if (root.isEmpty()) {
    return refused(QStringLiteral("missing_file"),
      QStringLiteral("the Product View repository root '%1' does not exist").arg(repo_root));
  }

  QString file = source;
  if (QFileInfo(source).isRelative()) {
    file = QDir(root).filePath(source);
    if (!QFileInfo(file).isFile() && !scene_dir.trimmed().isEmpty()) file = QDir(scene_dir.trimmed()).filePath(source);
  }
  const QFileInfo info(file);
  if (!info.isFile()) return refused(QStringLiteral("missing_file"), QStringLiteral("mesh file not found: %1").arg(source));
  const QString relative = QDir(root).relativeFilePath(info.canonicalFilePath());
  if (relative.startsWith(QStringLiteral("../")) || QDir::isAbsolutePath(relative)) {
    return refused(QStringLiteral("unsafe_path"),
      QStringLiteral("mesh %1 is outside the repository the Product View serves (%2)").arg(info.canonicalFilePath(), root));
  }
  for (const QString & served : kBrowserServedRoots) {
    if (relative.startsWith(served)) return {relative, QStringLiteral("available"), QString()};
  }

  const QString scene = scene_id.trimmed();
  if (scene.isEmpty() || scene == QStringLiteral(".") || scene == QStringLiteral("..") ||
      scene.contains(QLatin1Char('/')) || scene.contains(QLatin1Char('\\')))
  {
    return refused(QStringLiteral("unsafe_destination"),
      QStringLiteral("cannot stage %1 for scene id '%2'").arg(relative, scene_id));
  }
  const QString staged = QStringLiteral("%1/%2/%3").arg(kStagedAssetRoot, scene, relative);
  const QString destination = QDir(root).filePath(staged);

  QFile input(info.canonicalFilePath());
  if (!input.open(QIODevice::ReadOnly)) {
    return refused(QStringLiteral("stage_failed"),
      QStringLiteral("cannot read %1: %2").arg(info.canonicalFilePath(), input.errorString()));
  }
  const QByteArray bytes = input.readAll();
  QFile existing(destination);
  if (existing.open(QIODevice::ReadOnly) && existing.size() == bytes.size() && existing.readAll() == bytes) {
    return {staged, QStringLiteral("staged"), QString()};
  }
  existing.close();
  QSaveFile output(destination);
  if (!QDir().mkpath(QFileInfo(destination).absolutePath()) || !output.open(QIODevice::WriteOnly) ||
      output.write(bytes) != bytes.size() || !output.commit())
  {
    return refused(QStringLiteral("stage_failed"),
      QStringLiteral("cannot stage %1 to %2: %3").arg(relative, destination, output.errorString()));
  }
  return {staged, QStringLiteral("staged"), QString()};
}

}  // namespace workcell_builder
