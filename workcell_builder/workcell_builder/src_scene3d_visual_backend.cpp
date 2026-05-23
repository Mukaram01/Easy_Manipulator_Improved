#include "scene3d_visual_backend.hpp"

#include <QFile>
#include <QFileInfo>
#include <QTextStream>
#include <QRegExp>

#if __has_include(<assimp/Importer.hpp>)
#define WORKCELL_BUILDER_HAS_ASSIMP 1
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#endif

namespace workcell_builder
{
namespace
{
bool parse_ascii_stl(const QByteArray & bytes, Scene3DMeshResource & out, int lim)
{
  QTextStream ts(bytes);
  QString line;
  QVector3D v[3]; int vi=0;
  while (ts.readLineInto(&line)) {
    const auto t = line.trimmed();
    if (!t.startsWith("vertex ")) continue;
    const auto p = t.split(QRegExp("\\s+"));
    if (p.size() < 4) continue;
    v[vi++] = QVector3D(p[1].toFloat(), p[2].toFloat(), p[3].toFloat());
    if (vi == 3) { out.triangles.push_back({v[0],v[1],v[2]}); vi = 0; if ((int)out.triangles.size() > lim) return false; }
  }
  return !out.triangles.empty();
}
}

Scene3DMeshLoadResult Scene3DVisualBackend::load_mesh_resource(const QString & mesh_path, int triangle_limit)
{
  Scene3DMeshLoadResult r;
  QFileInfo info(mesh_path);
  if (!info.exists()) { r.warning = "mesh missing on disk"; return r; }
  const QString ext = info.suffix().toLower();
#if WORKCELL_BUILDER_HAS_ASSIMP
  if (ext == "stl" || ext == "dae" || ext == "obj") {
    Assimp::Importer importer;
    const aiScene * scene = importer.ReadFile(mesh_path.toStdString(), aiProcess_Triangulate | aiProcess_JoinIdenticalVertices);
    if (scene && scene->HasMeshes()) {
      r.used_assimp = true; r.loader = "assimp";
      for (unsigned i=0;i<scene->mNumMeshes;++i) {
        const aiMesh * m = scene->mMeshes[i];
        for (unsigned f=0; f<m->mNumFaces; ++f) {
          const aiFace & face = m->mFaces[f];
          if (face.mNumIndices != 3) continue;
          auto v0 = m->mVertices[face.mIndices[0]];
          auto v1 = m->mVertices[face.mIndices[1]];
          auto v2 = m->mVertices[face.mIndices[2]];
          r.mesh.triangles.push_back({QVector3D(v0.x,v0.y,v0.z), QVector3D(v1.x,v1.y,v1.z), QVector3D(v2.x,v2.y,v2.z)});
          if ((int)r.mesh.triangles.size() > triangle_limit) { r.warning = "mesh triangle count exceeds limit"; return r; }
        }
      }
      r.success = !r.mesh.triangles.empty();
      return r;
    }
    r.warning = QString::fromStdString(importer.GetErrorString());
  }
#endif
  if (ext == "stl") {
    QFile f(mesh_path); if (!f.open(QIODevice::ReadOnly)) { r.warning = "mesh unreadable"; return r; }
    r.used_fallback_stl = true; r.loader = "fallback_stl";
    r.success = parse_ascii_stl(f.readAll(), r.mesh, triangle_limit);
    if (!r.success) r.warning = "fallback stl parser failed";
    return r;
  }
  r.warning = "unsupported mesh format";
  return r;
}
} // namespace
