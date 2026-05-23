#ifndef WORKCELL_BUILDER_SCENE3D_VISUAL_BACKEND_HPP_
#define WORKCELL_BUILDER_SCENE3D_VISUAL_BACKEND_HPP_

#include <QVector3D>
#include <QString>
#include <vector>

namespace workcell_builder
{
struct Scene3DMeshTriangle { QVector3D a, b, c; };
struct Scene3DMeshResource { std::vector<Scene3DMeshTriangle> triangles; double collada_unit_meter{1.0}; };
struct Scene3DMeshLoadResult {
  bool success{false};
  bool used_assimp{false};
  bool used_fallback_stl{false};
  QString loader;
  QString warning;
  Scene3DMeshResource mesh;
};

class Scene3DVisualBackend
{
public:
  static Scene3DMeshLoadResult load_mesh_resource(const QString & mesh_path, int triangle_limit);
};
}  // namespace workcell_builder

#endif
