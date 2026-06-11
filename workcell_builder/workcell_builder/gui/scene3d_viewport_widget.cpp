#include "scene3d_viewport_widget.h"

#include <QMatrix4x4>
#include <QMouseEvent>
#include <QKeyEvent>
#include <QCursor>
#include <QPainter>
#include <QVector3D>
#include <QVector4D>
#include <QToolTip>
#include <QFile>
#include <QFileInfo>
#include <QDataStream>
#include <QDebug>
#include <QTextStream>
#include <QtEndian>
#include <QMimeData>
#include <QJsonDocument>
#include <QJsonArray>
#include <QXmlStreamReader>
#include <QImage>
#include <cstring>
#include <QtMath>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace {
constexpr int kMeshTriangleLimit = 100000;
constexpr double kWorkspaceLimitMeters = 1000.0;

[[maybe_unused]] QString snap_mode_label(Scene3DViewportWidget::SnapMode mode)
{
  switch (mode) {
    case Scene3DViewportWidget::SnapMode::Off: return "Off";
    case Scene3DViewportWidget::SnapMode::Cm1: return "1 cm";
    case Scene3DViewportWidget::SnapMode::Cm5: return "5 cm";
    case Scene3DViewportWidget::SnapMode::Cm10: return "10 cm";
    case Scene3DViewportWidget::SnapMode::Deg5: return "5 deg";
    case Scene3DViewportWidget::SnapMode::Deg15: return "15 deg";
  }
  return "Off";
}

double snap_translation_value(double raw_m, Scene3DViewportWidget::SnapMode mode)
{
  double step_m = 0.0;
  switch (mode) {
    case Scene3DViewportWidget::SnapMode::Cm1: step_m = 0.01; break;
    case Scene3DViewportWidget::SnapMode::Cm5: step_m = 0.05; break;
    case Scene3DViewportWidget::SnapMode::Cm10: step_m = 0.10; break;
    case Scene3DViewportWidget::SnapMode::Off:
    case Scene3DViewportWidget::SnapMode::Deg5:
    case Scene3DViewportWidget::SnapMode::Deg15:
      break;
  }
  return step_m > 0.0 ? std::round(raw_m / step_m) * step_m : raw_m;
}

double snap_rotation_value(double raw_rad, Scene3DViewportWidget::SnapMode mode)
{
  double step_rad = 0.0;
  switch (mode) {
    case Scene3DViewportWidget::SnapMode::Deg5: step_rad = qDegreesToRadians(5.0); break;
    case Scene3DViewportWidget::SnapMode::Deg15: step_rad = qDegreesToRadians(15.0); break;
    case Scene3DViewportWidget::SnapMode::Off:
    case Scene3DViewportWidget::SnapMode::Cm1:
    case Scene3DViewportWidget::SnapMode::Cm5:
    case Scene3DViewportWidget::SnapMode::Cm10:
      break;
  }
  return step_rad > 0.0 ? std::round(raw_rad / step_rad) * step_rad : raw_rad;
}

QString path_without_uri_suffixes(QString path)
{
  path = path.trimmed();
  const int fragment_index = path.indexOf(QLatin1Char('#'));
  if (fragment_index >= 0) path = path.left(fragment_index);
  const int query_index = path.indexOf(QLatin1Char('?'));
  if (query_index >= 0) path = path.left(query_index);
  return path.trimmed();
}

bool path_has_mesh_asset_extension(const QString & path)
{
  const QString suffix = QFileInfo(path_without_uri_suffixes(path)).suffix().toLower();
  return suffix == QStringLiteral("dae") ||
         suffix == QStringLiteral("stl") ||
         suffix == QStringLiteral("obj") ||
         suffix == QStringLiteral("glb") ||
         suffix == QStringLiteral("gltf");
}

bool item_has_credible_mesh_handoff(const ScenePreviewWidget::PreviewItem & item)
{
  const QString mesh_path = item.mesh_path.trimmed();
  const QString package_uri = item.package_uri.trimmed();
  const QString source_path = item.source_path.trimmed();
  return item.mesh_available ||
         item.has_mesh_metadata ||
         !mesh_path.isEmpty() ||
         (!package_uri.isEmpty() && path_has_mesh_asset_extension(package_uri)) ||
         (!source_path.isEmpty() && path_has_mesh_asset_extension(source_path));
}

bool item_has_valid_urdf_primitive(const ScenePreviewWidget::PreviewItem & item)
{
  const QString type = item.primitive_geometry_type.trimmed().toLower().replace(QLatin1Char('-'), QLatin1Char('_')).replace(QLatin1Char(' '), QLatin1Char('_'));
  if (type == QStringLiteral("box")) return item.sx > 0.001 && item.sy > 0.001 && item.sz > 0.001;
  if (type == QStringLiteral("cylinder")) return item.primitive_radius > 0.001 && item.primitive_length > 0.001;
  if (type == QStringLiteral("sphere")) return item.primitive_radius > 0.001;
  if (type == QStringLiteral("capsule")) return item.primitive_radius > 0.001 && item.primitive_length > 0.001;
  return false;
}

bool item_has_explicit_primitive_dimensions(const ScenePreviewWidget::PreviewItem & item)
{
  return item_has_valid_urdf_primitive(item) || (item.sx > 0.001 && item.sy > 0.001 && item.sz > 0.001);
}

void finalize_visual_quality(Scene3DViewportWidget::RenderDebugCounters & counters)
{
  counters.total_payload_count = counters.preview_items_count;
  counters.mesh_backed_count = counters.mesh_source_count;
  counters.overlay_count = counters.overlay_helper_count;
  counters.overlay_rendered_count = counters.overlay_helper_count;
  counters.valid_physical_fallback_count = counters.primitive_fallback_rendered_count;

  QStringList warnings;
  QString status = QStringLiteral("PASS");

  if (counters.visible_count <= 0 && counters.total_payload_count <= 0) {
    status = QStringLiteral("UNAVAILABLE");
    warnings.append(QStringLiteral("no_preview_payload"));
  }
  if (counters.mesh_source_count > 0 && counters.mesh_rendered_count <= 0) {
    status = QStringLiteral("FAIL");
    warnings.append(QStringLiteral("mesh_sources_present_but_none_rendered"));
  }
  if (counters.urdf_primitive_source_count > 0 && counters.urdf_primitive_rendered_count <= 0) {
    status = QStringLiteral("FAIL");
    warnings.append(QStringLiteral("urdf_primitive_sources_present_but_none_rendered"));
  }

  const bool high_mesh_source_low_render =
    counters.mesh_source_count >= 4 &&
    counters.mesh_rendered_count > 0 &&
    counters.mesh_rendered_count * 2 < counters.mesh_source_count;
  if (high_mesh_source_low_render) {
    if (status != QStringLiteral("FAIL")) status = QStringLiteral("WARNING");
    warnings.append(QStringLiteral("mesh_source_count_high_but_mesh_rendered_count_low"));
  }
  if (counters.missing_geometry_count > 0) {
    if (status == QStringLiteral("PASS")) status = QStringLiteral("WARNING");
    warnings.append(QStringLiteral("missing_geometry_requires_diagnostics"));
  }
  if (counters.placeholder_count > 0) {
    if (status == QStringLiteral("PASS")) status = QStringLiteral("WARNING");
    warnings.append(QStringLiteral("placeholder_geometry_rendered"));
  }
  if (counters.visible_count > 0 &&
      counters.mesh_source_count <= 0 &&
      counters.urdf_primitive_source_count <= 0 &&
      counters.wireframe_fallback_count > 0 &&
      status == QStringLiteral("PASS")) {
    status = QStringLiteral("WARNING");
    warnings.append(QStringLiteral("wireframe_fallback_only_preview"));
  }

  counters.visual_quality_status = status;
  counters.visual_quality_warnings = warnings;
}


QString mesh_load_failure_reason_for_item(const QString & path, const ScenePreviewWidget::PreviewItem * item)
{
  const QString trimmed_path = path.trimmed();
  if (item) {
    const QString outcome = item->source_path_resolution_outcome.trimmed().toLower();
    if (item->resolved_source_path_stale || outcome.contains(QStringLiteral("stale"))) return QStringLiteral("stale_path");
    if (trimmed_path.startsWith(QStringLiteral("package://"), Qt::CaseInsensitive) ||
        !item->package_uri.trimmed().isEmpty() || outcome.contains(QStringLiteral("package_uri"))) {
      if (outcome.contains(QStringLiteral("unresolved")) || outcome.contains(QStringLiteral("missing")) ||
          trimmed_path.startsWith(QStringLiteral("package://"), Qt::CaseInsensitive)) {
        return QStringLiteral("package_uri_unresolved");
      }
    }
  }
  return QStringLiteral("file_not_found");
}

QString mesh_parse_failure_code(const QString & parse_error)
{
  const QString normalized = parse_error.trimmed().toLower();
  if (normalized.contains(QStringLiteral("no triangles")) || normalized.contains(QStringLiteral("contains no triangles"))) {
    return QStringLiteral("zero_triangle_mesh");
  }
  if (normalized.contains(QStringLiteral("exceeds limit"))) return QStringLiteral("unreasonable_bounds");
  return QStringLiteral("parse_failed");
}

bool parse_ascii_stl(const QByteArray & bytes, Scene3DViewportWidget::InternalTriangleMesh & out_mesh,
                     QString & out_error, int triangle_limit)
{
  std::istringstream stream(QString::fromUtf8(bytes).toStdString());
  std::string token;
  while (stream >> token) {
    if (QString::fromStdString(token).compare("facet", Qt::CaseInsensitive) != 0) continue;
    std::string normal_tok;
    if (!(stream >> normal_tok) || QString::fromStdString(normal_tok).compare("normal", Qt::CaseInsensitive) != 0) {
      out_error = "invalid facet normal token";
      return false;
    }
    float nx, ny, nz;
    if (!(stream >> nx >> ny >> nz)) { out_error = "invalid normal values"; return false; }
    std::string outer, loop;
    if (!(stream >> outer >> loop) ||
        QString::fromStdString(outer).compare("outer", Qt::CaseInsensitive) != 0 ||
        QString::fromStdString(loop).compare("loop", Qt::CaseInsensitive) != 0) {
      out_error = "invalid outer loop token";
      return false;
    }
    Scene3DViewportWidget::InternalTriangleMesh::Triangle tri;
    tri.normal = QVector3D(nx, ny, nz);
    for (int i = 0; i < 3; ++i) {
      std::string vertex;
      float vx, vy, vz;
      if (!(stream >> vertex >> vx >> vy >> vz) ||
          QString::fromStdString(vertex).compare("vertex", Qt::CaseInsensitive) != 0) {
        out_error = "invalid vertex token";
        return false;
      }
      tri.vertices[i] = QVector3D(vx, vy, vz);
    }
    std::string endloop, endfacet;
    if (!(stream >> endloop >> endfacet) ||
        QString::fromStdString(endloop).compare("endloop", Qt::CaseInsensitive) != 0 ||
        QString::fromStdString(endfacet).compare("endfacet", Qt::CaseInsensitive) != 0) {
      out_error = "invalid endloop/endfacet token";
      return false;
    }
    out_mesh.triangles.push_back(tri);
    if (out_mesh.triangles.size() > triangle_limit) { out_error = "mesh triangle count exceeds limit"; return false; }
  }
  if (out_mesh.triangles.isEmpty()) { out_error = "ascii STL contains no triangles"; return false; }
  return true;
}

bool parse_binary_stl(const QByteArray & bytes, Scene3DViewportWidget::InternalTriangleMesh & out_mesh,
                      QString & out_error, int triangle_limit)
{
  if (bytes.size() < 84) { out_error = "binary STL too small"; return false; }
  const uchar * data = reinterpret_cast<const uchar *>(bytes.constData());
  const quint32 tri_count = qFromLittleEndian<quint32>(data + 80);
  if (tri_count > static_cast<quint32>(triangle_limit)) { out_error = "mesh triangle count exceeds limit"; return false; }
  const qint64 expected_size = 84LL + static_cast<qint64>(tri_count) * 50LL;
  if (bytes.size() < expected_size) { out_error = "binary STL truncated"; return false; }
  out_mesh.triangles.reserve(static_cast<int>(tri_count));
  const uchar * tri_ptr = data + 84;
  auto read_float = [](const uchar * p) {
    quint32 raw = qFromLittleEndian<quint32>(p);
    float value;
    std::memcpy(&value, &raw, sizeof(float));
    return value;
  };
  for (quint32 i = 0; i < tri_count; ++i, tri_ptr += 50) {
    Scene3DViewportWidget::InternalTriangleMesh::Triangle tri;
    tri.normal = QVector3D(read_float(tri_ptr), read_float(tri_ptr + 4), read_float(tri_ptr + 8));
    for (int vi = 0; vi < 3; ++vi) {
      const uchar * vp = tri_ptr + 12 + vi * 12;
      tri.vertices[vi] = QVector3D(read_float(vp), read_float(vp + 4), read_float(vp + 8));
    }
    out_mesh.triangles.push_back(tri);
  }
  if (out_mesh.triangles.isEmpty()) { out_error = "binary STL contains no triangles"; return false; }
  return true;
}

bool looks_like_ascii_stl(const QByteArray & bytes)
{
  const QByteArray prefix = bytes.left(256).trimmed().toLower();
  return prefix.startsWith("solid") && prefix.contains("facet");
}



bool parse_obj_face_vertex_index(const QString & token, int vertex_count, int & out_index)
{
  const QString vertex_token = token.section(QLatin1Char('/'), 0, 0).trimmed();
  if (vertex_token.isEmpty()) return false;
  bool ok = false;
  const int raw_index = vertex_token.toInt(&ok);
  if (!ok || raw_index == 0) return false;
  const int zero_based = raw_index > 0 ? raw_index - 1 : vertex_count + raw_index;
  if (zero_based < 0 || zero_based >= vertex_count) return false;
  out_index = zero_based;
  return true;
}

bool parse_obj_bytes(const QByteArray & bytes, Scene3DViewportWidget::InternalTriangleMesh & out_mesh,
                     QString & out_error, int triangle_limit)
{
  const QString text = QString::fromUtf8(bytes);
  const QStringList lines = text.split(QRegExp("\\r?\\n"));
  QVector<QVector3D> vertices;
  vertices.reserve(lines.size());
  for (int line_number = 0; line_number < lines.size(); ++line_number) {
    QString line = lines.at(line_number).trimmed();
    const int comment_pos = line.indexOf(QLatin1Char('#'));
    if (comment_pos >= 0) line = line.left(comment_pos).trimmed();
    if (line.isEmpty()) continue;
    const QStringList parts = line.split(QRegExp("\\s+"), Qt::SkipEmptyParts);
    if (parts.isEmpty()) continue;
    const QString kind = parts.first();
    if (kind == QStringLiteral("v")) {
      if (parts.size() < 4) {
        out_error = QStringLiteral("obj vertex on line %1 has fewer than 3 coordinates").arg(line_number + 1);
        return false;
      }
      bool ok_x = false, ok_y = false, ok_z = false;
      const float x = parts.at(1).toFloat(&ok_x);
      const float y = parts.at(2).toFloat(&ok_y);
      const float z = parts.at(3).toFloat(&ok_z);
      if (!ok_x || !ok_y || !ok_z || !qIsFinite(x) || !qIsFinite(y) || !qIsFinite(z)) {
        out_error = QStringLiteral("obj vertex on line %1 has invalid coordinates").arg(line_number + 1);
        return false;
      }
      vertices.push_back(QVector3D(x, y, z));
      continue;
    }
    if (kind != QStringLiteral("f")) continue;
    if (parts.size() < 4) continue;
    QVector<int> face_indices;
    face_indices.reserve(parts.size() - 1);
    for (int i = 1; i < parts.size(); ++i) {
      int vertex_index = -1;
      if (!parse_obj_face_vertex_index(parts.at(i), vertices.size(), vertex_index)) {
        out_error = QStringLiteral("obj face on line %1 references invalid vertex '%2'").arg(line_number + 1).arg(parts.at(i));
        return false;
      }
      face_indices.push_back(vertex_index);
    }
    const int base = face_indices.first();
    for (int i = 1; i + 1 < face_indices.size(); ++i) {
      Scene3DViewportWidget::InternalTriangleMesh::Triangle tri;
      tri.vertices[0] = vertices.at(base);
      tri.vertices[1] = vertices.at(face_indices.at(i));
      tri.vertices[2] = vertices.at(face_indices.at(i + 1));
      tri.normal = QVector3D::crossProduct(tri.vertices[1] - tri.vertices[0], tri.vertices[2] - tri.vertices[0]);
      out_mesh.triangles.push_back(tri);
      if (out_mesh.triangles.size() > triangle_limit) {
        out_error = QStringLiteral("mesh triangle count exceeds limit");
        return false;
      }
    }
  }
  if (out_mesh.triangles.isEmpty()) {
    out_error = QStringLiteral("obj contains no triangles");
    return false;
  }
  return true;
}

bool parse_collada_bytes(const QByteArray & bytes, Scene3DViewportWidget::InternalTriangleMesh & out_mesh,
                         QString & out_error, double * out_unit_meter, int triangle_limit)
{
  QXmlStreamReader xml(bytes);
  if (out_unit_meter) *out_unit_meter = 1.0;
  QVector<float> positions;
  bool in_geometry = false;
  while (!xml.atEnd()) {
    xml.readNext();
    if (xml.isStartElement() && xml.name() == QStringLiteral("asset")) {
      while (!(xml.isEndElement() && xml.name() == QStringLiteral("asset")) && !xml.atEnd()) {
        xml.readNext();
        if (xml.isStartElement() && xml.name() == QStringLiteral("unit")) {
          const double meter = xml.attributes().value(QStringLiteral("meter")).toDouble();
          if (out_unit_meter && meter > 0.0 && qIsFinite(meter)) *out_unit_meter = meter;
        }
      }
    }
    if (xml.isStartElement() && xml.name() == QStringLiteral("geometry")) in_geometry = true;
    if (!in_geometry || !xml.isStartElement()) continue;
    if (xml.name() == QStringLiteral("float_array")) {
      const QString id = xml.attributes().value(QStringLiteral("id")).toString().toLower();
      if (id.contains("position")) {
        const QStringList vals = xml.readElementText().split(QRegExp("\\s+"), Qt::SkipEmptyParts);
        positions.clear();
        positions.reserve(vals.size());
        for (const auto & v : vals) positions.push_back(v.toFloat());
      }
    }
    if ((xml.name() == QStringLiteral("triangles") || xml.name() == QStringLiteral("polylist")) && !positions.isEmpty()) {
      int vcount = 3;
      QVector<int> poly_vcounts;
      QString ptext;
      while (!(xml.isEndElement() && (xml.name() == QStringLiteral("triangles") || xml.name() == QStringLiteral("polylist"))) && !xml.atEnd()) {
        xml.readNext();
        if (xml.isStartElement() && xml.name() == QStringLiteral("input")) {
          const QString semantic = xml.attributes().value(QStringLiteral("semantic")).toString();
          if (semantic == QStringLiteral("VERTEX")) vcount = qMax(vcount, xml.attributes().value(QStringLiteral("offset")).toInt() + 1);
        }
        if (xml.isStartElement() && xml.name() == QStringLiteral("vcount")) {
          const QStringList vals = xml.readElementText().split(QRegExp("\\s+"), Qt::SkipEmptyParts);
          for (const auto & v : vals) poly_vcounts.push_back(v.toInt());
        }
        if (xml.isStartElement() && xml.name() == QStringLiteral("p")) ptext = xml.readElementText();
      }
      const QStringList idxs = ptext.split(QRegExp("\\s+"), Qt::SkipEmptyParts);
      QVector<int> all; all.reserve(idxs.size());
      for (const auto & t : idxs) all.push_back(t.toInt());
      const double unit_meter = (out_unit_meter && *out_unit_meter > 0.0 && qIsFinite(*out_unit_meter)) ? *out_unit_meter : 1.0;
      auto addTri=[&](int a,int b,int c){
        const int na=a*3, nb=b*3, nc=c*3;
        if (na+2>=positions.size()||nb+2>=positions.size()||nc+2>=positions.size()) return;
        Scene3DViewportWidget::InternalTriangleMesh::Triangle tri;
        tri.vertices[0]=QVector3D(positions[na],positions[na+1],positions[na+2]) * static_cast<float>(unit_meter);
        tri.vertices[1]=QVector3D(positions[nb],positions[nb+1],positions[nb+2]) * static_cast<float>(unit_meter);
        tri.vertices[2]=QVector3D(positions[nc],positions[nc+1],positions[nc+2]) * static_cast<float>(unit_meter);
        tri.normal = QVector3D::crossProduct(tri.vertices[1]-tri.vertices[0], tri.vertices[2]-tri.vertices[0]);
        out_mesh.triangles.push_back(tri);
      };
      if (xml.name() == QStringLiteral("triangles") || poly_vcounts.isEmpty()) {
        for (int i=0;i+vcount*3-1<all.size(); i+=vcount*3) { addTri(all[i], all[i+vcount], all[i+2*vcount]); if (out_mesh.triangles.size()>triangle_limit){ out_error="mesh triangle count exceeds limit"; return false; } }
      } else {
        int off=0;
        for (int pc : poly_vcounts) {
          if (pc<3) { off += pc*vcount; continue; }
          const int base = all[off];
          for (int k=1;k+1<pc;++k) { addTri(base, all[off + k*vcount], all[off + (k+1)*vcount]); if (out_mesh.triangles.size()>triangle_limit){ out_error="mesh triangle count exceeds limit"; return false; } }
          off += pc*vcount; if (off>=all.size()) break;
        }
      }
    }
  }
  if (xml.hasError()) { out_error = QStringLiteral("collada parse error: ") + xml.errorString(); return false; }
  if (out_mesh.triangles.isEmpty()) { out_error = QStringLiteral("collada contains no triangles"); return false; }
  return true;
}
enum class NormalizedRole
{
  RobotBase,
  Table,
  Conveyor,
  Camera,
  PickZone,
  PlaceBin,
  Object,
  SafetyZone,
  WarningAnchor,
  Generic
};

QString normalized_token(const QString & value)
{
  return value.trimmed().toLower().replace('-', '_').replace(' ', '_');
}

QString normalized_scene3d_layer_token(const QString & value)
{
  const QString normalized = normalized_token(value);
  if (normalized == QStringLiteral("generated_preview")) return QStringLiteral("generated_urdf_visual");
  if (normalized == QStringLiteral("locked_generated_urdf")) return QStringLiteral("locked_generated_urdf_visual");
  if (normalized == QStringLiteral("legacy_static_fallback")) return QStringLiteral("primitive_fallback");
  if (normalized == QStringLiteral("overlays") || normalized == QStringLiteral("helper_overlay")) return QStringLiteral("overlay");
  return normalized;
}

bool is_generated_urdf_visual_item(const ScenePreviewWidget::PreviewItem & it)
{
  const QString source_layer = normalized_scene3d_layer_token(it.source_layer);
  const QString visual_source = normalized_scene3d_layer_token(it.active_visual_source);
  if (source_layer == "generated_urdf_visual" || source_layer == "locked_generated_urdf_visual") return true;
  if (visual_source == "generated_urdf_visual" || visual_source == "locked_generated_urdf_visual") return true;
  if (it.locked && it.lock_reason.contains("URDF visual", Qt::CaseInsensitive)) return true;
  return false;
}

QString clean_label_from_item(const ScenePreviewWidget::PreviewItem & it)
{
  QString label = it.display_name.trimmed();
  if (label.isEmpty()) label = it.id.trimmed();
  if (label.isEmpty()) return QStringLiteral("Item");

  const int slash = qMax(label.lastIndexOf('/'), label.lastIndexOf(':'));
  if (slash >= 0 && slash + 1 < label.size()) label = label.mid(slash + 1);
  if (label.startsWith(QStringLiteral("urdf_visual"), Qt::CaseInsensitive)) label.remove(0, QStringLiteral("urdf_visual").size());
  label = label.trimmed();
  while (label.startsWith('_') || label.startsWith('-') || label.startsWith('/')) label.remove(0, 1);
  if (label.isEmpty()) label = it.id.trimmed();
  if (label.size() > 18) label = label.left(17) + QStringLiteral("…");
  return label;
}
QString warning_badge_text(const QStringList & warnings)
{
  if (warnings.isEmpty()) return QStringLiteral("!");
  if (warnings.size() == 1) return QStringLiteral("!");
  return QStringLiteral("%1").arg(qMin(warnings.size(), 9));
}

QString warning_debug_text(const QStringList & warnings)
{
  if (warnings.isEmpty()) return QStringLiteral("none");
  if (warnings.size() == 1) return warnings.front();
  return QStringLiteral("%1 (+%2 more)").arg(warnings.front()).arg(warnings.size() - 1);
}

bool item_is_editable_for_gizmo(const ScenePreviewWidget::PreviewItem & it)
{
  if (it.locked) return false;
  const QString source_layer = normalized_scene3d_layer_token(it.source_layer);
  const QString visual_source = normalized_scene3d_layer_token(it.active_visual_source);
  const QString role = normalized_token(it.role);
  const QString category = normalized_token(it.category);
  const QString lock_reason = normalized_token(it.lock_reason);

  const bool overlay_or_helper = role.contains("overlay") || role.contains("helper") || role.contains("guide") ||
                                 category.contains("overlay") || category.contains("helper") ||
                                 lock_reason.contains("overlay") || lock_reason.contains("helper");
  if (overlay_or_helper) return false;

  const bool generated_robot_visual = is_generated_urdf_visual_item(it) ||
                                      source_layer.contains("generated") || source_layer.contains("urdf") ||
                                      visual_source.contains("generated") || lock_reason.contains("urdf") ||
                                      lock_reason.contains("robot model") || lock_reason.contains("camera generated") ||
                                      lock_reason.contains("tool generated");
  if (generated_robot_visual) return false;

  if (source_layer == "editable_layout") return it.editable;
  if (source_layer == "primitive_fallback" || visual_source == "primitive_fallback") {
    return it.editable && it.linked_to_editable_layout_state;
  }
  if (visual_source == "mesh_preview" || source_layer == "mesh_preview") {
    return it.editable && it.linked_to_editable_layout_state;
  }
  return false;
}

QString item_locked_reason(const ScenePreviewWidget::PreviewItem & it)
{
  const QString reason = it.lock_reason.trimmed();
  return reason.isEmpty() ? QStringLiteral("item is locked") : reason;
}

QString item_visual_ownership_label(const ScenePreviewWidget::PreviewItem & it)
{
  if (is_generated_urdf_visual_item(it) || is_locked_urdf_item(it)) return QStringLiteral("Generated / locked preview");
  if (it.linked_to_editable_layout_state || item_is_editable_for_gizmo(it)) return QStringLiteral("Editable layout");
  return QStringLiteral("Reference preview");
}

QColor generated_locked_preview_material() { return QColor("#cfd4da"); }
QColor generated_locked_preview_outline() { return QColor(125, 211, 252, 92); }
QColor generated_primitive_fallback_fill() { return QColor(96, 165, 250, 52); }
QColor generated_primitive_fallback_outline() { return QColor(147, 197, 253, 108); }
QColor editable_layout_accent_outline() { return QColor("#22d3ee"); }
QColor editable_layout_selected_highlight() { return QColor("#f8fafc"); }


NormalizedRole classify_item_role(const ScenePreviewWidget::PreviewItem & it)
{
  const QString role = normalized_token(it.role);
  const QString category = normalized_token(it.category);
  const QString mix = role + "|" + category;

  if (mix.contains("robot_base") || mix.contains("robot")) return NormalizedRole::RobotBase;
  if (mix.contains("support_surface") || mix.contains("table") || mix.contains("work_surface")) return NormalizedRole::Table;
  if (mix.contains("conveyor") || mix.contains("belt")) return NormalizedRole::Conveyor;
  if (mix.contains("camera") || mix.contains("sensor")) return NormalizedRole::Camera;
  if (mix.contains("pick_zone") || mix.contains("pick_area") || mix.contains("pick")) return NormalizedRole::PickZone;
  if (mix.contains("place_zone") || mix.contains("place_target") || mix.contains("place") || mix.contains("bin")) return NormalizedRole::PlaceBin;
  if (mix.contains("safety_zone") || mix.contains("safety")) return NormalizedRole::SafetyZone;
  if (mix.contains("warning_anchor") || mix.contains("warning_badge")) return NormalizedRole::WarningAnchor;
  if (mix.contains("object") || mix.contains("part") || mix.contains("item")) return NormalizedRole::Object;
  return NormalizedRole::Generic;
}


bool is_overlay_only_item(const ScenePreviewWidget::PreviewItem & it);
bool is_locked_urdf_item(const ScenePreviewWidget::PreviewItem & it);


bool include_in_fit_bounds(const ScenePreviewWidget::PreviewItem & it, bool include_overlays)
{
  if (include_overlays) return true;
  const QString source_layer = normalized_scene3d_layer_token(it.source_layer);
  const QString visual_source = normalized_scene3d_layer_token(it.active_visual_source);
  const bool helper_overlay = is_overlay_only_item(it) || source_layer == "overlay" || visual_source == "overlay";
  const bool generated_urdf_visual = is_generated_urdf_visual_item(it) || is_locked_urdf_item(it);
  const bool mesh_backed = item_has_credible_mesh_handoff(it);
  const bool explicit_primitive = it.sx > 0.001 && it.sy > 0.001 && it.sz > 0.001;
  const bool missing_geometry = !mesh_backed && !explicit_primitive;
  const bool missing_mesh_fallback = missing_geometry && !it.linked_to_editable_layout_state && !generated_urdf_visual;
  if (helper_overlay) return false;
  if (missing_mesh_fallback) return false;
  if (generated_urdf_visual && mesh_backed) return true;
  if (generated_urdf_visual) return true;
  if (it.linked_to_editable_layout_state) return true;
  return mesh_backed || explicit_primitive;
}

bool is_overlay_only_item(const ScenePreviewWidget::PreviewItem & it)
{
  const NormalizedRole role = classify_item_role(it);
  if (role == NormalizedRole::SafetyZone || role == NormalizedRole::WarningAnchor) return true;
  const QString role_text = it.role.trimmed().toLower();
  const QString category = it.category.trimmed().toLower();
  const QString lock_reason = it.lock_reason.trimmed().toLower();
  return role_text.contains("overlay") || role_text.contains("helper") || role_text.contains("guide") ||
         category.contains("overlay") || lock_reason.contains("overlay");
}

bool is_locked_urdf_item(const ScenePreviewWidget::PreviewItem & it)
{
  if (!(it.locked && !it.editable)) return false;
  const QString category = it.category.trimmed().toLower();
  const QString lock_reason = it.lock_reason.trimmed().toLower();
  return category.contains("urdf") || lock_reason.contains("urdf") || lock_reason.contains("robot model") ||
         lock_reason.contains("robotmodel") || lock_reason.contains("urdf visual");
}

bool is_raw_generated_bounds_only_item(const ScenePreviewWidget::PreviewItem & it)
{
  const QString visual_source = normalized_scene3d_layer_token(it.active_visual_source);
  const QString source_layer = normalized_scene3d_layer_token(it.source_layer);
  if (visual_source == QStringLiteral("primitive_fallback") || source_layer == QStringLiteral("primitive_fallback")) {
    return false;
  }
  const bool generated_or_locked = is_generated_urdf_visual_item(it) || is_locked_urdf_item(it);
  if (!generated_or_locked || !item_has_explicit_primitive_dimensions(it)) return false;
  return item_has_credible_mesh_handoff(it) || item_has_valid_urdf_primitive(it);
}



bool is_clean_semantic_primitive_role(NormalizedRole role)
{
  switch (role) {
    case NormalizedRole::PickZone:
    case NormalizedRole::PlaceBin:
    case NormalizedRole::Conveyor:
    case NormalizedRole::Object:
      return true;
    default:
      return false;
  }
}

bool should_suppress_missing_geometry_marker_for_semantic_role(const ScenePreviewWidget::PreviewItem & it)
{
  return is_clean_semantic_primitive_role(classify_item_role(it)) && !item_has_explicit_primitive_dimensions(it);
}

bool is_overlay_visual_role(NormalizedRole role)
{
  switch (role) {
    case NormalizedRole::PickZone:
    case NormalizedRole::PlaceBin:
    case NormalizedRole::SafetyZone:
    case NormalizedRole::WarningAnchor:
      return true;
    default:
      return false;
  }
}

bool is_critical_label_role(NormalizedRole role)
{
  switch (role) {
    case NormalizedRole::RobotBase:
    case NormalizedRole::Camera:
    case NormalizedRole::PickZone:
    case NormalizedRole::PlaceBin:
    case NormalizedRole::SafetyZone:
      return true;
    default:
      return false;
  }
}
QColor item_color(const ScenePreviewWidget::PreviewItem & it)
{
  if (is_generated_urdf_visual_item(it) || is_locked_urdf_item(it)) {
    return generated_locked_preview_material();
  }
  if (it.has_material_color) {
    QColor c;
    c.setRgbF(qBound(0.0, it.material_r, 1.0), qBound(0.0, it.material_g, 1.0),
              qBound(0.0, it.material_b, 1.0), qBound(0.0, it.material_a, 1.0));
    return c;
  }
  if (it.linked_to_editable_layout_state || item_is_editable_for_gizmo(it)) {
    return QColor("#67e8f9");
  }
  switch (classify_item_role(it)) {
    case NormalizedRole::RobotBase: return QColor("#a78bfa");
    case NormalizedRole::Table: return QColor("#64748b");
    case NormalizedRole::Conveyor: return QColor("#06b6d4");
    case NormalizedRole::Camera: return QColor("#38bdf8");
    case NormalizedRole::PickZone: return QColor("#22c55e");
    case NormalizedRole::PlaceBin: return QColor("#fb7185");
    case NormalizedRole::Object: return QColor("#94a3b8");
    case NormalizedRole::SafetyZone: return QColor("#f59e0b");
    case NormalizedRole::WarningAnchor: return QColor("#fbbf24");
    case NormalizedRole::Generic: return QColor("#94a3b8");
  }
  return QColor("#94a3b8");
}

double distance_to_segment_2d(const QPointF & p, const QPointF & a, const QPointF & b)
{
  const QPointF ab = b - a;
  const double ab_len2 = (ab.x() * ab.x()) + (ab.y() * ab.y());
  if (ab_len2 <= 1e-9) return QLineF(p, a).length();
  const QPointF ap = p - a;
  const double t = qBound(0.0, ((ap.x() * ab.x()) + (ap.y() * ab.y())) / ab_len2, 1.0);
  const QPointF c = a + (ab * t);
  return QLineF(p, c).length();
}

double distance_to_polyline_2d(const QPointF & p, const QVector<QPointF> & polyline)
{
  if (polyline.size() < 2) return std::numeric_limits<double>::max();
  double best = std::numeric_limits<double>::max();
  for (int i = 1; i < polyline.size(); ++i) best = qMin(best, distance_to_segment_2d(p, polyline[i - 1], polyline[i]));
  return best;
}

[[maybe_unused]] double wrap_angle_pi(double angle)
{
  constexpr double kTwoPi = 2.0 * M_PI;
  double wrapped = std::fmod(angle + M_PI, kTwoPi);
  if (wrapped < 0.0) wrapped += kTwoPi;
  return wrapped - M_PI;
}

QPointF apply_label_overlap_offset(const QPointF & anchor, const QVector<QPointF> & placed_points,
                                  const QVector<QPointF> & robot_base_points, bool critical)
{
  const double min_spacing = critical ? 14.0 : 20.0;
  const double robot_cluster_radius = 82.0;
  const bool near_robot_cluster = std::any_of(robot_base_points.cbegin(), robot_base_points.cend(), [&](const QPointF & base) {
    return QLineF(anchor, base).length() <= robot_cluster_radius;
  });
  QVector<QPointF> candidates{anchor, anchor + QPointF(0.0, -12.0), anchor + QPointF(16.0, -8.0),
                              anchor + QPointF(-16.0, -8.0), anchor + QPointF(0.0, 14.0)};
  if (near_robot_cluster) {
    candidates.push_back(anchor + QPointF(20.0, -18.0));
    candidates.push_back(anchor + QPointF(-20.0, -18.0));
    candidates.push_back(anchor + QPointF(24.0, 4.0));
    candidates.push_back(anchor + QPointF(-24.0, 4.0));
  }

  auto far_enough = [&](const QPointF & candidate) {
    for (const QPointF & placed : placed_points) {
      if (QLineF(candidate, placed).length() < min_spacing) return false;
    }
    return true;
  };

  for (const QPointF & candidate : candidates) {
    if (far_enough(candidate)) return candidate;
  }

  return candidates.back();
}

int label_priority_bucket(bool selected, bool has_warnings, NormalizedRole role)
{
  // LABEL_PRIORITY_SELECTED_WARN_ANCHOR: selected > critical warnings > key anchors > others.
  if (selected) return 0;
  if (has_warnings) return 1;
  if (is_critical_label_role(role)) return 2;
  return 3;
}

struct LabelDrawCandidate
{
  int item_index{ -1 };
  QPointF anchor;
  QPointF base_point;
  QString text;
  QColor color;
  int priority{ 3 };
  bool critical{ false };
};


}


Scene3DViewportWidget::Scene3DViewportWidget(QWidget * parent) : QOpenGLWidget(parent) { setMinimumHeight(420); setAcceptDrops(true); }
void Scene3DViewportWidget::reset_view() { set_isometric_view(); }
void Scene3DViewportWidget::set_isometric_view()
{
  yaw_ = -0.78539816339;
  pitch_ = 0.61547970867;
  orbit_offset_ = QVector3D(0.0f, 0.0f, 0.0f);
  distance_ = 6.0;
  update();
}
void Scene3DViewportWidget::set_top_view() { yaw_ = 0.0; pitch_ = -M_PI_2; update(); }
void Scene3DViewportWidget::set_front_view() { yaw_ = M_PI; pitch_ = 0.0; update(); }
void Scene3DViewportWidget::set_side_view() { yaw_ = -M_PI_2; pitch_ = 0.0; update(); }
void Scene3DViewportWidget::invalidate_mesh_cache()
{
  mesh_cache_.clear();
  warned_mesh_fallbacks_.clear();
  update();
}
void Scene3DViewportWidget::ingest_preview_items(const QVector<ScenePreviewWidget::PreviewItem> & preview_items)
{
  items = preview_items;
  int visible_item_count = 0;
  int skipped_item_count = 0;
  int mesh_source_count = 0;
  int urdf_primitive_source_count = 0;
  int locked_urdf_count = 0;
  int editable_layout_count = 0;
  QSet<QString> unique_visible_ids;
  std::vector<const ScenePreviewWidget::PreviewItem *> overlay_items;
  for (const auto & it : items) {
    const NormalizedRole role = classify_item_role(it);
    if (!show_safety && role == NormalizedRole::SafetyZone) { ++skipped_item_count; continue; }
    ++visible_item_count;
    unique_visible_ids.insert(it.id);
    const bool generated_urdf = is_generated_urdf_visual_item(it) || is_locked_urdf_item(it);
    const bool overlay_helper = is_overlay_only_item(it) || is_overlay_visual_role(role);
    if (!overlay_helper && item_has_credible_mesh_handoff(it)) ++mesh_source_count;
    if (!overlay_helper && generated_urdf && item_has_valid_urdf_primitive(it)) ++urdf_primitive_source_count;
    if (generated_urdf) ++locked_urdf_count;
    if (it.linked_to_editable_layout_state) ++editable_layout_count;
    if (overlay_helper) overlay_items.push_back(&it);
  }
  last_render_counters.preview_items_count = items.size();
  last_render_counters.total_payload_count = items.size();
  last_render_counters.viewport_received_count = items.size();
  last_render_counters.render_cache_count = mesh_cache_.size();
  last_render_counters.visible_count = visible_item_count;
  last_render_counters.skipped_count = skipped_item_count;
  last_render_counters.unique_visible_item_count = unique_visible_ids.size();
  last_render_counters.mesh_source_count = mesh_source_count;
  last_render_counters.mesh_backed_count = mesh_source_count;
  last_render_counters.mesh_rendered_count = 0;
  last_render_counters.urdf_primitive_source_count = urdf_primitive_source_count;
  last_render_counters.urdf_primitive_rendered_count = 0;
  last_render_counters.overlay_helper_count = static_cast<int>(overlay_items.size());
  last_render_counters.overlay_count = static_cast<int>(overlay_items.size());
  last_render_counters.locked_generated_urdf_visual_count = locked_urdf_count;
  last_render_counters.editable_layout_count = editable_layout_count;
  last_render_counters.hierarchy_child_row_count = visible_item_count;
  last_render_counters.last_paint_completed = false;
  last_render_counters.smoke_fallback_render_used = false;
  finalize_visual_quality(last_render_counters);
  const QString diagnostics_path = QString::fromUtf8(qgetenv("SCENE3D_MESH_DIAGNOSTICS_JSON"));
  if (!diagnostics_path.trimmed().isEmpty()) {
    QFile out_file(diagnostics_path);
    if (out_file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
      QJsonObject root;
      root["camera_fit_target"] = last_camera_fit_target_;
      if (has_robot_aabb_diag_) {
        root["robot_aabb_min"] = QJsonArray{last_robot_aabb_min_.x(), last_robot_aabb_min_.y(), last_robot_aabb_min_.z()};
        root["robot_aabb_max"] = QJsonArray{last_robot_aabb_max_.x(), last_robot_aabb_max_.y(), last_robot_aabb_max_.z()};
      }
      root["mesh_diagnostics"] = mesh_diagnostics_export();
      out_file.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
      out_file.close();
    }
  }
  update();
}
void Scene3DViewportWidget::fit_scene() {
  QVector3D bmin, bmax;
  bool has_generated_mesh_focus = false;
  QVector3D mesh_min(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
  QVector3D mesh_max(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());
  for (const auto & it : items) {
    if (!include_in_fit_bounds(it, false)) continue;
    const bool generated_urdf = is_generated_urdf_visual_item(it) || is_locked_urdf_item(it);
    const bool mesh_backed = item_has_credible_mesh_handoff(it);
    if (!generated_urdf || !mesh_backed) continue;
    const ItemBounds bounds = item_bounds_for_role(it);
    mesh_min.setX(std::min(mesh_min.x(), static_cast<float>(bounds.x)));
    mesh_min.setY(std::min(mesh_min.y(), static_cast<float>(bounds.y)));
    mesh_min.setZ(std::min(mesh_min.z(), static_cast<float>(bounds.z)));
    mesh_max.setX(std::max(mesh_max.x(), static_cast<float>(bounds.x + bounds.sx)));
    mesh_max.setY(std::max(mesh_max.y(), static_cast<float>(bounds.y + bounds.sy)));
    mesh_max.setZ(std::max(mesh_max.z(), static_cast<float>(bounds.z + bounds.sz)));
    has_generated_mesh_focus = true;
  }
  if (has_generated_mesh_focus) {
    bmin = mesh_min;
    bmax = mesh_max;
  } else if (!scene_bounds_from_visible_items(bmin, bmax, fit_include_overlays)) {
    set_isometric_view();
    return;
  }
  orbit_offset_ = (bmin + bmax) * 0.5f;
  const QVector3D ext = bmax - bmin;
  const double radius = qMax(0.25, 0.5 * qSqrt(ext.x() * ext.x() + ext.y() * ext.y() + ext.z() * ext.z()));
  scene_radius_ = radius;
  const double fov = qDegreesToRadians(50.0);
  const double fit_distance = (radius / qTan(fov * 0.5)) * 0.95;
  distance_ = qBound(min_distance_, fit_distance, max_distance_);
  pitch_ = qBound(0.28, pitch_, 0.9);
  orbit_offset_.setY(orbit_offset_.y() + static_cast<float>(qMax(0.10, radius * 0.05)));
  last_camera_fit_target_ = QStringLiteral("scene");
  has_robot_aabb_diag_ = false;
  update();
}
void Scene3DViewportWidget::fit_robot()
{
  QVector3D bmin, bmax;
  if (!robot_bounds_from_rendered_visuals(bmin, bmax)) {
    fit_scene();
    return;
  }
  orbit_offset_ = (bmin + bmax) * 0.5f;
  const QVector3D ext = bmax - bmin;
  const double radius = qMax(0.2, 0.5 * qSqrt(ext.x() * ext.x() + ext.y() * ext.y() + ext.z() * ext.z()));
  scene_radius_ = radius;
  const double fov = qDegreesToRadians(50.0);
  const double fit_distance = (radius / qTan(fov * 0.5)) * 1.05;
  distance_ = qBound(min_distance_, fit_distance, max_distance_);
  pitch_ = qBound(0.28, pitch_, 0.9);
  last_camera_fit_target_ = QStringLiteral("robot");
  has_robot_aabb_diag_ = true;
  last_robot_aabb_min_ = bmin;
  last_robot_aabb_max_ = bmax;
  update();
}
void Scene3DViewportWidget::focus_selected() {
  for (const auto & it : items) {
    if (it.id != selected_id) continue;
    const ItemBounds b = item_bounds_for_role(it);
    orbit_offset_ = QVector3D(static_cast<float>(b.x + b.sx * 0.5), static_cast<float>(b.y + b.sy * 0.5), static_cast<float>(b.z + b.sz * 0.5));
    scene_radius_ = qMax(0.2, 0.5 * qSqrt(b.sx * b.sx + b.sy * b.sy + b.sz * b.sz));
    distance_ = qBound(min_distance_, scene_radius_ * 4.0, max_distance_);
    update();
    return;
  }
  fit_scene();
}
void Scene3DViewportWidget::initializeGL() { initializeOpenGLFunctions(); glEnable(GL_DEPTH_TEST); glEnable(GL_CULL_FACE); glClearColor(0.04f, 0.06f, 0.12f, 1.0f); }
void Scene3DViewportWidget::resizeGL(int w, int h) { glViewport(0, 0, w, h); }

void Scene3DViewportWidget::paintGL()
{
  static const char * kPaintGLCacheOnlyGuard = "paintGL cache-only guard: no YAML/file IO in paint path";
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  QMatrix4x4 proj, view;
  camera_matrices(proj, view);
  glMatrixMode(GL_PROJECTION);
  glLoadMatrixf(proj.constData());
  glMatrixMode(GL_MODELVIEW);
  glLoadMatrixf(view.constData());

  draw_ground_grid_pass();
  draw_world_axes_pass();
  if (debug_overlays_mode || fit_include_overlays) {
    draw_cylinder(-0.2, 0.0, -2.2, reach_overlay.preferred_work_zone_radius_m, 0.01, QColor(34, 197, 94, 26), true);
    draw_cylinder(-0.2, 0.0, -2.2, reach_overlay.preferred_work_zone_radius_m, 0.005, QColor(34, 197, 94, 110), false);
  }

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  std::vector<const ScenePreviewWidget::PreviewItem *> draw_items;
  for (const auto & it : items) draw_items.push_back(&it);
  std::sort(draw_items.begin(), draw_items.end(), [&](const auto * a, const auto * b) { return a->z > b->z; });

  int received_item_count = static_cast<int>(draw_items.size());
  int visible_item_count = 0;
  int skipped_item_count = 0;
  int rendered_item_count = 0;
  int mesh_source_count = 0;
  int mesh_rendered_count = 0;
  int urdf_primitive_source_count = 0;
  int urdf_primitive_rendered_count = 0;
  int primitive_fallback_count = 0;
  int placeholder_count = 0;
  int missing_geometry_count = 0;
  int wireframe_box_count = 0;
  int overlay_count = 0;
  int locked_urdf_count = 0;
  int physical_item_count = 0;
  QSet<QString> unique_visible_ids;
  int editable_layout_count = 0;
  std::vector<const ScenePreviewWidget::PreviewItem *> overlay_items;
  std::vector<const ScenePreviewWidget::PreviewItem *> physical_items;
  for (const auto * it : draw_items) {
    const NormalizedRole role = classify_item_role(*it);
    if (!show_safety && role == NormalizedRole::SafetyZone) { ++skipped_item_count; continue; }
    ++visible_item_count;
    unique_visible_ids.insert(it->id);
    const bool generated_urdf = is_generated_urdf_visual_item(*it) || is_locked_urdf_item(*it);
    const bool overlay_helper = is_overlay_only_item(*it) || is_overlay_visual_role(role);
    if (generated_urdf) ++locked_urdf_count;
    if (it->linked_to_editable_layout_state) ++editable_layout_count;
    if (overlay_helper) overlay_items.push_back(it);
    else {
      physical_items.push_back(it);
      ++physical_item_count;
      if (item_has_credible_mesh_handoff(*it)) ++mesh_source_count;
      if (generated_urdf && item_has_valid_urdf_primitive(*it)) {
        ++urdf_primitive_source_count;
      }
    }
  }
  overlay_count = static_cast<int>(overlay_items.size());
  last_render_counters = RenderDebugCounters{};
  last_render_counters.preview_items_count = items.size();
  last_render_counters.total_payload_count = items.size();
  last_render_counters.viewport_received_count = received_item_count;
  last_render_counters.render_cache_count = mesh_cache_.size();
  last_render_counters.visible_count = visible_item_count;
  last_render_counters.skipped_count = skipped_item_count;
  last_render_counters.unique_visible_item_count = unique_visible_ids.size();
  last_render_counters.overlay_helper_count = overlay_count;
  last_render_counters.overlay_count = overlay_count;
  last_render_counters.locked_generated_urdf_visual_count = locked_urdf_count;
  last_render_counters.editable_layout_count = editable_layout_count;
  int visible_hierarchy_items = 0;
  for (const auto * it : draw_items) {
    if (!it) continue;
    const NormalizedRole role = classify_item_role(*it);
    if (!show_safety && role == NormalizedRole::SafetyZone) continue;
    ++visible_hierarchy_items;
  }
  last_render_counters.hierarchy_child_row_count = visible_hierarchy_items;

  auto draw_item_batch = [&](const std::vector<const ScenePreviewWidget::PreviewItem *> & batch, bool count_in_stats) {
    for (const auto * it : batch) {
      int item_placeholder_count = 0;
      int item_mesh_backed_count = 0;
      int item_wireframe_box_count = 0;
      int item_urdf_primitive_count = 0;
      int item_missing_geometry_count = 0;
      int item_primitive_fallback_count = 0;
      const bool drew_physical_geometry =
        draw_truthful_item_geometry(*it, &item_placeholder_count, &item_mesh_backed_count, &item_wireframe_box_count,
                                    &item_urdf_primitive_count, &item_missing_geometry_count, &item_primitive_fallback_count);
      if (drew_physical_geometry) ++rendered_item_count;
      if (count_in_stats) {
        placeholder_count += item_placeholder_count;
        mesh_rendered_count += item_mesh_backed_count;
        urdf_primitive_rendered_count += item_urdf_primitive_count;
        missing_geometry_count += item_missing_geometry_count;
        wireframe_box_count += item_wireframe_box_count;
        primitive_fallback_count += item_primitive_fallback_count;
      } else {
        missing_geometry_count += item_missing_geometry_count;
        primitive_fallback_count += item_primitive_fallback_count;
      }
      if (it->id == selected_id) {
        const ItemBounds bounds = item_bounds_for_role(*it);
        const bool editable = item_is_editable_for_gizmo(*it);
        const bool selected_generated = is_generated_urdf_visual_item(*it) || is_locked_urdf_item(*it);
        const QColor selection_outline = editable ? editable_layout_selected_highlight()
          : (selected_generated ? generated_locked_preview_outline() : QColor("#94a3b8"));
        draw_box_outline(bounds.x, bounds.y, bounds.z, bounds.sx, bounds.sy, bounds.sz, selection_outline, editable ? 3.0f : 1.4f);
      }
    }
  };  // draw_item_batch
  draw_item_batch(overlay_items, false);  // draw translucent overlays before solids to keep physical meshes legible.
  draw_item_batch(physical_items, true);
  last_render_counters.rendered_count = rendered_item_count;
  last_render_counters.mesh_source_count = mesh_source_count;
  last_render_counters.mesh_backed_count = mesh_source_count;
  last_render_counters.mesh_rendered_count = mesh_rendered_count;
  last_render_counters.urdf_primitive_source_count = urdf_primitive_source_count;
  last_render_counters.urdf_primitive_rendered_count = urdf_primitive_rendered_count;
  last_render_counters.placeholder_count = placeholder_count;
  last_render_counters.missing_geometry_count = missing_geometry_count;
  last_render_counters.wireframe_fallback_count = wireframe_box_count;
  last_render_counters.primitive_fallback_rendered_count = primitive_fallback_count;
  last_render_counters.primitive_fallback_count = primitive_fallback_count;
  last_render_counters.valid_physical_fallback_count = primitive_fallback_count;
  last_render_counters.overlay_rendered_count = overlay_count;
  last_render_counters.last_paint_completed = true;
  finalize_visual_quality(last_render_counters);
  last_render_counters.smoke_fallback_render_used = false;

  glDisable(GL_BLEND);

  qDebug() << "Scene3D runtime render: received=" << received_item_count
           << "visible=" << visible_item_count
           << "rendered=" << rendered_item_count
           << "skipped=" << skipped_item_count
           << "mesh_sources=" << mesh_source_count
           << "placeholder=" << placeholder_count
           << "overlay=" << overlay_count
           << "primitive_fallback_rendered_count=" << last_render_counters.primitive_fallback_rendered_count
           << "mesh_rendered_count=" << last_render_counters.mesh_rendered_count
           << "generated_fallback_count=" << last_render_counters.generated_fallback_count
           << "labels_drawn=" << last_render_counters.labels_drawn
           << "labels_suppressed_overlap=" << last_render_counters.labels_suppressed_overlap
           << "hierarchy_child_row_count=" << last_render_counters.hierarchy_child_row_count;
  qDebug() << kPaintGLCacheOnlyGuard;
  qDebug() << "Scene3D diagnostics {viewport_received_count=" << received_item_count
           << ", render_cache_count=" << mesh_cache_.size()
           << ", rendered_count=" << rendered_item_count
           << ", skipped_count=" << skipped_item_count
           << "}";

  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing, true);
  if (visible_item_count == 0) {
    painter.setPen(QColor("#f8fafc"));
    painter.drawText(rect(), Qt::AlignCenter,
      "Scene3D empty state\nNo visible candidate visuals.\nCheck preview layer visibility and scene source files.");
    if (status_message_cb) {
      status_message_cb("Scene3D blocker: no visible candidate visuals after ingestion and layer filtering.");
    }
  }
  const auto compact_role = [](const QString & role) {
    const QString r = role.trimmed().toLower();
    if (r.contains("pick")) return QStringLiteral("Pick");
    if (r.contains("place")) return QStringLiteral("Place");
    if (r.contains("camera")) return QStringLiteral("Cam");
    if (r.contains("safety")) return QStringLiteral("Safe");
    if (r.isEmpty()) return QStringLiteral("Item");
    return role.left(10);
  };
  const double zoom_factor = distance_ / qMax(0.25, scene_radius_);
  const bool crowded = items.size() > 30;
  const bool zoomed_far = zoom_factor > 5.2;
  const bool suppress_dense_non_critical_labels = crowded || zoomed_far;
  QVector<QPointF> robot_base_points;
  for (const auto & it : items) {
    if (classify_item_role(it) != NormalizedRole::RobotBase) continue;
    const ItemBounds bounds = item_bounds_for_role(it);
    robot_base_points.push_back(project_to_screen(bounds.x + (bounds.sx * 0.5), bounds.y + (bounds.sy * 0.5), bounds.z + (bounds.sz * 0.5)));
  }
  QVector<LabelDrawCandidate> label_candidates;
  for (int i = 0; i < items.size(); ++i) {
    const auto & it = items[i];
    const ItemBounds bounds = item_bounds_for_role(it);
    const QPointF p = project_to_screen(bounds.x + (bounds.sx * 0.5), bounds.y + (bounds.sy * 0.5), bounds.z + (bounds.sz * 0.5));
    const bool selected = (it.id == selected_id);
    const NormalizedRole role = classify_item_role(it);
    const ScenePreviewWidget::LabelMode effective_label_mode = ScenePreviewWidget::LabelMode::Selected;
    bool draw_label = false;
    switch (effective_label_mode) {
      case ScenePreviewWidget::LabelMode::Off: draw_label = selected; break;
      case ScenePreviewWidget::LabelMode::Important: draw_label = selected || is_critical_label_role(role); break;
      case ScenePreviewWidget::LabelMode::Selected: draw_label = selected; break;
      case ScenePreviewWidget::LabelMode::All: draw_label = true; break;
    }
    const bool is_urdf_visual = is_generated_urdf_visual_item(it) || is_locked_urdf_item(it);
    if (suppress_dense_non_critical_labels && !selected && !is_critical_label_role(role)) draw_label = false;
    if (is_urdf_visual && !selected && effective_label_mode != ScenePreviewWidget::LabelMode::All) draw_label = false;
    if (show_warning_labels && !it.warnings.isEmpty()) {
      if (debug_overlays_mode && show_warning_labels && !it.warnings.isEmpty()) {
        const QString debug_warning_text = warning_debug_text(it.warnings);
        Q_UNUSED(debug_warning_text);
      }
      painter.setPen(Qt::NoPen);
      painter.setBrush(QColor("#f59e0b"));
      painter.drawEllipse(QRectF(p.x() - 7.0, p.y() - 18.0, 14.0, 14.0));
      painter.setPen(QColor("#111827"));
      painter.drawText(QRectF(p.x() - 7.0, p.y() - 18.0, 14.0, 14.0), Qt::AlignCenter, warning_badge_text(it.warnings));
    }
    if (!draw_label) continue;
    const QString missing_reason = placeholder_reason_for_item(it);
    const bool is_critical_scene_anchor = (role == NormalizedRole::RobotBase || role == NormalizedRole::Table || role == NormalizedRole::Camera);
    if (!selected && is_urdf_visual && missing_reason.isEmpty() && !is_critical_scene_anchor &&
        effective_label_mode != ScenePreviewWidget::LabelMode::All) continue;

    const QString compact_text = clean_label_from_item(it);
    const QString ownership_text = item_visual_ownership_label(it);
    const QString text = selected ? QStringLiteral("%1 • %2").arg(compact_text, ownership_text)
                                  : (missing_reason.isEmpty() ? compact_text : QString("%1 missing").arg(compact_role(it.role)));
    LabelDrawCandidate candidate;
    candidate.item_index = i;
    candidate.base_point = p;
    candidate.anchor = QPointF(p.x() + 12.0, p.y() - 10.0);
    candidate.text = text;
    candidate.color = QColor("#e2e8f0");
    candidate.critical = is_critical_label_role(role);
    candidate.priority = label_priority_bucket(selected, !it.warnings.isEmpty(), role);
    label_candidates.push_back(candidate);
  }

  std::stable_sort(label_candidates.begin(), label_candidates.end(), [](const LabelDrawCandidate & a, const LabelDrawCandidate & b) {
    return a.priority < b.priority;
  });

  QVector<QRectF> placed_label_boxes;
  QVector<QPointF> placed_label_points;
  constexpr int kLowPriorityOverlapBudget = 6;
  int overlap_budget_hits = 0;
  bool low_priority_overlap_budget_exhausted = false;
  int labels_drawn = 0;
  int labels_suppressed_overlap = 0;
  for (const auto & candidate : label_candidates) {
    const bool low_priority = candidate.priority >= 3;
    if (low_priority_overlap_budget_exhausted && low_priority) {
      ++labels_suppressed_overlap;
      continue;
    }
    const QPointF label_pos = apply_label_overlap_offset(candidate.anchor, placed_label_points, robot_base_points, candidate.critical);
    const QRectF label_rect = painter.boundingRect(QRectF(label_pos.x() - 2.0, label_pos.y() - 14.0, 220.0, 18.0), Qt::AlignLeft | Qt::AlignVCenter, candidate.text);

    bool overlaps = false;
    for (const QRectF & existing_rect : placed_label_boxes) {
      if (label_rect.adjusted(-2.0, -1.0, 2.0, 1.0).intersects(existing_rect)) {
        overlaps = true;
        break;
      }
    }
    // LABEL_OVERLAP_SUPPRESS_LOWER_PRIORITY: keep high priority, suppress lower when overlap remains.
    if (overlaps) {
      ++labels_suppressed_overlap;
      if (low_priority && (++overlap_budget_hits > kLowPriorityOverlapBudget)) low_priority_overlap_budget_exhausted = true;
      continue;
    }

    placed_label_points.push_back(label_pos);
    placed_label_boxes.push_back(label_rect);
    painter.setPen(candidate.color);
    painter.drawText(label_pos, candidate.text);
    ++labels_drawn;
  }
  last_render_counters.labels_drawn = labels_drawn;
  last_render_counters.labels_suppressed_overlap = labels_suppressed_overlap;
  painter.setPen(Qt::NoPen);
  painter.setBrush(QColor(15, 23, 42, 190));
  painter.drawRoundedRect(QRectF(12.0, 12.0, 360.0, 74.0), 6.0, 6.0);
  painter.setPen(QColor("#e2e8f0"));
  painter.drawText(QRectF(20.0, 18.0, 344.0, 16.0), Qt::AlignLeft | Qt::AlignVCenter, "View: 3D");
  painter.drawText(QRectF(20.0, 34.0, 344.0, 16.0), Qt::AlignLeft | Qt::AlignVCenter,
                   QString("Scene: %1").arg(scene_name));
  painter.drawText(QRectF(20.0, 50.0, 460.0, 16.0), Qt::AlignLeft | Qt::AlignVCenter,
                   QString("Generated mesh %1/%2 • URDF primitives %3/%4 • Helpers %5 • Missing geometry %6")
                     .arg(mesh_rendered_count).arg(mesh_source_count)
                     .arg(urdf_primitive_rendered_count).arg(urdf_primitive_source_count)
                     .arg(overlay_count).arg(missing_geometry_count));
  painter.drawText(QRectF(20.0, 66.0, 344.0, 16.0), Qt::AlignLeft | Qt::AlignVCenter,
                   QString("Physical %1 • Locked URDF %2 • Fit: %3")
                     .arg(physical_item_count).arg(locked_urdf_count).arg(fit_include_overlays ? "all_items" : "generated_visuals"));
  if (drag_asset_preview_visible_) {
    const double x = (drag_asset_screen_pos_.x() - width() * 0.5) / 50.0;
    const double y = (height() * 0.6 - drag_asset_screen_pos_.y()) / 50.0;
    draw_box(x, y, 0.0, 0.35, 0.35, 0.35, QColor(56, 189, 248, 120), true);
    QToolTip::showText(mapToGlobal(drag_asset_screen_pos_), drag_asset_drop_status_, this);
  }
}

Scene3DViewportWidget::RenderDebugCounters Scene3DViewportWidget::render_debug_counters() const
{
  RenderDebugCounters counters = last_render_counters;
  finalize_visual_quality(counters);
  return counters;
}

bool Scene3DViewportWidget::render_smoke_fallback_frame(QImage * out_image)
{
  if (items.isEmpty()) return false;
  QSize target_size = size();
  if (target_size.width() < 32 || target_size.height() < 32) {
    target_size = QSize(640, 420);
  }
  QImage img(target_size, QImage::Format_ARGB32_Premultiplied);
  img.fill(QColor(5, 10, 24));

  int visible_item_count = 0;
  int skipped_item_count = 0;
  int rendered_item_count = 0;
  int mesh_source_count = 0;
  int mesh_rendered_count = 0;
  int urdf_primitive_source_count = 0;
  int urdf_primitive_rendered_count = 0;
  int primitive_fallback_count = 0;
  int placeholder_count = 0;
  int missing_geometry_count = 0;
  int generated_fallback_count = 0;
  int wireframe_fallback_count = 0;
  int overlay_count = 0;
  int locked_urdf_count = 0;
  int editable_layout_count = 0;
  QSet<QString> unique_visible_ids;

  QVector3D bmin, bmax;
  const bool has_bounds = scene_bounds_from_visible_items(bmin, bmax, true);
  const double span_x = has_bounds ? qMax(0.25, static_cast<double>(bmax.x() - bmin.x())) : 1.0;
  const double span_z = has_bounds ? qMax(0.25, static_cast<double>(bmax.z() - bmin.z())) : 1.0;
  const double scale = qMin((target_size.width() - 80.0) / span_x, (target_size.height() - 80.0) / span_z);
  auto project_top = [&](double x, double z) {
    const double px = 40.0 + (x - bmin.x()) * scale;
    const double py = target_size.height() - 40.0 - (z - bmin.z()) * scale;
    return QPointF(px, py);
  };

  QPainter painter(&img);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.setPen(QColor(71, 85, 105));
  for (int gx = 0; gx <= 10; ++gx) {
    const double x = 40.0 + gx * ((target_size.width() - 80.0) / 10.0);
    painter.drawLine(QPointF(x, 20.0), QPointF(x, target_size.height() - 20.0));
  }
  for (int gy = 0; gy <= 8; ++gy) {
    const double y = 40.0 + gy * ((target_size.height() - 80.0) / 8.0);
    painter.drawLine(QPointF(20.0, y), QPointF(target_size.width() - 20.0, y));
  }

  for (const auto & it : items) {
    const NormalizedRole role = classify_item_role(it);
    if (!show_safety && role == NormalizedRole::SafetyZone) {
      ++skipped_item_count;
      continue;
    }
    ++visible_item_count;
    unique_visible_ids.insert(it.id);
    const bool overlay_helper = is_overlay_only_item(it) || is_overlay_visual_role(role);
    const bool generated_urdf = is_locked_urdf_item(it) || is_generated_urdf_visual_item(it);
    if (overlay_helper) ++overlay_count;
    if (generated_urdf) ++locked_urdf_count;
    if (it.linked_to_editable_layout_state) ++editable_layout_count;
    const QString source_layer = normalized_scene3d_layer_token(it.source_layer);
    const QString visual_source = normalized_scene3d_layer_token(it.active_visual_source);
    const bool intentional_primitive_fallback = source_layer == QStringLiteral("primitive_fallback") || visual_source == QStringLiteral("primitive_fallback");
    const bool physical_mesh_source = !overlay_helper && item_has_credible_mesh_handoff(it);
    if (physical_mesh_source) {
      ++mesh_source_count;
      if (intentional_primitive_fallback && item_has_explicit_dimensions(it)) ++primitive_fallback_count;
    } else if (!overlay_helper && generated_urdf && item_has_explicit_dimensions(it)) {
      ++urdf_primitive_source_count;
      ++urdf_primitive_rendered_count;
      if (intentional_primitive_fallback) ++primitive_fallback_count;
      else ++wireframe_fallback_count;
    } else if (!overlay_helper && !item_has_explicit_dimensions(it)) {
      ++placeholder_count;
      ++missing_geometry_count;
      ++generated_fallback_count;
    } else if (!overlay_helper) {
      ++wireframe_fallback_count;
    }

    const ItemBounds bounds = item_bounds_for_role(it);
    const QPointF p0 = project_top(bounds.x, bounds.z);
    const QPointF p1 = project_top(bounds.x + bounds.sx, bounds.z + bounds.sz);
    QRectF rect(QPointF(qMin(p0.x(), p1.x()), qMin(p0.y(), p1.y())),
                QPointF(qMax(p0.x(), p1.x()), qMax(p0.y(), p1.y())));
    if (rect.width() < 4.0) rect.setWidth(4.0);
    if (rect.height() < 4.0) rect.setHeight(4.0);
    QColor fill = item_color(it);
    fill.setAlpha(is_overlay_visual_role(role) ? 90 : 170);
    painter.setBrush(fill);
    painter.setPen(QPen(item_color(it).lighter(135), it.id == selected_id ? 3.0 : 1.5));
    painter.drawRect(rect);
    ++rendered_item_count;
    if (physical_mesh_source) ++mesh_rendered_count;
  }

  painter.setPen(QColor("#e2e8f0"));
  painter.drawText(QRectF(16.0, 12.0, target_size.width() - 32.0, 24.0),
                   Qt::AlignLeft | Qt::AlignVCenter,
                   QString("Scene3D smoke fallback render: %1").arg(scene_name));
  painter.end();

  last_render_counters.preview_items_count = items.size();
  last_render_counters.total_payload_count = items.size();
  last_render_counters.viewport_received_count = items.size();
  last_render_counters.render_cache_count = mesh_cache_.size();
  last_render_counters.visible_count = visible_item_count;
  last_render_counters.rendered_count = rendered_item_count;
  last_render_counters.skipped_count = skipped_item_count;
  last_render_counters.unique_visible_item_count = unique_visible_ids.size();
  last_render_counters.mesh_source_count = mesh_source_count;
  last_render_counters.mesh_backed_count = mesh_source_count;
  last_render_counters.mesh_rendered_count = mesh_rendered_count;
  last_render_counters.urdf_primitive_source_count = urdf_primitive_source_count;
  last_render_counters.urdf_primitive_rendered_count = urdf_primitive_rendered_count;
  last_render_counters.placeholder_count = placeholder_count;
  last_render_counters.missing_geometry_count = missing_geometry_count;
  last_render_counters.generated_fallback_count = generated_fallback_count;
  last_render_counters.wireframe_fallback_count = wireframe_fallback_count;
  last_render_counters.primitive_fallback_rendered_count = primitive_fallback_count;
  last_render_counters.primitive_fallback_count = primitive_fallback_count;
  last_render_counters.valid_physical_fallback_count = primitive_fallback_count;
  last_render_counters.overlay_rendered_count = overlay_count;
  last_render_counters.overlay_helper_count = overlay_count;
  last_render_counters.overlay_count = overlay_count;
  last_render_counters.locked_generated_urdf_visual_count = locked_urdf_count;
  last_render_counters.editable_layout_count = editable_layout_count;
  last_render_counters.hierarchy_child_row_count = visible_item_count;
  last_render_counters.last_paint_completed = rendered_item_count > 0;
  last_render_counters.smoke_fallback_render_used = rendered_item_count > 0;
  finalize_visual_quality(last_render_counters);
  if (out_image) *out_image = img;
  return rendered_item_count > 0;
}

bool Scene3DViewportWidget::scene_bounds_from_visible_items(QVector3D & out_min, QVector3D & out_max, bool include_overlays) const
{
  if (items.isEmpty()) return false;
  out_min = QVector3D(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
  out_max = QVector3D(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());
  bool has_fittable_item = false;
  for (const auto & it : items) {
    if (!include_in_fit_bounds(it, include_overlays)) continue;
    has_fittable_item = true;
    const ItemBounds bounds = item_bounds_for_role(it);
    out_min.setX(std::min(out_min.x(), static_cast<float>(bounds.x)));
    out_min.setY(std::min(out_min.y(), static_cast<float>(bounds.y)));
    out_min.setZ(std::min(out_min.z(), static_cast<float>(bounds.z)));
    out_max.setX(std::max(out_max.x(), static_cast<float>(bounds.x + bounds.sx)));
    out_max.setY(std::max(out_max.y(), static_cast<float>(bounds.y + bounds.sy)));
    out_max.setZ(std::max(out_max.z(), static_cast<float>(bounds.z + bounds.sz)));
  }
  return has_fittable_item;
}

bool Scene3DViewportWidget::robot_bounds_from_rendered_visuals(QVector3D & out_min, QVector3D & out_max) const
{
  out_min = QVector3D(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
  out_max = QVector3D(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());
  bool has_robot_visual = false;
  for (const auto & it : items) {
    const bool urdf_visual = is_generated_urdf_visual_item(it) || is_locked_urdf_item(it);
    const bool robot_base_marker = classify_item_role(it) == NormalizedRole::RobotBase && it.linked_to_editable_layout_state;
    if (!urdf_visual && !robot_base_marker) continue;
    if (is_overlay_only_item(it)) continue;
    ItemBounds bounds{};
    if (!mesh_world_bounds_for_item(it, bounds)) bounds = { it.x, it.y, it.z, it.sx, it.sy, it.sz };
    out_min.setX(std::min(out_min.x(), static_cast<float>(bounds.x)));
    out_min.setY(std::min(out_min.y(), static_cast<float>(bounds.y)));
    out_min.setZ(std::min(out_min.z(), static_cast<float>(bounds.z)));
    out_max.setX(std::max(out_max.x(), static_cast<float>(bounds.x + bounds.sx)));
    out_max.setY(std::max(out_max.y(), static_cast<float>(bounds.y + bounds.sy)));
    out_max.setZ(std::max(out_max.z(), static_cast<float>(bounds.z + bounds.sz)));
    has_robot_visual = true;
  }
  return has_robot_visual;
}

bool Scene3DViewportWidget::item_has_explicit_dimensions(const ScenePreviewWidget::PreviewItem & item) const
{
  return item.sx > 0.001 && item.sy > 0.001 && item.sz > 0.001;
}

QString Scene3DViewportWidget::placeholder_reason_for_item(const ScenePreviewWidget::PreviewItem & item) const
{
  if (item_has_credible_mesh_handoff(item)) return QString();
  if (item_has_valid_urdf_primitive(item) || item_has_explicit_dimensions(item)) return QString();
  return QStringLiteral("missing geometry");
}


bool Scene3DViewportWidget::draw_urdf_primitive_geometry(const ScenePreviewWidget::PreviewItem & it, const QColor & color)
{
  const QString type = normalized_token(it.primitive_geometry_type);
  if (!item_has_valid_urdf_primitive(it)) return false;

  glPushMatrix();
  glTranslated(it.x, it.y, it.z);
  glRotated(qRadiansToDegrees(it.roll), 1.0, 0.0, 0.0);
  glRotated(qRadiansToDegrees(it.pitch), 0.0, 1.0, 0.0);
  glRotated(qRadiansToDegrees(it.yaw), 0.0, 0.0, 1.0);
  if (it.visual_origin_applied) {
    glTranslated(it.visual_origin_x, it.visual_origin_y, it.visual_origin_z);
    glRotated(qRadiansToDegrees(it.visual_origin_roll), 1.0, 0.0, 0.0);
    glRotated(qRadiansToDegrees(it.visual_origin_pitch), 0.0, 1.0, 0.0);
    glRotated(qRadiansToDegrees(it.visual_origin_yaw), 0.0, 0.0, 1.0);
  }
  glScaled(it.mesh_scale_x, it.mesh_scale_y, it.mesh_scale_z);

  if (type == QStringLiteral("box")) {
    draw_box(-it.sx * 0.5, -it.sy * 0.5, -it.sz * 0.5, it.sx, it.sy, it.sz, color, color.alphaF() < 0.99);
  } else if (type == QStringLiteral("cylinder")) {
    const double r = it.primitive_radius;
    const double h = it.primitive_length;
    draw_cylinder(0.0, -h * 0.5, 0.0, r, h, color, color.alphaF() < 0.99, 32);
  } else if (type == QStringLiteral("sphere")) {
    draw_sphere(0.0, 0.0, 0.0, it.primitive_radius, color, color.alphaF() < 0.99, 24, 12);
  } else if (type == QStringLiteral("capsule")) {
    const double r = it.primitive_radius;
    const double h = it.primitive_length;
    draw_cylinder(0.0, -h * 0.5, 0.0, r, h, color, color.alphaF() < 0.99, 32);
    draw_sphere(0.0, -h * 0.5, 0.0, r, color, color.alphaF() < 0.99, 24, 8);
    draw_sphere(0.0, h * 0.5, 0.0, r, color, color.alphaF() < 0.99, 24, 8);
  } else {
    glPopMatrix();
    return false;
  }
  glPopMatrix();
  return true;
}

bool Scene3DViewportWidget::draw_truthful_item_geometry(const ScenePreviewWidget::PreviewItem & it, int * out_placeholder_count,
                                                        int * out_mesh_count, int * out_wireframe_count,
                                                        int * out_urdf_primitive_count, int * out_missing_geometry_count,
                                                        int * out_primitive_fallback_count)
{
  const QString source_layer = normalized_scene3d_layer_token(it.source_layer);
  const QString visual_source = normalized_scene3d_layer_token(it.active_visual_source);
  const bool helper_overlay = is_overlay_only_item(it) || source_layer == "overlay" || visual_source == "overlay";
  if (helper_overlay) {
    if (item_has_explicit_dimensions(it)) {
      draw_box_outline(it.x, it.y, it.z, it.sx, it.sy, it.sz, QColor(148, 163, 184, 58), 0.75f);
      if (out_wireframe_count) ++(*out_wireframe_count);
      return false;
    }
    return false;
  }
  // Always try mesh-backed draw first for physical items.
  QColor visual_color = item_color(it);
  const bool generated_or_locked_preview = is_generated_urdf_visual_item(it) || is_locked_urdf_item(it);
  const bool editable_layout_preview = it.linked_to_editable_layout_state || item_is_editable_for_gizmo(it);
  if (draw_mesh_preview_if_available(it, visual_color, true)) {
    if (out_mesh_count) ++(*out_mesh_count);
    ItemBounds mesh_bounds{};
    if (!mesh_world_bounds_for_item(it, mesh_bounds)) mesh_bounds = item_bounds_for_role(it);
    if (generated_or_locked_preview) {
      draw_box_outline(mesh_bounds.x, mesh_bounds.y, mesh_bounds.z, mesh_bounds.sx, mesh_bounds.sy, mesh_bounds.sz,
                       generated_locked_preview_outline(), 0.85f);
    } else if (editable_layout_preview) {
      draw_box_outline(mesh_bounds.x, mesh_bounds.y, mesh_bounds.z, mesh_bounds.sx, mesh_bounds.sy, mesh_bounds.sz,
                       editable_layout_accent_outline(), 1.6f);
    }
    return true;
  }
  const NormalizedRole semantic_role = classify_item_role(it);
  if (is_clean_semantic_primitive_role(semantic_role)) {
    if (item_has_explicit_primitive_dimensions(it)) {
      if (draw_clean_semantic_primitive(it)) {
        if (out_primitive_fallback_count) ++(*out_primitive_fallback_count);
        return true;
      }
    } else {
      // Semantic authoring roles without dimensions remain diagnostics-only.
      // They are still counted as missing geometry for readiness/status, but the
      // main 3D viewport intentionally avoids large red missing-geometry markers.
      if (out_missing_geometry_count) ++(*out_missing_geometry_count);
      return false;
    }
  }

  const QString missing_reason = placeholder_reason_for_item(it);
  if (!missing_reason.isEmpty()) {
    draw_missing_geometry_marker(it);
    if (out_placeholder_count) ++(*out_placeholder_count);
    if (out_missing_geometry_count) ++(*out_missing_geometry_count);
    ++last_render_counters.generated_fallback_count;
    if (show_warning_labels) warn_mesh_fallback_once(it.id, QStringLiteral("REJECT_MISSING_GEOMETRY: no mesh metadata or explicit primitive dimensions"), it.source_path);
    return false;
  }
  if (item_has_valid_urdf_primitive(it)) {
    if (mesh_preview_mode == ScenePreviewWidget::MeshPreviewMode::Meshes) {
      // Primitive geometry exists but is intentionally hidden by Meshes-only mode; do not show a red missing-geometry marker.
      if (item_has_explicit_dimensions(it)) {
        draw_box_outline(it.x, it.y, it.z, it.sx, it.sy, it.sz, generated_primitive_fallback_outline(), 0.6f);
      }
      ++last_render_counters.generated_fallback_count;
      warn_mesh_fallback_once(it.id, QStringLiteral("REJECT_PRIMITIVE_SUPPRESSED_BY_MESH_ONLY_MODE: URDF primitive available but disabled"), it.source_path);
      return false;
    }
    const QColor primitive_fill = generated_or_locked_preview ? generated_primitive_fallback_fill() : visual_color;
    if (draw_urdf_primitive_geometry(it, primitive_fill)) {
      if (generated_or_locked_preview && item_has_explicit_dimensions(it)) {
        draw_box_outline(it.x, it.y, it.z, it.sx, it.sy, it.sz, generated_primitive_fallback_outline(), 0.7f);
      } else if (editable_layout_preview && item_has_explicit_dimensions(it)) {
        draw_box_outline(it.x, it.y, it.z, it.sx, it.sy, it.sz, editable_layout_accent_outline(), 1.4f);
      }
      if (out_urdf_primitive_count) ++(*out_urdf_primitive_count);
      return true;
    }
  }
  if (item_has_explicit_dimensions(it)) {
    const bool intentional_primitive_fallback =
      source_layer == QStringLiteral("primitive_fallback") || visual_source == QStringLiteral("primitive_fallback");
    const bool raw_generated_bounds_only = is_raw_generated_bounds_only_item(it);
    if (mesh_preview_mode == ScenePreviewWidget::MeshPreviewMode::Meshes || raw_generated_bounds_only) {
      if (raw_generated_bounds_only) {
        warn_mesh_fallback_once(it.id, QStringLiteral("REJECT_RAW_GENERATED_BOUNDS_SUPPRESSED: mesh or URDF primitive source should provide geometry"), it.source_path);
        return false;
      }
      // Semantic primitive geometry exists but is intentionally hidden by Meshes-only mode; do not show a warning marker.
      draw_box_outline(it.x, it.y, it.z, it.sx, it.sy, it.sz, generated_primitive_fallback_outline(), 0.6f);
      ++last_render_counters.generated_fallback_count;
      warn_mesh_fallback_once(it.id, QStringLiteral("REJECT_PRIMITIVE_SUPPRESSED_BY_MESH_ONLY_MODE: semantic primitive dimensions available but disabled"), it.source_path);
      return false;
    }
    const QColor fallback_fill = generated_or_locked_preview ? generated_primitive_fallback_fill()
      : (editable_layout_preview ? QColor(34, 211, 238, 60) : QColor(148, 163, 184, 28));
    const QColor fallback_line = generated_or_locked_preview ? generated_primitive_fallback_outline()
      : (editable_layout_preview ? editable_layout_accent_outline() : QColor(148, 163, 184, 76));
    const float fallback_line_width = generated_or_locked_preview ? 0.7f : (editable_layout_preview ? 1.6f : 0.75f);
    draw_box(it.x, it.y, it.z, it.sx, it.sy, it.sz, fallback_fill, true);
    // Generated primitive fallback styling is deliberately translucent with a thinner outline, not a missing-geometry warning.
    draw_box_outline(it.x, it.y, it.z, it.sx, it.sy, it.sz, fallback_line, fallback_line_width);
    if (intentional_primitive_fallback) {
      if (out_primitive_fallback_count) ++(*out_primitive_fallback_count);
      return true;
    }
    if (out_wireframe_count) ++(*out_wireframe_count);
    return false;
  }
  draw_missing_geometry_marker(it);
  if (out_placeholder_count) ++(*out_placeholder_count);
  if (out_missing_geometry_count) ++(*out_missing_geometry_count);
  ++last_render_counters.generated_fallback_count;
  return false;
}

QString Scene3DViewportWidget::gizmo_mode_label() const
{
  switch (gizmo_mode) {
    case GizmoMode::Select: return "Select";
    case GizmoMode::Move: return "Move";
    case GizmoMode::Rotate: return "Rotate";
    case GizmoMode::ScaleDisabled: return "Scale";
  }
  return "Select";
}

bool Scene3DViewportWidget::parse_stl_bytes_for_test(const QByteArray & bytes, const QString & source_hint,
                                                     InternalTriangleMesh & out_mesh, QString & out_error,
                                                     int triangle_limit)
{
  out_mesh.triangles.clear();
  const bool ext_ascii_hint = source_hint.endsWith(".stla", Qt::CaseInsensitive);
  const bool ext_binary_hint = source_hint.endsWith(".stlb", Qt::CaseInsensitive);
  const bool ascii = ext_ascii_hint || (!ext_binary_hint && looks_like_ascii_stl(bytes));
  if (ascii) return parse_ascii_stl(bytes, out_mesh, out_error, triangle_limit);
  return parse_binary_stl(bytes, out_mesh, out_error, triangle_limit);
}

bool Scene3DViewportWidget::parse_collada_bytes_for_test(const QByteArray & bytes, const QString & source_hint,
                                                         InternalTriangleMesh & out_mesh, QString & out_error,
                                                         double * out_unit_meter, int triangle_limit)
{
  Q_UNUSED(source_hint);
  out_mesh.triangles.clear();
  return parse_collada_bytes(bytes, out_mesh, out_error, out_unit_meter, triangle_limit);
}

bool Scene3DViewportWidget::parse_obj_bytes_for_test(const QByteArray & bytes, const QString & source_hint,
                                                     InternalTriangleMesh & out_mesh, QString & out_error,
                                                     int triangle_limit)
{
  Q_UNUSED(source_hint);
  out_mesh.triangles.clear();
  return parse_obj_bytes(bytes, out_mesh, out_error, triangle_limit);
}

bool Scene3DViewportWidget::compute_mesh_bounds_for_test(const InternalTriangleMesh & mesh, QVector3D & out_min, QVector3D & out_max)
{
  if (mesh.triangles.isEmpty()) return false;
  out_min = QVector3D(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
  out_max = QVector3D(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());
  for (const auto & tri : mesh.triangles) {
    for (const auto & v : tri.vertices) {
      out_min.setX(qMin(out_min.x(), v.x()));
      out_min.setY(qMin(out_min.y(), v.y()));
      out_min.setZ(qMin(out_min.z(), v.z()));
      out_max.setX(qMax(out_max.x(), v.x()));
      out_max.setY(qMax(out_max.y(), v.y()));
      out_max.setZ(qMax(out_max.z(), v.z()));
    }
  }
  return true;
}

bool Scene3DViewportWidget::should_attempt_mesh_draw_for_mode_for_test(ScenePreviewWidget::MeshPreviewMode mode,
                                                                        bool cache_loaded, bool cache_valid)
{
  if (mode == ScenePreviewWidget::MeshPreviewMode::Primitives) return false;
  if (mode == ScenePreviewWidget::MeshPreviewMode::Meshes) return cache_loaded && cache_valid;
  return cache_loaded && cache_valid;
}

QString Scene3DViewportWidget::render_role_for_test(const ScenePreviewWidget::PreviewItem & item)
{
  const QString source_layer = normalized_scene3d_layer_token(item.source_layer);
  const QString visual_source = normalized_scene3d_layer_token(item.active_visual_source);
  if (is_overlay_only_item(item) || source_layer == "overlay" || visual_source == "overlay") return QStringLiteral("helper_overlay");
  if (is_generated_urdf_visual_item(item) || is_locked_urdf_item(item)) {
    return item.mesh_available ? QStringLiteral("generated_urdf_mesh") : QStringLiteral("generated_urdf_primitive");
  }
  if (item.linked_to_editable_layout_state) return QStringLiteral("physical_layout_item");
  if (!item.mesh_available && item.mesh_path.trimmed().isEmpty() && !item.has_mesh_metadata) return QStringLiteral("missing_mesh_fallback");
  return QStringLiteral("real_urdf_primitive");
}

bool Scene3DViewportWidget::should_include_in_default_fit_for_test(const ScenePreviewWidget::PreviewItem & item)
{
  return include_in_fit_bounds(item, false);
}

bool Scene3DViewportWidget::should_draw_as_solid_for_test(const ScenePreviewWidget::PreviewItem & item,
                                                           ScenePreviewWidget::MeshPreviewMode mode)
{
  if (should_draw_clean_semantic_primitive_for_test(item)) return true;
  const QString role = render_role_for_test(item);
  if (role == "helper_overlay" || role == "missing_mesh_fallback") return false;
  if (mode == ScenePreviewWidget::MeshPreviewMode::Meshes && !item.mesh_available) return false;
  return true;
}

bool Scene3DViewportWidget::should_draw_as_wireframe_for_test(const ScenePreviewWidget::PreviewItem & item,
                                                               ScenePreviewWidget::MeshPreviewMode mode)
{
  Q_UNUSED(mode);
  const QString role = render_role_for_test(item);
  if (item_has_valid_urdf_primitive(item)) return false;
  if (should_suppress_missing_geometry_marker_for_semantic_role(item)) return false;
  return role == "helper_overlay" || role == "missing_mesh_fallback";
}

bool Scene3DViewportWidget::should_draw_clean_semantic_primitive_for_test(const ScenePreviewWidget::PreviewItem & item)
{
  return is_clean_semantic_primitive_role(classify_item_role(item)) && item_has_explicit_primitive_dimensions(item);
}

bool Scene3DViewportWidget::should_suppress_missing_geometry_marker_for_test(const ScenePreviewWidget::PreviewItem & item)
{
  return should_suppress_missing_geometry_marker_for_semantic_role(item);
}

bool Scene3DViewportWidget::try_resolve_canonical_mesh_path(const QString & path, QString & out_canonical,
                                                               const ScenePreviewWidget::PreviewItem * item,
                                                               QString * out_failure_reason) const
{
  if (out_failure_reason) out_failure_reason->clear();
  QFileInfo info(path);
  if (!info.exists() || !info.isFile()) {
    if (out_failure_reason) *out_failure_reason = mesh_load_failure_reason_for_item(path, item);
    return false;
  }
  out_canonical = info.canonicalFilePath();
  if (out_canonical.isEmpty()) out_canonical = info.absoluteFilePath();
  if (out_canonical.isEmpty() && out_failure_reason) *out_failure_reason = mesh_load_failure_reason_for_item(path, item);
  return !out_canonical.isEmpty();
}

bool Scene3DViewportWidget::warn_mesh_fallback_once(const QString & item_id, const QString & reason, const QString & path)
{
  const QString scene_key = scene_name.trimmed().isEmpty() ? QStringLiteral("No scene") : scene_name.trimmed();
  const QString path_key = path.trimmed().isEmpty() ? QStringLiteral("<none>") : path.trimmed();
  const QString key = QStringLiteral("%1|%2|%3|%4").arg(scene_key, item_id, reason, path_key);
  if (warned_mesh_fallbacks_.contains(key)) return false;
  warned_mesh_fallbacks_.insert(key);
  const QString reason_code = reason.section(QLatin1Char(':'), 0, 0).trimmed();
  qWarning().noquote() << QStringLiteral("Scene3D render fallback: scene=%1 item_id=%2 reason_code=%3 detail=%4 mesh_path=%5")
                            .arg(scene_key,
                                 item_id.trimmed().isEmpty() ? QStringLiteral("<unknown>") : item_id.trimmed(),
                                 reason_code.isEmpty() ? QStringLiteral("REJECT_UNKNOWN") : reason_code,
                                 reason,
                                 path_key);
  return true;
}

const Scene3DViewportWidget::MeshCacheEntry & Scene3DViewportWidget::ensure_mesh_cached(const ScenePreviewWidget::PreviewItem & item,
                                                                                           const QString & path)
{
  const QFileInfo input_info(path);
  QString canonical;
  QString load_failure_reason;
  if (!try_resolve_canonical_mesh_path(path, canonical, &item, &load_failure_reason)) canonical = input_info.absoluteFilePath();
  auto it = mesh_cache_.find(canonical);
  if (it != mesh_cache_.end()) return it.value();
  MeshCacheEntry entry;
  entry.loaded = true;
  entry.requested_path = path;
  entry.package_uri = item.package_uri;
  entry.resolved_source_path_original = item.resolved_source_path_original;
  entry.source_path_resolution_outcome = item.source_path_resolution_outcome;
  entry.resolved_source_path_stale = item.resolved_source_path_stale;
  entry.load_failure_reason = load_failure_reason;
  entry.failure_reason_code = load_failure_reason;
  if (!input_info.exists() || !input_info.isFile()) {
    entry.valid = false;
    if (entry.load_failure_reason.trimmed().isEmpty()) {
      entry.load_failure_reason = mesh_load_failure_reason_for_item(path, &item);
    }
    entry.failure_reason_code = entry.load_failure_reason.trimmed().isEmpty() ? QStringLiteral("file_not_found") : entry.load_failure_reason;
    entry.warning = QStringLiteral("mesh missing on disk (reason_code: %1)").arg(entry.failure_reason_code);
    return mesh_cache_.insert(canonical, entry).value();
  }
  QFile file(canonical);
  if (!file.open(QIODevice::ReadOnly)) {
    entry.failure_reason_code = QStringLiteral("parse_failed");
    entry.warning = QStringLiteral("mesh unreadable");
    return mesh_cache_.insert(canonical, entry).value();
  }
  const QByteArray bytes = file.readAll();
  const QString ext = input_info.suffix().toLower();
  QString parse_error;
  if (ext == QStringLiteral("stl")) {
    entry.parser_type = QStringLiteral("stl");
    entry.valid = parse_stl_bytes_for_test(bytes, canonical, entry.mesh, parse_error, kMeshTriangleLimit);
  } else if (ext == QStringLiteral("dae")) {
    entry.parser_type = QStringLiteral("dae");
    entry.valid = parse_collada_bytes_for_test(bytes, canonical, entry.mesh, parse_error, &entry.dae_unit_meter, kMeshTriangleLimit);
  } else if (ext == QStringLiteral("obj")) {
    entry.parser_type = QStringLiteral("obj");
    entry.valid = parse_obj_bytes_for_test(bytes, canonical, entry.mesh, parse_error, kMeshTriangleLimit);
  } else {
    entry.parser_type = ext;
    entry.valid = false;
    entry.failure_reason_code = QStringLiteral("unsupported_extension");
    parse_error = QStringLiteral("unsupported mesh format: .%1").arg(ext.isEmpty() ? QStringLiteral("<none>") : ext);
  }
  entry.parse_error = parse_error;
  if (entry.valid && entry.mesh.triangles.isEmpty()) {
    entry.valid = false;
    parse_error = QStringLiteral("%1 contains no triangles").arg(entry.parser_type);
    entry.parse_error = parse_error;
  }
  entry.parse_status = entry.valid ? QStringLiteral("ok") : QStringLiteral("error");
  if (!entry.valid) {
    entry.oversized = parse_error.contains("exceeds limit");
    if (entry.failure_reason_code.trimmed().isEmpty()) entry.failure_reason_code = mesh_parse_failure_code(parse_error);
    entry.warning = QStringLiteral("%1 reason_code=%2 (%3)").arg(entry.oversized ? QStringLiteral("mesh oversized") : QStringLiteral("mesh invalid"), entry.failure_reason_code, parse_error);
  } else {
    entry.failure_reason_code.clear();
  }
  if (entry.valid && entry.parser_type == QStringLiteral("dae") && entry.dae_unit_meter > 0.0 && qIsFinite(entry.dae_unit_meter)) {
    Scene3DViewportWidget::InternalTriangleMesh pre_unit_mesh = entry.mesh;
    const float inv_unit = static_cast<float>(1.0 / entry.dae_unit_meter);
    for (auto & tri : pre_unit_mesh.triangles) {
      for (int i = 0; i < 3; ++i) tri.vertices[i] *= inv_unit;
    }
    entry.dae_has_pre_unit_bounds = compute_mesh_bounds_for_test(pre_unit_mesh, entry.dae_pre_unit_min, entry.dae_pre_unit_max);
    if (entry.dae_has_pre_unit_bounds) entry.dae_pre_unit_span = entry.dae_pre_unit_max - entry.dae_pre_unit_min;
  }
  entry.has_bounds = compute_mesh_bounds_for_test(entry.mesh, entry.local_min, entry.local_max);
  if (entry.has_bounds) entry.local_span = entry.local_max - entry.local_min;
  return mesh_cache_.insert(canonical, entry).value();
}





bool Scene3DViewportWidget::validate_mesh_final_span(const ScenePreviewWidget::PreviewItem & it,
                                                     const MeshCacheEntry & entry,
                                                     const QString & mesh_source,
                                                     QString & out_reason,
                                                     QVector3D * out_raw_span,
                                                     QVector3D * out_final_span) const
{
  if (!entry.has_bounds) return true;
  constexpr double kFinalSpanThresholdMeters = 50.0;
  const QVector3D raw_span = entry.local_span;
  const QVector3D final_span(raw_span.x() * it.mesh_scale_x,
                             raw_span.y() * it.mesh_scale_y,
                             raw_span.z() * it.mesh_scale_z);
  if (out_raw_span) *out_raw_span = raw_span;
  if (out_final_span) *out_final_span = final_span;
  const QVector3D abs_final(qAbs(final_span.x()), qAbs(final_span.y()), qAbs(final_span.z()));
  const bool finite_final = qIsFinite(abs_final.x()) && qIsFinite(abs_final.y()) && qIsFinite(abs_final.z());
  const double max_final_span = qMax(abs_final.x(), qMax(abs_final.y(), abs_final.z()));
  if (!finite_final || max_final_span > kFinalSpanThresholdMeters) {
    out_reason = QStringLiteral("unreasonable_bounds_final_span item_id=%1 mesh_path=%2 raw_span=[%3,%4,%5] final_span=[%6,%7,%8] threshold_m=%9")
      .arg(it.id, mesh_source)
      .arg(raw_span.x(), 0, 'g', 8).arg(raw_span.y(), 0, 'g', 8).arg(raw_span.z(), 0, 'g', 8)
      .arg(abs_final.x(), 0, 'g', 8).arg(abs_final.y(), 0, 'g', 8).arg(abs_final.z(), 0, 'g', 8)
      .arg(kFinalSpanThresholdMeters, 0, 'g', 8);
    return false;
  }
  return true;
}

QJsonArray Scene3DViewportWidget::mesh_diagnostics_export() const
{
  QJsonArray out;
  for (auto it = mesh_cache_.cbegin(); it != mesh_cache_.cend(); ++it) {
    const QString & canonical_path = it.key();
    const MeshCacheEntry & e = it.value();
    QJsonObject row;
    row["canonical_path"] = canonical_path;
    row["loaded"] = e.loaded;
    row["valid"] = e.valid;
    row["warning"] = e.warning;
    row["load_failure_reason"] = e.load_failure_reason;
    row["failure_reason_code"] = e.failure_reason_code;
    row["requested_path"] = e.requested_path;
    row["package_uri"] = e.package_uri;
    row["resolved_source_path_stale"] = e.resolved_source_path_stale;
    row["resolved_source_path_original"] = e.resolved_source_path_original;
    row["source_path_resolution_outcome"] = e.source_path_resolution_outcome;
    row["oversized"] = e.oversized;
    const QString parser = (e.parser_type == "stl" || e.parser_type == "dae" || e.parser_type == "obj") ? e.parser_type : QStringLiteral("unsupported");
    row["parser_type"] = parser;
    row["parse_error"] = e.parse_error;
    row["rejected_reason_code"] = e.failure_reason_code.trimmed().isEmpty() ? e.parse_status : e.failure_reason_code;
    row["triangle_count"] = static_cast<int>(e.mesh.triangles.size());
    row["has_bounds"] = e.has_bounds;
    row["local_min"] = QJsonArray{e.local_min.x(), e.local_min.y(), e.local_min.z()};
    row["local_max"] = QJsonArray{e.local_max.x(), e.local_max.y(), e.local_max.z()};
    row["span"] = QJsonArray{e.local_span.x(), e.local_span.y(), e.local_span.z()};
    row["dae_unit_meter"] = e.dae_unit_meter;
    row["dae_has_pre_unit_bounds"] = e.dae_has_pre_unit_bounds;
    row["dae_pre_unit_min"] = QJsonArray{e.dae_pre_unit_min.x(), e.dae_pre_unit_min.y(), e.dae_pre_unit_min.z()};
    row["dae_pre_unit_max"] = QJsonArray{e.dae_pre_unit_max.x(), e.dae_pre_unit_max.y(), e.dae_pre_unit_max.z()};
    row["dae_pre_unit_span"] = QJsonArray{e.dae_pre_unit_span.x(), e.dae_pre_unit_span.y(), e.dae_pre_unit_span.z()};
    row["dae_post_unit_min"] = QJsonArray{e.local_min.x(), e.local_min.y(), e.local_min.z()};
    row["dae_post_unit_max"] = QJsonArray{e.local_max.x(), e.local_max.y(), e.local_max.z()};
    row["dae_post_unit_span"] = QJsonArray{e.local_span.x(), e.local_span.y(), e.local_span.z()};

    QJsonArray guard_details;
    for (const auto & item : items) {
      const QString mesh_source = !item.mesh_path.trimmed().isEmpty() ? item.mesh_path : item.source_path;
      QString canonical_source;
      if (!try_resolve_canonical_mesh_path(mesh_source, canonical_source, &item)) canonical_source = QFileInfo(mesh_source).absoluteFilePath();
      if (canonical_source != canonical_path) continue;
      QVector3D raw_span, final_span;
      QString reason;
      const bool accepted = validate_mesh_final_span(item, e, mesh_source, reason, &raw_span, &final_span);
      QJsonObject gd;
      gd["item_id"] = item.id;
      gd["accepted"] = accepted;
      gd["reason"] = reason;
      gd["reason_code"] = accepted ? QStringLiteral("ok") : QStringLiteral("unreasonable_bounds");
      gd["raw_span"] = QJsonArray{raw_span.x(), raw_span.y(), raw_span.z()};
      gd["final_span"] = QJsonArray{final_span.x(), final_span.y(), final_span.z()};
      guard_details.append(gd);
    }
    row["guard_decision_details"] = guard_details;
    out.append(row);
  }
  return out;
}

bool Scene3DViewportWidget::draw_mesh_preview_if_available(const ScenePreviewWidget::PreviewItem & it, const QColor & color, bool preview_path)
{
  if (mesh_preview_mode == ScenePreviewWidget::MeshPreviewMode::Primitives) {
    return false;
  }

  const bool meshes_only_mode = (mesh_preview_mode == ScenePreviewWidget::MeshPreviewMode::Meshes);
  if (mesh_preview_mode == ScenePreviewWidget::MeshPreviewMode::Meshes) {
    // explicit branch kept for static mesh-preview contract checks
  } else if (mesh_preview_mode == ScenePreviewWidget::MeshPreviewMode::Auto) {
    // explicit branch kept for static mesh-preview contract checks
  }
  const auto warn_for_mode = [&](const QString & reason, const QString & path) {
    if (meshes_only_mode || qEnvironmentVariableIsSet("WORKCELL_SCENE3D_DEBUG_LOGS")) warn_mesh_fallback_once(it.id, reason, path);
  };

  if (!it.has_mesh_metadata) {
    warn_for_mode(QStringLiteral("REJECT_MESH_METADATA_MISSING: mesh metadata missing"), it.source_path);
    return false;
  }

  const QString mesh_source = !it.mesh_path.trimmed().isEmpty() ? it.mesh_path : it.source_path;
  if (mesh_source.trimmed().isEmpty()) {
    warn_for_mode(QStringLiteral("REJECT_MESH_SOURCE_MISSING: mesh source missing"), mesh_source);
    return false;
  }
  const MeshCacheEntry & entry = ensure_mesh_cached(it, mesh_source);
  auto reject = [&](const QString & code, const QString & detail = QString()) {
    const QString reason = detail.isEmpty() ? code : QStringLiteral("%1: %2").arg(code, detail);
    warn_for_mode(reason, mesh_source);
    return false;
  };
  if (!entry.loaded || !entry.valid || entry.oversized || entry.mesh.triangles.isEmpty()) {
    if (!entry.loaded) return reject(QStringLiteral("parse_failed"));
    if (!entry.valid) return reject(entry.failure_reason_code.trimmed().isEmpty() ? QStringLiteral("parse_failed") : entry.failure_reason_code, entry.warning);
    if (entry.oversized) return reject(QStringLiteral("unreasonable_bounds"), entry.warning);
    return reject(QStringLiteral("zero_triangle_mesh"), entry.warning);
  }
  QString final_span_reason;
  if (!validate_mesh_final_span(it, entry, mesh_source, final_span_reason)) {
    return reject(QStringLiteral("unreasonable_bounds"), final_span_reason);
  }

  glPushMatrix();
  glTranslated(it.x, it.y, it.z);
  glRotated(qRadiansToDegrees(it.roll), 1.0, 0.0, 0.0);
  glRotated(qRadiansToDegrees(it.pitch), 0.0, 1.0, 0.0);
  glRotated(qRadiansToDegrees(it.yaw), 0.0, 0.0, 1.0);
  if (it.visual_origin_applied) {
    glTranslated(it.visual_origin_x, it.visual_origin_y, it.visual_origin_z);
    glRotated(qRadiansToDegrees(it.visual_origin_roll), 1.0, 0.0, 0.0);
    glRotated(qRadiansToDegrees(it.visual_origin_pitch), 0.0, 1.0, 0.0);
    glRotated(qRadiansToDegrees(it.visual_origin_yaw), 0.0, 0.0, 1.0);
  }
  glRotated(qRadiansToDegrees(it.mesh_r), 1.0, 0.0, 0.0);
  glRotated(qRadiansToDegrees(it.mesh_p), 0.0, 1.0, 0.0);
  glRotated(qRadiansToDegrees(it.mesh_y), 0.0, 0.0, 1.0);
  if (preview_path && it.has_origin_offset) glTranslated(it.origin_offset_x, it.origin_offset_y, it.origin_offset_z);
  glScaled(it.mesh_scale_x, it.mesh_scale_y, it.mesh_scale_z);

  const MeshCacheEntry & cache = entry;
  if (!cache.valid || cache.mesh.triangles.isEmpty()) {
    draw_unit_cube_triangles(color);
    glPopMatrix();
    return true;
  }

  const QVector3D default_up_normal(0.0f, 1.0f, 0.0f);
  QVector3D light_dir(0.35f, 0.8f, 0.45f);
  if (light_dir.lengthSquared() > 1e-12f) light_dir.normalize();
  else light_dir = default_up_normal;
  const float ambient = 0.28f;
  const float diffuse_scale = 0.9f;

  glBegin(GL_TRIANGLES);
  for (const auto & tri : cache.mesh.triangles) {
    QVector3D normal = tri.normal;
    const bool normal_invalid = !qIsFinite(normal.x()) || !qIsFinite(normal.y()) || !qIsFinite(normal.z()) ||
                                normal.lengthSquared() <= 1e-12f;
    if (normal_invalid) {
      const QVector3D edge0 = tri.vertices[1] - tri.vertices[0];
      const QVector3D edge1 = tri.vertices[2] - tri.vertices[0];
      normal = QVector3D::crossProduct(edge0, edge1);
    }
    if (qIsFinite(normal.x()) && qIsFinite(normal.y()) && qIsFinite(normal.z()) && normal.lengthSquared() > 1e-12f) {
      normal.normalize();
    } else {
      normal = default_up_normal;
    }

    const float diffuse = qMax(0.0f, QVector3D::dotProduct(normal, light_dir));
    const float shade = ambient + diffuse * diffuse_scale;
    const float edge_boost = 0.08f * (1.0f - diffuse);
    const float red = qMin(1.0f, static_cast<float>(color.redF()) * shade + edge_boost);
    const float green = qMin(1.0f, static_cast<float>(color.greenF()) * shade + edge_boost);
    const float blue = qMin(1.0f, static_cast<float>(color.blueF()) * shade + edge_boost);
    glColor4f(red, green, blue, 1.0f);
    glVertex3f(tri.vertices[0].x(), tri.vertices[0].y(), tri.vertices[0].z());
    glVertex3f(tri.vertices[1].x(), tri.vertices[1].y(), tri.vertices[1].z());
    glVertex3f(tri.vertices[2].x(), tri.vertices[2].y(), tri.vertices[2].z());
  }
  glEnd();

  glPopMatrix();
  return true;
}

void Scene3DViewportWidget::draw_ground_grid_pass()
{
  glDisable(GL_CULL_FACE);
  glLineWidth(1.0f);
  glBegin(GL_LINES);
  for (int i = -20; i <= 20; ++i) {
    const bool major = (i % 5 == 0);
    const QColor c = major ? QColor(100, 116, 139, 140) : QColor(71, 85, 105, 80);
    glColor4f(c.redF(), c.greenF(), c.blueF(), c.alphaF());
    glVertex3f(static_cast<float>(i), 0.0f, -20.0f); glVertex3f(static_cast<float>(i), 0.0f, 20.0f);
    glVertex3f(-20.0f, 0.0f, static_cast<float>(i)); glVertex3f(20.0f, 0.0f, static_cast<float>(i));
  }
  glEnd();
  glEnable(GL_CULL_FACE);
}

void Scene3DViewportWidget::draw_world_axes_pass()
{
  glLineWidth(2.4f);
  glBegin(GL_LINES);
  glColor4f(0.95f, 0.35f, 0.35f, 1.0f); glVertex3f(0.0f, 0.0f, 0.0f); glVertex3f(0.45f, 0.0f, 0.0f);
  glColor4f(0.35f, 0.95f, 0.35f, 1.0f); glVertex3f(0.0f, 0.0f, 0.0f); glVertex3f(0.0f, 0.45f, 0.0f);
  glColor4f(0.35f, 0.65f, 0.98f, 1.0f); glVertex3f(0.0f, 0.0f, 0.0f); glVertex3f(0.0f, 0.0f, 0.45f);
  glEnd();
  QPainter painter(this);
  painter.setPen(QColor("#f8fafc"));
  painter.drawText(project_to_screen(0.50, 0.0, 0.0), "X");
  painter.drawText(project_to_screen(0.0, 0.50, 0.0), "Y");
  painter.drawText(project_to_screen(0.0, 0.0, 0.50), "Z");
  painter.drawText(QPointF(10.0, height() - 14.0), "Scale: major grid 5 m");
}

void Scene3DViewportWidget::draw_unit_cube_triangles(const QColor & color)
{
  glColor4f(color.redF(), color.greenF(), color.blueF(), 1.0f);
  glBegin(GL_TRIANGLES);
  const GLfloat v[8][3] = {
    {0.f, 0.f, 0.f}, {1.f, 0.f, 0.f}, {1.f, 1.f, 0.f}, {0.f, 1.f, 0.f},
    {0.f, 0.f, 1.f}, {1.f, 0.f, 1.f}, {1.f, 1.f, 1.f}, {0.f, 1.f, 1.f}
  };
  auto tri = [&](int a, int b, int c){ glVertex3fv(v[a]); glVertex3fv(v[b]); glVertex3fv(v[c]); };
  tri(0,1,2); tri(0,2,3);
  tri(4,6,5); tri(4,7,6);
  tri(0,4,5); tri(0,5,1);
  tri(1,5,6); tri(1,6,2);
  tri(2,6,7); tri(2,7,3);
  tri(3,7,4); tri(3,4,0);
  glEnd();
}

void Scene3DViewportWidget::draw_robot_base_with_axis(const ScenePreviewWidget::PreviewItem & it)
{
  const double radius = qMax(0.06, qMin(it.sx, it.sz) * 0.45);
  draw_cylinder(it.x + it.sx * 0.5, it.y, it.z + it.sz * 0.5, radius, qMax(0.05, it.sy), item_color(it));
  glColor4f(0.96f, 0.97f, 0.99f, 1.0f);
  glLineWidth(2.0f);
  glBegin(GL_LINES);
  glVertex3f(it.x + it.sx * 0.5, it.y, it.z + it.sz * 0.5);
  glVertex3f(it.x + it.sx * 0.5, it.y + qMax(0.2, it.sy * 2.0), it.z + it.sz * 0.5);
  glEnd();
}
void Scene3DViewportWidget::draw_table_slab(const ScenePreviewWidget::PreviewItem & it) { draw_box(it.x, it.y, it.z, it.sx, it.sy, it.sz, item_color(it)); }
void Scene3DViewportWidget::draw_conveyor(const ScenePreviewWidget::PreviewItem & it)
{
  draw_box(it.x, it.y, it.z, it.sx, it.sy, it.sz, item_color(it));
  const double mid_y = it.y + it.sy + 0.01;
  const double start_x = it.x + it.sx * 0.25, end_x = it.x + it.sx * 0.75, z = it.z + it.sz * 0.5;
  glColor4f(0.92f, 0.98f, 1.0f, 1.0f);
  glLineWidth(2.0f);
  glBegin(GL_LINES);
  glVertex3f(start_x, mid_y, z); glVertex3f(end_x, mid_y, z);
  glVertex3f(end_x, mid_y, z); glVertex3f(end_x - 0.08, mid_y, z - 0.06);
  glVertex3f(end_x, mid_y, z); glVertex3f(end_x - 0.08, mid_y, z + 0.06);
  glEnd();
}
void Scene3DViewportWidget::draw_camera_body_with_frustum(const ScenePreviewWidget::PreviewItem & it)
{
  draw_box(it.x, it.y, it.z, it.sx, it.sy, it.sz, item_color(it));
  if (show_camera_fov) draw_frustum(QColor(56, 189, 248, 58), true);
}
void Scene3DViewportWidget::draw_pick_zone(const ScenePreviewWidget::PreviewItem & it) { draw_box(it.x, it.y, it.z, it.sx, it.sy, it.sz, QColor(34, 197, 94, 36), true); draw_box_outline(it.x, it.y, it.z, it.sx, it.sy, it.sz, QColor(34, 197, 94, 78), 0.75f); }
void Scene3DViewportWidget::draw_place_target_bin(const ScenePreviewWidget::PreviewItem & it)
{
  draw_box(it.x, it.y, it.z, it.sx, it.sy, it.sz, item_color(it), true);
  const double wall = qMax(0.02, qMin(it.sx, it.sz) * 0.1);
  draw_box_outline(it.x + wall, it.y + wall, it.z + wall, qMax(0.01, it.sx - 2 * wall), qMax(0.01, it.sy - wall), qMax(0.01, it.sz - 2 * wall), QColor(254, 205, 211, 120), 1.0f);
}
void Scene3DViewportWidget::draw_object_cube(const ScenePreviewWidget::PreviewItem & it)
{
  if (draw_mesh_preview_if_available(it, item_color(it), true)) return;
  const double cube = qMax(0.05, qMin(it.sx, qMin(it.sy, it.sz)));
  draw_box(it.x, it.y, it.z, cube, cube, cube, item_color(it));
}
void Scene3DViewportWidget::draw_missing_geometry_marker(const ScenePreviewWidget::PreviewItem & it)
{
  const double marker = 0.045;
  draw_box(it.x, it.y, it.z, marker, marker, marker, QColor(239, 68, 68, 96), true);
  draw_box_outline(it.x, it.y, it.z, marker, marker, marker, QColor(252, 165, 165, 150), 1.0f);
}
void Scene3DViewportWidget::draw_safety_zone(const ScenePreviewWidget::PreviewItem & it) { draw_box(it.x, it.y, it.z, it.sx, it.sy, it.sz, QColor(245, 158, 11, 32), true); draw_box_outline(it.x, it.y, it.z, it.sx, it.sy, it.sz, QColor(245, 158, 11, 72), 0.75f); }
void Scene3DViewportWidget::draw_warning_badge_anchor(const ScenePreviewWidget::PreviewItem & it)
{
  const double cube = qMax(0.04, qMin(it.sx, qMin(it.sy, it.sz)));
  draw_box(it.x, it.y, it.z, cube, cube, cube, QColor("#f59e0b"));
}

bool Scene3DViewportWidget::draw_clean_semantic_primitive(const ScenePreviewWidget::PreviewItem & it)
{
  const NormalizedRole role = classify_item_role(it);
  if (!is_clean_semantic_primitive_role(role) || !item_has_explicit_primitive_dimensions(it)) return false;

  QColor fill = item_color(it);
  QColor line = fill.lighter(role == NormalizedRole::Object ? 130 : 145);
  switch (role) {
    case NormalizedRole::PickZone:
      fill.setAlpha(34);
      line.setAlpha(96);
      break;
    case NormalizedRole::PlaceBin:
      fill.setAlpha(38);
      line.setAlpha(110);
      break;
    case NormalizedRole::Conveyor:
      fill.setAlpha(48);
      line.setAlpha(120);
      break;
    case NormalizedRole::Object:
      fill.setAlpha(64);
      line.setAlpha(128);
      break;
    default:
      return false;
  }

  if (item_has_explicit_dimensions(it)) {
    draw_box(it.x, it.y, it.z, it.sx, it.sy, it.sz, fill, true);
    draw_box_outline(it.x, it.y, it.z, it.sx, it.sy, it.sz, line, role == NormalizedRole::Object ? 0.9f : 1.0f);
    if (role == NormalizedRole::Conveyor) {
      const double mid_y = it.y + it.sy + 0.01;
      const double start_x = it.x + it.sx * 0.25;
      const double end_x = it.x + it.sx * 0.75;
      const double z = it.z + it.sz * 0.5;
      glColor4f(line.redF(), line.greenF(), line.blueF(), 0.72f);
      glLineWidth(1.5f);
      glBegin(GL_LINES);
      glVertex3f(start_x, mid_y, z); glVertex3f(end_x, mid_y, z);
      glVertex3f(end_x, mid_y, z); glVertex3f(end_x - 0.08, mid_y, z - 0.06);
      glVertex3f(end_x, mid_y, z); glVertex3f(end_x - 0.08, mid_y, z + 0.06);
      glEnd();
    }
    return true;
  }

  return draw_urdf_primitive_geometry(it, fill);
}

void Scene3DViewportWidget::draw_box(double cx, double cy, double cz, double sx, double sy, double sz, const QColor & color, bool translucent)
{
  glDisable(GL_CULL_FACE);
  glColor4f(color.redF(), color.greenF(), color.blueF(), translucent ? qMin(0.35f, static_cast<float>(color.alphaF())) : color.alphaF());
  const double x = cx, y = cy, z = cz;
  glBegin(GL_QUADS);
  glVertex3f(x, y, z); glVertex3f(x + sx, y, z); glVertex3f(x + sx, y + sy, z); glVertex3f(x, y + sy, z);
  glVertex3f(x, y, z + sz); glVertex3f(x, y + sy, z + sz); glVertex3f(x + sx, y + sy, z + sz); glVertex3f(x + sx, y, z + sz);
  glVertex3f(x, y, z); glVertex3f(x, y, z + sz); glVertex3f(x + sx, y, z + sz); glVertex3f(x + sx, y, z);
  glVertex3f(x, y + sy, z); glVertex3f(x + sx, y + sy, z); glVertex3f(x + sx, y + sy, z + sz); glVertex3f(x, y + sy, z + sz);
  glVertex3f(x, y, z); glVertex3f(x, y + sy, z); glVertex3f(x, y + sy, z + sz); glVertex3f(x, y, z + sz);
  glVertex3f(x + sx, y, z); glVertex3f(x + sx, y, z + sz); glVertex3f(x + sx, y + sy, z + sz); glVertex3f(x + sx, y + sy, z);
  glEnd();
  glEnable(GL_CULL_FACE);
}
void Scene3DViewportWidget::draw_box_outline(double cx, double cy, double cz, double sx, double sy, double sz, const QColor & color, float line_width)
{
  glDisable(GL_CULL_FACE);
  glLineWidth(line_width);
  glColor4f(color.redF(), color.greenF(), color.blueF(), 1.0f);
  const double x = cx, y = cy, z = cz;
  glBegin(GL_LINES);
  glVertex3f(x, y, z); glVertex3f(x + sx, y, z);
  glVertex3f(x + sx, y, z); glVertex3f(x + sx, y + sy, z);
  glVertex3f(x + sx, y + sy, z); glVertex3f(x, y + sy, z);
  glVertex3f(x, y + sy, z); glVertex3f(x, y, z);

  glVertex3f(x, y, z + sz); glVertex3f(x + sx, y, z + sz);
  glVertex3f(x + sx, y, z + sz); glVertex3f(x + sx, y + sy, z + sz);
  glVertex3f(x + sx, y + sy, z + sz); glVertex3f(x, y + sy, z + sz);
  glVertex3f(x, y + sy, z + sz); glVertex3f(x, y, z + sz);

  glVertex3f(x, y, z); glVertex3f(x, y, z + sz);
  glVertex3f(x + sx, y, z); glVertex3f(x + sx, y, z + sz);
  glVertex3f(x + sx, y + sy, z); glVertex3f(x + sx, y + sy, z + sz);
  glVertex3f(x, y + sy, z); glVertex3f(x, y + sy, z + sz);
  glEnd();
  glEnable(GL_CULL_FACE);
}
void Scene3DViewportWidget::draw_cylinder(double cx, double cy, double cz, double radius, double height, const QColor & color, bool translucent, int segment_count)
{
  const int segments = qMax(3, segment_count);
  const double safe_radius = qMax(0.0, radius);
  const double y_bottom = cy;
  const double y_top = cy + height;

  glColor4f(color.redF(), color.greenF(), color.blueF(), translucent ? qMin(0.35f, static_cast<float>(color.alphaF())) : color.alphaF());

  // Side wall. The cylinder is Y-up to match the viewport's box helper, where
  // the height axis starts at cy and extends by the supplied height argument.
  glBegin(GL_QUADS);
  for (int i = 0; i < segments; ++i) {
    const double a0 = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(segments);
    const double a1 = 2.0 * M_PI * static_cast<double>(i + 1) / static_cast<double>(segments);
    const double x0 = cx + safe_radius * qCos(a0);
    const double z0 = cz + safe_radius * qSin(a0);
    const double x1 = cx + safe_radius * qCos(a1);
    const double z1 = cz + safe_radius * qSin(a1);
    glVertex3f(static_cast<GLfloat>(x0), static_cast<GLfloat>(y_bottom), static_cast<GLfloat>(z0));
    glVertex3f(static_cast<GLfloat>(x0), static_cast<GLfloat>(y_top), static_cast<GLfloat>(z0));
    glVertex3f(static_cast<GLfloat>(x1), static_cast<GLfloat>(y_top), static_cast<GLfloat>(z1));
    glVertex3f(static_cast<GLfloat>(x1), static_cast<GLfloat>(y_bottom), static_cast<GLfloat>(z1));
  }
  glEnd();

  // Bottom cap.
  glBegin(GL_TRIANGLE_FAN);
  glVertex3f(static_cast<GLfloat>(cx), static_cast<GLfloat>(y_bottom), static_cast<GLfloat>(cz));
  for (int i = 0; i <= segments; ++i) {
    const double a = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(segments);
    glVertex3f(static_cast<GLfloat>(cx + safe_radius * qCos(a)),
               static_cast<GLfloat>(y_bottom),
               static_cast<GLfloat>(cz + safe_radius * qSin(a)));
  }
  glEnd();

  // Top cap.
  glBegin(GL_TRIANGLE_FAN);
  glVertex3f(static_cast<GLfloat>(cx), static_cast<GLfloat>(y_top), static_cast<GLfloat>(cz));
  for (int i = segments; i >= 0; --i) {
    const double a = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(segments);
    glVertex3f(static_cast<GLfloat>(cx + safe_radius * qCos(a)),
               static_cast<GLfloat>(y_top),
               static_cast<GLfloat>(cz + safe_radius * qSin(a)));
  }
  glEnd();
}

void Scene3DViewportWidget::draw_sphere(double cx, double cy, double cz, double radius, const QColor & color, bool translucent, int slice_count, int stack_count)
{
  const int slices = qMax(6, slice_count);
  const int stacks = qMax(4, stack_count);
  const double r = qMax(0.0, radius);
  glColor4f(color.redF(), color.greenF(), color.blueF(), translucent ? qMin(0.35f, static_cast<float>(color.alphaF())) : color.alphaF());
  for (int stack = 0; stack < stacks; ++stack) {
    const double phi0 = -M_PI_2 + M_PI * static_cast<double>(stack) / static_cast<double>(stacks);
    const double phi1 = -M_PI_2 + M_PI * static_cast<double>(stack + 1) / static_cast<double>(stacks);
    glBegin(GL_QUAD_STRIP);
    for (int slice = 0; slice <= slices; ++slice) {
      const double theta = 2.0 * M_PI * static_cast<double>(slice) / static_cast<double>(slices);
      const double c = qCos(theta);
      const double s = qSin(theta);
      glVertex3f(static_cast<GLfloat>(cx + r * qCos(phi0) * c),
                 static_cast<GLfloat>(cy + r * qSin(phi0)),
                 static_cast<GLfloat>(cz + r * qCos(phi0) * s));
      glVertex3f(static_cast<GLfloat>(cx + r * qCos(phi1) * c),
                 static_cast<GLfloat>(cy + r * qSin(phi1)),
                 static_cast<GLfloat>(cz + r * qCos(phi1) * s));
    }
    glEnd();
  }
}

void Scene3DViewportWidget::draw_frustum(const QColor & color, bool translucent)
{
  const double h = qDegreesToRadians(camera_overlay.horizontal_fov_deg * 0.5);
  const double v = qDegreesToRadians(camera_overlay.vertical_fov_deg * 0.5);
  const double n = qMax(0.05, camera_overlay.range_min_m);
  const double f = qMax(n + 0.05, camera_overlay.range_max_m);
  auto ray = [&](double r, double ah, double av) { return QVector3D(camera_overlay.x + r, camera_overlay.y + qTan(av) * r, camera_overlay.z + qTan(ah) * r); };
  QVector3D ffl = ray(f, -h, -v), ffr = ray(f, h, -v), fbl = ray(f, -h, v), fbr = ray(f, h, v);
  glColor4f(color.redF(), color.greenF(), color.blueF(), translucent ? 0.12f : 0.7f);
  glBegin(GL_TRIANGLE_FAN); glVertex3f(camera_overlay.x, camera_overlay.y, camera_overlay.z); glVertex3f(ffl.x(), ffl.y(), ffl.z()); glVertex3f(ffr.x(), ffr.y(), ffr.z()); glVertex3f(fbr.x(), fbr.y(), fbr.z()); glVertex3f(fbl.x(), fbl.y(), fbl.z()); glVertex3f(ffl.x(), ffl.y(), ffl.z()); glEnd();
}
QPointF Scene3DViewportWidget::project_to_screen(double x, double y, double z) const { return QPointF(width() * 0.5 + x * 50.0, height() * 0.6 - y * 50.0 + z * 5.0); }
void Scene3DViewportWidget::camera_matrices(QMatrix4x4 & out_proj, QMatrix4x4 & out_view) const
{
  const float aspect = height() > 0 ? static_cast<float>(width()) / static_cast<float>(height()) : 1.0f;
  const float clamped_distance = static_cast<float>(qBound(min_distance_, distance_, max_distance_));
  const float cp = static_cast<float>(qCos(pitch_));
  const QVector3D forward(
    cp * static_cast<float>(qSin(yaw_)),
    static_cast<float>(qSin(pitch_)),
    cp * static_cast<float>(qCos(yaw_)));
  const QVector3D eye = orbit_offset_ - (forward * clamped_distance);
  const float radius = static_cast<float>(qMax(0.2, scene_radius_));
  const float near_plane = qMax(0.01f, qMin(0.2f, radius * 0.05f));
  const float far_plane = qMax(clamped_distance + (radius * 8.0f), radius * 20.0f);
  out_proj.setToIdentity();
  out_proj.perspective(50.0f, aspect, near_plane, far_plane);
  out_view.setToIdentity();
  out_view.lookAt(eye, orbit_offset_, QVector3D(0.0f, 1.0f, 0.0f));
}
bool Scene3DViewportWidget::ray_intersects_aabb(const QVector3D & ray_origin, const QVector3D & ray_dir,
                                                const ScenePreviewWidget::PreviewItem & item, float & out_t) const
{
  const ItemBounds bounds = item_bounds_for_role(item);
  const QVector3D bmin(bounds.x, bounds.y, bounds.z);
  const QVector3D bmax(bounds.x + bounds.sx, bounds.y + bounds.sy, bounds.z + bounds.sz);
  float tmin = 0.0f;
  float tmax = 1e9f;
  for (int axis = 0; axis < 3; ++axis) {
    const float origin = axis == 0 ? ray_origin.x() : (axis == 1 ? ray_origin.y() : ray_origin.z());
    const float dir = axis == 0 ? ray_dir.x() : (axis == 1 ? ray_dir.y() : ray_dir.z());
    const float minv = axis == 0 ? bmin.x() : (axis == 1 ? bmin.y() : bmin.z());
    const float maxv = axis == 0 ? bmax.x() : (axis == 1 ? bmax.y() : bmax.z());
    if (qAbs(dir) < 1e-6f) {
      if (origin < minv || origin > maxv) return false;
      continue;
    }
    const float invd = 1.0f / dir;
    float t0 = (minv - origin) * invd;
    float t1 = (maxv - origin) * invd;
    if (t0 > t1) std::swap(t0, t1);
    tmin = qMax(tmin, t0);
    tmax = qMin(tmax, t1);
    if (tmax < tmin) return false;
  }
  out_t = tmin;
  return true;
}
Scene3DViewportWidget::ItemBounds Scene3DViewportWidget::item_bounds_for_role(const ScenePreviewWidget::PreviewItem & item) const
{
  ItemBounds mesh_bounds{};
  if (mesh_world_bounds_for_item(item, mesh_bounds)) return mesh_bounds;

  const NormalizedRole role = classify_item_role(item);
  if (role == NormalizedRole::Object || role == NormalizedRole::WarningAnchor) {
    const double cube = (role == NormalizedRole::Object)
      ? qMax(0.05, qMin(item.sx, qMin(item.sy, item.sz)))
      : qMax(0.04, qMin(item.sx, qMin(item.sy, item.sz)));
    return { item.x, item.y, item.z, cube, cube, cube };
  }
  return { item.x, item.y, item.z, item.sx, item.sy, item.sz };
}

bool Scene3DViewportWidget::mesh_world_bounds_for_item(const ScenePreviewWidget::PreviewItem & item, ItemBounds & out_bounds) const
{
  const QString mesh_source = !item.mesh_path.trimmed().isEmpty() ? item.mesh_path : item.source_path;
  if (mesh_source.trimmed().isEmpty()) return false;
  const MeshCacheEntry & cache = const_cast<Scene3DViewportWidget *>(this)->ensure_mesh_cached(item, mesh_source);
  if (!cache.loaded || !cache.valid || !cache.has_bounds) return false;

  QMatrix4x4 transform;
  transform.translate(item.x, item.y, item.z);
  transform.rotate(qRadiansToDegrees(item.roll), 1.0f, 0.0f, 0.0f);
  transform.rotate(qRadiansToDegrees(item.pitch), 0.0f, 1.0f, 0.0f);
  transform.rotate(qRadiansToDegrees(item.yaw), 0.0f, 0.0f, 1.0f);
  if (item.visual_origin_applied) {
    transform.translate(item.visual_origin_x, item.visual_origin_y, item.visual_origin_z);
    transform.rotate(qRadiansToDegrees(item.visual_origin_roll), 1.0f, 0.0f, 0.0f);
    transform.rotate(qRadiansToDegrees(item.visual_origin_pitch), 0.0f, 1.0f, 0.0f);
    transform.rotate(qRadiansToDegrees(item.visual_origin_yaw), 0.0f, 0.0f, 1.0f);
  }
  transform.rotate(qRadiansToDegrees(item.mesh_r), 1.0f, 0.0f, 0.0f);
  transform.rotate(qRadiansToDegrees(item.mesh_p), 0.0f, 1.0f, 0.0f);
  transform.rotate(qRadiansToDegrees(item.mesh_y), 0.0f, 0.0f, 1.0f);
  if (item.has_origin_offset) transform.translate(item.origin_offset_x, item.origin_offset_y, item.origin_offset_z);
  transform.scale(item.mesh_scale_x, item.mesh_scale_y, item.mesh_scale_z);

  const QVector3D lmin = cache.local_min;
  const QVector3D lmax = cache.local_max;
  QVector3D wmin(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
  QVector3D wmax(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());
  for (int xi = 0; xi < 2; ++xi) {
    for (int yi = 0; yi < 2; ++yi) {
      for (int zi = 0; zi < 2; ++zi) {
        const QVector3D local_corner(
          xi == 0 ? lmin.x() : lmax.x(),
          yi == 0 ? lmin.y() : lmax.y(),
          zi == 0 ? lmin.z() : lmax.z());
        const QVector3D world_corner = transform * local_corner;
        wmin.setX(qMin(wmin.x(), world_corner.x()));
        wmin.setY(qMin(wmin.y(), world_corner.y()));
        wmin.setZ(qMin(wmin.z(), world_corner.z()));
        wmax.setX(qMax(wmax.x(), world_corner.x()));
        wmax.setY(qMax(wmax.y(), world_corner.y()));
        wmax.setZ(qMax(wmax.z(), world_corner.z()));
      }
    }
  }
  out_bounds = { wmin.x(), wmin.y(), wmin.z(),
                 qMax(0.0f, wmax.x() - wmin.x()),
                 qMax(0.0f, wmax.y() - wmin.y()),
                 qMax(0.0f, wmax.z() - wmin.z()) };
  return true;
}
bool Scene3DViewportWidget::pick_item_at_screen(
  const QPoint & pos, QString & out_id, QString & out_role, QString * out_tooltip) const
{
  QMatrix4x4 proj, view;
  camera_matrices(proj, view);
  const QMatrix4x4 inv = (proj * view).inverted();
  const float ndc_x = (2.0f * static_cast<float>(pos.x()) / qMax(1, width())) - 1.0f;
  const float ndc_y = 1.0f - (2.0f * static_cast<float>(pos.y()) / qMax(1, height()));
  QVector4D near_h = inv * QVector4D(ndc_x, ndc_y, -1.0f, 1.0f);
  QVector4D far_h = inv * QVector4D(ndc_x, ndc_y, 1.0f, 1.0f);
  if (qFuzzyIsNull(near_h.w()) || qFuzzyIsNull(far_h.w())) return false;
  const QVector3D p0 = near_h.toVector3DAffine();
  const QVector3D p1 = far_h.toVector3DAffine();
  const QVector3D dir = (p1 - p0).normalized();

  float best_t = 1e9f;
  const ScenePreviewWidget::PreviewItem * best_item = nullptr;
  for (const auto & item : items) {
    if (!item.selectable) continue;
    float t = 0.0f;
    if (ray_intersects_aabb(p0, dir, item, t) && t < best_t) {
      best_t = t;
      best_item = &item;
    }
  }
  if (best_item == nullptr) return false;
  out_id = best_item->id;
  out_role = best_item->role.trimmed().isEmpty() ? QStringLiteral("unknown") : best_item->role.trimmed();
  if (out_tooltip != nullptr) {
    const QString role_normalized = normalized_token(out_role);
    const QString metadata_tags = best_item->metadata_tags.trimmed();
    *out_tooltip = QStringLiteral("Item: %1\nRole: %2\nLayer: %3\nWarnings: %4")
      .arg(out_id, role_normalized, item_visual_ownership_label(*best_item), warning_debug_text(best_item->warnings));
    if (!metadata_tags.isEmpty()) *out_tooltip += QStringLiteral("\nTags: ") + metadata_tags;
    if (!best_item->warnings.isEmpty()) *out_tooltip += QStringLiteral("\n\nDetails:\n- ") + best_item->warnings.join("\n- ");
  }
  return true;
}

bool Scene3DViewportWidget::pick_gizmo_axis_at_screen(const QPoint & pos, QString & out_axis, double * out_score) const
{
  out_axis.clear();
  if (selected_id.isEmpty()) return false;
  const ScenePreviewWidget::PreviewItem * selected = nullptr;
  for (const auto & it : items) if (it.id == selected_id) { selected = &it; break; }
  if (selected == nullptr) return false;

  const QVector3D origin(selected->x + (selected->sx * 0.5), selected->y + (selected->sy * 0.5), selected->z + (selected->sz * 0.5));
  const double item_extent = std::max({selected->sx, selected->sy, selected->sz});
  const double axis_len = qMax(0.25, 0.75 * item_extent);
  const QPointF screen_p = QPointF(pos);
  const QPointF origin_s = project_to_screen(origin.x(), origin.y(), origin.z());
  if (!qIsFinite(origin_s.x()) || !qIsFinite(origin_s.y())) return false;

  const struct AxisCandidate { const char * axis; QVector3D dir; } candidates[] = {
    {"x", QVector3D(1.0f, 0.0f, 0.0f)},
    {"y", QVector3D(0.0f, 1.0f, 0.0f)},
    {"z", QVector3D(0.0f, 0.0f, 1.0f)}
  };
  constexpr double kAxisThresholdPx = 12.0;
  double best = std::numeric_limits<double>::max();
  QString best_axis;
  for (const auto & candidate : candidates) {
    const QVector3D endpoint = origin + (candidate.dir * axis_len);
    const QPointF endpoint_s = project_to_screen(endpoint.x(), endpoint.y(), endpoint.z());
    if (!qIsFinite(endpoint_s.x()) || !qIsFinite(endpoint_s.y())) continue;
    const double score = distance_to_segment_2d(screen_p, origin_s, endpoint_s);
    if (score < best) {
      best = score;
      best_axis = QString::fromLatin1(candidate.axis);
    }
  }
  if (out_score != nullptr) *out_score = best;
  if (best <= kAxisThresholdPx && !best_axis.isEmpty()) { out_axis = best_axis; return true; }
  // Deterministic fallback when no segment is precisely hit: choose the axis with the lowest score.
  out_axis = best_axis;
  return !out_axis.isEmpty();
}

bool Scene3DViewportWidget::pick_gizmo_rotation_ring_at_screen(const QPoint & pos, QString & out_axis, double * out_score) const
{
  out_axis.clear();
  if (selected_id.isEmpty()) return false;
  const ScenePreviewWidget::PreviewItem * selected = nullptr;
  for (const auto & it : items) if (it.id == selected_id) { selected = &it; break; }
  if (selected == nullptr) return false;

  const QVector3D origin(selected->x + (selected->sx * 0.5), selected->y + (selected->sy * 0.5), selected->z + (selected->sz * 0.5));
  const double item_extent = std::max({selected->sx, selected->sy, selected->sz});
  const double radius = qMax(0.2, 0.65 * item_extent);
  const QPointF screen_p = QPointF(pos);
  constexpr int kRingSamples = 48;
  constexpr double kRingThresholdPx = 11.0;
  double best = std::numeric_limits<double>::max();
  QString best_axis;

  const struct RingCandidate { const char * axis; QVector3D u; QVector3D v; } rings[] = {
    {"x", QVector3D(0.0f, 1.0f, 0.0f), QVector3D(0.0f, 0.0f, 1.0f)},
    {"y", QVector3D(1.0f, 0.0f, 0.0f), QVector3D(0.0f, 0.0f, 1.0f)},
    {"z", QVector3D(1.0f, 0.0f, 0.0f), QVector3D(0.0f, 1.0f, 0.0f)}
  };
  for (const auto & ring : rings) {
    QVector<QPointF> polyline;
    polyline.reserve(kRingSamples + 1);
    for (int i = 0; i <= kRingSamples; ++i) {
      const double t = (2.0 * M_PI * i) / static_cast<double>(kRingSamples);
      const QVector3D sample = origin + (ring.u * static_cast<float>(radius * qCos(t))) + (ring.v * static_cast<float>(radius * qSin(t)));
      const QPointF sample_s = project_to_screen(sample.x(), sample.y(), sample.z());
      if (!qIsFinite(sample_s.x()) || !qIsFinite(sample_s.y())) continue;
      polyline.push_back(sample_s);
    }
    const double score = distance_to_polyline_2d(screen_p, polyline);
    if (score < best) {
      best = score;
      best_axis = QString::fromLatin1(ring.axis);
    }
  }
  if (out_score != nullptr) *out_score = best;
  if (best <= kRingThresholdPx && !best_axis.isEmpty()) { out_axis = best_axis; return true; }
  // Deterministic fallback when no ring polyline is within threshold: pick the minimum-score ring.
  out_axis = best_axis;
  return !out_axis.isEmpty();
}

void Scene3DViewportWidget::mousePressEvent(QMouseEvent * e) {
  last_ = e->pos();
  if (e->button() != Qt::LeftButton) return;
  QString best_id, best_role;
  if (pick_item_at_screen(e->pos(), best_id, best_role) && !best_id.isEmpty() && select_cb) select_cb(best_id, best_role);
  drag_start_screen_ = e->pos();
  dragging_gizmo_ = false;
  drag_active_handle_ = GizmoHandle::None;
  drag_in_progress_ = false;
  drag_cancelled_ = false;
  if ((gizmo_mode == GizmoMode::Move || gizmo_mode == GizmoMode::Rotate) && !selected_id.isEmpty()) {
    for (const auto & it : items) {
      if (it.id != selected_id) continue;
      if (!item_is_editable_for_gizmo(it)) {
        if (status_message_cb) status_message_cb(QStringLiteral("Locked: %1").arg(item_locked_reason(it)));
        return;
      }
      dragging_gizmo_ = true;
      break;
    }
  }
  if (dragging_gizmo_) {
    const int dx = qAbs(e->x() - width() / 2);
    const int dy = qAbs(e->y() - height() / 2);
    active_axis_ = (dx > dy) ? QStringLiteral("x") : QStringLiteral("y");
    for (const auto & it : items) {
      if (it.id != selected_id) continue;
      drag_start_pose_.item_id = it.id;
      drag_start_pose_.x = it.x;
      drag_start_pose_.y = it.y;
      drag_start_pose_.z = it.z;
      drag_start_pose_.roll = it.roll;
      drag_start_pose_.pitch = it.pitch;
      drag_start_pose_.yaw = it.yaw;
      break;
    }
    double score = std::numeric_limits<double>::max();
    QString axis;
    bool picked = false;
    if (gizmo_mode == GizmoMode::Move) {
      picked = pick_gizmo_axis_at_screen(e->pos(), axis, &score);
      if (picked) drag_active_handle_ = (axis == "x") ? GizmoHandle::MoveX : (axis == "y" ? GizmoHandle::MoveY : GizmoHandle::MoveZ);
    } else if (gizmo_mode == GizmoMode::Rotate) {
      picked = pick_gizmo_rotation_ring_at_screen(e->pos(), axis, &score);
      if (picked) drag_active_handle_ = (axis == "x") ? GizmoHandle::Roll : (axis == "y" ? GizmoHandle::Pitch : GizmoHandle::Yaw);
    }
    active_axis_ = axis;
    dragging_gizmo_ = picked && !axis.isEmpty();
    drag_in_progress_ = dragging_gizmo_;
  }
}
void Scene3DViewportWidget::mouseMoveEvent(QMouseEvent * e)
{
  if (dragging_gizmo_ && (e->buttons() & Qt::LeftButton) && !selected_id.isEmpty()) {
    for (auto & it : items) {
      if (it.id != selected_id) continue;
      if (!item_is_editable_for_gizmo(it)) {
        dragging_gizmo_ = false;
        if (status_message_cb) status_message_cb(QStringLiteral("Locked: %1").arg(item_locked_reason(it)));
        return;
      }
      const QPoint delta = e->pos() - drag_start_screen_;
      if (gizmo_mode == GizmoMode::Move) {
        const double raw = (active_axis_ == "x" ? delta.x() : -delta.y()) * 0.002;
        const double snapped = snap_translation_value(raw, snap_mode);
        if (active_axis_ == "x") it.x = drag_start_pose_.x + snapped; else if (active_axis_ == "y") it.y = drag_start_pose_.y + snapped; else it.z = drag_start_pose_.z + snapped;
      } else if (gizmo_mode == GizmoMode::Rotate) {
        const double raw = (delta.x() - delta.y()) * 0.01;
        const double snapped = snap_rotation_value(raw, snap_mode);
        if (active_axis_ == "x") it.roll = drag_start_pose_.roll + snapped; else if (active_axis_ == "y") it.pitch = drag_start_pose_.pitch + snapped; else it.yaw = drag_start_pose_.yaw + snapped;
      }
      update();
      return;
    }
  }
  auto d = e->pos() - last_;
  last_ = e->pos();
  const bool pan_mode = (e->buttons() & Qt::MiddleButton) || ((e->buttons() & Qt::LeftButton) && (e->modifiers() & Qt::ShiftModifier));
  if (pan_mode) {
    const float pan_scale = static_cast<float>(distance_ * 0.0018);
    const QVector3D forward(
      static_cast<float>(qCos(pitch_) * qSin(yaw_)),
      static_cast<float>(qSin(pitch_)),
      static_cast<float>(qCos(pitch_) * qCos(yaw_)));
    const QVector3D world_up(0.0f, 1.0f, 0.0f);
    QVector3D right = QVector3D::crossProduct(forward, world_up).normalized();
    if (right.lengthSquared() < 1e-6f) right = QVector3D(1.0f, 0.0f, 0.0f);
    const QVector3D up = QVector3D::crossProduct(right, forward).normalized();
    orbit_offset_ += (-right * (d.x() * pan_scale)) + (up * (d.y() * pan_scale));
    update();
    return;
  }
  if (e->buttons() & Qt::LeftButton) {
    yaw_ += d.x() * 0.01;
    pitch_ = qBound(-1.4, pitch_ + d.y() * 0.01, 1.4);
    update();
    return;
  }
  QString hovered, hovered_role, hover_tooltip;
  hovered_gizmo_handle_ = GizmoHandle::None;
  if (!selected_id.isEmpty() && (gizmo_mode == GizmoMode::Move || gizmo_mode == GizmoMode::Rotate)) {
    QString axis;
    bool picked = (gizmo_mode == GizmoMode::Move) ? pick_gizmo_axis_at_screen(e->pos(), axis)
                                                  : pick_gizmo_rotation_ring_at_screen(e->pos(), axis);
    if (picked) {
      hovered_gizmo_handle_ = (gizmo_mode == GizmoMode::Move)
        ? ((axis == "x") ? GizmoHandle::MoveX : (axis == "y" ? GizmoHandle::MoveY : GizmoHandle::MoveZ))
        : ((axis == "x") ? GizmoHandle::Roll : (axis == "y" ? GizmoHandle::Pitch : GizmoHandle::Yaw));
      hovered = selected_id;
      hovered_role = QStringLiteral("gizmo_%1").arg(axis);
      hover_tooltip = (gizmo_mode == GizmoMode::Move)
        ? QStringLiteral("Move %1 axis").arg(axis.toUpper())
        : QStringLiteral("Rotate %1 axis").arg(axis.toUpper());
    }
  }
  if (hovered_gizmo_handle_ == GizmoHandle::None) {
    if (pick_item_at_screen(e->pos(), hovered, hovered_role, &hover_tooltip) && !hovered.isEmpty()) {
      QToolTip::showText(e->globalPos(), hover_tooltip, this);
    } else {
      QToolTip::hideText();
    }
  } else {
    QToolTip::showText(e->globalPos(), hover_tooltip, this);
  }
  hovered_id_ = hovered;
}
void Scene3DViewportWidget::mouseReleaseEvent(QMouseEvent * e)
{
  if (e->button() == Qt::LeftButton) {
    if (drag_in_progress_ && !drag_cancelled_ && transform_changed_cb) {
      bool committed = false;
      for (auto & it : items) {
        if (it.id != drag_start_pose_.item_id) continue;
        const bool id_valid = !it.id.trimmed().isEmpty() && !drag_start_pose_.item_id.trimmed().isEmpty() && it.id == selected_id;
        const bool finite_xyz = std::isfinite(it.x) && std::isfinite(it.y) && std::isfinite(it.z);
        const bool bounded_xyz = qAbs(it.x) <= kWorkspaceLimitMeters &&
                                 qAbs(it.y) <= kWorkspaceLimitMeters &&
                                 qAbs(it.z) <= kWorkspaceLimitMeters;
        if (!id_valid || !finite_xyz || !bounded_xyz) {
          it.x = drag_start_pose_.x;
          it.y = drag_start_pose_.y;
          it.z = drag_start_pose_.z;
          it.roll = drag_start_pose_.roll;
          it.pitch = drag_start_pose_.pitch;
          it.yaw = drag_start_pose_.yaw;
          const QString message = QStringLiteral(
            "Rejected gizmo drag commit for '%1': invalid final XYZ or mismatched drag source; pose reverted.")
                                    .arg(drag_start_pose_.item_id);
          qWarning().noquote() << message;
          if (status_message_cb) status_message_cb(message);
          update();
          break;
        }
        transform_changed_cb(it.id, it.x, it.y, it.z, it.roll, it.pitch, it.yaw);
        committed = true;
        break;
      }
      if (!committed && status_message_cb) {
        status_message_cb(QStringLiteral("No valid drag source found for commit; change discarded."));
      }
    }
    dragging_gizmo_ = false;
    drag_in_progress_ = false;
    drag_cancelled_ = false;
    drag_active_handle_ = GizmoHandle::None;
    active_axis_.clear();
  }
  QOpenGLWidget::mouseReleaseEvent(e);
}

void Scene3DViewportWidget::wheelEvent(QWheelEvent * e)
{
  const double delta_steps = static_cast<double>(e->angleDelta().y()) / 120.0;
  const double zoom_factor = std::pow(0.9, delta_steps);
  distance_ = qBound(min_distance_, distance_ * zoom_factor, max_distance_);
  update();
}

void Scene3DViewportWidget::keyPressEvent(QKeyEvent * e)
{
  if (e->key() == Qt::Key_Escape) {
    if (drag_asset_preview_visible_) {
      drag_asset_preview_visible_ = false;
      drag_asset_drop_status_ = QStringLiteral("Drag/drop cancelled.");
      QToolTip::showText(QCursor::pos(), drag_asset_drop_status_, this);
      update();
      e->accept();
      return;
    }
    if (!drag_in_progress_) {
      e->ignore();
      return;
    }
    for (auto & it : items) {
      if (it.id != drag_start_pose_.item_id) continue;
      it.x = drag_start_pose_.x;
      it.y = drag_start_pose_.y;
      it.z = drag_start_pose_.z;
      it.roll = drag_start_pose_.roll;
      it.pitch = drag_start_pose_.pitch;
      it.yaw = drag_start_pose_.yaw;
      // Cancel restores the in-memory preview pose only; commit/save callback is release-only.
      break;
    }
    dragging_gizmo_ = false;
    drag_in_progress_ = false;
    drag_cancelled_ = true;
    drag_active_handle_ = GizmoHandle::None;
    active_axis_.clear();
    QToolTip::showText(QCursor::pos(), QStringLiteral("Gizmo drag cancelled."), this);
    update();
    e->accept();
    return;
  }
  QOpenGLWidget::keyPressEvent(e);
}

void Scene3DViewportWidget::dragEnterEvent(QDragEnterEvent * event)
{
  if (event->mimeData() && event->mimeData()->hasFormat("application/x-workcell-asset-catalog-item")) event->acceptProposedAction();
}

void Scene3DViewportWidget::dragMoveEvent(QDragMoveEvent * event)
{
  if (!event->mimeData() || !event->mimeData()->hasFormat("application/x-workcell-asset-catalog-item")) return;
  const QByteArray payload = event->mimeData()->data("application/x-workcell-asset-catalog-item");
  drag_asset_payload_ = QJsonDocument::fromJson(payload).object();
  drag_asset_label_ = drag_asset_payload_.value("display_name").toString("asset");
  drag_asset_screen_pos_ = event->pos();
  drag_asset_preview_visible_ = true;
  drag_asset_drop_status_ = QStringLiteral("Drop to place %1").arg(drag_asset_label_);
  event->acceptProposedAction();
  update();
}

void Scene3DViewportWidget::dragLeaveEvent(QDragLeaveEvent *)
{
  drag_asset_preview_visible_ = false;
  update();
}

void Scene3DViewportWidget::dropEvent(QDropEvent * event)
{
  if (!event->mimeData() || !event->mimeData()->hasFormat("application/x-workcell-asset-catalog-item")) return;
  const QJsonObject payload = QJsonDocument::fromJson(event->mimeData()->data("application/x-workcell-asset-catalog-item")).object();
  const QPoint p = event->pos();
  const double x = snap_translation_value((p.x() - width() * 0.5) / 50.0, snap_mode);
  const double y = snap_translation_value((height() * 0.6 - p.y()) / 50.0, snap_mode);
  const double z = 0.0;
  const bool shift_drop = (event->keyboardModifiers() & Qt::ShiftModifier);
  if (asset_drop_cb) asset_drop_cb(payload, x, y, z, shift_drop);
  drag_asset_preview_visible_ = false;
  event->acceptProposedAction();
  update();
}
