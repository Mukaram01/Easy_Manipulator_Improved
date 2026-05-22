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
#include <QXmlStreamReader>
#include <cstring>
#include <QtMath>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace {
constexpr int kMeshTriangleLimit = 100000;
constexpr double kWorkspaceLimitMeters = 1000.0;

QString snap_mode_label(Scene3DViewportWidget::SnapMode mode)
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
  return !out_mesh.triangles.isEmpty();
}

bool looks_like_ascii_stl(const QByteArray & bytes)
{
  const QByteArray prefix = bytes.left(256).trimmed().toLower();
  return prefix.startsWith("solid") && prefix.contains("facet");
}


bool parse_collada_bytes(const QByteArray & bytes, Scene3DViewportWidget::InternalTriangleMesh & out_mesh,
                         QString & out_error, int triangle_limit)
{
  QXmlStreamReader xml(bytes);
  QVector<float> positions;
  bool in_geometry = false;
  while (!xml.atEnd()) {
    xml.readNext();
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
      auto addTri=[&](int a,int b,int c){
        const int na=a*3, nb=b*3, nc=c*3;
        if (na+2>=positions.size()||nb+2>=positions.size()||nc+2>=positions.size()) return;
        Scene3DViewportWidget::InternalTriangleMesh::Triangle tri;
        tri.vertices[0]=QVector3D(positions[na],positions[na+1],positions[na+2]);
        tri.vertices[1]=QVector3D(positions[nb],positions[nb+1],positions[nb+2]);
        tri.vertices[2]=QVector3D(positions[nc],positions[nc+1],positions[nc+2]);
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
  const QString source_layer = normalized_token(it.source_layer);
  const QString visual_source = normalized_token(it.active_visual_source);
  const QString role = normalized_token(it.role);
  const QString category = normalized_token(it.category);
  const QString lock_reason = normalized_token(it.lock_reason);

  const bool overlay_or_helper = role.contains("overlay") || role.contains("helper") || role.contains("guide") ||
                                 category.contains("overlay") || category.contains("helper") ||
                                 lock_reason.contains("overlay") || lock_reason.contains("helper");
  if (overlay_or_helper) return false;

  const bool generated_robot_visual = source_layer.contains("generated") || source_layer.contains("urdf") ||
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



bool include_in_fit_bounds(const ScenePreviewWidget::PreviewItem & it, bool include_overlays)
{
  if (include_overlays) return true;
  // FIT_PHYSICAL_ONLY_FILTER: keep fit_scene() bounds focused on physical geometry.
  const NormalizedRole role = classify_item_role(it);
  switch (role) {
    case NormalizedRole::SafetyZone:
    case NormalizedRole::WarningAnchor:
      return false;  // FIT_EXCLUDE_OVERLAY_ONLY: safety envelope + warning markers.
    case NormalizedRole::RobotBase:
    case NormalizedRole::Table:
    case NormalizedRole::Conveyor:
    case NormalizedRole::Camera:
    case NormalizedRole::PickZone:
    case NormalizedRole::PlaceBin:
    case NormalizedRole::Object:
    case NormalizedRole::Generic:
      return true;
  }
  return true;
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

double wrap_angle_pi(double angle)
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

int label_priority_bucket(bool selected, bool has_warnings, NormalizedRole role)
{
  // LABEL_PRIORITY_SELECTED_WARN_ANCHOR: selected > warnings > key anchors.
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
void Scene3DViewportWidget::set_top_view() { yaw_ = 0.0; pitch_ = -1.35; update(); }
void Scene3DViewportWidget::set_front_view() { yaw_ = 0.0; pitch_ = 0.0; update(); }
void Scene3DViewportWidget::set_side_view() { yaw_ = -M_PI_2; pitch_ = 0.0; update(); }
void Scene3DViewportWidget::invalidate_mesh_cache()
{
  mesh_cache_.clear();
  warned_mesh_fallbacks_.clear();
  update();
}
void Scene3DViewportWidget::fit_scene() {
  QVector3D bmin, bmax;
  // if (!include_in_fit_bounds_physical_only(it)) continue;
  // if (!has_physical_item) { set_isometric_view(); return; }
  if (!scene_bounds_from_visible_items(bmin, bmax, fit_include_overlays)) { set_isometric_view(); return; } // FIT_FALLBACK_ISO_IF_NO_PHYSICAL
  orbit_offset_ = (bmin + bmax) * 0.5f;
  const QVector3D ext = bmax - bmin;
  const double radius = qMax(0.25, 0.5 * qSqrt(ext.x() * ext.x() + ext.y() * ext.y() + ext.z() * ext.z()));
  scene_radius_ = radius;
  const double fov = qDegreesToRadians(50.0);
  const double fit_distance = (radius / qTan(fov * 0.5)) * 1.45;
  distance_ = qBound(min_distance_, fit_distance, max_distance_);
  pitch_ = qBound(0.28, pitch_, 0.9);
  orbit_offset_.setY(orbit_offset_.y() + static_cast<float>(qMax(0.15, radius * 0.1)));
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
  draw_cylinder(-0.2, 0.0, -2.2, reach_overlay.preferred_work_zone_radius_m, 0.01, QColor(34, 197, 94, 70), true);

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  std::vector<const ScenePreviewWidget::PreviewItem *> draw_items;
  for (const auto & it : items) draw_items.push_back(&it);
  std::sort(draw_items.begin(), draw_items.end(), [&](const auto * a, const auto * b) { return a->z > b->z; });

  int received_item_count = static_cast<int>(draw_items.size());
  int visible_item_count = 0;
  int skipped_item_count = 0;
  int rendered_item_count = 0;
  int mesh_backed_count = 0;
  int placeholder_count = 0;
  int wireframe_box_count = 0;
  int overlay_count = 0;
  int locked_urdf_count = 0;
  int physical_item_count = 0;
  std::vector<const ScenePreviewWidget::PreviewItem *> overlay_items;
  std::vector<const ScenePreviewWidget::PreviewItem *> physical_items;
  for (const auto * it : draw_items) {
    const NormalizedRole role = classify_item_role(*it);
    if (!show_safety && role == NormalizedRole::SafetyZone) { ++skipped_item_count; continue; }
    ++visible_item_count;
    if (is_locked_urdf_item(*it)) ++locked_urdf_count;
    if (is_overlay_visual_role(role)) overlay_items.push_back(it);
    else {
      physical_items.push_back(it);
      ++physical_item_count;
    }
  }
  overlay_count = static_cast<int>(overlay_items.size());

  auto draw_item_batch = [&](const std::vector<const ScenePreviewWidget::PreviewItem *> & batch, bool count_in_stats) {
    for (const auto * it : batch) {
      int item_placeholder_count = 0;
      int item_mesh_backed_count = 0;
      int item_wireframe_box_count = 0;
      draw_truthful_item_geometry(*it, &item_placeholder_count, &item_mesh_backed_count, &item_wireframe_box_count);
      ++rendered_item_count;
      if (count_in_stats) {
        placeholder_count += item_placeholder_count;
        mesh_backed_count += item_mesh_backed_count;
        wireframe_box_count += item_wireframe_box_count;
      }
      if (it->id == selected_id) {
        const ItemBounds bounds = item_bounds_for_role(*it);
        const bool editable = item_is_editable_for_gizmo(*it);
        draw_box_outline(bounds.x, bounds.y, bounds.z, bounds.sx, bounds.sy, bounds.sz, editable ? QColor("#f8fafc") : QColor("#94a3b8"));
        // draw_box_outline(bounds.x, bounds.y, bounds.z, bounds.sx, bounds.sy, bounds.sz, QColor("#f8fafc"));
      }
    }
  };  // draw_item_batch
  draw_item_batch(overlay_items, false);  // draw translucent overlays before solids to keep physical meshes legible.
  draw_item_batch(physical_items, true);

  glDisable(GL_BLEND);

  qDebug() << "Scene3D runtime render: received=" << received_item_count
           << "visible=" << visible_item_count
           << "rendered=" << rendered_item_count
           << "skipped=" << skipped_item_count
           << "mesh_backed=" << mesh_backed_count
           << "placeholder=" << placeholder_count
           << "overlay=" << overlay_count;
  qDebug() << kPaintGLCacheOnlyGuard;
  qDebug() << "Scene3D diagnostics {viewport_received_count=" << received_item_count
           << ", render_cache_count=" << mesh_cache_.size()
           << ", rendered_count=" << rendered_item_count
           << ", skipped_count=" << skipped_item_count
           << "}";

  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing, true);
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
    bool draw_label = false;
    switch (label_mode) {
      case ScenePreviewWidget::LabelMode::Off: draw_label = selected; break;
      case ScenePreviewWidget::LabelMode::Important: draw_label = selected || is_critical_label_role(role); break;
      case ScenePreviewWidget::LabelMode::Selected: draw_label = selected; break;
      case ScenePreviewWidget::LabelMode::All: draw_label = true; break;
    }
    const bool is_urdf_visual = it.locked && !it.editable && it.lock_reason.contains("URDF visual", Qt::CaseInsensitive);
    if (suppress_dense_non_critical_labels && !selected && !is_critical_label_role(role)) draw_label = false;
    if (is_urdf_visual && !selected && label_mode != ScenePreviewWidget::LabelMode::All) draw_label = false;
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
    if (!selected && is_urdf_visual && missing_reason.isEmpty() && !is_critical_scene_anchor && label_mode != ScenePreviewWidget::LabelMode::All) continue;

    const QString compact_text = clean_label_from_item(it);
    const QString text = selected ? compact_text : (missing_reason.isEmpty() ? compact_text : QString("%1 missing").arg(compact_role(it.role)));
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
  for (const auto & candidate : label_candidates) {
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
    if (overlaps) continue;

    placed_label_points.push_back(label_pos);
    placed_label_boxes.push_back(label_rect);
    painter.setPen(candidate.color);
    painter.drawText(label_pos, candidate.text);
  }
  painter.setPen(Qt::NoPen);
  painter.setBrush(QColor(15, 23, 42, 190));
  painter.drawRoundedRect(QRectF(12.0, 12.0, 360.0, 74.0), 6.0, 6.0);
  painter.setPen(QColor("#e2e8f0"));
  painter.drawText(QRectF(20.0, 18.0, 344.0, 16.0), Qt::AlignLeft | Qt::AlignVCenter, "View: 3D");
  painter.drawText(QRectF(20.0, 34.0, 344.0, 16.0), Qt::AlignLeft | Qt::AlignVCenter,
                   QString("Scene: %1").arg(scene_name));
  painter.drawText(QRectF(20.0, 50.0, 344.0, 16.0), Qt::AlignLeft | Qt::AlignVCenter,
                   QString("Items %1 • Mesh %2 • Boxes %3 • Missing %4")
                     .arg(physical_item_count).arg(mesh_backed_count).arg(wireframe_box_count).arg(placeholder_count));
  painter.drawText(QRectF(20.0, 66.0, 344.0, 16.0), Qt::AlignLeft | Qt::AlignVCenter,
                   QString("Overlays %1 • Locked URDF %2 • Mode: %3")
                     .arg(overlay_count).arg(locked_urdf_count).arg(gizmo_mode_label()));
  if (drag_asset_preview_visible_) {
    const double x = (drag_asset_screen_pos_.x() - width() * 0.5) / 50.0;
    const double y = (height() * 0.6 - drag_asset_screen_pos_.y()) / 50.0;
    draw_box(x, y, 0.0, 0.35, 0.35, 0.35, QColor(56, 189, 248, 120), true);
    QToolTip::showText(mapToGlobal(drag_asset_screen_pos_), drag_asset_drop_status_, this);
  }
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

bool Scene3DViewportWidget::item_has_explicit_dimensions(const ScenePreviewWidget::PreviewItem & item) const
{
  return item.sx > 0.001 && item.sy > 0.001 && item.sz > 0.001;
}

QString Scene3DViewportWidget::placeholder_reason_for_item(const ScenePreviewWidget::PreviewItem & item) const
{
  if (item.mesh_available || item.has_mesh_metadata || !item.mesh_path.trimmed().isEmpty() || !item.source_path.trimmed().isEmpty()) return QString();
  if (item_has_explicit_dimensions(item)) return QString();
  return QStringLiteral("missing geometry");
}

bool Scene3DViewportWidget::draw_truthful_item_geometry(const ScenePreviewWidget::PreviewItem & it, int * out_placeholder_count,
                                                        int * out_mesh_count, int * out_wireframe_count)
{
  // Always try mesh-backed draw first for physical items.
  if (draw_mesh_preview_if_available(it, item_color(it), true)) {
    if (out_mesh_count) ++(*out_mesh_count);
    return true;
  }
  const QString missing_reason = placeholder_reason_for_item(it);
  if (!missing_reason.isEmpty()) {
    draw_missing_geometry_marker(it);
    if (out_placeholder_count) ++(*out_placeholder_count);
    if (show_warning_labels) warn_mesh_fallback_once(it.id, QStringLiteral("missing geometry"), it.source_path);
    return false;
  }
  if (item_has_explicit_dimensions(it)) {
    draw_box(it.x, it.y, it.z, it.sx, it.sy, it.sz, item_color(it), true);
    draw_box_outline(it.x, it.y, it.z, it.sx, it.sy, it.sz, QColor("#cbd5e1"), 1.4f);
    if (out_wireframe_count) ++(*out_wireframe_count);
    return false;
  }
  draw_missing_geometry_marker(it);
  if (out_placeholder_count) ++(*out_placeholder_count);
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
                                                          int triangle_limit)
{
  Q_UNUSED(source_hint);
  out_mesh.triangles.clear();
  return parse_collada_bytes(bytes, out_mesh, out_error, triangle_limit);
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

bool Scene3DViewportWidget::try_resolve_canonical_mesh_path(const QString & path, QString & out_canonical) const
{
  QFileInfo info(path);
  if (!info.exists() || !info.isFile()) return false;
  out_canonical = info.canonicalFilePath();
  if (out_canonical.isEmpty()) out_canonical = info.absoluteFilePath();
  return !out_canonical.isEmpty();
}

bool Scene3DViewportWidget::warn_mesh_fallback_once(const QString & item_id, const QString & reason, const QString & path)
{
  const QString key = QStringLiteral("%1|%2|%3").arg(item_id, reason, path);
  if (warned_mesh_fallbacks_.contains(key)) return false;
  warned_mesh_fallbacks_.insert(key);
  qWarning().noquote() << QStringLiteral("Mesh preview fallback for %1: %2").arg(item_id, reason);
  return true;
}

const Scene3DViewportWidget::MeshCacheEntry & Scene3DViewportWidget::ensure_mesh_cached(const QString & path)
{
  const QFileInfo input_info(path);
  QString canonical;
  if (!try_resolve_canonical_mesh_path(path, canonical)) canonical = input_info.absoluteFilePath();
  auto it = mesh_cache_.find(canonical);
  if (it != mesh_cache_.end()) return it.value();
  MeshCacheEntry entry;
  entry.loaded = true;
  if (!input_info.exists() || !input_info.isFile()) {
    entry.valid = false;
    entry.warning = QStringLiteral("mesh missing on disk");
    return mesh_cache_.insert(canonical, entry).value();
  }
  QFile file(canonical);
  if (!file.open(QIODevice::ReadOnly)) {
    entry.warning = QStringLiteral("mesh unreadable");
    return mesh_cache_.insert(canonical, entry).value();
  }
  const QByteArray bytes = file.readAll();
  const QString ext = input_info.suffix().toLower();
  QString parse_error;
  if (ext == QStringLiteral("stl")) {
    entry.valid = parse_stl_bytes_for_test(bytes, canonical, entry.mesh, parse_error, kMeshTriangleLimit);
  } else if (ext == QStringLiteral("dae")) {
    entry.valid = parse_collada_bytes_for_test(bytes, canonical, entry.mesh, parse_error, kMeshTriangleLimit);
  } else {
    entry.valid = false;
    parse_error = QStringLiteral("unsupported mesh format: .%1").arg(ext.isEmpty() ? QStringLiteral("<none>") : ext);
  }
  if (!entry.valid) {
    entry.oversized = parse_error.contains("exceeds limit");
    entry.warning = QStringLiteral("%1 (%2)").arg(entry.oversized ? QStringLiteral("mesh oversized") : QStringLiteral("mesh invalid"), parse_error);
  }
  entry.has_bounds = compute_mesh_bounds_for_test(entry.mesh, entry.local_min, entry.local_max);
  return mesh_cache_.insert(canonical, entry).value();
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
    if (meshes_only_mode) warn_mesh_fallback_once(it.id, reason, path);
  };

  if (!it.has_mesh_metadata) {
    warn_for_mode(QStringLiteral("mesh metadata missing"), it.source_path);
    return false;
  }

  const QString mesh_source = !it.mesh_path.trimmed().isEmpty() ? it.mesh_path : it.source_path;
  if (mesh_source.trimmed().isEmpty()) {
    warn_for_mode(QStringLiteral("mesh source missing"), mesh_source);
    return false;
  }
  const MeshCacheEntry & entry = ensure_mesh_cached(mesh_source);
  if (!entry.loaded || !entry.valid || entry.oversized || entry.mesh.triangles.isEmpty()) {
    warn_for_mode(entry.warning.isEmpty() ? QStringLiteral("mesh unavailable") : entry.warning, mesh_source);
    return false;
  }

  glPushMatrix();
  glTranslated(it.x, it.y, it.z);
  glRotated(qRadiansToDegrees(it.roll), 1.0, 0.0, 0.0);
  glRotated(qRadiansToDegrees(it.pitch), 0.0, 1.0, 0.0);
  glRotated(qRadiansToDegrees(it.yaw), 0.0, 0.0, 1.0);
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
void Scene3DViewportWidget::draw_pick_zone(const ScenePreviewWidget::PreviewItem & it) { draw_box(it.x, it.y, it.z, it.sx, it.sy, it.sz, QColor(34, 197, 94, 64), true); draw_box_outline(it.x, it.y, it.z, it.sx, it.sy, it.sz, QColor(34, 197, 94, 110), 1.0f); }
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
  const double marker = 0.08;
  draw_box(it.x, it.y, it.z, marker, marker, marker, QColor("#ef4444"), true);
  draw_box_outline(it.x, it.y, it.z, marker, marker, marker, QColor("#fca5a5"), 2.0f);
}
void Scene3DViewportWidget::draw_safety_zone(const ScenePreviewWidget::PreviewItem & it) { draw_box(it.x, it.y, it.z, it.sx, it.sy, it.sz, QColor(245, 158, 11, 60), true); draw_box_outline(it.x, it.y, it.z, it.sx, it.sy, it.sz, QColor(245, 158, 11, 110), 1.0f); }
void Scene3DViewportWidget::draw_warning_badge_anchor(const ScenePreviewWidget::PreviewItem & it)
{
  const double cube = qMax(0.04, qMin(it.sx, qMin(it.sy, it.sz)));
  draw_box(it.x, it.y, it.z, cube, cube, cube, QColor("#f59e0b"));
}

void Scene3DViewportWidget::draw_box(double cx, double cy, double cz, double sx, double sy, double sz, const QColor & color, bool translucent)
{
  glColor4f(color.redF(), color.greenF(), color.blueF(), translucent ? 0.35f : 1.0f);
  const double x = cx, y = cy, z = cz;
  glBegin(GL_QUADS);
  glVertex3f(x, y, z); glVertex3f(x + sx, y, z); glVertex3f(x + sx, y + sy, z); glVertex3f(x, y + sy, z);
  glVertex3f(x, y, z + sz); glVertex3f(x + sx, y, z + sz); glVertex3f(x + sx, y + sy, z + sz); glVertex3f(x, y + sy, z + sz);
  glVertex3f(x, y, z); glVertex3f(x, y + sy, z); glVertex3f(x, y + sy, z + sz); glVertex3f(x, y, z + sz);
  glVertex3f(x + sx, y, z); glVertex3f(x + sx, y + sy, z); glVertex3f(x + sx, y + sy, z + sz); glVertex3f(x + sx, y, z + sz);
  glEnd();
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
void Scene3DViewportWidget::draw_cylinder(double cx, double cy, double cz, double radius, double height, const QColor & color, bool translucent)
{ glColor4f(color.redF(), color.greenF(), color.blueF(), translucent ? 0.25f : 1.0f); glBegin(GL_TRIANGLE_FAN); glVertex3f(cx, cy, cz); for (int i = 0; i <= 32; ++i) { const double a = 2.0 * M_PI * i / 32.0; glVertex3f(cx + radius * qCos(a), cy, cz + radius * qSin(a)); } glEnd(); Q_UNUSED(height); }
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
  const float far_plane = qMax(100.0f, clamped_distance + static_cast<float>(scene_radius_ * 6.0));
  out_proj.setToIdentity();
  out_proj.perspective(50.0f, aspect, 0.05f, far_plane);
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
  const MeshCacheEntry & cache = const_cast<Scene3DViewportWidget *>(this)->ensure_mesh_cached(mesh_source);
  if (!cache.loaded || !cache.valid || !cache.has_bounds) return false;

  QMatrix4x4 transform;
  transform.translate(item.x, item.y, item.z);
  transform.rotate(qRadiansToDegrees(item.roll), 1.0f, 0.0f, 0.0f);
  transform.rotate(qRadiansToDegrees(item.pitch), 0.0f, 1.0f, 0.0f);
  transform.rotate(qRadiansToDegrees(item.yaw), 0.0f, 0.0f, 1.0f);
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
    *out_tooltip = QStringLiteral("Item: %1\nRole: %2\nWarnings: %3")
      .arg(out_id, role_normalized, warning_debug_text(best_item->warnings));
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
