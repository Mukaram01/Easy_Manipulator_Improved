#include "scene3d_viewport_widget.h"

#include <QMatrix4x4>
#include <QMouseEvent>
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
#include <cstring>
#include <QtMath>

#include <algorithm>
#include <limits>
#include <sstream>

namespace {
constexpr int kMeshTriangleLimit = 100000;

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


bool is_high_priority_role(NormalizedRole role)
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
}


Scene3DViewportWidget::Scene3DViewportWidget(QWidget * parent) : QOpenGLWidget(parent) { setMinimumHeight(420); }
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
void Scene3DViewportWidget::invalidate_mesh_cache() { update(); }
void Scene3DViewportWidget::fit_scene() {
  if (items.isEmpty()) { set_isometric_view(); return; }
  QVector3D bmin(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
  QVector3D bmax(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());
  bool has_fittable_item = false;
  for (const auto & it : items) {
    if (!include_in_fit_bounds(it, fit_include_overlays)) continue;
    has_fittable_item = true;
    bmin.setX(std::min(bmin.x(), static_cast<float>(it.x)));
    bmin.setY(std::min(bmin.y(), static_cast<float>(it.y)));
    bmin.setZ(std::min(bmin.z(), static_cast<float>(it.z)));
    bmax.setX(std::max(bmax.x(), static_cast<float>(it.x + it.sx)));
    bmax.setY(std::max(bmax.y(), static_cast<float>(it.y + it.sy)));
    bmax.setZ(std::max(bmax.z(), static_cast<float>(it.z + it.sz)));
  }
  if (!has_fittable_item) { set_isometric_view(); return; } // FIT_FALLBACK_ISO_IF_NO_PHYSICAL
  orbit_offset_ = (bmin + bmax) * 0.5f;
  const QVector3D ext = bmax - bmin;
  const double radius = qMax(0.25, 0.5 * qSqrt(ext.x() * ext.x() + ext.y() * ext.y() + ext.z() * ext.z()));
  scene_radius_ = radius;
  const double fov = qDegreesToRadians(50.0);
  const double fit_distance = (radius / qTan(fov * 0.5)) * 1.25;
  distance_ = qBound(min_distance_, fit_distance, max_distance_);
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
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  QMatrix4x4 proj, view;
  camera_matrices(proj, view);
  glMatrixMode(GL_PROJECTION);
  glLoadMatrixf(proj.constData());
  glMatrixMode(GL_MODELVIEW);
  glLoadMatrixf(view.constData());

  draw_cylinder(-0.2, 0.0, -2.2, reach_overlay.preferred_work_zone_radius_m, 0.01, QColor(34, 197, 94, 70), true);

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  std::vector<const ScenePreviewWidget::PreviewItem *> draw_items;
  for (const auto & it : items) draw_items.push_back(&it);
  std::sort(draw_items.begin(), draw_items.end(), [&](const auto * a, const auto * b) { return a->z > b->z; });

  for (const auto * it : draw_items) {
    if (classify_item_role(*it) == NormalizedRole::Object && !it->source_path.trimmed().isEmpty()) {
      (void)ensure_mesh_cached(it->source_path);
    }
    const NormalizedRole role = classify_item_role(*it);
    if (!show_safety && role == NormalizedRole::SafetyZone) continue;

    switch (role) {
      case NormalizedRole::RobotBase: draw_robot_base_with_axis(*it); break;
      case NormalizedRole::Table: draw_table_slab(*it); break;
      case NormalizedRole::Conveyor: draw_conveyor(*it); break;
      case NormalizedRole::Camera: draw_camera_body_with_frustum(*it); break;
      case NormalizedRole::PickZone: draw_pick_zone(*it); break;
      case NormalizedRole::PlaceBin: draw_place_target_bin(*it); break;
      case NormalizedRole::Object: draw_object_cube(*it); break;
      case NormalizedRole::SafetyZone: draw_safety_zone(*it); break;
      case NormalizedRole::WarningAnchor: draw_warning_badge_anchor(*it); break;
      case NormalizedRole::Generic:
        if (!draw_mesh_preview_if_available(*it, item_color(*it), true))
          draw_box(it->x, it->y, it->z, it->sx, it->sy, it->sz, item_color(*it));
        break;
    }
    if (it->id == selected_id) {
      const ItemBounds bounds = item_bounds_for_role(*it);
      draw_box_outline(bounds.x, bounds.y, bounds.z, bounds.sx, bounds.sy, bounds.sz, QColor("#f8fafc"));
    }
  }

  glDisable(GL_BLEND);

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
  for (const auto & it : items) {
    const QPointF p = project_to_screen(it.x + (it.sx * 0.5), it.y + (it.sy * 0.5), it.z + it.sz + 0.08);
    const bool selected = (it.id == selected_id);
    const NormalizedRole role = classify_item_role(it);
    bool draw_label = false;
    switch (label_mode) {
      case ScenePreviewWidget::LabelMode::Off:
        draw_label = false;
        break;
      case ScenePreviewWidget::LabelMode::Important:
        draw_label = selected || is_high_priority_role(role);
        break;
      case ScenePreviewWidget::LabelMode::Selected:
        draw_label = selected;
        break;
      case ScenePreviewWidget::LabelMode::All:
        draw_label = true;
        break;
    }
    if (show_warning_labels && !it.warnings.isEmpty()) {
      painter.setPen(Qt::NoPen);
      painter.setBrush(QColor("#f59e0b"));
      painter.drawEllipse(QRectF(p.x() - 7.0, p.y() - 18.0, 14.0, 14.0));
      painter.setPen(QColor("#111827"));
      painter.drawText(QRectF(p.x() - 7.0, p.y() - 18.0, 14.0, 14.0), Qt::AlignCenter, warning_badge_text(it.warnings));
    }
    if (draw_label) {
      const QString text = selected ? it.id : compact_role(it.role);
      painter.setPen(QColor("#e2e8f0"));
      painter.drawText(QPointF(p.x() + 10.0, p.y() - 8.0), text);
    }
    if (debug_overlays_mode && show_warning_labels && !it.warnings.isEmpty()) {
      painter.setPen(QColor("#fca5a5"));
      painter.drawText(QPointF(p.x() + 10.0, p.y() + 10.0), warning_debug_text(it.warnings));
    }
  }
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

const Scene3DViewportWidget::MeshCacheEntry & Scene3DViewportWidget::ensure_mesh_cached(const QString & path)
{
  QString canonical;
  if (!try_resolve_canonical_mesh_path(path, canonical)) {
    static MeshCacheEntry missing;
    missing.loaded = true;
    missing.valid = false;
    missing.warning = QStringLiteral("mesh missing on disk");
    qWarning() << "Scene3DViewportWidget mesh fallback: missing mesh" << path;
    return missing;
  }
  auto it = mesh_cache_.find(canonical);
  if (it != mesh_cache_.end()) return it.value();
  MeshCacheEntry entry;
  entry.loaded = true;
  QFile file(canonical);
  if (!file.open(QIODevice::ReadOnly)) {
    entry.warning = QStringLiteral("mesh unreadable");
    qWarning() << "Scene3DViewportWidget mesh fallback: unreadable mesh" << canonical;
    return mesh_cache_.insert(canonical, entry).value();
  }
  const QByteArray bytes = file.readAll();
  QString parse_error;
  entry.valid = parse_stl_bytes_for_test(bytes, canonical, entry.mesh, parse_error, kMeshTriangleLimit);
  if (!entry.valid) {
    entry.oversized = parse_error.contains("exceeds limit");
    entry.warning = entry.oversized ? QStringLiteral("mesh oversized") : QStringLiteral("mesh invalid");
    qWarning() << "Scene3DViewportWidget mesh fallback:"
               << (entry.oversized ? "oversized mesh" : "invalid mesh")
               << canonical << "reason:" << parse_error;
  }
  entry.has_bounds = compute_mesh_bounds_for_test(entry.mesh, entry.min_bounds, entry.max_bounds);
  return mesh_cache_.insert(canonical, entry).value();
}




bool Scene3DViewportWidget::draw_mesh_preview_if_available(const ScenePreviewWidget::PreviewItem & it, const QColor & color, bool preview_path)
{
  if (mesh_preview_mode == ScenePreviewWidget::MeshPreviewMode::Primitives) return false;
  if (!it.has_mesh_metadata) {
    if (mesh_preview_mode == ScenePreviewWidget::MeshPreviewMode::Meshes) {
      qWarning() << "Scene3DViewportWidget mesh fallback: mesh metadata missing for" << it.id;
    }
    return false;
  }

  const QString mesh_source = !it.mesh_path.trimmed().isEmpty() ? it.mesh_path : it.source_path;
  if (mesh_source.trimmed().isEmpty()) {
    if (mesh_preview_mode == ScenePreviewWidget::MeshPreviewMode::Meshes) {
      qWarning() << "Scene3DViewportWidget mesh fallback: mesh source missing for" << it.id;
    }
    return false;
  }
  const MeshCacheEntry & entry = ensure_mesh_cached(mesh_source);
  if (!entry.loaded || !entry.valid || entry.oversized || entry.mesh.triangles.isEmpty()) {
    if (mesh_preview_mode == ScenePreviewWidget::MeshPreviewMode::Meshes) {
      qWarning() << "Scene3DViewportWidget mesh fallback:" << (entry.warning.isEmpty() ? QStringLiteral("mesh unavailable") : entry.warning) << "for" << it.id;
    }
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

  glColor4f(color.redF(), color.greenF(), color.blueF(), 1.0f);
  glBegin(GL_TRIANGLES);
  for (const auto & tri : entry.mesh.triangles) {
    glNormal3f(tri.normal.x(), tri.normal.y(), tri.normal.z());
    glVertex3f(tri.vertices[0].x(), tri.vertices[0].y(), tri.vertices[0].z());
    glVertex3f(tri.vertices[1].x(), tri.vertices[1].y(), tri.vertices[1].z());
    glVertex3f(tri.vertices[2].x(), tri.vertices[2].y(), tri.vertices[2].z());
  }
  glEnd();

  glPopMatrix();
  return true;
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
  if (show_camera_fov) draw_frustum(QColor(56, 189, 248, 96), true);
}
void Scene3DViewportWidget::draw_pick_zone(const ScenePreviewWidget::PreviewItem & it) { draw_box(it.x, it.y, it.z, it.sx, it.sy, it.sz, QColor(34, 197, 94, 96), true); }
void Scene3DViewportWidget::draw_place_target_bin(const ScenePreviewWidget::PreviewItem & it)
{
  draw_box(it.x, it.y, it.z, it.sx, it.sy, it.sz, item_color(it), true);
  const double wall = qMax(0.02, qMin(it.sx, it.sz) * 0.1);
  draw_box_outline(it.x + wall, it.y + wall, it.z + wall, qMax(0.01, it.sx - 2 * wall), qMax(0.01, it.sy - wall), qMax(0.01, it.sz - 2 * wall), QColor("#fecdd3"), 1.5f);
}
void Scene3DViewportWidget::draw_object_cube(const ScenePreviewWidget::PreviewItem & it)
{
  if (draw_mesh_preview_if_available(it, item_color(it), true)) return;
  const double cube = qMax(0.05, qMin(it.sx, qMin(it.sy, it.sz)));
  draw_box(it.x, it.y, it.z, cube, cube, cube, item_color(it));
}
void Scene3DViewportWidget::draw_safety_zone(const ScenePreviewWidget::PreviewItem & it) { draw_box(it.x, it.y, it.z, it.sx, it.sy, it.sz, QColor(245, 158, 11, 96), true); }
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
  glColor4f(color.redF(), color.greenF(), color.blueF(), translucent ? 0.2f : 1.0f);
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
  const NormalizedRole role = classify_item_role(item);
  if (role == NormalizedRole::Object || role == NormalizedRole::WarningAnchor) {
    const double cube = (role == NormalizedRole::Object)
      ? qMax(0.05, qMin(item.sx, qMin(item.sy, item.sz)))
      : qMax(0.04, qMin(item.sx, qMin(item.sy, item.sz)));
    return { item.x, item.y, item.z, cube, cube, cube };
  }
  return { item.x, item.y, item.z, item.sx, item.sy, item.sz };
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
    *out_tooltip = QStringLiteral("Item: %1\nRole: %2\nWarnings: %3")
      .arg(out_id, role_normalized, warning_debug_text(best_item->warnings));
    if (!best_item->warnings.isEmpty()) *out_tooltip += QStringLiteral("\n\nDetails:\n- ") + best_item->warnings.join("\n- ");
  }
  return true;
}
void Scene3DViewportWidget::mousePressEvent(QMouseEvent * e) {
  last_ = e->pos();
  if (e->button() != Qt::LeftButton) return;
  QString best_id, best_role;
  if (pick_item_at_screen(e->pos(), best_id, best_role) && !best_id.isEmpty() && select_cb) select_cb(best_id, best_role);
}
void Scene3DViewportWidget::mouseMoveEvent(QMouseEvent * e)
{
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
  if (pick_item_at_screen(e->pos(), hovered, hovered_role, &hover_tooltip) && !hovered.isEmpty()) {
    QToolTip::showText(e->globalPos(), hover_tooltip, this);
  } else {
    QToolTip::hideText();
  }
  hovered_id_ = hovered;
}
void Scene3DViewportWidget::wheelEvent(QWheelEvent * e)
{
  const double delta_steps = static_cast<double>(e->angleDelta().y()) / 120.0;
  const double zoom_factor = std::pow(0.9, delta_steps);
  distance_ = qBound(min_distance_, distance_ * zoom_factor, max_distance_);
  update();
}
