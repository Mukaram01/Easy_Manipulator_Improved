#include "scene3d_viewport_widget.h"

#include <QLineF>
#include <QMatrix4x4>
#include <QMouseEvent>
#include <QPainter>
#include <QVector3D>
#include <QVector4D>
#include <QtMath>

#include <algorithm>
#include <limits>

namespace {
QColor item_color(const ScenePreviewWidget::PreviewItem & it)
{
  if (it.category.contains("robot", Qt::CaseInsensitive)) return QColor("#a78bfa");
  if (it.category.contains("camera", Qt::CaseInsensitive)) return QColor("#38bdf8");
  if (it.category.contains("conveyor", Qt::CaseInsensitive)) return QColor("#06b6d4");
  if (it.category.contains("bin", Qt::CaseInsensitive) || it.role.contains("pick", Qt::CaseInsensitive)) return QColor("#34d399");
  if (it.role.contains("place", Qt::CaseInsensitive)) return QColor("#fb7185");
  return QColor("#94a3b8");
}
}

Scene3DViewportWidget::Scene3DViewportWidget(QWidget * parent) : QOpenGLWidget(parent) { setMinimumHeight(420); }
void Scene3DViewportWidget::reset_view() { set_isometric_view(); }
void Scene3DViewportWidget::set_isometric_view() { yaw_ = -0.9; pitch_ = 0.7; orbit_offset_ = QVector3D(0.0f, 0.0f, 0.0f); distance_ = 6.0; update(); }
void Scene3DViewportWidget::set_top_view() { yaw_ = 0.0; pitch_ = -1.35; update(); }
void Scene3DViewportWidget::set_front_view() { yaw_ = 0.0; pitch_ = 0.0; update(); }
void Scene3DViewportWidget::set_side_view() { yaw_ = -M_PI_2; pitch_ = 0.0; update(); }
void Scene3DViewportWidget::fit_scene() {
  if (items.isEmpty()) { set_isometric_view(); return; }
  QVector3D bmin(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
  QVector3D bmax(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());
  for (const auto & it : items) {
    bmin.setX(std::min(bmin.x(), static_cast<float>(it.x)));
    bmin.setY(std::min(bmin.y(), static_cast<float>(it.y)));
    bmin.setZ(std::min(bmin.z(), static_cast<float>(it.z)));
    bmax.setX(std::max(bmax.x(), static_cast<float>(it.x + it.sx)));
    bmax.setY(std::max(bmax.y(), static_cast<float>(it.y + it.sy)));
    bmax.setZ(std::max(bmax.z(), static_cast<float>(it.z + it.sz)));
  }
  orbit_offset_ = (bmin + bmax) * 0.5f;
  const QVector3D ext = bmax - bmin;
  const double radius = qMax(0.25, 0.5 * qSqrt(ext.x() * ext.x() + ext.y() * ext.y() + ext.z() * ext.z()));
  const double fov = qDegreesToRadians(50.0);
  const double fit_distance = (radius / qTan(fov * 0.5)) * 1.25;
  distance_ = qBound(min_distance_, fit_distance, max_distance_);
  update();
}
void Scene3DViewportWidget::focus_selected() { fit_scene(); }
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
  draw_box(-1.2, 0.07, -1.0, 2.4, 0.15, 1.6, QColor("#64748b"));
  draw_box(-2.4, 0.05, -0.2, 1.2, 0.1, 0.6, QColor("#06b6d4"));

  std::vector<const ScenePreviewWidget::PreviewItem *> solids;
  std::vector<const ScenePreviewWidget::PreviewItem *> transparent;
  for (const auto & it : items) {
    if (!show_safety && it.category.contains("safety", Qt::CaseInsensitive)) continue;
    if (it.category.contains("safety", Qt::CaseInsensitive)) transparent.push_back(&it);
    else solids.push_back(&it);
  }
  for (const auto * it : solids) {
    draw_box(it->x, it->y, it->z, it->sx, it->sy, it->sz, item_color(*it));
    if (it->id == selected_id) draw_box_outline(it->x, it->y, it->z, it->sx, it->sy, it->sz, QColor("#f8fafc"));
  }

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  std::sort(transparent.begin(), transparent.end(), [&](const auto * a, const auto * b) { return a->z > b->z; });
  for (const auto * it : transparent) {
    draw_box(it->x, it->y, it->z, it->sx, it->sy, it->sz, item_color(*it), true);
    if (it->id == selected_id) draw_box_outline(it->x, it->y, it->z, it->sx, it->sy, it->sz, QColor("#e2e8f0"));
  }
  if (show_camera_fov) draw_frustum(QColor(56, 189, 248, 96), true);
  glDisable(GL_BLEND);
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
  out_proj.setToIdentity();
  out_proj.perspective(50.0f, aspect, 0.05f, 100.0f);
  out_view.setToIdentity();
  out_view.translate(0.0f, -0.5f, -6.0f * zoom_);
  out_view.rotate(qRadiansToDegrees(pitch_), 1.0f, 0.0f, 0.0f);
  out_view.rotate(qRadiansToDegrees(yaw_), 0.0f, 1.0f, 0.0f);
}
bool Scene3DViewportWidget::ray_intersects_aabb(const QVector3D & ray_origin, const QVector3D & ray_dir,
                                                const ScenePreviewWidget::PreviewItem & item, float & out_t) const
{
  const QVector3D bmin(item.x, item.y, item.z);
  const QVector3D bmax(item.x + item.sx, item.y + item.sy, item.z + item.sz);
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
void Scene3DViewportWidget::mousePressEvent(QMouseEvent * e) {
  last_ = e->pos();
  if (e->button() != Qt::LeftButton) return;
  QMatrix4x4 proj, view;
  camera_matrices(proj, view);
  const QMatrix4x4 inv = (proj * view).inverted();
  const float ndc_x = (2.0f * static_cast<float>(e->position().x()) / qMax(1, width())) - 1.0f;
  const float ndc_y = 1.0f - (2.0f * static_cast<float>(e->position().y()) / qMax(1, height()));
  QVector4D near_h = inv * QVector4D(ndc_x, ndc_y, -1.0f, 1.0f);
  QVector4D far_h = inv * QVector4D(ndc_x, ndc_y, 1.0f, 1.0f);
  if (qFuzzyIsNull(near_h.w()) || qFuzzyIsNull(far_h.w())) return;
  QVector3D p0 = near_h.toVector3DAffine();
  QVector3D p1 = far_h.toVector3DAffine();
  QVector3D dir = (p1 - p0).normalized();
  QString best_id, best_role;
  float best_t = 1e9f;
  for (const auto & item : items) {
    if (!item.selectable) continue;
    float t = 0.0f;
    if (ray_intersects_aabb(p0, dir, item, t) && t < best_t) {
      best_t = t;
      best_id = item.id;
      best_role = item.role.trimmed().isEmpty() ? QStringLiteral("unknown") : item.role.trimmed();
    }
  }
  if (!best_id.isEmpty() && select_cb) select_cb(best_id, best_role);
}
void Scene3DViewportWidget::mouseMoveEvent(QMouseEvent * e) { auto d = e->pos() - last_; last_ = e->pos(); if (e->buttons() & Qt::LeftButton) { yaw_ += d.x() * 0.01; pitch_ = qBound(-1.4, pitch_ + d.y() * 0.01, 1.4); update(); } }
void Scene3DViewportWidget::wheelEvent(QWheelEvent * e) { zoom_ = qBound(0.4, zoom_ + (e->angleDelta().y() > 0 ? -0.1 : 0.1), 2.2); update(); }
