#include "arrange_icons.hpp"

#include <QFont>
#include <algorithm>
#include <QPainter>
#include <QPainterPath>
#include <QPixmap>

#include "workcell_builder_ui_utils.hpp"

namespace workcell_builder
{
namespace
{

// 16 x 16 line icons in the studio style (1.6 px stroke). The reference part is
// drawn in the accent colour, the moving parts in the ink colour, and dashed
// lines are the alignment guide.
void paint_icon(QPainter & p, const QString & name, const QColor & ink, const QColor & accent)
{
  const QPen line(ink, 1.6, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
  const QPen guide(accent, 1.4, Qt::DashLine, Qt::RoundCap);
  const auto ref_box = [&](const QRectF & r) { p.setPen(Qt::NoPen); p.setBrush(accent); p.drawRoundedRect(r, 1.2, 1.2); p.setBrush(Qt::NoBrush); };
  const auto part = [&](const QRectF & r) { p.setPen(line); p.setBrush(Qt::NoBrush); p.drawRoundedRect(r, 1.2, 1.2); };
  const auto letter = [&](const QString & text) {
    QFont font = p.font(); font.setPixelSize(6); font.setBold(true); p.setFont(font);
    p.setPen(accent); p.drawText(QRectF(10.5, 9.5, 6, 6.5), Qt::AlignCenter, text);
  };
  const auto arrow = [&](QPointF from, QPointF to) {
    p.setPen(line); p.drawLine(from, to);
    const QPointF d = (to - from) / std::max(1e-6, std::hypot(to.x() - from.x(), to.y() - from.y()));
    const QPointF n(-d.y(), d.x());
    p.drawLine(to, to - d * 2.6 + n * 2.0); p.drawLine(to, to - d * 2.6 - n * 2.0);
  };
  // Edge alignment: a guide on one side, the reference and a moving part flush to it.
  struct Edge { const char * name; bool vertical_guide; bool far_side; };
  static const Edge edges[] = {{"align_left", true, false}, {"align_right", true, true}, {"align_front", false, true},
    {"align_back", false, false}, {"align_top", false, false}, {"align_bottom", false, true}};
  for (const auto & edge : edges) {
    if (name != QLatin1String(edge.name)) continue;
    const double g = edge.far_side ? 14.0 : 2.0;
    p.setPen(guide);
    if (edge.vertical_guide) {
      p.drawLine(QPointF(g, 1), QPointF(g, 15));
      ref_box(edge.far_side ? QRectF(5, 3, 8, 4) : QRectF(3, 3, 8, 4));
      part(edge.far_side ? QRectF(8.5, 9.5, 4.5, 3.5) : QRectF(3, 9.5, 4.5, 3.5));
    } else {
      p.drawLine(QPointF(1, g), QPointF(15, g));
      ref_box(edge.far_side ? QRectF(3, 5, 4, 8) : QRectF(3, 3, 4, 8));
      part(edge.far_side ? QRectF(9.5, 8.5, 3.5, 4.5) : QRectF(9.5, 3, 3.5, 4.5));
    }
    const bool z = name == QLatin1String("align_top") || name == QLatin1String("align_bottom");
    letter(z ? QStringLiteral("Z") : (edge.vertical_guide ? QStringLiteral("X") : QStringLiteral("Y")));
    return;
  }
  if (name.startsWith(QLatin1String("centre_")) && name.size() == 8) {
    const bool vertical = name.endsWith('x');
    p.setPen(guide);
    if (vertical) {
      p.drawLine(QPointF(8, 1), QPointF(8, 15)); ref_box(QRectF(3, 2.5, 10, 4)); part(QRectF(5, 9.5, 6, 3.5));
    } else {
      p.drawLine(QPointF(1, 8), QPointF(15, 8)); ref_box(QRectF(2.5, 3, 4, 10)); part(QRectF(9, 5, 3.5, 6));
    }
    letter(name.right(1).toUpper());
    return;
  }
  if (name == QLatin1String("centre_xy") || name == QLatin1String("centre_ref")) {
    ref_box(QRectF(2, 2, 12, 12));
    p.setPen(QPen(Qt::white, 1.6)); p.drawRoundedRect(QRectF(5, 5, 6, 6), 1, 1);
    if (name == QLatin1String("centre_ref")) { p.setBrush(Qt::white); p.drawEllipse(QPointF(8, 8), 1.2, 1.2); }
    else { p.setPen(QPen(Qt::white, 1.0, Qt::DashLine)); p.drawLine(QPointF(8, 2.5), QPointF(8, 13.5)); p.drawLine(QPointF(2.5, 8), QPointF(13.5, 8)); }
    return;
  }
  if (name == QLatin1String("place_top")) {
    ref_box(QRectF(1.5, 11, 13, 3.5)); part(QRectF(5, 4.5, 6, 5)); arrow(QPointF(8, 0.8), QPointF(8, 4));
    return;
  }
  if (name.startsWith(QLatin1String("dist_"))) {
    const bool horizontal = name.endsWith('x');
    if (horizontal) {
      ref_box(QRectF(1.5, 4, 3, 6)); part(QRectF(6.5, 4, 3, 6)); part(QRectF(11.5, 4, 3, 6));
    } else {
      ref_box(QRectF(4, 1.5, 6, 3)); part(QRectF(4, 6.5, 6, 3)); part(QRectF(4, 11.5, 6, 3));
    }
    letter(name.right(1).toUpper());
    return;
  }
  if (name == QLatin1String("gap")) {
    ref_box(QRectF(1.5, 3, 3.5, 10)); part(QRectF(11, 3, 3.5, 10));
    arrow(QPointF(8, 8), QPointF(5.8, 8)); arrow(QPointF(8, 8), QPointF(10.2, 8));
    return;
  }
  if (name == QLatin1String("move")) {
    arrow(QPointF(8, 8), QPointF(8, 1.8)); arrow(QPointF(8, 8), QPointF(8, 14.2));
    arrow(QPointF(8, 8), QPointF(1.8, 8)); arrow(QPointF(8, 8), QPointF(14.2, 8));
    p.setPen(QPen(accent, 1.6)); p.drawArc(QRectF(4.5, 4.5, 7, 7), 30 * 16, 120 * 16);
    return;
  }
  if (name == QLatin1String("snap_grid")) {
    p.setPen(Qt::NoPen); p.setBrush(ink);
    for (const double x : {2.5, 8.0, 13.5}) for (const double y : {2.5, 8.0, 13.5}) p.drawEllipse(QPointF(x, y), 0.9, 0.9);
    p.setBrush(Qt::NoBrush); p.setPen(QPen(accent, 1.6)); p.drawRoundedRect(QRectF(5.5, 5.5, 5, 5), 1, 1);
    return;
  }
  if (name == QLatin1String("mount") || name == QLatin1String("detach")) {
    ref_box(QRectF(1.5, 10.5, 13, 4));
    part(QRectF(4.5, 2, 7, 5));
    p.setPen(line);
    if (name == QLatin1String("mount")) { p.drawLine(QPointF(8, 7), QPointF(8, 10.5)); p.drawEllipse(QPointF(8, 8.8), 1.1, 1.1); }
    else { p.drawLine(QPointF(8, 7), QPointF(8, 7.8)); p.drawLine(QPointF(8, 9.6), QPointF(8, 10.5)); p.drawLine(QPointF(6, 9.4), QPointF(10, 8)); }
    return;
  }
  if (name == QLatin1String("flush")) {
    ref_box(QRectF(1.5, 9, 13, 5)); part(QRectF(3.5, 3, 9, 5.2));
    return;
  }
  if (name == QLatin1String("coincident")) {
    p.setPen(QPen(accent, 1.6)); p.drawLine(QPointF(8, 2), QPointF(8, 14)); p.drawLine(QPointF(2, 8), QPointF(14, 8));
    p.setPen(line); p.drawLine(QPointF(3.8, 3.8), QPointF(12.2, 12.2)); p.drawLine(QPointF(12.2, 3.8), QPointF(3.8, 12.2));
    return;
  }
  if (name == QLatin1String("concentric")) {
    p.setPen(QPen(accent, 1.6)); p.drawEllipse(QPointF(8, 8), 6, 6);
    p.setPen(line); p.drawEllipse(QPointF(8, 8), 3, 3); p.drawPoint(QPointF(8, 8));
    return;
  }
  if (name == QLatin1String("group")) {
    p.setPen(QPen(ink, 1.2, Qt::DashLine)); p.drawRoundedRect(QRectF(1.5, 1.5, 13, 13), 2, 2);
    ref_box(QRectF(4, 4, 4, 4)); part(QRectF(8.5, 8.5, 3.5, 3.5));
    return;
  }
}

}  // namespace

QIcon arrangeIcon(const QString & name)
{
  const auto & tokens = studioTokens();
  QIcon icon;
  // Normal, and an explicit muted Disabled rendering so unavailable tools read as such.
  for (const auto mode : {QIcon::Normal, QIcon::Disabled}) {
    const QColor ink(mode == QIcon::Normal ? tokens.text : tokens.textMuted);
    QColor accent(mode == QIcon::Normal ? tokens.accent : tokens.border);
    for (const int scale : {1, 2}) {
      QPixmap pixmap(16 * scale, 16 * scale);
      pixmap.setDevicePixelRatio(scale);
      pixmap.fill(Qt::transparent);
      QPainter painter(&pixmap);
      painter.setRenderHint(QPainter::Antialiasing);
      painter.setRenderHint(QPainter::TextAntialiasing);
      paint_icon(painter, name, ink, accent);
      painter.end();
      icon.addPixmap(pixmap, mode);
    }
  }
  return icon;
}

}  // namespace workcell_builder
