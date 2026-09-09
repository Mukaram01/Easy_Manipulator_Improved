#pragma once

#include <QGraphicsItem>

namespace workcell_builder {
// Already validated authored coordinates must not pass through the canvas drag
// snap filter a second time. Restore notifications for subsequent user drags.
inline void set_authored_canvas_position(QGraphicsItem * item, const QPointF & position)
{
  const bool sends_changes = item->flags().testFlag(QGraphicsItem::ItemSendsGeometryChanges);
  item->setFlag(QGraphicsItem::ItemSendsGeometryChanges, false);
  item->setPos(position);
  item->setFlag(QGraphicsItem::ItemSendsGeometryChanges, sends_changes);
}
}  // namespace workcell_builder
