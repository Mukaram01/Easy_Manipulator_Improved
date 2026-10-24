#pragma once

// Qt deletes a widget's children in ~QWidget: after the derived class's
// destructor body and members are gone, but before ~QObject removes the
// connections that target the widget. A child that emits while it is being
// destroyed then runs the owner's slots and lambdas on destroyed members
// (QGraphicsScene emits selectionChanged when its selected items are deleted).
// Call this first in the owner's destructor: afterwards no descendant can call
// back into the owner.

#include <QObject>

namespace workcell_builder
{

inline void retire_inbound_connections_from_descendants(QObject * owner)
{
  if (!owner) return;
  const auto descendants = owner->findChildren<QObject *>();
  for (QObject * descendant : descendants) QObject::disconnect(descendant, nullptr, owner, nullptr);
}

}  // namespace workcell_builder
