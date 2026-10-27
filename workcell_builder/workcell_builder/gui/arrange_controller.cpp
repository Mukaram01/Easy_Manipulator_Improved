#include "arrange_controller.hpp"

#include <QCryptographicHash>
#include <QHash>
#include <QJsonDocument>

#include <algorithm>
#include <utility>

namespace workcell_builder
{

ArrangeController::ArrangeController(Hooks hooks, QObject * parent)
: QObject(parent), hooks_(std::move(hooks))
{
}

QString ArrangeController::display_name(const QString & id) const
{
  const QString name = hooks_.display_name ? hooks_.display_name(id).trimmed() : QString();
  return name.isEmpty() ? id : name;
}

QString ArrangeController::availability(const QString & op, const QJsonObject & params) const
{
  const Selection selection = hooks_.selection ? hooks_.selection() : Selection{};
  if (selection.members.size() < 2 || selection.reference.isEmpty()) {
    return QStringLiteral("Select two or more parts: click the reference first, then Ctrl+click the parts to arrange.");
  }
  if (const QString blocker = hooks_.reference_blocker ? hooks_.reference_blocker(selection.reference) : QString();
      !blocker.isEmpty()) {
    return QStringLiteral("%1 Choose another reference from the selection menu.").arg(blocker);
  }
  if (op == QStringLiteral("distribute") && params.value("mode").toString() != QStringLiteral("gap") &&
      selection.members.size() < 3) {
    return QStringLiteral("Equal spacing needs three or more parts. With two, use Distribute with a gap.");
  }
  if (op != QStringLiteral("detach")) {
    for (const auto & id : selection.members) {
      if (id == selection.reference) continue;
      const QString blocker = hooks_.moving_blocker ? hooks_.moving_blocker(id) : QString();
      if (!blocker.isEmpty()) {
        return QStringLiteral("%1 Make it the reference, or remove it from the selection (Ctrl+click).").arg(blocker);
      }
    }
  }
  if (op == QStringLiteral("mount") && hooks_.reference_world && !hooks_.reference_world(selection.reference).isEmpty()) {
    return QStringLiteral("The reference %1 is locked; parts can only be mounted on an editable part.")
      .arg(display_name(selection.reference));
  }
  return QString();
}

QJsonObject ArrangeController::request_for(const QString & op, const QJsonObject & params, const Selection & selection) const
{
  QJsonObject names;
  for (const auto & id : selection.members) names.insert(id, display_name(id));
  QJsonObject request{{"operation", "arrange"}, {"op", op}, {"reference", selection.reference},
    {"members", QJsonArray::fromStringList(selection.members)}, {"params", params}, {"names", names}};
  const QJsonObject world = hooks_.reference_world ? hooks_.reference_world(selection.reference) : QJsonObject{};
  if (!world.isEmpty()) request.insert("reference_world", world);
  return request;
}

QString ArrangeController::items_signature() const
{
  const QJsonArray items = hooks_.current_items ? hooks_.current_items() : QJsonArray{};
  return QString::fromLatin1(QCryptographicHash::hash(QJsonDocument(items).toJson(QJsonDocument::Compact),
    QCryptographicHash::Sha1).toHex());
}

bool ArrangeController::start(const QString & op, const QJsonObject & params, const QString & label, QString * error)
{
  const QString blocker = availability(op, params);
  if (!blocker.isEmpty()) {
    if (error) *error = blocker;
    return false;
  }
  active_ = true;
  op_ = op;
  params_ = params;
  label_ = label;
  replan();
  emit changed();
  return true;
}

void ArrangeController::set_params(const QJsonObject & params)
{
  if (!active_) return;
  params_ = params;
  replan();
  emit changed();
}

void ArrangeController::cancel(const QString & why)
{
  if (!active_) return;
  active_ = false;
  preview_ = {};
  status_ = why.isEmpty() ? QStringLiteral("%1 cancelled; nothing changed.").arg(label_) : why;
  emit changed();
  emit finished(false, status_);
}

void ArrangeController::replan()
{
  preview_ = {};
  planned_selection_ = hooks_.selection ? hooks_.selection() : Selection{};
  planned_signature_ = items_signature();
  const QString blocker = availability(op_, params_);
  if (!blocker.isEmpty()) {
    preview_.error = blocker;
    status_ = blocker;
    return;
  }
  preview_.request = request_for(op_, params_, planned_selection_);
  const auto response = hooks_.run_resolver ? hooks_.run_resolver(preview_.request) : QJsonObject{};
  if (!response.value("ok").toBool()) {
    preview_.error = response.value("error").toString().trimmed();
    if (preview_.error.isEmpty()) preview_.error = QStringLiteral("The arrangement could not be planned; nothing changed.");
    status_ = preview_.error;
    return;
  }
  const auto result = response.value("result").toObject();
  preview_.ok = true;
  preview_.report = result.value("report").toObject();
  QHash<QString, QJsonObject> rows;
  for (const auto & value : result.value("rows").toArray()) rows.insert(value.toObject().value("id").toString(), value.toObject());
  double largest = 0.;
  QStringList followers;
  for (const auto & value : preview_.report.value("moved").toArray()) {
    const auto entry = value.toObject();
    const QString id = entry.value("id").toString();
    const auto row = rows.value(id);
    preview_.moved.append(QJsonObject{{"id", id}, {"display_name", display_name(id)},
      {"distance_m", entry.value("distance_m")}, {"to", entry.value("to")}, {"follower", entry.value("follower")}});
    preview_.ghost.append(QJsonObject{{"id", id}, {"xyz", row.value("pose_xyz")}, {"rpy", row.value("pose_rpy")}});
    largest = std::max(largest, entry.value("distance_m").toDouble());
    if (entry.value("follower").toBool()) followers << display_name(id);
  }
  QStringList parts;
  if (op_ == QStringLiteral("mount")) {
    QStringList mounted;
    for (const auto & id : planned_selection_.members) if (id != planned_selection_.reference) mounted << display_name(id);
    parts << QStringLiteral("Mount %1 on %2 with a fixed offset: they keep their place now and follow it later.")
      .arg(mounted.join(QStringLiteral(", ")), display_name(planned_selection_.reference));
  } else if (op_ == QStringLiteral("detach")) {
    QStringList detached;
    for (const auto & id : preview_.report.value("detached").toArray()) detached << display_name(id.toString());
    parts << QStringLiteral("Detach %1: world positions are kept; they no longer follow a parent.").arg(detached.join(", "));
  } else if (preview_.moved.isEmpty()) {
    parts << QStringLiteral("Everything is already in place; Apply changes nothing.");
  } else {
    parts << QStringLiteral("%1 part(s) move, the largest by %2 mm. The reference %3 stays put.")
      .arg(preview_.moved.size() - followers.size()).arg(largest * 1000., 0, 'f', 1).arg(display_name(planned_selection_.reference));
  }
  if (op_ == QStringLiteral("distribute")) {
    QStringList order;
    for (const auto & id : preview_.report.value("order").toArray()) order << display_name(id.toString());
    parts << QStringLiteral("Order along %1: %2 · gap %3 mm.").arg(preview_.report.value("axis").toString().toUpper(),
      order.join(QStringLiteral(" → "))).arg(preview_.report.value("gap_m").toDouble() * 1000., 0, 'f', 1);
  }
  if (!followers.isEmpty()) parts << QStringLiteral("Also moves (mounted on a moving part): %1.").arg(followers.join(", "));
  for (const auto & note : preview_.report.value("notes").toArray()) parts << QStringLiteral("⚠ %1.").arg(note.toString());
  preview_.summary = parts.join(QStringLiteral(" "));
  status_ = QStringLiteral("Check the ghosts, then Apply (Enter) or Cancel (Esc).");
}

bool ArrangeController::apply(QString * error)
{
  const auto fail = [error](const QString & message) { if (error) *error = message; return false; };
  if (!active_ || !preview_.ok) return fail(preview_.error.isEmpty() ? QStringLiteral("Nothing to apply.") : preview_.error);
  const Selection now = hooks_.selection ? hooks_.selection() : Selection{};
  if (items_signature() != planned_signature_ || now.members != planned_selection_.members ||
      now.reference != planned_selection_.reference) {
    replan();
    emit changed();
    return fail(QStringLiteral("The scene or the selection changed since the preview. Check the updated preview, then Apply again."));
  }
  const QString message = QStringLiteral("%1 applied to %2 part(s); Undo reverts it in one step.")
    .arg(label_).arg(std::max(1, static_cast<int>(planned_selection_.members.size()) - 1));
  applying_ = true;
  const bool committed = hooks_.commit && hooks_.commit(preview_.request);
  applying_ = false;
  if (!committed) return fail(QStringLiteral("%1 was not applied; the scene is unchanged.").arg(label_));
  active_ = false;
  preview_ = {};
  status_ = message;
  emit changed();
  emit finished(true, message);
  return true;
}

void ArrangeController::scene_changed()
{
  if (!active_ || applying_) return;
  const QString blocker = availability(op_, params_);
  if (!blocker.isEmpty()) {
    cancel(QStringLiteral("%1 closed: %2 Nothing was applied.").arg(label_, blocker));
    return;
  }
  replan();
  emit changed();
}

QJsonObject ArrangeController::overlay() const
{
  QJsonObject overlay{{"active", active_}};
  if (!active_) return overlay;
  overlay.insert("members", QJsonArray::fromStringList(planned_selection_.members));
  overlay.insert("reference", planned_selection_.reference);
  overlay.insert("ghost", preview_.ok ? preview_.ghost : QJsonArray{});
  overlay.insert("preview_ok", preview_.ok);
  return overlay;
}

QJsonObject ArrangeController::evidence() const
{
  return {{"active", active_}, {"op", op_}, {"label", label_}, {"params", params_}, {"status", status_},
    {"selection", QJsonObject{{"members", QJsonArray::fromStringList(planned_selection_.members)},
      {"reference", planned_selection_.reference}}},
    {"preview", QJsonObject{{"ok", preview_.ok}, {"error", preview_.error}, {"request", preview_.request},
      {"report", preview_.report}, {"moved", preview_.moved}, {"summary", preview_.summary}}}};
}

}  // namespace workcell_builder
