#include "align_arrange_controller.hpp"

#include <QHash>
#include <QtMath>

#include <algorithm>
#include <cmath>
#include <utility>

namespace workcell_builder
{
namespace
{

const char * const kAxisKeys[3] = {"x", "y", "z"};

QString signed_mm(double metres)
{
  const double mm = std::abs(metres) < 5e-5 ? 0. : metres * 1000.;
  return QStringLiteral("%1%2").arg(mm > 0. ? QStringLiteral("+") : QString()).arg(mm, 0, 'f', 1);
}

}  // namespace

AlignArrangeController::AlignArrangeController(Hooks hooks, QObject * parent)
: QObject(parent), hooks_(std::move(hooks))
{
}

QString AlignArrangeController::display_name(const QString & id) const
{
  const QString name = hooks_.display_name ? hooks_.display_name(id).trimmed() : QString();
  return name.isEmpty() ? id : name;
}

QString AlignArrangeController::rule_label(Axis axis, const QString & rule)
{
  if (rule == QStringLiteral("centre")) return QStringLiteral("Centre %1").arg(QLatin1Char("XYZ"[static_cast<int>(axis)]));
  if (rule == QStringLiteral("on_top")) return QStringLiteral("Place on top");
  static const char * const kMin[3] = {"Align left", "Align front", "Align bottom"};
  static const char * const kMax[3] = {"Align right", "Align back", "Align top"};
  if (rule == QStringLiteral("min")) return QString::fromLatin1(kMin[static_cast<int>(axis)]);
  if (rule == QStringLiteral("max")) return QString::fromLatin1(kMax[static_cast<int>(axis)]);
  return QString();
}

bool AlignArrangeController::has_rules() const
{
  for (const auto & rule : rules_) if (!rule.isEmpty()) return true;
  return false;
}

QString AlignArrangeController::operation_text() const
{
  QStringList parts;
  for (int k = 0; k < 3; ++k) {
    if (rules_[k].isEmpty()) continue;
    QString text = rule_label(static_cast<Axis>(k), rules_[k]);
    if (std::abs(offset_mm_[k]) > 1e-9) text += QStringLiteral(" %1 mm").arg(signed_mm(offset_mm_[k] / 1000.));
    parts << text;
  }
  return parts.join(QStringLiteral(", "));
}

bool AlignArrangeController::begin(const QString & moving, QString * error)
{
  const QString id = moving.trimmed();
  const auto fail = [error](const QString & message) { if (error) *error = message; return false; };
  if (id.isEmpty()) return fail(QStringLiteral("Select the part to move first, then choose Align & Arrange."));
  const QString blocker = hooks_.moving_blocker ? hooks_.moving_blocker(id) : QString();
  if (!blocker.isEmpty()) return fail(blocker);
  active_ = true;
  moving_ = id;
  reference_.clear();
  reference_world_ = {};
  rules_ = {};
  offset_mm_ = {0., 0., 0.};
  keep_relative_ = false;
  preview_ = {};
  status_ = QStringLiteral("Choose the reference part: click it in the 3D view or the Scene Hierarchy, or pick it "
    "from the list.");
  emit changed();
  return true;
}

void AlignArrangeController::cancel(const QString & why)
{
  if (!active_) return;
  active_ = false;
  preview_ = {};
  status_ = why.isEmpty() ? QStringLiteral("Align & Arrange cancelled; nothing changed.") : why;
  emit changed();
  emit finished(false, status_);
}

void AlignArrangeController::set_reference(const QString & id)
{
  if (!active_) return;
  const QString candidate = id.trimmed();
  if (candidate.isEmpty() || candidate == reference_) return;
  if (candidate == moving_) {
    status_ = QStringLiteral("%1 is the part being moved. Choose a different part as the reference.").arg(display_name(moving_));
    emit changed();
    return;
  }
  QJsonObject world;
  const QString blocker = hooks_.reference_blocker ? hooks_.reference_blocker(candidate, &world) : QString();
  if (!blocker.isEmpty()) {
    status_ = blocker;
    emit changed();
    return;
  }
  reference_ = candidate;
  reference_world_ = world;
  if (keep_relative_ && !keep_relative_blocker().isEmpty()) keep_relative_ = false;
  replan();
  emit changed();
}

void AlignArrangeController::set_rule(Axis axis, const QString & rule)
{
  if (!active_) return;
  auto & current = rules_[static_cast<int>(axis)];
  current = current == rule ? QString() : rule;
  replan();
  emit changed();
}

void AlignArrangeController::centre_on_reference()
{
  if (!active_) return;
  const bool both = rules_[0] == QStringLiteral("centre") && rules_[1] == QStringLiteral("centre");
  rules_[0] = rules_[1] = both ? QString() : QStringLiteral("centre");
  replan();
  emit changed();
}

void AlignArrangeController::clear_rules()
{
  if (!active_) return;
  rules_ = {};
  offset_mm_ = {0., 0., 0.};
  replan();
  emit changed();
}

void AlignArrangeController::set_offset_mm(Axis axis, double millimetres)
{
  if (!active_ || !std::isfinite(millimetres)) return;
  offset_mm_[static_cast<int>(axis)] = millimetres;
  replan();
  emit changed();
}

void AlignArrangeController::set_keep_relative(bool keep)
{
  if (!active_ || keep == keep_relative_) return;
  if (keep && !keep_relative_blocker().isEmpty()) {
    status_ = keep_relative_blocker();
    emit changed();
    return;
  }
  keep_relative_ = keep;
  replan();
  emit changed();
}

QString AlignArrangeController::keep_relative_blocker() const
{
  if (reference_.isEmpty()) return QStringLiteral("Choose a reference part first.");
  if (!reference_world_.isEmpty()) {
    return QStringLiteral("%1 is locked, so %2 cannot be mounted on it. Use Align once, or unlock the reference.")
      .arg(display_name(reference_), display_name(moving_));
  }
  return QString();
}

void AlignArrangeController::replan()
{
  preview_ = {};
  if (reference_.isEmpty()) {
    status_ = QStringLiteral("Choose the reference part: click it in the 3D view or the Scene Hierarchy, or pick it "
      "from the list.");
    return;
  }
  if (!has_rules()) {
    status_ = QStringLiteral("Choose how %1 lines up with %2: Centre, Align or Place on top.")
      .arg(display_name(moving_), display_name(reference_));
    return;
  }
  QJsonObject rules;
  QJsonArray offset;
  for (int k = 0; k < 3; ++k) {
    rules.insert(kAxisKeys[k], rules_[k].isEmpty() ? QJsonValue() : QJsonValue(rules_[k]));
    offset.append(offset_mm_[k] / 1000.);
  }
  QJsonObject names{{moving_, display_name(moving_)}, {reference_, display_name(reference_)}};
  QJsonObject request{{"operation", "align"}, {"id", moving_}, {"reference", reference_}, {"rules", rules},
    {"offset_m", offset}, {"keep_relative", keep_relative_}, {"names", names}};
  if (!reference_world_.isEmpty()) request.insert("reference_world", reference_world_);
  if (hooks_.live_dimensions) request.insert("live_dimensions", hooks_.live_dimensions());
  const auto response = hooks_.run_resolver ? hooks_.run_resolver(request) : QJsonObject{};
  if (!response.value("ok").toBool()) {
    preview_.error = response.value("error").toString().trimmed();
    if (preview_.error.isEmpty()) preview_.error = QStringLiteral("The alignment could not be planned; nothing changed.");
    status_ = preview_.error;
    return;
  }
  const auto result = response.value("result").toObject();
  preview_.ok = true;
  preview_.request = result.value("request").toObject();
  preview_.report = result.value("report").toObject();
  QHash<QString, QJsonObject> before;
  if (hooks_.current_items) {
    for (const auto & value : hooks_.current_items()) before.insert(value.toObject().value("id").toString(), value.toObject());
  }
  for (const auto & value : result.value("rows").toArray()) {
    const auto row = value.toObject();
    const QString id = row.value("id").toString();
    const auto old = before.value(id);
    const auto xyz = row.value("pose_xyz").toArray(), rpy = row.value("pose_rpy").toArray();
    const auto oxyz = old.value("pose_xyz").toArray(), orpy = old.value("pose_rpy").toArray();
    if (xyz.size() != 3 || oxyz.size() != 3 || rpy.size() != 3 || orpy.size() != 3) continue;
    const double moved = std::hypot(xyz[0].toDouble() - oxyz[0].toDouble(), xyz[1].toDouble() - oxyz[1].toDouble(),
      xyz[2].toDouble() - oxyz[2].toDouble());
    double turned = 0.;
    for (int k = 0; k < 3; ++k) {
      turned = std::max(turned, std::abs(std::remainder(rpy[k].toDouble() - orpy[k].toDouble(), 2. * M_PI)));
    }
    if (moved <= 1e-9 && turned <= 1e-9) continue;
    preview_.moved.append(QJsonObject{{"id", id}, {"display_name", display_name(id)}, {"xyz", xyz}, {"rpy", rpy},
      {"moved_m", moved}});
    if (id != moving_) preview_.followers << display_name(id);
  }
  const auto report = preview_.report;
  const auto delta = report.value("delta_world_m").toArray();
  const auto target = report.value("target_world").toObject().value("xyz").toArray();
  if (delta.size() == 3 && target.size() == 3) {
    const double distance = report.value("distance_m").toDouble();
    preview_.summary = distance < 5e-8
      ? QStringLiteral("%1 is already in this position; Apply changes nothing.").arg(display_name(moving_))
      : QStringLiteral("%1 moves %2 mm (world ΔX %3 · ΔY %4 · ΔZ %5 mm) to X %6 · Y %7 · Z %8 m. Orientation unchanged.")
          .arg(display_name(moving_)).arg(distance * 1000., 0, 'f', 1)
          .arg(signed_mm(delta[0].toDouble()), signed_mm(delta[1].toDouble()), signed_mm(delta[2].toDouble()))
          .arg(target[0].toDouble(), 0, 'f', 4).arg(target[1].toDouble(), 0, 'f', 4).arg(target[2].toDouble(), 0, 'f', 4);
  }
  status_ = QStringLiteral("Check the ghost and the numbers, then Apply.");
}

bool AlignArrangeController::apply(QString * error)
{
  if (!active_ || !preview_.ok) {
    if (error) *error = preview_.error.isEmpty() ? QStringLiteral("Choose a reference and an operation first.") : preview_.error;
    return false;
  }
  const QString operations = operation_text();
  const QString message = keep_relative_
    ? QStringLiteral("%1 aligned to %2 (%3) and kept relative to it: it now follows %2.")
        .arg(display_name(moving_), display_name(reference_), operations)
    : QStringLiteral("%1 aligned to %2 (%3). Undo reverts it.").arg(display_name(moving_), display_name(reference_), operations);
  applying_ = true;
  const bool committed = hooks_.commit && hooks_.commit(preview_.request);
  applying_ = false;
  if (!committed) {
    if (error) *error = QStringLiteral("The alignment was not applied; the scene is unchanged.");
    return false;
  }
  active_ = false;
  preview_ = {};
  status_ = message;
  emit changed();
  emit finished(true, message);
  return true;
}

void AlignArrangeController::scene_changed()
{
  if (!active_ || applying_) return;
  const QString blocker = hooks_.moving_blocker ? hooks_.moving_blocker(moving_) : QString();
  if (!blocker.isEmpty()) {
    cancel(QStringLiteral("Align & Arrange cancelled: %1 Nothing was applied.").arg(blocker));
    return;
  }
  if (!reference_.isEmpty()) {
    QJsonObject world;
    const QString reference_problem = hooks_.reference_blocker ? hooks_.reference_blocker(reference_, &world) : QString();
    if (!reference_problem.isEmpty()) {
      reference_.clear();
      reference_world_ = {};
      keep_relative_ = false;
      replan();
      status_ = QStringLiteral("The scene changed and the reference is no longer usable (%1). Choose another reference.")
        .arg(reference_problem);
      emit changed();
      return;
    }
    reference_world_ = world;
  }
  replan();
  emit changed();
}

QJsonObject AlignArrangeController::overlay() const
{
  QJsonObject overlay{{"active", active_}};
  if (!active_) return overlay;
  overlay.insert("moving", moving_);
  overlay.insert("reference", reference_);
  QJsonArray ghost;
  if (preview_.ok) {
    for (const auto & value : preview_.moved) {
      const auto row = value.toObject();
      ghost.append(QJsonObject{{"id", row.value("id")}, {"xyz", row.value("xyz")}, {"rpy", row.value("rpy")}});
    }
  }
  overlay.insert("ghost", ghost);
  overlay.insert("preview_ok", preview_.ok);
  return overlay;
}

QJsonObject AlignArrangeController::evidence() const
{
  QJsonObject rules;
  QJsonArray offset;
  for (int k = 0; k < 3; ++k) {
    rules.insert(kAxisKeys[k], rules_[k]);
    offset.append(offset_mm_[k]);
  }
  return {{"active", active_}, {"moving", moving_}, {"reference", reference_}, {"rules", rules}, {"offset_mm", offset},
    {"keep_relative", keep_relative_}, {"status", status_},
    {"preview", QJsonObject{{"ok", preview_.ok}, {"error", preview_.error}, {"request", preview_.request},
      {"report", preview_.report}, {"moved", preview_.moved}, {"summary", preview_.summary},
      {"followers", QJsonArray::fromStringList(preview_.followers)}}}};
}

}  // namespace workcell_builder
