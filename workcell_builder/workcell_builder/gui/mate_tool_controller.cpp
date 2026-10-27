#include "mate_tool_controller.hpp"

#include <QJsonDocument>
#include <QtMath>

#include <cmath>
#include <utility>

namespace workcell_builder
{
namespace
{

QString step_name(MateToolController::Step step)
{
  switch (step) {
    case MateToolController::Step::PickChild: return QStringLiteral("pick_child");
    case MateToolController::Step::PickParent: return QStringLiteral("pick_parent");
    case MateToolController::Step::Ready: return QStringLiteral("ready");
    default: return QStringLiteral("inactive");
  }
}

QJsonArray json3(const MateVec3 & v) { return QJsonArray{v[0], v[1], v[2]}; }

QJsonObject pick_json(const std::optional<MateToolController::Pick> & pick)
{
  if (!pick) return {};
  return {{"asset_id", pick->asset_id}, {"datum", pick->datum}, {"datum_type", pick->datum_type},
    {"datum_label", pick->datum_label}, {"how", pick->how},
    {"resolution", pick->resolution.reason_code.isEmpty() ? QJsonValue() : QJsonValue(mate_pick_to_json(pick->resolution))}};
}

}  // namespace

MateToolController::MateToolController(Hooks hooks, QObject * parent)
: QObject(parent), hooks_(std::move(hooks))
{
}

QString MateToolController::display_name(const QString & id) const
{
  const QString name = hooks_.display_name ? hooks_.display_name(id) : QString();
  return name.trimmed().isEmpty() ? id : name.trimmed();
}

void MateToolController::set_status(const QString & status)
{
  status_ = status;
}

bool MateToolController::refresh_frames(QString * error)
{
  const auto response = hooks_.run_resolver ? hooks_.run_resolver(QJsonObject{{"operation", "datum_frames"}}) : QJsonObject{};
  if (!response.value("ok").toBool()) {
    if (error) {
      *error = response.value("error").toString().trimmed();
      if (error->isEmpty()) *error = QStringLiteral("The asset mounting resolver did not return datum frames.");
    }
    return false;
  }
  frames_ = parse_mate_frames(response.value("result").toObject());
  return true;
}

bool MateToolController::begin(const QString & selected_asset, QString * error)
{
  QString reason;
  if (!refresh_frames(&reason)) {
    if (error) *error = QStringLiteral("Mate cannot start: %1").arg(reason);
    return false;
  }
  if (frames_.isEmpty()) {
    if (error) *error = QStringLiteral("Mate cannot start: the cell has no editable physical assets.");
    return false;
  }
  step_ = Step::PickChild;
  child_.reset(); parent_.reset(); preview_ = {};
  alignment_ = QStringLiteral("flush"); alignment_chosen_ = false; offset_ = {0., 0., 0., 0., 0., 0.};
  isolate_ = false; view_ = {};
  picks_log_ = {};
  pending_asset_ = frames_.contains(selected_asset) ? selected_asset : QString();
  set_status(pending_asset_.isEmpty()
    ? QStringLiteral("Step 1 of 2: click the face of the part you want to move, for example the bottom of a bin.")
    : QStringLiteral("Step 1 of 2: click the face of %1 that should touch the other part. Its mounting faces are marked; "
        "pick a marker if the face is hidden.").arg(display_name(pending_asset_)));
  emit changed();
  return true;
}

void MateToolController::cancel(const QString & why)
{
  if (step_ == Step::Inactive) return;
  step_ = Step::Inactive;
  child_.reset(); parent_.reset(); preview_ = {}; pending_asset_.clear();
  set_status(why.isEmpty() ? QStringLiteral("Mate cancelled; nothing changed.") : why);
  emit changed();
  emit finished(false, status_);
}

QString MateToolController::friendly_reason(const MatePickResolution & r) const
{
  const QString name = display_name(r.asset_id);
  const QString code = r.reason_code;
  if (code == QStringLiteral("MATCHED") || code == QStringLiteral("DATUM_MARKER")) {
    return QStringLiteral("%1 · %2").arg(name, r.datum_label);
  }
  if (code == QStringLiteral("NO_MATCHING_DATUM")) {
    QStringList verified;
    if (frames_.contains(r.asset_id)) {
      for (const auto & datum : frames_.value(r.asset_id).datums) {
        if (datum.type == QStringLiteral("plane")) verified << QStringLiteral("%1 (%2)").arg(datum.name, mate_direction_text(datum.normal));
      }
    }
    return QStringLiteral("This face of %1 (%2) is not a verified mounting surface. Verified surfaces: %3. Click one of "
      "them or its marker, or choose a datum from the list.")
      .arg(name, mate_direction_text(r.normal_world), verified.join(QStringLiteral(", ")));
  }
  if (code == QStringLiteral("NO_PLANE_DATUMS")) {
    return QStringLiteral("%1 has no verified mounting faces. Click one of its datum markers, or choose a datum from the list.").arg(name);
  }
  if (code == QStringLiteral("PLACEHOLDER_GEOMETRY")) {
    return QStringLiteral("%1 is drawn as a placeholder shape, not its real mesh, so its faces cannot be trusted. Click its "
      "datum marker, or choose a datum from the list.").arg(name);
  }
  if (code == QStringLiteral("STALE_VIEW")) {
    return QStringLiteral("The 3D view draws %1 away from its authored pose (it may still be refreshing). Click its datum "
      "marker, or wait and pick again.").arg(name);
  }
  if (code == QStringLiteral("NOT_A_MATING_ASSET")) {
    return QStringLiteral("%1 cannot be mated: robots, generated previews and task zones are not placed assets.").arg(name);
  }
  if (code == QStringLiteral("NO_ASSET")) return QStringLiteral("That click did not hit a part. Click a face of a placed asset.");
  QString reason = r.reason;
  if (!r.asset_id.isEmpty()) reason.replace(r.asset_id, name);
  return reason;
}

QString MateToolController::short_reason(const MatePickResolution & r)
{
  const QString code = r.reason_code;
  if (code == QStringLiteral("NO_MATCHING_DATUM")) return QStringLiteral("not a mounting surface");
  if (code == QStringLiteral("NO_PLANE_DATUMS")) return QStringLiteral("no mounting faces; use a datum marker");
  if (code == QStringLiteral("PLACEHOLDER_GEOMETRY")) return QStringLiteral("placeholder shape; use its datum marker");
  if (code == QStringLiteral("STALE_VIEW")) return QStringLiteral("view not current; use its datum marker");
  if (code == QStringLiteral("NOT_A_MATING_ASSET")) return QStringLiteral("cannot be mated");
  if (code == QStringLiteral("AMBIGUOUS_DATUM")) return QStringLiteral("matches two datums; choose one from the list");
  if (code == QStringLiteral("DEGENERATE_NORMAL")) return QStringLiteral("no usable face normal here");
  return r.reason;
}

MatePickResolution MateToolController::hover(const MateRawPick & raw) const
{
  return resolve_mate_pick(raw, frames_);
}

MatePickResolution MateToolController::pick(const MateRawPick & raw)
{
  const auto resolution = resolve_mate_pick(raw, frames_);
  if (step_ == Step::Inactive) return resolution;
  picks_log_.append(QJsonObject{{"step", step_name(step_)}, {"resolution", mate_pick_to_json(resolution)}});
  // A click that hits no part (background, floor, a hidden part) is a miss: it never
  // changes, replaces or cancels a pick. Say so, and say what is still picked.
  const bool missed = resolution.reason_code == QStringLiteral("NO_ASSET");
  const auto kept = [this](const std::optional<Pick> & pick) {
    return QStringLiteral("%1 · %2").arg(display_name(pick->asset_id), pick->datum_label.isEmpty() ? pick->datum : pick->datum_label);
  };
  if (step_ == Step::Ready) {
    set_status(missed
      ? QStringLiteral("That click missed every part, so nothing changed. Still picked: %1 on %2. Apply, or Replace one.")
          .arg(kept(child_), kept(parent_))
      : QStringLiteral("Both faces are picked. Use Replace to change one, or Apply."));
    emit changed();
    return resolution;
  }
  const Role role = step_ == Step::PickChild ? Role::Child : Role::Parent;
  if (!resolution.valid) {
    pending_asset_ = frames_.contains(resolution.asset_id) ? resolution.asset_id : pending_asset_;
    QString status = friendly_reason(resolution);
    if (missed && child_) status += QStringLiteral(" %1 stays picked as the part to move.").arg(kept(child_));
    set_status(status);
    emit changed();
    return resolution;
  }
  if (role == Role::Parent && child_ && resolution.asset_id == child_->asset_id) {
    set_status(QStringLiteral("%1 cannot be mated to itself. Click a face on the part it should sit on.")
      .arg(display_name(resolution.asset_id)));
    emit changed();
    return resolution;
  }
  Pick pick;
  pick.asset_id = resolution.asset_id; pick.datum = resolution.datum_name; pick.datum_type = resolution.datum_type;
  pick.datum_label = resolution.datum_label;
  pick.how = raw.geometry == MatePickGeometry::DatumMarker ? QStringLiteral("clicked datum marker") : QStringLiteral("clicked face");
  pick.resolution = resolution;
  accept_pick(role, pick);
  return resolution;
}

void MateToolController::hint_asset(const QString & asset_id)
{
  if (step_ == Step::Inactive || asset_id.isEmpty()) return;
  picks_log_.append(QJsonObject{{"step", step_name(step_)}, {"asset_hint", asset_id}});
  if (!frames_.contains(asset_id)) {
    set_status(QStringLiteral("%1 is not an editable physical asset. Robots, generated previews and task zones cannot be mated.")
      .arg(display_name(asset_id)));
  } else if (step_ == Step::Ready) {
    set_status(QStringLiteral("Both parts are picked. Use Replace to change one, or Apply."));
  } else if (step_ == Step::PickParent && child_ && child_->asset_id == asset_id) {
    set_status(QStringLiteral("%1 cannot be mated to itself. Choose the part it should sit on.").arg(display_name(asset_id)));
  } else {
    pending_asset_ = asset_id;
    set_status(QStringLiteral("%1 selected. Click its face in the 3D view, or choose its datum from the list.")
      .arg(display_name(asset_id)));
  }
  emit changed();
}

bool MateToolController::choose_datum(Role role, const QString & asset_id, const QString & datum, QString * error)
{
  const auto fail = [error](const QString & message) { if (error) *error = message; return false; };
  if (step_ == Step::Inactive) return fail(QStringLiteral("The Mate tool is not active."));
  const auto found = frames_.constFind(asset_id);
  if (found == frames_.constEnd()) return fail(QStringLiteral("%1 is not an editable physical asset.").arg(asset_id));
  const MateDatumFrame * frame = found.value().datum(datum);
  if (!frame) return fail(QStringLiteral("%1 has no datum named %2.").arg(display_name(asset_id), datum));
  if (role == Role::Parent && child_ && child_->asset_id == asset_id) {
    return fail(QStringLiteral("%1 cannot be mated to itself.").arg(display_name(asset_id)));
  }
  if (role == Role::Child && parent_ && parent_->asset_id == asset_id) parent_.reset();
  Pick pick;
  pick.asset_id = asset_id; pick.datum = frame->name; pick.datum_type = frame->type;
  pick.datum_label = mate_datum_display(*frame); pick.how = QStringLiteral("chosen manually");
  picks_log_.append(QJsonObject{{"step", step_name(step_)}, {"manual", QJsonObject{{"asset_id", asset_id}, {"datum", datum},
    {"role", role == Role::Child ? "child" : "parent"}}}});
  accept_pick(role, pick);
  return true;
}

void MateToolController::accept_pick(Role role, const Pick & pick)
{
  pending_asset_.clear();
  // A hidden face was reached through isolation or a face view: back to the
  // whole cell for the next pick.
  if (isolate_ || (!view_.isEmpty() && !view_.value("reset").toBool())) {
    isolate_ = false;
    view_ = QJsonObject{{"seq", ++view_seq_}, {"reset", true}};
  }
  if (role == Role::Child) {
    child_ = pick;
    if (parent_ && parent_->asset_id == pick.asset_id) parent_.reset();
  } else {
    parent_ = pick;
  }
  if (!child_) {
    step_ = Step::PickChild;
    set_status(QStringLiteral("Step 1 of 2: click the face of the part you want to move."));
  } else if (!parent_) {
    step_ = Step::PickParent;
    set_status(QStringLiteral("Step 2 of 2: click the face on the part %1 should sit on, for example the table top.")
      .arg(display_name(child_->asset_id)));
  } else {
    step_ = Step::Ready;
    if (!alignment_chosen_) {
      alignment_ = child_->datum_type == QStringLiteral("plane") && parent_->datum_type == QStringLiteral("plane")
        ? QStringLiteral("flush") : QStringLiteral("coincident");
    }
    set_status(QStringLiteral("Check the preview, then Apply."));
  }
  update_preview(true);
  emit changed();
}

void MateToolController::replace(Role role)
{
  if (step_ == Step::Inactive) return;
  if (role == Role::Child) {
    child_.reset();
    step_ = Step::PickChild;
    set_status(QStringLiteral("Click the new face on the part you want to move."));
  } else {
    parent_.reset();
    step_ = child_ ? Step::PickParent : Step::PickChild;
    set_status(QStringLiteral("Click the new face on the part it should sit on."));
  }
  preview_ = {};
  emit changed();
}

bool MateToolController::flush_allowed(QString * reason) const
{
  if (!child_ || !parent_) return true;
  if (child_->datum_type == QStringLiteral("plane") && parent_->datum_type == QStringLiteral("plane")) return true;
  if (reason) {
    QStringList not_planes;
    for (const auto * pick : {&*child_, &*parent_}) {
      if (pick->datum_type == QStringLiteral("plane")) continue;
      const bool vowel = !pick->datum_type.isEmpty() && QStringLiteral("aeiou").contains(pick->datum_type.at(0));
      not_planes << QStringLiteral("%1 is %2 %3 datum").arg(pick->datum, vowel ? QStringLiteral("an") : QStringLiteral("a"), pick->datum_type);
    }
    *reason = QStringLiteral("Flush needs two plane faces (%1). Use Coincident.").arg(not_planes.join(QStringLiteral(", ")));
  }
  return false;
}

QString MateToolController::focus_asset() const
{
  if (!pending_asset_.isEmpty()) return pending_asset_;
  if (step_ == Step::PickChild) return child_ ? child_->asset_id : QString();
  if (step_ == Step::PickParent) return parent_ ? parent_->asset_id : (child_ ? child_->asset_id : QString());
  return parent_ ? parent_->asset_id : QString();
}

void MateToolController::set_isolate(bool isolate)
{
  if (isolate_ == isolate) return;
  isolate_ = isolate;
  emit changed();
}

bool MateToolController::view_datum(const QString & datum, QString * error)
{
  const QString asset = focus_asset();
  const auto found = frames_.constFind(asset);
  const MateDatumFrame * frame = found == frames_.constEnd() ? nullptr : found.value().datum(datum);
  if (!frame) {
    if (error) *error = QStringLiteral("Pick or choose a part first, then a datum to look at.");
    return false;
  }
  view_ = QJsonObject{{"seq", ++view_seq_}, {"asset_id", asset}, {"datum", datum},
    {"origin", QJsonArray{frame->origin[0], frame->origin[1], frame->origin[2]}},
    {"normal", QJsonArray{frame->normal[0], frame->normal[1], frame->normal[2]}}};
  picks_log_.append(QJsonObject{{"step", step_name(step_)}, {"face_view", view_}, {"isolate", isolate_}});
  emit changed();
  return true;
}

void MateToolController::set_alignment(const QString & alignment)
{
  if (alignment == alignment_ && alignment_chosen_) return;
  alignment_ = alignment;
  alignment_chosen_ = true;
  update_preview(true);
  emit changed();
}

void MateToolController::set_offset(const std::array<double, 6> & offset)
{
  offset_ = offset;
  update_preview(false);
  emit changed();
}

void MateToolController::reset_offset()
{
  update_preview(true);
  emit changed();
}

QString MateToolController::readable(QString error) const
{
  // Resolver messages name assets by ID; show the names the user sees.
  for (const auto & asset : frames_) {
    const QString name = display_name(asset.asset_id);
    if (name == asset.asset_id) continue;
    error.replace(QStringLiteral("'%1'").arg(asset.asset_id), name);
    error.replace(QStringLiteral("asset %1:").arg(asset.asset_id), name + QStringLiteral(":"));
    error.replace(QStringLiteral("%1.").arg(asset.asset_id), name + QStringLiteral("."));
  }
  return error;
}

void MateToolController::update_preview(bool reseat)
{
  preview_ = {};
  if (!child_ || !parent_) return;
  QJsonObject request{{"id", child_->asset_id}, {"parent_object", parent_->asset_id}, {"parent_link", parent_->datum},
    {"child_datum", child_->datum}, {"alignment", alignment_}};
  const auto relation_origin = [](const QJsonArray & rows, const QString & id, std::array<double, 6> * out) {
    for (const auto & value : rows) {
      const auto row = value.toObject();
      if (row.value("id").toString() != id) continue;
      const auto relation = QJsonDocument::fromJson(row.value("mounting_yaml").toString().toUtf8()).object();
      const auto origin = relation.value("origin").toObject();
      if (origin.isEmpty()) return false;
      const char * keys[6] = {"x", "y", "z", "roll", "pitch", "yaw"};
      for (int k = 0; k < 6; ++k) (*out)[k] = origin.value(keys[k]).toDouble();
      return true;
    }
    return false;
  };
  if (reseat) {
    // Measure where the child datum is now, in the parent datum frame (the
    // resolver keeps the world pose when no offset is given), then seat it.
    const auto measured = hooks_.run_resolver(request);
    std::array<double, 6> origin{{0., 0., 0., 0., 0., 0.}};
    if (!measured.value("ok").toBool() || !relation_origin(measured.value("result").toArray(), child_->asset_id, &origin)) {
      preview_.ok = false;
      preview_.request = request;
      preview_.error = measured.value("error").toString().trimmed().isEmpty()
        ? QStringLiteral("The resolver could not measure the current offset.") : readable(measured.value("error").toString());
      return;
    }
    offset_ = seated_mate_offset(origin);
  }
  request.insert("relative_pose", QJsonObject{{"xyz", QJsonArray{offset_[0], offset_[1], offset_[2]}},
    {"rpy", QJsonArray{offset_[3], offset_[4], offset_[5]}}});
  preview_.request = request;
  preview_.offset = offset_;
  const auto result = hooks_.run_resolver(request);
  if (!result.value("ok").toBool()) {
    preview_.ok = false;
    preview_.error = readable(result.value("error").toString());
    return;
  }
  preview_.ok = true;
  preview_.rows = result.value("result").toArray();
  QHash<QString, QJsonObject> before;
  if (hooks_.current_items) {
    for (const auto & value : hooks_.current_items()) before.insert(value.toObject().value("id").toString(), value.toObject());
  }
  for (const auto & value : preview_.rows) {
    const auto row = value.toObject();
    const QString id = row.value("id").toString();
    const auto old = before.value(id);
    const auto xyz = row.value("pose_xyz").toArray(), rpy = row.value("pose_rpy").toArray();
    const auto oxyz = old.value("pose_xyz").toArray(), orpy = old.value("pose_rpy").toArray();
    if (oxyz.size() != 3 || orpy.size() != 3) continue;
    const double moved = std::hypot(xyz[0].toDouble() - oxyz[0].toDouble(), xyz[1].toDouble() - oxyz[1].toDouble(),
      xyz[2].toDouble() - oxyz[2].toDouble());
    double turned = 0.;
    for (int k = 0; k < 3; ++k) {
      turned = std::max(turned, std::abs(std::remainder(rpy[k].toDouble() - orpy[k].toDouble(), 2. * M_PI)));
    }
    if (id == child_->asset_id) {
      preview_.child_summary = QStringLiteral("%1 moves %2 mm to X %3 · Y %4 · Z %5 m, roll %6° · pitch %7° · yaw %8°.")
        .arg(display_name(id)).arg(moved * 1000., 0, 'f', 1)
        .arg(xyz[0].toDouble(), 0, 'f', 4).arg(xyz[1].toDouble(), 0, 'f', 4).arg(xyz[2].toDouble(), 0, 'f', 4)
        .arg(qRadiansToDegrees(rpy[0].toDouble()), 0, 'f', 1).arg(qRadiansToDegrees(rpy[1].toDouble()), 0, 'f', 1)
        .arg(qRadiansToDegrees(rpy[2].toDouble()), 0, 'f', 1);
    }
    if (moved <= 1e-9 && turned <= 1e-9) continue;
    preview_.moved.append(QJsonObject{{"id", id}, {"display_name", display_name(id)}, {"xyz", xyz}, {"rpy", rpy},
      {"moved_m", moved}, {"turned_rad", turned}});
    if (id != child_->asset_id) preview_.followers << display_name(id);
    if (hooks_.zones_following) {
      for (const auto & zone : hooks_.zones_following(id)) {
        preview_.followers << QStringLiteral("%1 (destination zone, keeps its target-local placement)").arg(zone);
      }
    }
  }
}

bool MateToolController::apply(QString * error)
{
  if (step_ != Step::Ready || !preview_.ok) {
    if (error) *error = preview_.error.isEmpty() ? QStringLiteral("Pick both faces and check the preview first.") : preview_.error;
    return false;
  }
  const auto request = preview_.request;
  applying_ = true;
  const bool committed = hooks_.commit && hooks_.commit(request);
  applying_ = false;
  if (!committed) {
    if (error) *error = QStringLiteral("The mate was not applied; the scene is unchanged.");
    return false;
  }
  const QString message = QStringLiteral("Mated %1 (%2) %3 %4 (%5).")
    .arg(display_name(child_->asset_id), child_->datum,
      alignment_ == QStringLiteral("flush") ? QStringLiteral("flush on") : QStringLiteral("coincident with"),
      display_name(parent_->asset_id), parent_->datum);
  step_ = Step::Inactive;
  child_.reset(); parent_.reset(); preview_ = {}; pending_asset_.clear();
  set_status(message);
  emit changed();
  emit finished(true, message);
  return true;
}

void MateToolController::scene_changed()
{
  if (step_ == Step::Inactive || applying_) return;
  QString reason;
  if (!refresh_frames(&reason)) {
    cancel(QStringLiteral("Mate cancelled: the scene changed and its datums could not be read (%1). Nothing was applied.").arg(reason));
    return;
  }
  QStringList dropped;
  for (auto * pick : {&child_, &parent_}) {
    if (!*pick) continue;
    const auto found = frames_.constFind((*pick)->asset_id);
    if (found == frames_.constEnd() || !found.value().datum((*pick)->datum)) {
      dropped << display_name((*pick)->asset_id);
      pick->reset();
    }
  }
  step_ = !child_ ? Step::PickChild : (!parent_ ? Step::PickParent : Step::Ready);
  if (!dropped.isEmpty()) {
    set_status(QStringLiteral("The scene changed: the picked face of %1 no longer exists. Pick again.")
      .arg(dropped.join(QStringLiteral(", "))));
  }
  update_preview(false);
  emit changed();
}

QJsonObject MateToolController::overlay() const
{
  QJsonObject overlay{{"active", step_ != Step::Inactive}, {"step", step_name(step_)}};
  if (step_ == Step::Inactive) return overlay;
  QJsonArray markers;
  for (const auto & asset : frames_) {
    for (const auto & datum : asset.datums) {
      QString role = QStringLiteral("candidate");
      if (child_ && child_->asset_id == asset.asset_id && child_->datum == datum.name) role = QStringLiteral("child");
      if (parent_ && parent_->asset_id == asset.asset_id && parent_->datum == datum.name) role = QStringLiteral("parent");
      markers.append(QJsonObject{{"asset_id", asset.asset_id}, {"datum", datum.name}, {"type", datum.type},
        {"label", mate_datum_display(datum)}, {"origin", json3(datum.origin)}, {"normal", json3(datum.normal)},
        {"x_axis", json3(datum.x_axis)}, {"role", role}});
    }
  }
  overlay.insert("markers", markers);
  overlay.insert("child_asset", child_ ? child_->asset_id : QString());
  overlay.insert("parent_asset", parent_ ? parent_->asset_id : QString());
  overlay.insert("pending_asset", pending_asset_);
  QJsonArray ghost;
  if (preview_.ok) {
    for (const auto & value : preview_.moved) {
      const auto row = value.toObject();
      ghost.append(QJsonObject{{"id", row.value("id")}, {"xyz", row.value("xyz")}, {"rpy", row.value("rpy")}});
    }
  }
  overlay.insert("ghost", ghost);
  overlay.insert("preview_ok", preview_.ok);
  overlay.insert("isolate_asset", isolate_ ? focus_asset() : QString());
  if (!view_.isEmpty()) overlay.insert("view", view_);
  return overlay;
}

QJsonObject MateToolController::evidence() const
{
  return {{"step", step_name(step_)}, {"status", status_}, {"backend_note", backend_note_},
    {"child", pick_json(child_)}, {"parent", pick_json(parent_)},
    {"alignment", alignment_}, {"offset", QJsonArray{offset_[0], offset_[1], offset_[2], offset_[3], offset_[4], offset_[5]}},
    {"preview", QJsonObject{{"ok", preview_.ok}, {"error", preview_.error}, {"request", preview_.request},
      {"moved", preview_.moved}, {"child_summary", preview_.child_summary},
      {"followers", QJsonArray::fromStringList(preview_.followers)}}},
    {"pending_asset", pending_asset_}, {"picks", picks_log_}};
}

}  // namespace workcell_builder
