#include "mate_pick.hpp"

#include <QJsonArray>

#include <algorithm>
#include <cmath>

namespace workcell_builder
{
namespace
{

using Mat3 = std::array<MateVec3, 3>;  // rows

constexpr double kPi = 3.14159265358979323846;

Mat3 rotation_from_rpy(double roll, double pitch, double yaw)
{
  const double cr = std::cos(roll), sr = std::sin(roll), cp = std::cos(pitch), sp = std::sin(pitch);
  const double cy = std::cos(yaw), sy = std::sin(yaw);
  return {{{cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
    {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
    {-sp, cp * sr, cp * cr}}};
}

MateVec3 transposed_apply(const Mat3 & r, const MateVec3 & v)
{
  return {r[0][0] * v[0] + r[1][0] * v[1] + r[2][0] * v[2],
    r[0][1] * v[0] + r[1][1] * v[1] + r[2][1] * v[2],
    r[0][2] * v[0] + r[1][2] * v[1] + r[2][2] * v[2]};
}

double dot(const MateVec3 & a, const MateVec3 & b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

MateVec3 minus(const MateVec3 & a, const MateVec3 & b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }

double norm(const MateVec3 & a) { return std::sqrt(dot(a, a)); }

bool finite(const MateVec3 & a) { return std::isfinite(a[0]) && std::isfinite(a[1]) && std::isfinite(a[2]); }

double rotation_difference(const std::array<double, 6> & a, const std::array<double, 6> & b)
{
  const auto ra = rotation_from_rpy(a[3], a[4], a[5]), rb = rotation_from_rpy(b[3], b[4], b[5]);
  double trace = 0.;
  for (int i = 0; i < 3; ++i) for (int k = 0; k < 3; ++k) trace += ra[k][i] * rb[k][i];
  return std::acos(std::clamp((trace - 1.) / 2., -1., 1.));
}

MateVec3 vec3(const QJsonValue & value)
{
  const auto a = value.toArray();
  if (a.size() != 3) return {std::nan(""), std::nan(""), std::nan("")};
  return {a[0].toDouble(std::nan("")), a[1].toDouble(std::nan("")), a[2].toDouble(std::nan(""))};
}

QJsonArray json3(const MateVec3 & v) { return QJsonArray{v[0], v[1], v[2]}; }

QString millimetres(double metres) { return QString::number(metres * 1000., 'f', 1) + QStringLiteral(" mm"); }

}  // namespace

const MateDatumFrame * MateAssetFrames::datum(const QString & name) const
{
  for (const auto & d : datums) if (d.name == name) return &d;
  return nullptr;
}

MateFrameIndex parse_mate_frames(const QJsonObject & result)
{
  MateFrameIndex index;
  for (auto it = result.begin(); it != result.end(); ++it) {
    const auto asset = it.value().toObject();
    MateAssetFrames frames;
    frames.asset_id = it.key();
    const auto world = asset.value("world").toObject();
    const auto xyz = vec3(world.value("xyz")), rpy = vec3(world.value("rpy"));
    frames.world = {xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2]};
    const auto datums = asset.value("datums").toObject();
    QStringList names = datums.keys();
    std::sort(names.begin(), names.end());
    for (const auto & name : names) {
      const auto d = datums.value(name).toObject();
      MateDatumFrame frame;
      frame.name = name;
      frame.type = d.value("datum_type").toString();
      frame.label = d.value("label").toString();
      frame.source = d.value("source").toString();
      frame.origin = vec3(d.value("xyz"));
      frame.normal = vec3(d.value("normal"));
      frame.x_axis = vec3(d.value("x_axis"));
      frame.rpy = vec3(d.value("rpy"));
      frames.datums.push_back(frame);
    }
    index.insert(frames.asset_id, frames);
  }
  return index;
}

QString mate_direction_text(const MateVec3 & direction)
{
  const double length = norm(direction);
  if (!(length > 1e-12) || !finite(direction)) return QStringLiteral("has no direction");
  const MateVec3 d{direction[0] / length, direction[1] / length, direction[2] / length};
  const double cone = std::cos(20. * kPi / 180.);
  if (d[2] >= cone) return QStringLiteral("points up");
  if (d[2] <= -cone) return QStringLiteral("points down");
  if (std::abs(d[0]) >= cone) return d[0] > 0 ? QStringLiteral("points +X") : QStringLiteral("points −X");
  if (std::abs(d[1]) >= cone) return d[1] > 0 ? QStringLiteral("points +Y") : QStringLiteral("points −Y");
  return QStringLiteral("points (%1, %2, %3)").arg(d[0], 0, 'f', 2).arg(d[1], 0, 'f', 2).arg(d[2], 0, 'f', 2);
}

QString mate_datum_display(const MateDatumFrame & datum)
{
  if (!datum.label.trimmed().isEmpty()) return datum.label.trimmed();
  if (datum.name == QStringLiteral("origin")) return QStringLiteral("Asset origin");
  if (datum.type == QStringLiteral("plane")) {
    return QStringLiteral("%1 · plane (%2)").arg(datum.name, mate_direction_text(datum.normal));
  }
  return QStringLiteral("%1 · %2").arg(datum.name, datum.type);
}

MatePickResolution resolve_mate_pick(const MateRawPick & pick, const MateFrameIndex & frames, const MatePickPolicy & policy)
{
  MatePickResolution r;
  r.raw = pick;
  r.asset_id = pick.asset_id;
  r.point_world = pick.point_world;
  const auto fail = [&r](const QString & code, const QString & reason) {
    r.valid = false; r.reason_code = code; r.reason = reason; return r;
  };
  if (pick.asset_id.trimmed().isEmpty()) {
    return fail(QStringLiteral("NO_ASSET"), QStringLiteral("The click did not hit an asset. Click a face of a placed asset."));
  }
  const auto found = frames.constFind(pick.asset_id);
  if (found == frames.constEnd()) {
    return fail(QStringLiteral("NOT_A_MATING_ASSET"),
      QStringLiteral("%1 is not an editable physical asset. Robots, generated previews and task zones cannot be mated; "
        "click a placed asset such as a table, bin or stand.").arg(pick.asset_id));
  }
  const MateAssetFrames & asset = found.value();
  if (!finite(pick.point_world)) {
    return fail(QStringLiteral("INVALID_HIT"), QStringLiteral("The 3D view reported an invalid hit point; pick again."));
  }
  if (pick.has_object_pose) {
    const MateVec3 drawn{pick.object_pose_world[0], pick.object_pose_world[1], pick.object_pose_world[2]};
    const MateVec3 resolved{asset.world[0], asset.world[1], asset.world[2]};
    const double moved = norm(minus(drawn, resolved));
    const double turned = rotation_difference(pick.object_pose_world, asset.world);
    if (!(moved <= policy.stale_position_m) || !(turned <= policy.stale_angle_rad)) {
      return fail(QStringLiteral("STALE_VIEW"),
        QStringLiteral("The 3D view draws %1 %2 and %3° away from its authored pose (the view may still be refreshing, "
          "or it cannot draw this orientation exactly). Pick its datum marker instead, or wait and pick again.")
          .arg(pick.asset_id, millimetres(moved)).arg(turned * 180. / kPi, 0, 'f', 2));
    }
  }
  const auto r_world = rotation_from_rpy(asset.world[3], asset.world[4], asset.world[5]);
  r.point_local = transposed_apply(r_world, minus(pick.point_world, {asset.world[0], asset.world[1], asset.world[2]}));

  if (pick.geometry == MatePickGeometry::Surface && pick.geometry_source == QStringLiteral("fallback_primitive")) {
    return fail(QStringLiteral("PLACEHOLDER_GEOMETRY"),
      QStringLiteral("The mesh of %1 is not loaded; a placeholder shape was clicked. Wait for the mesh, then pick again, "
        "or pick a datum marker.").arg(pick.asset_id));
  }
  if (pick.geometry == MatePickGeometry::DatumMarker) {
    const MateDatumFrame * datum = asset.datum(pick.datum_name);
    if (!datum) {
      return fail(QStringLiteral("STALE_DATUM"),
        QStringLiteral("The datum marker %1 is no longer defined on %2; pick again.").arg(pick.datum_name, pick.asset_id));
    }
    r.valid = true;
    r.reason_code = QStringLiteral("DATUM_MARKER");
    r.datum_name = datum->name; r.datum_type = datum->type; r.datum_label = mate_datum_display(*datum); r.datum_source = datum->source;
    r.normal_world = datum->normal;
    r.normal_local = transposed_apply(r_world, datum->normal);
    r.plane_distance_m = 0.; r.normal_angle_deg = 0.;
    r.reason = QStringLiteral("Picked the %1 datum marker of %2.").arg(r.datum_label, pick.asset_id);
    return r;
  }

  MateVec3 normal = pick.normal_world;
  const double length = norm(normal);
  if (!finite(normal) || !(length > 1e-9)) {
    return fail(QStringLiteral("DEGENERATE_NORMAL"),
      QStringLiteral("The clicked face of %1 has no usable surface normal (degenerate triangle). "
        "Click elsewhere on the face or choose a datum manually.").arg(pick.asset_id));
  }
  for (auto & v : normal) v /= length;
  // The visible side of the face: its outward normal faces the viewer.
  if (finite(pick.ray_direction_world) && dot(normal, pick.ray_direction_world) > 0.) for (auto & v : normal) v = -v;
  r.normal_world = normal;
  r.normal_local = transposed_apply(r_world, normal);

  const double max_cos = std::cos(policy.max_normal_angle_deg * kPi / 180.);
  std::vector<MatePickCandidate> accepted;
  for (const auto & datum : asset.datums) {
    if (datum.type != QStringLiteral("plane")) continue;
    MatePickCandidate c;
    c.datum_name = datum.name; c.type = datum.type;
    const double n_len = norm(datum.normal);
    if (!finite(datum.normal) || !finite(datum.origin) || !(n_len > 1e-9)) {
      c.note = QStringLiteral("datum frame is invalid");
      r.candidates.push_back(c); continue;
    }
    const MateVec3 n{datum.normal[0] / n_len, datum.normal[1] / n_len, datum.normal[2] / n_len};
    const double cosine = std::clamp(dot(normal, n), -1., 1.);
    c.normal_angle_deg = std::acos(cosine) * 180. / kPi;
    c.plane_distance_m = std::abs(dot(minus(pick.point_world, datum.origin), n));
    c.accepted = cosine >= max_cos && c.plane_distance_m <= policy.max_plane_distance_m;
    if (!c.accepted) {
      QStringList why;
      if (cosine < max_cos) why << QStringLiteral("the face %1 but the datum %2").arg(mate_direction_text(normal), mate_direction_text(n));
      if (c.plane_distance_m > policy.max_plane_distance_m) {
        why << QStringLiteral("the face is %1 from the datum plane").arg(millimetres(c.plane_distance_m));
      }
      c.note = why.join(QStringLiteral("; "));
    }
    r.candidates.push_back(c);
    if (c.accepted) accepted.push_back(c);
  }
  if (r.candidates.empty()) {
    return fail(QStringLiteral("NO_PLANE_DATUMS"),
      QStringLiteral("%1 has no verified mounting surfaces (no plane datums). Pick one of its datum markers or "
        "choose a datum manually.").arg(pick.asset_id));
  }
  if (accepted.empty()) {
    QStringList listed;
    for (const auto & c : r.candidates) listed << QStringLiteral("%1 (%2)").arg(c.datum_name, c.note);
    return fail(QStringLiteral("NO_MATCHING_DATUM"),
      QStringLiteral("This face of %1 (%2) is not a verified mounting surface. Verified surfaces: %3. "
        "Click a verified surface or its marker, or choose a datum manually.")
        .arg(pick.asset_id, mate_direction_text(normal), listed.join(QStringLiteral("; "))));
  }
  std::sort(accepted.begin(), accepted.end(), [](const MatePickCandidate & a, const MatePickCandidate & b) {
    return a.plane_distance_m != b.plane_distance_m ? a.plane_distance_m < b.plane_distance_m : a.datum_name < b.datum_name;
  });
  if (accepted.size() > 1 && accepted[1].plane_distance_m - accepted[0].plane_distance_m <= policy.ambiguity_margin_m) {
    return fail(QStringLiteral("AMBIGUOUS_DATUM"),
      QStringLiteral("This face of %1 matches both %2 and %3; choose the datum manually.")
        .arg(pick.asset_id, accepted[0].datum_name, accepted[1].datum_name));
  }
  const MateDatumFrame * datum = asset.datum(accepted.front().datum_name);
  r.valid = true;
  r.reason_code = QStringLiteral("MATCHED");
  r.datum_name = datum->name; r.datum_type = datum->type; r.datum_label = mate_datum_display(*datum); r.datum_source = datum->source;
  r.plane_distance_m = accepted.front().plane_distance_m;
  r.normal_angle_deg = accepted.front().normal_angle_deg;
  r.reason = QStringLiteral("Matched %1: the clicked face is %2 from the datum plane and %3° from its normal.")
    .arg(r.datum_label, millimetres(r.plane_distance_m)).arg(r.normal_angle_deg, 0, 'f', 1);
  return r;
}

QJsonObject mate_pick_to_json(const MatePickResolution & r)
{
  QJsonArray candidates;
  for (const auto & c : r.candidates) {
    candidates.append(QJsonObject{{"datum", c.datum_name}, {"type", c.type}, {"plane_distance_m", c.plane_distance_m},
      {"normal_angle_deg", c.normal_angle_deg}, {"accepted", c.accepted}, {"note", c.note}});
  }
  QJsonObject raw{{"backend", r.raw.backend}, {"asset_id", r.raw.asset_id},
    {"geometry", r.raw.geometry == MatePickGeometry::DatumMarker ? "datum_marker" : "surface"},
    {"geometry_source", r.raw.geometry_source}, {"triangle_index", r.raw.triangle_index},
    {"point_world", json3(r.raw.point_world)}, {"normal_world", json3(r.raw.normal_world)},
    {"ray_direction_world", json3(r.raw.ray_direction_world)}};
  if (!r.raw.datum_name.isEmpty()) raw.insert("datum_name", r.raw.datum_name);
  if (r.raw.has_object_pose) {
    raw.insert("object_pose_world", QJsonArray{r.raw.object_pose_world[0], r.raw.object_pose_world[1], r.raw.object_pose_world[2],
      r.raw.object_pose_world[3], r.raw.object_pose_world[4], r.raw.object_pose_world[5]});
  }
  return {{"valid", r.valid}, {"reason_code", r.reason_code}, {"reason", r.reason}, {"asset_id", r.asset_id},
    {"datum", r.datum_name}, {"datum_type", r.datum_type}, {"datum_label", r.datum_label}, {"datum_source", r.datum_source},
    {"plane_distance_m", r.plane_distance_m}, {"normal_angle_deg", r.normal_angle_deg},
    {"point_world", json3(r.point_world)}, {"normal_world", json3(r.normal_world)},
    {"point_local", json3(r.point_local)}, {"normal_local", json3(r.normal_local)},
    {"candidates", candidates}, {"raw", raw}};
}

std::array<double, 6> seated_mate_offset(const std::array<double, 6> & measured)
{
  return {measured[0], measured[1], 0., 0., 0., measured[5]};
}

}  // namespace workcell_builder
