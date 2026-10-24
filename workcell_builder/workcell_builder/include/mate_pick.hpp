#pragma once

// Shared semantic-pick contract for visual mating (R3.2).
//
// Viewport backends (embedded Web3D, native Scene3D) only report what the ray
// actually hit: the asset, the world-space hit point and surface normal, or an
// authored datum marker. This module turns that raw hit into an authored datum
// with one explicit, deterministic policy, so the backends never define mate
// semantics themselves. Datum world frames come from scripts/asset_mounting.py
// (`datum_frames`), which owns the transform math; Apply and preview go through
// the same resolver's edit operation.

#include <QJsonObject>
#include <QMap>
#include <QString>
#include <QStringList>

#include <array>
#include <vector>

namespace workcell_builder
{

using MateVec3 = std::array<double, 3>;

enum class MatePickGeometry
{
  Surface,      // a ray hit on rendered asset geometry
  DatumMarker,  // a click on a drawn datum marker: the datum is named exactly
};

// What a backend reports for one click (or hover), in ROS world coordinates.
struct MateRawPick
{
  QString backend;            // "embedded_web3d" | "native_scene3d" | "test"
  QString asset_id;           // stable scene ID of the hit asset
  MatePickGeometry geometry{MatePickGeometry::Surface};
  QString datum_name;         // DatumMarker only
  MateVec3 point_world{{0., 0., 0.}};
  MateVec3 normal_world{{0., 0., 1.}};  // Surface: geometric face normal (any sign)
  MateVec3 ray_direction_world{{0., 0., -1.}};
  bool has_object_pose{false};          // asset frame as drawn, for staleness checks
  std::array<double, 6> object_pose_world{{0., 0., 0., 0., 0., 0.}};  // xyz, rpy
  QString geometry_source;    // mesh path, "primitive box", "datum marker", ...
  int triangle_index{-1};
};

struct MateDatumFrame
{
  QString name;
  QString type;    // origin | point | axis | plane
  QString label;
  QString source;  // where the datum is defined (catalog review, asset anchor, ...)
  MateVec3 origin{{0., 0., 0.}};
  MateVec3 normal{{0., 0., 1.}};  // local +Z in world: plane normal / axis direction
  MateVec3 x_axis{{1., 0., 0.}};
  MateVec3 rpy{{0., 0., 0.}};
};

struct MateAssetFrames
{
  QString asset_id;
  std::array<double, 6> world{{0., 0., 0., 0., 0., 0.}};  // resolved asset pose, xyz rpy
  std::vector<MateDatumFrame> datums;

  const MateDatumFrame * datum(const QString & name) const;
};

using MateFrameIndex = QMap<QString, MateAssetFrames>;

// Explicit matching policy for "this face is that plane datum".
struct MatePickPolicy
{
  double max_plane_distance_m{0.005};   // hit point to datum plane
  double max_normal_angle_deg{10.0};    // visible face normal to datum outward normal
  double ambiguity_margin_m{0.0005};    // two planes this close in distance are ambiguous
  double stale_position_m{1e-4};        // drawn asset pose vs resolved pose
  double stale_angle_rad{1e-4};
};

struct MatePickCandidate
{
  QString datum_name;
  QString type;
  double plane_distance_m{0.};
  double normal_angle_deg{0.};
  bool accepted{false};
  QString note;
};

struct MatePickResolution
{
  bool valid{false};
  QString asset_id;
  QString datum_name;
  QString datum_type;
  QString datum_label;
  QString datum_source;
  QString reason;              // beginner-facing; why the pick is valid or not
  QString reason_code;         // machine-readable (MATCHED, NO_MATCHING_DATUM, ...)
  double plane_distance_m{-1.};
  double normal_angle_deg{-1.};
  MateVec3 point_world{{0., 0., 0.}};
  MateVec3 normal_world{{0., 0., 0.}};   // oriented towards the viewer
  MateVec3 point_local{{0., 0., 0.}};    // in the asset frame (display only)
  MateVec3 normal_local{{0., 0., 0.}};
  std::vector<MatePickCandidate> candidates;
  MateRawPick raw;
};

// Parses the resolver's `datum_frames` result: {asset_id: {world, datums}}.
MateFrameIndex parse_mate_frames(const QJsonObject & result);

// The single matching policy used by every backend, for clicks and hover.
MatePickResolution resolve_mate_pick(const MateRawPick & pick, const MateFrameIndex & frames,
  const MatePickPolicy & policy = {});

QJsonObject mate_pick_to_json(const MatePickResolution & resolution);

// "points up", "points down", "points +X" ... for a world direction.
QString mate_direction_text(const MateVec3 & direction);

// Short beginner-facing name of a datum: "Bottom face (normal points down)" or
// "surface · plane (points up)" when the datum has no label.
QString mate_datum_display(const MateDatumFrame & datum);

// Offset proposal for a new mate. The resolver measures the child's current
// offset in the parent datum frame (an edit without relative_pose keeps the
// world pose); the proposal keeps the in-plane position and the rotation about
// the normal and removes the gap and tilt, so the datums seat exactly.
std::array<double, 6> seated_mate_offset(const std::array<double, 6> & measured);

// The embedded viewer numbers its Mate clicks, hovers and Esc presses, and
// Builder acts only on numbers it has not seen. The viewer's counts become the
// new baseline when its Mate is inactive (it records nothing then) or when a
// count went backwards (a reloaded viewer counts from zero again); otherwise
// clicks after a reload would be ignored without any message.
struct MateViewerCounters
{
  qint64 pick = 0;
  qint64 hover = 0;
  qint64 cancel = 0;
};
bool mate_viewer_counters_rebase(const MateViewerCounters & seen, const MateViewerCounters & viewer, bool viewer_active);

}  // namespace workcell_builder
