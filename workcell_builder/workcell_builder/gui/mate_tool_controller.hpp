#pragma once

// Visual Mate tool (R3.2): CAD-style "pick a face on the moving part, pick a
// face on the fixed part, preview, apply". The controller owns the operation
// state only. Picks are resolved by the shared semantic-pick policy
// (mate_pick.hpp); datum frames, previews and Apply all go through
// scripts/asset_mounting.py, the same resolver the Mount To dialog, Save and
// Generate use. Nothing is written to the scene until Apply.

#include <QJsonArray>
#include <QJsonObject>
#include <QObject>
#include <QString>

#include <array>
#include <functional>
#include <optional>

#include "mate_pick.hpp"

namespace workcell_builder
{

class MateToolController : public QObject
{
  Q_OBJECT

public:
  enum class Step { Inactive, PickChild, PickParent, Ready };
  enum class Role { Child, Parent };

  struct Hooks
  {
    // Resolver request against the current editor state ({"ok", "result"|"error"}).
    std::function<QJsonObject(const QJsonObject &)> run_resolver;
    // The existing commit path (undo/redo, dirty state, viewport sync).
    std::function<bool(const QJsonObject &)> commit;
    // Current authored rows (id, pose_xyz, pose_rpy, ...) for movement reporting.
    std::function<QJsonArray()> current_items;
    std::function<QString(const QString &)> display_name;
    // Destination zones bound to a physical target follow it.
    std::function<QStringList(const QString &)> zones_following;
  };

  struct Pick
  {
    QString asset_id;
    QString datum;
    QString datum_type;
    QString datum_label;
    QString how;  // "clicked face", "clicked datum marker", "chosen manually"
    MatePickResolution resolution;  // surface/marker picks only
  };

  struct Preview
  {
    bool ok{false};
    QString error;
    QJsonObject request;           // exactly what Apply sends
    QJsonArray rows;               // resolver result rows
    QJsonArray moved;              // [{id, display_name, xyz, rpy, moved_m, turned_rad}]
    QString child_summary;
    QStringList followers;
    std::array<double, 6> offset{{0., 0., 0., 0., 0., 0.}};
  };

  explicit MateToolController(Hooks hooks, QObject * parent = nullptr);

  // Starts a mate. A selected asset becomes the child candidate; its face is
  // still picked explicitly. Returns false with a reason when no frames exist.
  bool begin(const QString & selected_asset, QString * error);
  void cancel(const QString & why = QString());
  bool active() const { return step_ != Step::Inactive; }
  Step step() const { return step_; }

  // A viewport click. Returns the resolution that was applied (or rejected).
  MatePickResolution pick(const MateRawPick & raw);
  // A viewport hover: resolved with the same policy, never changes state.
  MatePickResolution hover(const MateRawPick & raw) const;
  // An asset chosen in the hierarchy (or by an asset-level viewport click): it
  // becomes the part for the current step; its face or datum is still chosen.
  void hint_asset(const QString & asset_id);
  // The manual datum selector (also the fallback when a face is not a datum).
  bool choose_datum(Role role, const QString & asset_id, const QString & datum, QString * error);
  void replace(Role role);
  // Access to hidden faces: show only the current part, and look at one of its
  // datums from outside along the datum normal (view only; nothing changes).
  void set_isolate(bool isolate);
  bool isolate() const { return isolate_; }
  bool view_datum(const QString & datum, QString * error);
  QString focus_asset() const;
  void set_alignment(const QString & alignment);
  void set_offset(const std::array<double, 6> & offset);
  void reset_offset();
  bool apply(QString * error);
  // The authored scene changed outside the tool (undo, load, ...): refresh
  // frames, keep picks that still exist, drop the rest with a reason.
  void scene_changed();

  const std::optional<Pick> & child() const { return child_; }
  const std::optional<Pick> & parent() const { return parent_; }
  const QString & alignment() const { return alignment_; }
  bool flush_allowed(QString * reason) const;
  const Preview & preview() const { return preview_; }
  const MateFrameIndex & frames() const { return frames_; }
  QString status() const { return status_; }
  // How faces can be picked in the active viewport (empty: visual picking available).
  void set_backend_note(const QString & note) { backend_note_ = note; emit changed(); }
  const QString & backend_note() const { return backend_note_; }
  QString pending_asset() const { return pending_asset_; }
  QJsonObject overlay() const;   // datum markers, highlights and ghost rows for the viewports
  QJsonObject evidence() const;  // machine-readable state for acceptance runs
  QString display_name(const QString & id) const;
  // Beginner-facing text for a pick (the technical reason stays in the evidence).
  QString friendly_reason(const MatePickResolution & resolution) const;
  static QString short_reason(const MatePickResolution & resolution);

signals:
  void changed();
  void finished(bool applied, const QString & message);

private:
  bool refresh_frames(QString * error);
  void accept_pick(Role role, const Pick & pick);
  void update_preview(bool reseat);
  QString readable(QString error) const;
  void set_status(const QString & status);

  Hooks hooks_;
  Step step_{Step::Inactive};
  MateFrameIndex frames_;
  std::optional<Pick> child_;
  std::optional<Pick> parent_;
  QString pending_asset_;      // asset of a rejected face pick, for the manual selector
  QString alignment_{QStringLiteral("flush")};
  bool alignment_chosen_{false};
  std::array<double, 6> offset_{{0., 0., 0., 0., 0., 0.}};
  Preview preview_;
  QString status_;
  QJsonArray picks_log_;
  QString backend_note_;
  bool isolate_{false};
  int view_seq_{0};
  QJsonObject view_;
  bool applying_{false};
};

}  // namespace workcell_builder
