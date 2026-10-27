#pragma once

// Align & Arrange: place one part relative to a reference part (centre, align
// edges, place on top, exact offsets). The controller owns the operation state
// only. Bounds and the target pose come from scripts/asset_alignment.py through
// the asset mounting resolver, and Apply sends the planned edit through the same
// commit path as Mount To and Mate (undo/redo, dirty state, viewport sync).
//
// Align once changes the pose only. "Keep relative" also mounts the part on the
// reference (the existing fixed Mount relation), so it follows the reference.
// Every result is a bounding-box alignment, never a CAD face mate.

#include <QJsonArray>
#include <QJsonObject>
#include <QObject>
#include <QString>
#include <QStringList>

#include <array>
#include <functional>

namespace workcell_builder
{

class AlignArrangeController : public QObject
{
  Q_OBJECT

public:
  enum class Axis { X = 0, Y = 1, Z = 2 };

  struct Hooks
  {
    // Resolver request against the current editor state ({"ok", "result"|"error"}).
    std::function<QJsonObject(const QJsonObject &)> run_resolver;
    // The existing commit path (undo/redo, dirty state, viewport sync).
    std::function<bool(const QJsonObject &)> commit;
    // Current authored rows (id, pose_xyz, pose_rpy, ...) for movement reporting.
    std::function<QJsonArray()> current_items;
    std::function<QString(const QString &)> display_name;
    // Why a part cannot be moved (empty: it can).
    std::function<QString(const QString &)> moving_blocker;
    // Why a part cannot be the reference (empty: it can). A locked reference
    // reports its world pose ({"xyz", "rpy"}) because it is not an editable row.
    std::function<QString(const QString &, QJsonObject *)> reference_blocker;
    // Live box sizes ({id: [x, y, z]}), so unsaved size edits are not ignored.
    std::function<QJsonObject()> live_dimensions;
  };

  struct Preview
  {
    bool ok{false};
    QString error;
    QJsonObject request;  // exactly what Apply sends
    QJsonObject report;   // resolver report: bounds, sources, deltas, notes
    QJsonArray moved;     // [{id, display_name, xyz, rpy, moved_m}]
    QString summary;      // moving part displacement, numerically
    QStringList followers;
  };

  explicit AlignArrangeController(Hooks hooks, QObject * parent = nullptr);

  bool begin(const QString & moving, QString * error);
  void cancel(const QString & why = QString());
  bool active() const { return active_; }

  // A part chosen in the hierarchy, the viewport or the list.
  void set_reference(const QString & id);
  // Sets the rule of one axis; the same rule again clears it.
  // X: centre | min (left) | max (right); Y: centre | min (front) | max (back);
  // Z: centre | min (bottom) | max (top) | on_top.
  void set_rule(Axis axis, const QString & rule);
  void centre_on_reference();  // X and Y centred
  void clear_rules();
  void set_offset_mm(Axis axis, double millimetres);
  void set_keep_relative(bool keep);
  bool apply(QString * error);
  // The authored scene changed outside the tool (undo, load, ...): replan or end.
  void scene_changed();

  const QString & moving() const { return moving_; }
  const QString & reference() const { return reference_; }
  QString rule(Axis axis) const { return rules_[static_cast<int>(axis)]; }
  double offset_mm(Axis axis) const { return offset_mm_[static_cast<int>(axis)]; }
  bool keep_relative() const { return keep_relative_; }
  bool has_rules() const;
  // Why "keep relative" is unavailable (empty: available).
  QString keep_relative_blocker() const;
  const Preview & preview() const { return preview_; }
  QString status() const { return status_; }
  QString display_name(const QString & id) const;
  static QString rule_label(Axis axis, const QString & rule);
  QJsonObject overlay() const;   // ghost and highlights for the viewport
  QJsonObject evidence() const;  // machine-readable state for acceptance runs

signals:
  void changed();
  void finished(bool applied, const QString & message);

private:
  void replan();
  QString operation_text() const;

  Hooks hooks_;
  bool active_{false};
  bool applying_{false};
  QString moving_;
  QString reference_;
  QJsonObject reference_world_;
  std::array<QString, 3> rules_;
  std::array<double, 3> offset_mm_{{0., 0., 0.}};
  bool keep_relative_{false};
  Preview preview_;
  QString status_;
};

}  // namespace workcell_builder
