#pragma once

// Multi-object arrangement for the Scene Builder toolbar: align, distribute,
// precise move/rotate, grid snap, fixed mount and detach for the current
// selection and its reference. The controller owns one pending operation only.
// scripts/asset_arrangement.py (through the asset mounting resolver) plans it;
// Apply commits the very same request as ONE undoable edit through the Mount To
// commit path, after checking that the scene has not changed since the preview.

#include <QJsonArray>
#include <QJsonObject>
#include <QObject>
#include <QString>
#include <QStringList>

#include <functional>

namespace workcell_builder
{

class ArrangeController : public QObject
{
  Q_OBJECT

public:
  struct Selection
  {
    QStringList members;  // selection order; includes the reference
    QString reference;
  };

  struct Hooks
  {
    std::function<QJsonObject(const QJsonObject &)> run_resolver;
    std::function<bool(const QJsonObject &)> commit;
    std::function<QJsonArray()> current_items;
    std::function<QString(const QString &)> display_name;
    std::function<Selection()> selection;
    // Why a selected part cannot move (empty: it can).
    std::function<QString(const QString &)> moving_blocker;
    // Why a part cannot be the reference (generated robot/tool visuals, task zones).
    std::function<QString(const QString &)> reference_blocker;
    // World pose ({"xyz","rpy"}) of a locked reference (empty when it is an editable row).
    std::function<QJsonObject(const QString &)> reference_world;
  };

  struct Preview
  {
    bool ok{false};
    QString error;
    QJsonObject request;  // exactly what Apply commits
    QJsonObject report;
    QJsonArray moved;     // [{id, display_name, distance_m, to, follower}]
    QJsonArray ghost;     // [{id, xyz, rpy}] for the viewport
    QString summary;
  };

  explicit ArrangeController(Hooks hooks, QObject * parent = nullptr);

  // Why an operation is unavailable for the current selection (empty: available).
  // op: align | distribute | move | snap_grid | mount | detach; params as for start().
  QString availability(const QString & op, const QJsonObject & params = {}) const;

  // Plans an operation; the preview stays open until apply() or cancel().
  bool start(const QString & op, const QJsonObject & params, const QString & label, QString * error);
  void set_params(const QJsonObject & params);
  void cancel(const QString & why = QString());
  bool apply(QString * error);
  // The scene or the selection changed: replan, or close when no longer possible.
  void scene_changed();

  bool active() const { return active_; }
  const QString & op() const { return op_; }
  const QString & label() const { return label_; }
  const QJsonObject & params() const { return params_; }
  const Preview & preview() const { return preview_; }
  const QString & status() const { return status_; }
  QJsonObject overlay() const;
  QJsonObject evidence() const;
  QString display_name(const QString & id) const;

signals:
  void changed();
  void finished(bool applied, const QString & message);

private:
  QJsonObject request_for(const QString & op, const QJsonObject & params, const Selection & selection) const;
  QString items_signature() const;
  void replan();

  Hooks hooks_;
  bool active_{false};
  bool applying_{false};
  QString op_;
  QString label_;
  QJsonObject params_;
  Selection planned_selection_;
  QString planned_signature_;
  Preview preview_;
  QString status_;
};

}  // namespace workcell_builder
