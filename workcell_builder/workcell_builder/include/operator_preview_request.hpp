#pragma once

// Builder -> Operator HMI hand-off. The Builder owns the engineering truth; the
// request only tells the HMI which scene to project and whether the editor holds
// unsaved task edits (sent as a read-only draft) or unsaved layout edits (which
// the HMI can only report, because geometry reaches disk on Save).

#include <QJsonObject>
#include <QString>

#include <optional>

namespace workcell_builder
{

struct OperatorPreviewRequest
{
  int revision{0};
  QString scene_dir;
  QString scene_name;
  QString workspace_root;
  QString scripts_dir;          // where the Builder resolved its helper scripts
  bool task_dirty{false};       // the task editor holds unsaved edits
  QJsonObject task_draft;       // canonical v2 draft; empty unless task_dirty
  bool layout_dirty{false};     // unsaved physical-layout edits (not projected)
  bool activate{false};         // user asked for the preview now: refresh and raise
  qint64 builder_pid{0};
  QString requested_at;         // ISO-8601 UTC

  QJsonObject to_json() const;
  static std::optional<OperatorPreviewRequest> from_json(const QJsonObject & json, QString * error);
};

// Atomic write (QSaveFile) so the HMI never reads a half-written request.
bool write_operator_preview_request(
  const QString & path, const OperatorPreviewRequest & request, QString * error);
std::optional<OperatorPreviewRequest> read_operator_preview_request(
  const QString & path, QString * error);

}  // namespace workcell_builder
