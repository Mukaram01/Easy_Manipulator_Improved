#pragma once

#include <QString>
#include <QStringList>

namespace workcell_builder
{
// Decides what Save Layout does with one browser (Product View) transform edit.
// Only a positively identified derived record may be skipped; anything the
// authored layout cannot account for fails closed so no intended edit is lost.
enum class WebEditDisposition { Persist, SkipDerived, Reject };

struct WebEditFacts
{
  QString item_id;
  bool canvas_found{false};        // find_canvas_item_by_stable_id() != nullptr
  bool locked{false};
  QString source_layer;            // RoleSourceLayer of the canvas item
  bool deleted{false};             // id in deleted_layout_item_ids_
  bool known_derived{false};       // id is a scene-declared derived record (task destination zone)
};

inline WebEditDisposition classify_web_edit(const WebEditFacts & facts, QString * reason = nullptr)
{
  const auto say = [reason](const QString & text) { if (reason) *reason = text; };
  if (facts.item_id.trimmed().isEmpty()) {
    say(QStringLiteral("Browser edit has no item ID. No file was written; the unified authored session remains dirty."));
    return WebEditDisposition::Reject;
  }
  if (!facts.canvas_found) {
    if (facts.known_derived) {
      say(QStringLiteral("Save Layout: ignored browser transform for '%1'; it is the task destination zone that follows its target and has no authored layout entry.")
        .arg(facts.item_id));
      return WebEditDisposition::SkipDerived;
    }
    say(QStringLiteral("Persisted item '%1' is unknown to the authored layout (missing, stale, or renamed). No file was written; reopen the scene and retry.")
      .arg(facts.item_id));
    return WebEditDisposition::Reject;
  }
  if (facts.deleted) {
    say(QStringLiteral("Persisted item '%1' was deleted in this session; its browser edit cannot be saved. No file was written.").arg(facts.item_id));
    return WebEditDisposition::Reject;
  }
  if (facts.locked) {
    say(QStringLiteral("Persisted item '%1' is locked; unlock it before saving a transform. No file was written.").arg(facts.item_id));
    return WebEditDisposition::Reject;
  }
  if (facts.source_layer.trimmed() != QStringLiteral("editable_layout")) {
    say(QStringLiteral("Persisted item '%1' is outside editable_layout (layer '%2'); generated or preview items cannot be saved. No file was written.")
      .arg(facts.item_id, facts.source_layer.trimmed()));
    return WebEditDisposition::Reject;
  }
  return WebEditDisposition::Persist;
}
}  // namespace workcell_builder
