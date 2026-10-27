#pragma once

// The Scene Builder's selection: an ordered set of parts with one reference
// (anchor). The first part selected is the reference until the user chooses
// another; removing the reference passes it to the earliest remaining member.
// A single selection is a set of one, so existing single-object behaviour is the
// size()==1 case. Hierarchy, 3D view and Inspector all read this one model.

#include <QSet>
#include <QString>
#include <QStringList>

namespace workcell_builder
{

class SceneSelectionSet
{
public:
  const QStringList & members() const { return members_; }
  const QString & reference() const { return reference_; }
  int size() const { return members_.size(); }
  bool isEmpty() const { return members_.isEmpty(); }
  bool contains(const QString & id) const { return members_.contains(id); }
  // The part the Inspector shows: the reference.
  QString primary() const { return reference_; }

  void clear() { members_.clear(); reference_.clear(); }

  // A plain click: exactly this part.
  void select_only(const QString & id)
  {
    clear();
    if (!id.trimmed().isEmpty()) { members_ << id.trimmed(); reference_ = id.trimmed(); }
  }

  // Ctrl+click: add or remove one part. Returns true when the part is now selected.
  bool toggle(const QString & id)
  {
    const QString part = id.trimmed();
    if (part.isEmpty()) return false;
    if (members_.removeAll(part) > 0) {
      if (reference_ == part) reference_ = members_.isEmpty() ? QString() : members_.first();
      return false;
    }
    members_ << part;
    if (reference_.isEmpty()) reference_ = part;
    return true;
  }

  // Shift+click in the hierarchy: add every part of an ordered range (keeps the reference).
  void extend(const QStringList & range)
  {
    for (const auto & id : range) {
      const QString part = id.trimmed();
      if (part.isEmpty() || members_.contains(part)) continue;
      members_ << part;
      if (reference_.isEmpty()) reference_ = part;
    }
  }

  // Make a selected part the reference. Returns false for a part outside the selection.
  bool set_reference(const QString & id)
  {
    if (!members_.contains(id)) return false;
    reference_ = id;
    return true;
  }

  // Scene reload, delete or cell switch: drop parts that no longer exist.
  // Returns true when the selection changed.
  bool prune(const QSet<QString> & existing)
  {
    const int before = members_.size();
    QStringList kept;
    for (const auto & id : members_) if (existing.contains(id)) kept << id;
    members_ = kept;
    if (!members_.contains(reference_)) reference_ = members_.isEmpty() ? QString() : members_.first();
    return members_.size() != before;
  }

  // Members other than the reference, in selection order.
  QStringList others() const
  {
    QStringList result;
    for (const auto & id : members_) if (id != reference_) result << id;
    return result;
  }

private:
  QStringList members_;
  QString reference_;
};

}  // namespace workcell_builder
