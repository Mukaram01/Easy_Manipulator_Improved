#pragma once

#include <QString>

namespace workcell_builder
{
// The inspector shows the normalised hierarchy role ("place_target_bin") while the
// authored layout keeps the raw role ("target_bin"). Re-applying the displayed
// value is not an edit; only a role that differs from both is one.
inline bool semantic_role_actually_changed(
  const QString & chosen_role, const QString & authored_role, const QString & normalised_authored_role)
{
  const QString chosen = chosen_role.trimmed();
  if (chosen.isEmpty()) return false;
  if (chosen == authored_role.trimmed()) return false;
  if (!normalised_authored_role.trimmed().isEmpty() && chosen == normalised_authored_role.trimmed()) return false;
  return true;
}

// The role a Save writes: the authored role unless the user really chose another
// one. A display alias (or a re-applied alias) never replaces authored semantics.
inline QString semantic_role_to_persist(
  const QString & editor_role, const QString & authored_role, const QString & normalised_authored_role)
{
  if (authored_role.trimmed().isEmpty()) return editor_role.trimmed();
  return semantic_role_actually_changed(editor_role, authored_role, normalised_authored_role)
    ? editor_role.trimmed() : authored_role.trimmed();
}
}  // namespace workcell_builder
