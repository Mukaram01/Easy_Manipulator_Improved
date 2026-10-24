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
}  // namespace workcell_builder
