#include "include/workspace_validation.hpp"

#include <QDir>
#include <QFileInfo>

namespace workcell_builder
{
bool is_valid_workcell_workspace(const QString & path)
{
  const QFileInfo root_info(path);
  if (!root_info.exists() || !root_info.isDir()) {
    return false;
  }

  const QDir root(path);
  if (!root.exists("src")) {
    return false;
  }

  const QDir src(root.filePath("src"));
  return src.exists("easy_manipulation_deployment") || src.exists("scenes") || src.exists("assets");
}
}  // namespace workcell_builder
