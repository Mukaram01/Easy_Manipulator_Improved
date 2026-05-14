#include "workcell_studio_layout_merge.hpp"

#include <QProcess>
#include <QDir>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>

namespace fs = boost::filesystem;
namespace workcell_builder {
LayoutMergeResult merge_workcell_studio_layout(const fs::path& scene_dir)
{
  LayoutMergeResult out;
  const fs::path layout_file = scene_dir / "layout" / "workcell_studio_layout.yaml";
  out.layout_applied = fs::exists(layout_file);
  if (!out.layout_applied) {
    out.warnings.push_back("Run Generate Scene to apply layout");
    return out;
  }

  const fs::path repo_root = scene_dir.parent_path().parent_path();
  const fs::path script_path = repo_root / "scripts" / "workcell_studio_layout_merge.py";
  if (!fs::exists(script_path)) {
    out.blockers.push_back("Layout merge script missing: scripts/workcell_studio_layout_merge.py");
    return out;
  }

  QProcess proc;
  proc.setProgram("python3");
  proc.setArguments({QString::fromStdString(script_path.string()), QString::fromStdString(scene_dir.string())});
  proc.setWorkingDirectory(QString::fromStdString(repo_root.string()));
  proc.start();
  proc.waitForFinished(-1);
  out.stdout_log = QString::fromUtf8(proc.readAllStandardOutput()).toStdString();
  out.stderr_log = QString::fromUtf8(proc.readAllStandardError()).toStdString();

  const fs::path report = scene_dir / "generated" / "workcell_studio_layout_merge_report.json";
  const fs::path summary = scene_dir / "generated" / "workcell_studio_layout_merge_summary.txt";
  out.report_path = report.string();
  out.summary_path = summary.string();
  out.status = (proc.exitStatus() == QProcess::NormalExit && proc.exitCode() == 0 && fs::exists(report));

  if (fs::exists(report)) {
    QFile f(QString::fromStdString(report.string()));
    if (f.open(QIODevice::ReadOnly | QIODevice::Text)) {
      const auto doc = QJsonDocument::fromJson(f.readAll());
      const auto obj = doc.object();
      out.layout_applied = obj.value("layout_applied").toBool(out.layout_applied);
      out.generated_from_saved_layout = obj.value("generated_from_saved_layout").toBool(false);
      for (const auto & w : obj.value("warnings").toArray()) out.warnings.push_back(w.toString().toStdString());
      for (const auto & b : obj.value("blockers").toArray()) out.blockers.push_back(b.toString().toStdString());
      for (const auto & a : obj.value("generated_artifacts").toArray()) out.generated_artifacts.push_back(a.toString().toStdString());
    }
  }
  if (!out.status && out.blockers.empty()) out.blockers.push_back("Layout merge failed");
  return out;
}
}
