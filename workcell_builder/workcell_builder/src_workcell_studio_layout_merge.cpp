#include "workcell_studio_layout_merge.hpp"

#include <QProcess>
#include <QDir>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <fstream>

namespace fs = boost::filesystem;
namespace workcell_builder {

namespace {

void write_layout_merge_fallback_report(
  const fs::path& scene_dir,
  const std::vector<std::string>& warnings,
  const std::vector<std::string>& blockers,
  bool layout_applied)
{
  boost::system::error_code ec;
  fs::create_directories(scene_dir / "generated", ec);
  const fs::path report = scene_dir / "generated" / "workcell_studio_layout_merge_report.json";
  const fs::path summary = scene_dir / "generated" / "workcell_studio_layout_merge_summary.txt";

  QJsonObject root;
  root.insert("layout_applied", layout_applied);
  root.insert("generated_from_saved_layout", layout_applied);
  QJsonArray warning_json;
  for (const auto& w : warnings) warning_json.push_back(QString::fromStdString(w));
  root.insert("warnings", warning_json);
  QJsonArray blocker_json;
  for (const auto& b : blockers) blocker_json.push_back(QString::fromStdString(b));
  root.insert("blockers", blocker_json);
  root.insert("merge_mode", "fallback");
  root.insert("layout_merge_fallback_used", true);
  root.insert("layout_merge_status", blockers.empty() ? "WARN" : "BLOCKED");
  root.insert("layout_merge_warning", "missing script fallback used");

  QFile report_file(QString::fromStdString(report.string()));
  if (report_file.open(QIODevice::WriteOnly | QIODevice::Text)) {
    report_file.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    report_file.write("\n");
  }

  std::ofstream summary_out(summary.string());
  if (summary_out.is_open()) {
    summary_out << "layout_applied=" << (layout_applied ? "true" : "false") << "\n";
    summary_out << "generated_from_saved_layout=" << (layout_applied ? "true" : "false") << "\n";
    summary_out << "layout_merge_fallback_used=true\n";
    for (const auto& w : warnings) summary_out << "warning=" << w << "\n";
    for (const auto& b : blockers) summary_out << "blocker=" << b << "\n";
  }
}

}  // namespace

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
    const std::vector<std::pair<fs::path, std::string>> required_authoring_files{
      {scene_dir / "environment.yaml", "Missing required authoring file: environment.yaml. Next action: click Save Layout."},
      {scene_dir / "layout" / "workcell_studio_layout.yaml", "Missing required authoring file: layout/workcell_studio_layout.yaml. Next action: click Save Layout."},
      {scene_dir / "cell_definition.yaml", "Missing required authoring file: cell_definition.yaml. Next action: click Generate YAML."}
    };
    for (const auto& required_file : required_authoring_files) {
      if (!fs::exists(required_file.first)) {
        out.blockers.push_back(required_file.second);
      }
    }
    out.warnings.push_back("missing script fallback used");
    write_layout_merge_fallback_report(scene_dir, out.warnings, out.blockers, out.layout_applied);
    out.report_path = (scene_dir / "generated" / "workcell_studio_layout_merge_report.json").string();
    out.summary_path = (scene_dir / "generated" / "workcell_studio_layout_merge_summary.txt").string();
    out.status = out.blockers.empty();
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
