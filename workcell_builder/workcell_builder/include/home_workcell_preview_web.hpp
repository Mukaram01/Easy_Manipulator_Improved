#pragma once

// Read-only visual preview for the Home inspector.
//
// Preferred evidence order:
//   1. existing PNG scene evidence (smoke/acceptance/Product View snapshot)
//   2. existing static_preview.svg/html
//   3. generate static_preview.svg/html from the scene's cell_definition.yaml
//   4. render a harmless metadata schematic so Home never shows a blank/error box
//
// The WebEngine view is read-only and local-file backed. It does not create a
// second authoring surface and it never launches ROS, MoveIt, or real hardware.

#include <QApplication>
#include <QBoxLayout>
#include <QColor>
#include <QDir>
#include <QFileInfo>
#include <QLabel>
#include <QLayout>
#include <QMainWindow>
#include <QPixmap>
#include <QPointer>
#include <QProcess>
#include <QSizePolicy>
#include <QTableWidget>
#include <QUrl>

#ifdef WORKCELL_BUILDER_HAS_WEBENGINE
#include <QWebEnginePage>
#include <QWebEngineSettings>
#include <QWebEngineView>
#endif

#include "home_workcells_target_shell.hpp"

namespace workcell_builder
{
namespace home_workcells
{

inline QString repo_root_for_home_preview(const QString & workspace_root)
{
  const QString candidate = QDir(workspace_root).filePath(QStringLiteral("src/easy_manipulation_deployment"));
  return QFileInfo(candidate).isDir() ? QDir::cleanPath(candidate) : QString();
}

inline QString generated_static_preview_document(const QString & workspace_root, const QString & scene_id)
{
  const QString scene_dir = scene_dir_for_id(workspace_root, scene_id);
  if (scene_dir.isEmpty()) return QString();

  const QString svg = QDir(scene_dir).filePath(QStringLiteral("preview/static_preview.svg"));
  const QString html = QDir(scene_dir).filePath(QStringLiteral("preview/static_preview.html"));
  if (QFileInfo(svg).isFile()) return svg;
  if (QFileInfo(html).isFile()) return html;

  const QString repo_root = repo_root_for_home_preview(workspace_root);
  const QString script = QDir(repo_root).filePath(QStringLiteral("scripts/generate_workcell_static_preview.py"));
  const QString cell_definition = QDir(scene_dir).filePath(QStringLiteral("cell_definition.yaml"));
  if (!QFileInfo(script).isFile() || !QFileInfo(cell_definition).isFile()) return QString();

  const QString output_dir = QDir(scene_dir).filePath(QStringLiteral("preview"));
  QDir().mkpath(output_dir);

  QStringList arguments{
    script,
    QStringLiteral("--cell-definition"), cell_definition,
    QStringLiteral("--output-dir"), output_dir,
    QStringLiteral("--title"), friendly_workcell_name(scene_id)};

  const QString environment_layout = QDir(scene_dir).filePath(QStringLiteral("environment_layout.yaml"));
  if (QFileInfo(environment_layout).isFile()) {
    arguments << QStringLiteral("--environment-layout") << environment_layout;
  }
  const QString task_intent = QDir(scene_dir).filePath(QStringLiteral("workcell_builder_task_intent.yaml"));
  if (QFileInfo(task_intent).isFile()) {
    arguments << QStringLiteral("--task-intent") << task_intent;
  }
  const QString task_recipe = QDir(scene_dir).filePath(QStringLiteral("task_recipe.yaml"));
  if (QFileInfo(task_recipe).isFile()) {
    arguments << QStringLiteral("--task-recipe") << task_recipe;
  }

  QProcess process;
  process.setWorkingDirectory(repo_root);
  process.start(QStringLiteral("python3"), arguments);
  if (!process.waitForStarted(1500)) return QString();
  if (!process.waitForFinished(6000)) {
    process.kill();
    process.waitForFinished(500);
    return QString();
  }
  if (process.exitStatus() != QProcess::NormalExit || process.exitCode() != 0) return QString();

  if (QFileInfo(svg).isFile()) return svg;
  if (QFileInfo(html).isFile()) return html;
  return QString();
}

inline QString best_home_preview_document(const QString & workspace_root, const QString & scene_id)
{
  // Real scene evidence wins over the generated schematic.
  const QString image = find_preview_path(workspace_root, scene_id);
  if (!image.isEmpty() && QFileInfo(image).isFile()) return image;
  return generated_static_preview_document(workspace_root, scene_id);
}

inline QString local_visual_html(const QString & file_path)
{
  const QString url = QUrl::fromLocalFile(file_path).toString(QUrl::FullyEncoded);
  return QStringLiteral(
    "<!doctype html><html><head><meta charset='utf-8'>"
    "<style>html,body{margin:0;width:100%%;height:100%%;overflow:hidden;background:#111b25;}"
    "body{display:flex;align-items:center;justify-content:center;}"
    "img{display:block;max-width:100%%;max-height:100%%;width:auto;height:auto;object-fit:contain;}"
    "</style></head><body><img src='%1'></body></html>").arg(url);
}

inline QString metadata_schematic_html(
  const QString & scene_id, const QString & robot, const QString & tool)
{
  const QString title = friendly_workcell_name(scene_id).toHtmlEscaped();
  const QString robot_text = robot.toHtmlEscaped();
  const QString tool_text = tool.toHtmlEscaped();
  return QStringLiteral(
    "<!doctype html><html><head><meta charset='utf-8'>"
    "<style>html,body{margin:0;width:100%%;height:100%%;overflow:hidden;background:#111b25;color:#dbe8f4;font-family:sans-serif;}"
    ".scene{position:relative;width:100%%;height:100%%;background:radial-gradient(circle at 50%% 30%%,#243444,#111b25 65%%);}"
    ".grid{position:absolute;left:8%%;right:8%%;bottom:12%%;height:40%%;transform:skewX(-12deg);border:1px solid #4f6273;background:repeating-linear-gradient(0deg,transparent,transparent 23px,#263847 24px),repeating-linear-gradient(90deg,transparent,transparent 31px,#263847 32px);}"
    ".robot{position:absolute;left:31%%;bottom:29%%;width:38px;height:92px;border-radius:18px;background:#a9b6c2;box-shadow:0 0 0 6px #576878 inset;}"
    ".arm{position:absolute;left:34%%;bottom:58%%;width:92px;height:18px;border-radius:10px;background:#99a9b8;transform:rotate(18deg);transform-origin:left center;}"
    ".bin{position:absolute;right:23%%;bottom:26%%;width:64px;height:45px;border:3px solid #235df1;background:#0b2f9a;border-radius:4px;transform:skewY(-7deg);}"
    ".caption{position:absolute;left:14px;top:12px;font-size:12px;font-weight:700;color:#eef6ff;}"
    ".meta{position:absolute;left:14px;bottom:10px;font-size:10px;color:#9fb2c5;}"
    "</style></head><body><div class='scene'><div class='caption'>%1</div><div class='grid'></div><div class='robot'></div><div class='arm'></div><div class='bin'></div><div class='meta'>%2 · %3 · read-only scene preview</div></div></body></html>")
    .arg(title, robot_text, tool_text);
}

#ifdef WORKCELL_BUILDER_HAS_WEBENGINE
inline void refresh_home_web_preview(
  QMainWindow * window, QTableWidget * table, const QString & workspace_root)
{
  auto * web = window ? window->findChild<QWebEngineView *>(QStringLiteral("studioTargetWebPreview")) : nullptr;
  if (!web || !table) return;

  const int row = table->currentRow();
  const bool selected = row >= 0 && row < table->rowCount() && table->item(row, 0);
  if (!selected) {
    web->setHtml(metadata_schematic_html(QStringLiteral("workcell"), QStringLiteral("Robot"), QStringLiteral("Tool")));
    return;
  }

  const QString scene_id = scene_id_at(table, row);
  const QString robot = table->item(row, 2) ? clean_robot(table->item(row, 2)->text()) : QStringLiteral("Robot");
  const QString tool = table->item(row, 3) ? clean_tool(table->item(row, 3)->text()) : QStringLiteral("Tool");
  const QString document = best_home_preview_document(workspace_root, scene_id);

  if (!document.isEmpty()) {
    const QString suffix = QFileInfo(document).suffix().toLower();
    if (suffix == QStringLiteral("png") || suffix == QStringLiteral("jpg") ||
        suffix == QStringLiteral("jpeg") || suffix == QStringLiteral("webp") ||
        suffix == QStringLiteral("svg")) {
      web->setZoomFactor(1.0);
      web->setHtml(local_visual_html(document), QUrl::fromLocalFile(QFileInfo(document).absolutePath() + QDir::separator()));
      web->setToolTip(document);
      return;
    }
    if (suffix == QStringLiteral("html") || suffix == QStringLiteral("htm")) {
      web->setZoomFactor(0.42);
      web->setUrl(QUrl::fromLocalFile(document));
      web->setToolTip(document);
      return;
    }
  }

  // Last-resort visual is intentionally graphical rather than an error panel.
  web->setZoomFactor(1.0);
  web->setHtml(metadata_schematic_html(scene_id, robot, tool));
  web->setToolTip(QStringLiteral("Generated read-only metadata schematic"));
}
#endif

inline void install_home_web_preview(
  QMainWindow * window, const QString & workspace_root)
{
  if (!window || QApplication::arguments().contains(QStringLiteral("--scene3d-smoke"))) return;
  if (window->property("studioHomeWebPreviewInstalled").toBool()) return;

  QTableWidget * table = scene_table(window);
  QLabel * legacy_preview = window->findChild<QLabel *>(QStringLiteral("studioTargetPreview"));
  if (!table || !legacy_preview || !legacy_preview->parentWidget() || !legacy_preview->parentWidget()->layout()) return;
  window->setProperty("studioHomeWebPreviewInstalled", true);

#ifdef WORKCELL_BUILDER_HAS_WEBENGINE
  auto * web = new QWebEngineView(legacy_preview->parentWidget());
  web->setObjectName(QStringLiteral("studioTargetWebPreview"));
  web->setMinimumHeight(220);
  web->setMaximumHeight(250);
  web->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  web->setContextMenuPolicy(Qt::NoContextMenu);
  web->setAcceptDrops(false);
  web->settings()->setAttribute(QWebEngineSettings::JavascriptEnabled, false);
  web->settings()->setAttribute(QWebEngineSettings::LocalContentCanAccessFileUrls, true);
  web->settings()->setAttribute(QWebEngineSettings::LocalContentCanAccessRemoteUrls, false);
  web->page()->setBackgroundColor(QColor(QStringLiteral("#111B25")));

  QLayout * layout = legacy_preview->parentWidget()->layout();
  const int index = layout->indexOf(legacy_preview);
  if (auto * box = qobject_cast<QBoxLayout *>(layout)) box->insertWidget(qMax(0, index), web);
  else layout->addWidget(web);
  legacy_preview->hide();

  const QPointer<QMainWindow> safe_window(window);
  QObject::connect(table, &QTableWidget::cellClicked, web,
    [safe_window, table, workspace_root](int, int) {
      if (safe_window) refresh_home_web_preview(safe_window, table, workspace_root);
    });
  QObject::connect(table, &QTableWidget::itemSelectionChanged, web,
    [safe_window, table, workspace_root]() {
      if (safe_window) refresh_home_web_preview(safe_window, table, workspace_root);
    });

  refresh_home_web_preview(window, table, workspace_root);
#else
  // Non-WebEngine builds retain the existing QLabel, but never show an error.
  legacy_preview->setText(QStringLiteral("Scene preview available in Product View"));
  legacy_preview->setToolTip(QStringLiteral("This build does not include Qt WebEngine."));
#endif
}

}  // namespace home_workcells
}  // namespace workcell_builder
