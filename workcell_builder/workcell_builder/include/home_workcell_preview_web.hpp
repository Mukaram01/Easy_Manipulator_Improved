#pragma once

// Fast read-only Home preview.
//
// Home is a library/selection surface, not a second Product View.  It therefore
// never creates another QWebEngineView and never reloads the Three.js scene.
// Instead it shows a persistent raster snapshot immediately.  When the canonical
// ScenePreviewWidget/Product View finishes rendering the selected scene, Home
// captures the existing #scene-canvas once and refreshes the cache.
//
// Cache files live under QStandardPaths::CacheLocation, never under scenes/.
// They are keyed by scene id + current scene modification stamp so edits naturally
// invalidate the exact cache entry.  The most recent older Product View snapshot
// may be shown while the canonical renderer refreshes, which keeps Home instant
// without creating a second renderer/server/scene export path.

#include <QApplication>
#include <QByteArray>
#include <QColor>
#include <QDateTime>
#include <QDir>
#include <QFileInfo>
#include <QImage>
#include <QLabel>
#include <QMainWindow>
#include <QPixmap>
#include <QPointer>
#include <QSize>
#include <QStandardPaths>
#include <QTableWidget>
#include <QVariant>
#include <QtGlobal>

#ifdef WORKCELL_BUILDER_HAS_WEBENGINE
#include <QWebEnginePage>
#include <QWebEngineView>
#endif

#include "home_workcells_target_shell.hpp"
#include "scene_preview_widget.h"

namespace workcell_builder
{
namespace home_workcells
{

inline QString selected_home_scene_id(QTableWidget * table)
{
  if (!table) return QString();
  const int row = table->currentRow();
  if (row < 0 || row >= table->rowCount() || !table->item(row, 0)) return QString();
  return scene_id_at(table, row);
}

inline QString safe_preview_cache_component(QString value)
{
  value = value.trimmed();
  if (value.isEmpty()) return QStringLiteral("workcell");
  for (int i = 0; i < value.size(); ++i) {
    const QChar c = value.at(i);
    if (!c.isLetterOrNumber() && c != QLatin1Char('_') && c != QLatin1Char('-')) value[i] = QLatin1Char('_');
  }
  return value;
}

inline QString home_preview_cache_root()
{
  QString root = QStandardPaths::writableLocation(QStandardPaths::CacheLocation);
  if (root.trimmed().isEmpty()) {
    root = QDir(QDir::tempPath()).filePath(QStringLiteral("workcell_studio"));
  }
  root = QDir(root).filePath(QStringLiteral("home_previews"));
  QDir().mkpath(root);
  return QDir::cleanPath(root);
}

inline qint64 home_preview_scene_stamp(const QString & workspace_root, const QString & scene_id)
{
  const QDateTime updated = scene_last_updated(workspace_root, scene_id);
  return updated.isValid() ? updated.toMSecsSinceEpoch() : 0;
}

inline QString exact_home_preview_cache_path(const QString & workspace_root, const QString & scene_id)
{
  const QString safe_scene = safe_preview_cache_component(scene_id);
  return QDir(home_preview_cache_root()).filePath(
    QStringLiteral("%1-%2.png").arg(safe_scene).arg(home_preview_scene_stamp(workspace_root, scene_id)));
}

inline QString newest_home_preview_cache_path(const QString & scene_id)
{
  QDir cache(home_preview_cache_root());
  const QString prefix = safe_preview_cache_component(scene_id) + QLatin1Char('-');
  const QStringList matches = cache.entryList(
    QStringList{prefix + QStringLiteral("*.png")}, QDir::Files, QDir::Time);
  return matches.isEmpty() ? QString() : cache.filePath(matches.first());
}

inline void prune_old_home_preview_cache(const QString & scene_id, const QString & keep_path)
{
  QDir cache(home_preview_cache_root());
  const QString prefix = safe_preview_cache_component(scene_id) + QLatin1Char('-');
  const QString keep_name = QFileInfo(keep_path).fileName();
  const QStringList matches = cache.entryList(
    QStringList{prefix + QStringLiteral("*.png")}, QDir::Files, QDir::Time);
  int retained_old = 0;
  for (const QString & name : matches) {
    if (name == keep_name) continue;
    // Keep one previous real Product View snapshot as a useful stale-while-refreshing fallback.
    if (retained_old++ == 0) continue;
    cache.remove(name);
  }
}

inline bool preview_pixmap_has_useful_variation(const QPixmap & pixmap)
{
  if (pixmap.isNull() || pixmap.width() < 64 || pixmap.height() < 48) return false;
  const QImage image = pixmap.toImage().convertToFormat(QImage::Format_RGB32);
  if (image.isNull()) return false;
  QColor first;
  bool have_first = false;
  int varied = 0;
  const int step_x = qMax(1, image.width() / 12);
  const int step_y = qMax(1, image.height() / 8);
  for (int y = 0; y < image.height(); y += step_y) {
    for (int x = 0; x < image.width(); x += step_x) {
      const QColor color(image.pixel(x, y));
      if (!have_first) {
        first = color;
        have_first = true;
        continue;
      }
      if (qAbs(color.red() - first.red()) + qAbs(color.green() - first.green()) +
          qAbs(color.blue() - first.blue()) > 36) {
        if (++varied >= 4) return true;
      }
    }
  }
  return false;
}

inline void show_preview_pixmap(QLabel * label, const QPixmap & pixmap, const QString & tooltip)
{
  if (!label || pixmap.isNull()) return;
  QSize target = label->contentsRect().size();
  if (target.width() < 120 || target.height() < 90) target = QSize(360, 220);
  label->setText(QString());
  label->setPixmap(pixmap.scaled(target, Qt::KeepAspectRatio, Qt::SmoothTransformation));
  label->setToolTip(tooltip);
}

inline bool show_preview_file(QLabel * label, const QString & path, const QString & tooltip)
{
  if (!label || path.trimmed().isEmpty() || !QFileInfo(path).isFile()) return false;
  QPixmap pixmap(path);
  if (!preview_pixmap_has_useful_variation(pixmap)) return false;
  show_preview_pixmap(label, pixmap, tooltip);
  return true;
}

inline void show_fast_home_preview(
  QTableWidget * table, QLabel * label, const QString & workspace_root)
{
  if (!table || !label) return;
  const QString scene_id = selected_home_scene_id(table);
  label->setPixmap(QPixmap());

  if (scene_id.isEmpty()) {
    label->setText(QStringLiteral("Select a workcell\nto preview its scene"));
    label->setToolTip(QString());
    return;
  }

  const QString exact = exact_home_preview_cache_path(workspace_root, scene_id);
  if (show_preview_file(label, exact, QStringLiteral("Cached Product View snapshot · current scene"))) return;

  const QString previous = newest_home_preview_cache_path(scene_id);
  if (show_preview_file(label, previous,
      QStringLiteral("Cached Product View snapshot · refreshing in background"))) return;

  // Existing smoke/acceptance/Product View PNG evidence is also instant and truthful.
  const QString stored = find_preview_path(workspace_root, scene_id);
  if (show_preview_file(label, stored, QStringLiteral("Stored scene snapshot · %1").arg(stored))) return;

  label->setText(QStringLiteral("Preparing first preview…\nThis workcell will open instantly next time."));
  label->setToolTip(QStringLiteral(
    "The canonical Product View is rendering once in the background. Home will cache the resulting scene image."));
}

#ifdef WORKCELL_BUILDER_HAS_WEBENGINE
inline ScenePreviewWidget * canonical_product_view(QMainWindow * window)
{
  return window ? window->findChild<ScenePreviewWidget *>() : nullptr;
}

inline QWebEngineView * canonical_product_view_web(ScenePreviewWidget * preview)
{
  return preview ? preview->findChild<QWebEngineView *>(QStringLiteral("embeddedWeb3dProductView")) : nullptr;
}

inline void capture_canonical_product_view_snapshot(
  QMainWindow * window, QTableWidget * table, QLabel * label, const QString & workspace_root)
{
  if (!window || !table || !label) return;
  const QString scene_id = selected_home_scene_id(table);
  if (scene_id.isEmpty()) return;

  ScenePreviewWidget * source_preview = canonical_product_view(window);
  if (!source_preview || !source_preview->runtime_preview_has_usable_content()) return;
  const ScenePreviewWidget::PreviewContext context = source_preview->preview_context();
  if (context.scene_id.trimmed() != scene_id) return;

  QWebEngineView * source_web = canonical_product_view_web(source_preview);
  if (!source_web || !source_web->page()) return;

  const QString cache_path = exact_home_preview_cache_path(workspace_root, scene_id);
  if (show_preview_file(label, cache_path, QStringLiteral("Cached Product View snapshot · current scene"))) return;

  const QString request_key = scene_id + QLatin1Char('|') + cache_path;
  if (label->property("homeSnapshotCaptureInFlight").toString() == request_key) return;
  label->setProperty("homeSnapshotCaptureInFlight", request_key);

  // Capture the already-rendered canonical canvas.  This is deliberately not a
  // second WebEngine navigation: no duplicate Three.js scene, mesh fetch, GL
  // compositor surface, HTTP server, or scene preparation is created for Home.
  static const char kCaptureCanvasScript[] = R"JS(
(() => {
  const canvas = document.getElementById('scene-canvas');
  if (!canvas || canvas.width < 64 || canvas.height < 48) return '';
  try { return canvas.toDataURL('image/png'); } catch (_) { return ''; }
})()
)JS";

  const QPointer<QLabel> safe_label(label);
  const QPointer<QTableWidget> safe_table(table);
  source_web->page()->runJavaScript(QString::fromUtf8(kCaptureCanvasScript),
    [safe_label, safe_table, workspace_root, scene_id, cache_path, request_key](const QVariant & value) {
      if (!safe_label) return;
      if (safe_label->property("homeSnapshotCaptureInFlight").toString() == request_key)
        safe_label->setProperty("homeSnapshotCaptureInFlight", QString());

      const QString data_url = value.toString();
      static const QString prefix = QStringLiteral("data:image/png;base64,");
      if (!data_url.startsWith(prefix)) return;
      const QByteArray bytes = QByteArray::fromBase64(data_url.mid(prefix.size()).toLatin1());
      QPixmap pixmap;
      if (!pixmap.loadFromData(bytes, "PNG") || !preview_pixmap_has_useful_variation(pixmap)) return;

      QDir().mkpath(QFileInfo(cache_path).absolutePath());
      if (!pixmap.save(cache_path, "PNG")) return;
      prune_old_home_preview_cache(scene_id, cache_path);

      if (!safe_table || selected_home_scene_id(safe_table) != scene_id) return;
      show_preview_pixmap(safe_label, pixmap, QStringLiteral("Cached Product View snapshot · current scene"));
    });
}
#endif

inline void install_home_snapshot_preview(
  QMainWindow * window, const QString & workspace_root)
{
  if (!window || QApplication::arguments().contains(QStringLiteral("--scene3d-smoke"))) return;
  if (window->property("studioHomeSnapshotPreviewInstalled").toBool()) return;

  QTableWidget * table = scene_table(window);
  QLabel * preview_label = window->findChild<QLabel *>(QStringLiteral("studioTargetPreview"));
  if (!table || !preview_label) return;
  window->setProperty("studioHomeSnapshotPreviewInstalled", true);

  // Keep the target shell's existing lightweight QLabel.  A Home-page preview
  // should cost roughly the same as displaying an image, not another browser.
  preview_label->show();
  preview_label->setAlignment(Qt::AlignCenter);
  preview_label->setMinimumHeight(220);
  preview_label->setMaximumHeight(250);
  preview_label->setScaledContents(false);

  const QPointer<QMainWindow> safe_window(window);
  const QPointer<QLabel> safe_label(preview_label);
  QObject::connect(table, &QTableWidget::itemSelectionChanged, preview_label,
    [safe_window, safe_label, table, workspace_root]() {
      if (!safe_window || !safe_label) return;
      show_fast_home_preview(table, safe_label, workspace_root);
#ifdef WORKCELL_BUILDER_HAS_WEBENGINE
      capture_canonical_product_view_snapshot(safe_window, table, safe_label, workspace_root);
#endif
    });

#ifdef WORKCELL_BUILDER_HAS_WEBENGINE
  if (ScenePreviewWidget * source_preview = canonical_product_view(window)) {
    QObject::connect(source_preview, &ScenePreviewWidget::embedded_product_view_runtime_state_changed,
      preview_label, [safe_window, safe_label, table, workspace_root](const QString &, bool usable) {
        if (!usable || !safe_window || !safe_label) return;
        capture_canonical_product_view_snapshot(safe_window, table, safe_label, workspace_root);
      });
    QObject::connect(source_preview, &ScenePreviewWidget::post_save_product_view_refresh_finished,
      preview_label,
      [safe_window, safe_label, table, workspace_root](int, quint64, quint64, bool success, const QString &) {
        if (!success || !safe_window || !safe_label) return;
        capture_canonical_product_view_snapshot(safe_window, table, safe_label, workspace_root);
      });
  }
#endif

  show_fast_home_preview(table, preview_label, workspace_root);
#ifdef WORKCELL_BUILDER_HAS_WEBENGINE
  capture_canonical_product_view_snapshot(window, table, preview_label, workspace_root);
#endif
}

}  // namespace home_workcells
}  // namespace workcell_builder
