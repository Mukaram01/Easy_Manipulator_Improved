#pragma once

// Read-only visual preview for the Home inspector.
//
// Home does not generate its own scene renderer.  The selected scene is already
// prepared by the canonical ScenePreviewWidget/Product View lifecycle, even while
// the Home page is visible.  Once that Product View reports usable content, Home
// mirrors the exact prepared loopback viewer URL into a small viewport-only
// QWebEngineView.  This keeps Home visually truthful without introducing another
// scene export contract, another HTTP server, or another source-of-truth path.
//
// Before Product View is ready, Home may show an existing PNG snapshot.  If no
// snapshot exists, it shows a neutral graphical "preparing" card.  It never
// generates files under scenes/, never displays the old static SVG schematic,
// and never presents a blank/error panel as the normal preview state.

#include <QApplication>
#include <QBoxLayout>
#include <QColor>
#include <QDir>
#include <QFileInfo>
#include <QLabel>
#include <QLayout>
#include <QMainWindow>
#include <QPointer>
#include <QSizePolicy>
#include <QTableWidget>
#include <QUrl>

#ifdef WORKCELL_BUILDER_HAS_WEBENGINE
#include <QWebEnginePage>
#include <QWebEngineSettings>
#include <QWebEngineView>
#endif

#include "home_workcells_target_shell.hpp"
#include "scene_preview_widget.h"

namespace workcell_builder
{
namespace home_workcells
{

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

inline QString preparing_product_view_html(
  const QString & scene_id, const QString & robot, const QString & tool)
{
  const QString title = friendly_workcell_name(scene_id).toHtmlEscaped();
  const QString robot_text = robot.toHtmlEscaped();
  const QString tool_text = tool.toHtmlEscaped();
  return QStringLiteral(
    "<!doctype html><html><head><meta charset='utf-8'>"
    "<style>html,body{margin:0;width:100%%;height:100%%;overflow:hidden;background:#111b25;color:#dbe8f4;font-family:sans-serif;}"
    ".scene{position:relative;width:100%%;height:100%%;background:radial-gradient(circle at 52%% 32%%,#263746,#111b25 67%%);}"
    ".grid{position:absolute;left:7%%;right:7%%;bottom:10%%;height:43%%;transform:perspective(420px) rotateX(58deg);transform-origin:center bottom;border:1px solid #405463;background:repeating-linear-gradient(0deg,transparent,transparent 21px,#263846 22px),repeating-linear-gradient(90deg,transparent,transparent 29px,#263846 30px);}"
    ".base{position:absolute;left:34%%;bottom:25%%;width:45px;height:22px;border-radius:50%%;background:#6f8291;}"
    ".robot{position:absolute;left:36%%;bottom:28%%;width:26px;height:77px;border-radius:14px;background:linear-gradient(#bdc8d1,#6c7e8c);}"
    ".arm{position:absolute;left:38%%;bottom:55%%;width:86px;height:15px;border-radius:9px;background:#9dacb8;transform:rotate(17deg);transform-origin:left center;}"
    ".bin{position:absolute;right:22%%;bottom:25%%;width:62px;height:42px;border:3px solid #2864f0;background:#0c3198;border-radius:4px;transform:skewY(-6deg);}"
    ".badge{position:absolute;right:12px;top:12px;padding:5px 8px;border-radius:999px;background:#18334a;color:#9fd5ff;font-size:9px;font-weight:700;letter-spacing:.04em;}"
    ".caption{position:absolute;left:14px;top:13px;font-size:12px;font-weight:700;color:#eef6ff;}"
    ".meta{position:absolute;left:14px;bottom:10px;font-size:9px;color:#9fb2c5;}"
    "</style></head><body><div class='scene'><div class='caption'>%1</div><div class='badge'>PREPARING LIVE 3D</div>"
    "<div class='grid'></div><div class='base'></div><div class='robot'></div><div class='arm'></div><div class='bin'></div>"
    "<div class='meta'>%2 · %3</div></div></body></html>")
    .arg(title, robot_text, tool_text);
}

#ifdef WORKCELL_BUILDER_HAS_WEBENGINE
inline bool is_canonical_product_view_url(const QUrl & url)
{
  if (!url.isValid() || url.scheme().compare(QStringLiteral("http"), Qt::CaseInsensitive) != 0) return false;
  const QString host = url.host().toLower();
  if (host != QStringLiteral("127.0.0.1") && host != QStringLiteral("localhost")) return false;
  return url.path() == QStringLiteral("/workcell_studio_web/viewer/index.html");
}

inline QString selected_home_scene_id(QTableWidget * table)
{
  if (!table) return QString();
  const int row = table->currentRow();
  if (row < 0 || row >= table->rowCount() || !table->item(row, 0)) return QString();
  return scene_id_at(table, row);
}

inline ScenePreviewWidget * canonical_product_view(QMainWindow * window)
{
  return window ? window->findChild<ScenePreviewWidget *>() : nullptr;
}

inline QWebEngineView * canonical_product_view_web(ScenePreviewWidget * preview)
{
  return preview ? preview->findChild<QWebEngineView *>(QStringLiteral("embeddedWeb3dProductView")) : nullptr;
}

inline void apply_home_viewport_only_chrome(QWebEngineView * web)
{
  if (!web || !is_canonical_product_view_url(web->url())) return;
  // The mirrored page is the real Product View, but Home only needs the rendered
  // scene.  Hide authoring/review chrome and make the canvas fill the inspector.
  // Pointer interaction is disabled at the QWidget level as a second read-only
  // boundary, so this page cannot mutate preview transforms from Home.
  static const char kViewportOnlyScript[] = R"JS(
(() => {
  const id = 'workcell-home-preview-style';
  let style = document.getElementById(id);
  if (!style) {
    style = document.createElement('style');
    style.id = id;
    document.head.appendChild(style);
  }
  style.textContent = `
    html, body { margin:0 !important; width:100% !important; height:100% !important; overflow:hidden !important; background:#111b25 !important; }
    .topbar, .object-panel, .details-panel, #scene-health, #dirty-state,
    #initial-pose-status, #placement-status, #error-state, .label-layer { display:none !important; }
    .app-shell { display:block !important; width:100% !important; height:100vh !important; min-height:0 !important; margin:0 !important; padding:0 !important; }
    .viewport-panel { position:relative !important; display:block !important; width:100% !important; height:100vh !important; min-width:0 !important; min-height:0 !important; margin:0 !important; padding:0 !important; border:0 !important; overflow:hidden !important; background:#111b25 !important; }
    #scene-canvas { position:absolute !important; inset:0 !important; display:block !important; width:100% !important; height:100% !important; margin:0 !important; }
  `;
  document.documentElement.style.background = '#111b25';
  document.body.style.background = '#111b25';
  window.dispatchEvent(new Event('resize'));
  return true;
})()
)JS";
  web->page()->runJavaScript(QString::fromUtf8(kViewportOnlyScript));
}

inline bool mirror_canonical_product_view_if_ready(
  QMainWindow * window, QTableWidget * table, QWebEngineView * home_web)
{
  if (!window || !table || !home_web) return false;
  const QString scene_id = selected_home_scene_id(table);
  if (scene_id.isEmpty()) return false;

  ScenePreviewWidget * preview = canonical_product_view(window);
  if (!preview || !preview->runtime_preview_has_usable_content()) return false;
  const ScenePreviewWidget::PreviewContext context = preview->preview_context();
  if (context.scene_id.trimmed() != scene_id) return false;

  QWebEngineView * source_web = canonical_product_view_web(preview);
  if (!source_web) return false;
  const QUrl viewer_url = source_web->url();
  if (!is_canonical_product_view_url(viewer_url)) return false;

  const QString active = home_web->property("homeMirroredProductViewUrl").toString();
  if (active == viewer_url.toString() && home_web->url() == viewer_url) {
    apply_home_viewport_only_chrome(home_web);
    return true;
  }

  home_web->setProperty("homeMirroredProductViewUrl", viewer_url.toString());
  home_web->setProperty("homeMirroredSceneId", scene_id);
  home_web->setToolTip(QStringLiteral("Live read-only Product View · %1").arg(scene_id));
  home_web->setZoomFactor(1.0);
  home_web->setUrl(viewer_url);
  return true;
}

inline void show_home_preview_fallback(
  QTableWidget * table, QWebEngineView * web, const QString & workspace_root)
{
  if (!table || !web) return;
  const int row = table->currentRow();
  const bool selected = row >= 0 && row < table->rowCount() && table->item(row, 0);
  const QString scene_id = selected ? scene_id_at(table, row) : QStringLiteral("workcell");
  const QString robot = selected && table->item(row, 2)
    ? clean_robot(table->item(row, 2)->text()) : QStringLiteral("Robot");
  const QString tool = selected && table->item(row, 3)
    ? clean_tool(table->item(row, 3)->text()) : QStringLiteral("Tool");

  web->setProperty("homeMirroredProductViewUrl", QString());
  web->setProperty("homeMirroredSceneId", scene_id);

  // A real stored raster snapshot is still useful while the live viewer starts.
  // Deliberately ignore generated static SVG/HTML previews: they are schematic,
  // not Product View evidence, and were visually misleading in Home.
  const QString image = selected ? find_preview_path(workspace_root, scene_id) : QString();
  if (!image.isEmpty() && QFileInfo(image).isFile()) {
    const QString suffix = QFileInfo(image).suffix().toLower();
    if (suffix == QStringLiteral("png") || suffix == QStringLiteral("jpg") ||
        suffix == QStringLiteral("jpeg") || suffix == QStringLiteral("webp")) {
      web->setZoomFactor(1.0);
      web->setHtml(local_visual_html(image),
        QUrl::fromLocalFile(QFileInfo(image).absolutePath() + QDir::separator()));
      web->setToolTip(QStringLiteral("Stored scene snapshot · %1").arg(image));
      return;
    }
  }

  web->setZoomFactor(1.0);
  web->setHtml(preparing_product_view_html(scene_id, robot, tool));
  web->setToolTip(QStringLiteral("Preparing the live read-only Product View."));
}

inline void refresh_home_web_preview(
  QMainWindow * window, QTableWidget * table, const QString & workspace_root)
{
  auto * web = window ? window->findChild<QWebEngineView *>(QStringLiteral("studioTargetWebPreview")) : nullptr;
  if (!web || !table) return;
  if (mirror_canonical_product_view_if_ready(window, table, web)) return;
  show_home_preview_fallback(table, web, workspace_root);
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
  web->setAttribute(Qt::WA_TransparentForMouseEvents, true);
  web->settings()->setAttribute(QWebEngineSettings::JavascriptEnabled, true);
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
  QObject::connect(web, &QWebEngineView::loadFinished, web, [web](bool ok) {
    if (ok) apply_home_viewport_only_chrome(web);
  });

  if (ScenePreviewWidget * source_preview = canonical_product_view(window)) {
    QObject::connect(source_preview, &ScenePreviewWidget::embedded_product_view_runtime_state_changed,
      web, [safe_window, table, workspace_root](const QString &, bool usable) {
        if (usable && safe_window) refresh_home_web_preview(safe_window, table, workspace_root);
      });
    if (QWebEngineView * source_web = canonical_product_view_web(source_preview)) {
      QObject::connect(source_web, &QWebEngineView::urlChanged, web,
        [safe_window, table, workspace_root](const QUrl &) {
          if (safe_window) refresh_home_web_preview(safe_window, table, workspace_root);
        });
    }
  }

  refresh_home_web_preview(window, table, workspace_root);
#else
  // Non-WebEngine builds retain the existing QLabel. Product View remains the
  // authoritative visual surface and Home does not fabricate a scene image.
  legacy_preview->setText(QStringLiteral("Open Product View for the live 3D scene"));
  legacy_preview->setToolTip(QStringLiteral("This build does not include Qt WebEngine."));
#endif
}

}  // namespace home_workcells
}  // namespace workcell_builder
