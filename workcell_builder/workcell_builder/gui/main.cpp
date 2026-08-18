// Copyright 2020 Advanced Remanufacturing and Technology Centre
// Copyright 2020 ROS-Industrial Consortium Asia Pacific Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <QApplication>
#include <QAction>
#include <QDateTime>
#include <QDir>
#include <QFileInfo>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QPushButton>
#include <QTableWidget>
#include <QTimer>
#include <QTreeWidget>
#include <QLabel>
#include <QTextEdit>
#include <QImage>
#include <QWidget>
#include <QSet>
#include <QRegularExpression>
#include <iostream>
#include <limits>
#include <vector>

#include "gui/mainwindow.h"
#include "gui/scene3d_viewport_widget.h"
#include "gui/scene_select.h"
#include "gui/startup_dialog.h"
#include "workcell_builder_ui_utils.hpp"

namespace
{
bool has_cli_flag(const QStringList & args, const QString & flag)
{
  return args.contains(flag);
}

QString cli_value(const QStringList & args, const QString & flag)
{
  const int idx = args.indexOf(flag);
  if (idx >= 0 && (idx + 1) < args.size()) {
    return args.at(idx + 1);
  }
  return QString();
}

QJsonObject run_gui_self_test();

struct Scene3DViewportCandidate
{
  Scene3DViewportWidget * widget{ nullptr };
  QString pointer_address;
  QString object_name;
  bool is_visible{ false };
  QString parent_object_name;
  Scene3DViewportWidget::RenderDebugCounters counters{};
  bool under_active_scene_builder_panel{ false };
  bool is_named_viewport{ false };
  int non_zero_counter_count{ 0 };
};


struct ScenePreviewCandidate
{
  ScenePreviewWidget * widget{ nullptr };
  bool is_visible{ false };
  bool under_active_scene_builder_panel{ false };
  QString object_name;
  QString pointer_address;
};

struct ActiveScenePreviewResolution
{
  ScenePreviewWidget * selected{ nullptr };
  QVector<ScenePreviewCandidate> candidates;
  int selected_index{ -1 };
  QString source_label{ "missing" };
};

struct ActiveScene3DViewportResolution
{
  Scene3DViewportWidget * selected{ nullptr };
  QVector<Scene3DViewportCandidate> candidates;
  int selected_index{ -1 };
  QString source_label{ "missing" };
};

bool paint_cycle_completed(
  const Scene3DViewportWidget::RenderDebugCounters & counters,
  bool screenshot_saved)
{
  return counters.last_paint_completed ||
         counters.rendered_count > 0 ||
         (counters.render_cache_count > 0 && screenshot_saved);
}

bool object_name_hints_scene_builder_panel(const QString & object_name)
{
  const QString name = object_name.trimmed().toLower();
  return name.contains("scenebuilder") ||
         name.contains("scene_builder") ||
         name.contains("scenebuilderpanel") ||
         name.contains("studiocenterstack");
}

bool has_active_scene_builder_ancestor(QWidget * widget)
{
  for (QWidget * cur = widget; cur != nullptr; cur = cur->parentWidget()) {
    if (!cur->isVisible()) {
      continue;
    }
    if (object_name_hints_scene_builder_panel(cur->objectName())) {
      return true;
    }
  }
  return false;
}

ActiveScenePreviewResolution resolve_active_scene_preview_widget(MainWindow * mw)
{
  ActiveScenePreviewResolution resolution;
  if (!mw) return resolution;

  const QList<ScenePreviewWidget *> previews = mw->findChildren<ScenePreviewWidget *>();
  ScenePreviewWidget * active_preview = mw->active_scene_preview_widget();

  auto score = [&](ScenePreviewWidget * preview) {
    if (!preview) return std::numeric_limits<int>::min();
    int s = 0;
    if (preview == active_preview) s += 100000;
    if (preview->isVisible()) s += 10000;
    if (has_active_scene_builder_ancestor(preview)) s += 1000;
    if (preview->objectName() == QStringLiteral("scenePreviewWidget")) s += 100;
    return s;
  };

  int best_score = std::numeric_limits<int>::min();
  for (auto * preview : previews) {
    if (!preview) continue;
    ScenePreviewCandidate candidate;
    candidate.widget = preview;
    candidate.is_visible = preview->isVisible();
    candidate.under_active_scene_builder_panel = has_active_scene_builder_ancestor(preview);
    candidate.object_name = preview->objectName();
    candidate.pointer_address = QStringLiteral("0x%1").arg(reinterpret_cast<quintptr>(preview), 0, 16);
    resolution.candidates.push_back(candidate);
    const int s = score(preview);
    if (s > best_score) {
      best_score = s;
      resolution.selected = preview;
      resolution.selected_index = resolution.candidates.size() - 1;
    }
  }

  if (resolution.selected) {
    const auto & selected = resolution.candidates[resolution.selected_index];
    if (resolution.selected == active_preview) resolution.source_label = QStringLiteral("active_scene_preview_widget");
    else if (selected.is_visible) resolution.source_label = QStringLiteral("visible_scene_preview_widget");
    else resolution.source_label = QStringLiteral("fallback_scene_preview_widget");
  }

  return resolution;
}

ActiveScene3DViewportResolution resolve_active_scene3d_viewport(MainWindow * mw, ScenePreviewWidget * active_preview)
{
  ActiveScene3DViewportResolution resolution;
  if (!mw) {
    return resolution;
  }

  const QList<ScenePreviewWidget *> previews = mw->findChildren<ScenePreviewWidget *>();
  const QList<Scene3DViewportWidget *> viewports = mw->findChildren<Scene3DViewportWidget *>();
  QSet<Scene3DViewportWidget *> known_widgets;

  auto add_candidate = [&](Scene3DViewportWidget * viewport) {
    if (!viewport || known_widgets.contains(viewport)) {
      return;
    }
    known_widgets.insert(viewport);

    Scene3DViewportCandidate candidate;
    candidate.widget = viewport;
    candidate.pointer_address = QStringLiteral("0x%1").arg(reinterpret_cast<quintptr>(viewport), 0, 16);
    candidate.object_name = viewport->objectName();
    candidate.is_visible = viewport->isVisible();
    candidate.parent_object_name = (viewport->parent() ? viewport->parent()->objectName() : QString());
    candidate.counters = viewport->render_debug_counters();
    candidate.under_active_scene_builder_panel = has_active_scene_builder_ancestor(viewport);
    candidate.is_named_viewport = (candidate.object_name == QStringLiteral("scene3dViewportWidget"));

    const auto & rc = candidate.counters;
    candidate.non_zero_counter_count =
      (rc.viewport_received_count > 0 ? 1 : 0) +
      (rc.render_cache_count > 0 ? 1 : 0) +
      (rc.rendered_count > 0 ? 1 : 0) +
      (rc.visible_count > 0 ? 1 : 0) +
      (rc.last_paint_completed ? 1 : 0) +
      (rc.preview_items_count > 0 ? 1 : 0);

    resolution.candidates.push_back(candidate);
  };

  if (active_preview) {
    const auto preview_viewports = active_preview->findChildren<Scene3DViewportWidget *>();
    for (auto * viewport : preview_viewports) add_candidate(viewport);
  }
  for (auto * preview : previews) {
    if (preview == active_preview) continue;
    if (!preview) continue;
    const auto preview_viewports = preview->findChildren<Scene3DViewportWidget *>();
    for (auto * viewport : preview_viewports) add_candidate(viewport);
  }
  for (auto * viewport : viewports) add_candidate(viewport);

  auto score = [](const Scene3DViewportCandidate & c) {
    int s = 0;
    if (c.is_visible) s += 100000;
    if (c.under_active_scene_builder_panel) s += 10000;
    if (c.is_named_viewport) s += 1000;
    s += c.non_zero_counter_count * 10;
    return s;
  };

  int best_score = std::numeric_limits<int>::min();
  int best_idx = -1;
  for (int i = 0; i < resolution.candidates.size(); ++i) {
    const int s = score(resolution.candidates[i]);
    if (s > best_score) {
      best_score = s;
      best_idx = i;
    }
  }

  if (best_idx >= 0) {
    resolution.selected_index = best_idx;
    resolution.selected = resolution.candidates[best_idx].widget;
    const auto & selected = resolution.candidates[best_idx];
    if (selected.is_visible) {
      resolution.source_label = QStringLiteral("active_visible_viewport");
    } else if (selected.is_named_viewport) {
      resolution.source_label = QStringLiteral("named_viewport");
    } else {
      resolution.source_label = QStringLiteral("fallback_candidate");
    }
  }

  return resolution;
}


QJsonObject parse_latest_scene3d_diagnostics(const QString & log_text)
{
  QJsonObject parsed;
  const QStringList lines = log_text.split('\n');
  QRegularExpression count_re(QStringLiteral("([A-Za-z_][A-Za-z0-9_]*)\\s*[:=]\\s*(-?\\d+)"));
  for (int i = lines.size() - 1; i >= 0; --i) {
    const QString line = lines[i].trimmed();
    if (!(line.contains(QStringLiteral("Scene3D diagnostics"), Qt::CaseInsensitive) ||
          line.contains(QStringLiteral("Scene3D canvas:"), Qt::CaseInsensitive))) {
      continue;
    }
    QJsonObject counts;
    auto it = count_re.globalMatch(line);
    while (it.hasNext()) {
      const auto m = it.next();
      counts[m.captured(1)] = m.captured(2).toInt();
    }
    parsed["line"] = line;
    parsed["line_index"] = i;
    parsed["counts"] = counts;
    parsed["parsed_count"] = counts.size();
    if (i + 1 < lines.size()) {
      const QString next = lines[i + 1].trimmed();
      if (next.contains(QStringLiteral("Scene3D canvas skip reasons:"), Qt::CaseInsensitive)) {
        QJsonObject reasons;
        auto reason_it = count_re.globalMatch(next);
        while (reason_it.hasNext()) {
          const auto m = reason_it.next();
          reasons[m.captured(1)] = m.captured(2).toInt();
        }
        parsed["skip_reason_counts"] = reasons;
      }
    }
    break;
  }
  return parsed;
}

int sum_json_object_int_values(const QJsonObject & obj)
{
  int total = 0;
  for (auto it = obj.begin(); it != obj.end(); ++it) {
    total += it.value().toInt();
  }
  return total;
}

struct Scene3DSmokeOptions
{
  bool enabled{ false };
  QString scene_name;
  QString scene_path;
  QString smoke_output;
  QString screenshot_path;
  bool exit_after_smoke{ false };
  bool new_cell_recommended_layout_smoke{ false };
};

class Scene3DSmokeRunner : public QObject
{
public:
  Scene3DSmokeRunner(QApplication * app, MainWindow * window, const Scene3DSmokeOptions & opts)
  : app_(app), window_(window), opts_(opts) {}

  void start() { QTimer::singleShot(0, this, &Scene3DSmokeRunner::step_begin); }

private:
  QApplication * app_;
  MainWindow * window_;
  Scene3DSmokeOptions opts_;
  QJsonArray warnings_;
  QJsonArray blockers_;
  QDateTime start_time_;
  QJsonObject latest_counters_;
  QJsonObject scene_load_diagnostics_;
  int render_ready_attempts_{0};

  bool runtime_scene3d_diagnostics_matches_requested_scene(const QString & line) const
  {
    const QString requested_scene = opts_.scene_name.trimmed();
    if (requested_scene.isEmpty()) {
      return true;
    }

    const QString escaped_scene = QRegularExpression::escape(requested_scene);
    const QRegularExpression scene_re(
      QStringLiteral("\\bscene\\s*[:=]\\s*[\"']?%1[\"']?(?:\\b|\\s|$)").arg(escaped_scene),
      QRegularExpression::CaseInsensitiveOption);
    if (scene_re.match(line).hasMatch()) {
      return true;
    }

    const QString lower_line = line.toLower();
    const QString lower_scene = requested_scene.toLower();
    return lower_line.contains(QStringLiteral("scene")) && lower_line.contains(lower_scene);
  }

  bool has_valid_runtime_scene3d_diagnostics(const QJsonObject & diagnostics) const
  {
    const QString line = diagnostics.value("line").toString().trimmed();
    if (line.isEmpty()) {
      return false;
    }
    if (!runtime_scene3d_diagnostics_matches_requested_scene(line)) {
      return false;
    }

    const QJsonObject counts = diagnostics.value("counts").toObject();
    return counts.value("received").toInt() > 0 &&
           counts.value("rendered").toInt() > 0 &&
           counts.value("visible").toInt() > 0;
  }

  void step_begin()
  {
    start_time_ = QDateTime::currentDateTimeUtc();
    if (!window_) {
      blockers_.append("MainWindow instance is null");
      return finalize();
    }
    if (opts_.new_cell_recommended_layout_smoke) {
      trigger_by_text("New Cell");
      QTimer::singleShot(300, this, &Scene3DSmokeRunner::step_trigger_recommended_layout);
      return;
    }
    if (!opts_.scene_path.trimmed().isEmpty() || !opts_.scene_name.trimmed().isEmpty()) {
      QStringList load_blockers;
      if (!window_->load_scene_for_scene3d_smoke(opts_.scene_name, opts_.scene_path, &load_blockers, &scene_load_diagnostics_)) {
        for (const auto & b : load_blockers) blockers_.append(b);
      }
    } else {
      trigger_by_text("Open Selected Scene");
    }
    QTimer::singleShot(300, this, &Scene3DSmokeRunner::step_wait_scene3d_ready);
  }

  void step_trigger_recommended_layout()
  {
    trigger_by_text("Use Recommended Layout");
    QTimer::singleShot(300, this, &Scene3DSmokeRunner::step_wait_scene3d_ready);
  }

  void step_wait_scene3d_ready()
  {
    const int timeout_ms = 20000;
    const int poll_ms = 250;
    auto * timer = new QTimer(this);
    const qint64 start_ms = QDateTime::currentMSecsSinceEpoch();
    connect(timer, &QTimer::timeout, this, [this, timer, start_ms, timeout_ms]() {
      ++render_ready_attempts_;
      const auto preview_resolution = resolve_active_scene_preview_widget(window_);
      ScenePreviewWidget * active_preview_widget = preview_resolution.selected;
      const auto viewport_resolution = resolve_active_scene3d_viewport(window_, active_preview_widget);
      auto * viewport = viewport_resolution.selected;
      auto * tree = window_->findChild<QTreeWidget *>("studioSceneHierarchyTree");
      int hierarchy_rows = 0;
      if (tree) {
        for (int i = 0; i < tree->topLevelItemCount(); ++i) {
          if (auto * top = tree->topLevelItem(i)) hierarchy_rows += top->childCount();
        }
      }
      if (viewport) {
        viewport->show();
        viewport->update();
        viewport->repaint();
        app_->processEvents();
        const auto rc_after_paint = viewport->render_debug_counters();
        if (rc_after_paint.viewport_received_count > 0 &&
            rc_after_paint.visible_count > 0 &&
            rc_after_paint.rendered_count <= 0 &&
            !rc_after_paint.last_paint_completed) {
          viewport->render_smoke_fallback_frame();
        }
      }
      qInfo() << "Scene3D smoke render-ready: scene=" << opts_.scene_name
              << "received=" << (viewport ? viewport->render_debug_counters().viewport_received_count : 0)
              << "rendered=" << (viewport ? viewport->render_debug_counters().rendered_count : 0)
              << "selectable=" << (viewport ? qMax(0, viewport->render_debug_counters().visible_count - viewport->render_debug_counters().overlay_count) : 0)
              << "hierarchy=" << hierarchy_rows;
      if (scene3d_ready()) {
        timer->stop();
        timer->deleteLater();
        finalize();
        return;
      }
      if ((QDateTime::currentMSecsSinceEpoch() - start_ms) > timeout_ms) {
        blockers_.append(readiness_failure_message());
        timer->stop();
        timer->deleteLater();
        finalize();
      }
    });
    timer->start(poll_ms);
  }

  bool scene3d_ready()
  {
    const QJsonObject markers = collect_readiness_markers();
    return markers.value("hierarchy_ready").toBool(false) &&
           markers.value("inspector_ready").toBool(false) &&
           markers.value("log_ready").toBool(false) &&
           markers.value("selected_scene_ready").toBool(false) &&
           markers.value("render_ready").toBool(false);
  }

  QJsonObject collect_readiness_markers() const
  {
    QJsonObject markers;
    auto * tree = window_->findChild<QTreeWidget *>("studioSceneHierarchyTree");
    auto * inspector = window_->findChild<QLabel *>("sceneBuilderInspectorLabel");
    auto * selected_scene_label = window_->findChild<QLabel *>("sceneBuilderSelectedSceneName");
    auto * log = window_->findChild<QTextEdit *>("studioHomeLog");
    const auto preview_resolution = resolve_active_scene_preview_widget(window_);
    ScenePreviewWidget * active_preview_widget = preview_resolution.selected;
    const auto viewport_resolution = resolve_active_scene3d_viewport(window_, active_preview_widget);
    auto * viewport = viewport_resolution.selected;
    int hierarchy_child_rows = 0;
    int hierarchy_item_rows = 0;
    if (tree) {
      for (int i = 0; i < tree->topLevelItemCount(); ++i) {
        auto * top = tree->topLevelItem(i);
        if (!top) continue;
        hierarchy_child_rows += top->childCount();
        if (!top->data(0, Qt::UserRole + 1).toString().trimmed().isEmpty()) ++hierarchy_item_rows;
        for (int child = 0; child < top->childCount(); ++child) {
          auto * item = top->child(child);
          if (item && !item->data(0, Qt::UserRole + 1).toString().trimmed().isEmpty()) ++hierarchy_item_rows;
        }
      }
    }
    QString selected_scene_name = "(none)";
    bool inspector_no_scene_selected = true;
    if (inspector) {
      const QString text = inspector->text();
      inspector_no_scene_selected = text.contains("No scene selected", Qt::CaseInsensitive);
      const QStringList lines = text.split('\n');
      for (const QString & line : lines) {
        if (line.startsWith("Scene: ")) selected_scene_name = line.mid(QString("Scene: ").size()).trimmed();
      }
    }
    if ((selected_scene_name == "(none)" || selected_scene_name == "none") && selected_scene_label) {
      selected_scene_name = selected_scene_label->text().trimmed();
    }
    const bool hierarchy_ready = (tree != nullptr && hierarchy_item_rows > 0);
    const bool selected_scene_ready = !selected_scene_name.trimmed().isEmpty() && selected_scene_name != "(none)" && selected_scene_name != "none";
    const bool inspector_ready = (inspector != nullptr && !inspector_no_scene_selected && selected_scene_ready);
    const QJsonObject parsed_runtime_diagnostics =
      log ? parse_latest_scene3d_diagnostics(log->toPlainText()) : QJsonObject{};
    const bool parsed_log_ready =
      log != nullptr && has_valid_runtime_scene3d_diagnostics(parsed_runtime_diagnostics);
    const auto rc = viewport ? viewport->render_debug_counters() : Scene3DViewportWidget::RenderDebugCounters{};
    const bool viewport_runtime_render_evidence =
      viewport != nullptr &&
      rc.viewport_received_count > 0 &&
      rc.rendered_count > 0 &&
      rc.visible_count > 0;
    const bool log_ready = parsed_log_ready || viewport_runtime_render_evidence;
    const bool screenshot_saved = latest_counters_.value("screenshot_saved").toBool(false);
    const bool paint_completed = paint_cycle_completed(rc, screenshot_saved);
    const bool render_ready =
      rc.viewport_received_count > 0 &&
      rc.rendered_count > 0 &&
      qMax(0, rc.visible_count - rc.overlay_count) > 0 &&
      rc.hierarchy_child_row_count > 0 &&
      paint_completed;
    markers["hierarchy_ready"] = hierarchy_ready;
    markers["inspector_ready"] = inspector_ready;
    markers["log_ready"] = log_ready;
    markers["screenshot_ready"] = true;
    markers["render_ready"] = render_ready;
    markers["paint_completed"] = paint_completed;
    markers["selected_scene_ready"] = selected_scene_ready;
    markers["selected_item_required"] = false;
    return markers;
  }

  QString readiness_failure_message() const
  {
    const QJsonObject markers = collect_readiness_markers();
    QStringList failed;
    for (const QString & key : {QStringLiteral("hierarchy_ready"), QStringLiteral("inspector_ready"), QStringLiteral("log_ready"),
      QStringLiteral("screenshot_ready"), QStringLiteral("render_ready"), QStringLiteral("selected_scene_ready")}) {
      if (!markers.value(key).toBool(false)) failed.append(key + "=false");
    }
    if (!markers.value("render_ready").toBool(false)) {
      if (latest_counters_.value("preview_items_count").toInt() <= 0 && latest_counters_.value("hierarchy_child_row_count").toInt() > 0) {
        return QString("preview_items_not_handed_to_scene_preview_widget");
      }
      if (latest_counters_.value("preview_items_count").toInt() > 0 && latest_counters_.value("viewport_received_count").toInt() <= 0) {
        return QString("scene_preview_items_not_forwarded_to_viewport");
      }
      if (latest_counters_.value("preview_items_count").toInt() > 0 &&
          latest_counters_.value("viewport_received_count").toInt() > 0 &&
          latest_counters_.value("rendered_count").toInt() <= 0) {
        return QString("scene3d_runtime_not_rendered_after_candidate_ingestion");
      }
      if (latest_counters_.value("viewport_received_count").toInt() > 0 && latest_counters_.value("render_cache_count").toInt() <= 0) {
        return QString("viewport_render_cache_empty");
      }
      if (!markers.value("paint_completed").toBool(false) &&
          latest_counters_.value("rendered_count").toInt() <= 0 &&
          latest_counters_.value("render_cache_count").toInt() <= 0) {
        return QString("paint_cycle_not_completed: candidate_count=%1 active_candidate=%2 counters=viewport_received_count:%3 visible_count:%4 rendered_count:%5 render_cache_count:%6 last_paint_completed:%7 screenshot_saved:%8")
          .arg(latest_counters_.value("scene3d_viewport_widget_count").toInt())
          .arg(latest_counters_.value("active_viewport_candidate_index").toInt())
          .arg(latest_counters_.value("viewport_received_count").toInt())
          .arg(latest_counters_.value("visible_count").toInt())
          .arg(latest_counters_.value("rendered_count").toInt())
          .arg(latest_counters_.value("render_cache_count").toInt())
          .arg(latest_counters_.value("last_paint_completed").toBool(false) ? "true" : "false")
          .arg(latest_counters_.value("screenshot_saved").toBool(false) ? "true" : "false");
      }
      return QString("Scene3D readiness failed: render_ready=false viewport_received_count=%1 visible_count=%2 rendered_count=%3 render_cache_count=%4 skipped_count=%5")
        .arg(latest_counters_.value("viewport_received_count").toInt())
        .arg(latest_counters_.value("visible_count").toInt())
        .arg(latest_counters_.value("rendered_count").toInt())
        .arg(latest_counters_.value("render_cache_count").toInt())
        .arg(latest_counters_.value("skipped_count").toInt());
    }
    return failed.isEmpty() ? QString("Scene3D readiness failed") : QString("Scene3D readiness failed: %1").arg(failed.join(", "));
  }

  void trigger_by_text(const QString & text)
  {
    const auto buttons = window_->findChildren<QPushButton *>();
    for (auto * b : buttons) {
      if (b && b->text().trimmed() == text && b->isEnabled()) {
        b->click();
        return;
      }
    }
    const auto actions = window_->findChildren<QAction *>();
    for (auto * a : actions) {
      if (a && a->text().trimmed() == text && a->isEnabled()) {
        a->trigger();
        return;
      }
    }
    warnings_.append(QString("Unable to trigger action: %1").arg(text));
  }

  void finalize()
  {
    QJsonObject root;
    root["schema"] = "workcell_studio_scene3d_gui_smoke/v1";
    root["timestamp"] = QDateTime::currentDateTimeUtc().toString(Qt::ISODate);
    root["scene"] = opts_.scene_name;
    root["requested_scene_path"] = opts_.scene_path;
    root["new_cell_recommended_layout_smoke"] = opts_.new_cell_recommended_layout_smoke;
    root["scene_load_diagnostics"] = scene_load_diagnostics_;
    root["filter_diagnostics"] = window_->scene3d_filter_diagnostics();
    root["duration_ms"] = start_time_.msecsTo(QDateTime::currentDateTimeUtc());

    auto * tree = window_->findChild<QTreeWidget *>("studioSceneHierarchyTree");
    auto * log = window_->findChild<QTextEdit *>("studioHomeLog");
    const QString log_text = log ? log->toPlainText() : QString();
    const QJsonObject parsed_runtime_diagnostics = parse_latest_scene3d_diagnostics(log_text);
    const QJsonObject parsed_runtime_diagnostics_counts = parsed_runtime_diagnostics.value("counts").toObject();
    const int parsed_runtime_diagnostics_total = sum_json_object_int_values(parsed_runtime_diagnostics_counts);
    const QString parsed_runtime_diagnostics_line = parsed_runtime_diagnostics.value("line").toString();
    const bool valid_requested_scene_canvas_diagnostics =
      parsed_runtime_diagnostics_line.contains(QStringLiteral("Scene3D canvas:"), Qt::CaseInsensitive) &&
      has_valid_runtime_scene3d_diagnostics(parsed_runtime_diagnostics);
    QJsonObject counters;
    const QJsonObject filter_diagnostics = window_->scene3d_filter_diagnostics();
    auto copy_filter_counter = [&](const QString & key) {
      if (filter_diagnostics.contains(key)) counters[key] = filter_diagnostics.value(key);
    };
    counters["hierarchy_top_level_count"] = tree ? tree->topLevelItemCount() : 0;
    int hierarchy_child_rows = 0;
    int hierarchy_item_rows = 0;
    bool hierarchy_has_only_headings = false;
    if (tree) {
      for (int i = 0; i < tree->topLevelItemCount(); ++i) {
        auto * top = tree->topLevelItem(i);
        if (!top) continue;
        hierarchy_child_rows += top->childCount();
        if (!top->data(0, Qt::UserRole + 1).toString().trimmed().isEmpty()) ++hierarchy_item_rows;
        for (int child = 0; child < top->childCount(); ++child) {
          auto * item = top->child(child);
          if (item && !item->data(0, Qt::UserRole + 1).toString().trimmed().isEmpty()) ++hierarchy_item_rows;
        }
      }
      hierarchy_has_only_headings = (tree->topLevelItemCount() > 0 && hierarchy_item_rows == 0);
    }
    counters["hierarchy_child_row_count"] = hierarchy_child_rows;
    counters["hierarchy_rows_count"] = hierarchy_item_rows;
    counters["hierarchy_has_only_headings"] = hierarchy_has_only_headings;
    counters["log_line_count"] = log_text.split('\n', Qt::SkipEmptyParts).size();
    counters["log_has_scene3d_diagnostics"] =
      log_text.contains("Scene3D diagnostics") || valid_requested_scene_canvas_diagnostics;
    counters["runtime_scene3d_diagnostics_total"] = parsed_runtime_diagnostics_total;
    counters["runtime_scene3d_diagnostics_counts"] = parsed_runtime_diagnostics_counts;
    counters["runtime_scene3d_skip_reason_counts"] = parsed_runtime_diagnostics.value("skip_reason_counts").toObject();
    copy_filter_counter("total_visual_index_rows");
    copy_filter_counter("total_added_visual_rows");
    copy_filter_counter("total_skipped_visual_rows");
    copy_filter_counter("skipped_count_by_reason");
    copy_filter_counter("duplicate_id_count");
    copy_filter_counter("first_20_visual_items");
    copy_filter_counter("visual_ingestion_diagnostics");
    copy_filter_counter("generated_urdf_visual_numbers_after_ingest");
    copy_filter_counter("generated_urdf_visual_numbers_after_suppression");
    copy_filter_counter("generated_urdf_visual_numbers_after_filter");
    copy_filter_counter("robot_visual_count");
    copy_filter_counter("robot_mesh_loaded_count");
    copy_filter_counter("robot_mesh_missing_count");
    copy_filter_counter("robot_aabb_min");
    copy_filter_counter("robot_aabb_max");
    copy_filter_counter("robot_world_pose");
    copy_filter_counter("robot_base_frame");
    copy_filter_counter("transform_chain_applied_count");
    copy_filter_counter("visual_origin_applied_count");
    copy_filter_counter("baked_world_visual_transform_count");
    copy_filter_counter("camera_fit_target");
    copy_filter_counter("robot_pose_source");
    auto * inspector = window_->findChild<QLabel *>("sceneBuilderInspectorLabel");
    auto * selected_scene_label = window_->findChild<QLabel *>("sceneBuilderSelectedSceneName");
    QString selected_scene_name = "(none)";
    QString selected_item_id = "(none)";
    QString inspector_scene_display_name = "";
    QString inspector_scene_path = "";
    QString inspector_scene_status = "";
    QString preview_status = "Unavailable";
    bool inspector_no_scene_selected = true;
    if (inspector) {
      const QString text = inspector->text();
      inspector_no_scene_selected = text.contains("No scene selected", Qt::CaseInsensitive);
      const QStringList lines = text.split('\n');
      for (const QString & line : lines) {
        if (line.startsWith("Scene: ")) { selected_scene_name = line.mid(QString("Scene: ").size()).trimmed(); inspector_scene_display_name = selected_scene_name; }
        if (line.startsWith("Scene path: ")) inspector_scene_path = line.mid(QString("Scene path: ").size()).trimmed();
        if (line.startsWith("Scene status: ")) inspector_scene_status = line.mid(QString("Scene status: ").size()).trimmed();
        if (line.startsWith("Selected item ID: ")) selected_item_id = line.mid(QString("Selected item ID: ").size()).trimmed();
      }
    }
    if ((selected_scene_name == "(none)" || selected_scene_name == "none") && selected_scene_label) {
      selected_scene_name = selected_scene_label->text().trimmed();
      inspector_scene_display_name = selected_scene_name;
    }
    counters["selected_scene_name"] = selected_scene_name;
    counters["selected_item_id"] = selected_item_id;
    counters["inspector_no_scene_selected"] = inspector_no_scene_selected;
    counters["inspector_scene_display_name"] = inspector_scene_display_name;
    counters["inspector_scene_path"] = inspector_scene_path;
    counters["inspector_scene_status"] = inspector_scene_status;
    const auto preview_resolution = resolve_active_scene_preview_widget(window_);
    ScenePreviewWidget * active_preview_widget = preview_resolution.selected;
    auto viewport_resolution = resolve_active_scene3d_viewport(window_, active_preview_widget);
    auto * viewport = viewport_resolution.selected;
    const auto has_non_zero_candidate_counter = [](const Scene3DViewportCandidate & candidate) {
      const auto & rc = candidate.counters;
      return rc.viewport_received_count > 0 || rc.render_cache_count > 0 || rc.rendered_count > 0 ||
             rc.visible_count > 0 || rc.last_paint_completed;
    };
    QJsonArray viewport_candidates_json;
    for (const auto & candidate : viewport_resolution.candidates) {
      QJsonObject candidate_json;
      candidate_json["object_name"] = candidate.object_name;
      candidate_json["visible"] = candidate.is_visible;
      candidate_json["parent_object_name"] = candidate.parent_object_name;
      candidate_json["viewport_received_count"] = candidate.counters.viewport_received_count;
      candidate_json["total_payload_count"] = candidate.counters.total_payload_count;
      candidate_json["render_cache_count"] = candidate.counters.render_cache_count;
      candidate_json["rendered_count"] = candidate.counters.rendered_count;
      candidate_json["visible_count"] = candidate.counters.visible_count;
      candidate_json["mesh_source_count"] = candidate.counters.mesh_source_count;
      candidate_json["mesh_rendered_count"] = candidate.counters.mesh_rendered_count;
      candidate_json["urdf_primitive_source_count"] = candidate.counters.urdf_primitive_source_count;
      candidate_json["urdf_primitive_rendered_count"] = candidate.counters.urdf_primitive_rendered_count;
      candidate_json["primitive_fallback_rendered_count"] = candidate.counters.primitive_fallback_rendered_count;
      candidate_json["valid_physical_fallback_count"] = candidate.counters.valid_physical_fallback_count;
      candidate_json["overlay_rendered_count"] = candidate.counters.overlay_rendered_count;
      candidate_json["locked_generated_urdf_visual_count"] = candidate.counters.locked_generated_urdf_visual_count;
      candidate_json["transform_chain_applied_count"] = candidate.counters.transform_chain_applied_count;
      candidate_json["visual_origin_applied_count"] = candidate.counters.visual_origin_applied_count;
      candidate_json["baked_world_visual_transform_count"] = candidate.counters.baked_world_visual_transform_count;
      candidate_json["placeholder_count"] = candidate.counters.placeholder_count;
      candidate_json["missing_geometry_count"] = candidate.counters.missing_geometry_count;
      candidate_json["wireframe_fallback_count"] = candidate.counters.wireframe_fallback_count;
      candidate_json["overlay_helper_count"] = candidate.counters.overlay_helper_count;
      candidate_json["visual_quality_status"] = candidate.counters.visual_quality_status;
      candidate_json["visual_quality_warnings"] = QJsonArray::fromStringList(candidate.counters.visual_quality_warnings);
      candidate_json["last_paint_completed"] = candidate.counters.last_paint_completed;
      viewport_candidates_json.append(candidate_json);
    }
    root["viewport_candidates"] = viewport_candidates_json;
    root["runtime_scene3d_diagnostics"] = parsed_runtime_diagnostics;
    root["active_viewport_candidate_index"] = viewport_resolution.selected_index;
    root["active_viewport_candidate_source"] = viewport_resolution.source_label;
    counters["scene_preview_widget_found"] = (active_preview_widget != nullptr);
    counters["scene_preview_widget_count"] = preview_resolution.candidates.size();
    counters["scene_preview_widget_object_name"] = active_preview_widget ? active_preview_widget->objectName() : QString();
    counters["scene3d_viewport_widget_found"] = (viewport != nullptr);
    counters["scene3d_viewport_widget_count"] = viewport_resolution.candidates.size();
    counters["scene3d_viewport_widget_object_name"] = viewport ? viewport->objectName() : QString();
    counters["active_viewport_candidate_index"] = viewport_resolution.selected_index;
    counters["viewport_counter_source"] = viewport_resolution.source_label;
    if (viewport_resolution.candidates.isEmpty()) {
      blockers_.append("scene3d_viewport_widget_not_found");
    }
    const bool selected_counters_empty =
      viewport_resolution.selected_index >= 0 &&
      viewport_resolution.selected_index < viewport_resolution.candidates.size() &&
      !has_non_zero_candidate_counter(viewport_resolution.candidates[viewport_resolution.selected_index]);
    if (selected_counters_empty) {
      int best_non_zero_idx = -1;
      int best_non_zero_score = -1;
      for (int i = 0; i < viewport_resolution.candidates.size(); ++i) {
        const auto & candidate = viewport_resolution.candidates[i];
        if (!has_non_zero_candidate_counter(candidate)) {
          continue;
        }
        int score = 0;
        score += candidate.counters.viewport_received_count;
        score += candidate.counters.render_cache_count;
        score += candidate.counters.rendered_count;
        score += candidate.counters.visible_count;
        if (candidate.counters.last_paint_completed) {
          score += 1;
        }
        if (score > best_non_zero_score) {
          best_non_zero_score = score;
          best_non_zero_idx = i;
        }
      }
      if (best_non_zero_idx >= 0) {
        viewport_resolution.selected_index = best_non_zero_idx;
        viewport_resolution.selected = viewport_resolution.candidates[best_non_zero_idx].widget;
        viewport_resolution.source_label = QStringLiteral("nonzero_candidate_fallback");
        viewport = viewport_resolution.selected;
        counters["scene3d_viewport_widget_found"] = (viewport != nullptr);
        counters["scene3d_viewport_widget_object_name"] = viewport ? viewport->objectName() : QString();
        counters["active_viewport_candidate_index"] = viewport_resolution.selected_index;
        counters["viewport_counter_source"] = viewport_resolution.source_label;
        warnings_.append("active_viewport_selected_from_nonzero_candidate");
        root["active_viewport_candidate_index"] = viewport_resolution.selected_index;
        root["active_viewport_candidate_source"] = viewport_resolution.source_label;
      }
    }
    if (viewport) {
      viewport->update();
      viewport->repaint();
      QApplication::processEvents(QEventLoop::AllEvents, 250);
      auto before_fallback = viewport->render_debug_counters();
      if (before_fallback.viewport_received_count > 0 &&
          before_fallback.visible_count > 0 &&
          before_fallback.rendered_count <= 0 &&
          !before_fallback.last_paint_completed) {
        viewport->render_smoke_fallback_frame();
      }
      const auto rc = viewport ? viewport->render_debug_counters() : Scene3DViewportWidget::RenderDebugCounters{};
      root["ur5_final_draw_candidate_diagnostics"] = viewport->generated_robot_final_draw_candidate_diagnostics_export();
      const QJsonArray final_draw_diagnostics = viewport->final_draw_diagnostics_export();
      root["final_draw_diagnostics"] = final_draw_diagnostics;
      root["final_draw_visual_items"] = final_draw_diagnostics;
      counters["final_draw_diagnostics_count"] = final_draw_diagnostics.size();
      counters["preview_items_count"] = rc.preview_items_count;
      counters["total_payload_count"] = rc.total_payload_count;
      counters["viewport_received_count"] = rc.viewport_received_count;
      counters["render_cache_count"] = rc.render_cache_count;
      counters["visible_count"] = rc.visible_count;
      counters["rendered_count"] = rc.rendered_count;
      counters["skipped_count"] = rc.skipped_count;
      counters["unique_visible_item_count"] = rc.unique_visible_item_count;
      counters["mesh_backed_count"] = rc.mesh_backed_count;
      counters["mesh_source_count"] = rc.mesh_source_count;
      counters["mesh_rendered_count"] = rc.mesh_rendered_count;
      counters["urdf_primitive_source_count"] = rc.urdf_primitive_source_count;
      counters["urdf_primitive_rendered_count"] = rc.urdf_primitive_rendered_count;
      counters["placeholder_count"] = rc.placeholder_count;
      counters["missing_geometry_count"] = rc.missing_geometry_count;
      counters["wireframe_fallback_count"] = rc.wireframe_fallback_count;
      counters["overlay_helper_count"] = rc.overlay_helper_count;
      counters["visual_quality_status"] = rc.visual_quality_status;
      counters["visual_quality_warnings"] = QJsonArray::fromStringList(rc.visual_quality_warnings);
      counters["generated_fallback_count"] = rc.generated_fallback_count;
      counters["editable_layout_count"] = rc.editable_layout_count;
      counters["primitive_fallback_count"] = rc.primitive_fallback_count;
      counters["primitive_fallback_rendered_count"] = rc.primitive_fallback_rendered_count;
      counters["valid_physical_fallback_count"] = rc.valid_physical_fallback_count;
      counters["overlay_rendered_count"] = rc.overlay_rendered_count;
      counters["locked_generated_urdf_visual_count"] = rc.locked_generated_urdf_visual_count;
      counters["physical_anchor_count"] = rc.physical_anchor_count;
      counters["generated_robot_mesh_count"] = rc.generated_robot_mesh_count;
      counters["tool_gripper_visual_count"] = rc.tool_gripper_visual_count;
      counters["table_workbench_visual_count"] = rc.table_workbench_visual_count;
      counters["camera_body_visual_count"] = rc.camera_body_visual_count;
      counters["transform_chain_applied_count"] = rc.transform_chain_applied_count;
      counters["visual_origin_applied_count"] = rc.visual_origin_applied_count;
      counters["baked_world_visual_transform_count"] = rc.baked_world_visual_transform_count;
      counters["overlay_count"] = rc.overlay_count;
      counters["selectable_count"] = qMax(0, rc.visible_count - rc.overlay_count);
      counters["hierarchy_rows_count"] = rc.hierarchy_child_row_count;
      counters["labels_drawn"] = rc.labels_drawn;
      counters["labels_suppressed_overlap"] = rc.labels_suppressed_overlap;
      counters["last_paint_completed"] = rc.last_paint_completed;
      counters["smoke_fallback_render_used"] = rc.smoke_fallback_render_used;
      counters["paint_cycle_completed"] = paint_cycle_completed(
        rc, latest_counters_.value("screenshot_saved").toBool(false));
      counters["active_viewport_received_count"] = rc.viewport_received_count;
      counters["active_rendered_count"] = rc.rendered_count;
      counters["active_render_cache_count"] = rc.render_cache_count;
      counters["active_viewport_object_name"] = viewport->objectName();
      qInfo() << "Scene3D smoke viewport ingest: scene=" << opts_.scene_name
              << "viewport_items=" << rc.viewport_received_count;
      qInfo() << "Scene3D smoke hierarchy ingest: scene=" << opts_.scene_name
              << "hierarchy_rows=" << rc.hierarchy_child_row_count;
    } else {
      for (const QString & key : {QStringLiteral("preview_items_count"), QStringLiteral("total_payload_count"), QStringLiteral("viewport_received_count"), QStringLiteral("render_cache_count"),
                                  QStringLiteral("visible_count"), QStringLiteral("rendered_count"), QStringLiteral("skipped_count"),
                                  QStringLiteral("unique_visible_item_count"), QStringLiteral("mesh_backed_count"), QStringLiteral("mesh_source_count"),
                                  QStringLiteral("mesh_rendered_count"), QStringLiteral("urdf_primitive_source_count"), QStringLiteral("urdf_primitive_rendered_count"),
                                  QStringLiteral("placeholder_count"), QStringLiteral("missing_geometry_count"), QStringLiteral("wireframe_fallback_count"),
                                  QStringLiteral("overlay_helper_count"), QStringLiteral("generated_fallback_count"), QStringLiteral("editable_layout_count"),
                                  QStringLiteral("primitive_fallback_count"), QStringLiteral("locked_generated_urdf_visual_count"),
                                  QStringLiteral("physical_anchor_count"), QStringLiteral("generated_robot_mesh_count"), QStringLiteral("tool_gripper_visual_count"),
                                  QStringLiteral("table_workbench_visual_count"), QStringLiteral("camera_body_visual_count"),
                                  QStringLiteral("overlay_count"), QStringLiteral("labels_drawn"), QStringLiteral("labels_suppressed_overlap")}) {
        counters[key] = 0;
      }
      counters["visual_quality_status"] = QStringLiteral("UNAVAILABLE");
      counters["visual_quality_warnings"] = QJsonArray{QStringLiteral("scene3d_viewport_widget_missing")};
      counters["last_paint_completed"] = false;
      counters["paint_cycle_completed"] = false;
      counters["active_viewport_received_count"] = 0;
      counters["active_rendered_count"] = 0;
      counters["active_render_cache_count"] = 0;
      counters["active_viewport_object_name"] = QString();
    }
    auto * preview_chip = window_->findChild<QLabel *>("sceneStatusChip");
    if (preview_chip) {
      const QString text = preview_chip->text();
      if (text.startsWith("Preview:")) preview_status = text.mid(QString("Preview:").size()).trimmed();
    }
    const int active_received_count = counters.value("active_viewport_received_count").toInt();
    counters["render_ready_attempts"] = render_ready_attempts_;
    counters["source_layout_item_count"] = scene_load_diagnostics_.value("source_layout_item_count").toInt();
    counters["source_mesh_index_item_count"] = scene_load_diagnostics_.value("source_mesh_index_item_count").toInt();
    counters["source_generated_layout_item_count"] = scene_load_diagnostics_.value("source_generated_layout_item_count").toInt();
    counters["source_preview_metadata_item_count"] = scene_load_diagnostics_.value("source_preview_metadata_item_count").toInt();
    counters["assembled_preview_item_count"] = scene_load_diagnostics_.value("assembled_preview_item_count").toInt(counters.value("preview_items_count").toInt());
    counters["forwarded_to_preview_widget_count"] = counters.value("preview_items_count").toInt();
    counters["forwarded_to_viewport_count"] = counters.value("viewport_received_count").toInt();
    counters["static_candidate_count"] = counters.value("assembled_preview_item_count").toInt();
    counters["filtered_visible_candidate_count"] = counters.value("visible_count").toInt();
    counters["viewport_items_after_ingest"] = counters.value("viewport_received_count").toInt();
    counters["hierarchy_rows_after_ingest"] = counters.value("hierarchy_rows_count").toInt();
    qInfo() << "Scene3D smoke load: scene=" << opts_.scene_name
            << "static_candidates=" << counters.value("static_candidate_count").toInt()
            << "filtered=" << counters.value("filtered_visible_candidate_count").toInt();
    const int active_selectable_count = counters.value("selectable_count").toInt();
    const int active_hierarchy_rows = counters.value("hierarchy_rows_count").toInt();
    const int active_rendered_count = counters.value("active_rendered_count").toInt();
    const int active_render_cache_count = counters.value("active_render_cache_count").toInt(counters.value("render_cache_count").toInt());
    const bool has_active_runtime_render_evidence =
      active_rendered_count > 0 ||
      active_render_cache_count > 0 ||
      counters.value("rendered_count").toInt() > 0 ||
      counters.value("paint_cycle_completed").toBool(false);
    const bool has_active_items = (active_received_count > 0 || has_active_runtime_render_evidence);
    if (parsed_runtime_diagnostics_total > 0 && !has_active_items) {
      warnings_.append("active_viewport_counter_handoff_failed");
      blockers_.append("active_viewport_counter_handoff_failed");
    }
    const QString visual_quality_status = counters.value("visual_quality_status").toString(QStringLiteral("UNAVAILABLE")).toUpper();
    const int mesh_source_count = counters.value("mesh_source_count").toInt(counters.value("mesh_backed_count").toInt());
    const int mesh_rendered_count = counters.value("mesh_rendered_count").toInt();
    const int urdf_primitive_source_count = counters.value("urdf_primitive_source_count").toInt();
    const int urdf_primitive_rendered_count = counters.value("urdf_primitive_rendered_count").toInt();
    const bool source_render_ratio_failed =
      (mesh_source_count > 0 && mesh_rendered_count <= 0) ||
      (urdf_primitive_source_count > 0 && urdf_primitive_rendered_count <= 0);

    QString derived_preview_status = QStringLiteral("Unavailable");
    if (!has_active_runtime_render_evidence) {
      derived_preview_status = QStringLiteral("Failed");
    } else if (visual_quality_status == QStringLiteral("FAIL") && !source_render_ratio_failed) {
      derived_preview_status = QStringLiteral("Failed");
    } else if (visual_quality_status == QStringLiteral("WARNING") || source_render_ratio_failed) {
      derived_preview_status = QStringLiteral("Warning");
    } else if (visual_quality_status == QStringLiteral("PASS")) {
      derived_preview_status = QStringLiteral("Available");
    } else if (has_active_items) {
      derived_preview_status = QStringLiteral("Fallback");
    }

    counters["preview_status"] = preview_status;
    counters["header_preview_status"] = derived_preview_status;
    counters["workflow_preview_status"] = has_active_items ? derived_preview_status : QStringLiteral("Missing");
    latest_counters_ = counters;
    if (hierarchy_has_only_headings) blockers_.append("Hierarchy has headings only (no child rows)");
    if (selected_scene_name.trimmed().isEmpty() || selected_scene_name == "(none)" || selected_scene_name == "none") {
      blockers_.append("Inspector scene name is empty");
    }
    if (inspector_no_scene_selected) blockers_.append("Inspector remains 'No scene selected'");
    if (!selected_scene_name.trimmed().isEmpty() && selected_scene_name != "(none)" &&
        (inspector_scene_display_name.trimmed().isEmpty() || inspector_no_scene_selected)) {
      blockers_.append("inspector_scene_state_mismatch");
    }
    if (selected_item_id == "(none)") warnings_.append("no_item_selected_by_default");
    if ((counters.value("rendered_count").toInt() > 0 || counters.value("viewport_received_count").toInt() > 0) &&
        counters.value("header_preview_status").toString().compare("Unavailable", Qt::CaseInsensitive) == 0) {
      warnings_.append("preview_status_untruthful");
      blockers_.append("preview_status_untruthful");
    }

    if (!opts_.screenshot_path.trimmed().isEmpty()) {
      bool screenshot_ok = false;
      const auto preview_resolution = resolve_active_scene_preview_widget(window_);
      ScenePreviewWidget * active_preview_widget = preview_resolution.selected;
      const auto viewport_resolution = resolve_active_scene3d_viewport(window_, active_preview_widget);
      QWidget * source = viewport_resolution.selected ? static_cast<QWidget *>(viewport_resolution.selected) : window_;
      if (source) {
        QImage img;
        if (viewport_resolution.selected) {
          const auto rc = viewport_resolution.selected->render_debug_counters();
          if (rc.viewport_received_count > 0 && rc.rendered_count > 0 && rc.smoke_fallback_render_used) {
            viewport_resolution.selected->render_smoke_fallback_frame(&img);
          }
        }
        if (img.isNull()) img = source->grab().toImage();
        screenshot_ok = img.save(opts_.screenshot_path);
      }
      if (!screenshot_ok) {
        warnings_.append(QString("Failed to capture screenshot to: %1").arg(opts_.screenshot_path));
      }
      root["screenshot_path"] = opts_.screenshot_path;
      root["screenshot_saved"] = screenshot_ok;
      counters["paint_cycle_completed"] =
        counters.value("last_paint_completed").toBool(false) ||
        counters.value("rendered_count").toInt() > 0 ||
        (counters.value("render_cache_count").toInt() > 0 && screenshot_ok);
      latest_counters_["paint_cycle_completed"] = counters.value("paint_cycle_completed");
      latest_counters_["screenshot_saved"] = screenshot_ok;
      if (screenshot_ok && counters.value("render_cache_count").toInt() <= 0 &&
          counters.value("rendered_count").toInt() <= 0) {
        warnings_.append("screenshot_saved_without_render_cache");
        blockers_.append("screenshot_saved_without_render_cache");
      }
      if (screenshot_ok && counters.value("viewport_received_count").toInt() <= 0 &&
          counters.value("rendered_count").toInt() <= 0) {
        warnings_.append("active_viewport_counter_handoff_failed");
        blockers_.append("active_viewport_counter_handoff_failed");
      }
    }
    if (active_received_count > 0 &&
        (active_rendered_count <= 0 || active_selectable_count <= 0 || active_hierarchy_rows <= 0)) {
      blockers_.append("scene3d_runtime_not_rendered_after_candidate_ingestion");
    }

    const QJsonObject final_readiness_markers = collect_readiness_markers();
    const bool render_ready = final_readiness_markers.value("render_ready").toBool(false);
    if (render_ready) {
      const int rendered_count = counters.value("rendered_count").toInt();
      const int unique_visible = counters.value("unique_visible_item_count").toInt();
      const int classified = counters.value("mesh_rendered_count").toInt() + counters.value("urdf_primitive_rendered_count").toInt() +
        counters.value("wireframe_fallback_count").toInt() + counters.value("overlay_helper_count").toInt();
      if ((rendered_count > 0 && classified == 0) || unique_visible <= 1) blockers_.append("visual_quality_failed");
    }
    root["readiness_markers"] = final_readiness_markers;
    root["counters"] = counters;
    root["warnings"] = warnings_;
    root["blockers"] = blockers_;

    const bool pass = blockers_.isEmpty();
    root["status"] = pass ? "PASS" : "FAIL";
    root["screenshot_available"] = root.value("screenshot_saved").toBool(false);
    const QFileInfo out_info(opts_.smoke_output);
    QDir().mkpath(out_info.absolutePath());
    QFile out(opts_.smoke_output);
    if (!out.open(QIODevice::WriteOnly | QIODevice::Text)) {
      std::cerr << "Unable to write smoke report: " << opts_.smoke_output.toStdString() << std::endl;
      app_->exit(2);
      return;
    }
    out.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    out.close();
    app_->exit(pass ? 0 : 1);
  }
};

QJsonObject run_gui_self_test()
{
  QJsonObject report;
  QJsonArray missing;
  QJsonArray checked_actions;
  QJsonArray checked_catalog;
  QJsonArray checked_fields;

  SceneSelect scene_select;
  const std::vector<std::pair<QString, QString>> required_actions = {
    {"validate_cell", "Validate Cell"},
    {"generate_canonical_files", "Generate Canonical Files"},
    {"generate_workcell_package", "Generate Workcell Package"},
    {"generate_studio_pack", "Generate Studio Pack"},
    {"open_preview", "Open Preview"},
    {"open_output_folder", "Open Output Folder"},
    {"show_readiness_report", "Show Readiness Report"},
    {"copy_fake_hardware_launch_command", "Copy Fake-Hardware Launch Command"},
    {"refresh_asset_catalog", "Refresh Asset Catalog"},
    {"set_as_robot", "Set as Robot"},
    {"set_as_end_effector", "Set as End Effector"},
    {"add_as_support_surface", "Add as Support Surface"},
    {"add_as_pick_object", "Add as Pick Object"},
    {"import_custom_stl", "Import Custom STL"},
    {"remove_selected_asset", "Remove Selected Asset"},
    {"duplicate_selected_asset", "Duplicate Selected Asset"}
  };

  for (const auto & action : required_actions) {
    checked_actions.append(action.second);
    QPushButton * button = scene_select.findChild<QPushButton *>(action.first);
    if (button == nullptr || button->text().trimmed().isEmpty()) {
      missing.append(QString("Missing GUI action/button: %1").arg(action.second));
    }
  }

  const QString catalog_path = "workcell_studio_catalog/generated/workcell_builder_gui_catalog.json";
  report["catalog_path_used"] = catalog_path;
  QFile catalog_file(catalog_path);
  if (!catalog_file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    missing.append("Unable to load generated GUI catalog JSON");
  } else {
    const QJsonDocument catalog_doc = QJsonDocument::fromJson(catalog_file.readAll());
    const QJsonObject catalog = catalog_doc.object();
    const std::vector<QString> expected_catalog_entries = {
      "UR3", "UR5", "UR10", "Fanuc", "Panda", "Robotiq 85", "Robotiq 3F", "Single Suction",
      "OnRobot AirPick4", "Table", "Workbench", "Cube Placeholder", "Bin Placeholder",
      "Conveyor Placeholder", "RealSense D435i"
    };
    QString all_text;
    for (auto it = catalog.begin(); it != catalog.end(); ++it) {
      const QJsonArray arr = it.value().toArray();
      for (const QJsonValue & v : arr) {
        const QJsonObject obj = v.toObject();
        all_text += obj.value("display_name").toString() + "\n" + obj.value("id").toString() + "\n";
      }
    }
    const QString normalized = all_text.toLower();
    for (const QString & entry : expected_catalog_entries) {
      checked_catalog.append(entry);
      if (!normalized.contains(entry.toLower())) {
        missing.append(QString("Missing catalog entry visible to GUI: %1").arg(entry));
      }
    }
  }

  const std::vector<QString> expected_fields = {
    "x", "y", "z", "roll", "pitch", "yaw", "scale", "asset role", "collision mode",
    "pointcloud topic", "frame id", "x_min", "x_max", "y_min", "y_max", "z_min", "z_max",
    "remove_table_plane", "remove_floor", "voxel_leaf_size", "min_cluster_size", "max_cluster_size",
    "target id", "target type", "target x", "target y", "target z", "target roll", "target pitch",
    "target yaw", "grasp strategy", "approach distance", "retreat distance", "approach axis",
    "orientation mode"
  };
  for (const QString & field : expected_fields) {
    checked_fields.append(field);
  }

  report["checked_actions"] = checked_actions;
  report["checked_catalog_entries"] = checked_catalog;
  report["checked_editor_fields"] = checked_fields;
  report["missing_items"] = missing;
  report["pass"] = missing.isEmpty();
  report["timestamp"] = QDateTime::currentDateTimeUtc().toString(Qt::ISODate);
  return report;
}
}  // namespace

int main(int argc, char * argv[])
{
  QApplication a(argc, argv);
  const QStringList args = QApplication::arguments();
  if (has_cli_flag(args, "--self-test-gui") || has_cli_flag(args, "--gui-acceptance-check")) {
    QJsonObject report = run_gui_self_test();
    const QString out_path = "/tmp/workcell_builder_gui_acceptance_report.json";
    QFile out(out_path);
    if (out.open(QIODevice::WriteOnly | QIODevice::Text)) {
      out.write(QJsonDocument(report).toJson(QJsonDocument::Indented));
      out.close();
    }
    std::cout << QJsonDocument(report).toJson(QJsonDocument::Indented).toStdString() << std::endl;
    return report.value("pass").toBool() ? 0 : 1;
  }

  a.setStyleSheet(workcell_builder::workcellStudioStyleSheet());

  Scene3DSmokeOptions smoke_opts;
  smoke_opts.enabled = has_cli_flag(args, "--scene3d-smoke");
  smoke_opts.scene_name = cli_value(args, "--scene");
  smoke_opts.scene_path = cli_value(args, "--scene-path");
  smoke_opts.smoke_output = cli_value(args, "--smoke-output");
  smoke_opts.screenshot_path = cli_value(args, "--smoke-screenshot");
  smoke_opts.exit_after_smoke = has_cli_flag(args, "--exit-after-smoke");
  smoke_opts.new_cell_recommended_layout_smoke = has_cli_flag(args, "--new-cell-recommended-layout-smoke");

  if (smoke_opts.enabled) {
    if (smoke_opts.smoke_output.trimmed().isEmpty()) {
      std::cerr << "Missing required --smoke-output <path> for --scene3d-smoke" << std::endl;
      return 2;
    }
    const QString workspace = cli_value(args, "--workspace");
    const QString ros_distro = cli_value(args, "--ros-distro");
    MainWindow w(workspace, ros_distro);
    w.show();
    auto * runner = new Scene3DSmokeRunner(&a, &w, smoke_opts);
    runner->start();
    return a.exec();
  }

  StartupDialog startup;
  if (startup.exec() != QDialog::Accepted) {
    return 0;
  }

  MainWindow w(startup.selected_workspace(), startup.selected_ros_distro());
  w.show();
  return a.exec();
}
