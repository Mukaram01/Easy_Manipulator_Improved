#pragma once

#include "scene_preview_widget.h"

#include <QWidget>

#ifdef WORKCELL_BUILDER_HAS_WEBENGINE

#include <QApplication>
#include <QBoxLayout>
#include <QDir>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QMessageBox>
#include <QPointer>
#include <QProcess>
#include <QProcessEnvironment>
#include <QPushButton>
#include <QRegularExpression>
#include <QSaveFile>
#include <QSizePolicy>
#include <QTimer>
#include <QUrl>
#include <QUrlQuery>
#include <QVariantMap>
#include <QWebEnginePage>
#include <QWebEngineView>

namespace workcell_builder
{
namespace embedded_web_edit_save_detail
{
constexpr const char * kPatchSchema = "workcell_studio_web_scene_edit_patch/v1";
constexpr const char * kPatchCreator = "static_web_viewer";
constexpr const char * kSceneSuffix = ".web_scene.json";

inline QString canonicalPath(const QString & path)
{
  const QFileInfo info(path);
  const QString canonical = info.canonicalFilePath();
  return QDir::cleanPath(canonical.isEmpty() ? info.absoluteFilePath() : canonical);
}

inline bool repoMarkersExist(const QString & root)
{
  const QDir dir(root);
  return QFileInfo::exists(dir.filePath(QStringLiteral("workcell_studio_web/viewer/index.html"))) &&
    QFileInfo::exists(dir.filePath(QStringLiteral("scripts/run_workcell_studio_web_edit_workflow.py"))) &&
    QFileInfo(dir.filePath(QStringLiteral("scenes"))).isDir();
}

inline QString findRepoRoot()
{
  const QString configured = QProcessEnvironment::systemEnvironment()
    .value(QStringLiteral("WORKCELL_STUDIO_REPO_ROOT")).trimmed();
  if (!configured.isEmpty() && repoMarkersExist(configured)) return canonicalPath(configured);
  return {};
}

inline QString sceneIdFromViewerUrl(const QUrl & url)
{
  const QString scene_path = QUrlQuery(url).queryItemValue(QStringLiteral("scene"), QUrl::FullyDecoded);
  const QString filename = QFileInfo(scene_path).fileName();
  if (!scene_path.startsWith(QStringLiteral("build/workcell_studio_web_scene/")) ||
      !filename.endsWith(QString::fromUtf8(kSceneSuffix))) return {};
  const QString scene_id = filename.left(filename.size() - static_cast<int>(qstrlen(kSceneSuffix)));
  static const QRegularExpression safe_id(QStringLiteral("^[A-Za-z0-9][A-Za-z0-9_-]*$"));
  return safe_id.match(scene_id).hasMatch() ? scene_id : QString();
}

inline QString resolveSceneDir(const QString & repo_root, const QString & scene_id)
{
  const QString candidate = QDir(repo_root).filePath(QStringLiteral("scenes/%1").arg(scene_id));
  const QFileInfo info(candidate);
  return info.exists() && info.isDir() ? canonicalPath(candidate) : QString();
}

class EmbeddedWebEditSaveController : public QObject
{
public:
  EmbeddedWebEditSaveController(
    ScenePreviewWidget * preview, QWebEngineView * view, QPushButton * save_button,
    QLabel * dirty_label)
  : QObject(preview), preview_(preview), view_(view), save_button_(save_button), dirty_label_(dirty_label)
  {
    installed_ = preview_ && view_ && save_button_;
    if (!installed_) return;
    configureControls();

    connect(preview_, &ScenePreviewWidget::embedded_authoring_save_requested,
      this, [this]() { requestSave(); });
    connect(preview_, &ScenePreviewWidget::post_save_product_view_refresh_finished,
      this, [this](int revision, quint64, quint64, bool success, const QString & detail) {
        if (!saved_reload_pending_ || revision != saved_reload_revision_) return;
        if (!success) {
          saved_reload_pending_ = false;
          saved_reload_revision_ = 0;
          busy_ = false;
          setStatus(QStringLiteral("Saved; Product View refresh failed—edits retained"), QStringLiteral("error"));
          logPhase(QStringLiteral("reload failed: %1; persisted YAML was not written again and browser edits were preserved").arg(detail));
          QMessageBox::critical(preview_, QStringLiteral("Save Product View Layout"),
            QStringLiteral("The authored YAML was saved and verified, but the canonical Product View could not be regenerated or loaded. "
              "The current browser edits were retained. Correct the Product View preparation error, then choose Save Layout again; the YAML will not be rewritten automatically.\n\n%1").arg(detail));
          pollEditorState();
          return;
        }
        saved_reload_pending_ = false;
        saved_reload_revision_ = 0;
        busy_ = false;
        setStatus(QStringLiteral("Saved and reloaded"), QStringLiteral("success"));
        logPhase(QStringLiteral("reload complete: matching regenerated Product View scene_ready"));
        restoreSelectionAfterReload();
      });
    connect(view_, &QWebEngineView::loadFinished, this, [this](bool) {
      last_polled_url_ = QUrl();
      QTimer::singleShot(250, this, [this]() { pollEditorState(); });
    });
    poll_timer_.setInterval(350);
    connect(&poll_timer_, &QTimer::timeout, this, [this]() { pollEditorState(); });
    poll_timer_.start();
    pollEditorState();
  }

  bool installed() const { return installed_; }

private:
  enum class WorkflowPhase { DryRun, Write };

  void restoreSelectionAfterReload()
  {
    const QString selected_id = selected_item_id_before_save_;
    selected_item_id_before_save_.clear();
    const QString encoded = QString::fromUtf8(
      QJsonDocument(QJsonArray{selected_id}).toJson(QJsonDocument::Compact)).mid(1).chopped(1);
    const QString script = QStringLiteral(
      "(() => { const api=window.__WORKCELL_EDITOR_API_V1__; if (!api) return false; "
      "api.selectItem(%1); return api.getState().selectedItemId === %1 && !api.getState().dirty; })()").arg(encoded);
    view_->page()->runJavaScript(script, [this](const QVariant & restored) {
      if (!restored.toBool()) setStatus(QStringLiteral("Saved; selection could not be restored"), QStringLiteral("warning"));
      pollEditorState();
    });
  }

  void configureControls()
  {
    save_button_->setToolTip(QStringLiteral(
      "Validate the current Web3D edit patch, create source-YAML backups, apply it atomically, regenerate and reload Product View. No robot motion is started."));
    save_button_->setEnabled(false);
  }

  void logPhase(const QString & phase) { emit preview_->studio_log_requested(QStringLiteral("Web3D Save Layout: %1").arg(phase)); }

  static QString boundedWorkflowOutput(QString output)
  {
    output = output.trimmed();
    constexpr int kLimit = 8000;
    if (output.size() <= kLimit) return output;
    return output.left(kLimit) + QStringLiteral("\n… subprocess output truncated …");
  }

  void logPatchSummary(const QJsonObject & patch)
  {
    const QJsonArray edits = patch.value(QStringLiteral("edits")).toArray();
    logPhase(QStringLiteral("patch summary: scene_id=%1 dirty_edit_count=%2")
      .arg(patch.value(QStringLiteral("scene_id")).toString()).arg(edits.size()));
    for (const QJsonValue & value : edits) {
      const QJsonObject edit = value.toObject();
      const auto transformText = [](const QJsonValue & value) {
        const QJsonObject transform = value.toObject();
        const QJsonObject pose = transform.value(QStringLiteral("pose")).toObject();
        const QJsonObject xyz = pose.value(QStringLiteral("xyz")).toObject();
        const QJsonObject rpy = pose.value(QStringLiteral("rpy")).toObject();
        return QStringLiteral("xyz=[%1,%2,%3] rpy=[%4,%5,%6]")
          .arg(xyz.value(QStringLiteral("x")).toDouble(), 0, 'g', 8)
          .arg(xyz.value(QStringLiteral("y")).toDouble(), 0, 'g', 8)
          .arg(xyz.value(QStringLiteral("z")).toDouble(), 0, 'g', 8)
          .arg(rpy.value(QStringLiteral("x")).toDouble(), 0, 'g', 8)
          .arg(rpy.value(QStringLiteral("y")).toDouble(), 0, 'g', 8)
          .arg(rpy.value(QStringLiteral("z")).toDouble(), 0, 'g', 8);
      };
      logPhase(QStringLiteral("patch edit: item_id=%1 operation=%2 old_%3 new_%4 persistence_source=%5")
        .arg(edit.value(QStringLiteral("item_id")).toString(),
          edit.value(QStringLiteral("operation")).toString(),
          transformText(edit.value(QStringLiteral("old_transform"))),
          transformText(edit.value(QStringLiteral("new_transform"))),
          edit.value(QStringLiteral("persistence_source")).toString()));
    }
  }

  void setStatus(const QString & text, const QString & level = QStringLiteral("info"))
  {
    if (!status_label_) return;
    status_label_->setText(text);
    status_label_->setVisible(!text.isEmpty());
    QString background = QStringLiteral("#e7f0fb");
    QString foreground = QStringLiteral("#2769b3");
    if (level == QStringLiteral("success")) {
      background = QStringLiteral("#e8f5eb"); foreground = QStringLiteral("#217a34");
    } else if (level == QStringLiteral("warning")) {
      background = QStringLiteral("#fff3e0"); foreground = QStringLiteral("#8a5200");
    } else if (level == QStringLiteral("error")) {
      background = QStringLiteral("#fdecea"); foreground = QStringLiteral("#b3261e");
    }
    status_label_->setStyleSheet(QStringLiteral(
      "background:%1;color:%2;border:1px solid #c7d3df;border-radius:8px;padding:2px 6px;font-weight:600;")
      .arg(background, foreground));
  }

  bool saveContextIsCurrent() const
  {
    if (!view_ || !preview_ || view_->url() != expected_url_ ||
        sceneIdFromViewerUrl(view_->url()) != scene_id_) return false;
    return saveTargetContextIsActive();
  }

  bool saveTargetContextIsActive() const
  {
    if (!view_ || !preview_ || sceneIdFromViewerUrl(view_->url()) != scene_id_) return false;
    const ScenePreviewWidget::PreviewContext current = preview_->preview_context();
    if (using_environment_fallback_) {
      return current.scene_id.trimmed().isEmpty() &&
        current.absolute_repo_root.trimmed().isEmpty() &&
        current.absolute_scene_dir.trimmed().isEmpty() && findRepoRoot() == repo_root_;
    }
    return current.scene_id.trimmed() == scene_id_ &&
      canonicalPath(current.absolute_repo_root.trimmed()) == repo_root_ &&
      canonicalPath(current.absolute_scene_dir.trimmed()) == scene_dir_;
  }

  void reportSceneChanged()
  {
    busy_ = false;
    if (save_button_) save_button_->setEnabled(false);
    setStatus(QStringLiteral("Scene changed—reload required; Web3D edits preserved"), QStringLiteral("warning"));
    logPhase(QStringLiteral("validation failed: viewer URL or PreviewContext changed; Web3D edits preserved"));
  }

  void reportSavedButSceneChanged()
  {
    saved_reload_pending_ = false;
    saved_reload_revision_ = 0;
    busy_ = false;
    if (save_button_) save_button_->setEnabled(false);
    setStatus(QStringLiteral("Saved; active scene changed before Product View refresh"), QStringLiteral("warning"));
    logPhase(QStringLiteral("saved; active scene changed before reload; persisted YAML remains authoritative"));
    QMessageBox::information(preview_, QStringLiteral("Save Product View Layout"),
      QStringLiteral("The authored YAML was saved and verified, but the active Product View changed before the post-save refresh could start. "
        "Reopen the saved scene to load the canonical result."));
  }

  bool resolveSaveContext(QString * error)
  {
    expected_url_ = view_ ? view_->url() : QUrl();
    const QString url_scene_id = sceneIdFromViewerUrl(expected_url_);
    if (url_scene_id.isEmpty()) {
      if (error) *error = QStringLiteral("The embedded viewer URL does not identify a safe active scene.");
      return false;
    }

    const ScenePreviewWidget::PreviewContext context = preview_ ?
      preview_->preview_context() : ScenePreviewWidget::PreviewContext{};
    using_environment_fallback_ = context.scene_id.trimmed().isEmpty() &&
      context.absolute_repo_root.trimmed().isEmpty() && context.absolute_scene_dir.trimmed().isEmpty();
    scene_id_ = context.scene_id.trimmed();
    repo_root_ = context.absolute_repo_root.trimmed().isEmpty() ?
      QString() : canonicalPath(context.absolute_repo_root.trimmed());
    scene_dir_ = context.absolute_scene_dir.trimmed().isEmpty() ?
      QString() : canonicalPath(context.absolute_scene_dir.trimmed());

    // The environment is retained only for older callers that have not yet
    // supplied a PreviewContext. Never infer a repository from the process CWD.
    if (context.absolute_repo_root.trimmed().isEmpty()) repo_root_ = findRepoRoot();
    if (scene_id_.isEmpty()) scene_id_ = url_scene_id;
    if (context.absolute_scene_dir.trimmed().isEmpty() && !repo_root_.isEmpty()) {
      scene_dir_ = resolveSceneDir(repo_root_, scene_id_);
    }

    if (scene_id_ != url_scene_id) {
      if (error) *error = QStringLiteral(
        "Active Product View scene '%1' does not match PreviewContext scene '%2'. Web3D edits were preserved; reload Product View before saving.")
        .arg(url_scene_id, scene_id_);
      return false;
    }
    const QString expected_scene_path = QStringLiteral("build/workcell_studio_web_scene/%1%2")
      .arg(scene_id_, QString::fromUtf8(kSceneSuffix));
    if (QUrlQuery(expected_url_).queryItemValue(QStringLiteral("scene"), QUrl::FullyDecoded) !=
        expected_scene_path) {
      if (error) *error = QStringLiteral("Active Product View URL does not match the PreviewContext scene output.");
      return false;
    }
    if (repo_root_.isEmpty()) {
      if (error) *error = QStringLiteral("Workcell Studio repository root could not be resolved.");
      return false;
    }
    if (!repoMarkersExist(repo_root_)) {
      if (error) *error = QStringLiteral("PreviewContext repository root is invalid: %1").arg(repo_root_);
      return false;
    }

    const QString scenes_root = canonicalPath(QDir(repo_root_).filePath(QStringLiteral("scenes")));
    const QFileInfo scene_info(scene_dir_);
    if (!scene_info.exists() || !scene_info.isDir() ||
        QFileInfo(scene_dir_).fileName() != scene_id_ ||
        QDir::cleanPath(QFileInfo(scene_dir_).absolutePath()) != scenes_root) {
      if (error) *error = QStringLiteral(
        "PreviewContext scene directory is invalid or outside the repository scenes directory: %1")
        .arg(scene_dir_);
      return false;
    }

    const QString context_key = QStringLiteral("%1|%2|%3").arg(repo_root_, scene_dir_, scene_id_);
    if (context_key != last_logged_context_key_) {
      last_logged_context_key_ = context_key;
      logPhase(QStringLiteral("resolved save context: repo_root=%1 scene_dir=%2 scene_id=%3")
        .arg(repo_root_, scene_dir_, scene_id_));
    }
    patch_path_ = QDir(repo_root_).filePath(
      QStringLiteral("build/workcell_studio_web_scene/%1.qt_edit_patch.json").arg(scene_id_));
    return true;
  }

  bool writePatchAtomically(const QJsonObject & patch, QString * error)
  {
    QDir().mkpath(QFileInfo(patch_path_).absolutePath());
    QSaveFile output(patch_path_);
    if (!output.open(QIODevice::WriteOnly)) {
      if (error) *error = QStringLiteral("Could not open patch output: %1").arg(output.errorString());
      return false;
    }
    if (output.write(QJsonDocument(patch).toJson(QJsonDocument::Indented)) < 0 || !output.commit()) {
      if (error) *error = QStringLiteral("Could not atomically write patch: %1").arg(output.errorString());
      return false;
    }
    return true;
  }

  void requestSave()
  {
    if (busy_ || !view_) return;
    logPhase(QStringLiteral("save requested"));
    QString context_error;
    if (!resolveSaveContext(&context_error)) {
      setStatus(QStringLiteral("Validation failed"), QStringLiteral("error"));
      logPhase(QStringLiteral("validation failed: save context is not current; Web3D edits preserved"));
      QMessageBox::warning(preview_, QStringLiteral("Save Product View Layout"), context_error);
      return;
    }

    busy_ = true;
    if (save_button_) save_button_->setEnabled(false);
    setStatus(QStringLiteral("Checking edits…"));
    logPhase(QStringLiteral("checking edits"));
    static const char kPatchScript[] = R"JS(
(() => {
  const api = window.__WORKCELL_EDITOR_API_V1__;
  if (!api) return {ok:false,error:'Embedded Product View editor is unavailable.'};
  return {ok:true,state:api.getState(),patch:api.getEditPatch()};
})()
)JS";
    view_->page()->runJavaScript(QString::fromUtf8(kPatchScript), [this](const QVariant & value) {
      if (!saveContextIsCurrent()) {
        reportSceneChanged();
        return;
      }
      const QVariantMap payload = value.toMap();
      if (!payload.value(QStringLiteral("ok")).toBool()) {
        busy_ = false;
        setStatus(QStringLiteral("Validation failed"), QStringLiteral("error"));
        QMessageBox::warning(preview_, QStringLiteral("Save Product View Layout"),
          payload.value(QStringLiteral("error")).toString());
        logPhase(QStringLiteral("validation failed: editor API unavailable; Web3D edits preserved"));
        return;
      }

      const QJsonDocument patch_doc = QJsonDocument::fromVariant(payload.value(QStringLiteral("patch")));
      if (!patch_doc.isObject()) {
        busy_ = false;
        setStatus(QStringLiteral("Validation failed"), QStringLiteral("error"));
        logPhase(QStringLiteral("validation failed: patch is not an object; Web3D edits preserved"));
        return;
      }
      const QJsonObject patch = patch_doc.object();
      const QVariantMap editor_state = payload.value(QStringLiteral("state")).toMap();
      const QJsonArray edits = patch.value(QStringLiteral("edits")).toArray();
      if (patch.value(QStringLiteral("schema_version")).toString() != QString::fromUtf8(kPatchSchema) ||
          patch.value(QStringLiteral("created_by")).toString() != QString::fromUtf8(kPatchCreator) ||
          patch.value(QStringLiteral("scene_id")).toString() != scene_id_ ||
          !patch.value(QStringLiteral("edits")).isArray()) {
        busy_ = false;
        setStatus(QStringLiteral("Validation failed"), QStringLiteral("error"));
        QMessageBox::warning(preview_, QStringLiteral("Save Product View Layout"),
          QStringLiteral("The browser returned an invalid or stale edit-patch contract."));
        logPhase(QStringLiteral("validation failed: invalid or stale patch; Web3D edits preserved"));
        return;
      }
      if (edits.isEmpty()) {
        busy_ = false;
        setStatus(QStringLiteral("No changes"), QStringLiteral("info"));
        pollEditorState();
        return;
      }
      selected_item_id_before_save_ = editor_state.value(QStringLiteral("selectedItemId")).toString();

      logPatchSummary(patch);

      QString write_error;
      if (!writePatchAtomically(patch, &write_error)) {
        busy_ = false;
        setStatus(QStringLiteral("Validation failed"), QStringLiteral("error"));
        QMessageBox::critical(preview_, QStringLiteral("Save Product View Layout"), write_error);
        logPhase(QStringLiteral("validation failed: patch staging write failed; Web3D edits preserved"));
        return;
      }
      logPhase(QStringLiteral("validation started"));
      startWorkflow(WorkflowPhase::DryRun);
    });
  }

  void startWorkflow(WorkflowPhase phase)
  {
    if (!saveContextIsCurrent()) {
      reportSceneChanged();
      return;
    }
    if (process_) process_->deleteLater();
    process_ = new QProcess(this);
    process_->setProgram(QStringLiteral("python3"));
    QStringList arguments{
      QStringLiteral("scripts/run_workcell_studio_web_edit_workflow.py"),
      QStringLiteral("--scene"), scene_dir_,
      QStringLiteral("--patch"), patch_path_,
      QStringLiteral("--output-dir"), QStringLiteral("build/workcell_studio_web_scene")};
    if (phase == WorkflowPhase::DryRun) {
      arguments << QStringLiteral("--dry-run-apply");
      setStatus(QStringLiteral("Validating…"));
    } else {
      arguments << QStringLiteral("--write");
      setStatus(QStringLiteral("Saving…"));
      logPhase(QStringLiteral("saving"));
    }
    process_->setArguments(arguments);
    process_->setWorkingDirectory(repo_root_);
    process_->setProcessChannelMode(QProcess::MergedChannels);
    QProcess * const process = process_;
    connect(process, &QProcess::errorOccurred, this, [this, process](QProcess::ProcessError) {
      if (process != process_) return;
      busy_ = false;
      setStatus(QStringLiteral("Validation failed"), QStringLiteral("error"));
      QMessageBox::critical(preview_, QStringLiteral("Save Product View Layout"),
        QStringLiteral("Could not start the save workflow for %1: %2. The Web3D edit remains unsaved.")
          .arg(scene_id_, process->errorString()));
      logPhase(QStringLiteral("validation failed: workflow could not start; Web3D edits preserved"));
      pollEditorState();
    });
    connect(process, qOverload<int, QProcess::ExitStatus>(&QProcess::finished), this,
      [this, process, phase](int exit_code, QProcess::ExitStatus exit_status) {
        const QString output = boundedWorkflowOutput(QString::fromLocal8Bit(process->readAll()));
        if (process == process_) process_ = nullptr;
        process->deleteLater();
        const bool ok = exit_status == QProcess::NormalExit && exit_code == 0;
        if (phase == WorkflowPhase::DryRun && !saveContextIsCurrent()) {
          reportSceneChanged();
          return;
        }
        if (!ok) {
          busy_ = false;
          setStatus(phase == WorkflowPhase::DryRun ? QStringLiteral("Validation failed") : QStringLiteral("Save failed"),
            QStringLiteral("error"));
          QMessageBox::critical(preview_, QStringLiteral("Save Product View Layout"),
            QStringLiteral("%1\n\n%2")
              .arg(phase == WorkflowPhase::DryRun ? QStringLiteral("The edit patch did not pass validation.") :
                QStringLiteral("The edit patch could not be saved."), output));
          const QString phase_name = phase == WorkflowPhase::DryRun ? QStringLiteral("validation") : QStringLiteral("write");
          logPhase(QStringLiteral("%1 failed: exit_code=%2 subprocess_output=%3; Web3D edits preserved")
            .arg(phase_name).arg(exit_code).arg(output.isEmpty() ? QStringLiteral("(empty)") : output));
          pollEditorState();
          return;
        }

        if (phase == WorkflowPhase::DryRun) {
          const QString confirmation = QStringLiteral(
            "Validation passed for %1. Apply these edits now?\n\n"
            "Qt will create timestamped backups, update only approved editable source YAML, verify persistence, "
            "regenerate the Web3D scene, refresh Product View, and reload it.\n\n"
            "This does not launch controllers, execute MoveIt plans, or move real hardware.").arg(scene_id_);
          if (QMessageBox::question(preview_, QStringLiteral("Confirm Save Product View Layout"), confirmation,
              QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes) {
            busy_ = false;
            setStatus(QStringLiteral("Save cancelled"), QStringLiteral("warning"));
            pollEditorState();
            return;
          }
          startWorkflow(WorkflowPhase::Write);
          return;
        }

        if (!saveTargetContextIsActive()) {
          reportSavedButSceneChanged();
          return;
        }
        setStatus(QStringLiteral("Saved; refreshing Product View…"), QStringLiteral("success"));
        logPhase(QStringLiteral("saved"));
        saved_reload_pending_ = true;
        saved_reload_revision_ = preview_->request_post_save_product_view_refresh();
        if (saved_reload_revision_ <= 0) {
          saved_reload_pending_ = false;
          busy_ = false;
          setStatus(QStringLiteral("Saved; Product View refresh could not start"), QStringLiteral("error"));
          logPhase(QStringLiteral("reload failed: Product View lifecycle context unavailable; browser edits preserved"));
          pollEditorState();
          return;
        }
        logPhase(QStringLiteral("reload: forced canonical Product View regeneration requested for payload_revision=%1")
          .arg(saved_reload_revision_));
      });
    process_->start();
  }

  void pollEditorState()
  {
    if (!installed_ || !view_ || busy_ || state_poll_pending_) return;
    if (!preview_->embedded_web_authoring_active()) {
      if (save_button_) save_button_->setEnabled(true);
      return;
    }
    const QUrl poll_url = view_->url();
    if (sceneIdFromViewerUrl(poll_url).isEmpty()) {
      if (save_button_) save_button_->setEnabled(false);
      return;
    }
    state_poll_pending_ = true;
    static const char kStateScript[] = R"JS(
(() => {
  const api = window.__WORKCELL_EDITOR_API_V1__;
  if (!api) return {ready:false,dirty:false,dirtyCount:0,sceneId:'',validDirtyTransforms:false};
  const state = api.getState();
  const patch = api.getEditPatch();
  const edits = Array.isArray(patch?.edits) ? patch.edits : [];
  const finiteTransform = transform => ['x','y','z'].every(axis =>
    Number.isFinite(Number(transform?.pose?.xyz?.[axis])) &&
    Number.isFinite(Number(transform?.pose?.rpy?.[axis])) &&
    Number.isFinite(Number(transform?.scale?.[axis])));
  const validDirtyTransforms = Boolean(state.dirty) && edits.length > 0 &&
    edits.length === Number(state.dirtyCount || 0) && edits.every(edit =>
      typeof edit?.item_id === 'string' && edit.item_id.length > 0 &&
      finiteTransform(edit.old_transform) && finiteTransform(edit.new_transform));
  return {ready:Boolean(state.ready),dirty:Boolean(state.dirty),dirtyCount:Number(state.dirtyCount||0),sceneId:String(state.sceneId||''),validDirtyTransforms};
})()
)JS";
    view_->page()->runJavaScript(QString::fromUtf8(kStateScript), [this, poll_url](const QVariant & value) {
      state_poll_pending_ = false;
      if (!view_ || view_->url() != poll_url || busy_) return;
      const QVariantMap state = value.toMap();
      const bool ready = state.value(QStringLiteral("ready")).toBool();
      const bool dirty = state.value(QStringLiteral("dirty")).toBool();
      const bool valid_dirty_transforms = state.value(QStringLiteral("validDirtyTransforms")).toBool();
      const QString reported_scene = state.value(QStringLiteral("sceneId")).toString();
      const QString url_scene = sceneIdFromViewerUrl(poll_url);
      const bool matching_scene = !url_scene.isEmpty() && reported_scene == url_scene;
      const bool diagnostic_preview = preview_ && preview_->property("diagnostic_preview_active").toBool();
      if (save_button_) {
        save_button_->setEnabled(ready && dirty && valid_dirty_transforms && matching_scene && !diagnostic_preview);
        if (diagnostic_preview) save_button_->setToolTip(QStringLiteral(
          "Save Layout is disabled for a diagnostic preview. Fix scene authoring blockers first."));
      }
      if (dirty_label_) {
        const int dirty_count = state.value(QStringLiteral("dirtyCount")).toInt();
        dirty_label_->setText(dirty && matching_scene ?
          QStringLiteral("Unsaved Layout Edits: %1 (Web3D)").arg(dirty_count) :
          QStringLiteral("Unsaved Layout Edits: none"));
      }
      last_polled_url_ = poll_url;
    });
  }

  QPointer<ScenePreviewWidget> preview_;
  QPointer<QWebEngineView> view_;
  QPointer<QPushButton> save_button_;
  QPointer<QLabel> dirty_label_;
  QPointer<QLabel> status_label_;
  QPointer<QProcess> process_;
  QTimer poll_timer_{this};
  QUrl expected_url_;
  QUrl last_polled_url_;
  QString scene_id_;
  QString repo_root_;
  QString scene_dir_;
  QString patch_path_;
  QString last_logged_context_key_;
  bool installed_{false};
  bool busy_{false};
  bool state_poll_pending_{false};
  bool saved_reload_pending_{false};
  int saved_reload_revision_{0};
  bool using_environment_fallback_{false};
  QString selected_item_id_before_save_;
};
}  // namespace embedded_web_edit_save_detail

inline void installEmbeddedWebEditSaveController(
  ScenePreviewWidget * preview, QPushButton * save_button, QLabel * dirty_label)
{
  if (!preview || preview->property("workcell_embedded_save_controller").toBool()) return;
  QWebEngineView * view = preview->findChild<QWebEngineView *>(QStringLiteral("embeddedWeb3dProductView"));
  if (!view) return;
  auto * controller = new embedded_web_edit_save_detail::EmbeddedWebEditSaveController(
    preview, view, save_button, dirty_label);
  if (controller->installed()) preview->setProperty("workcell_embedded_save_controller", true);
  else controller->deleteLater();
}
}  // namespace workcell_builder

#else

namespace workcell_builder
{
inline void installEmbeddedWebEditSaveController(ScenePreviewWidget *, QPushButton *, QLabel *) {}
}  // namespace workcell_builder

#endif
