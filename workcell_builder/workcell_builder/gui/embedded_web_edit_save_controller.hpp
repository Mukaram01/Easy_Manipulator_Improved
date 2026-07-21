#pragma once

#include <QWidget>

#ifdef WORKCELL_BUILDER_HAS_WEBENGINE

#include <QApplication>
#include <QBoxLayout>
#include <QCoreApplication>
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
  const auto walk = [](const QString & start) {
    QDir dir(canonicalPath(start));
    for (int depth = 0; depth < 16; ++depth) {
      if (repoMarkersExist(dir.absolutePath())) return canonicalPath(dir.absolutePath());
      if (!dir.cdUp()) break;
    }
    return QString();
  };

  const QString configured = QProcessEnvironment::systemEnvironment()
    .value(QStringLiteral("WORKCELL_STUDIO_REPO_ROOT")).trimmed();
  if (!configured.isEmpty() && repoMarkersExist(configured)) return canonicalPath(configured);

  for (const QString & candidate : {QDir::currentPath(), QCoreApplication::applicationDirPath()}) {
    const QString root = walk(candidate);
    if (!root.isEmpty()) return root;
  }
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
  const QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
  QStringList candidates;
  for (const QString & key : {QStringLiteral("WORKCELL_STUDIO_SELECTED_SCENE_DIR"),
      QStringLiteral("WORKCELL_STUDIO_SCENE_DIR")}) {
    const QString exact = env.value(key).trimmed();
    if (!exact.isEmpty()) candidates << exact;
  }
  const QString scenes_root = env.value(QStringLiteral("WORKCELL_STUDIO_SCENES_PATH")).trimmed();
  if (!scenes_root.isEmpty()) candidates << QDir(scenes_root).filePath(scene_id);
  candidates << QDir(repo_root).filePath(QStringLiteral("scenes/%1").arg(scene_id));

  for (const QString & candidate : candidates) {
    const QFileInfo info(candidate);
    if (!info.exists() || !info.isDir()) continue;
    const QString resolved = canonicalPath(candidate);
    if (QFileInfo(resolved).fileName() == scene_id) return resolved;
  }
  return {};
}

class EmbeddedWebEditSaveController : public QObject
{
public:
  EmbeddedWebEditSaveController(QWidget * preview, QWebEngineView * view)
  : QObject(preview), preview_(preview), view_(view)
  {
    installed_ = createControls();
    if (!installed_) return;

    connect(save_button_, &QPushButton::clicked, this, [this]() { requestSave(); });
    connect(view_, &QWebEngineView::loadFinished, this, [this](bool) {
      last_polled_url_ = {};
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

  bool createControls()
  {
    if (!preview_ || !view_) return false;
    QPushButton * fit_button = nullptr;
    for (QPushButton * button : preview_->findChildren<QPushButton *>()) {
      if (button->text() == QStringLiteral("Fit")) {
        fit_button = button;
        break;
      }
    }
    if (!fit_button) return false;

    QBoxLayout * toolbar = nullptr;
    for (QBoxLayout * layout : preview_->findChildren<QBoxLayout *>()) {
      if (layout->indexOf(fit_button) >= 0) {
        toolbar = layout;
        break;
      }
    }
    if (!toolbar) return false;

    save_button_ = new QPushButton(QStringLiteral("Save layout"), preview_);
    save_button_->setObjectName(QStringLiteral("embeddedSaveLayoutButton"));
    save_button_->setProperty("class", "safe_action");
    save_button_->setToolTip(QStringLiteral(
      "Validate the current Web3D edit patch, create source-YAML backups, apply it through Qt, regenerate, validate and reload Product View. No robot motion is started."));
    save_button_->setEnabled(false);
    save_button_->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);

    status_label_ = new QLabel(preview_);
    status_label_->setObjectName(QStringLiteral("embeddedSaveLayoutStatus"));
    status_label_->setWordWrap(true);
    status_label_->setMinimumWidth(0);
    status_label_->setMaximumWidth(190);
    status_label_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    status_label_->setVisible(false);

    const int fit_index = toolbar->indexOf(fit_button);
    toolbar->insertWidget(fit_index + 1, save_button_);
    toolbar->insertWidget(fit_index + 2, status_label_);
    return true;
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
    return view_ && view_->url() == expected_url_ &&
      sceneIdFromViewerUrl(view_->url()) == scene_id_;
  }

  void reportSceneChanged()
  {
    busy_ = false;
    if (save_button_) save_button_->setEnabled(false);
    setStatus(QStringLiteral("Scene changed—reload required"), QStringLiteral("warning"));
  }

  bool resolveSaveContext(QString * error)
  {
    expected_url_ = view_ ? view_->url() : QUrl();
    scene_id_ = sceneIdFromViewerUrl(expected_url_);
    if (scene_id_.isEmpty()) {
      if (error) *error = QStringLiteral("The embedded viewer URL does not identify a safe active scene.");
      return false;
    }
    repo_root_ = findRepoRoot();
    if (repo_root_.isEmpty()) {
      if (error) *error = QStringLiteral("Workcell Studio repository root could not be resolved.");
      return false;
    }
    scene_dir_ = resolveSceneDir(repo_root_, scene_id_);
    if (scene_dir_.isEmpty()) {
      if (error) *error = QStringLiteral("Selected scene directory could not be resolved for %1.").arg(scene_id_);
      return false;
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
    QString context_error;
    if (!resolveSaveContext(&context_error)) {
      setStatus(QStringLiteral("Validation failed"), QStringLiteral("error"));
      QMessageBox::warning(preview_, QStringLiteral("Save Product View Layout"), context_error);
      return;
    }

    busy_ = true;
    if (save_button_) save_button_->setEnabled(false);
    setStatus(QStringLiteral("Checking edits…"));
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
        return;
      }

      const QJsonDocument patch_doc = QJsonDocument::fromVariant(payload.value(QStringLiteral("patch")));
      if (!patch_doc.isObject()) {
        busy_ = false;
        setStatus(QStringLiteral("Validation failed"), QStringLiteral("error"));
        return;
      }
      const QJsonObject patch = patch_doc.object();
      const QJsonArray edits = patch.value(QStringLiteral("edits")).toArray();
      if (patch.value(QStringLiteral("schema_version")).toString() != QString::fromUtf8(kPatchSchema) ||
          patch.value(QStringLiteral("created_by")).toString() != QString::fromUtf8(kPatchCreator) ||
          patch.value(QStringLiteral("scene_id")).toString() != scene_id_ ||
          !patch.value(QStringLiteral("edits")).isArray()) {
        busy_ = false;
        setStatus(QStringLiteral("Validation failed"), QStringLiteral("error"));
        QMessageBox::warning(preview_, QStringLiteral("Save Product View Layout"),
          QStringLiteral("The browser returned an invalid or stale edit-patch contract."));
        return;
      }
      if (edits.isEmpty()) {
        busy_ = false;
        setStatus(QStringLiteral("No changes"), QStringLiteral("info"));
        pollEditorState();
        return;
      }

      QString write_error;
      if (!writePatchAtomically(patch, &write_error)) {
        busy_ = false;
        setStatus(QStringLiteral("Validation failed"), QStringLiteral("error"));
        QMessageBox::critical(preview_, QStringLiteral("Save Product View Layout"), write_error);
        return;
      }
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
      arguments << QStringLiteral("--write") << QStringLiteral("--generate-and-validate");
      setStatus(QStringLiteral("Saving…"));
    }
    process_->setArguments(arguments);
    process_->setWorkingDirectory(repo_root_);
    process_->setProcessChannelMode(QProcess::MergedChannels);
    QProcess * const process = process_;
    connect(process, &QProcess::errorOccurred, this, [this, process](QProcess::ProcessError) {
      if (process != process_) return;
      busy_ = false;
      setStatus(QStringLiteral("Validation failed"), QStringLiteral("error"));
    });
    connect(process, qOverload<int, QProcess::ExitStatus>(&QProcess::finished), this,
      [this, process, phase](int exit_code, QProcess::ExitStatus exit_status) {
        const QString output = QString::fromLocal8Bit(process->readAll());
        if (process == process_) process_ = nullptr;
        process->deleteLater();
        if (!saveContextIsCurrent()) {
          reportSceneChanged();
          return;
        }
        const bool ok = exit_status == QProcess::NormalExit && exit_code == 0;
        if (!ok) {
          busy_ = false;
          setStatus(phase == WorkflowPhase::DryRun ? QStringLiteral("Validation failed") : QStringLiteral("Save failed"),
            QStringLiteral("error"));
          QMessageBox::critical(preview_, QStringLiteral("Save Product View Layout"),
            QStringLiteral("%1\n\n%2")
              .arg(phase == WorkflowPhase::DryRun ? QStringLiteral("The edit patch did not pass validation.") :
                QStringLiteral("The edit patch could not be saved."), output.left(8000)));
          pollEditorState();
          return;
        }

        if (phase == WorkflowPhase::DryRun) {
          const QString confirmation = QStringLiteral(
            "Validation passed for %1. Apply these edits now?\n\n"
            "Qt will create timestamped backups, update only approved editable source YAML, verify persistence, "
            "regenerate and validate the scene, refresh Product View, and reload it.\n\n"
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

        busy_ = false;
        setStatus(QStringLiteral("Saved"), QStringLiteral("success"));
        if (view_) view_->reload();
        QTimer::singleShot(600, this, [this]() { pollEditorState(); });
      });
    process_->start();
  }

  void pollEditorState()
  {
    if (!installed_ || !view_ || busy_ || state_poll_pending_) return;
    const QUrl poll_url = view_->url();
    if (sceneIdFromViewerUrl(poll_url).isEmpty()) {
      if (save_button_) save_button_->setEnabled(false);
      return;
    }
    state_poll_pending_ = true;
    static const char kStateScript[] = R"JS(
(() => {
  const api = window.__WORKCELL_EDITOR_API_V1__;
  if (!api) return {ready:false,dirty:false,dirtyCount:0,sceneId:''};
  const state = api.getState();
  return {ready:Boolean(state.ready),dirty:Boolean(state.dirty),dirtyCount:Number(state.dirtyCount||0),sceneId:String(state.sceneId||'')};
})()
)JS";
    view_->page()->runJavaScript(QString::fromUtf8(kStateScript), [this, poll_url](const QVariant & value) {
      state_poll_pending_ = false;
      if (!view_ || view_->url() != poll_url || busy_) return;
      const QVariantMap state = value.toMap();
      const bool ready = state.value(QStringLiteral("ready")).toBool();
      const bool dirty = state.value(QStringLiteral("dirty")).toBool();
      const QString reported_scene = state.value(QStringLiteral("sceneId")).toString();
      const QString url_scene = sceneIdFromViewerUrl(poll_url);
      if (save_button_) save_button_->setEnabled(ready && dirty && !url_scene.isEmpty() && reported_scene == url_scene);
      last_polled_url_ = poll_url;
    });
  }

  QPointer<QWidget> preview_;
  QPointer<QWebEngineView> view_;
  QPointer<QPushButton> save_button_;
  QPointer<QLabel> status_label_;
  QPointer<QProcess> process_;
  QTimer poll_timer_{this};
  QUrl expected_url_;
  QUrl last_polled_url_;
  QString scene_id_;
  QString repo_root_;
  QString scene_dir_;
  QString patch_path_;
  bool installed_{false};
  bool busy_{false};
  bool state_poll_pending_{false};
};
}  // namespace embedded_web_edit_save_detail

inline void installEmbeddedWebEditSaveControllers(QWidget * root)
{
  if (!root) return;
  QList<QWidget *> previews;
  if (root->objectName() == QStringLiteral("scenePreviewWidget")) previews << root;
  previews.append(root->findChildren<QWidget *>(QStringLiteral("scenePreviewWidget"), Qt::FindChildrenRecursively));
  for (QWidget * preview : previews) {
    if (!preview || preview->property("workcell_embedded_save_controller").toBool()) continue;
    QWebEngineView * view = preview->findChild<QWebEngineView *>(QStringLiteral("embeddedWeb3dProductView"));
    if (!view) continue;
    auto * controller = new embedded_web_edit_save_detail::EmbeddedWebEditSaveController(preview, view);
    if (controller->installed()) preview->setProperty("workcell_embedded_save_controller", true);
    else controller->deleteLater();
  }
}
}  // namespace workcell_builder

#else

namespace workcell_builder
{
inline void installEmbeddedWebEditSaveControllers(QWidget *) {}
}  // namespace workcell_builder

#endif
