#pragma once

#include <QWidget>

#ifdef WORKCELL_BUILDER_HAS_WEBENGINE

#include "embedded_web_edit_save_controller.hpp"

#include <QBoxLayout>
#include <QFile>
#include <QInputDialog>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QMap>
#include <QMessageBox>
#include <QPointer>
#include <QProcess>
#include <QPushButton>
#include <QSizePolicy>
#include <QTimer>
#include <QVariantMap>
#include <QWebEnginePage>
#include <QWebEngineView>

namespace workcell_builder
{
namespace embedded_web_curated_add_detail
{
using embedded_web_edit_save_detail::findRepoRoot;
using embedded_web_edit_save_detail::resolveSceneDir;
using embedded_web_edit_save_detail::sceneIdFromViewerUrl;

struct CatalogChoice
{
  QString asset_id;
  QString label;
  QString category;
  QString license;
  QString source_note;
  QJsonArray dimensions;
};

class EmbeddedWebCuratedAddController : public QObject
{
public:
  EmbeddedWebCuratedAddController(QWidget * preview, QWebEngineView * view)
  : QObject(preview), preview_(preview), view_(view)
  {
    installed_ = createControls();
    if (!installed_) return;
    connect(add_button_, &QPushButton::clicked, this, [this]() { requestAdd(); });
    connect(view_, &QWebEngineView::loadFinished, this, [this](bool) {
      QTimer::singleShot(250, this, [this]() { pollEditorState(); });
    });
    poll_timer_.setInterval(400);
    connect(&poll_timer_, &QTimer::timeout, this, [this]() { pollEditorState(); });
    poll_timer_.start();
    pollEditorState();
  }

  bool installed() const { return installed_; }

private:
  enum class Phase { Plan, Write, GenerateValidate, Refresh };

  bool createControls()
  {
    if (!preview_ || !view_) return false;
    QPushButton * fit = nullptr;
    for (QPushButton * button : preview_->findChildren<QPushButton *>()) {
      if (button->text() == QStringLiteral("Fit")) { fit = button; break; }
    }
    if (!fit) return false;
    QBoxLayout * toolbar = nullptr;
    for (QBoxLayout * layout : preview_->findChildren<QBoxLayout *>()) {
      if (layout->indexOf(fit) >= 0) { toolbar = layout; break; }
    }
    if (!toolbar) return false;

    add_button_ = new QPushButton(QStringLiteral("Add object"), preview_);
    add_button_->setObjectName(QStringLiteral("embeddedCuratedAddObjectButton"));
    add_button_->setProperty("class", "primary_action");
    add_button_->setEnabled(false);
    add_button_->setToolTip(QStringLiteral(
      "Choose an approved bin, tray, fixture or pick object. Workcell Studio finds a collision-free table position, backs up the editable layout, validates, regenerates and reloads Product View. No robot motion is started."));
    add_button_->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);

    status_label_ = new QLabel(preview_);
    status_label_->setObjectName(QStringLiteral("embeddedCuratedAddObjectStatus"));
    status_label_->setWordWrap(true);
    status_label_->setMaximumWidth(150);
    status_label_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    status_label_->setVisible(false);

    const int fit_index = toolbar->indexOf(fit);
    toolbar->insertWidget(fit_index + 1, add_button_);
    toolbar->insertWidget(fit_index + 2, status_label_);
    return true;
  }

  void setStatus(const QString & text, const QString & level = QStringLiteral("info"))
  {
    if (!status_label_) return;
    status_label_->setText(text); status_label_->setVisible(!text.isEmpty());
    QString bg = QStringLiteral("#e7f0fb"), fg = QStringLiteral("#2769b3");
    if (level == QStringLiteral("success")) { bg = QStringLiteral("#e8f5eb"); fg = QStringLiteral("#217a34"); }
    else if (level == QStringLiteral("warning")) { bg = QStringLiteral("#fff3e0"); fg = QStringLiteral("#8a5200"); }
    else if (level == QStringLiteral("error")) { bg = QStringLiteral("#fdecea"); fg = QStringLiteral("#b3261e"); }
    status_label_->setStyleSheet(QStringLiteral(
      "background:%1;color:%2;border:1px solid #c7d3df;border-radius:8px;padding:2px 6px;font-weight:600;").arg(bg, fg));
  }

  bool contextCurrent() const
  {
    return view_ && view_->url() == expected_url_ && sceneIdFromViewerUrl(view_->url()) == scene_id_;
  }

  void sceneChanged()
  {
    busy_ = false;
    if (add_button_) add_button_->setEnabled(false);
    setStatus(QStringLiteral("Scene changed—reload required"), QStringLiteral("warning"));
  }

  bool resolveContext(QString * error)
  {
    expected_url_ = view_ ? view_->url() : QUrl();
    scene_id_ = sceneIdFromViewerUrl(expected_url_);
    repo_root_ = findRepoRoot();
    scene_dir_ = resolveSceneDir(repo_root_, scene_id_);
    if (scene_id_.isEmpty() || repo_root_.isEmpty() || scene_dir_.isEmpty()) {
      if (error) *error = QStringLiteral("The active Product View scene or repository could not be resolved safely.");
      return false;
    }
    return true;
  }

  QList<CatalogChoice> catalog(QString * error) const
  {
    const QDir root(repo_root_);
    QFile profile(root.filePath(QStringLiteral(
      "workcell_builder/workcell_builder/config/asset_profiles/environment_assets.json")));
    QFile curated(root.filePath(QStringLiteral(
      "workcell_builder/workcell_builder/config/asset_profiles/curated_add_objects.json")));
    if (!profile.open(QIODevice::ReadOnly) || !curated.open(QIODevice::ReadOnly)) {
      if (error) *error = QStringLiteral("The curated object catalog could not be read.");
      return {};
    }
    const QJsonDocument profile_doc = QJsonDocument::fromJson(profile.readAll());
    const QJsonDocument curated_doc = QJsonDocument::fromJson(curated.readAll());
    if (!profile_doc.isArray() || !curated_doc.isObject() ||
        curated_doc.object().value(QStringLiteral("schema_version")).toString() !=
          QStringLiteral("workcell_studio_curated_add_objects/v1")) {
      if (error) *error = QStringLiteral("The curated object catalog contract is invalid.");
      return {};
    }
    QMap<QString, QJsonObject> profiles;
    for (const QJsonValue & value : profile_doc.array()) {
      const QJsonObject item = value.toObject();
      profiles.insert(item.value(QStringLiteral("asset_id")).toString(), item);
    }
    QList<CatalogChoice> choices;
    for (const QJsonValue & value : curated_doc.object().value(QStringLiteral("objects")).toArray()) {
      const QString id = value.toObject().value(QStringLiteral("asset_id")).toString();
      const QJsonObject item = profiles.value(id);
      if (id.isEmpty() || item.isEmpty()) continue;
      choices.push_back({id, item.value(QStringLiteral("label")).toString(),
        item.value(QStringLiteral("category")).toString(), item.value(QStringLiteral("license")).toString(),
        item.value(QStringLiteral("source_note")).toString(), item.value(QStringLiteral("default_dimensions_m")).toArray()});
    }
    std::sort(choices.begin(), choices.end(), [](const CatalogChoice & a, const CatalogChoice & b) {
      return a.category == b.category ? a.label < b.label : a.category < b.category;
    });
    if (choices.isEmpty() && error) *error = QStringLiteral("No approved curated objects are available.");
    return choices;
  }

  void requestAdd()
  {
    if (busy_ || !view_) return;
    QString error;
    if (!resolveContext(&error)) {
      setStatus(QStringLiteral("Add failed"), QStringLiteral("error"));
      QMessageBox::warning(preview_, QStringLiteral("Add Curated Object"), error); return;
    }
    const QList<CatalogChoice> choices = catalog(&error);
    if (choices.isEmpty()) {
      setStatus(QStringLiteral("Add failed"), QStringLiteral("error"));
      QMessageBox::warning(preview_, QStringLiteral("Add Curated Object"), error); return;
    }
    QStringList labels; QMap<QString, CatalogChoice> by_label;
    for (const CatalogChoice & choice : choices) {
      const QString dims = QStringLiteral("%1 × %2 × %3 m")
        .arg(choice.dimensions.value(0).toDouble(), 0, 'f', 2)
        .arg(choice.dimensions.value(1).toDouble(), 0, 'f', 2)
        .arg(choice.dimensions.value(2).toDouble(), 0, 'f', 2);
      const QString label = QStringLiteral("%1 — %2 (%3)").arg(choice.category, choice.label, dims);
      labels << label; by_label.insert(label, choice);
    }
    bool accepted = false;
    const QString selected = QInputDialog::getItem(preview_, QStringLiteral("Add Curated Object"),
      QStringLiteral("Choose an approved object:"), labels, 0, false, &accepted);
    if (!accepted || selected.isEmpty()) return;
    selected_ = by_label.value(selected);
    busy_ = true; if (add_button_) add_button_->setEnabled(false);
    start(Phase::Plan);
  }

  void start(Phase phase)
  {
    if (!contextCurrent()) { sceneChanged(); return; }
    if (process_) process_->deleteLater();
    process_ = new QProcess(this); process_->setProgram(QStringLiteral("python3"));
    QStringList args;
    if (phase == Phase::Plan || phase == Phase::Write) {
      args << QStringLiteral("scripts/add_workcell_studio_curated_object.py")
           << QStringLiteral("--scene") << scene_dir_ << QStringLiteral("--asset-id") << selected_.asset_id
           << QStringLiteral("--json");
      if (phase == Phase::Write) {
        args << QStringLiteral("--instance-id") << planned_instance_
             << QStringLiteral("--expected-layout-sha256") << planned_sha_
             << QStringLiteral("--write") << QStringLiteral("--backup");
      }
      setStatus(phase == Phase::Plan ? QStringLiteral("Planning…") : QStringLiteral("Adding…"));
    } else if (phase == Phase::GenerateValidate) {
      args << QStringLiteral("scripts/run_workcell_studio_web_edit_workflow.py")
           << QStringLiteral("--scene") << scene_dir_ << QStringLiteral("--generate-and-validate");
      setStatus(QStringLiteral("Generating…"));
    } else {
      args << QStringLiteral("scripts/ensure_workcell_studio_web_scene_fresh.py")
           << QStringLiteral("--scene") << scene_dir_ << QStringLiteral("--output")
           << QStringLiteral("build/workcell_studio_web_scene/%1.web_scene.json").arg(scene_id_)
           << QStringLiteral("--stage-assets") << QStringLiteral("--force");
      setStatus(QStringLiteral("Refreshing…"));
    }
    process_->setArguments(args); process_->setWorkingDirectory(repo_root_);
    process_->setProcessChannelMode(QProcess::MergedChannels);
    QProcess * const process = process_;
    connect(process, qOverload<int, QProcess::ExitStatus>(&QProcess::finished), this,
      [this, process, phase](int code, QProcess::ExitStatus status) {
        const QByteArray output = process->readAll();
        if (process == process_) process_ = nullptr; process->deleteLater();
        if (!contextCurrent()) { sceneChanged(); return; }
        if (status != QProcess::NormalExit || code != 0) {
          busy_ = false; setStatus(QStringLiteral("Add failed"), QStringLiteral("error"));
          QMessageBox::critical(preview_, QStringLiteral("Add Curated Object"),
            QStringLiteral("The curated object workflow failed.\n\n%1").arg(QString::fromLocal8Bit(output).left(8000)));
          pollEditorState(); return;
        }
        phaseFinished(phase, output);
      });
    process_->start();
  }

  void phaseFinished(Phase phase, const QByteArray & output)
  {
    if (phase == Phase::Plan) {
      const QJsonObject plan = QJsonDocument::fromJson(output.trimmed()).object();
      if (plan.value(QStringLiteral("status")).toString() != QStringLiteral("planned") ||
          plan.value(QStringLiteral("scene_id")).toString() != scene_id_) {
        busy_ = false; setStatus(QStringLiteral("Add failed"), QStringLiteral("error")); return;
      }
      planned_instance_ = plan.value(QStringLiteral("instance_id")).toString();
      planned_sha_ = plan.value(QStringLiteral("layout_sha256")).toString();
      const QJsonArray xyz = plan.value(QStringLiteral("pose_xyz")).toArray();
      const QString message = QStringLiteral(
        "Add %1 as %2?\n\nSupport: %3\nPose XYZ: %4, %5, %6 m\nLicense: %7\nSource: %8\n\n"
        "Workcell Studio will back up and atomically update only the editable layout, then regenerate, validate and reload Product View. No robot motion is started.")
        .arg(selected_.label, planned_instance_, plan.value(QStringLiteral("support_surface_id")).toString())
        .arg(xyz.value(0).toDouble(), 0, 'f', 3).arg(xyz.value(1).toDouble(), 0, 'f', 3)
        .arg(xyz.value(2).toDouble(), 0, 'f', 3).arg(selected_.license, selected_.source_note);
      if (QMessageBox::question(preview_, QStringLiteral("Confirm Add Curated Object"), message,
          QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes) {
        busy_ = false; setStatus(QStringLiteral("Add cancelled"), QStringLiteral("warning")); pollEditorState(); return;
      }
      start(Phase::Write); return;
    }
    if (phase == Phase::Write) { start(Phase::GenerateValidate); return; }
    if (phase == Phase::GenerateValidate) { start(Phase::Refresh); return; }
    busy_ = false; setStatus(QStringLiteral("Added"), QStringLiteral("success"));
    if (view_) view_->reload(); QTimer::singleShot(700, this, [this]() { pollEditorState(); });
  }

  void pollEditorState()
  {
    if (!installed_ || !view_ || busy_ || poll_pending_) return;
    const QUrl url = view_->url(); const QString url_scene = sceneIdFromViewerUrl(url);
    if (url_scene.isEmpty()) { if (add_button_) add_button_->setEnabled(false); return; }
    poll_pending_ = true;
    static const char kState[] = R"JS((() => { const a=window.__WORKCELL_EDITOR_API_V1__; if(!a)return {ready:false,dirty:true,sceneId:''}; const s=a.getState(); return {ready:Boolean(s.ready),dirty:Boolean(s.dirty),sceneId:String(s.sceneId||'')}; })())JS";
    view_->page()->runJavaScript(QString::fromUtf8(kState), [this, url, url_scene](const QVariant & value) {
      poll_pending_ = false; if (!view_ || view_->url() != url || busy_) return;
      const QVariantMap state = value.toMap();
      if (add_button_) add_button_->setEnabled(state.value(QStringLiteral("ready")).toBool() &&
        !state.value(QStringLiteral("dirty")).toBool() && state.value(QStringLiteral("sceneId")).toString() == url_scene);
      add_button_->setToolTip(state.value(QStringLiteral("dirty")).toBool() ?
        QStringLiteral("Save or undo current Product View edits before adding an object.") :
        QStringLiteral("Add an approved catalog object to a collision-free support-surface position."));
    });
  }

  QPointer<QWidget> preview_; QPointer<QWebEngineView> view_; QPointer<QPushButton> add_button_;
  QPointer<QLabel> status_label_; QPointer<QProcess> process_; QTimer poll_timer_{this};
  QUrl expected_url_; QString scene_id_, repo_root_, scene_dir_, planned_instance_, planned_sha_;
  CatalogChoice selected_; bool installed_{false}, busy_{false}, poll_pending_{false};
};
}  // namespace embedded_web_curated_add_detail

inline void installEmbeddedWebCuratedAddControllers(QWidget * root)
{
  if (!root) return;
  QList<QWidget *> previews;
  if (root->objectName() == QStringLiteral("scenePreviewWidget")) previews << root;
  previews.append(root->findChildren<QWidget *>(QStringLiteral("scenePreviewWidget"), Qt::FindChildrenRecursively));
  for (QWidget * preview : previews) {
    if (!preview || preview->property("workcell_embedded_curated_add_controller").toBool()) continue;
    auto * view = preview->findChild<QWebEngineView *>(QStringLiteral("embeddedWeb3dProductView"));
    if (!view) continue;
    auto * controller = new embedded_web_curated_add_detail::EmbeddedWebCuratedAddController(preview, view);
    if (controller->installed()) preview->setProperty("workcell_embedded_curated_add_controller", true);
    else controller->deleteLater();
  }
}
}  // namespace workcell_builder

#else
namespace workcell_builder { inline void installEmbeddedWebCuratedAddControllers(QWidget *) {} }
#endif
