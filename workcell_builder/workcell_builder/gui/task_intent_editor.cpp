#include "task_intent_editor.h"
#include <QBrush>
#include <QCheckBox>
#include <QColor>
#include <QComboBox>
#include <QDateTime>
#include <QDir>
#include <QDirIterator>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QFrame>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QPlainTextEdit>
#include <QLineEdit>
#include <QMessageBox>
#include <QHBoxLayout>
#include <QListWidget>
#include <QProcess>
#include <QPixmap>
#include <QPushButton>
#include <QRegularExpression>
#include <QSaveFile>
#include <QSet>
#include <QSpinBox>
#include <QTableWidget>
#include <QHeaderView>
#include <QSignalBlocker>
#include <QTemporaryDir>
#include <QTimer>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>
#include <memory>
#include <yaml-cpp/yaml.h>

namespace {
// Pick-sequence list items carry the saved binding of one recorded object and its placement;
// the item text is only a description of them. The saved task is the authority (sequence_json).
constexpr int kObjectIdRole = Qt::UserRole;
constexpr int kPlaceOptionsRole = Qt::UserRole + 1;
constexpr int kManualPoseRole = Qt::UserRole + 2;     // target-local pose, empty object = Automatic
constexpr int kBindingRole = Qt::UserRole + 3;        // class_id, observed_xyz_m, dimensions_m
constexpr int kDestinationRole = Qt::UserRole + 6;    // original manual asset/region binding
constexpr int kReviewRole = Qt::UserRole + 4;         // status, message, place_label from the review

QString sequence_object_id(const QListWidgetItem * item)
{
  const QString id = item->data(kObjectIdRole).toString();
  return id.isEmpty() ? item->text() : id;
}

bool same_pose(const QJsonObject & a, const QJsonObject & b)
{
  for (const auto & key : {"xyz_m", "rpy_rad"}) {
    const auto x = a.value(key).toArray(), y = b.value(key).toArray();
    if (x.size() != 3 || y.size() != 3) return false;
    for (int i = 0; i < 3; ++i) if (std::abs(x[i].toDouble() - y[i].toDouble()) > 1e-6) return false;
  }
  return true;
}

// The offered place matching the item's manual pose ("" = Automatic, "custom" = no offered place matches).
QString chosen_place_id(const QListWidgetItem * item)
{
  const auto pose = item->data(kManualPoseRole).toJsonObject();
  if (pose.isEmpty()) return QString();
  for (const auto & value : item->data(kPlaceOptionsRole).toJsonArray())
    if (same_pose(value.toObject().value("local_pose").toObject(), pose)) return value.toObject().value("id").toString();
  return "custom";
}

QString place_description(const QListWidgetItem * item)
{
  const auto pose = item->data(kManualPoseRole).toJsonObject();
  const auto review = item->data(kReviewRole).toJsonObject();
  if (pose.isEmpty()) return review.value("place_label").toString().startsWith("Automatic")
                             ? review.value("place_label").toString() : QString("Automatic");
  const QString id = chosen_place_id(item);
  for (const auto & value : item->data(kPlaceOptionsRole).toJsonArray())
    if (value.toObject().value("id").toString() == id) return value.toObject().value("label").toString();
  const auto xyz = pose.value("xyz_m").toArray();
  return QString("Custom place (x %1, y %2 m)").arg(xyz[0].toDouble(), 0, 'f', 3).arg(xyz[1].toDouble(), 0, 'f', 3);
}

void show_sequence_item(QListWidgetItem * item, int position)
{
  QString id = sequence_object_id(item);
  if (id.startsWith("runtime::")) id = id.mid(9);
  const auto binding = item->data(kBindingRole).toJsonObject();
  const auto review = item->data(kReviewRole).toJsonObject();
  const QString status = review.value("status").toString();
  const bool included = item->checkState() == Qt::Checked;
  QString text = included ? QString("%1. %2").arg(position).arg(id) : QString("— %1").arg(id);
  if (binding.contains("class_id")) text += " (" + binding.value("class_id").toString() + ")";
  text += included ? "  →  " + place_description(item) : QString("  (not in the sequence)");
  if (included && (status == "NEEDS_REVIEW" || status == "BLOCKED")) text = "⚠ " + text + "  —  " + QString(status).replace('_', ' ');
  item->setText(text);
  item->setForeground(!included ? QBrush(Qt::gray) : status == "BLOCKED" ? QBrush(QColor("#b3261e"))
                      : status == "NEEDS_REVIEW" ? QBrush(QColor("#9a5b00")) : QBrush());
  item->setToolTip(review.value("message").toString());
}
}  // namespace

namespace {
QJsonValue value_at(QJsonObject root, const QString & path)
{
  QJsonValue value(root);
  for (const auto & key : path.split('/')) value = value.toObject().value(key);
  return value;
}
QString json_value(const QJsonValue & value)
{
  QByteArray data = QJsonDocument(QJsonArray{value}).toJson(QJsonDocument::Compact);
  return QString::fromUtf8(data.mid(1, data.size() - 2));
}

// What an observation file declares about itself, for display only: planning eligibility is always the
// authoring helper's planning_input verdict.
struct ObservationFile {
  bool ok{false};
  QString problem, mode, camera, description;
  QStringList classes;
  int objects{0};
  double stamp_s{0.0};
};
ObservationFile read_observation_file(const QString & path)
{
  ObservationFile file;
  QFile raw(path);
  if (!raw.open(QIODevice::ReadOnly)) { file.problem = "file not found"; return file; }
  const QByteArray bytes = raw.readAll();
  if (!bytes.contains("detected_objects/v1")) { file.problem = "not a detected_objects/v1 observation file"; return file; }
  try {
    const YAML::Node root = YAML::Load(bytes.toStdString());
    if (!root.IsMap() || !root["schema_version"] || root["schema_version"].as<std::string>("") != "detected_objects/v1") {
      file.problem = "not a detected_objects/v1 observation file"; return file;
    }
    const YAML::Node source = root["source"], objects = root["objects"];
    if (source && source.IsMap()) {
      file.mode = QString::fromStdString(source["mode"].as<std::string>(""));
      file.camera = QString::fromStdString(source["camera_source"].as<std::string>(""));
      file.description = QString::fromStdString(source["description"].as<std::string>(""));
      if (source["source_stamp_ns"]) file.stamp_s = source["source_stamp_ns"].as<double>(0.0) / 1e9;
    }
    if (objects && objects.IsSequence()) {
      file.objects = static_cast<int>(objects.size());
      for (const auto & object : objects) {
        const auto name = object.IsMap() ? QString::fromStdString(object["class_id"].as<std::string>("")) : QString();
        if (!name.isEmpty() && !file.classes.contains(name)) file.classes << name;
      }
    }
    file.ok = true;
  } catch (const std::exception &) {
    file.problem = "the file does not parse";
  }
  return file;
}

// The repository the cell belongs to, as seen from the cell's own path (so stored paths stay in one
// namespace); a cell saved elsewhere falls back to the repository of the installed authoring helper.
QString repository_root(const QString & scene, const QString & helper)
{
  for (QDir dir(scene); !scene.isEmpty();) {
    if (QFileInfo::exists(dir.filePath("scripts/task_intent_authoring.py")) && QFileInfo(dir.filePath("config/runtime")).isDir())
      return dir.absolutePath();
    if (!dir.cdUp()) break;
  }
  QDir scripts = QFileInfo(QFileInfo(helper).canonicalFilePath()).dir();
  return !helper.isEmpty() && scripts.cdUp() ? scripts.absolutePath() : QString();
}

QString elapsed_text(double seconds)
{
  if (seconds < 120) return QString("%1 s").arg(std::max(0.0, seconds), 0, 'f', 0);
  if (seconds < 7200) return QString("%1 min").arg(seconds / 60, 0, 'f', 0);
  return QString("%1 h").arg(seconds / 3600, 0, 'f', 1);
}

// A class-routed task bound to live EPD on the simulated camera (no saved capture) is observed by its own
// sorting session every cycle (task_intent_v2.session_rendered_perception).
bool session_observed(const QJsonObject & report)
{
  const auto intent = report["task_intent"].toObject();
  const auto binding = intent["perception"].toObject();
  const auto routing = intent["routing"].toObject();
  return binding["mode"].toString() == "live_epd" && binding["camera_source"].toString() == "simulated_rgbd" &&
    binding["replay_path"].toString().isEmpty() && routing["mode"].toString() == "class" && !routing["rules"].toArray().isEmpty();
}

// The next step for a planning_input blocker (its code is the text before the first ':').
QString next_input_action(const QString & blocker, const QString & mode)
{
  const QString code = blocker.section(':', 0, 0).trimmed();
  const bool live = mode == "live_epd";
  if (code == "TASK_INPUT_UNAVAILABLE")
    return live ? "start EPD for the selected camera, then press Capture live EPD objects."
         : mode == "off" ? "to plan on fake hardware, set Perception mode to Recorded observations (replay) and choose a recording."
         : "choose a recording under Existing input (the reviewed commissioning recording is config/runtime/r1_4_replay.yaml), or Choose observation file….";
  if (code == "TASK_INPUT_INVALID") return "this file is not a detected_objects/v1 file with measured objects; choose another input.";
  if (code == "REPLAY_INPUT_REQUIRED") return "this file is a live capture, not a recording: choose a recording, or set Perception mode to Live EPD.";
  if (code == "LIVE_INPUT_REQUIRED") return "Live EPD never plans from a recording: capture live EPD objects, or set Perception mode to a recording.";
  if (code == "CAMERA_SOURCE_REQUIRED") return "choose the Live camera source.";
  if (code == "CAMERA_SOURCE_MISMATCH") return "capture again with the selected camera, or select the camera the capture came from.";
  if (code == "OBSERVATION_UNAVAILABLE")
    return live ? "the capture is too old or has no matching object: capture again just before planning."
                : "no object in this input matches the object class, confidence and search region: check What / Pick eligibility, or choose another input.";
  if (code == "OBSERVATION_PRODUCER_REJECTED")
    return live ? "this source does not accept a live capture: check Perception mode and the camera source."
                : "this Perception mode does not plan from a recording on fake hardware: set Perception mode to Recorded observations (replay).";
  if (blocker.contains("Edit the placement")) return "edit the placement, or choose another input.";
  return "resolve the reason above, then Save task.";
}
}

TaskIntentEditor::TaskIntentEditor(QWidget * parent) : QGroupBox("Task Authoring — Pick && Place", parent)  // "&&": literal "&", not a mnemonic
{
  setObjectName("environmentTaskEditor");
  auto * layout = new QVBoxLayout(this);
  auto section = [&](const QString & label) {
    auto * group = new QGroupBox(label, this); layout->addWidget(group);
    return new QFormLayout(group);
  };
  auto * what = section("What / Pick eligibility");
  text_field(what, "taskTargetClass", "Object class", "pick/selection/object_filter/class_id");
  {
    // Classes that must never become pick targets (e.g. pick cups, ignore bottles).
    auto * excluded = new QLineEdit(this); excluded->setObjectName("taskExcludeClasses");
    excluded->setPlaceholderText("comma-separated classes, e.g. bottle"); what->addRow("Exclude classes", excluded);
    const QString path = "pick/selection/object_filter/exclude_class_ids";
    readers_[path] = [excluded](const QJsonValue & v) {
      QStringList names; for (const auto & item : v.toArray()) names << item.toString();
      excluded->setText(names.join(", "));
    };
    connect(excluded, &QLineEdit::textEdited, this, [this, path](const QString & text) {
      QJsonArray names;
      for (const auto & part : text.split(',')) if (!part.trimmed().isEmpty()) names.append(part.trimmed());
      set_field(path, names);
    });
  }
  pick_zones_ = choice_field(what, "taskPickZone", "Search region", "pick/selection/zone_ref", {});
  auto * perception = section("Object input");
  auto * mode = new QComboBox(this); mode->setObjectName("taskPerceptionMode");
  mode->addItem("Off — simulator-measured objects (physics only)", "off");
  mode->addItem("Recorded observations (replay)", "replayed_snapshot");
  mode->addItem("Live EPD camera observations", "live_epd");
  perception->addRow("Perception mode", mode);
  auto * mode_help = new QLabel(this); mode_help->setObjectName("taskInputModeHelp");
  mode_help->setWordWrap(true); perception->addRow(mode_help);
  auto * camera = new QComboBox(this); camera->setObjectName("taskCameraSource");
  camera->addItem("Choose camera source…", "");
  camera->addItem("Simulated RGB-D (Gazebo + EPD)", "simulated_rgbd");
  camera->addItem("Intel RealSense D435i (EPD)", "realsense_d435i");
  camera->setEnabled(false);
  perception->addRow("Live camera source", camera);
  readers_["perception/camera_source"] = [camera](const QJsonValue & value) {
    const int index = value.isString() ? camera->findData(value.toString()) : -1;
    camera->setCurrentIndex(index >= 0 ? index : 0);
  };
  connect(camera, qOverload<int>(&QComboBox::currentIndexChanged), this, [this, mode, camera](int index) {
    if (loading_ || index < 0 || mode->currentData().toString() != "live_epd") return;
    const auto source = camera->currentData().toString();
    set_field("perception/camera_source", source.isEmpty() ? QJsonValue(QJsonValue::Null) : QJsonValue(source));
  });
  readers_["perception/mode"] = [mode](const QJsonValue & value) {
    mode->setCurrentIndex(value.isString() ? mode->findData(value.toString()) : -1);
  };
  connect(mode, qOverload<int>(&QComboBox::currentIndexChanged), this, [this, mode, camera](int index) {
    if (loading_ || index < 0) return;
    const auto value = mode->currentData().toString();
    set_field("perception/mode", value);
    set_field("pick/selection/source_type", value == "off" ? "manual_simulated" : "perception");
    set_field("pick/selection/source_ref", value == "off" ? QString::fromStdString(model_.pick_selection.zone_ref) : "detected_objects/v1");
    if (value != "live_epd") set_field("perception/camera_source", QJsonValue(QJsonValue::Null));
    // The chosen file is kept; planning_input refuses a recording as live input (and a capture as a recording).
    age_follows_mode_ = true;  // the next check caps the maximum age at the new source's limit
    refresh_fields();
    refresh_recordings();
    camera->setEnabled(value == "live_epd");
  });
  auto * recordings = new QComboBox(this); recordings->setObjectName("taskObservationRecording");
  perception->addRow("Existing input", recordings);
  connect(recordings, qOverload<int>(&QComboBox::activated), this, [this, recordings](int index) {
    const auto path = recordings->itemData(index).toString();
    if (!path.isEmpty()) { set_field("perception/replay_path", path); refresh_fields(); }
  });
  text_field(perception, "taskReplayPath", "Observation snapshot", "perception/replay_path");
  auto * choose = new QPushButton("Choose observation file…", this); choose->setObjectName("taskChooseObservations");
  perception->addRow(choose);
  connect(choose, &QPushButton::clicked, this, [this] {
    const auto path = QFileDialog::getOpenFileName(this, "Choose existing detected_objects/v1 observations", scene_, "Observations (*.yaml *.yml *.json)");
    if (!path.isEmpty()) { set_field("perception/replay_path", stored_input_path(path)); refresh_fields(); }
  });
  auto * input_status = new QLabel(this); input_status->setObjectName("taskInputStatus");
  input_status->setWordWrap(true); input_status->setFrameShape(QFrame::StyledPanel);
  input_status->setAlignment(Qt::AlignTop | Qt::AlignLeft);
  input_status->setTextInteractionFlags(Qt::TextSelectableByMouse);
  input_status->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);  // long paths must not widen the inspector
  perception->addRow(input_status);
  auto * probe_button = new QPushButton("Capture RGB-D preview (read-only)", this);
  probe_button->setObjectName("taskCameraCapture");
  probe_button->setEnabled(false);
  perception->addRow(probe_button);
  auto * preview_status = new QLabel("Save and select a live camera to inspect existing ROS streams.", this);
  preview_status->setObjectName("taskCameraCaptureStatus");
  preview_status->setWordWrap(true);
  perception->addRow(preview_status);
  auto * preview_row = new QHBoxLayout;
  auto * rgb_preview = new QLabel("RGB — no capture", this);
  auto * depth_preview = new QLabel("Depth — no capture", this);
  rgb_preview->setObjectName("taskCameraRgbPreview");
  depth_preview->setObjectName("taskCameraDepthPreview");
  for (auto * preview : {rgb_preview, depth_preview}) {
    preview->setMinimumSize(170, 112);
    preview->setMaximumSize(240, 160);
    preview->setAlignment(Qt::AlignCenter);
    preview->setScaledContents(false);
    preview_row->addWidget(preview);
  }
  perception->addRow(preview_row);
  connect(probe_button, &QPushButton::clicked, this,
          [this, probe_button, preview_status, rgb_preview, depth_preview] {
    const auto * selected = findChild<QComboBox *>("taskCameraSource");
    const auto * live_mode = findChild<QComboBox *>("taskPerceptionMode");
    if (scene_.isEmpty() || dirty_ || live_mode->currentData().toString() != "live_epd" ||
        selected->currentData().toString().isEmpty()) {
      preview_status->setText("BLOCKED — Save the Task with a selected live camera before preview.");
      return;
    }
    const QString script = QFileInfo(helper_).dir().absoluteFilePath("workcell_camera_probe.py");
    if (!QFileInfo::exists(script)) {
      preview_status->setText("BLOCKED — workcell_camera_probe.py is not installed beside the Task helper.");
      return;
    }
    const auto temporary = std::make_shared<QTemporaryDir>();
    if (!temporary->isValid()) {
      preview_status->setText("BLOCKED — Cannot create private camera preview directory.");
      return;
    }
    const QString captured_scene = scene_;
    const QString source = selected->currentData().toString();
    auto * process = new QProcess(this);
    probe_button->setEnabled(false);
    rgb_preview->setText("Waiting for RGB…"); depth_preview->setText("Waiting for depth…");
    preview_status->setText("Capturing existing ROS RGB/depth topics; no camera or robot is launched.");
    connect(process, &QProcess::errorOccurred, this,
            [probe_button, preview_status](QProcess::ProcessError error) {
      if (error == QProcess::FailedToStart) {
        preview_status->setText("BLOCKED — Python camera probe failed to start.");
        probe_button->setEnabled(true);
      }
    });
    connect(process, qOverload<int, QProcess::ExitStatus>(&QProcess::finished), this,
            [this, process, temporary, captured_scene, probe_button, preview_status,
             rgb_preview, depth_preview](int exit_code, QProcess::ExitStatus exit_status) {
      const auto parsed = QJsonDocument::fromJson(process->readAllStandardOutput()).object();
      process->deleteLater();
      if (captured_scene != scene_) return;  // Never display another cell's capture.
      probe_button->setEnabled(findChild<QComboBox *>("taskPerceptionMode")->currentData().toString() == "live_epd");
      const auto errors = parsed.value("errors").toArray();
      const auto warnings = parsed.value("warnings").toArray();
      QStringList messages;
      for (const auto & error : errors) messages << error.toString();
      for (const auto & warning : warnings) messages << warning.toString();
      const bool captured = exit_status == QProcess::NormalExit && exit_code == 0 &&
          parsed.value("status").toString() == "PREVIEW_CAPTURED";
      const QString headline = captured ? "RGB-D preview captured — NOT calibrated or motion-ready." :
          "BLOCKED — camera preview incomplete.";
      preview_status->setText(headline + (messages.isEmpty() ? "" : "\n" + messages.join("\n")));
      for (const auto & item : {qMakePair(rgb_preview, QString("rgb")),
                                qMakePair(depth_preview, QString("depth"))}) {
        QPixmap pixmap(temporary->filePath(item.second + ".png"));
        if (pixmap.isNull()) {
          item.first->setPixmap(QPixmap());
          item.first->setText(item.second.toUpper() + " unavailable");
        } else {
          item.first->setPixmap(pixmap.scaled(item.first->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
        }
      }
    });
    process->start("python3", {script, "--scene", captured_scene, "--camera-source", source,
                               "--output-dir", temporary->path(), "--timeout", "5"});
  });
  auto * epd_button = new QPushButton("Capture live EPD objects (no robot motion)", this);
  epd_button->setObjectName("taskCaptureLiveEpd");
  epd_button->setEnabled(false);
  perception->addRow(epd_button);
  auto * epd_status = new QLabel("A saved live task and a running EPD localization/tracking topic are required.", this);
  epd_status->setObjectName("taskCaptureLiveEpdStatus");
  epd_status->setWordWrap(true); perception->addRow(epd_status);
  connect(epd_button, &QPushButton::clicked, this, [this, epd_button, epd_status] {
    const auto * mode = findChild<QComboBox *>("taskPerceptionMode");
    const auto * source = findChild<QComboBox *>("taskCameraSource");
    if (dirty_ || scene_.isEmpty() || mode->currentData().toString() != "live_epd" ||
        source->currentData().toString().isEmpty()) {
      epd_status->setText("BLOCKED — Select a live camera and save the task before EPD capture.");
      return;
    }
    const QString script = QFileInfo(helper_).dir().absoluteFilePath("capture_selected_live_epd.py");
    if (!QFileInfo::exists(script)) {
      epd_status->setText("BLOCKED — capture_selected_live_epd.py is unavailable beside the Task helper.");
      return;
    }
    const QString captured_scene = scene_;
    const QString captured_source = source->currentData().toString();
    auto * process = new QProcess(this);
    epd_button->setEnabled(false);
    epd_status->setText("Waiting for live EPD detections and observed world-frame geometry; no motion.");
    connect(process, &QProcess::errorOccurred, this,
            [epd_button, epd_status](QProcess::ProcessError error) {
      if (error == QProcess::FailedToStart) {
        epd_status->setText("BLOCKED — EPD capture helper could not start.");
        epd_button->setEnabled(true);
      }
    });
    connect(process, qOverload<int, QProcess::ExitStatus>(&QProcess::finished), this,
            [this, process, captured_scene, captured_source, epd_button, epd_status]
            (int exit_code, QProcess::ExitStatus exit_status) {
      const auto report = QJsonDocument::fromJson(process->readAllStandardOutput()).object();
      process->deleteLater();
      if (captured_scene != scene_) return;
      const auto source = findChild<QComboBox *>("taskCameraSource")->currentData().toString();
      const bool same_selection = !dirty_ && source == captured_source &&
          findChild<QComboBox *>("taskPerceptionMode")->currentData().toString() == "live_epd";
      epd_button->setEnabled(same_selection);
      if (exit_status != QProcess::NormalExit || exit_code != 0 ||
          report.value("status").toString() != "CAPTURE_QUALIFIED_FOR_TASK_INPUT") {
        epd_status->setText("BLOCKED — " + report.value("error").toString(
          "No eligible EPD capture; check saved camera/EPD topic, measurement and TF diagnostics."));
        return;
      }
      const QString observed = report.value("capture_path").toString();
      if (!same_selection || observed.isEmpty()) {
        epd_status->setText("Capture saved at " + observed +
          " but the task or camera changed during capture. Review the file and select it manually.");
        return;
      }
      // Adopt the qualified candidate into the in-memory task. The user must
      // explicitly save; no old replay or authored source file is overwritten.
      set_field("perception/replay_path", observed);
      refresh_fields();
      epd_status->setText("EPD task observation captured: " + observed +
        "\nReview and Save Task. Source hardware identity and full grasp remain unqualified.");
    });
    process->start("python3", {script, "--scene", captured_scene, "--timeout", "10"});
  });
  auto * scene_preflight = new QPushButton("Check live PlanningScene binding (no ROS start)", this);
  scene_preflight->setObjectName("taskLiveScenePreflight");
  scene_preflight->setEnabled(false);
  perception->addRow(scene_preflight);
  auto * preflight_status = new QLabel("Check after saving and generating your live task.", this);
  preflight_status->setObjectName("taskLiveScenePreflightStatus");
  preflight_status->setWordWrap(true);
  perception->addRow(preflight_status);
  connect(scene_preflight, &QPushButton::clicked, this, [this, scene_preflight, preflight_status] {
    const QString source = findChild<QComboBox *>("taskCameraSource")->currentData().toString();
    if (dirty_ || scene_.isEmpty() || source.isEmpty() ||
        findChild<QComboBox *>("taskPerceptionMode")->currentData().toString() != "live_epd") {
      preflight_status->setText("BLOCKED — Save the selected live task before checking PlanningScene binding.");
      return;
    }
    const QString script = QFileInfo(helper_).dir().absoluteFilePath("epd_dynamic_planning_scene_node.py");
    if (!QFileInfo::exists(script)) {
      preflight_status->setText("BLOCKED — Existing continuous EPD PlanningScene helper is not installed.");
      return;
    }
    const QString selected_scene = scene_;
    auto * process = new QProcess(this);
    scene_preflight->setEnabled(false);
    preflight_status->setText("Checking saved camera, task and generated scene parity — no ROS started.");
    connect(process, &QProcess::errorOccurred, this,
            [scene_preflight, preflight_status](QProcess::ProcessError error) {
      if (error == QProcess::FailedToStart) {
        preflight_status->setText("BLOCKED — PlanningScene preflight helper could not start.");
        scene_preflight->setEnabled(true);
      }
    });
    connect(process, qOverload<int, QProcess::ExitStatus>(&QProcess::finished), this,
            [this, process, selected_scene, source, scene_preflight, preflight_status]
            (int exit_code, QProcess::ExitStatus exit_status) {
      const auto report = QJsonDocument::fromJson(process->readAllStandardOutput()).object();
      process->deleteLater();
      if (selected_scene != scene_) return;
      const bool unchanged = !dirty_ &&
          findChild<QComboBox *>("taskCameraSource")->currentData().toString() == source &&
          findChild<QComboBox *>("taskPerceptionMode")->currentData().toString() == "live_epd";
      scene_preflight->setEnabled(unchanged);
      if (!unchanged) {
        preflight_status->setText("BLOCKED — Task/camera changed during preflight; save and recheck.");
      } else if (exit_status == QProcess::NormalExit && exit_code == 0 &&
                 report.value("status").toString() == "PREFLIGHT_CONFIGURED_ONLY") {
        preflight_status->setText("Configuration consistent: " + report.value("camera_id").toString() +
          " / " + report.value("topic").toString() +
          "\nNo ROS started; camera identity and planning/motion are NOT qualified.");
      } else {
        preflight_status->setText("BLOCKED — " + report.value("error").toString(
          "Check saved TaskIntent, camera, Generate output and EPD topic parity."));
      }
    });
    process->start("python3", {script, "--task-intent-scene", selected_scene, "--preflight-only"});
  });
  auto * batch_button = new QPushButton("Preview multi-object task queue (no motion)", this);
  batch_button->setObjectName("taskSimulationBatchPreview");
  batch_button->setEnabled(false);
  perception->addRow(batch_button);
  auto * batch_status = new QLabel("Select an explicit commissioning recording and Save Task to preview the queue.", this);
  batch_status->setObjectName("taskSimulationBatchStatus");
  batch_status->setWordWrap(true);
  perception->addRow(batch_status);
  auto * batch_details = new QPlainTextEdit(this);
  batch_details->setObjectName("taskSimulationBatchDetails");
  batch_details->setReadOnly(true);
  batch_details->setMinimumHeight(135);
  batch_details->setMaximumHeight(205);
  batch_details->setPlainText("No batch preview has been run.");
  perception->addRow(batch_details);
  connect(batch_button, &QPushButton::clicked, this,
          [this, batch_button, batch_status, batch_details] {
    const auto * mode = findChild<QComboBox *>("taskPerceptionMode");
    const QString selection = mode->currentData().toString();
    const bool allowed = selection == "off" || selection == "replayed_snapshot";
    if (scene_.isEmpty() || dirty_ || !allowed) {
      batch_status->setText("BLOCKED — Save an explicit commissioning or replay task before preview.");
      return;
    }
    const QString script = QFileInfo(helper_).dir().absoluteFilePath("simulation_batch_preview.py");
    if (!QFileInfo::exists(script)) {
      batch_status->setText("BLOCKED — simulation_batch_preview.py is not installed with the Task helper.");
      return;
    }
    const QString original_scene = scene_;
    const QString original_mode = selection;
    const QString original_snapshot = findChild<QLineEdit *>("taskReplayPath")->text();
    auto * process = new QProcess(this);
    batch_button->setEnabled(false);
    batch_status->setText("Checking saved task and commissioning objects; no ROS, MoveIt or robot execution.");
    batch_details->setPlainText("Working…");
    connect(process, &QProcess::errorOccurred, this,
            [this, process, original_scene, batch_button, batch_status, batch_details](QProcess::ProcessError error) {
      if (error == QProcess::FailedToStart && original_scene == scene_) {
        batch_status->setText("BLOCKED — batch preview helper failed to start.");
        batch_details->setPlainText("No planning or motion was attempted.");
        const QString selected = findChild<QComboBox *>("taskPerceptionMode")->currentData().toString();
        batch_button->setEnabled(!dirty_ && (selected == "off" || selected == "replayed_snapshot"));
        process->deleteLater();
      }
    });
    connect(process, qOverload<int, QProcess::ExitStatus>(&QProcess::finished), this,
            [this, process, original_scene, original_mode, original_snapshot,
             batch_button, batch_status, batch_details](int exit_code, QProcess::ExitStatus exit_status) {
      const QJsonObject report = QJsonDocument::fromJson(process->readAllStandardOutput()).object();
      process->deleteLater();
      if (original_scene != scene_) return;
      const auto * mode = findChild<QComboBox *>("taskPerceptionMode");
      const bool unchanged = !dirty_ && mode->currentData().toString() == original_mode &&
          findChild<QLineEdit *>("taskReplayPath")->text() == original_snapshot;
      batch_button->setEnabled(unchanged && (original_mode == "off" || original_mode == "replayed_snapshot"));
      if (!unchanged) {
        batch_status->setText("BLOCKED — Task or recording changed during preview. Save and try again.");
        batch_details->setPlainText("Old results cannot be used after changing the task.");
        return;
      }
      const bool ready = exit_status == QProcess::NormalExit && exit_code == 0 &&
          report.value("status").toString() == "ELIGIBILITY_ONLY";
      if (!ready) {
        batch_status->setText("BLOCKED — " + report.value("reason").toString("No eligible commissioning objects."));
        batch_details->setPlainText("No planning or motion attempted.");
        return;
      }
      batch_status->setText(QString("Eligibility only: %1 queued of %2 eligible (%3 observed). Not MoveIt-approved.")
          .arg(report.value("queued").toInt())
          .arg(report.value("total_eligible").toInt())
          .arg(report.value("total_observed").toInt()));
      QStringList details;
      const auto queue = report.value("queue").toArray();
      rebuild_sequence(report);
      for (const auto & entry : queue) {
        const auto item = entry.toObject();
        details << QString("Queue %1: %2 [%3] → %4")
            .arg(item.value("order").toInt())
            .arg(item.value("object_id").toString(), item.value("class_id").toString(),
                 item.value("destination_region_ref").toString());
      }
      for (const auto & entry : report.value("observed_objects").toArray()) {
        const auto item = entry.toObject();
        if (item.value("role").toString() == "NON_TARGET_OBSTACLE") {
          details << QString("Not picked: %1 — %2 (retained for future collision checks)")
              .arg(item.value("object_id").toString(), item.value("reason").toString());
        }
      }
      details << "Read-only commissioning preview. No live simulator, EPD, MoveIt plan or robot motion.";
      batch_details->setPlainText(details.join("\n"));
    });
    process->start("python3", {script, "--scene", original_scene, "--limit", "20"});
  });
  auto * check_row = new QHBoxLayout;
  auto * check_button = new QPushButton("Check plans (no motion)", this);
  check_button->setObjectName("taskSimulationBatchPlanCheck");
  check_button->setEnabled(false);
  check_button->setToolTip("Plans every queued object with MoveIt in a private fake-hardware scene. Nothing moves.");
  auto * cancel_button = new QPushButton("Cancel check", this);
  cancel_button->setObjectName("taskSimulationBatchPlanCancel");
  cancel_button->setEnabled(false);
  check_row->addWidget(check_button);
  check_row->addWidget(cancel_button);
  perception->addRow(check_row);
  auto * check_help = new QLabel("Check plans starts a private fake-hardware MoveIt scene for each queued object "
    "(about half a minute each) with every recorded object in place as an obstacle. It plans only: the robot is "
    "never moved and results are not saved into the task.", this);
  check_help->setWordWrap(true);
  perception->addRow(check_help);
  connect(check_button, &QPushButton::clicked, this, [this] { start_plan_check(false); });
  auto * order_label = new QLabel("Pick sequence — tick the objects to pick, drag to set the order, and choose "
                                  "where each one goes. Saved with the task.", this);
  order_label->setWordWrap(true);
  perception->addRow(order_label);
  auto * order = new QListWidget(this);
  order->setObjectName("taskSequenceOrder");
  order->setDragDropMode(QAbstractItemView::InternalMove);
  order->setDefaultDropAction(Qt::MoveAction);
  order->setMaximumHeight(130);
  perception->addRow(order);
  auto * sequence_place = new QComboBox(this);
  sequence_place->setObjectName("taskSequencePlace");
  sequence_place->setEnabled(false);
  sequence_place->setToolTip("Automatic gives the object the next free place when the sequence is checked, after "
                             "places chosen for other objects are reserved. A numbered place is a fixed spot in the "
                             "destination with room for this object, its clearance and the planner's placement tolerance.");
  perception->addRow("Place for selected object:", sequence_place);
  auto * place_detail = new QLabel(this);
  place_detail->setObjectName("taskSequencePlaceDetail");
  place_detail->setWordWrap(true);
  place_detail->setTextInteractionFlags(Qt::TextSelectableByMouse);
  perception->addRow(place_detail);
  auto * summary = new QLabel("Preview the queue to build a pick sequence.", this);
  summary->setObjectName("taskSequenceSummary");
  summary->setWordWrap(true);
  perception->addRow(summary);
  connect(order, &QListWidget::currentItemChanged, this, [this](QListWidgetItem *) { show_selected_place(); });
  connect(sequence_place, qOverload<int>(&QComboBox::activated), this, [this, order, sequence_place](int) {
    auto * item = order->currentItem();
    if (!item || sequence_loading_) return;
    const QString id = sequence_place->currentData().toString();
    if (id == "custom") return;  // Keeps the saved custom place.
    QJsonObject pose;
    for (const auto & value : item->data(kPlaceOptionsRole).toJsonArray())
      if (value.toObject().value("id").toString() == id) pose = value.toObject().value("local_pose").toObject();
    item->setData(kManualPoseRole, pose);
    // Only an explicit place selection may bind a local pose to a new destination.
    item->setData(kDestinationRole, pose.isEmpty() ? QJsonObject() : QJsonObject{
        {"asset_ref", QString::fromStdString(model_.place.asset_ref)},
        {"region_ref", QString::fromStdString(model_.place.region_ref)}});
    item->setData(kReviewRole, QJsonObject());  // Re-reviewed on validation.
    write_sequence();
  });
  connect(order, &QListWidget::itemChanged, this, [this](QListWidgetItem * item) {
    if (sequence_loading_ || item->data(Qt::UserRole + 5).toBool() == (item->checkState() == Qt::Checked)) return;
    item->setData(Qt::UserRole + 5, item->checkState() == Qt::Checked);
    write_sequence();
  });
  auto * sequence_button = new QPushButton("Check full sequence (no motion)", this);
  sequence_button->setObjectName("taskSimulationSequenceCheck");
  sequence_button->setEnabled(false);
  sequence_button->setToolTip("Plans the saved sequence in order. Each complete plan virtually places the object "
                              "before the next one is planned. PLANNED — NOT EXECUTED.");
  perception->addRow(sequence_button);
  connect(sequence_button, &QPushButton::clicked, this, [this] { start_plan_check(true); });
  connect(order->model(), &QAbstractItemModel::rowsMoved, this, [this] { write_sequence(); });
  connect(cancel_button, &QPushButton::clicked, this, [this, cancel_button] {
    if (!plan_check_) return;
    cancel_button->setEnabled(false);
    findChild<QLabel *>("taskSimulationBatchStatus")->setText(
      "Cancelling — stopping the private planning scene cleanly. No motion was performed.");
    plan_check_->terminate();  // SIGTERM: the checker stops its owned launch before exiting.
  });
  auto * grasp = section("How to grasp");
  choice_field(grasp, "taskGraspPolicy", "Grasp policy", "pick/grasp/policy", {"AUTO", "PREFERRED", "EXACT"});
  strategies_ = choice_field(grasp, "taskGraspIntent", "Grasp strategy", "pick/grasp/strategy_ref", {});
  number_field(grasp, "taskApproachDistance", "Approach distance", "pick/grasp/approach/distance_m", " m");
  number_field(grasp, "taskRetreatDistance", "Lift / retreat distance", "pick/grasp/lift/distance_m", " m");
  auto * where = section("Where");
  assets_ = choice_field(where, "taskPlaceAsset", "Physical target", "place/target/asset_ref", {});
  regions_ = choice_field(where, "taskPlaceRegion", "Placement region", "place/target/region_ref", {});
  {
    // Where the robot places, as the validator resolves it (physical_destination): the physical target, its
    // target-local placement region (which moves with the target) and the resulting place pose in world.
    auto * summary = new QLabel(this); summary->setObjectName("taskDestinationSummary");
    summary->setWordWrap(true); summary->setFrameShape(QFrame::StyledPanel);
    summary->setAlignment(Qt::AlignTop | Qt::AlignLeft);
    summary->setTextInteractionFlags(Qt::TextSelectableByMouse);
    summary->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);  // wraps, never widens the panel
    where->addRow(summary);
    auto * show = new QPushButton("Show physical target in 3D", this); show->setObjectName("taskShowDestination");
    show->setToolTip("Select the task's physical target in Product View and the Inspector.");
    where->addRow(show);
    connect(show, &QPushButton::clicked, this, [this] {
      const auto target = physical_destination(QString::fromStdString(model_.place.asset_ref));
      if (!target.isEmpty()) emit show_target_requested(target["target_layout"].toString());
    });
  }
  {
    // Class routing (routing.mode class): a rule table sending each detected class to an authored placement
    // region. Many classes may share a region; a class without an enabled rule follows the unrouted policy.
    // No rows = direct pick/place into the placement region above.
    auto * box = new QWidget(this);
    auto * column = new QVBoxLayout(box); column->setContentsMargins(0, 0, 0, 0);
    routing_table_ = new QTableWidget(0, 4, box); routing_table_->setObjectName("taskRoutingTable");
    routing_table_->setHorizontalHeaderLabels({"Class", "Destination region", "On", "Priority"});
    // The destination takes the remaining width: region names (e.g. a bin's left/right compartment) must stay distinct.
    routing_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    routing_table_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
    routing_table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Interactive);
    routing_table_->horizontalHeader()->resizeSection(0, 100);
    routing_table_->verticalHeader()->setVisible(false);
    routing_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    routing_table_->setSelectionMode(QAbstractItemView::SingleSelection);
    routing_table_->setMinimumHeight(140);
    routing_table_->setToolTip("Each class is placed only in its own region. Classes come from the perception "
                               "vocabulary; regions from this cell. Higher priority is picked first.");
    column->addWidget(routing_table_);
    auto * buttons = new QHBoxLayout;
    auto * add = new QPushButton("Add rule", box); add->setObjectName("taskRoutingAdd");
    auto * remove = new QPushButton("Delete rule", box); remove->setObjectName("taskRoutingDelete");
    buttons->addWidget(add); buttons->addWidget(remove); buttons->addStretch(1);
    column->addLayout(buttons);
    // Full width: class and region names must stay readable in the narrow task panel.
    where->addRow(new QLabel("Route by class", this));
    where->addRow(box);
    unrouted_policy_ = new QComboBox(this); unrouted_policy_->setObjectName("taskUnroutedPolicy");
    unrouted_policy_->addItem("Leave in place (not picked)", "leave_in_place");
    unrouted_policy_->addItem("Stop the sorting session", "stop_session");
    where->addRow("Unrouted objects", unrouted_policy_);
    max_attempts_ = new QSpinBox(this); max_attempts_->setObjectName("taskMaxAttempts");
    max_attempts_->setRange(1, 5); max_attempts_->setValue(2);
    where->addRow("Attempts per object", max_attempts_);
    readers_["routing"] = [this](const QJsonValue & v) { fill_routing(v.toObject()); };
    connect(add, &QPushButton::clicked, this, [this] {
      add_routing_row(QJsonObject{{"when", QJsonObject{{"attribute", "class"}, {"equals", QString()}}},
                                  {"destination", regions_->currentText()}});
      write_routing();
    });
    connect(remove, &QPushButton::clicked, this, [this] {
      const int row = routing_table_->currentRow();
      if (row < 0) return;
      routing_table_->removeRow(row);
      write_routing();
    });
    connect(unrouted_policy_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this] { write_routing(); });
    connect(max_attempts_, qOverload<int>(&QSpinBox::valueChanged), this, [this] { write_routing(); });
  }
  auto * place = section("How to place");
  choice_field(place, "taskPlacePolicy", "Placement policy", "place/placement/policy", {"AUTO", "PREFERRED", "EXACT"});
  vector_field(place, "taskLocalXYZ", "Requested local XYZ (m)", "place/placement/requested_local_pose/xyz_m");
  vector_field(place, "taskLocalRPY", "Requested local RPY (rad)", "place/placement/requested_local_pose/rpy_rad");
  number_field(place, "taskPlaceClearance", "Placement clearance", "place/placement/clearance_m", " m");
  number_field(place, "taskPlaceApproach", "Place approach", "place/placement/approach/distance_m", " m");
  number_field(place, "taskPlaceRetreat", "Place retreat", "place/placement/retreat/distance_m", " m");
  // Explicit consent for the qualified physics simulator; the executor refuses a backend the saved task does not name.
  auto * run = section("Run");
  auto * backend = new QComboBox(this); backend->setObjectName("taskExecutionBackend");
  backend->addItem("MoveIt fake hardware (plan / preview)", "fake");
  backend->addItem("Qualified physics simulation (Gazebo)", "simulator");
  run->addRow("Execution backend", backend);
  readers_["safety/execution_backend"] = [backend](const QJsonValue & v) {
    const auto value = v.toString("fake");
    if (backend->findData(value) < 0) backend->addItem(value, value);  // never relabel an unknown authored value
    backend->setCurrentIndex(backend->findData(value));
  };
  connect(backend, qOverload<int>(&QComboBox::currentIndexChanged), this, [this, backend](int) {
    const auto value = backend->currentData().toString();
    set_field("safety/execution_backend", value);
    set_field("safety/require_fake_hardware", value == "fake");
  });
  // Wraps: unwrapped, this sentence forced a 633 px minimum on the whole Scene Builder inspector.
  auto * physics_note = new QLabel("Physics simulation runs only from an explicit cycle action after its safety gates; real robot locked.", this);
  physics_note->setWordWrap(true);
  run->addRow(physics_note);
  auto * help = new QLabel("AUTO chooses a valid strategy / the region's authored placement. PREFERRED tries your request first and reports any fallback. EXACT keeps your request and blocks planning if it cannot be satisfied. In a full-sequence check, AUTO and PREFERRED give each object its own free place in the region; EXACT sends every object to the same place. You can also choose a place for each object under Pick sequence. Local poses use the physical target's frame; values are never clamped.", this);
  help->setWordWrap(true); layout->addWidget(help);
  auto * advanced = new QGroupBox("Advanced task settings", this);
  advanced->setCheckable(true); advanced->setChecked(false); layout->addWidget(advanced);
  auto * advanced_layout = new QVBoxLayout(advanced);
  auto * fields = new QWidget(advanced); auto * form = new QFormLayout(fields);
  advanced_layout->addWidget(fields); fields->hide();
  connect(advanced, &QGroupBox::toggled, fields, &QWidget::setVisible);
  text_field(form, "taskSource", "Source reference", "pick/selection/source_ref");
  text_field(form, "taskSourceType", "Source type", "pick/selection/source_type");
  number_field(form, "taskMinimumConfidence", "Minimum confidence (−1 = unset)", "pick/selection/object_filter/min_confidence", "", true);
  number_field(form, "taskMaximumAge", "Maximum observation age", "pick/selection/object_filter/max_age_seconds", " s", true);
  text_field(form, "taskGraspAxis", "Grasp approach axis", "pick/grasp/approach/axis");
  text_field(form, "taskLiftAxis", "Lift axis", "pick/grasp/lift/axis");
  text_field(form, "taskOrientation", "Grasp orientation", "pick/grasp/orientation/mode");
  vector_field(form, "taskRollAngles", "Allowed rolls (deg)", "pick/grasp/orientation/allowed_roll_deg", 0);
  vector_field(form, "taskYawAngles", "Allowed yaws (deg)", "pick/grasp/orientation/allowed_yaw_deg", 0);
  vector_field(form, "taskTolerance", "Grasp tolerance (rad)", "pick/grasp/orientation/tolerance_rad");
  vector_field(form, "taskTcpXYZ", "TCP offset XYZ (m)", "pick/grasp/tcp_offset_xyz_m");
  vector_field(form, "taskTcpRPY", "TCP offset RPY (rad)", "pick/grasp/tcp_offset_rpy_rad");
  auto * contact = new QCheckBox(this); contact->setObjectName("taskContactRequired");
  form->addRow("Contact required", contact);
  readers_["pick/grasp/contact/required"] = [contact](const QJsonValue & v) { contact->setChecked(v.toBool()); };
  connect(contact, &QCheckBox::toggled, this, [this](bool value) { set_field("pick/grasp/contact/required", value); });
  number_field(form, "taskContactQuality", "Minimum contact quality", "pick/grasp/contact/min_quality", "");
  number_field(form, "taskApertureMin", "Minimum aperture", "pick/grasp/aperture/min_m", " m");
  number_field(form, "taskApertureMax", "Maximum aperture", "pick/grasp/aperture/max_m", " m");
  text_field(form, "taskPlaceOrientation", "Placement orientation", "place/placement/orientation/mode");
  vector_field(form, "taskPlaceOrientationRPY", "Placement orientation RPY (rad)", "place/placement/orientation/rpy_rad");
  vector_field(form, "taskPlaceTolerance", "Placement tolerance (rad)", "place/placement/orientation/tolerance_rad");
  text_field(form, "taskPlaceApproachAxis", "Place approach axis", "place/placement/approach/axis");
  text_field(form, "taskPlaceRetreatAxis", "Place retreat axis", "place/placement/retreat/axis");
  form->addRow(new QLabel("Release: installed tool release. Real robot locked.", this));
  status_ = new QLabel(this); status_->setObjectName("taskValidationStatus"); status_->setWordWrap(true);
  // Long identifiers in the status cannot wrap; they must not set the whole inspector's minimum width.
  status_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  layout->addWidget(status_);
  auto * buttons = new QHBoxLayout;
  auto * save_button = new QPushButton("Save task", this); save_button->setObjectName("taskSave");
  auto * validate_button = new QPushButton("Validate task", this); validate_button->setObjectName("taskValidate");
  auto * revert_button = new QPushButton("Revert", this); revert_button->setObjectName("taskRevert");
  revert_button->setToolTip("Discard unsaved task changes and reload the saved task.");
  revert_button->setEnabled(false);
  buttons->addWidget(save_button); buttons->addWidget(revert_button); buttons->addWidget(validate_button); layout->addLayout(buttons);
  connect(save_button, &QPushButton::clicked, this, [this] { save(); });
  connect(revert_button, &QPushButton::clicked, this, [this] { revert(); });
  connect(validate_button, &QPushButton::clicked, this, &TaskIntentEditor::validate_now);
  validation_timer_ = new QTimer(this); validation_timer_->setSingleShot(true);
  connect(validation_timer_, &QTimer::timeout, this, &TaskIntentEditor::validate_now);
  // The editor lives in a ~400 px inspector: choices size to a readable minimum rather than to their
  // longest entry (the full text stays in the drop-down list).
  for (auto * combo : findChildren<QComboBox *>()) {
    combo->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
    combo->setMinimumContentsLength(14);
  }
}

void TaskIntentEditor::text_field(QFormLayout * form, const QString & name, const QString & label, const QString & path)
{
  auto * edit = new QLineEdit(this); edit->setObjectName(name); form->addRow(label, edit);
  readers_[path] = [edit](const QJsonValue & v) { edit->setText(v.toString()); };
  connect(edit, &QLineEdit::textEdited, this, [this, path](const QString & value) { set_field(path, value); });
}
void TaskIntentEditor::number_field(QFormLayout * form, const QString & name, const QString & label, const QString & path, const QString & unit, bool nullable)
{
  auto * edit = new QDoubleSpinBox(this); edit->setObjectName(name);
  edit->setDecimals(12); edit->setRange(-1e12, 1e12); edit->setSuffix(unit); form->addRow(label, edit);
  readers_[path] = [edit](const QJsonValue & v) { edit->setValue(v.isDouble() ? v.toDouble() : -1.0); };
  connect(edit, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this, path, nullable](double value) {
    set_field(path, nullable && value == -1.0 ? QJsonValue(QJsonValue::Null) : QJsonValue(value));
  });
}
QComboBox * TaskIntentEditor::choice_field(QFormLayout * form, const QString & name, const QString & label, const QString & path, const QStringList & choices)
{
  auto * edit = new QComboBox(this); edit->setObjectName(name); edit->setEditable(choices.isEmpty());
  edit->addItems(choices); form->addRow(label, edit);
  readers_[path] = [edit](const QJsonValue & v) {
    const auto text = v.toString();
    if (edit->findText(text) < 0) edit->addItem(text);
    edit->setCurrentText(text);
  };
  connect(edit, &QComboBox::currentTextChanged, this, [this, path](const QString & value) {
    set_field(path, value.isEmpty() ? QJsonValue(QJsonValue::Null) : QJsonValue(value));
  });
  return edit;
}
void TaskIntentEditor::vector_field(QFormLayout * form, const QString & name, const QString & label, const QString & path, int size)
{
  auto * edit = new QLineEdit(this); edit->setObjectName(name); edit->setPlaceholderText(size == 3 ? "x, y, z" : "comma-separated angles"); form->addRow(label, edit);
  readers_[path] = [edit](const QJsonValue & v) {
    QStringList values; for (const auto & item : v.toArray()) values << QString::number(item.toDouble(), 'g', 17);
    edit->setText(values.join(", "));
  };
  connect(edit, &QLineEdit::textEdited, this, [this, path, size, label](const QString & text) {
    const auto values = text.split(','); QJsonArray array;
    bool valid = (size == 0 || values.size() == size);
    for (const auto & part : values) { bool ok = false; double value = part.trimmed().toDouble(&ok); valid &= ok && std::isfinite(value); array.append(value); }
    if (!valid) {
      input_errors_[path] = label + ": enter finite comma-separated numbers.";
      dirty_ = true; status_->setText("BLOCKED — " + input_errors_[path]); emit edited(); return;
    }
    input_errors_.remove(path); set_field(path, array);
  });
}

void TaskIntentEditor::set_field(const QString & path, const QJsonValue & value)
{
  if (loading_ || model_.authored_yaml.empty()) return;
  if (path == "pick/selection/object_filter/max_age_seconds" && !clamping_age_) { age_follows_mode_ = false; input_note_.clear(); }
  std::vector<std::string> keys; for (const auto & key : path.split('/')) keys.push_back(key.toStdString());
  try { model_.set_field(keys, json_value(value).toStdString()); }
  catch (const std::exception & exc) { error_ = QString::fromUtf8(exc.what()); status_->setText("BLOCKED — " + error_); return; }
  if (path == "pick/selection/zone_ref" && findChild<QComboBox *>("taskPerceptionMode")->currentData().toString() == "off") {
    model_.set_field({"pick", "selection", "source_ref"}, json_value(value).toStdString());
  }
  dirty_ = true; error_.clear();
  findChild<QPushButton *>("taskCameraCapture")->setEnabled(false);
  findChild<QPushButton *>("taskCaptureLiveEpd")->setEnabled(false);
  findChild<QPushButton *>("taskLiveScenePreflight")->setEnabled(false);
  findChild<QPushButton *>("taskSimulationBatchPreview")->setEnabled(false);
  findChild<QPushButton *>("taskSimulationBatchPlanCheck")->setEnabled(false);
  findChild<QPushButton *>("taskSimulationSequenceCheck")->setEnabled(false);
  clear_sequence_results("Task changed — earlier plan results no longer apply.");
  status_->setText("Unsaved task — validate and save before planning.");
  update_dirty_ui();
  validation_timer_->start(250); emit edited();
}
void TaskIntentEditor::refresh_fields()
{
  loading_ = true;
  const auto root = QJsonDocument::fromJson(QByteArray::fromStdString(workcell_builder::canonical_task_intent_json_for_testing(model_.to_yaml()))).object();
  for (auto it = readers_.begin(); it != readers_.end(); ++it) it.value()(value_at(root, it.key()));
  const bool live = findChild<QComboBox *>("taskPerceptionMode")->currentData().toString() == "live_epd";
  findChild<QComboBox *>("taskCameraSource")->setEnabled(live);
  const bool camera_ready = live && !dirty_ &&
    !findChild<QComboBox *>("taskCameraSource")->currentData().toString().isEmpty();
  findChild<QPushButton *>("taskCameraCapture")->setEnabled(camera_ready);
  findChild<QPushButton *>("taskCaptureLiveEpd")->setEnabled(camera_ready);
  findChild<QPushButton *>("taskLiveScenePreflight")->setEnabled(camera_ready);
  const QString mode = findChild<QComboBox *>("taskPerceptionMode")->currentData().toString();
  const bool batch_ready = !dirty_ && (mode == "off" || mode == "replayed_snapshot") &&
      !findChild<QLineEdit *>("taskReplayPath")->text().trimmed().isEmpty();
  findChild<QPushButton *>("taskSimulationBatchPreview")->setEnabled(batch_ready);
  findChild<QPushButton *>("taskSimulationBatchPlanCheck")->setEnabled(batch_ready && !plan_check_);
  findChild<QPushButton *>("taskSimulationSequenceCheck")->setEnabled(batch_ready && !plan_check_);
  loading_ = false;
  refresh_input_status(); refresh_destination_summary();
}
QString TaskIntentEditor::stored_input_path(const QString & path) const
{
  // For a cell inside this repository, a file of the same repository is saved relative to the cell, so the task
  // names the same file in every checkout and runtime view; any other file keeps its absolute path.
  const QString root = repository_root(scene_, helper_);
  const QString file = QFileInfo(path).absoluteFilePath();
  const bool same_repository = !root.isEmpty() && file.startsWith(root + "/") &&
    QDir(scene_).absolutePath().startsWith(root + "/");
  return same_repository ? QDir(scene_).relativeFilePath(file) : file;
}
void TaskIntentEditor::refresh_recordings()
{
  auto * combo = findChild<QComboBox *>("taskObservationRecording");
  const bool live = findChild<QComboBox *>("taskPerceptionMode")->currentData().toString() == "live_epd";
  const QString root = repository_root(scene_, helper_);
  // The cell's own recordings and saved EPD captures first, then the repository's reviewed commissioning
  // recordings. A mode lists only its own kind: a recording is never offered as live input, nor a capture as a recording.
  QStringList files;
  if (!scene_.isEmpty()) {
    QDirIterator cell(scene_, {"*.yaml", "*.yml", "*.json"}, QDir::Files, QDirIterator::Subdirectories);
    while (cell.hasNext()) {
      const QString file = cell.next(), relative = QDir(scene_).relativeFilePath(file);
      // Of an EPD capture, only its qualified result is task input (the raw ROS capture is kept for diagnosis).
      if (relative.startsWith("generated/") || relative.startsWith("assets/") || relative.startsWith("launch/") ||
          (relative.startsWith("captures/") && QFileInfo(file).fileName() != "qualified_detected_objects.yaml")) continue;
      files << file;
    }
  }
  if (!root.isEmpty())
    for (const auto & info : QDir(root + "/config/runtime").entryInfoList({"*.yaml", "*.yml", "*.json"}, QDir::Files, QDir::Name))
      files << info.absoluteFilePath();
  struct Entry { QString stored, label, tip; double stamp; };
  QList<Entry> entries;
  for (const auto & file : files) {
    const auto observed = read_observation_file(file);
    if (!observed.ok || observed.objects == 0 || (observed.mode == "live_epd") != live ||
        (!live && observed.mode != "replayed_snapshot"))
      continue;
    const QString stored = stored_input_path(file);
    const QString shown = !root.isEmpty() && file.startsWith(root + "/") && !file.startsWith(QDir(scene_).absolutePath() + "/")
      ? QDir(root).relativeFilePath(file) : QDir(scene_).relativeFilePath(file);
    QString label = QString("%1 — %2 object(s)%3").arg(shown).arg(observed.objects)
      .arg(observed.classes.isEmpty() ? QString() : ": " + observed.classes.join(", "));
    if (live && observed.stamp_s > 0)
      label += ", captured " + elapsed_text(QDateTime::currentMSecsSinceEpoch() / 1000.0 - observed.stamp_s) + " ago";
    entries.append({stored, label, file + (observed.description.isEmpty() ? QString() : "\n" + observed.description), observed.stamp_s});
  }
  if (live) std::stable_sort(entries.begin(), entries.end(), [](const Entry & a, const Entry & b) { return a.stamp > b.stamp; });
  const QSignalBlocker blocker(combo);
  combo->clear();
  combo->addItem(entries.isEmpty()
    ? (live ? "No saved EPD capture — use Capture live EPD objects" : "No recording found — use Choose observation file…")
    : (live ? "Choose a saved EPD capture…" : "Choose a recording…"), QString());
  for (const auto & entry : entries) {
    combo->addItem(entry.label, entry.stored);
    combo->setItemData(combo->count() - 1, entry.tip, Qt::ToolTipRole);
  }
  refresh_input_status();
}
void TaskIntentEditor::follow_mode_freshness(const QJsonObject & report)
{
  // After a mode change, an age above the new source's freshness limit is lowered to that limit (as the
  // validator states it); an explicit age edit, or an age already within the limit, is never changed.
  if (!age_follows_mode_) return;
  age_follows_mode_ = false;
  static const QRegularExpression limit(
    R"(^OBSERVATION_FRESHNESS_POLICY: object_filter\.max_age_seconds (\S+) must be in \(0, ([0-9.]+)\] s for (\S+) observations)");
  for (const auto & value : report["errors"].toArray()) {
    const auto match = limit.match(value.toString());
    bool current_ok = false, limit_ok = false;
    const double current = match.captured(1).toDouble(&current_ok), ceiling = match.captured(2).toDouble(&limit_ok);
    if (!match.hasMatch() || !current_ok || !limit_ok || !(current > ceiling) || !(ceiling > 0)) continue;
    clamping_age_ = true;
    set_field("pick/selection/object_filter/max_age_seconds", ceiling);
    clamping_age_ = false;
    refresh_fields();
    input_note_ = QString("Maximum observation age lowered from %1 s to %2 s: the freshness limit for this input "
                          "(%3). Advanced task settings can lower it further.").arg(match.captured(1), match.captured(2), match.captured(3));
    refresh_input_status();
    return;
  }
}
void TaskIntentEditor::refresh_input_status()
{
  auto * label = findChild<QLabel *>("taskInputStatus");
  auto * help = findChild<QLabel *>("taskInputModeHelp");
  auto * combo = findChild<QComboBox *>("taskObservationRecording");
  if (!label || !help || !combo) return;
  const QString mode = findChild<QComboBox *>("taskPerceptionMode")->currentData().toString();
  const bool live = mode == "live_epd";
  help->setText(
    mode == "off" ? "Off: the task's objects are measured by the physics simulator, which is locked for new cells. "
                    "To plan on fake hardware (Plan Only), choose Recorded observations and a commissioning recording."
    : mode == "replayed_snapshot" ? "Recorded observations: plan from a saved recording of measured objects "
                    "(detected_objects/v1, source replayed_snapshot), such as the reviewed commissioning recording. No "
                    "camera is needed. Planning re-times the recording when it starts; it never counts as a live observation."
    : live ? "Live EPD: plan from current EPD observations of the selected camera. Start EPD, then press Capture live "
             "EPD objects. The capture must declare that camera, give world-frame poses (TF) and still be fresh when "
             "planning starts; a recording is never used instead. This setting does not start a camera."
    : "Choose how the task receives its objects.");
  const QString stored = findChild<QLineEdit *>("taskReplayPath")->text().trimmed();
  {
    const QSignalBlocker blocker(combo);
    const int index = stored.isEmpty() ? 0 : combo->findData(stored);
    combo->setCurrentIndex(index >= 0 ? index : 0);
  }
  QStringList lines, input_classes;
  if (stored.isEmpty()) {
    lines << "Input: none selected.";
  } else {
    const QString file = QDir(scene_).absoluteFilePath(QDir::fromNativeSeparators(stored));
    const auto observed = read_observation_file(file);
    if (!observed.ok) {
      lines << "Input: " + stored + " — " + observed.problem + ".";
    } else {
      lines << "Input: " + stored;
      QString source = "Source: " + (observed.mode.isEmpty() ? QString("undeclared") : observed.mode);
      if (!observed.camera.isEmpty()) source += " · camera " + observed.camera;
      source += QString(" · %1 object(s)").arg(observed.objects);
      input_classes = observed.classes;
      if (!observed.classes.isEmpty()) source += ": " + observed.classes.join(", ");
      lines << source;
      if (!observed.description.isEmpty()) lines << observed.description;
      if (observed.mode == "replayed_snapshot") {
        lines << "Freshness: a recording is re-timed when planning starts.";
      } else if (observed.stamp_s > 0) {
        const auto age = value_at(QJsonDocument::fromJson(QByteArray::fromStdString(
          workcell_builder::canonical_task_intent_json_for_testing(model_.to_yaml()))).object(),
          "pick/selection/object_filter/max_age_seconds");
        lines << QString("Freshness: captured %1 (%2 ago); planning accepts observations at most %3 s old.")
          .arg(QDateTime::fromMSecsSinceEpoch(static_cast<qint64>(observed.stamp_s * 1000)).toString("yyyy-MM-dd HH:mm:ss"),
               elapsed_text(QDateTime::currentMSecsSinceEpoch() / 1000.0 - observed.stamp_s),
               age.isDouble() ? QString::number(age.toDouble()) : QString("—"));
      }
    }
  }
  const bool checking = validation_timer_ && validation_timer_->isActive();
  if (checking) {
    lines << "Plan Only: checking the input…";
  } else if (!input_errors_.isEmpty() || !error_.isEmpty()) {
    const QString first = !input_errors_.isEmpty() ? input_errors_.first() : error_.section('\n', 0, 0);
    lines << "Plan Only: blocked — " + first;
    if (first.startsWith("OBJECT_CLASS_REQUIRED"))
      lines << "Next: type the Object class to pick" +
               (input_classes.isEmpty() ? QString(".") : " (this input contains: " + input_classes.join(", ") + ").");
    else
      lines << "Next: fix the errors listed in the task status below, then Save task.";
  } else if (!input_blocker_.isEmpty()) {
    lines << "Plan Only: blocked — " + input_blocker_;
    if (session_observed(report_))
      lines << "Next: run Plan / Simulate › Start Sorting (each cycle observes the cell), or save a capture for Plan Saved Task.";
    else
      lines << "Next: " + next_input_action(input_blocker_, mode);
  } else if (report_.contains("planning_input")) {
    lines << (dirty_ ? "Plan Only: this input passes the planner's input check — Save task to use it."
                     : "Plan Only: input ready — the saved input passes the planner's input check.")
          << (dirty_ ? "Next: Save task." : "Next: Generate and Validate the cell, then Plan / Simulate › Plan Saved Task.");
  }
  if (!input_note_.isEmpty()) lines << input_note_;
  label->setText(lines.join("\n"));
}
QString TaskIntentEditor::producer_blocker(const QJsonObject & report) const
{
  // planning_input does not check which producer may feed the task's source; the executor does at
  // FILTER_TARGETS (task_intent_v2.observation_age_cap). On fake hardware a saved recording is replayed and
  // anything else is a live sensor, so the same check runs here on the same function.
  const auto input = report["planning_input"].toObject();
  const auto intent = report["task_intent"].toObject();
  if (input.isEmpty() || intent.isEmpty() ||
      intent["safety"].toObject()["execution_backend"].toString("fake") != "fake") return QString();
  const QString scripts = QFileInfo(QFileInfo(helper_).canonicalFilePath()).absolutePath();
  QProcess process;
  process.start("python3", {"-c",
    "import json, sys\n"
    "sys.path.insert(0, sys.argv[1])\n"
    "from task_intent_v2 import freshness_source_type, observation_age_cap\n"
    "request = json.load(sys.stdin)\n"
    "try:\n"
    "    observation_age_cap(freshness_source_type(request['intent']), request['producer'])\n"
    "except ValueError as exc:\n"
    "    print(exc)\n", scripts});
  if (!process.waitForStarted(3000)) return "OBSERVATION_PRODUCER_UNCHECKED: the producer check could not start.";
  process.write(QJsonDocument(QJsonObject{{"intent", intent},
    {"producer", input["replay"].toBool() ? "replay" : "live_sensor"}}).toJson(QJsonDocument::Compact));
  process.closeWriteChannel();
  if (!process.waitForFinished(10000) || process.exitStatus() != QProcess::NormalExit || process.exitCode() != 0) {
    process.kill(); process.waitForFinished(1000);
    return "OBSERVATION_PRODUCER_UNCHECKED: " + QString::fromUtf8(process.readAllStandardError()).trimmed().right(300);
  }
  return QString::fromUtf8(process.readAllStandardOutput()).trimmed();
}
QJsonObject TaskIntentEditor::request(bool draft)
{
  QProcess process; QStringList args{helper_, scene_}; if (draft) args << "--stdin";
  process.start("python3", args);
  if (!process.waitForStarted(3000)) throw std::runtime_error("Task validation helper could not start");
  if (draft) process.write(QByteArray::fromStdString(workcell_builder::canonical_task_intent_json_for_testing(model_.to_yaml())));
  process.closeWriteChannel();
  if (!process.waitForFinished(10000)) { process.kill(); process.waitForFinished(1000); throw std::runtime_error("Task validation timed out"); }
  const auto report = QJsonDocument::fromJson(process.readAllStandardOutput()).object();
  if (report.isEmpty()) throw std::runtime_error(("Task validation failed: " + process.readAllStandardError()).toStdString());
  return report;
}
bool TaskIntentEditor::confirm_scene_change(const QString & scene)
{
  if (scene == scene_ || !dirty_) return true;
  const auto answer = QMessageBox::question(this, "Unsaved task",
    "Save task edits before switching cells?",
    QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel, QMessageBox::Save);
  if (answer == QMessageBox::Save) return save();
  if (answer != QMessageBox::Discard) return false;
  dirty_ = false;  // Explicit discard reloads the source even if navigation is later canceled.
  model_ = {};
  return load_scene(scene_, helper_);
}
bool TaskIntentEditor::load_scene(const QString & scene, const QString & helper)
{
  if (!confirm_scene_change(scene)) return false;
  if (scene == scene_ && !model_.authored_yaml.empty()) {
    if (dirty_) return true;
    QFile file(scene_ + "/config/workcell_builder_task_intent.yaml");
    const auto current = file.open(QIODevice::ReadOnly) ? QString::fromUtf8(file.readAll()) : QString();
    if (current == loaded_bytes_) { setEnabled(true); return true; }
  }
  if (scene != scene_) { stop_plan_check(90000); geometry_unsaved_ = false; }
  scene_ = scene; helper_ = helper; dirty_ = false; error_.clear(); input_blocker_.clear(); input_errors_.clear();
  age_follows_mode_ = false; input_note_.clear();
  findChild<QLabel *>("taskCameraRgbPreview")->setPixmap(QPixmap());
  findChild<QLabel *>("taskCameraRgbPreview")->setText("RGB — no capture");
  findChild<QLabel *>("taskCameraDepthPreview")->setPixmap(QPixmap());
  findChild<QLabel *>("taskCameraDepthPreview")->setText("Depth — no capture");
  findChild<QLabel *>("taskCameraCaptureStatus")->setText("Save and select a live camera to inspect existing ROS streams.");
  findChild<QLabel *>("taskCaptureLiveEpdStatus")->setText("A saved live task and a running EPD localization/tracking topic are required.");
  findChild<QLabel *>("taskLiveScenePreflightStatus")->setText("Check after saving and generating your live task.");
  findChild<QLabel *>("taskSimulationBatchStatus")->setText(
      "Select an explicit commissioning recording and Save Task to preview the queue.");
  findChild<QPlainTextEdit *>("taskSimulationBatchDetails")->setPlainText("No batch preview has been run.");
  sequence_loading_ = true; findChild<QListWidget *>("taskSequenceOrder")->clear(); sequence_loading_ = false;
  sequence_rebind_pending_ = false;
  validation_timer_->stop(); model_ = {};
  try {
    QFile file(scene_ + "/config/workcell_builder_task_intent.yaml");
    loaded_bytes_ = file.open(QIODevice::ReadOnly) ? QString::fromUtf8(file.readAll()) : QString();
    auto report = request(false);
    if (!report.contains("task_intent")) throw std::runtime_error(report["errors"].toArray().first().toString().toStdString());
    model_ = workcell_builder::TaskIntentModel::from_yaml(QJsonDocument(report["task_intent"].toObject()).toJson().toStdString());
    loading_ = true;
    for (auto pair : {qMakePair(strategies_, QString("strategies")), qMakePair(assets_, QString("assets")), qMakePair(pick_zones_, QString("pick_zones"))}) {
      pair.first->clear(); for (const auto & value : report[pair.second].toArray()) pair.first->addItem(value.toString());
    }
    regions_->clear(); regions_->addItems(report["regions"].toObject().keys());
    region_ids_ = report["regions"].toObject().keys();
    report_ = report;                      // routing rows read the region -> target map while the fields load
    vocabulary_.clear();
    for (const auto & value : report["class_vocabulary"].toObject()["classes"].toArray()) vocabulary_ << value.toString();
    strategies_->setEditable(false);
    refresh_fields(); refresh_recordings(); setEnabled(true); apply_report(report); rebuild_sequence(report); update_dirty_ui(); return true;
  } catch (const std::exception & exc) {
    error_ = QString::fromUtf8(exc.what()); status_->setText("BLOCKED — " + error_); return false;
  }
}
void TaskIntentEditor::apply_report(const QJsonObject & report)
{
  report_ = report;
  QStringList messages; for (const auto & value : report["errors"].toArray()) messages << value.toString();
  {
    // Generate exports the pick class as the perception input's required class and refuses observation input
    // without one (export_builder_scene_to_cell_definition, generate_workcell_from_cell_definition).
    const auto intent = report["task_intent"].toObject();
    const QString perception = intent["perception"].toObject()["mode"].toString();
    const auto routing = intent["routing"].toObject();
    const bool routed = routing["mode"].toString() == "class" && !routing["rules"].toArray().isEmpty();
    const QString pick_class = intent["pick"].toObject()["selection"].toObject()["object_filter"].toObject()["class_id"].toString();
    if ((perception == "replayed_snapshot" || perception == "live_epd") && !routed && pick_class.trimmed().isEmpty())
      messages.prepend("OBJECT_CLASS_REQUIRED: enter the Object class to pick (What / Pick eligibility); Generate needs "
                       "it for observation input.");
  }
  error_ = messages.join("\n");
  input_blocker_ = report["planning_input_blocker"].toString();
  if (input_blocker_.isEmpty()) input_blocker_ = producer_blocker(report);
  // A class-routed task bound to live EPD on the simulated camera (no saved capture) is observed by its own sorting
  // session every cycle: the missing saved input blocks Plan Saved Task only (blocker(true) keeps it), not the task.
  const bool session = session_observed(report);
  if (!input_blocker_.isEmpty())
    messages << (session ? "Input: each sorting cycle renders the cell with its camera and classifies the frame with "
                                    "EPD (Plan / Simulate > Start Sorting). Plan Saved Task needs a recorded input: " + input_blocker_
                                  : input_blocker_);
  for (const auto & value : report["warnings"].toArray()) messages << value.toString();
  const auto destination = report["resolved_destination"].toObject();
  if (!destination.isEmpty()) {
    messages << "Resolved destination in world (m): " + json_value(destination["pose_xyz"]);
  }
  const auto status = (error_.isEmpty() && (input_blocker_.isEmpty() || session))
    ? (report["warnings"].toArray().isEmpty() ? "Authoring valid" : "WARNING") : "BLOCKED";
  status_->setText(QString("%1%2\n%3\nIntent hash: %4\nPlanning still requires complete-cycle validation.")
    .arg(status, dirty_ ? " — unsaved" : "", messages.join("\n"), report["normalized_intent_sha256"].toString("unavailable")));
  if (report.contains("sequence_review")) apply_sequence_review(report["sequence_review"].toObject());
  else if (auto * summary = findChild<QLabel *>("taskSequenceSummary"); summary && dirty_ &&
           summary->text().startsWith("Unsaved changes — checking"))
    summary->setText("Unsaved changes — no sequence: the check will use the whole queue, all automatic.");
  update_dirty_ui();  refresh_input_status();  refresh_destination_summary();
}
void TaskIntentEditor::validate_now()
{
  validation_timer_->stop();
  if (model_.authored_yaml.empty() || !input_errors_.isEmpty()) return;
  try {
    QFile file(scene_ + "/config/workcell_builder_task_intent.yaml");
    const auto current = file.open(QIODevice::ReadOnly) ? QString::fromUtf8(file.readAll()) : QString();
    if (current != loaded_bytes_) throw std::runtime_error(
      "Task changed on disk. Reopen before validating; unsaved edits are preserved.");
    const auto report = request(true);
    apply_report(report);
    follow_mode_freshness(report);
  }
  catch (const std::exception & exc) { error_ = QString::fromUtf8(exc.what()); status_->setText("BLOCKED — " + error_); }
}
QString TaskIntentEditor::blocker(bool require_input) const
{
  if (dirty_) return "Save task edits before planning.";
  if (!input_errors_.isEmpty()) return input_errors_.first();
  return !error_.isEmpty() ? error_ : require_input ? input_blocker_ : QString();
}
bool TaskIntentEditor::save(QString * error)
{
  try {
    if (!input_errors_.isEmpty()) throw std::runtime_error(input_errors_.first().toStdString());
    if (model_.authored_yaml.empty()) throw std::runtime_error("No task loaded. Configure or reopen the task.");
    if (sequence_rebind_pending_) write_sequence();  // Explicit Save after the changed-recording warning.
    auto report = request(true);
    validation_timer_->stop();  // this check is of the draft being saved; a pending one would repeat it
    const auto normalized = report["task_intent"].toObject();
    const QByteArray bytes = QJsonDocument(normalized).toJson(QJsonDocument::Indented);
    // Invalid EXACT is a savable draft, never a reason to normalize a different pose.
    if (normalized.isEmpty()) throw std::runtime_error("Task normalization returned no authored model");
    const auto path = scene_ + "/config/workcell_builder_task_intent.yaml";
    QFile previous(path); const auto current = previous.open(QIODevice::ReadOnly) ? QString::fromUtf8(previous.readAll()) : QString(); previous.close();
    if (current != loaded_bytes_) throw std::runtime_error("Task changed on disk. Reopen before saving to preserve the other edits.");
    QDir().mkpath(scene_ + "/config"); QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly) || file.write(bytes) != bytes.size() || !file.commit()) throw std::runtime_error(file.errorString().toStdString());
    QFile reopened(path); if (!reopened.open(QIODevice::ReadOnly) || reopened.readAll() != bytes) throw std::runtime_error("Task save/reopen verification failed");
    model_ = workcell_builder::TaskIntentModel::from_yaml(bytes.toStdString());
    loaded_bytes_ = QString::fromUtf8(bytes); dirty_ = false; refresh_fields(); apply_report(report); emit saved(); return true;
  } catch (const std::exception & exc) {
    error_ = QString::fromUtf8(exc.what()); status_->setText("Save blocked — " + error_); if (error) *error = error_; return false;
  }
}
void TaskIntentEditor::bind_pick(const QString & id)
{
  const auto mode = findChild<QComboBox *>("taskPerceptionMode")->currentData().toString();
  if (mode.isEmpty() || mode == "off") set_field("pick/selection/source_ref", id);
  set_field("pick/selection/zone_ref", id); refresh_fields();
}
void TaskIntentEditor::bind_destination(const QString & id)
{
  const auto regions = report_["regions"].toObject();
  if (regions.contains(id)) {
    set_field("place/target/region_ref", id); set_field("place/target/asset_ref", regions[id]);
  } else {
    set_field("place/target/asset_ref", id);
    QStringList matches; for (auto it = regions.begin(); it != regions.end(); ++it) if (it.value().toString() == id) matches << it.key();
    if (matches.size() == 1) set_field("place/target/region_ref", matches.first());
  }
  refresh_fields();
}

QJsonObject TaskIntentEditor::physical_destination(const QString & id) const
{
  // Roles come from the saved physical model only: an asset is a physical target when a placement region
  // names it (target_ref); a region is target-local and never an independent destination.
  if (id.trimmed().isEmpty() || scene_.isEmpty()) return {};
  const QFileInfo environment(scene_ + "/environment.yaml");
  const QString key = environment.absoluteFilePath() + "@" + QString::number(environment.lastModified().toMSecsSinceEpoch()) +
                      ":" + QString::number(environment.size());
  if (key != destinations_key_) {
    destinations_key_ = key; destinations_.clear();
    try {
      const auto root = YAML::LoadFile(environment.absoluteFilePath().toStdString());
      const auto env = root["environment"] ? root["environment"] : root;
      const auto text = [](const YAML::Node & node, const char * name) {
        return node[name] && node[name].IsScalar() ? QString::fromStdString(node[name].as<std::string>()) : QString();
      };
      QMap<QString, QStringList> regions;   // target -> its placement regions
      for (const auto & zone : env["task_zones"])
        if (!text(zone, "target_ref").isEmpty()) regions[text(zone, "target_ref")] << text(zone, "id");
      for (const auto & asset : env["assets"]) {
        const QString target = text(asset, "id");
        if (!regions.contains(target)) continue;
        const QString layout = text(asset, "layout_item_ref").isEmpty() ? target : text(asset, "layout_item_ref");
        const QJsonObject entry{{"kind", "target"}, {"target", target}, {"target_layout", layout},
                                {"regions", QJsonArray::fromStringList(regions[target])}};
        destinations_[target] = entry; destinations_[layout] = entry;
      }
      for (const auto & zone : env["task_zones"]) {
        const QString target = text(zone, "target_ref");
        if (target.isEmpty() || !destinations_.contains(target)) continue;
        auto entry = destinations_[target];
        entry["kind"] = "region"; entry["region"] = text(zone, "id");
        destinations_[text(zone, "id")] = entry;
        if (!text(zone, "layout_item_ref").isEmpty()) destinations_[text(zone, "layout_item_ref")] = entry;
      }
    } catch (const std::exception &) { destinations_.clear(); }
  }
  return destinations_.value(id.trimmed());
}

void TaskIntentEditor::set_geometry_unsaved(bool unsaved)
{
  if (geometry_unsaved_ == unsaved) return;
  geometry_unsaved_ = unsaved; refresh_destination_summary();
}

QString TaskIntentEditor::destination_summary() const
{
  const auto asset = QString::fromStdString(model_.place.asset_ref);
  const auto region = QString::fromStdString(model_.place.region_ref);
  if (asset.isEmpty() && region.isEmpty())
    return "Destination: none. Choose a Physical target, or select a bin in the 3D view and use "
           "Inspector › Use Selected as Task Destination.";
  const auto xyz = [](const QJsonValue & value) {
    const auto a = value.toArray();
    return QString("(%1, %2, %3)").arg(a[0].toDouble(), 0, 'f', 3).arg(a[1].toDouble(), 0, 'f', 3).arg(a[2].toDouble(), 0, 'f', 3);
  };
  const auto degrees = [](const QJsonValue & value) {
    const auto a = value.toArray(); const double k = 180.0 / M_PI;
    return QString("(%1°, %2°, %3°)").arg(a[0].toDouble() * k, 0, 'f', 1).arg(a[1].toDouble() * k, 0, 'f', 1).arg(a[2].toDouble() * k, 0, 'f', 1);
  };
  QStringList lines;
  const auto d = resolved_destination();
  if (d.isEmpty() || d["target_id"].toString() != asset || d["id"].toString() != region) {
    lines << QString("Physical target: %1 · Placement region: %2").arg(asset.isEmpty() ? "none" : asset, region.isEmpty() ? "none" : region);
    QString reason;
    for (const auto & value : report_["errors"].toArray()) {
      const auto error = value.toString();
      if (error.startsWith("PLACE_") || error.startsWith("R19_DESTINATION") || error.startsWith("TASK_ROUTE")) { reason = error; break; }
    }
    lines << (!reason.isEmpty() ? "Place pose: BLOCKED — " + reason
              : dirty_ ? QString("Place pose: checking the unsaved change…")
                       : QString("Place pose: not resolved — the task has errors (see below); planning stays blocked."));
    if (!reason.isEmpty())
      lines << "Next: select the destination bin in the 3D view and use Inspector › Use Selected as Task Destination, "
               "or choose a physical target and one of its placement regions above.";
  } else {
    lines << QString("Physical target: %1 — world %2 m, RPY %3").arg(asset, xyz(d["target_pose_xyz"]), degrees(d["target_pose_rpy"]));
    const auto local = d.contains("region_local") ? d["region_local"].toObject() : d["placement_local"].toObject();
    const auto size = local["dimensions"].toArray();
    lines << QString("Placement region: %1 — target-local offset %2 m, size %3 × %4 × %5 m; it moves with its target.")
               .arg(region, xyz(local["pose_xyz"])).arg(size[0].toDouble(), 0, 'f', 3).arg(size[1].toDouble(), 0, 'f', 3)
               .arg(size[2].toDouble(), 0, 'f', 3);
    const auto policy = QString::fromStdString(model_.place.policy);
    if (d.contains("region_local"))
      lines << QString("Place pose (%1): target-local %2 m → world %3 m.").arg(policy, xyz(d["placement_local"].toObject()["pose_xyz"]), xyz(d["pose_xyz"]));
    else
      lines << QString("Place pose (%1): region centre, world %2 m. With several objects the planner gives each a "
                       "free place inside the region.").arg(policy, xyz(d["pose_xyz"]));
    lines << "Every place is still checked by the collision-aware MoveIt plan.";
  }
  if (geometry_unsaved_)
    lines << "Unsaved 3D edits: these coordinates are from the saved layout. Save to update them.";
  return lines.join("\n");
}

void TaskIntentEditor::refresh_destination_summary()
{
  if (auto * label = findChild<QLabel *>("taskDestinationSummary")) label->setText(destination_summary());
  if (auto * show = findChild<QPushButton *>("taskShowDestination"))
    show->setEnabled(!physical_destination(QString::fromStdString(model_.place.asset_ref)).isEmpty());
  emit destination_changed();
}

TaskIntentEditor::~TaskIntentEditor()
{
  // Never orphan an owned MoveIt launch: let the checker shut it down before exit.
  stop_plan_check(90000);
}

bool TaskIntentEditor::stop_plan_check(int wait_ms)
{
  if (!plan_check_) return true;
  auto * process = plan_check_;
  process->disconnect(this);
  plan_check_ = nullptr;
  process->terminate();
  const bool clean = process->waitForFinished(wait_ms);
  if (!clean) { process->kill(); process->waitForFinished(5000); }
  process->deleteLater();
  if (auto * cancel = findChild<QPushButton *>("taskSimulationBatchPlanCancel")) cancel->setEnabled(false);
  return clean;
}

void TaskIntentEditor::rebuild_sequence(const QJsonObject & report)
{
  auto * order = findChild<QListWidget *>("taskSequenceOrder");
  const auto review = report.value("sequence_review").toObject();
  const bool saved = !review.isEmpty() && review.value("status").toString() != "NOT_CONFIGURED";
  const bool preview = report.contains("queue");
  QMap<QString, QJsonObject> observed;
  for (const auto & value : report.value("observed_objects").toArray())
    observed[value.toObject().value("object_id").toString()] = value.toObject();
  const auto root = QJsonDocument::fromJson(QByteArray::fromStdString(
      workcell_builder::canonical_task_intent_json_for_testing(model_.to_yaml()))).object();
  QMap<QString, QJsonObject> saved_destinations;
  for (const auto & value : value_at(root, "sequence/items").toArray()) {
    const auto item = value.toObject();
    const auto placement = item.value("placement").toObject();
    saved_destinations[item.value("object_id").toString()] = QJsonObject{
        {"asset_ref", placement.value("asset_ref")}, {"region_ref", placement.value("region_ref")}};
  }
  const QString saved_sha = value_at(root, "sequence/source_sha256").toString();
  sequence_source_sha_ = preview ? report.value("source_sha256").toString() : saved_sha;
  sequence_rebind_pending_ = saved && preview && sequence_source_sha_ != saved_sha;
  sequence_loading_ = true;
  order->clear();
  QSet<QString> listed;
  auto add = [&](const QString & id, const QJsonObject & binding, const QJsonArray & options,
                 const QJsonObject & manual, bool included, const QJsonObject & item_review) {
    auto * item = new QListWidgetItem(order);
    item->setFlags(item->flags() | Qt::ItemIsUserCheckable | Qt::ItemIsDragEnabled);
    item->setCheckState(included ? Qt::Checked : Qt::Unchecked);
    item->setData(Qt::UserRole + 5, included);
    item->setData(kObjectIdRole, id);
    item->setData(kBindingRole, binding);
    item->setData(kPlaceOptionsRole, options);
    item->setData(kManualPoseRole, manual);
    item->setData(kDestinationRole, saved_destinations.value(id));
    item->setData(kReviewRole, item_review);
    listed.insert(id);
  };
  auto current_binding = [&](const QString & id) {
    const auto o = observed.value(id);
    const auto pose = o.value("world_pose").toArray();
    return QJsonObject{{"class_id", o.value("class_id")}, {"observed_xyz_m", QJsonArray{pose[0], pose[1], pose[2]}},
                       {"dimensions_m", o.value("dimensions_m")}};
  };
  QMap<QString, QJsonArray> queue_options;
  for (const auto & value : report.value("queue").toArray())
    queue_options[value.toObject().value("object_id").toString()] = value.toObject().value("place_options").toArray();
  if (saved) {
    for (const auto & value : review.value("items").toArray()) {
      const auto item = value.toObject();
      const QString id = item.value("object_id").toString();
      // A changed recording is re-read only into the list; Save binds it (see sequence_rebind_pending_).
      const auto binding = sequence_rebind_pending_ && observed.contains(id) ? current_binding(id)
          : QJsonObject{{"class_id", item.value("class_id")}, {"observed_xyz_m", item.value("observed_xyz_m")},
                        {"dimensions_m", item.value("dimensions_m")}};
      const auto options = item.value("place_options").toArray().isEmpty() ? queue_options.value(id)
                                                                            : item.value("place_options").toArray();
      add(id, binding, options, item.value("mode").toString() == "MANUAL" ? item.value("local_pose").toObject()
                                                                          : QJsonObject(), true, item);
    }
  }
  for (const auto & value : report.value("queue").toArray()) {
    const QString id = value.toObject().value("object_id").toString();
    if (!listed.contains(id)) add(id, current_binding(id), queue_options.value(id), {}, !saved, {});
  }
  sequence_loading_ = false;
  refresh_sequence_texts();
  if (order->count()) order->setCurrentRow(0);
  auto * summary = findChild<QLabel *>("taskSequenceSummary");
  if (sequence_rebind_pending_) {
    summary->setText("The observation recording changed since this sequence was saved, so the saved objects may be "
                     "different detections. They are shown as read from the current recording: check each object "
                     "and place, then Save to bind the sequence to this recording.");
  } else if (saved) {
    apply_sequence_review(review);
  } else if (preview) {
    summary->setText(QString("No saved sequence: the check uses all %1 eligible object(s) in queue order with "
                             "automatic places. Untick, reorder or choose places, then Save, to keep a sequence.")
                       .arg(order->count()));
  } else {
    summary->setText("No saved sequence. Preview the queue to choose objects, order and places.");
  }
  show_selected_place();
}

void TaskIntentEditor::apply_sequence_review(const QJsonObject & review)
{
  auto * order = findChild<QListWidget *>("taskSequenceOrder");
  auto * summary = findChild<QLabel *>("taskSequenceSummary");
  if (!order || review.isEmpty()) return;
  QMap<QString, QJsonObject> items;
  for (const auto & value : review.value("items").toArray()) items[value.toObject().value("object_id").toString()] = value.toObject();
  sequence_loading_ = true;
  for (int i = 0; i < order->count(); ++i) {
    auto * item = order->item(i);
    const auto reviewed = items.value(sequence_object_id(item));
    item->setData(kReviewRole, item->checkState() == Qt::Checked ? reviewed : QJsonObject());
    if (item->data(kPlaceOptionsRole).toJsonArray().isEmpty() && !reviewed.value("place_options").toArray().isEmpty())
      item->setData(kPlaceOptionsRole, reviewed.value("place_options").toArray());
  }
  sequence_loading_ = false;
  refresh_sequence_texts();
  if (!sequence_rebind_pending_)
    summary->setText((dirty_ ? QString("Unsaved changes — ") : QString()) + review.value("summary").toString());
  show_selected_place();
}

void TaskIntentEditor::refresh_sequence_texts()
{
  auto * order = findChild<QListWidget *>("taskSequenceOrder");
  const bool was = sequence_loading_;
  sequence_loading_ = true;
  int position = 0;
  for (int i = 0; i < order->count(); ++i)
    show_sequence_item(order->item(i), order->item(i)->checkState() == Qt::Checked ? ++position : 0);
  sequence_loading_ = was;
}

void TaskIntentEditor::show_selected_place()
{
  auto * order = findChild<QListWidget *>("taskSequenceOrder");
  auto * place = findChild<QComboBox *>("taskSequencePlace");
  auto * detail = findChild<QLabel *>("taskSequencePlaceDetail");
  auto * item = order->currentItem();
  const QSignalBlocker block(place);
  place->clear();
  place->addItem("Automatic (next free place)", QString());
  if (!item) { place->setEnabled(false); detail->clear(); return; }
  for (const auto & value : item->data(kPlaceOptionsRole).toJsonArray())
    place->addItem(value.toObject().value("label").toString(), value.toObject().value("id").toString());
  const QString chosen = chosen_place_id(item);
  if (chosen == "custom") place->addItem(place_description(item), "custom");
  place->setCurrentIndex(std::max(0, place->findData(chosen)));
  const bool included = item->checkState() == Qt::Checked;
  place->setEnabled(included && place->count() > 1);
  const auto review = item->data(kReviewRole).toJsonObject();
  QStringList lines;
  const auto manual = item->data(kManualPoseRole).toJsonObject();
  const auto expected = review.value("expected_local_pose").toObject();
  auto coordinates = [](const QJsonObject & pose) {
    const auto xyz = pose.value("xyz_m").toArray();
    return QString("x %1, y %2, z %3 m").arg(xyz[0].toDouble(), 0, 'f', 3).arg(xyz[1].toDouble(), 0, 'f', 3)
                                        .arg(xyz[2].toDouble(), 0, 'f', 3);
  };
  if (!included) lines << "Not in the sequence: tick it to pick this object.";
  else if (!manual.isEmpty()) lines << QString("Chosen place, in the %1 frame: %2.").arg(
      item->data(kDestinationRole).toJsonObject().value("asset_ref").toString(), coordinates(manual));
  else if (!expected.isEmpty()) lines << QString("Automatic: allocated when the sequence is checked (expected %1, "
                                                 "in the %2 frame).").arg(coordinates(expected),
                                                 QString::fromStdString(model_.place.asset_ref));
  else lines << "Automatic: allocated when the sequence is checked, after chosen places are reserved.";
  if (included && item->data(kPlaceOptionsRole).toJsonArray().isEmpty())
    lines << "No fixed place fits this object in the destination with the current clearance.";
  if (included && !review.value("message").toString().isEmpty()) lines << review.value("message").toString();
  detail->setText(lines.join("\n"));
}

void TaskIntentEditor::write_sequence()
{
  if (sequence_loading_ || model_.authored_yaml.empty()) return;
  auto * order = findChild<QListWidget *>("taskSequenceOrder");
  QJsonArray items;
  for (int i = 0; i < order->count(); ++i) {
    auto * row = order->item(i);
    const auto binding = row->data(kBindingRole).toJsonObject();
    if (row->checkState() != Qt::Checked || binding.isEmpty()) continue;
    const auto pose = row->data(kManualPoseRole).toJsonObject();
    const auto destination = row->data(kDestinationRole).toJsonObject();
    const QJsonObject placement = pose.isEmpty() ? QJsonObject{{"mode", "AUTO"}} : QJsonObject{
        {"mode", "MANUAL"}, {"asset_ref", destination.value("asset_ref")},
        {"region_ref", destination.value("region_ref")}, {"local_pose", pose}};
    items.append(QJsonObject{{"object_id", sequence_object_id(row)}, {"class_id", binding.value("class_id")},
                             {"observed_xyz_m", binding.value("observed_xyz_m")},
                             {"dimensions_m", binding.value("dimensions_m")}, {"placement", placement}});
  }
  sequence_rebind_pending_ = false;
  // No ticked object: no saved sequence (the check falls back to the whole queue, all automatic).
  set_field("sequence", items.isEmpty() ? QJsonValue() : QJsonValue(QJsonObject{
      {"source_sha256", sequence_source_sha_}, {"items", items}}));
  refresh_sequence_texts();
  findChild<QLabel *>("taskSequenceSummary")->setText("Unsaved changes — checking the sequence…");
  show_selected_place();
}

void TaskIntentEditor::update_dirty_ui()
{
  if (auto * revert = findChild<QPushButton *>("taskRevert")) revert->setEnabled(dirty_ && !plan_check_);
}

void TaskIntentEditor::revert()
{
  if (!dirty_ || scene_.isEmpty()) return;
  dirty_ = false;
  model_ = {};
  load_scene(scene_, helper_);
}

void TaskIntentEditor::clear_sequence_results(const QString & why)
{
  if (plan_check_) return;
  auto * details = findChild<QPlainTextEdit *>("taskSimulationBatchDetails");
  if (details && details->toPlainText().contains("PLANNED — NOT EXECUTED")) details->setPlainText(why);
}

void TaskIntentEditor::start_plan_check(bool sequence)
{
  auto * status = findChild<QLabel *>("taskSimulationBatchStatus");
  auto * details = findChild<QPlainTextEdit *>("taskSimulationBatchDetails");
  const QString mode = findChild<QComboBox *>("taskPerceptionMode")->currentData().toString();
  if (plan_check_) return;
  if (scene_.isEmpty() || dirty_ || (mode != "off" && mode != "replayed_snapshot")) {
    status->setText("BLOCKED — Save an explicit commissioning or replay task, then Generate, before checking plans.");
    return;
  }
  const QString script = QFileInfo(helper_).dir().absoluteFilePath("simulation_batch_plan_check.py");
  if (!QFileInfo::exists(script)) {
    status->setText("BLOCKED — simulation_batch_plan_check.py is not installed with the Task helper.");
    return;
  }
  QFile task(scene_ + "/config/workcell_builder_task_intent.yaml");
  plan_check_task_bytes_ = task.open(QIODevice::ReadOnly) ? QString::fromUtf8(task.readAll()) : QString();
  plan_check_scene_ = scene_;
  plan_check_buffer_.clear(); plan_check_lines_.clear(); plan_check_report_ = QJsonObject(); plan_check_total_ = 0;
  plan_check_ = new QProcess(this);
  plan_check_->setProcessChannelMode(QProcess::SeparateChannels);
  plan_check_sequence_ = sequence;
  findChild<QPushButton *>("taskSimulationBatchPlanCheck")->setEnabled(false);
  findChild<QPushButton *>("taskSimulationSequenceCheck")->setEnabled(false);
  findChild<QPushButton *>("taskSimulationBatchPreview")->setEnabled(false);
  findChild<QPushButton *>("taskSimulationBatchPlanCancel")->setEnabled(true);
  status->setText("Checking plans — building the queue from the saved task. No robot motion.");
  details->setPlainText("Plan-only check started. Each object gets a fresh private MoveIt scene.");
  connect(plan_check_, &QProcess::readyReadStandardOutput, this, &TaskIntentEditor::on_plan_check_output);
  connect(plan_check_, &QProcess::errorOccurred, this, [this](QProcess::ProcessError error) {
    if (error == QProcess::FailedToStart) finish_plan_check(-1, false);
  });
  connect(plan_check_, qOverload<int, QProcess::ExitStatus>(&QProcess::finished), this,
          [this](int code, QProcess::ExitStatus exit) { finish_plan_check(code, exit == QProcess::NormalExit); });
  QStringList args{script, "--scene", plan_check_scene_, "--limit", "10"};
  if (sequence) {
    // The checker reads the saved, generated task: the same sequence the CLI and a reopened Builder use.
    args << "--sequence";
    int automatic = 0, included = 0;
    auto * order = findChild<QListWidget *>("taskSequenceOrder");
    for (int i = 0; i < order->count(); ++i) {
      if (order->item(i)->checkState() != Qt::Checked) continue;
      ++included;
      if (order->item(i)->data(kManualPoseRole).toJsonObject().isEmpty()) ++automatic;
    }
    QString text = "Checking full sequence — one private fake-hardware MoveIt scene, objects planned in order. No motion.";
    auto * place_policy = findChild<QComboBox *>("taskPlacePolicy");
    if (place_policy && place_policy->currentText() == "EXACT" && automatic > 1 && included > 1) {
      text += "\nPlacement policy is EXACT: every object is planned to the same place. "
              "Choose AUTO to give each object its own free place in the destination.";
    }
    status->setText(text);
  }
  plan_check_->start("python3", args);
}

void TaskIntentEditor::on_plan_check_output()
{
  if (!plan_check_) return;
  plan_check_buffer_ += plan_check_->readAllStandardOutput();
  auto * status = findChild<QLabel *>("taskSimulationBatchStatus");
  auto * details = findChild<QPlainTextEdit *>("taskSimulationBatchDetails");
  int newline;
  while ((newline = plan_check_buffer_.indexOf('\n')) >= 0) {
    const QJsonObject e = QJsonDocument::fromJson(plan_check_buffer_.left(newline)).object();
    plan_check_buffer_.remove(0, newline + 1);
    const QString kind = e.value("event").toString();
    if (kind == "batch_start") {
      plan_check_total_ = e.value("queued").toInt();
    } else if (kind == "object_start") {
      status->setText(QString("Checking %1 of %2: %3 (%4) — starting private fake-hardware MoveIt scene. No motion.")
        .arg(e.value("order").toInt()).arg(plan_check_total_)
        .arg(e.value("object_id").toString(), e.value("class_id").toString()));
    } else if (kind == "stage" && e.value("stage").toString().startsWith("SEQUENCE_OBJECT_")) {
      status->setText(QString("Sequence: planning object %1 of %2 against the current virtual scene. No motion.")
        .arg(e.value("stage").toString().section('_', 2)).arg(plan_check_total_));
    } else if (kind == "stage") {
      status->setText(status->text().section(" — ", 0, 0) + " — " + e.value("stage").toString() + ". No motion.");
    } else if (kind == "object_result") {
      plan_check_lines_ << QString("%1. %2: %3%4 — %5").arg(e.value("order").toInt())
        .arg(e.value("object_id").toString(), e.value("outcome").toString(),
             e.value("first_failed_stage").isString() ? " at " + e.value("first_failed_stage").toString() : QString(),
             e.value("reason").toString());
      details->setPlainText(plan_check_lines_.join("\n"));
    } else if (kind == "batch_result") {
      plan_check_report_ = e.value("report").toObject();
    }
  }
}

void TaskIntentEditor::finish_plan_check(int exit_code, bool normal_exit)
{
  if (!plan_check_) return;
  on_plan_check_output();
  plan_check_->deleteLater();
  plan_check_ = nullptr;
  auto * status = findChild<QLabel *>("taskSimulationBatchStatus");
  auto * details = findChild<QPlainTextEdit *>("taskSimulationBatchDetails");
  findChild<QPushButton *>("taskSimulationBatchPlanCancel")->setEnabled(false);
  refresh_fields();
  QFile task(scene_ + "/config/workcell_builder_task_intent.yaml");
  const QString current = task.open(QIODevice::ReadOnly) ? QString::fromUtf8(task.readAll()) : QString();
  if (plan_check_scene_ != scene_ || dirty_ || current != plan_check_task_bytes_) {
    status->setText("BLOCKED — Task changed during the plan check. Save, Generate and check again.");
    details->setPlainText("Old plan results cannot be used after changing the task. No motion was performed.");
    return;
  }
  const QJsonObject r = plan_check_report_;
  const QString state = r.value("status").toString(normal_exit ? "BLOCKED" : "FAILED");
  if (state != "CHECKED" && state != "CANCELLED") {
    status->setText("BLOCKED — " + r.value("reason").toString(
      exit_code < 0 ? "plan check helper failed to start" : "plan check did not produce a result"));
    details->setPlainText("No complete plan check. No robot motion was performed." +
                          (r.contains("output_dir") ? "\nEvidence: " + r.value("output_dir").toString() : QString()));
    return;
  }
  if (plan_check_sequence_) {
    const QJsonObject block = r.value("first_blocking").toObject();
    const QString seq = r.value("sequence_status").toString();
    status->setText(seq == "SEQUENCE_PLANNED"
      ? QString("PLANNED — NOT EXECUTED: the full %1-object sequence plans in MoveIt. No robot motion.")
          .arg(r.value("objects").toArray().size())
      : QString("%1 — sequence blocked at position %2 (%3): %4 at %5. PLANNED — NOT EXECUTED.")
          .arg(state == "CANCELLED" ? "Cancelled" : "SEQUENCE_BLOCKED")
          .arg(block.value("position").toInt()).arg(block.value("object_id").toString(),
               block.value("outcome").toString(), block.value("first_failed_stage").toString("—")));
    QStringList lines{"PLANNED — NOT EXECUTED. Virtual scene only; no robot or simulated motion."};
    for (const auto & value : r.value("objects").toArray()) {
      const auto o = value.toObject();
      QString line = QString("%1. %2 [%3] → %4: %5").arg(o.value("position").toInt())
        .arg(o.value("object_id").toString(), o.value("class_id").toString(),
             o.value("destination_region_ref").toString(), o.value("outcome").toString());
      if (o.contains("revision_before")) {
        line += QString("  (scene rev %1%2, robot start %3)").arg(o.value("revision_before").toInt())
          .arg(o.value("revision_after").isDouble() ? QString("→%1").arg(o.value("revision_after").toInt()) : QString())
          .arg(o.value("robot_start_matches_previous_end").toBool() ? "consistent" : "INCONSISTENT");
      }
      if (o.value("outcome").toString() == "PLANNABLE") {
        const auto local = o.value("place_local_pose").toObject().value("xyz_m").toArray();
        const auto allocation = o.value("placement_allocation").toObject();
        const QString how = allocation.value("operator_selected").toBool() ? " (your chosen place)"
                          : allocation.value("applied").toBool() ? " (free place allocated)" : "";
        line += local.size() == 3
          ? QString("\n   grasp %1; virtually placed in %2 at target-local (%3, %4, %5) m%6")
              .arg(o.value("grasp_strategy").toString("resolved"), o.value("destination_region_ref").toString())
              .arg(local[0].toDouble(), 0, 'f', 3).arg(local[1].toDouble(), 0, 'f', 3).arg(local[2].toDouble(), 0, 'f', 3)
              .arg(how)
          : QString("\n   grasp %1; virtually placed at destination").arg(o.value("grasp_strategy").toString("resolved"));
      } else {
        line += QString("\n   first failed stage: %1\n   reason: %2").arg(o.value("first_failed_stage").toString("—"),
                                                                       o.value("reason").toString());
        const auto hits = o.value("collision_objects").toVariant().toStringList();
        if (!hits.isEmpty()) line += "\n   collides with: " + hits.join(", ");
      }
      lines << line;
    }
    for (const auto & value : r.value("virtual_scene").toArray()) {
      const auto m = value.toObject();
      const auto from = m.value("from_pose").toArray(), to = m.value("to_pose").toArray();
      lines << QString("Virtual scene rev %1: %2 moved (%3, %4, %5) → (%6, %7, %8)")
        .arg(m.value("revision").toInt()).arg(m.value("object_id").toString())
        .arg(from[0].toDouble(), 0, 'f', 3).arg(from[1].toDouble(), 0, 'f', 3).arg(from[2].toDouble(), 0, 'f', 3)
        .arg(to[0].toDouble(), 0, 'f', 3).arg(to[1].toDouble(), 0, 'f', 3).arg(to[2].toDouble(), 0, 'f', 3);
    }
    for (const auto & value : r.value("obstacles").toArray()) {
      const auto o = value.toObject();
      lines << QString("Obstacle throughout (not picked): %1 [%2] — %3").arg(o.value("object_id").toString(),
        o.value("class_id").toString(), o.value("reason").toString());
    }
    lines << "Evidence: " + r.value("output_dir").toString();
    details->setPlainText(lines.join("\n"));
    return;
  }
  const QJsonObject counts = r.value("outcome_counts").toObject();
  QStringList summary;
  for (auto it = counts.begin(); it != counts.end(); ++it) {
    if (it.value().toInt() > 0) summary << QString("%1 %2").arg(it.value().toInt()).arg(it.key());
  }
  status->setText(QString("%1 plan check (no motion): %2 objects — %3. Sequence not validated.")
    .arg(state == "CANCELLED" ? "Cancelled" : "Finished")
    .arg(r.value("objects").toArray().size())
    .arg(summary.join(", ")));
  QStringList lines;
  for (const auto & value : r.value("objects").toArray()) {
    const auto o = value.toObject();
    QString line = QString("%1. %2 [%3] → %4: %5").arg(o.value("order").toInt())
      .arg(o.value("object_id").toString(), o.value("class_id").toString(),
           o.value("destination_region_ref").toString(), o.value("outcome").toString());
    if (o.value("outcome").toString() != "PLANNABLE") {
      line += QString("\n   first failed stage: %1\n   reason: %2")
        .arg(o.value("first_failed_stage").toString("—"), o.value("reason").toString());
      const auto hits = o.value("collision_objects").toVariant().toStringList();
      if (!hits.isEmpty()) line += "\n   collides with: " + hits.join(", ");
    } else {
      line += QString(" (grasp strategy %1, all nine stages planned)").arg(o.value("selected_grasp_strategy").toString("resolved"));
    }
    lines << line;
  }
  for (const auto & value : r.value("obstacles").toArray()) {
    const auto o = value.toObject();
    lines << QString("Obstacle (not picked): %1 [%2] — %3").arg(o.value("object_id").toString(),
      o.value("class_id").toString(), o.value("reason").toString());
  }
  lines << "Each object was planned from the same recorded scene; picking one first is not simulated, "
           "so the batch order is NOT validated.";
  lines << "GRASP plans are not reachability-free execution approvals. No robot movement was performed.";
  lines << "Evidence: " + r.value("output_dir").toString();
  details->setPlainText(lines.join("\n"));
}

void TaskIntentEditor::add_routing_row(const QJsonObject & rule)
{
  const int row = routing_table_->rowCount();
  routing_table_->insertRow(row);
  auto * cls = new QComboBox(routing_table_); cls->setEditable(true); cls->addItems(vocabulary_);
  // Size to a short class name, not the longest vocabulary label, so the destination column stays readable.
  cls->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon); cls->setMinimumContentsLength(5);
  cls->setCurrentText(rule.value("when").toObject().value("equals").toString());
  cls->setProperty("rule_id", rule.value("id").toString());
  cls->setObjectName(QString("taskRoutingClass%1").arg(row));
  auto * destination = new QComboBox(routing_table_); destination->addItems(region_ids_);
  const QString region = rule.value("destination").toString();
  if (!region.isEmpty() && destination->findText(region) < 0) destination->addItem(region);   // shown, then validated
  destination->setCurrentText(region);
  destination->setObjectName(QString("taskRoutingDestination%1").arg(row));
  const auto regions = report_.value("regions").toObject();
  for (int i = 0; i < destination->count(); ++i)
    destination->setItemData(i, destination->itemText(i) + " in " + regions.value(destination->itemText(i)).toString("?"), Qt::ToolTipRole);
  destination->setToolTip(destination->currentText() + " in " + regions.value(destination->currentText()).toString("?"));
  connect(destination, &QComboBox::currentTextChanged, destination, [destination, regions](const QString & text) {
    destination->setToolTip(text + " in " + regions.value(text).toString("?"));
  });
  auto * enabled = new QCheckBox(routing_table_); enabled->setChecked(rule.value("enabled").toBool(true));
  enabled->setObjectName(QString("taskRoutingEnabled%1").arg(row));
  auto * priority = new QSpinBox(routing_table_); priority->setRange(-1000, 1000);
  priority->setValue(rule.value("priority").toInt(0));
  priority->setMaximumWidth(72);
  priority->setObjectName(QString("taskRoutingPriority%1").arg(row));
  routing_table_->setCellWidget(row, 0, cls);
  routing_table_->setCellWidget(row, 1, destination);
  routing_table_->setCellWidget(row, 2, enabled);
  routing_table_->setCellWidget(row, 3, priority);
  connect(cls, &QComboBox::currentTextChanged, this, [this] { write_routing(); });
  connect(destination, &QComboBox::currentTextChanged, this, [this] { write_routing(); });
  connect(enabled, &QCheckBox::toggled, this, [this] { write_routing(); });
  connect(priority, qOverload<int>(&QSpinBox::valueChanged), this, [this] { write_routing(); });
}

void TaskIntentEditor::fill_routing(const QJsonObject & routing)
{
  routing_loading_ = true;
  routing_table_->setRowCount(0);
  const bool routed = routing.value("mode").toString() == "class";
  if (routed)
    for (const auto & item : routing.value("rules").toArray()) add_routing_row(item.toObject());
  const int policy = unrouted_policy_->findData(routing.value("unrouted_policy").toString("leave_in_place"));
  unrouted_policy_->setCurrentIndex(policy >= 0 ? policy : 0);
  max_attempts_->setValue(routing.value("max_attempts_per_object").toInt(2));
  unrouted_policy_->setEnabled(routed); max_attempts_->setEnabled(routed);
  routing_loading_ = false;
}

void TaskIntentEditor::write_routing()
{
  if (routing_loading_ || loading_) return;
  // Rule ids stay stable across edits; a new row gets the next free route_<n>.
  QSet<QString> used;
  for (int row = 0; row < routing_table_->rowCount(); ++row)
    used << static_cast<QComboBox *>(routing_table_->cellWidget(row, 0))->property("rule_id").toString();
  QJsonArray rules;
  for (int row = 0, next = 1; row < routing_table_->rowCount(); ++row) {
    auto * cls = static_cast<QComboBox *>(routing_table_->cellWidget(row, 0));
    QString id = cls->property("rule_id").toString();
    if (id.isEmpty()) {
      while (used.contains(QString("route_%1").arg(next))) ++next;
      id = QString("route_%1").arg(next); used << id; cls->setProperty("rule_id", id);
    }
    QJsonObject rule{{"id", id}, {"when", QJsonObject{{"attribute", "class"}, {"equals", cls->currentText().trimmed()}}},
                     {"destination", static_cast<QComboBox *>(routing_table_->cellWidget(row, 1))->currentText()},
                     {"enabled", static_cast<QCheckBox *>(routing_table_->cellWidget(row, 2))->isChecked()}};
    if (const int priority = static_cast<QSpinBox *>(routing_table_->cellWidget(row, 3))->value()) rule["priority"] = priority;
    rules.append(rule);
  }
  const bool routed = !rules.isEmpty();
  unrouted_policy_->setEnabled(routed); max_attempts_->setEnabled(routed);
  // No rows: direct pick/place into the authored placement region.
  set_field("routing", routed ? QJsonObject{{"mode", "class"}, {"unrouted_policy", unrouted_policy_->currentData().toString()},
                                            {"max_attempts_per_object", max_attempts_->value()}, {"rules", rules}}
                              : QJsonObject{{"mode", "direct"}, {"rules", QJsonArray()}});
}
