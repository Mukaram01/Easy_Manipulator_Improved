#include "task_intent_editor.h"
#include <QCheckBox>
#include <QComboBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QPlainTextEdit>
#include <QLineEdit>
#include <QMessageBox>
#include <QHBoxLayout>
#include <QProcess>
#include <QPushButton>
#include <QSaveFile>
#include <QSignalBlocker>
#include <QTimer>
#include <QVBoxLayout>
#include <cmath>

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
}

TaskIntentEditor::TaskIntentEditor(QWidget * parent) : QGroupBox("Task Authoring — Pick & Place", parent)
{
  setObjectName("environmentTaskEditor");
  auto * layout = new QVBoxLayout(this);
  auto section = [&](const QString & label) {
    auto * group = new QGroupBox(label, this); layout->addWidget(group);
    return new QFormLayout(group);
  };
  auto * what = section("What / Pick eligibility");
  text_field(what, "taskTargetClass", "Object class", "pick/selection/object_filter/class_id");

  pick_zones_ = choice_field(what, "taskPickZone", "Search region", "pick/selection/zone_ref", {});
  auto * perception = section("Object input");
  auto * mode = new QComboBox(this); mode->setObjectName("taskPerceptionMode");
  mode->addItem("Off — commissioning objects", "off");
  mode->addItem("Recorded / replayed observations", "replayed_snapshot");
  mode->addItem("Live — normalized adapter snapshot", "live_epd");
  perception->addRow("Perception mode", mode);
  readers_["perception/mode"] = [mode](const QJsonValue & value) {
    mode->setCurrentIndex(value.isString() ? mode->findData(value.toString()) : -1);
  };
  connect(mode, qOverload<int>(&QComboBox::currentIndexChanged), this, [this, mode](int index) {
    if (loading_ || index < 0) return;
    const auto value = mode->currentData().toString();
    set_field("perception/mode", value);
    set_field("pick/selection/source_type", value == "off" ? "manual_simulated" : "perception");
    set_field("pick/selection/source_ref", value == "off" ? QString::fromStdString(model_.pick_selection.zone_ref) : "detected_objects/v1");
    refresh_fields();
  });
  text_field(perception, "taskReplayPath", "Observation snapshot", "perception/replay_path");
  auto * choose = new QPushButton("Choose observation file…", this); choose->setObjectName("taskChooseObservations");
  perception->addRow(choose);
  connect(choose, &QPushButton::clicked, this, [this] {
    const auto path = QFileDialog::getOpenFileName(this, "Choose existing detected_objects/v1 observations", scene_, "Observations (*.yaml *.yml *.json)");
    if (!path.isEmpty()) { set_field("perception/replay_path", path); refresh_fields(); }
  });
  auto * input_help = new QLabel("No workpiece geometry is invented. Off/replay uses explicit commissioning observations. Live requires a current normalized adapter snapshot; stale or unavailable objects block planning.", this);
  input_help->setWordWrap(true); perception->addRow(input_help);
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
  connect(check_button, &QPushButton::clicked, this, &TaskIntentEditor::start_plan_check);
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
  auto * place = section("How to place");
  choice_field(place, "taskPlacePolicy", "Placement policy", "place/placement/policy", {"AUTO", "PREFERRED", "EXACT"});
  vector_field(place, "taskLocalXYZ", "Requested local XYZ (m)", "place/placement/requested_local_pose/xyz_m");
  vector_field(place, "taskLocalRPY", "Requested local RPY (rad)", "place/placement/requested_local_pose/rpy_rad");
  number_field(place, "taskPlaceClearance", "Placement clearance", "place/placement/clearance_m", " m");
  number_field(place, "taskPlaceApproach", "Place approach", "place/placement/approach/distance_m", " m");
  number_field(place, "taskPlaceRetreat", "Place retreat", "place/placement/retreat/distance_m", " m");
  auto * help = new QLabel("AUTO chooses a valid strategy / the region's authored placement. PREFERRED tries your request first and reports any fallback. EXACT keeps your request and blocks planning if it cannot be satisfied. Local poses use the physical target's frame; values are never clamped.", this);
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
  form->addRow(new QLabel("Release: installed tool release. Fake hardware required; real robot locked.", this));
  status_ = new QLabel(this); status_->setObjectName("taskValidationStatus"); status_->setWordWrap(true); layout->addWidget(status_);
  auto * buttons = new QHBoxLayout;
  auto * save_button = new QPushButton("Save task", this); save_button->setObjectName("taskSave");
  auto * validate_button = new QPushButton("Validate task", this); validate_button->setObjectName("taskValidate");
  buttons->addWidget(save_button); buttons->addWidget(validate_button); layout->addLayout(buttons);
  connect(save_button, &QPushButton::clicked, this, [this] { save(); });
  connect(validate_button, &QPushButton::clicked, this, &TaskIntentEditor::validate_now);
  validation_timer_ = new QTimer(this); validation_timer_->setSingleShot(true);
  connect(validation_timer_, &QTimer::timeout, this, &TaskIntentEditor::validate_now);
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
  std::vector<std::string> keys; for (const auto & key : path.split('/')) keys.push_back(key.toStdString());
  try { model_.set_field(keys, json_value(value).toStdString()); }
  catch (const std::exception & exc) { error_ = QString::fromUtf8(exc.what()); status_->setText("BLOCKED — " + error_); return; }
  if (path == "pick/selection/zone_ref" && findChild<QComboBox *>("taskPerceptionMode")->currentData().toString() == "off") {
    model_.set_field({"pick", "selection", "source_ref"}, json_value(value).toStdString());
  }
  dirty_ = true; error_.clear();
  findChild<QPushButton *>("taskSimulationBatchPreview")->setEnabled(false);
  findChild<QPushButton *>("taskSimulationBatchPlanCheck")->setEnabled(false);
  status_->setText("Unsaved task — validate and save before planning.");
  validation_timer_->start(250); emit edited();
}
void TaskIntentEditor::refresh_fields()
{
  loading_ = true;
  const auto root = QJsonDocument::fromJson(QByteArray::fromStdString(workcell_builder::canonical_task_intent_json_for_testing(model_.to_yaml()))).object();
  for (auto it = readers_.begin(); it != readers_.end(); ++it) it.value()(value_at(root, it.key()));
  const QString mode = findChild<QComboBox *>("taskPerceptionMode")->currentData().toString();
  const bool batch_ready = !dirty_ && (mode == "off" || mode == "replayed_snapshot") &&
      !findChild<QLineEdit *>("taskReplayPath")->text().trimmed().isEmpty();
  findChild<QPushButton *>("taskSimulationBatchPreview")->setEnabled(batch_ready);
  findChild<QPushButton *>("taskSimulationBatchPlanCheck")->setEnabled(batch_ready && !plan_check_);
  loading_ = false;
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
  if (scene != scene_) stop_plan_check(90000);
  scene_ = scene; helper_ = helper; dirty_ = false; error_.clear(); input_blocker_.clear(); input_errors_.clear();
  findChild<QLabel *>("taskSimulationBatchStatus")->setText(
      "Select an explicit commissioning recording and Save Task to preview the queue.");
  findChild<QPlainTextEdit *>("taskSimulationBatchDetails")->setPlainText("No batch preview has been run.");
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
    strategies_->setEditable(false);
    refresh_fields(); setEnabled(true); apply_report(report); return true;
  } catch (const std::exception & exc) {
    error_ = QString::fromUtf8(exc.what()); status_->setText("BLOCKED — " + error_); return false;
  }
}
void TaskIntentEditor::apply_report(const QJsonObject & report)
{
  report_ = report;
  QStringList messages; for (const auto & value : report["errors"].toArray()) messages << value.toString();
  error_ = messages.join("\n");
  input_blocker_ = report["planning_input_blocker"].toString();
  if (!input_blocker_.isEmpty()) messages << input_blocker_;
  for (const auto & value : report["warnings"].toArray()) messages << value.toString();
  const auto destination = report["resolved_destination"].toObject();
  if (!destination.isEmpty()) {
    messages << "Resolved destination in world (m): " + json_value(destination["pose_xyz"]);
  }
  const auto status = (error_.isEmpty() && input_blocker_.isEmpty()) ? (report["warnings"].toArray().isEmpty() ? "Authoring valid" : "WARNING") : "BLOCKED";
  status_->setText(QString("%1%2\n%3\nIntent hash: %4\nPlanning still requires complete-cycle validation.")
    .arg(status, dirty_ ? " — unsaved" : "", messages.join("\n"), report["normalized_intent_sha256"].toString("unavailable")));
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
    apply_report(request(true));
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
    auto report = request(true);
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

void TaskIntentEditor::start_plan_check()
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
  findChild<QPushButton *>("taskSimulationBatchPlanCheck")->setEnabled(false);
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
  plan_check_->start("python3", {script, "--scene", plan_check_scene_, "--limit", "10"});
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
    details->setPlainText("No complete plan check. No robot motion was performed.");
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
