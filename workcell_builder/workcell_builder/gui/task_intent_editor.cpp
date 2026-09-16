#include "task_intent_editor.h"
#include <QCheckBox>
#include <QComboBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFormLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QLabel>
#include <QLineEdit>
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
  text_field(what, "taskSource", "Source reference", "pick/selection/source_ref");
  pick_zones_ = choice_field(what, "taskPickZone", "Search region", "pick/selection/zone_ref", {});
  auto * grasp = section("How to grasp");
  choice_field(grasp, "taskGraspPolicy", "Grasp policy", "pick/grasp/policy", {"AUTO", "PREFERRED", "EXACT"});
  strategies_ = choice_field(grasp, "taskGraspIntent", "Grasp strategy", "pick/grasp/strategy_ref", {});
  auto * where = section("Where");
  assets_ = choice_field(where, "taskPlaceAsset", "Physical target", "place/target/asset_ref", {});
  regions_ = choice_field(where, "taskPlaceRegion", "Placement region", "place/target/region_ref", {});
  auto * place = section("How to place");
  choice_field(place, "taskPlacePolicy", "Placement policy", "place/placement/policy", {"AUTO", "PREFERRED", "EXACT"});
  vector_field(place, "taskLocalXYZ", "Requested local XYZ (m)", "place/placement/requested_local_pose/xyz_m");
  vector_field(place, "taskLocalRPY", "Requested local RPY (rad)", "place/placement/requested_local_pose/rpy_rad");
  auto * help = new QLabel("AUTO chooses a valid strategy / the region's authored placement. PREFERRED tries your request first and reports any fallback. EXACT keeps your request and blocks planning if it cannot be satisfied. Local poses use the physical target's frame; values are never clamped.", this);
  help->setWordWrap(true); layout->addWidget(help);
  auto * advanced = new QGroupBox("Advanced task settings", this);
  advanced->setCheckable(true); advanced->setChecked(false); layout->addWidget(advanced);
  auto * advanced_layout = new QVBoxLayout(advanced);
  auto * fields = new QWidget(advanced); auto * form = new QFormLayout(fields);
  advanced_layout->addWidget(fields); fields->hide();
  connect(advanced, &QGroupBox::toggled, fields, &QWidget::setVisible);
  text_field(form, "taskSourceType", "Source type", "pick/selection/source_type");
  number_field(form, "taskMinimumConfidence", "Minimum confidence (−1 = unset)", "pick/selection/object_filter/min_confidence", "", true);
  number_field(form, "taskMaximumAge", "Maximum observation age", "pick/selection/object_filter/max_age_seconds", " s", true);
  text_field(form, "taskGraspAxis", "Grasp approach axis", "pick/grasp/approach/axis");
  number_field(form, "taskApproachDistance", "Grasp approach", "pick/grasp/approach/distance_m", " m");
  text_field(form, "taskLiftAxis", "Lift axis", "pick/grasp/lift/axis");
  number_field(form, "taskRetreatDistance", "Lift distance", "pick/grasp/lift/distance_m", " m");
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
  number_field(form, "taskPlaceApproach", "Place approach", "place/placement/approach/distance_m", " m");
  number_field(form, "taskPlaceClearance", "Place clearance", "place/placement/clearance_m", " m");
  text_field(form, "taskPlaceRetreatAxis", "Place retreat axis", "place/placement/retreat/axis");
  number_field(form, "taskPlaceRetreat", "Place retreat", "place/placement/retreat/distance_m", " m");
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
  dirty_ = true; error_.clear(); status_->setText("Unsaved task — validate and save before planning.");
  validation_timer_->start(250); emit edited();
}
void TaskIntentEditor::refresh_fields()
{
  loading_ = true;
  const auto root = QJsonDocument::fromJson(QByteArray::fromStdString(workcell_builder::canonical_task_intent_json_for_testing(model_.to_yaml()))).object();
  for (auto it = readers_.begin(); it != readers_.end(); ++it) it.value()(value_at(root, it.key()));
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
bool TaskIntentEditor::load_scene(const QString & scene, const QString & helper)
{
  if (scene == scene_ && !model_.authored_yaml.empty()) return true;
  scene_ = scene; helper_ = helper; dirty_ = false; error_.clear(); input_errors_.clear();
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
  for (const auto & value : report["warnings"].toArray()) messages << value.toString();
  const auto destination = report["resolved_destination"].toObject();
  if (!destination.isEmpty()) {
    messages << "Resolved destination in world (m): " + json_value(destination["pose_xyz"]);
  }
  const auto status = error_.isEmpty() ? (report["warnings"].toArray().isEmpty() ? "Authoring valid" : "WARNING") : "BLOCKED";
  status_->setText(QString("%1%2\n%3\nIntent hash: %4\nPlanning still requires complete-cycle validation.")
    .arg(status, dirty_ ? " — unsaved" : "", messages.join("\n"), report["normalized_intent_sha256"].toString("unavailable")));
}
void TaskIntentEditor::validate_now()
{
  validation_timer_->stop();
  if (model_.authored_yaml.empty() || !input_errors_.isEmpty()) return;
  try { apply_report(request(true)); }
  catch (const std::exception & exc) { error_ = QString::fromUtf8(exc.what()); status_->setText("BLOCKED — " + error_); }
}
QString TaskIntentEditor::blocker() const
{
  if (dirty_) return "Save task edits before planning.";
  if (!input_errors_.isEmpty()) return input_errors_.first();
  return error_;
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
  set_field("pick/selection/source_ref", id); set_field("pick/selection/zone_ref", id); refresh_fields();
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
