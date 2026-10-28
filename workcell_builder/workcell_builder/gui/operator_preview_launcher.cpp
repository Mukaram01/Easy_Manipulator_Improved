#include "operator_preview_launcher.hpp"

#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QFileInfo>
#include <QProcess>
#include <QJsonDocument>
#include <QSaveFile>
#include <QUuid>

#include <cstdio>

#include "operator_simulation_link.hpp"

namespace workcell_builder
{

OperatorPreviewLauncher::OperatorPreviewLauncher(QObject * parent)
: QObject(parent),
  // Random per launcher: a command made from any other request (or a standalone HMI) never matches.
  control_token_(QUuid::createUuid().toString(QUuid::WithoutBraces) + QUuid::createUuid().toString(QUuid::WithoutBraces))
{}

OperatorPreviewLauncher::~OperatorPreviewLauncher() { shutdown(2000); }

QString OperatorPreviewLauncher::hmi_program() const
{
  if (!hmi_program_override_.isEmpty()) return hmi_program_override_;
  return QDir(QCoreApplication::applicationDirPath()).filePath("workcell_operator_hmi");
}

QString OperatorPreviewLauncher::request_path() const
{
  return directory_ ? QDir(directory_->path()).filePath("operator_preview_request.json") : QString();
}

bool OperatorPreviewLauncher::running() const
{
  return process_ && process_->state() != QProcess::NotRunning;
}

qint64 OperatorPreviewLauncher::process_id() const { return running() ? process_->processId() : 0; }

bool OperatorPreviewLauncher::write_request(OperatorPreviewRequest & request, bool activate, QString * error)
{
  if (!directory_) {
    directory_ = std::make_unique<QTemporaryDir>(QDir::tempPath() + "/workcell_operator_preview_XXXXXX");
    if (!directory_->isValid()) {
      if (error) *error = "Cannot create the preview hand-off directory: " + directory_->errorString();
      directory_.reset();
      return false;
    }
  }
  request.revision = ++revision_;
  request.activate = activate;
  request.builder_pid = QCoreApplication::applicationPid();
  request.requested_at = QDateTime::currentDateTimeUtc().toString(Qt::ISODate);
  request.control_token = control_token_;
  if (!request.task_dirty) request.task_draft = {};
  return write_operator_preview_request(request_path(), request, error);
}

bool OperatorPreviewLauncher::open_or_refresh(OperatorPreviewRequest request, QString * error)
{
  if (request.scene_dir.isEmpty() || !QFileInfo(request.scene_dir).isDir()) {
    if (error) *error = "Select a saved cell before opening the Operator Preview.";
    return false;
  }
  if (request.task_dirty && request.task_draft.isEmpty()) {
    if (error) *error = "The task editor reports unsaved edits but produced no draft.";
    return false;
  }
  if (!write_request(request, true, error)) return false;
  if (running()) {
    emit state_changed("refreshed", QString("Operator Preview refreshed (revision %1).").arg(request.revision));
    return true;
  }
  const QString program = hmi_program();
  if (!QFileInfo(program).isExecutable()) {
    if (error) *error = "Operator HMI executable not found: " + program;
    return false;
  }
  if (!process_) {
    process_ = new QProcess(this);
    // stdout is the HMI's command channel to this Builder; its stderr (logs) stays forwarded.
    process_->setProcessChannelMode(QProcess::ForwardedErrorChannel);
    connect(process_, &QProcess::readyReadStandardOutput, this, &OperatorPreviewLauncher::read_hmi_output);
    connect(process_, qOverload<int, QProcess::ExitStatus>(&QProcess::finished), this,
            [this](int code, QProcess::ExitStatus status) {
              emit state_changed("closed", status == QProcess::NormalExit
                ? QString("Operator Preview closed (exit %1).").arg(code)
                : QString("Operator Preview stopped unexpectedly."));
            });
  }
  QStringList arguments{"--request", request_path(), "--managed"};
  arguments << extra_arguments_;
  process_->start(program, arguments);
  if (!process_->waitForStarted(5000)) {
    if (error) *error = "Operator HMI did not start: " + process_->errorString();
    return false;
  }
  emit state_changed("opened", QString("Operator Preview opened for %1 (revision %2).")
    .arg(request.scene_name).arg(request.revision));
  return true;
}

void OperatorPreviewLauncher::read_hmi_output()
{
  output_buffer_ += process_->readAllStandardOutput();
  int end;
  while ((end = output_buffer_.indexOf('\n')) >= 0) {
    const QByteArray line = output_buffer_.left(end + 1);
    output_buffer_.remove(0, end + 1);
    QJsonObject command;
    if (parse_operator_command_line(line, &command)) {
      emit simulation_command(command);
    } else {
      std::fwrite(line.constData(), 1, static_cast<size_t>(line.size()), stdout);   // any other output, unchanged
      std::fflush(stdout);
    }
  }
}

void OperatorPreviewLauncher::publish_simulation_status(const QJsonObject & status)
{
  if (!directory_) return;
  const QByteArray bytes = QJsonDocument(status).toJson(QJsonDocument::Indented);
  if (bytes == last_status_) return;
  QSaveFile file(operator_status_file(request_path()));
  if (file.open(QIODevice::WriteOnly) && file.write(bytes) == bytes.size() && file.commit()) last_status_ = bytes;
}

void OperatorPreviewLauncher::publish_update(OperatorPreviewRequest request)
{
  if (!running()) return;
  QString error;
  write_request(request, false, &error);
}

void OperatorPreviewLauncher::shutdown(int timeout_ms)
{
  if (!running()) return;
  process_->disconnect(this);
  process_->terminate();  // SIGTERM: the HMI stops its checks and quits
  if (!process_->waitForFinished(timeout_ms)) {
    process_->kill();
    process_->waitForFinished(1000);
  }
}

}  // namespace workcell_builder
