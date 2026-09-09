#pragma once

#include <QObject>
#include <QProcess>
#include <QJsonDocument>
#include <QJsonObject>
#include <QDir>
#include <QFileInfo>
#include <QTimer>
#include <QUuid>
#include <functional>

// One process/socket authority. Request identities belong to ScenePreviewWidget;
// this session can serve successive scene requests from the same repository.
class OwnedProductViewServer : public QObject
{
public:
  explicit OwnedProductViewServer(QObject * parent = nullptr) : QObject(parent) {}
  ~OwnedProductViewServer() override { stop(); }
  std::function<void(int)> ready;
  std::function<void(const QString &)> failed;
  std::function<void(const QString &)> diagnostic;

  int port() const { return port_; }
  QString root() const { return root_; }
  qint64 pid() const { return process_ ? process_->processId() : 0; }
  bool running() const { return process_ && process_->state() == QProcess::Running && port_ > 0; }

  void stop()
  {
    ++epoch_;
    auto * process = process_;
    process_ = nullptr;
    port_ = 0;
    root_.clear();
    if (!process) return;
    disconnect(process, nullptr, this, nullptr);
    const qint64 old_pid = process->processId();
    process->closeWriteChannel();
    if (process->state() != QProcess::NotRunning && !process->waitForFinished(1000)) {
      process->terminate();
      if (!process->waitForFinished(1000)) {
        process->kill();
        process->waitForFinished(1000);
      }
    }
    if (diagnostic) diagnostic(QStringLiteral("Product View owned server stopped: pid=%1 exited=%2")
      .arg(old_pid).arg(process->state() == QProcess::NotRunning));
    process->deleteLater();
  }

  void start(const QString & repo_root)
  {
    stop();
    root_ = QFileInfo(repo_root).canonicalFilePath();
    const QString session = QUuid::createUuid().toString(QUuid::WithoutBraces);
    const quint64 epoch = epoch_;
    auto * process = new QProcess(this);
    process_ = process;
    stdout_.clear();
    stderr_.clear();
    failure_reported_ = false;
    process->setProcessChannelMode(QProcess::SeparateChannels);
    process->setWorkingDirectory(root_);
    process->setProgram(QStringLiteral("python3"));
    process->setArguments({QDir(root_).filePath(QStringLiteral("scripts/workcell_product_view_server.py")),
      QStringLiteral("--directory"), root_, QStringLiteral("--session"), session});
    connect(process, &QProcess::readyReadStandardError, this, [this, process, epoch]() {
      if (!current(process, epoch)) return;
      const QByteArray chunk = process->readAllStandardError();
      stderr_ += chunk;
      if (diagnostic && !chunk.isEmpty()) diagnostic(QString::fromUtf8(chunk));
    });
    connect(process, &QProcess::readyReadStandardOutput, this, [this, process, epoch, session]() {
      if (!current(process, epoch)) return;
      stdout_ += process->readAllStandardOutput();
      if (port_ || !stdout_.contains('\n')) return;
      const auto object = QJsonDocument::fromJson(stdout_.left(stdout_.indexOf('\n'))).object();
      const int port = object.value(QStringLiteral("port")).toInt();
      if (object.value(QStringLiteral("schema")).toString() != QStringLiteral("workcell_product_view_server/v1") ||
          object.value(QStringLiteral("host")).toString() != QStringLiteral("127.0.0.1") ||
          object.value(QStringLiteral("repo_root")).toString() != root_ ||
          object.value(QStringLiteral("session")).toString() != session ||
          object.value(QStringLiteral("pid")).toVariant().toLongLong() != process->processId() ||
          port <= 0 || port > 65535) {
        reportFailure(QStringLiteral("Invalid owned server handshake: %1").arg(QString::fromUtf8(stdout_)));
        return;
      }
      port_ = port;
      if (diagnostic) diagnostic(QStringLiteral("Product View owned server bound: pid=%1 port=%2 session=%3 root=%4")
        .arg(process->processId()).arg(port).arg(session, root_));
      if (ready) ready(port);
    });
    connect(process, qOverload<int, QProcess::ExitStatus>(&QProcess::finished), this,
      [this, process, epoch](int code, QProcess::ExitStatus) {
        if (!current(process, epoch)) return;
        stderr_ += process->readAllStandardError();
        reportFailure(QStringLiteral("Owned server exited with code %1:\n%2").arg(code).arg(QString::fromUtf8(stderr_)));
      });
    connect(process, &QProcess::errorOccurred, this, [this, process, epoch](QProcess::ProcessError error) {
      if (!current(process, epoch) || error != QProcess::FailedToStart) return;
      reportFailure(QStringLiteral("Owned server failed to start: %1").arg(process->errorString()));
    });
    QTimer::singleShot(8000, this, [this, process, epoch]() {
      if (current(process, epoch) && !port_) reportFailure(QStringLiteral("Owned server handshake timed out:\n%1").arg(QString::fromUtf8(stderr_)));
    });
    process->start();
  }

private:
  bool current(QProcess * process, quint64 epoch) const { return process_ == process && epoch_ == epoch; }
  void reportFailure(const QString & detail)
  {
    if (failure_reported_) return;
    failure_reported_ = true;
    if (diagnostic) diagnostic(detail);
    if (failed) failed(detail);
  }
  QProcess * process_{nullptr};
  quint64 epoch_{0};
  int port_{0};
  QString root_;
  QByteArray stdout_, stderr_;
  bool failure_reported_{false};
};
